"""Linux input-event reader; axis convention follows the reference SpaceMouse app.

Uses the kernel event interface, including Wireless BT devices, without evdev,
hidapi, spacenavd, or a dependency on the external reference checkout.
"""

import errno
import fcntl
import os
from pathlib import Path
import struct
import time

import numpy as np


def discover_devices():
    devices = {}
    for event in sorted(Path('/sys/class/input').glob('event*')):
        try:
            name = (event / 'device/name').read_text().strip()
        except OSError:
            continue
        if any(word in name.lower() for word in ('3dconnexion', 'spacemouse', 'spacenavigator')):
            devices[str(Path('/dev/input') / event.name)] = name
    return devices


class SpaceMouse:
    EVENT = struct.Struct('llHHi')
    ABS_INFO = struct.Struct('6i')

    def __init__(self, device=None, axis_scale=350.0, timeout=0.25):
        if not np.isfinite(axis_scale) or axis_scale <= 0:
            raise ValueError('axis_scale must be finite and positive')
        if not np.isfinite(timeout) or timeout <= 0:
            raise ValueError('timeout must be finite and positive')
        if device is None:
            devices = discover_devices()
            if len(devices) != 1:
                raise RuntimeError('Expected one SpaceMouse; use --list-devices and --device /dev/input/eventN: %s' % devices)
            device = next(iter(devices))
        self.device = device
        self.axis_scale = axis_scale
        self.timeout = timeout
        self.axes = np.zeros(6, dtype=float)
        self.buttons = np.zeros(2, dtype=bool)
        self.last_axis_time = 0.0
        self.axis_events = 0
        self.button_events = 0
        self.axis_peak = 0.0
        self.absolute_axes = set()
        self.fd = None
        self.lock = None
        try:
            self.lock = open('/tmp/franka-spacemouse-%d.lock' % os.getuid(), 'a')
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.fd = os.open(device, os.O_RDONLY | os.O_NONBLOCK)
            for code in range(6):
                try:
                    self._absolute_value(code)
                except OSError as exc:
                    if exc.errno not in (errno.EINVAL, errno.ENOTTY):
                        raise
                else:
                    self.absolute_axes.add(code)
        except Exception:
            self.close()
            raise

    def _absolute_value(self, code):
        # EVIOCGABS(code), from linux/input.h: query, never change device state.
        request = (2 << 30) | (self.ABS_INFO.size << 16) | (ord('E') << 8) | (0x40 + code)
        info = bytearray(self.ABS_INFO.size)
        fcntl.ioctl(self.fd, request, info)
        return self.ABS_INFO.unpack(info)[0]

    def read(self):
        """Return mapped axes and press counts (also preserves quick clicks)."""
        if self.fd is None:
            raise RuntimeError('SpaceMouse is closed')
        presses = np.zeros(2, dtype=int)
        while True:
            try:
                data = os.read(self.fd, self.EVENT.size * 128)
            except BlockingIOError:
                break
            except OSError as exc:
                raise ConnectionError('SpaceMouse disconnected: %s' % self.device) from exc
            if not data:
                raise ConnectionError('SpaceMouse input closed: %s' % self.device)
            if len(data) % self.EVENT.size:
                raise RuntimeError('Incomplete Linux input event')
            for _, _, kind, code, value in self.EVENT.iter_unpack(data):
                if kind == 0 and code == 3:  # SYN_DROPPED: cannot trust cached state.
                    raise RuntimeError('SpaceMouse event buffer overflow; restart the input process')
                if kind in (2, 3) and 0 <= code < 6:  # EV_REL / EV_ABS
                    self.axes[code] = np.clip(value / self.axis_scale, -1, 1)
                    self.axis_peak = max(self.axis_peak, abs(float(self.axes[code])))
                    self.last_axis_time = time.monotonic()
                    self.axis_events += 1
                elif kind == 1 and code in (256, 257):  # EV_KEY, BTN_0 / BTN_1
                    self.button_events += 1
                    index = code - 256
                    pressed = value != 0
                    if pressed and not self.buttons[index]:
                        presses[index] += 1
                    self.buttons[index] = pressed
        stale = time.monotonic() - self.last_axis_time > self.timeout
        for code in range(6):
            if code in self.absolute_axes:
                # EV_ABS reports changes, not a heartbeat. A held deflection may
                # produce no new event; query the kernel's current state instead
                # of timing it out. Release reads zero and unplug raises an error.
                try:
                    value = self._absolute_value(code)
                except OSError as exc:
                    raise ConnectionError('Cannot read SpaceMouse absolute axis: %s' % self.device) from exc
                self.axes[code] = np.clip(value / self.axis_scale, -1, 1)
            elif stale:
                self.axes[code] = 0
        self.axis_peak = max(self.axis_peak, float(np.max(np.abs(self.axes))))
        # Same mapping as the reference: [y, x, -z, ry, rx, -rz].
        mapped = self.axes[[1, 0, 2, 4, 3, 5]] * np.array([1, 1, -1, 1, 1, -1])
        return mapped, presses

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
        if self.lock is not None:
            self.lock.close()
            self.lock = None


class MouseControl:
    SPEEDS = (0.05, 0.15, 0.30, 0.50)
    THRESHOLDS = (0.48, 0.45, 0.42, 0.40)
    # Camera reads and recording can make the loop slower than its requested
    # rate. Integrate elapsed time independently of that rate, but never make
    # up more than 100 ms of motion after a stall.
    MAX_MOTION_DT = 0.1

    def __init__(self, hz=100, translation_speed=0.10, rotation_speed=0.60,
                 gripper_closing=0.0):
        values = np.array([hz, translation_speed, rotation_speed], dtype=float)
        if not np.isfinite(values).all() or (values <= 0).any():
            raise ValueError('hz and speeds must be finite and positive')
        self.hz = hz
        self.translation_speed = translation_speed
        self.rotation_speed = rotation_speed
        self.speed_level = 0
        self.gripper_closing = float(np.clip(gripper_closing, 0, 1))
        self.motion_dt = 0.0

    def update(self, axes, presses, dt):
        self.speed_level = (self.speed_level + int(presses[0])) % len(self.SPEEDS)
        if int(presses[1]) % 2:
            self.gripper_closing = 0.0 if self.gripper_closing >= 0.5 else 1.0
        axes = np.asarray(axes, dtype=float)
        if axes.shape != (6,) or not np.isfinite(axes).all() or not np.isfinite(dt):
            raise ValueError('Invalid SpaceMouse sample')
        discrete = np.where(np.abs(axes) > self.THRESHOLDS[self.speed_level], np.sign(axes), 0)
        self.motion_dt = float(np.clip(dt, 0, self.MAX_MOTION_DT))
        scale = self.SPEEDS[self.speed_level] * self.motion_dt
        return (discrete[:3] * self.translation_speed * scale,
                discrete[3:] * self.rotation_speed * scale, self.gripper_closing)
