import threading
import time
from collections import deque
from typing import Optional, Tuple

import numpy as np
import serial


FRAME_LEN = 14
HEADER = b"\xAA\x55"


def crc8(data: bytes) -> int:
    crc = 0x00
    poly = 0x8C
    for byte in data:
        crc ^= byte
        for _ in range(8):
            carry = crc & 0x01
            crc >>= 1
            if carry:
                crc ^= poly
    return crc


def decode_sign_magnitude(raw: int, bits: int, scale: float = 10.0) -> float:
    sign_bit = 1 << (bits - 1)
    magnitude = raw & (sign_bit - 1)
    return (-magnitude if raw & sign_bit else magnitude) / scale


def parse_frame(frame: bytes) -> Optional[Tuple[float, ...]]:
    if len(frame) != FRAME_LEN or not frame.startswith(HEADER):
        return None
    if frame[13] != crc8(frame[2:13]):
        return None

    fx_raw = (frame[2] << 9) | (frame[3] << 1) | ((frame[4] & 0x80) >> 7)
    fy_raw = ((frame[4] & 0x7F) << 10) | (frame[5] << 2) | ((frame[6] & 0xC0) >> 6)
    fz_raw = ((frame[6] & 0x3F) << 11) | (frame[7] << 3) | ((frame[8] & 0xE0) >> 5)
    mx_raw = ((frame[8] & 0x1F) << 7) | ((frame[9] & 0xFE) >> 1)
    my_raw = ((frame[9] & 0x01) << 11) | (frame[10] << 3) | ((frame[11] & 0xE0) >> 5)
    mz_raw = ((frame[11] & 0x1F) << 7) | ((frame[12] & 0xFE) >> 1)

    return (
        decode_sign_magnitude(fx_raw, 17),
        decode_sign_magnitude(fy_raw, 17),
        decode_sign_magnitude(fz_raw, 17),
        decode_sign_magnitude(mx_raw, 12),
        decode_sign_magnitude(my_raw, 12),
        decode_sign_magnitude(mz_raw, 12),
    )


class ForceSensor:
    """Continuously read the serial force sensor and expose its latest valid frame."""

    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 460800):
        self._serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
        )
        self._latest: Optional[np.ndarray] = None
        self._latest_monotonic = 0.0
        self._zero_offset = np.zeros(6, dtype=np.float64)
        self._samples = deque(maxlen=5000)
        self._sequence = 0
        self._last_observation_sequence: Optional[int] = None
        self._condition = threading.Condition()
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        print(f"Force sensor opened: {port}, baudrate={baudrate}")

    def _read_loop(self) -> None:
        buffer = bytearray()
        while not self._stop_event.is_set():
            chunk = self._serial.read(max(self._serial.in_waiting, 1))
            if not chunk:
                continue
            buffer.extend(chunk)

            while len(buffer) >= FRAME_LEN:
                header_index = buffer.find(HEADER)
                if header_index == -1:
                    del buffer[:-1]
                    break
                if header_index:
                    del buffer[:header_index]
                if len(buffer) < FRAME_LEN:
                    break

                result = parse_frame(bytes(buffer[:FRAME_LEN]))
                if result is None:
                    del buffer[0]
                    continue

                del buffer[:FRAME_LEN]
                raw_sample = np.asarray(result, dtype=np.float64)
                sample_monotonic = time.monotonic()
                sample_time_ns = time.time_ns()
                with self._condition:
                    sample = raw_sample - self._zero_offset
                    self._sequence += 1
                    self._latest = sample
                    self._latest_monotonic = sample_monotonic
                    self._samples.append((self._sequence, sample_time_ns, sample))
                    self._condition.notify_all()

    def get_latest(
        self, wait_timeout: float = 1.0, max_age: float = 0.5
    ) -> np.ndarray:
        deadline = time.monotonic() + wait_timeout
        while True:
            with self._condition:
                sample_age = time.monotonic() - self._latest_monotonic
                if self._latest is not None and sample_age <= max_age:
                    return self._latest.copy()
            if time.monotonic() >= deadline:
                raise RuntimeError("Force sensor has no recent valid frame")
            time.sleep(0.001)

    def capture_observation_windows(
        self, max_age: float = 0.5
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Capture raw, causal force windows without waiting for future samples."""
        with self._condition:
            sample_age = time.monotonic() - self._latest_monotonic
            if self._latest is None or sample_age > max_age:
                raise RuntimeError("Force sensor has no recent valid frame")

            observation_time_ns = time.time_ns()
            center_sequence = self._sequence
            samples_at_observation = list(self._samples)
            samples_before = [
                item for item in samples_at_observation if item[0] <= center_sequence
            ]
            if len(samples_before) < 50:
                raise RuntimeError(
                    "Force sensor needs at least 50 frames before an observation"
                )

            latest = samples_before[-1][2].copy()
            before_50 = np.asarray(
                [item[2] for item in samples_before[-50:]], dtype=np.float64
            )
            if self._last_observation_sequence is None:
                between_observations = np.empty((0, 6), dtype=np.float64)
            else:
                between_observations = np.asarray(
                    [
                        item[2]
                        for item in samples_before
                        if self._last_observation_sequence < item[0]
                    ],
                    dtype=np.float64,
                ).reshape(-1, 6)
            history_1s = np.asarray(
                [
                    item[2]
                    for item in samples_before
                    if item[1] >= observation_time_ns - 1_000_000_000
                ],
                dtype=np.float64,
            ).reshape(-1, 6)
            history_2s = np.asarray(
                [
                    item[2]
                    for item in samples_before
                    if item[1] >= observation_time_ns - 2_000_000_000
                ],
                dtype=np.float64,
            ).reshape(-1, 6)

            self._last_observation_sequence = center_sequence

        return latest, before_50, between_observations, history_1s, history_2s

    def tare(self, baseline_frames: int = 500, fresh_frames: int = 50) -> np.ndarray:
        """Zero the sensor from recent samples and start a fresh episode history."""
        with self._condition:
            if len(self._samples) < baseline_frames:
                raise RuntimeError(
                    f"Force sensor needs {baseline_frames} frames before taring; "
                    f"only {len(self._samples)} are available"
                )

            # Buffered samples already have the previous offset removed. Adding
            # their mean to the old offset produces the new absolute raw offset.
            baseline = np.mean(
                [item[2] for item in list(self._samples)[-baseline_frames:]], axis=0
            )
            self._zero_offset = self._zero_offset + baseline
            start_sequence = self._sequence
            self._samples.clear()
            self._last_observation_sequence = None
            self._latest = None
            self._latest_monotonic = 0.0

            deadline = time.monotonic() + 1.0
            while self._sequence < start_sequence + fresh_frames:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError(
                        f"Timed out waiting for {fresh_frames} fresh force frames after taring"
                    )
                self._condition.wait(timeout=remaining)

            return self._zero_offset.copy()

    def close(self) -> None:
        self._stop_event.set()
        self._thread.join(timeout=0.2)
        self._serial.close()
