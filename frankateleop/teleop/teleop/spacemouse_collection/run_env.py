"""SpaceMouse teleoperation using the unchanged four-camera GELLO recorder."""

import argparse
import datetime
from pathlib import Path
import time

import numpy as np
import zmq

from teleop.data_utils.format_obs import save_frame
from teleop.env import RobotEnv
from teleop.spacemouse_collection.input import MouseControl, SpaceMouse, discover_devices
from teleop.zmq_core.camera_node import ZMQClientCamera


# Identical to experiments/run_env.py; do not resize or convert camera output.
CAMERA_OBS_KEYS = {
    'wrist_front': ('wrist_rgb_front', 'wrist_depth_front'),
    'wrist_back': ('wrist_rgb_back', 'wrist_depth_back'),
    'base': ('base_rgb', 'base_depth'),
    'local': ('local_rgb', 'local_depth'),
}


class CollectionCamera(ZMQClientCamera):
    """Keep the existing camera protocol, adding connection diagnostics only."""

    def __init__(self, name, port):
        super().__init__(port=port, host='127.0.0.1')
        self.name = name
        self.endpoint = 'tcp://127.0.0.1:%d' % port
        self.received_frame = False
        self._socket.setsockopt(zmq.RCVTIMEO, 5000)
        self._socket.setsockopt(zmq.SNDTIMEO, 5000)
        self._socket.setsockopt(zmq.LINGER, 0)

    def read(self, img_size=None):
        if not self.received_frame:
            print('Waiting for camera %s at %s ...' % (self.name, self.endpoint), flush=True)
        try:
            frame = super().read(img_size)
        except zmq.Again as exc:
            raise RuntimeError(
                'Camera %s at %s did not respond within 5 seconds. '
                'Start all four camera nodes (5000-5003) and keep them running. '
                'For RGB-only collection, run from the frankateleop directory in '
                'another polymetis terminal: python teleop/experiments/launch_camera_nodes.py '
                '--hostname=127.0.0.1 --no-depth. If already running, check that '
                'this camera process has not exited and is bound to the expected port.'
                % (self.name, self.endpoint)
            ) from exc
        if not self.received_frame:
            print('Camera %s ready.' % self.name, flush=True)
            self.received_frame = True
        return frame


def run(args):
    if args.list_devices:
        for path, name in discover_devices().items():
            print('%s: %s' % (path, name))
        return
    control = MouseControl(args.hz, args.translation_speed, args.rotation_speed)
    mouse = SpaceMouse(args.device, args.axis_scale)
    cameras = {}
    robot = None
    keyboard = None
    try:
        print('SpaceMouse: %s; left button = speed, right button = gripper' % mouse.device, flush=True)
        print('Input scale=%.1f, initial threshold=%.2f, loop limit=%.1f Hz' % (
            mouse.axis_scale, control.THRESHOLDS[control.speed_level], args.hz), flush=True)
        print('Translation speed levels (mm/s): %s; rotation (rad/s): %s. '
              'Motion uses elapsed time, capped at %.0f ms per step.' % (
                  [round(control.translation_speed * level * 1000, 3) for level in control.SPEEDS],
                  [round(control.rotation_speed * level, 3) for level in control.SPEEDS],
                  control.MAX_MOTION_DT * 1000), flush=True)
        print('Absolute axes: %s; held values are read from the kernel.' % sorted(mouse.absolute_axes), flush=True)
        if args.dry_run:
            print('Dry run: no robot/camera connections and no dataset writes.', flush=True)
            previous = time.monotonic()
            while True:
                now = time.monotonic()
                axes, presses = mouse.read()
                translation, rotation, gripper = control.update(axes, presses, now - previous)
                previous = now
                print('axes=%s  speed=%d  dxyz=%s  drot=%s  gripper_closing=%.1f  events(axis/button)=%d/%d  peak_abs_axis=%.3f' % (
                    np.array2string(axes, precision=2), control.speed_level + 1,
                    translation, rotation, gripper, mouse.axis_events, mouse.button_events,
                    mouse.axis_peak), flush=True)
                time.sleep(1 / args.hz)

        from teleop.spacemouse_collection.node import SpaceMouseClient
        from teleop.data_utils.keyboard_interface import KBReset
        print('Connecting to SpaceMouse robot node at 127.0.0.1:%d ...' % args.port, flush=True)
        robot = SpaceMouseClient(args.port)
        print('Robot node connected; reading all four cameras before enabling teleoperation.', flush=True)
        for name in CAMERA_OBS_KEYS:
            camera = CollectionCamera(name, port=getattr(args, name + '_camera_port'))
            cameras[name] = camera
        env = RobotEnv(robot, control_rate_hz=args.hz, camera_dict=cameras,
                       camera_obs_keys=CAMERA_OBS_KEYS)
        obs = env.get_obs()
        control.gripper_closing = float(np.clip(1 - obs['gripper_position'][0], 0, 1))
        keyboard = KBReset()
        save_path = None
        previous = time.monotonic()
        last_warning = 0.0
        last_debug = previous
        debug_frames = 0
        previous_action = None
        # Drain input accumulated during initialization, before enabling commands.
        mouse.read()
        print('Ready. Focus the recording window: S = tare/start, Q = stop recording; Ctrl+C = exit.', flush=True)
        while True:
            now = time.monotonic()
            axes, presses = mouse.read()
            loop_dt = now - previous
            translation, rotation, gripper = control.update(axes, presses, loop_dt)
            previous = now
            if presses.any():
                level = control.SPEEDS[control.speed_level]
                print('Speed level %d/4; translation=%.1f mm/s; rotation=%.3f rad/s; gripper closing=%.1f' % (
                    control.speed_level + 1, control.translation_speed * level * 1000,
                    control.rotation_speed * level, gripper), flush=True)
            plan = robot.plan_action(translation, rotation, gripper)
            action = np.asarray(plan['action'], dtype=float)
            target_change = (0.0 if previous_action is None else
                             float(np.max(np.abs(action[:7] - previous_action[:7]))))
            if plan['reason'] and now - last_warning >= 1:
                print(plan['reason'], flush=True)
                last_warning = now
            timestamp = datetime.datetime.now()
            state = keyboard.update()
            if state == 'start':
                robot.zero_force_sensor()
                save_path = Path(args.data_dir).expanduser() / 'teleop' / datetime.datetime.now().strftime('%m%d_%H%M%S')
                save_path.mkdir(parents=True, exist_ok=False)
                print('Recording: %s' % save_path, flush=True)
                # Taring can block for a second. Do not integrate that pause
                # into the first movement of the new recording episode.
                previous = time.monotonic()
            elif state == 'save':
                # Existing saver writes obs + control, with the original timestamp name.
                save_frame(save_path, timestamp, obs, action)
            elif state == 'normal':
                if save_path is not None:
                    print('Saved: %s' % save_path, flush=True)
                save_path = None
            previous_joints = np.asarray(obs['joint_positions'][:7]).copy()
            obs = env.step(action)
            previous_action = action.copy()
            debug_frames += 1
            finished = time.monotonic()
            if args.debug and finished - last_debug >= 0.5:
                measured = np.asarray(obs['joint_positions'][:7])
                measured_change = float(np.max(np.abs(measured - previous_joints)))
                tracking_error = float(np.max(np.abs(action[:7] - measured)))
                print(
                    '[control] axes=%s threshold=%.2f speed=%d '
                    'dxyz_mm=%s drot_rad=%s target_dq=%.6f measured_dq=%.6f '
                    'tracking_error=%.6f loop_hz=%.1f loop_dt_ms=%.1f motion_dt_ms=%.1f '
                    'events(axis/button)=%d/%d status=%s' % (
                        np.array2string(axes, precision=3), control.THRESHOLDS[control.speed_level],
                        control.speed_level + 1, np.array2string(translation * 1000, precision=3),
                        np.array2string(rotation, precision=5), target_change, measured_change,
                        tracking_error, debug_frames / (finished - last_debug),
                        loop_dt * 1000, control.motion_dt * 1000,
                        mouse.axis_events, mouse.button_events, plan['reason'] or 'ok'), flush=True)
                last_debug = finished
                debug_frames = 0
    finally:
        mouse.close()
        for camera in cameras.values():
            camera._socket.close(linger=0)
            camera._context.term()
        if robot is not None:
            robot.close()
        if keyboard is not None:
            import pygame
            pygame.quit()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=6001)
    parser.add_argument('--device', help='Linux input event device; auto-detect when exactly one is found')
    parser.add_argument('--list-devices', action='store_true')
    parser.add_argument('--dry-run', action='store_true', help='Only read mouse; never connect to robot/cameras')
    parser.add_argument('--debug', action='store_true', help='Print mouse input, requested and measured joint changes; dataset unchanged')
    parser.add_argument('--hz', type=float, default=100, help='Collection loop limit, same default as GELLO')
    parser.add_argument('--axis-scale', type=float, default=350)
    parser.add_argument('--translation-speed', type=float, default=0.10, help='m/s before speed-level multiplier')
    parser.add_argument('--rotation-speed', type=float, default=0.60, help='rad/s before speed-level multiplier')
    parser.add_argument('--data-dir', default='~/bc_data')
    for name, port in zip(CAMERA_OBS_KEYS, (5000, 5001, 5002, 5003)):
        parser.add_argument('--' + name.replace('_', '-') + '-camera-port', type=int, default=port)
    args = parser.parse_args()
    try:
        run(args)
    except KeyboardInterrupt:
        print('\nSpaceMouse collection stopped; robot retains its last joint target.')
    except (OSError, RuntimeError, ValueError, zmq.ZMQError) as exc:
        parser.exit(1, 'SpaceMouse collection stopped: %s\n' % exc)


if __name__ == '__main__':
    main()
