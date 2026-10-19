"""Additive FR3 node: Cartesian mouse increments -> existing 8D joint control."""

import argparse
import pickle

import numpy as np
from scipy.spatial.transform import Rotation
import torch
import zmq

from teleop.robots.fr3 import fr3Robot


PROTOCOL = 'fr3-spacemouse-collection-v1'


def vector(value, size):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError('Expected %d finite values' % size)
    return result


class SpaceMouseRobot(fr3Robot):
    # get_observations and zero_force_sensor are inherited without changes.
    def __init__(self, max_joint_delta=0.015, max_translation_delta=0.005,
                 max_rotation_delta=0.03, joint_limit_margin=0.05,
                 max_joint_tracking_error=0.2, **kwargs):
        limits = np.array([max_joint_delta, max_translation_delta, max_rotation_delta,
                           joint_limit_margin, max_joint_tracking_error])
        if not np.isfinite(limits).all() or (limits <= 0).any():
            raise ValueError('Motion limits must be finite and positive')
        super().__init__(**kwargs)
        self.max_joint_delta = max_joint_delta
        self.max_translation_delta = max_translation_delta
        self.max_rotation_delta = max_rotation_delta
        self.max_joint_tracking_error = max_joint_tracking_error
        lower, upper = self.robot.robot_model.get_joint_angle_limits()
        self.lower = np.asarray(lower) + joint_limit_margin
        self.upper = np.asarray(upper) - joint_limit_margin
        state = self.get_joint_state()
        # Observations use opening fraction; actions use closing fraction.
        self.target = np.r_[state[:7], np.clip(1 - state[-1], 0, 1)]
        self.planned = self.target.copy()

    def plan_action(self, translation, rotation, gripper):
        translation = vector(translation, 3)
        rotation = vector(rotation, 3)
        if not np.isfinite(gripper) or not 0 <= gripper <= 1:
            raise ValueError('Gripper closing fraction must be in [0, 1]')
        for delta, limit in ((translation, self.max_translation_delta),
                             (rotation, self.max_rotation_delta)):
            norm = np.linalg.norm(delta)
            if norm > limit:
                delta *= limit / norm
        action = self.target.copy()
        action[-1] = gripper
        reason = ''
        if np.any(translation) or np.any(rotation):
            measured = vector(self.robot.get_joint_positions(), 7)
            if np.max(np.abs(measured - self.target[:7])) > self.max_joint_tracking_error:
                reason = 'Joint tracking error exceeded limit; release mouse and check robot'
            else:
                q0 = torch.tensor(self.target[:7], dtype=torch.float32)
                position, quaternion = self.robot.robot_model.forward_kinematics(q0)
                desired_position = np.asarray(position) + translation
                # Left multiplication applies the increment in the robot base frame.
                desired_quaternion = (Rotation.from_rotvec(rotation) *
                                      Rotation.from_quat(np.asarray(quaternion))).as_quat()
                # The default IK epsilon (1e-4) would swallow the smallest
                # 100 Hz mouse increments. Solve at a finer tolerance here.
                joints = self.robot.robot_model.inverse_kinematics(
                    torch.tensor(desired_position, dtype=torch.float32),
                    torch.tensor(desired_quaternion, dtype=torch.float32),
                    rest_pose=q0, eps=1e-6, max_iters=200)
                joints = vector(joints, 7)
                actual_position, actual_quaternion = self.robot.robot_model.forward_kinematics(
                    torch.tensor(joints, dtype=torch.float32))
                position_error = np.linalg.norm(np.asarray(actual_position) - desired_position)
                rotation_error = (Rotation.from_quat(desired_quaternion) *
                                  Rotation.from_quat(np.asarray(actual_quaternion)).inv()).magnitude()
                success = position_error < 1e-5 and rotation_error < 1e-5
                if not success:
                    reason = 'IK failed; holding previous joint target'
                elif np.any(joints < self.lower) or np.any(joints > self.upper):
                    reason = 'Joint limit margin reached; holding previous joint target'
                else:
                    delta = joints - self.target[:7]
                    delta *= min(1.0, self.max_joint_delta / max(np.max(np.abs(delta)), 1e-12))
                    action[:7] += delta
        self.planned = action.copy()
        return {'action': action, 'reason': reason}

    def command_joint_state(self, joint_state):
        action = vector(joint_state, 8)
        if not np.array_equal(action, self.planned):
            raise ValueError('Command must match the last planned 8D action')
        # Same motor / gripper command as GELLO; save_frame records this exact action.
        super().command_joint_state(action)
        self.target = action.copy()


class SpaceMouseServer:
    METHODS = {'num_dofs', 'get_joint_state', 'get_observations',
               'command_joint_state', 'zero_force_sensor', 'plan_action'}

    def __init__(self, port=6001):
        self.robot = None
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.setsockopt(zmq.RCVTIMEO, 500)
        try:
            # Keep the pickle protocol confined to the local machine.
            self.socket.bind('tcp://127.0.0.1:%d' % port)
        except Exception:
            self.close()
            raise

    def serve(self):
        while True:
            try:
                message = self.socket.recv()
            except zmq.Again:
                continue
            try:
                request = pickle.loads(message)
                method = request['method']
                if method == 'handshake':
                    result = PROTOCOL
                elif method in self.METHODS:
                    result = getattr(self.robot, method)(**request.get('args', {}))
                else:
                    raise ValueError('Unsupported method: %s' % method)
                reply = {'ok': True, 'result': result}
            except Exception as exc:
                reply = {'ok': False, 'error': '%s: %s' % (type(exc).__name__, exc)}
            self.socket.send(pickle.dumps(reply))

    def close(self):
        self.socket.close(linger=0)
        self.context.term()


class SpaceMouseClient:
    def __init__(self, port=6001, timeout_ms=15000):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.setsockopt(zmq.SNDTIMEO, timeout_ms)
        self.socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
        self.socket.connect('tcp://127.0.0.1:%d' % port)
        try:
            if self.call('handshake') != PROTOCOL:
                raise RuntimeError('Wrong node: start 5_launch_node_sm.sh')
        except Exception:
            self.close()
            raise

    def call(self, method, **kwargs):
        try:
            self.socket.send(pickle.dumps({'method': method, 'args': kwargs}))
            reply = pickle.loads(self.socket.recv())
        except zmq.Again as exc:
            raise RuntimeError('Robot RPC timed out; check 5_launch_node_sm.sh') from exc
        if not isinstance(reply, dict) or not reply.get('ok'):
            raise RuntimeError('SpaceMouse node error (use 5_launch_node_sm.sh): %s' % reply)
        return reply['result']

    def num_dofs(self):
        return 8

    def get_joint_state(self):
        return self.call('get_joint_state')

    def get_observations(self):
        return self.call('get_observations')

    def zero_force_sensor(self):
        return self.call('zero_force_sensor')

    def plan_action(self, translation, rotation, gripper):
        return self.call('plan_action', translation=translation, rotation=rotation, gripper=gripper)

    def command_joint_state(self, joint_state):
        return self.call('command_joint_state', joint_state=joint_state)

    def close(self):
        self.socket.close(linger=0)
        self.context.term()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-ip', default='192.168.1.100')
    parser.add_argument('--robot-port', type=int, default=50051)
    parser.add_argument('--gripper-port', type=int, default=50052)
    parser.add_argument('--port', type=int, default=6001)
    parser.add_argument('--force-sensor-port', default='/dev/ttyACM0')
    parser.add_argument('--max-joint-delta', type=float, default=0.015)
    parser.add_argument('--max-translation-delta', type=float, default=0.005)
    parser.add_argument('--max-rotation-delta', type=float, default=0.03)
    parser.add_argument('--joint-limit-margin', type=float, default=0.05)
    parser.add_argument('--max-joint-tracking-error', type=float, default=0.2)
    args = parser.parse_args()
    # Reserve the port before connecting / homing the robot.
    server = SpaceMouseServer(args.port)
    try:
        print('Starting FR3: go_home, open gripper, start joint impedance (same as GELLO).', flush=True)
        server.robot = SpaceMouseRobot(
            robot_ip=args.robot_ip, franka_port=args.robot_port,
            frankahand_port=args.gripper_port, force_sensor_port=args.force_sensor_port,
            max_joint_delta=args.max_joint_delta,
            max_translation_delta=args.max_translation_delta,
            max_rotation_delta=args.max_rotation_delta,
            joint_limit_margin=args.joint_limit_margin,
            max_joint_tracking_error=args.max_joint_tracking_error)
        print('SpaceMouse node ready on 127.0.0.1:%d' % args.port, flush=True)
        server.serve()
    except KeyboardInterrupt:
        pass
    finally:
        server.close()
        if server.robot is not None:
            server.robot.force_sensor.close()


if __name__ == '__main__':
    main()
