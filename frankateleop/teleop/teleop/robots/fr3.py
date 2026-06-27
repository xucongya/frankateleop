import time
import torch
from typing import Dict
from typing import Optional
import numpy as np
from teleop.robots.robot import Robot

MAX_OPEN = 0.09
TORQUE_OBS_KEYS = (
    "joint_torques_computed",
    "prev_joint_torques_computed",
    "prev_joint_torques_computed_safened",
    "motor_torques_measured",
    "motor_torques_external",
    "motor_torques_desired",
)


def _to_numpy(value) -> np.ndarray:
    if isinstance(value, torch.Tensor):
        return value.detach().cpu().numpy()
    return np.asarray(value, dtype=float)


def _estimate_ee_external_wrench(
    robot, joint_positions: np.ndarray, joint_torques: np.ndarray
) -> np.ndarray:
    try:
        jacobian = robot.robot_model.compute_jacobian(torch.Tensor(joint_positions))
        jacobian = _to_numpy(jacobian)
        wrench, *_ = np.linalg.lstsq(jacobian.T, joint_torques, rcond=None)
        return wrench
    except Exception:
        return np.full(6, np.nan)


class fr3Robot(Robot):
    """A class representing a UR robot."""

    def __init__(
            self, 
            robot_ip: str = "192.168.1.100", 
            franka_port: int=50051, 
            frankahand_port: int = 50053,
            joint_positions_desired: Optional[torch.Tensor] = None,
            ):
            
        from polymetis import GripperInterface, RobotInterface
        print(f"Connecting to robot at IP: {robot_ip}")

        self.robot = RobotInterface(
            ip_address=robot_ip,
            port=franka_port,
            enforce_version=False,
        )
        self.gripper = GripperInterface(
            ip_address=robot_ip,
            port=frankahand_port,
        )
        if joint_positions_desired is None:
            self.robot.go_home()
        
        else:
            if joint_positions_desired.shape != (7,):
                raise ValueError(f"Franka requires 7 joints params, current input is: {joint_positions_desired.shape}")
            print("init robot")
            self.joint_positions_desired = joint_positions_desired
            self.robot.move_to_joint_positions(self.joint_positions_desired)
            
        self.robot.start_joint_impedance()
        self.gripper.goto(width=MAX_OPEN, speed=255, force=255)
        time.sleep(1)

    def num_dofs(self) -> int:
        """Get the number of joints of the robot.

        Returns:
            int: The number of joints of the robot.
        """
        return 8

    def get_joint_state(self) -> np.ndarray:
        """Get the current state of the leader robot.

        Returns:
            T: The current state of the leader robot.
        """
        robot_joints = self.robot.get_joint_positions()
        gripper_pos = self.gripper.get_state()
        pos = np.append(robot_joints, gripper_pos.width / MAX_OPEN)
        return pos

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        """Command the leader robot to a given state.

        Args:
            joint_state (np.ndarray): The state to command the leader robot to.
        """
        import torch

        self.robot.update_desired_joint_positions(torch.tensor(joint_state[:-1]))
        self.gripper.goto(width=(MAX_OPEN * (1 - joint_state[-1])), speed=1, force=1)

    def get_observations(self) -> Dict[str, np.ndarray]:
        robot_state = self.robot.get_robot_state()
        robot_joints = _to_numpy(robot_state.joint_positions)
        gripper_state = self.gripper.get_state()
        joints = np.append(robot_joints, gripper_state.width / MAX_OPEN)
        joint_velocities = np.append(_to_numpy(robot_state.joint_velocities), 0.0)
        ee_pos, ee_quat = self.robot.robot_model.forward_kinematics(
            torch.Tensor(robot_joints)
        )
        pos_quat = np.concatenate((_to_numpy(ee_pos), _to_numpy(ee_quat)))
        gripper_pos = np.array([joints[-1]])
        observations = {
            "joint_positions": joints,
            "joint_velocities": joint_velocities,
            "ee_pos_quat": pos_quat,
            "gripper_position": gripper_pos,
        }
        for key in TORQUE_OBS_KEYS:
            observations[key] = _to_numpy(getattr(robot_state, key))

        observations["ee_external_wrench"] = _estimate_ee_external_wrench(
            self.robot,
            robot_joints,
            observations["motor_torques_external"],
        )
        return observations


def main():
    robot = fr3Robot()
    current_joints = robot.get_joint_state()
    # move a small delta 0.1 rad
    move_joints = current_joints + 0.05
    # make last joint (gripper) closed
    move_joints[-1] = 0.5
    time.sleep(1)
    m = 0.09
    robot.gripper.goto(1 * m, speed=255, force=255)
    time.sleep(1)
    robot.gripper.goto(1.05 * m, speed=255, force=255)
    time.sleep(1)
    robot.gripper.goto(1.1 * m, speed=255, force=255)
    time.sleep(1)


if __name__ == "__main__":
    main()
