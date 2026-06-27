"""读取 leader 各关节角度，可选读取 follower 从臂并计算 delta。

Usage:
    # 仅读取 leader
    python -m teleop.experiments.check_offset \
        --teleop_port=/dev/serial/by-id/usb-...

    # leader + follower 对比
    python -m teleop.experiments.check_offset \
        --teleop_port=/dev/serial/by-id/usb-... \
        --control_port=6001
"""

import argparse
import sys

import numpy as np

from teleop.agents.teleop_agent import PORT_CONFIG_MAP
from teleop.zmq_core.robot_node import ZMQClientRobot


def main():
    parser = argparse.ArgumentParser(
        description="Read leader joint angles, optionally compare with follower."
    )
    parser.add_argument(
        "--teleop_port",
        type=str,
        required=True,
        help="Leader serial port, e.g. /dev/serial/by-id/usb-...",
    )
    parser.add_argument(
        "--control_port",
        type=int,
        default=None,
        help="Follower ZMQ port (e.g. 6001). If set, also reads follower joints.",
    )
    parser.add_argument(
        "--hostname",
        type=str,
        default="127.0.0.1",
        help="Follower ZMQ hostname (default 127.0.0.1).",
    )
    args = parser.parse_args()

    port = args.teleop_port

    if port not in PORT_CONFIG_MAP:
        print(f"Error: port '{port}' not found in PORT_CONFIG_MAP", file=sys.stderr)
        print("Available ports:", file=sys.stderr)
        for p in PORT_CONFIG_MAP:
            print(f"  {p}", file=sys.stderr)
        sys.exit(1)

    config = PORT_CONFIG_MAP[port]

    # --- Leader ---
    print("Connecting to leader...")
    leader_robot = config.make_robot(port=port)
    leader_joints = np.asarray(leader_robot.get_joint_state())
    print(f"  Leader DOF: {len(leader_joints)}")

    # --- Follower (optional) ---
    follower_joints = None
    if args.control_port is not None:
        print(f"Connecting to follower (ZMQ port {args.control_port})...")
        try:
            follower_client = ZMQClientRobot(port=args.control_port, host=args.hostname)
            obs = follower_client.get_observations()
            follower_joints = np.asarray(obs["joint_positions"])
            print(f"  Follower DOF: {len(follower_joints)}")
        except Exception as e:
            print(f"  Warning: failed to read follower: {e}")
            print("  Skipping follower data.")

    # --- Print ---
    print()
    print(f"{'j':>5} | {'leader':>10} | {'follower':>10} | {'delta':>10}")
    print("-" * 46)

    for i in range(len(leader_joints)):
        lv = leader_joints[i]
        if follower_joints is not None and i < len(follower_joints):
            fv = follower_joints[i]
            dv = lv - fv
            print(f"  [{i:>2}] | {lv:>10.4f} | {fv:>10.4f} | {dv:>10.4f}")
        else:
            print(f"  [{i:>2}] | {lv:>10.4f} | {'—':>10} | {'—':>10}")

    # Extra gripper joint if follower has more DOF
    if follower_joints is not None and len(follower_joints) > len(leader_joints):
        for i in range(len(leader_joints), len(follower_joints)):
            fv = follower_joints[i]
            print(f"  [{i:>2}] | {'—':>10} | {fv:>10.4f} | {'—':>10}")

    print(f"\nTotal: leader={len(leader_joints)}",
          f"follower={len(follower_joints) if follower_joints is not None else 0}")
    print("Done.\n")


if __name__ == "__main__":
    main()

