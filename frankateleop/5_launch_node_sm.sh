#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate polymetis
export PYTHONPATH="$SCRIPT_DIR/teleop${PYTHONPATH:+:$PYTHONPATH}"
export no_proxy="${no_proxy:+$no_proxy,}localhost,127.0.0.1,${FR3_ROBOT_IP:-192.168.1.100}"
export NO_PROXY="$no_proxy"

exec python3 -m teleop.spacemouse_collection.node \
    --robot-ip "${FR3_ROBOT_IP:-192.168.1.100}" \
    --robot-port "${FR3_ROBOT_PORT:-50051}" \
    --gripper-port "${FR3_GRIPPER_PORT:-50052}" \
    --port "${FR3_ZMQ_PORT:-6001}" \
    --force-sensor-port "${FORCE_SENSOR_PORT:-/dev/ttyACM0}" \
    "$@"
