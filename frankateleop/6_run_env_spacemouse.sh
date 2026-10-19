#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate polymetis
export PYTHONPATH="$SCRIPT_DIR/teleop${PYTHONPATH:+:$PYTHONPATH}"

exec python3 -m teleop.spacemouse_collection.run_env \
    --port "${FR3_ZMQ_PORT:-6001}" \
    --hz "${SPACEMOUSE_HZ:-100}" \
    --wrist-front-camera-port 5000 \
    --wrist-back-camera-port 5001 \
    --base-camera-port 5002 \
    --local-camera-port 5003 \
    "$@"
