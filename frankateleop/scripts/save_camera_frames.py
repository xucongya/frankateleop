#!/usr/bin/env python3
"""Save one RGB image per running camera node, named <port>.png.

Start launch_camera_nodes.py first, then run in the polymetis environment:
    python scripts/save_camera_frames.py
    python scripts/save_camera_frames.py --ports 5000 5002 --output-dir ./snapshots

Works with both RGB-D and RGB-only nodes. Images are requested sequentially;
they are not synchronized across cameras. Existing image files are not replaced.
"""

import argparse
from datetime import datetime
from pathlib import Path
import pickle
import sys

import cv2
import numpy as np
import zmq


def read_rgb(context: zmq.Context, hostname: str, port: int, timeout_ms: int) -> np.ndarray:
    socket = context.socket(zmq.REQ)
    try:
        socket.setsockopt(zmq.LINGER, 0)
        socket.setsockopt(zmq.SNDTIMEO, timeout_ms)
        socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
        socket.connect(f"tcp://{hostname}:{port}")
        # Same request as ZMQClientCamera.read(): None keeps the native image size.
        socket.send(pickle.dumps(None))
        rgb, _depth = pickle.loads(socket.recv())
        if not isinstance(rgb, np.ndarray) or rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError("Camera response must contain an RGB array of shape (H, W, 3)")
        if rgb.dtype != np.uint8:
            raise ValueError(f"Expected uint8 RGB image, got {rgb.dtype}")
        return rgb
    finally:
        socket.close(linger=0)


def save_rgb(path: Path, rgb: np.ndarray) -> None:
    # The camera driver already converts BGR to RGB; OpenCV encodes BGR.
    success, png = cv2.imencode(".png", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    if not success:
        raise RuntimeError("PNG encoding failed")
    with path.open("xb") as stream:
        stream.write(png.tobytes())


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hostname", default="127.0.0.1")
    parser.add_argument("--ports", type=int, nargs="+", default=[5000, 5001, 5002, 5003])
    parser.add_argument(
        "--output-dir", type=Path,
        help="Output directory; default: scripts/camera_snapshots/<timestamp>.",
    )
    parser.add_argument("--timeout-ms", type=int, default=5000, help="Socket timeout in milliseconds.")
    args = parser.parse_args()
    if args.timeout_ms <= 0:
        parser.error("--timeout-ms must be positive")
    if any(port < 1 or port > 65535 for port in args.ports):
        parser.error("--ports must be between 1 and 65535")

    ports = list(dict.fromkeys(args.ports))
    output_dir = args.output_dir or (
        Path(__file__).resolve().parent / "camera_snapshots" / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    )
    output_dir = output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    print(f"Output directory: {output_dir}", flush=True)

    saved = 0
    context = zmq.Context()
    try:
        for port in ports:
            path = output_dir / f"{port}.png"
            try:
                if path.exists():
                    raise FileExistsError(f"Image already exists: {path}")
                rgb = read_rgb(context, args.hostname, port, args.timeout_ms)
                save_rgb(path, rgb)
                saved += 1
                print(f"[{port}] Saved {path} (RGB {rgb.shape})", flush=True)
            except zmq.Again:
                print(f"[{port}] Timed out; check that the camera node is running.", file=sys.stderr)
            except Exception as exc:
                print(f"[{port}] Failed: {exc}", file=sys.stderr)
    finally:
        context.term()

    print(f"Saved {saved}/{len(ports)} images.")
    return 0 if saved == len(ports) else 1


if __name__ == "__main__":
    sys.exit(main())
