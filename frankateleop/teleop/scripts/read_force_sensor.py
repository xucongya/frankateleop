#!/usr/bin/env python3
"""Display the latest measurements from the end-effector force sensor."""

import sys
from pathlib import Path


TELEOP_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TELEOP_ROOT))

from teleop.force_sensor import ForceSensor


def main() -> None:
    sensor = ForceSensor()
    try:
        while True:
            fx, fy, fz, mx, my, mz = sensor.get_latest()
            print(
                f"Fx={fx:8.2f} N, Fy={fy:8.2f} N, Fz={fz:8.2f} N, "
                f"Mx={mx:8.2f} Nm, My={my:8.2f} Nm, Mz={mz:8.2f} Nm"
            )
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        sensor.close()


if __name__ == "__main__":
    main()
