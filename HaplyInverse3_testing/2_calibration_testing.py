'''
Read haply position data, calibrates, and outputs the calibrated position. Check calibration effects.
'''

import time
import numpy as np
import asyncio
from haply.haply_interface import HaplyInterface
from teleop.haply_calibrator import HaplyCalibrator

async def main():
    haply = HaplyInterface()
    await haply.connect()

    # Create calibrator
    calibrator = HaplyCalibrator(
        filter_type='butter',
        cutoff_freq=5.0,      # low‑pass at 5 Hz
        fs=100.0,
        scale=1.0,            # no amplification yet
        offset=np.zeros(3),   # will calibrate after gathering data
        max_velocity=0.3,     # 0.3 m/s max speed
        deadband=0.002
    )

    try:
        while True:
            await haply.send_force([0.0, 0.0, 0.0])
            raw = haply.get_latest_position()
            if raw is not None:
                # Apply calibration (dt automatically tracked)
                calibrated = calibrator.calibrate(raw)
                print(f"Raw: {raw}  ->  Calibrated: {calibrated}")
            else:
                print("No position data")
            await asyncio.sleep(0.01)  # ~100 Hz

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        await haply.close()

if __name__ == "__main__":
    asyncio.run(main())
