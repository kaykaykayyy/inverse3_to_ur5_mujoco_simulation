'''
Read haply position data and print it out, check position data
'''

import time
import numpy as np
import csv
import asyncio
import mujoco
import os
from datetime import datetime

from haply.haply_interface import HaplyInterface

PRESETS = {
    "ready": np.array([0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0]),
}

async def main():
    haply = HaplyInterface()
    await haply.connect()

    try:
        while True:
            # Send zero force to keep the connection alive and request an update
            await haply.send_force([0.0, 0.0, 0.0])
            
            # Get the latest position (non‑blocking)
            pos = haply.get_latest_position()
            if pos is not None:
                print(f"Haply Position: {pos}")
                # Inverse kinematics
                # joint_angles = inverse_kinematics(pos)
                # print(f"Joint angles: {joint_angles}")
            else:
                print("No position data yet")
            
            # Small sleep to control loop rate (e.g., 100 Hz)
            await asyncio.sleep(0.01)

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        await haply.close()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[INFO] Testing stopped")