import time
import numpy as np
import asyncio
from haply.haply_interface import HaplyInterface
from teleop.haply_calibrator import HaplyCalibrator
from robots.ur5_ik import UR5IK
from robots.mujoco_connection import MuJoCoConnection
import mujoco

# ==========================
# Configuration
# ==========================
TELEOP_BUTTON = "a"          # Dead‑man switch button
CONTROL_RATE = 100           # Hz (simulation steps per second)
RESET_SETTLE_TIME = 0.5      # seconds to hold reset before teleop

XML_PATH = r"C:\Users\slimp\OneDrive\Documents\SJTU\Summer 2026\MuJoCo\ur5_arm\universal_robots_ur5e\scene.xml"
ACTUATOR_NAMES = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]

PRESETS = {
    "ready": np.array([0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0]),
    "look_down": np.array([0.0, -np.pi/2,   np.pi/2,  -np.pi/2,  -np.pi/2, 0.0])
}

async def main():
    # 1. Connect to Haply
    haply = HaplyInterface()
    await haply.connect()

    # 2. Calibrator (tune as needed)
    calibrator = HaplyCalibrator(
        filter_type='butter',
        cutoff_freq=5.0,
        fs=100.0,
        scale=1.0,
        offset=np.zeros(3),
        max_velocity=0.3,
        deadband=0.002
    )

    # 3. IK solver (orientation fixed to ready pose)
    ik_solver = UR5IK(orientation=None,
                      initial_joints=PRESETS["look_down"],
                      weight_position=1.0,
                      weight_orientation=0.0,
                      weight_joint_limits=10.0,
                      weight_singularity=0.01)

    # 4. MuJoCo setup
    model = mujoco.MjModel.from_xml_path(XML_PATH)
    mujoco_conn = MuJoCoConnection(model, actuator_names=ACTUATOR_NAMES)
    await mujoco_conn.connect()

    # Reset simulation to ready position initially
    mujoco_conn.reset_to_preset(PRESETS["look_down"])

    teleop_active = False
    print("[INFO] Ready. Press and hold 'A' on the Verse Grip to start teleoperation.")

    try:
        while True:
            # Send zero force to keep Haply connection alive
            await haply.send_force([0.0, 0.0, 0.0])

            # Read raw position and update calibrator
            raw = haply.get_latest_position()
            calibrated = None
            if raw is not None:
                calibrated = calibrator.calibrate(raw)

            # Check dead‑man switch
            button_pressed = True #haply.get_button_state(TELEOP_BUTTON)

            if button_pressed and not teleop_active:
                # Start teleoperation: reset arm to ready and activate
                print("[TELEOP] Activating...")
                mujoco_conn.reset_to_preset(PRESETS["look_down"])
                ik_solver.set_initial_guess(PRESETS["look_down"]) # Synchronizes optimization seed

                # Optionally wait a short moment for the arm to settle
                await asyncio.sleep(RESET_SETTLE_TIME)
                teleop_active = True
                print("[TELEOP] Active. Move the Haply device.")

            elif not button_pressed and teleop_active:
                # Stop teleoperation
                print("[TELEOP] Stopped.")
                teleop_active = False
                # Optionally hold last position (do nothing, simulation continues)

            if teleop_active and calibrated is not None:
                # Compute IK from calibrated position
                joint_angles = ik_solver.compute_ik(calibrated)
                if joint_angles is not None:
                    # Apply to MuJoCo actuators
                    mujoco_conn.set_joint_positions(joint_angles)
                    print(f"Position: {calibrated} -> Joints: {joint_angles}")
                else:
                    print("IK failed – holding last position")
            else:
                # If teleop inactive but we have a calibrated position, just print for debugging
                if calibrated is not None:
                    pass  # silent

            # Step simulation (fixed timestep)
            mujoco_conn.step()
            # Maintain control rate (approx)
            await asyncio.sleep(1.0 / CONTROL_RATE)

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        await haply.close()

if __name__ == "__main__":
    asyncio.run(main())
