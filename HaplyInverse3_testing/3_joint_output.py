'''
Read haply position data, calibrates,input numerical IK from robot look down preset.  
'''
import time
import numpy as np
import asyncio
from haply.haply_interface import HaplyInterface
from teleop.haply_calibrator import HaplyCalibrator
from robots.ur5_ik import UR5IK

async def main():
    # 1. Connect to Haply
    haply = HaplyInterface()
    await haply.connect()

    # 2. Create calibrator
    calibrator = HaplyCalibrator(
        filter_type='butter',
        cutoff_freq=5.0,      # low‑pass at 5 Hz
        fs=100.0,
        scale=1.0,            # no amplification yet
        offset=np.zeros(3),   # will calibrate after gathering data
        max_velocity=0.3,     # 0.3 m/s max speed
        deadband=0.002
    )

    # 3. Initialize IK solver
    ik_solver = UR5IK(
        orientation=None,
        weight_position=1.0,
        weight_orientation=0.0,
        weight_joint_limits=10.0,
        weight_singularity=0.01
    )

    # 4. Define the initial look_down configuration as your baseline state
    LOOK_DOWN_PRESET = np.array([0.0, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0.0])
    
    # This variable tracks the seed position. We initialize it to look_down.
    current_joint_seed = LOOK_DOWN_PRESET.copy()

    print("[INFO] Warm-start configured to 'look_down'. Streaming joint angles and calculation times...")

    try:
        while True:
            # Send zero force to keep Haply connection alive
            await haply.send_force([0.0, 0.0, 0.0])
            
            raw = haply.get_latest_position()
            if raw is not None:
                # Apply calibration filtering
                calibrated = calibrator.calibrate(raw)
                
                # Benchmark start time for the IK computation block
                ik_start = time.perf_counter()
                
                # Pass BOTH the target position AND our current joint seed into your solver
                # Note: Depending on how your UR5IK class is written, the seed parameter 
                # might be named 'q0', 'initial_guess', or 'seed'. Adjust if necessary.
                joint_angles = ik_solver.compute_ik(calibrated, initial_guess=current_joint_seed)
                
                # Calculate duration in milliseconds
                ik_duration_ms = (time.perf_counter() - ik_start) * 1000
                
                if joint_angles is not None:
                    print(f"XYZ: {np.round(calibrated, 4)} -> Joints: {np.round(joint_angles, 4)} | IK Time: {ik_duration_ms:.1f}ms")
                    
                    # WARM-START UPDATE: Save these successful angles to be the seed for the NEXT frame.
                    # This ensures the solver only has to calculate tiny incremental changes.
                    current_joint_seed = np.array(joint_angles)
                else:
                    print(f"XYZ: {np.round(calibrated, 4)} -> [IK FAILED] | Resetting seed to baseline layout | IK Time: {ik_duration_ms:.1f}ms")
                    # If it fails, fall back to look_down so it doesn't get permanently lost
                    current_joint_seed = LOOK_DOWN_PRESET.copy()
            else:
                print("No position data received from device buffer.")
                
            # Maintain tracking loop rate
            await asyncio.sleep(0.01)

    except KeyboardInterrupt:
        print("\nExiting diagnostic script...")
    finally:
        await haply.close()

if __name__ == "__main__":
    asyncio.run(main())