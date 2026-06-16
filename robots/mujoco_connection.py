import asyncio
import mujoco
import mujoco.viewer
import numpy as np

class MuJoCoConnection:
    """
    Simple wrapper to send joint position commands to a MuJoCo simulation.
    Launches a passive viewer that stays synchronized with the simulation.
    """

    def __init__(self, model, actuator_names=None, actuator_ids=None):
        """
        Args:
            model: MjModel instance.
            actuator_names: List of actuator names (optional if ids given).
            actuator_ids: List of actuator IDs (optional if names given).
        """
        self.model = model
        self.data = mujoco.MjData(model)
        self.actuator_ids = actuator_ids
        if actuator_names is not None and actuator_ids is None:
            self.actuator_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
                                  for name in actuator_names]
        self.viewer = None

    async def connect(self):
        """Launch the passive MuJoCo viewer."""
        # Launch passive viewer (runs in a background thread, non‑blocking)
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        # Wait a moment for the viewer to initialise
        await asyncio.sleep(0.1)
        print("[MuJoCo] Viewer launched and connected.")
        return True

    def set_joint_positions(self, qpos):
        """
        Apply joint positions via actuator control.
        Assumes actuators are configured for position control.
        """
        if self.actuator_ids is None:
            raise ValueError("No actuator ids provided")
        for i, act_id in enumerate(self.actuator_ids):
            self.data.ctrl[act_id] = qpos[i]

    def step(self):
        """Advance simulation by one timestep and refresh the viewer."""
        mujoco.mj_step(self.model, self.data)
        if self.viewer is not None:
            self.viewer.sync()

    def reset_to_preset(self, joint_angles):
        """Reset the simulation to a given joint configuration."""
        n_joints = len(joint_angles)
        self.data.qpos[:n_joints] = joint_angles
        self.data.qvel[:n_joints] = 0
        mujoco.mj_forward(self.model, self.data)
        self.set_joint_positions(joint_angles)
        if self.viewer is not None:
            self.viewer.sync()

    def close(self):
        """Close the viewer if it exists."""
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None