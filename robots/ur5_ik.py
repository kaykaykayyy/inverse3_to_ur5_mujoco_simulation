import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R
#from ur_kinematics import inverse

class UR5IK:
    """
    Numerical IK solver for UR5 robot.
    Tracks Cartesian position (x,y,z) with fixed orientation.
    """

    # UR5 DH parameters (from Universal Robots URDF)
    DH = {
        'a':     [0,        -0.425,   -0.3922,   0,        0,        0      ],
        'd':     [0.089159, 0,        0,        0.10915,  0.09465,  0.0823 ],
        'alpha': [np.pi/2,  0,        0,        np.pi/2,  -np.pi/2, 0      ]
    }

    # Joint limits (radians) – typical UR5 ranges
    JOINT_LIMITS = [
        (-2*np.pi, 2*np.pi),   # joint 1
        (-2*np.pi, 2*np.pi),   # joint 2
        (-2*np.pi, 2*np.pi),   # joint 3
        (-2*np.pi, 2*np.pi),   # joint 4
        (-2*np.pi, 2*np.pi),   # joint 5
        (-2*np.pi, 2*np.pi)    # joint 6
    ]

    def __init__(self, orientation=None, initial_joints=None, weight_position=1.0, 
                 weight_orientation=0.0, weight_joint_limits=10.0, weight_singularity=0.0):
        """
        Args:
            orientation: Desired end‑effector orientation (rotation matrix, quaternion, or scipy Rotation).
                         If None, orientation is locked to the "ready" pose.
            weight_position: Weight for position error cost.
            weight_orientation: Weight for orientation error cost (0 = ignore).
            weight_joint_limits: Penalty weight for exceeding joint limits.
            weight_singularity: Penalty weight for low manipulability (position Jacobian only).
        """
        self.weight_position = weight_position
        self.weight_orientation = weight_orientation
        self.weight_joint_limits = weight_joint_limits
        self.weight_singularity = weight_singularity

        # CHANGE HERE: Set default joints to initial_joints if provided, otherwise all 0s
        if initial_joints is not None:
            self.ready_joints = np.asarray(initial_joints).copy()
        else:
            self.ready_joints = np.zeros(6)
        
        # Compute the end‑effector pose at the ready configuration
        self.ready_pose = self.forward_kinematics(self.ready_joints)
        self.ready_position = self.ready_pose[:3, 3]
        self.ready_orientation = self.ready_pose[:3, :3]

        # Set desired orientation
        if orientation is None:
            self.desired_orientation = self.ready_orientation.copy()
        else:
            self.set_desired_orientation(orientation)

        # Store last computed joint angles for continuity
        self._last_joints = self.ready_joints.copy()

        # For singularity avoidance: small epsilon
        self._eps_singular = 1e-6

    # ------------------- Forward Kinematics -------------------
    def dh_transform(self, a, d, alpha, theta):
        """Standard DH transformation matrix."""
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0,             np.sin(alpha),                np.cos(alpha),                d],
            [0,             0,                            0,                            1]
        ])

    def forward_kinematics(self, joints):
        """Compute end‑effector transformation matrix (4x4) from joint angles."""
        T = np.eye(4)
        for i in range(6):
            T = T @ self.dh_transform(self.DH['a'][i], self.DH['d'][i],
                                       self.DH['alpha'][i], joints[i])
        return T

    def get_end_effector_pose(self, joints):
        """Return (position, orientation) as (3-vector, 3x3 matrix)."""
        T = self.forward_kinematics(joints)
        return T[:3, 3], T[:3, :3]

    # ------------------- Cost Function -------------------
    def _joint_limit_penalty(self, joints):
        """Quadratic penalty for violating joint limits."""
        penalty = 0.0
        for i, (low, high) in enumerate(self.JOINT_LIMITS):
            if joints[i] < low:
                penalty += (low - joints[i])**2
            elif joints[i] > high:
                penalty += (joints[i] - high)**2
        return penalty

    def _singularity_penalty(self, joints):
        """
        Penalise low manipulability using ONLY the position Jacobian (3x6).
        This avoids orientation‑related numerical issues.
        """
        eps = 1e-6
        J_pos = np.zeros((3, 6))
        pos0, _ = self.get_end_effector_pose(joints)
        for i in range(6):
            dq = np.zeros(6)
            dq[i] = eps
            pos1, _ = self.get_end_effector_pose(joints + dq)
            J_pos[:, i] = (pos1 - pos0) / eps

        # Compute manipulability: sqrt(det(J_pos * J_pos^T))
        JJT = J_pos @ J_pos.T
        # Regularise to avoid negative determinants from rounding errors
        JJT_reg = JJT + np.eye(3) * 1e-8
        try:
            det_val = np.linalg.det(JJT_reg)
            if det_val < 0:
                det_val = 0.0
            manipulability = np.sqrt(det_val)
        except:
            manipulability = 0.0

        # Return high penalty when manipulability is low
        return 1.0 / (manipulability + self._eps_singular)

    def _cost(self, joints, target_position):
        """
        Joint cost = position error + orientation error + joint limits + singularity.
        """
        joints = np.asarray(joints)
        pos, ori = self.get_end_effector_pose(joints)

        # Position error
        pos_err = np.linalg.norm(pos - target_position)
        cost = self.weight_position * pos_err**2

        # Orientation error (only if weight > 0)
        if self.weight_orientation > 0:
            R_err = self.desired_orientation @ ori.T
            # Clamp to avoid numerical issues with acos
            cos_angle = np.clip((np.trace(R_err) - 1) / 2, -1, 1)
            angle_err = np.arccos(cos_angle)
            cost += self.weight_orientation * angle_err**2

        # Joint limit penalty
        cost += self.weight_joint_limits * self._joint_limit_penalty(joints)

        # Singularity penalty (only if weight > 0)
        if self.weight_singularity > 0:
            cost += self.weight_singularity * self._singularity_penalty(joints)

        return cost

    # ------------------- Public Methods -------------------
    def set_desired_orientation(self, orientation):
        """Set target orientation (rotation matrix, quaternion, or scipy Rotation)."""
        if isinstance(orientation, R):
            self.desired_orientation = orientation.as_matrix()
        elif isinstance(orientation, np.ndarray) and orientation.shape == (3,3):
            self.desired_orientation = orientation
        elif isinstance(orientation, (tuple, list, np.ndarray)) and len(orientation) == 4:
            # quaternion (w,x,y,z)
            self.desired_orientation = R.from_quat(orientation).as_matrix()
        else:
            raise ValueError("Orientation must be 3x3 matrix, 4‑element quaternion, or scipy Rotation")

    def compute_ik(self, target_position, initial_guess=None, method='L-BFGS-B'):
        """
        Solve inverse kinematics for a desired Cartesian position.
        Args:
            target_position: (x, y, z) in meters.
            initial_guess: 6‑element array of joint angles (rad). If None, uses last solution.
            method: Scipy optimizer method.
        Returns:
            joint_angles (6‑element numpy array) or None if failed.
        """
        if initial_guess is not None:
            initial_guess = np.asarray(initial_guess)
        elif self._last_joints is not None:
            initial_guess = self._last_joints.copy()
        else:
            initial_guess = self.ready_joints.copy()

        # Bounds for joint angles
        bounds = self.JOINT_LIMITS

        # Optimise
        try:
            result = minimize(
                fun=self._cost,
                x0=initial_guess,
                args=(np.asarray(target_position),),
                method=method,
                bounds=bounds,
                # MODIFICATION: Reduced maxiter from 100 to 20, and loosened ftol from 1e-6 to 1e-4.
                # This guarantees the solver bails out fast once a highly-usable solution is found.
                options={'maxiter': 15, 'ftol': 1e-3}
            )
        except Exception as e:
            print(f"IK exception: {e}")
            return None

        if result.success:
            self._last_joints = result.x
            return result.x
        else:
            print(f"IK failed: {result.message}")
            return None

    def reset_to_ready(self):
        """Reset the last joint state to the ready position."""
        self._last_joints = self.ready_joints.copy()

    # ADD THIS NEW METHOD HERE:
    def set_initial_guess(self, joint_angles):
        """
        Manually override the internal tracking memory seed.
        Use this when resetting the simulation model to a specific preset.
        """
        self._last_joints = np.asarray(joint_angles).copy()

    @property
    def joint_angles(self):
        """Return the last successfully computed joint angles."""
        return self._last_joints.copy()