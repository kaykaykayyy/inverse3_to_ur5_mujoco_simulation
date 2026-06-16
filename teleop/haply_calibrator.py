import numpy as np
from scipy.signal import butter, lfilter
from collections import deque
import time

class HaplyCalibrator:
    """
    Calibration and filtering pipeline for Haply Inverse3 position stream.
    
    Features:
        - Low‑pass Butterworth filter (or exponential smoothing)
        - Workspace remapping (scale + offset)
        - Speed (velocity) limiting
        - Optional deadband to ignore micro‑movements
    """

    def __init__(self,
                 filter_type='butter',    # 'butter' or 'exp'
                 cutoff_freq=5.0,         # Hz (only for butter)
                 fs=100.0,                # sample rate (Hz)
                 exp_alpha=0.2,           # smoothing factor for exp filter
                 scale=1.0,               # amplification factor
                 offset=np.zeros(3),      # translation offset
                 max_velocity=0.5,        # m/s
                 deadband=0.001):         # ignore changes below this (m)
        """
        Args:
            filter_type: 'butter' (Butterworth) or 'exp' (exponential smoothing)
            cutoff_freq: cutoff frequency for Butterworth filter (Hz)
            fs: expected sample rate (Hz)
            exp_alpha: smoothing factor (0..1), lower = more smoothing
            scale: scale factor for each axis (amplification)
            offset: [x, y, z] offset to add after scaling
            max_velocity: maximum allowed velocity (m/s) per axis
            deadband: if position change < deadband, keep previous value
        """
        self.filter_type = filter_type
        self.scale = np.asarray(scale) if isinstance(scale, (list, tuple, np.ndarray)) else np.array([scale, scale, scale])
        self.offset = np.asarray(offset)
        self.max_velocity = max_velocity
        self.deadband = deadband

        # State variables
        self._prev_calibrated = np.zeros(3)
        self._prev_raw = None
        self._last_timestamp = None

        # Exponential smoothing state
        self._exp_state = np.zeros(3)

        # Butterworth filter state (one filter per axis)
        if filter_type == 'butter':
            nyq = 0.5 * fs
            normal_cutoff = cutoff_freq / nyq
            self.b, self.a = butter(2, normal_cutoff, btype='low', analog=False)
            self._z = [np.zeros(2) for _ in range(3)]  # filter state per axis

    def _lowpass_butter(self, x_axis, axis_idx):
        """Apply Butterworth filter to a single axis."""
        y, self._z[axis_idx] = lfilter(self.b, self.a, [x_axis], zi=self._z[axis_idx])
        return y[0]

    def _lowpass_exp(self, x_new):
        """Exponential smoothing."""
        self._exp_state = self.exp_alpha * x_new + (1 - self.exp_alpha) * self._exp_state
        return self._exp_state.copy()

    def _apply_deadband(self, new_pos):
        """If movement is smaller than deadband, revert to previous."""
        diff = np.linalg.norm(new_pos - self._prev_calibrated)
        if diff < self.deadband:
            return self._prev_calibrated.copy()
        return new_pos

    def _apply_velocity_limit(self, new_pos, dt):
        """Limit the change per axis to max_velocity * dt."""
        if dt <= 0 or self.max_velocity <= 0:
            return new_pos
        delta = new_pos - self._prev_calibrated
        max_delta = self.max_velocity * dt
        if np.linalg.norm(delta) > max_delta:
            delta = delta / np.linalg.norm(delta) * max_delta
        return self._prev_calibrated + delta

    def calibrate(self, raw_pos, dt=None):
        """
        Apply full calibration pipeline to a raw position.

        Args:
            raw_pos: numpy array (x, y, z) in meters from Haply
            dt: time since last call (s). If None, internal timestamp used.

        Returns:
            calibrated position (x, y, z) as numpy array
        """
        if raw_pos is None:
            return None

        # 1. Scale and offset (raw -> workspace)
        scaled = raw_pos * self.scale + self.offset

        # 2. Filter
        if self.filter_type == 'butter':
            filtered = np.array([
                self._lowpass_butter(scaled[0], 0),
                self._lowpass_butter(scaled[1], 1),
                self._lowpass_butter(scaled[2], 2)
            ])
        else:  # exponential
            filtered = self._lowpass_exp(scaled)

        # 3. Deadband
        deadbanded = self._apply_deadband(filtered)

        # 4. Velocity limiting
        if dt is None:
            if self._last_timestamp is not None:
                dt = time.time() - self._last_timestamp
                dt = max(0.001, min(dt, 0.1))  # clamp
            else:
                dt = 0.01  # default
        limited = self._apply_velocity_limit(deadbanded, dt)

        # Update state
        self._prev_calibrated = limited.copy()
        self._prev_raw = raw_pos.copy()
        self._last_timestamp = time.time()

        return limited

    def reset(self):
        """Reset filter states and previous positions."""
        self._prev_calibrated = np.zeros(3)
        self._prev_raw = None
        self._last_timestamp = None
        self._exp_state = np.zeros(3)
        if self.filter_type == 'butter':
            self._z = [np.zeros(2) for _ in range(3)]

    def set_scale(self, scale):
        """Update scale factor (amplification) per axis."""
        self.scale = np.asarray(scale)

    def set_offset(self, offset):
        """Update workspace offset."""
        self.offset = np.asarray(offset)

    def set_max_velocity(self, v_max):
        """Update maximum allowed Cartesian velocity (m/s)."""
        self.max_velocity = v_max