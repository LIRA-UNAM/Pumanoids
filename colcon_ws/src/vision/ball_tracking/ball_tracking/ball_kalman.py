"""
Constant-velocity Kalman filter for tracking a ball's (x, y) position on a field
from a noisy position sensor (e.g. geometry from a CV bounding box) that:

  * only reports position (no velocity),
  * produces occasional gross outliers,
  * only reports when the ball is visible (missing measurements otherwise).

State:        x = [px, py, vx, vy]^T   (position + velocity, velocity is estimated)
Measurement:  z = [px, py]^T           (position only)

Design choices
--------------
- Velocity is part of the state and is inferred from successive positions.
- Outliers are rejected with a chi-square gate on the Mahalanobis distance of the
  innovation, which scales the threshold by the filter's *current* uncertainty.
- Missing detections run predict() only, so uncertainty grows and the filter can
  coast through occlusion and re-lock when the ball reappears.
- If too many measurements are gated out in a row, the filter assumes it lost
  track and re-initializes on the next detection.
"""

import numpy as np


class BallKalmanFilter:
    def __init__(
        self,
        dt=1.0 / 30.0,          # nominal timestep (s), e.g. frame period
        process_accel_std=6.0,  # std of unmodeled acceleration (field-units / s^2)
        meas_pos_std=0.4,       # std of position measurement noise (field-units)
        gate_threshold=9.21,    # chi-square(2 dof) gate; 9.21 -> 99%, 5.99 -> 95%
        max_consecutive_rejections=6,
        init_vel_var=1e3,       # initial variance on the (unknown) velocity
    ):
        self.dt = dt
        self.q_std = process_accel_std
        self.r_std = meas_pos_std
        self.gate = gate_threshold
        self.max_rej = max_consecutive_rejections
        self.init_vel_var = init_vel_var

        # We observe position only.
        self.H = np.array([[1.0, 0, 0, 0],
                           [0, 1.0, 0, 0]])
        self.R = (self.r_std ** 2) * np.eye(2)

        self.x = None            # state estimate [px, py, vx, vy]
        self.P = None            # state covariance (4x4)
        self.initialized = False
        self.consec_rej = 0

    # --- model matrices -------------------------------------------------
    def _F(self, dt):
        """Constant-velocity state transition."""
        return np.array([[1, 0, dt, 0],
                         [0, 1, 0, dt],
                         [0, 0, 1,  0],
                         [0, 0, 0,  1]], dtype=float)

    def _Q(self, dt):
        """Process noise from a white-noise-acceleration model.

        Treat unknown acceleration as zero-mean white noise with std q_std.
        G maps acceleration [ax, ay] into the state, then Q = G diag(q^2) G^T.
        This correctly couples the position/velocity noise instead of using an
        ad-hoc diagonal.
        """
        G = np.array([[0.5 * dt * dt, 0],
                      [0, 0.5 * dt * dt],
                      [dt, 0],
                      [0, dt]], dtype=float)
        return G @ G.T * (self.q_std ** 2)

    # --- lifecycle ------------------------------------------------------
    def initialize(self, z):
        z = np.asarray(z, dtype=float)
        self.x = np.array([z[0], z[1], 0.0, 0.0])
        # Position known ~ to measurement noise; velocity is unknown -> large var.
        self.P = np.diag([self.r_std ** 2, self.r_std ** 2,
                          self.init_vel_var, self.init_vel_var])
        self.initialized = True
        self.consec_rej = 0

    def reset(self):
        """Drop the current track; the next detection will re-initialize."""
        self.x = None
        self.P = None
        self.initialized = False
        self.consec_rej = 0

    def predict(self, dt=None):
        if not self.initialized:
            return None
        dt = self.dt if dt is None else dt
        F = self._F(dt)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self._Q(dt)
        return self.x[:2].copy()

    def update(self, z, dt=None):
        """Advance one step.

        Parameters
        ----------
        z  : (2,) array-like measured [x, y], or None if the ball is not visible.
        dt : optional actual timestep for this step (defaults to self.dt).

        Returns the current position estimate (2,) or None before first fix.
        """
        # Wait for the first valid detection to initialize.
        if not self.initialized:
            if z is not None:
                self.initialize(z)
            return self.estimate()

        # Always predict forward.
        self.predict(dt)

        # No detection this frame -> coast on the prediction only.
        if z is None:
            return self.estimate()

        z = np.asarray(z, dtype=float)
        y = z - self.H @ self.x                       # innovation
        S = self.H @ self.P @ self.H.T + self.R       # innovation covariance
        d2 = float(y @ np.linalg.solve(S, y))         # squared Mahalanobis dist

        # Outlier gate.
        if d2 > self.gate:
            self.consec_rej += 1
            # Persistent rejection means our prediction has diverged from reality
            # (e.g. the ball changed direction while occluded). Re-acquire.
            if self.consec_rej >= self.max_rej:
                self.initialize(z)
            return self.estimate()

        # Accepted -> Kalman update (Joseph form for numerical stability).
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        I = np.eye(4)
        self.P = (I - K @ self.H) @ self.P @ (I - K @ self.H).T + K @ self.R @ K.T
        self.consec_rej = 0
        return self.estimate()

    # --- accessors ------------------------------------------------------
    def estimate(self):
        return None if not self.initialized else self.x[:2].copy()

    @property
    def position(self):
        return self.estimate()

    @property
    def velocity(self):
        return None if not self.initialized else self.x[2:].copy()

    @property
    def position_std(self):
        """1-sigma position uncertainty (useful for drawing a search region)."""
        return None if not self.initialized else np.sqrt(np.diag(self.P)[:2])
