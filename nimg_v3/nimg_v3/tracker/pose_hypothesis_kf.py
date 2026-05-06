"""
Pose Hypothesis Kalman Filter — FoundationPose++ 내부용 pre-filter.

architecture §4.2 에 대응. Layer-D 의 `PoseKalmanFilter` 와 분리: 이쪽은 고노이즈
허용, 짧은 τ — FP refine 에 들어갈 "초기 가설" 을 만드는 것이 목적.

상태: 간단하게 t (3), q (4, unit), v (3), ω (3) 을 분리 관리.
"""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as R


class PoseHypothesisKF:
    def __init__(self, process_noise: float = 0.05, meas_noise: float = 0.02):
        self.q_noise = process_noise
        self.r_noise = meas_noise
        self.initialized = False
        self.t: np.ndarray = np.zeros(3)
        self.v: np.ndarray = np.zeros(3)
        self.quat: np.ndarray = np.array([0, 0, 0, 1], dtype=np.float64)  # x,y,z,w
        self.omega: np.ndarray = np.zeros(3)   # axis-angle rad/s
        self.last_ts: float | None = None

    # ------------------------------------------------------------------
    def reset(self, T: np.ndarray, ts: float):
        self.t = T[:3, 3].astype(np.float64)
        self.quat = R.from_matrix(T[:3, :3]).as_quat()
        self.v = np.zeros(3)
        self.omega = np.zeros(3)
        self.last_ts = ts
        self.initialized = True

    def predict(self, ts: float) -> tuple[np.ndarray, np.ndarray, float]:
        """Returns predicted (quat, t, dt)."""
        if not self.initialized:
            return self.quat.copy(), self.t.copy(), 0.0
        dt = max(0.0, ts - (self.last_ts or ts))
        t_pred = self.t + self.v * dt
        if np.linalg.norm(self.omega) > 1e-6:
            dq = R.from_rotvec(self.omega * dt).as_quat()
            quat_pred = _quat_mul(dq, self.quat)
        else:
            quat_pred = self.quat.copy()
        return quat_pred, t_pred, dt

    def update(self, T: np.ndarray, ts: float):
        """Measurement update with robust α blending (low-cost alternative to full KF)."""
        if not self.initialized:
            self.reset(T, ts)
            return
        dt = max(1e-3, ts - (self.last_ts or ts))
        t_meas = T[:3, 3].astype(np.float64)
        quat_meas = R.from_matrix(T[:3, :3]).as_quat()
        alpha_t = 1.0 - np.exp(-dt / (self.r_noise + 0.05))
        alpha_q = 1.0 - np.exp(-dt / (self.r_noise + 0.1))
        # velocity estimate from diff
        new_v = (t_meas - self.t) / dt
        self.v = 0.8 * self.v + 0.2 * new_v
        # angular velocity
        dq = _quat_mul(quat_meas, _quat_conj(self.quat))
        rvec = R.from_quat(dq).as_rotvec()
        new_omega = rvec / dt
        self.omega = 0.8 * self.omega + 0.2 * new_omega
        # blend state
        self.t = (1 - alpha_t) * self.t + alpha_t * t_meas
        self.quat = _slerp(self.quat, quat_meas, alpha_q)
        self.last_ts = ts


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def _quat_conj(q: np.ndarray) -> np.ndarray:
    return np.array([-q[0], -q[1], -q[2], q[3]])


def _slerp(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1; dot = -dot
    if dot > 0.9995:
        out = q0 + t * (q1 - q0)
    else:
        theta = np.arccos(np.clip(dot, -1.0, 1.0))
        sin_t = np.sin(theta)
        out = (np.sin((1 - t) * theta) / sin_t) * q0 + (np.sin(t * theta) / sin_t) * q1
    return out / (np.linalg.norm(out) + 1e-12)
