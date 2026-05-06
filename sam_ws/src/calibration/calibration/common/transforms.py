"""SE(3) / so(3) utilities — pure numpy, no external deps beyond scipy (optional)."""
from __future__ import annotations

import math
import numpy as np


def skew(v):
    v = np.asarray(v, dtype=float).ravel()
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    ])


def rodrigues(rvec):
    """Rodrigues: so(3) vector → rotation matrix (3x3)."""
    rvec = np.asarray(rvec, dtype=float).ravel()
    theta = np.linalg.norm(rvec)
    if theta < 1e-12:
        return np.eye(3)
    k = rvec / theta
    K = skew(k)
    return np.eye(3) + math.sin(theta) * K + (1.0 - math.cos(theta)) * (K @ K)


def rotation_matrix_to_rvec(R):
    """Inverse Rodrigues: 3x3 rotation → so(3) vector (3,)."""
    R = np.asarray(R, dtype=float)
    cos_theta = (np.trace(R) - 1.0) * 0.5
    cos_theta = max(-1.0, min(1.0, cos_theta))
    theta = math.acos(cos_theta)
    if theta < 1e-12:
        return np.zeros(3)
    if abs(theta - math.pi) < 1e-6:
        # Symmetric case
        M = 0.5 * (R + np.eye(3))
        vec = np.array([math.sqrt(max(0.0, M[i, i])) for i in range(3)])
        return vec * theta
    s = 1.0 / (2.0 * math.sin(theta))
    return theta * s * np.array([R[2, 1] - R[1, 2],
                                 R[0, 2] - R[2, 0],
                                 R[1, 0] - R[0, 1]])


def se3_from_rvec_tvec(rvec, tvec):
    T = np.eye(4)
    T[:3, :3] = rodrigues(rvec)
    T[:3, 3] = np.asarray(tvec, dtype=float).ravel()
    return T


def se3_to_rvec_tvec(T):
    T = np.asarray(T, dtype=float)
    return rotation_matrix_to_rvec(T[:3, :3]), T[:3, 3].copy()


def se3_inverse(T):
    T = np.asarray(T, dtype=float)
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def se3_compose(*Ts):
    out = np.eye(4)
    for T in Ts:
        out = out @ T
    return out


def transform_points(T, pts):
    """T: 4x4, pts: (N,3) → (N,3)."""
    pts = np.asarray(pts, dtype=float)
    return pts @ T[:3, :3].T + T[:3, 3]


def rigid_transform_3d(src, dst):
    """Umeyama (no scale). src, dst: (N,3) with correspondences → T_src_to_dst."""
    src = np.asarray(src, dtype=float)
    dst = np.asarray(dst, dtype=float)
    assert src.shape == dst.shape and src.shape[1] == 3 and src.shape[0] >= 3
    c_s = src.mean(axis=0)
    c_d = dst.mean(axis=0)
    S = src - c_s
    D = dst - c_d
    H = S.T @ D
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    t = c_d - R @ c_s
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def se3_log(T):
    """SE(3) → R^6 (omega, v)."""
    R = T[:3, :3]
    rvec = rotation_matrix_to_rvec(R)
    theta = np.linalg.norm(rvec)
    t = T[:3, 3]
    if theta < 1e-8:
        v = t
    else:
        K = skew(rvec / theta)
        A = (np.eye(3)
             - 0.5 * theta * K
             + (1.0 - 0.5 * theta / math.tan(0.5 * theta)) * (K @ K))
        v = A @ t
    return np.concatenate([rvec, v])


def se3_median(Ts):
    """Robust mean of SE(3) via geodesic L1 approximation (few iterations)."""
    Ts = list(Ts)
    # Init: first sample
    T = Ts[0].copy()
    for _ in range(10):
        deltas = np.stack([se3_log(se3_inverse(T) @ Ti) for Ti in Ts], axis=0)
        norms = np.linalg.norm(deltas, axis=1)
        w = 1.0 / np.maximum(norms, 1e-6)
        w = w / w.sum()
        mean = (w[:, None] * deltas).sum(axis=0)
        if np.linalg.norm(mean) < 1e-8:
            break
        # Apply mean in tangent
        rvec = mean[:3]
        tvec = mean[3:]
        dT = se3_from_rvec_tvec(rvec, tvec)
        T = T @ dT
    return T


def angle_between(v1, v2):
    v1 = np.asarray(v1, dtype=float).ravel()
    v2 = np.asarray(v2, dtype=float).ravel()
    n1 = np.linalg.norm(v1)
    n2 = np.linalg.norm(v2)
    if n1 < 1e-12 or n2 < 1e-12:
        return 0.0
    c = np.dot(v1, v2) / (n1 * n2)
    c = max(-1.0, min(1.0, c))
    return math.acos(c)


def rvec_from_matrix(R):
    return rotation_matrix_to_rvec(R)


def matrix_from_rvec(rvec):
    return rodrigues(rvec)
