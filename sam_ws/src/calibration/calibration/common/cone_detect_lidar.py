"""LiDAR cone apex detection (plane RANSAC → DBSCAN → cone RANSAC per cluster)."""
from __future__ import annotations

from typing import Optional
import math
import numpy as np

from .cone_ransac import fit_cone_ransac
from .board_definition import BoardDef


def _read_pcd_simple(path: str) -> np.ndarray:
    """Minimal PCD (ASCII/binary xyz) reader."""
    import struct
    header = {}
    with open(path, 'rb') as f:
        while True:
            raw = f.readline()
            line = raw.decode('ascii', errors='ignore').strip()
            if line.startswith('#') or not line:
                continue
            if line.startswith('DATA'):
                header['DATA'] = line.split()[1]
                break
            parts = line.split()
            if len(parts) >= 2:
                header[parts[0]] = parts[1:]
        width = int(header['WIDTH'][0])
        height = int(header.get('HEIGHT', ['1'])[0])
        n = width * height
        data_mode = header.get('DATA', 'ascii')
        if data_mode == 'ascii':
            pts = []
            for _ in range(n):
                line = f.readline().decode('ascii', errors='ignore').strip()
                if not line:
                    continue
                tok = line.split()
                if len(tok) >= 3:
                    pts.append([float(tok[0]), float(tok[1]), float(tok[2])])
            return np.asarray(pts, dtype=float)
        elif data_mode in ('binary', 'binary_compressed'):
            if data_mode == 'binary_compressed':
                raise NotImplementedError("binary_compressed PCD not supported")
            raw = f.read(n * 12)
            data = np.frombuffer(raw, dtype='<f4', count=n * 3).reshape(-1, 3)
            return data.astype(float)
        else:
            raise ValueError(f"Unknown PCD DATA type: {data_mode}")


def load_pcd(path: str) -> np.ndarray:
    """Load point cloud from PCD. Falls back to Open3D if available."""
    try:
        import open3d as o3d
        pcd = o3d.io.read_point_cloud(path)
        pts = np.asarray(pcd.points)
        if pts.size > 0:
            return pts
    except Exception:
        pass
    return _read_pcd_simple(path)


def ransac_plane(pts: np.ndarray, tol_m: float, n_iters: int = 2000,
                 rng_seed: int = 42):
    """Simple RANSAC plane fit. Returns (a, b, c, d) with a²+b²+c²=1, inliers."""
    rng = np.random.default_rng(rng_seed)
    n = len(pts)
    if n < 10:
        return None, None
    best_inliers = None
    best_plane = None
    for _ in range(n_iters):
        idx = rng.choice(n, 3, replace=False)
        p1, p2, p3 = pts[idx]
        v1 = p2 - p1
        v2 = p3 - p1
        normal = np.cross(v1, v2)
        nn = np.linalg.norm(normal)
        if nn < 1e-9:
            continue
        normal = normal / nn
        d = -np.dot(normal, p1)
        dists = np.abs(pts @ normal + d)
        inliers = dists < tol_m
        if best_inliers is None or inliers.sum() > best_inliers.sum():
            best_inliers = inliers
            best_plane = (float(normal[0]), float(normal[1]),
                          float(normal[2]), float(d))
    if best_plane is None:
        return None, None
    # Refine via least-squares on inliers
    inl = best_inliers
    A = pts[inl]
    centroid = A.mean(axis=0)
    cov = (A - centroid).T @ (A - centroid)
    eigvals, eigvecs = np.linalg.eigh(cov)
    normal = eigvecs[:, 0]
    d = -np.dot(normal, centroid)
    plane = (float(normal[0]), float(normal[1]), float(normal[2]), float(d))
    dists = np.abs(pts @ normal + d)
    inliers = dists < tol_m
    return plane, inliers


def _dbscan(pts: np.ndarray, eps: float, min_samples: int) -> np.ndarray:
    """Minimal DBSCAN implementation (scipy cKDTree for neighborhood queries)."""
    try:
        from scipy.spatial import cKDTree
    except ImportError:
        raise RuntimeError("scipy is required for DBSCAN. pip install scipy")
    n = len(pts)
    labels = -np.ones(n, dtype=int)
    tree = cKDTree(pts)
    visited = np.zeros(n, dtype=bool)
    cluster_id = -1
    for i in range(n):
        if visited[i]:
            continue
        visited[i] = True
        neighbors = tree.query_ball_point(pts[i], r=eps)
        if len(neighbors) < min_samples:
            continue  # noise
        cluster_id += 1
        labels[i] = cluster_id
        queue = list(neighbors)
        while queue:
            j = queue.pop()
            if not visited[j]:
                visited[j] = True
                j_nbrs = tree.query_ball_point(pts[j], r=eps)
                if len(j_nbrs) >= min_samples:
                    queue.extend(j_nbrs)
            if labels[j] == -1:
                labels[j] = cluster_id
    return labels


def detect_cones_lidar(pcd_path: str, board_def: BoardDef) -> Optional[np.ndarray]:
    """Returns (4, 3) apex coordinates in LiDAR frame, ordered by board cone id.

    None if detection fails (e.g., cluster count != 4).
    """
    pts = load_pcd(pcd_path)
    if len(pts) < 100:
        return None

    cfg = board_def.detection
    plane, inliers = ransac_plane(pts,
                                  tol_m=cfg.get('plane_tol_m', 0.01),
                                  n_iters=cfg.get('plane_iters', 2000))
    if plane is None:
        return None
    a, b, c, d = plane
    normal = np.array([a, b, c], dtype=float)

    # Orient normal to point "above" the board (away from sensor if most pts are below)
    signed = pts @ normal + d
    if np.median(signed) > 0:
        # Median above plane → likely LiDAR sees the board from below? Flip.
        normal = -normal
        d = -d

    # Points above the plane by height_margin_frac * cone_height
    cone_h = float(board_def.cones[0].height_m) if board_def.cones else 0.05
    margin = cfg.get('height_margin_frac', 0.25) * cone_h
    signed = pts @ normal + d
    above_mask = signed > margin
    candidates = pts[above_mask]
    if len(candidates) < 20:
        return None

    labels = _dbscan(candidates,
                     eps=cfg.get('dbscan_eps_m', 0.05),
                     min_samples=cfg.get('min_cluster_pts', 10))
    unique = [l for l in set(labels) if l >= 0]
    if len(unique) < 4:
        return None

    # Keep 4 largest clusters
    cluster_sizes = [(l, int((labels == l).sum())) for l in unique]
    cluster_sizes.sort(key=lambda x: -x[1])
    kept = [l for l, _ in cluster_sizes[:4]]

    apexes = []
    half = float(board_def.cones[0].half_angle_rad)
    for cid in kept:
        cluster_pts = candidates[labels == cid]
        fit = fit_cone_ransac(cluster_pts, axis_prior=normal,
                              half_angle_rad=half,
                              tol_m=cfg.get('cone_ransac_tol_m', 0.005))
        if fit is None:
            continue
        apexes.append(fit.apex)

    if len(apexes) != 4:
        return None

    apexes = np.stack(apexes, axis=0)
    # Order by matching to expected board apex positions (minimize total distance
    # after rigid transform from LiDAR → board frame). We use a simple pairing
    # based on projecting apex onto plane basis and matching quadrants.
    apexes_ordered = _order_apexes_by_geometry(apexes, board_def)
    return apexes_ordered


def _order_apexes_by_geometry(apexes_lidar: np.ndarray,
                              board_def: BoardDef) -> np.ndarray:
    """Map 4 detected apexes to board order using pairwise-distance matching."""
    from itertools import permutations
    expected = board_def.apex_3d_board()
    exp_d = np.linalg.norm(expected[:, None, :] - expected[None, :, :], axis=-1)
    best_perm = None
    best_score = float('inf')
    for perm in permutations(range(4)):
        perm_arr = np.asarray(perm)
        got = apexes_lidar[perm_arr]
        got_d = np.linalg.norm(got[:, None, :] - got[None, :, :], axis=-1)
        score = np.linalg.norm(got_d - exp_d)
        if score < best_score:
            best_score = score
            best_perm = perm_arr
    return apexes_lidar[best_perm]
