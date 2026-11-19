"""Offline small detached-cluster cleanup; geometric heuristic, not free-space proof.

Connectivity is evaluated on the COMPLETE map. Only components fully inside an
explicit inspection region may be removed. Thin-structure evidence takes priority.
"""
import numpy as np
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components


def detached_clusters(points, protected, lower, upper, radius=.15,
                      maximum_points=30, maximum_extent=.60, floor_height=.12):
    p = np.asarray(points, dtype=float)
    protected = np.asarray(protected, dtype=bool)
    lower, upper = np.asarray(lower), np.asarray(upper)
    if p.ndim != 2 or p.shape[1] != 3 or protected.shape != (len(p),):
        raise ValueError('Expected Nx3 points and N protection flags')
    if not np.isfinite(p).all() or radius <= 0 or np.any(lower >= upper):
        raise ValueError('Invalid geometry or inspection bounds')
    removed = np.zeros(len(p), bool)
    if not len(p):
        return removed, []
    pairs = cKDTree(p).query_pairs(radius, output_type='ndarray')
    graph = coo_matrix((np.ones(len(pairs), bool), (pairs[:, 0], pairs[:, 1])),
                       shape=(len(p), len(p)))
    _, labels = connected_components(graph, directed=False)
    counts = np.bincount(labels)
    order = np.argsort(labels)
    offsets = np.r_[0, np.cumsum(counts)]
    report = []
    for identity in np.flatnonzero(counts <= maximum_points):
        ids = order[offsets[identity]:offsets[identity + 1]]
        q = p[ids]
        lo, hi = q.min(0), q.max(0)
        if (np.any(lo <= lower) or np.any(hi >= upper)
                or lo[2] <= floor_height or np.max(hi - lo) > maximum_extent
                or protected[ids].any()):
            continue
        removed[ids] = True
        report.append(dict(points=len(ids), min_xyz_m=lo.tolist(),
                           max_xyz_m=hi.tolist()))
    return removed, report
