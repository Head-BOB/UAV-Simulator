"""
Pareto Analysis & Multi-Objective Ranking Tools
Phase 3 - Dev 4 (Deliverable P4)
Tests: Q-01 through Q-08
"""

import numpy as np
from typing import List, Tuple

def dominates(a: np.ndarray, b: np.ndarray) -> bool:
    """
    Returns True if design 'a' dominates design 'b' (all objectives minimized).
    a dominates b iff for all i, a[i] <= b[i], and for at least one i, a[i] < b[i].
    Returns standard Python bool.
    """
    a = np.asarray(a)
    b = np.asarray(b)
    return bool(np.all(a <= b) and np.any(a < b))

def fast_nondominated_sort(objectives: np.ndarray) -> List[List[int]]:
    """
    Partitions design indices into Pareto ranks (Front 0, Front 1, ...).
    Matches Section 4.3 hand check.
    """
    N = len(objectives)
    domination_counts = np.zeros(N, dtype=int)
    dominated_sets = [[] for _ in range(N)]
    fronts: List[List[int]] = [[]]

    for p in range(N):
        for q in range(N):
            if p == q:
                continue
            if dominates(objectives[p], objectives[q]):
                dominated_sets[p].append(q)
            elif dominates(objectives[q], objectives[p]):
                domination_counts[p] += 1

        if domination_counts[p] == 0:
            fronts[0].append(p)

    i = 0
    while len(fronts[i]) > 0:
        next_front = []
        for p in fronts[i]:
            for q in dominated_sets[p]:
                domination_counts[q] -= 1
                if domination_counts[q] == 0:
                    next_front.append(q)
        i += 1
        if len(next_front) > 0:
            fronts.append(next_front)
        else:
            break

    return fronts

def crowding_distance(front_objectives: np.ndarray) -> np.ndarray:
    """
    Computes crowding distances within a single front to maintain diversity.
    End points receive np.inf.
    Matches Section 4.3 and Test Q-04, Q-05.
    """
    front_objectives = np.asarray(front_objectives)
    n_points, n_objs = front_objectives.shape
    if n_points <= 2:
        return np.full(n_points, np.inf)

    distances = np.zeros(n_points, dtype=float)

    for m in range(n_objs):
        sorted_indices = np.argsort(front_objectives[:, m])
        distances[sorted_indices[0]] = np.inf
        distances[sorted_indices[-1]] = np.inf

        obj_range = front_objectives[sorted_indices[-1], m] - front_objectives[sorted_indices[0], m]
        if obj_range < 1e-12:
            continue

        for i in range(1, n_points - 1):
            prev_val = front_objectives[sorted_indices[i - 1], m]
            next_val = front_objectives[sorted_indices[i + 1], m]
            distances[sorted_indices[i]] += (next_val - prev_val) / obj_range

    return distances

def hypervolume_2d(front_2d: np.ndarray, reference_point: Tuple[float, float]) -> float:
    """
    Calculates exact 2D hypervolume dominated by a front up to a fixed reference point.
    Matches Section 4.6 and Test Q-06.
    """
    front_2d = np.asarray(front_2d)
    ref_x, ref_y = reference_point

    valid_mask = (front_2d[:, 0] <= ref_x) & (front_2d[:, 1] <= ref_y)
    valid_pts = front_2d[valid_mask]
    if len(valid_pts) == 0:
        return 0.0

    front_indices = fast_nondominated_sort(valid_pts)[0]
    pareto_pts = valid_pts[front_indices]

    sorted_idx = np.argsort(pareto_pts[:, 0])
    sorted_pts = pareto_pts[sorted_idx]

    total_hv = 0.0
    current_y = ref_y

    for pt in sorted_pts:
        x, y = pt[0], pt[1]
        if y < current_y:
            total_hv += (ref_x - x) * (current_y - y)
            current_y = y

    return float(total_hv)
