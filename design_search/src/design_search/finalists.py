"""
Finalist Selection & Knee-Point Analysis
Phase 3 - Dev 4 (Deliverable P8)
"""

import numpy as np
from typing import List, Dict, Any

def calculate_knee_point(front_2d_normalized: np.ndarray) -> int:
    """
    Computes the knee point on a normalized 2D front (largest distance to line x + y = 1).
    Distance = |x + y - 1| / sqrt(2).
    Matches Section 7.3 worked example.
    """
    pts = np.asarray(front_2d_normalized)
    distances = np.abs(pts[:, 0] + pts[:, 1] - 1.0) / np.sqrt(2.0)
    return int(np.argmax(distances))

def select_campaign_finalists(pareto_pop: np.ndarray, pareto_objs: np.ndarray) -> List[Dict[str, Any]]:
    """
    Selects up to 5 finalists for high-fidelity validation.
    """
    n_pts, n_objs = pareto_objs.shape
    finalists = []

    # 1. Extreme Points (best on each objective)
    for m in range(n_objs):
        best_idx = int(np.argmin(pareto_objs[:, m]))
        finalists.append({
            "type": f"extreme_obj_{m}",
            "index": best_idx,
            "parameters": pareto_pop[best_idx].tolist(),
            "objectives": pareto_objs[best_idx].tolist()
        })

    # 2. Knee Point (normalized trade-off)
    min_o = np.min(pareto_objs, axis=0)
    max_o = np.max(pareto_objs, axis=0)
    denom = np.where((max_o - min_o) < 1e-9, 1.0, max_o - min_o)
    norm_objs = (pareto_objs - min_o) / denom

    if n_objs >= 2:
        knee_idx = calculate_knee_point(norm_objs[:, :2])
        finalists.append({
            "type": "knee_point",
            "index": knee_idx,
            "parameters": pareto_pop[knee_idx].tolist(),
            "objectives": pareto_objs[knee_idx].tolist()
        })

    return finalists
