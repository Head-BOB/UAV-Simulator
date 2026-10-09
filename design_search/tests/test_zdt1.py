"""
Proof of Search: ZDT1 Benchmark Acceptance Test (ST3-08)
Phase 3 - Dev 4 (Deliverable P6)
"""

import sys
import os
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src")))

from design_search.nsga2 import NSGA2Optimizer

def zdt1_eval(x: np.ndarray):
    """
    Standard ZDT1 benchmark: 30 continuous variables in [0, 1].
    f1 = x1
    g = 1 + 9 * sum(x2..x30) / 29
    f2 = g * (1 - sqrt(f1 / g))
    """
    f1 = float(x[0])
    g = 1.0 + (9.0 / 29.0) * float(np.sum(x[1:]))
    f2 = g * (1.0 - np.sqrt(f1 / g))
    violation = 0.0 # Unconstrained benchmark
    return np.array([f1, f2]), violation

def test_st3_08_zdt1_benchmark():
    n_vars = 30
    bounds = [(0.0, 1.0) for _ in range(n_vars)]

    optimizer = NSGA2Optimizer(
        eval_fn=zdt1_eval,
        bounds=bounds,
        pop_size=100,
        n_generations=200,
        crossover_prob=0.9,
        mutation_prob=1.0 / n_vars,
        eta_c=20.0,
        eta_m=20.0,
        seed=42
    )

    results = optimizer.optimize()
    pareto_objs = results["pareto_objs"]

    # Calculate distance to analytical true curve: f2_true = 1 - sqrt(f1)
    f1_vals = pareto_objs[:, 0]
    f2_actual = pareto_objs[:, 1]
    f2_true = 1.0 - np.sqrt(f1_vals)

    vertical_distances = np.abs(f2_actual - f2_true)
    mean_dist = float(np.mean(vertical_distances))
    worst_dist = float(np.max(vertical_distances))

    print(f"\nZDT1 Benchmark Results:")
    print(f"  Front Points Count: {len(pareto_objs)}")
    print(f"  Mean Distance to True Front: {mean_dist:.4f} (Criterion: < 0.02)")
    print(f"  Worst Distance to True Front: {worst_dist:.4f} (Criterion: < 0.10)")

    assert mean_dist < 0.02, f"ZDT1 mean distance {mean_dist:.4f} exceeded 0.02"
    assert worst_dist < 0.10, f"ZDT1 worst distance {worst_dist:.4f} exceeded 0.10"
