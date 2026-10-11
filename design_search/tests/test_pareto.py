"""
Unit Tests for Pareto Tools & Genetic Operators
Phase 3 - Dev 4 (Tests Q-01 to Q-12)
"""

import sys
import os
import numpy as np
import pytest

# Add src to python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src")))

from design_search.pareto import (
    dominates,
    fast_nondominated_sort,
    crowding_distance,
    hypervolume_2d
)
from design_search.sampling import latin_hypercube
from design_search.operators import (
    simulated_binary_crossover,
    polynomial_mutation,
    constraint_dominates
)

def test_q01_dominance():
    assert dominates(np.array([1.0, 5.0]), np.array([2.0, 6.0])) is True
    # Identical points do not dominate each other
    assert dominates(np.array([1.0, 5.0]), np.array([1.0, 5.0])) is False
    assert dominates(np.array([2.0, 6.0]), np.array([1.0, 5.0])) is False

def test_q02_q03_six_design_fronts():
    # Designs: (1,5), (2,3), (3,4), (4,1), (2,3), (5,5)
    objs = np.array([
        [1.0, 5.0],  # 0
        [2.0, 3.0],  # 1
        [3.0, 4.0],  # 2
        [4.0, 1.0],  # 3
        [2.0, 3.0],  # 4 (duplicate of 1)
        [5.0, 5.0],  # 5
    ])
    fronts = fast_nondominated_sort(objs)
    # Expected: Front 0 = {0, 1, 3, 4}, Front 1 = {2}, Front 2 = {5}
    assert set(fronts[0]) == {0, 1, 3, 4}
    assert set(fronts[1]) == {2}
    assert set(fronts[2]) == {5}

def test_q04_crowding_distance_three_points():
    pts = np.array([
        [1.0, 5.0],
        [2.0, 3.0],
        [4.0, 1.0]
    ])
    dists = crowding_distance(pts)
    assert np.isinf(dists[0])
    assert abs(dists[1] - 2.0) < 1e-9
    assert np.isinf(dists[2])

def test_q05_crowding_distance_five_points():
    pts = np.array([
        [1.0, 5.0],
        [2.0, 3.0],
        [3.0, 2.0],
        [4.0, 1.5],
        [6.0, 1.0]
    ])
    dists = crowding_distance(pts)
    assert np.isinf(dists[0])
    assert abs(dists[1] - 1.15) < 1e-9
    assert abs(dists[2] - 0.775) < 1e-9
    assert abs(dists[3] - 0.85) < 1e-9
    assert np.isinf(dists[4])

def test_q06_q07_q08_hypervolume():
    front = np.array([
        [1.0, 5.0],
        [2.0, 3.0],
        [4.0, 1.0]
    ])
    ref = (6.0, 6.0)
    # Q-06: Base hypervolume = 5 + 8 + 4 = 17
    hv = hypervolume_2d(front, ref)
    assert abs(hv - 17.0) < 1e-9

    # Q-07: Adding a dominated point does not change hypervolume
    front_with_dom = np.vstack([front, [5.0, 5.0]])
    hv_dom = hypervolume_2d(front_with_dom, ref)
    assert abs(hv_dom - 17.0) < 1e-9

    # Q-08: Points outside reference are ignored without error
    front_outside = np.vstack([front, [7.0, 7.0]])
    hv_out = hypervolume_2d(front_outside, ref)
    assert abs(hv_out - 17.0) < 1e-9

def test_q09_latin_hypercube():
    N, d = 10, 3
    sample = latin_hypercube(N, d, seed=42)
    # Check that each 1/N interval has exactly one sample per dimension
    for j in range(d):
        bins = np.floor(sample[:, j] * N).astype(int)
        assert len(np.unique(bins)) == N

def test_q10_constraint_domination():
    # Feasible (viol=0.0) vs Infeasible (viol=0.5)
    assert constraint_dominates(0, 0.0, 0, 0.5) == 1
    assert constraint_dominates(0, 0.5, 0, 0.0) == -1
    # Two infeasible: V = 0.1 vs V = 0.3 -> smaller V wins
    assert constraint_dominates(0, 0.1, 0, 0.3) == 1
    assert constraint_dominates(0, 0.3, 0, 0.1) == -1

def test_q11_sbx_bounds():
    bounds = [(0.0, 10.0), (-5.0, 5.0), (100.0, 500.0)]
    rng = np.random.default_rng(123)
    p1 = np.array([2.0, 0.0, 200.0])
    p2 = np.array([8.0, 2.0, 400.0])

    for _ in range(10_000):
        c1, c2 = simulated_binary_crossover(p1, p2, bounds, eta_c=20.0, rng=rng)
        for i in range(len(bounds)):
            assert bounds[i][0] <= c1[i] <= bounds[i][1]
            assert bounds[i][0] <= c2[i] <= bounds[i][1]
