"""
Genetic Operators: SBX, Polynomial Mutation, and Constraint Domination
Phase 3 - Dev 4 (Deliverable P4)
Tests: Q-10, Q-11, Q-12
"""

import numpy as np
from typing import Tuple, List

def simulated_binary_crossover(
    parent1: np.ndarray,
    parent2: np.ndarray,
    bounds: List[Tuple[float, float]],
    eta_c: float = 20.0,
    rng: np.random.Generator = None
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Simulated Binary Crossover (SBX) for continuous decision variables.
    Children are strictly bounded within `bounds`.
    """
    if rng is None:
        rng = np.random.default_rng()

    p1 = np.asarray(parent1, dtype=float)
    p2 = np.asarray(parent2, dtype=float)
    c1 = np.copy(p1)
    c2 = np.copy(p2)

    for i in range(len(p1)):
        lb, ub = bounds[i]
        if abs(p1[i] - p2[i]) > 1e-12:
            y1 = min(p1[i], p2[i])
            y2 = max(p1[i], p2[i])

            rand = rng.uniform(0.0, 1.0)
            beta = 1.0 + (2.0 * (y1 - lb) / (y2 - y1))
            alpha = 2.0 - (beta ** -(eta_c + 1.0))
            if rand <= (1.0 / alpha):
                beta_q = (rand * alpha) ** (1.0 / (eta_c + 1.0))
            else:
                beta_q = (1.0 / (2.0 - (rand * alpha))) ** (1.0 / (eta_c + 1.0))
            c1_val = 0.5 * ((y1 + y2) - beta_q * (y2 - y1))

            beta = 1.0 + (2.0 * (ub - y2) / (y2 - y1))
            alpha = 2.0 - (beta ** -(eta_c + 1.0))
            if rand <= (1.0 / alpha):
                beta_q = (rand * alpha) ** (1.0 / (eta_c + 1.0))
            else:
                beta_q = (1.0 / (2.0 - (rand * alpha))) ** (1.0 / (eta_c + 1.0))
            c2_val = 0.5 * ((y1 + y2) + beta_q * (y2 - y1))

            c1[i] = np.clip(c1_val, lb, ub)
            c2[i] = np.clip(c2_val, lb, ub)

    return c1, c2

def polynomial_mutation(
    individual: np.ndarray,
    bounds: List[Tuple[float, float]],
    eta_m: float = 20.0,
    prob_mut: float = 0.1,
    rng: np.random.Generator = None
) -> np.ndarray:
    """
    Polynomial mutation for continuous parameters.
    Guarantees mutated values remain strictly within `bounds`.
    """
    if rng is None:
        rng = np.random.default_rng()

    mutated = np.copy(individual)
    for i in range(len(mutated)):
        if rng.uniform(0.0, 1.0) <= prob_mut:
            lb, ub = bounds[i]
            y = mutated[i]
            delta1 = (y - lb) / (ub - lb)
            delta2 = (ub - y) / (ub - lb)
            rand = rng.uniform(0.0, 1.0)
            mut_pow = 1.0 / (eta_m + 1.0)

            if rand < 0.5:
                xy = 1.0 - delta1
                val = 2.0 * rand + (1.0 - 2.0 * rand) * (xy ** (eta_m + 1.0))
                delta_q = (val ** mut_pow) - 1.0
            else:
                xy = 1.0 - delta2
                val = 2.0 * (1.0 - rand) + 2.0 * (rand - 0.5) * (xy ** (eta_m + 1.0))
                delta_q = 1.0 - (val ** mut_pow)

            mutated[i] = np.clip(y + delta_q * (ub - lb), lb, ub)

    return mutated

def constraint_dominates(
    rank_a: int, viol_a: float,
    rank_b: int, viol_b: float
) -> int:
    """
    Compares two individuals via NSGA-II constraint domination.
    Returns:
       1 if 'a' wins
      -1 if 'b' wins
       0 if tied
    Matches Section 4.4 and Test Q-10.
    """
    is_feas_a = viol_a <= 1e-9
    is_feas_b = viol_b <= 1e-9

    if is_feas_a and not is_feas_b:
        return 1
    if not is_feas_a and is_feas_b:
        return -1
    if not is_feas_a and not is_feas_b:
        if viol_a < viol_b:
            return 1
        elif viol_b < viol_a:
            return -1
        return 0

    # Both feasible -> compare Pareto front rank
    if rank_a < rank_b:
        return 1
    elif rank_b < rank_a:
        return -1
    return 0
