"""
NSGA-II Multi-Objective Genetic Algorithm Engine
Phase 3 - Dev 4 (Deliverable P5 & P6)
"""

import numpy as np
from typing import List, Tuple, Callable, Dict, Any
from design_search.pareto import fast_nondominated_sort, crowding_distance
from design_search.operators import (
    simulated_binary_crossover,
    polynomial_mutation,
    constraint_dominates
)

class NSGA2Optimizer:
    def __init__(
        self,
        eval_fn: Callable[[np.ndarray], Tuple[np.ndarray, float]],
        bounds: List[Tuple[float, float]],
        pop_size: int = 100,
        n_generations: int = 200,
        crossover_prob: float = 0.9,
        mutation_prob: float = 0.1,
        eta_c: float = 20.0,
        eta_m: float = 20.0,
        seed: int = 42
    ):
        self.eval_fn = eval_fn
        self.bounds = bounds
        self.n_vars = len(bounds)
        self.pop_size = pop_size
        self.n_generations = n_generations
        self.crossover_prob = crossover_prob
        self.mutation_prob = mutation_prob
        self.eta_c = eta_c
        self.eta_m = eta_m
        self.rng = np.random.default_rng(seed)

    def optimize(self) -> Dict[str, Any]:
        """
        Runs the full NSGA-II loop and returns the final population, objectives, and archive.
        """
        # 1. Initialize random population within bounds
        pop = np.zeros((self.pop_size, self.n_vars))
        for i in range(self.n_vars):
            lb, ub = self.bounds[i]
            pop[:, i] = self.rng.uniform(lb, ub, size=self.pop_size)

        # Evaluate Generation 0
        objs = []
        viols = []
        for ind in pop:
            f, v = self.eval_fn(ind)
            objs.append(f)
            viols.append(v)
        objs = np.array(objs)
        viols = np.array(viols)

        # Evolution loop
        for gen in range(self.n_generations):
            # Sort population into fronts
            fronts = fast_nondominated_sort(objs)

            # Assign front ranks
            ranks = np.zeros(len(pop), dtype=int)
            for r, front in enumerate(fronts):
                for idx in front:
                    ranks[idx] = r

            # Assign crowding distance per front
            crowd = np.zeros(len(pop), dtype=float)
            for front in fronts:
                if len(front) > 0:
                    dists = crowding_distance(objs[front])
                    for idx, d in zip(front, dists):
                        crowd[idx] = d

            # 2. Tournament Selection & Offspring Generation
            offspring = []
            while len(offspring) < self.pop_size:
                # Binary tournament selection
                p1_idx = self._tournament(ranks, viols, crowd)
                p2_idx = self._tournament(ranks, viols, crowd)

                if self.rng.uniform(0.0, 1.0) < self.crossover_prob:
                    c1, c2 = simulated_binary_crossover(
                        pop[p1_idx], pop[p2_idx], self.bounds, self.eta_c, self.rng
                    )
                else:
                    c1, c2 = np.copy(pop[p1_idx]), np.copy(pop[p2_idx])

                c1 = polynomial_mutation(c1, self.bounds, self.eta_m, self.mutation_prob, self.rng)
                c2 = polynomial_mutation(c2, self.bounds, self.eta_m, self.mutation_prob, self.rng)

                offspring.append(c1)
                if len(offspring) < self.pop_size:
                    offspring.append(c2)

            offspring = np.array(offspring)

            # Evaluate Offspring
            off_objs = []
            off_viols = []
            for ind in offspring:
                f, v = self.eval_fn(ind)
                off_objs.append(f)
                off_viols.append(v)
            off_objs = np.array(off_objs)
            off_viols = np.array(off_viols)

            # 3. Combine Parent + Child (2N) and Select Best N
            comb_pop = np.vstack([pop, offspring])
            comb_objs = np.vstack([objs, off_objs])
            comb_viols = np.concatenate([viols, off_viols])

            comb_fronts = fast_nondominated_sort(comb_objs)

            new_pop_indices = []
            for front in comb_fronts:
                if len(new_pop_indices) + len(front) <= self.pop_size:
                    new_pop_indices.extend(front)
                else:
                    needed = self.pop_size - len(new_pop_indices)
                    dists = crowding_distance(comb_objs[front])
                    sorted_order = np.argsort(dists)[::-1] # Highest crowding distance first
                    for i in range(needed):
                        new_pop_indices.append(front[sorted_order[i]])
                    break

            pop = comb_pop[new_pop_indices]
            objs = comb_objs[new_pop_indices]
            viols = comb_viols[new_pop_indices]

        # Extract final Pareto front (Rank 0)
        final_fronts = fast_nondominated_sort(objs)
        pareto_indices = final_fronts[0]

        return {
            "pareto_pop": pop[pareto_indices],
            "pareto_objs": objs[pareto_indices],
            "all_pop": pop,
            "all_objs": objs,
            "all_viols": viols
        }

    def _tournament(self, ranks: np.ndarray, viols: np.ndarray, crowd: np.ndarray) -> int:
        idx1 = self.rng.integers(0, len(ranks))
        idx2 = self.rng.integers(0, len(ranks))
        cmp = constraint_dominates(ranks[idx1], viols[idx1], ranks[idx2], viols[idx2])
        if cmp == 1:
            return idx1
        elif cmp == -1:
            return idx2
        else:
            # Tied on constraint domination -> pick larger crowding distance
            return idx1 if crowd[idx1] >= crowd[idx2] else idx2
