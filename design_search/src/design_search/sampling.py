"""
Stratified Latin Hypercube Sampling
Phase 3 - Dev 4 (Deliverable P4)
Test: Q-09
"""

import numpy as np

def latin_hypercube(n: int, d: int, seed: int) -> np.ndarray:
    """
    Generates an n x d Latin Hypercube Sample in [0, 1]^d.
    Each dimension has exactly one sample in each 1/n sub-interval.
    Matches Section 4.5 and Test Q-09.
    """
    rng = np.random.default_rng(seed)
    result = np.zeros((n, d), dtype=float)

    for j in range(d):
        intervals = np.arange(n)
        rng.shuffle(intervals)
        random_offsets = rng.uniform(0.0, 1.0, size=n)
        result[:, j] = (intervals + random_offsets) / n

    return result
