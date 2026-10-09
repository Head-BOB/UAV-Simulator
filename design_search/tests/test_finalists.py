"""
Knee Point Unit Test
Matches Section 7.3 Worked Example
"""

import sys
import os
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src")))
from design_search.finalists import calculate_knee_point

def test_knee_point_hand_example():
    # Points: (0, 1), (0.2, 0.35), (0.5, 0.15), (1, 0)
    pts = np.array([
        [0.0, 1.0],
        [0.2, 0.35],
        [0.5, 0.15],
        [1.0, 0.0]
    ])
    # Distances are: 0.0, 0.3182, 0.2475, 0.0
    # Knee is point at index 1: (0.2, 0.35)
    knee_idx = calculate_knee_point(pts)
    assert knee_idx == 1
