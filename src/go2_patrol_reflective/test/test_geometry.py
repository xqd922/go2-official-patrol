import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np

from go2_patrol_reflective.geometry import euclidean_cluster, ransac_line


def test_cluster_separates_tape_from_noise():
    points = np.array(
        [
            [0, 0], [0.05, 0.01], [0.10, 0.02], [0.15, 0.03],
            [8.0, 2.0],
        ],
        dtype=float,
    )
    labels = euclidean_cluster(points, eps=0.2, min_points=3)
    assert labels[:4].tolist() == [0, 0, 0, 0]
    assert labels[-1] == -1


def test_ransac_finds_strong_line():
    x = np.linspace(0, 2.0, 20)
    y = x * 0.5 + 0.05
    points = np.column_stack([x, y])
    line, inlier_count = ransac_line(
        points,
        min_points_for_line=6,
        inlier_distance=0.08,
        iterations=100,
    )
    assert line is not None
    assert inlier_count >= 18
