import math

import numpy as np


def euclidean_cluster(points_xy, eps, min_points, max_points=10000):
    """Small DBSCAN-style 2D clustering with grid acceleration."""
    points = np.asarray(points_xy, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) == 0:
        return np.empty(0, dtype=np.int32)
    if max_points > 0 and len(points) > max_points:
        order = np.argsort(np.linalg.norm(points, axis=1))
        points = points[order[:max_points]]

    keys = np.floor(points / max(eps, 1e-6)).astype(np.int64)
    buckets = {}
    for index, key in enumerate(keys):
        buckets.setdefault(tuple(key), []).append(index)

    radius = 1  # bucket spacing is eps, so one adjacent cell in every direction suffices
    labels = np.full(len(points), -2, dtype=np.int32)
    cluster_id = 0

    def neighbors_at(point):
        key = keys[point]
        candidates = []
        for ix in range(int(key[0]) - radius, int(key[0]) + radius + 1):
            for iy in range(int(key[1]) - radius, int(key[1]) + radius + 1):
                candidates.extend(buckets.get((ix, iy), ()))
        arr = np.asarray(candidates, dtype=np.int32)
        if len(arr) == 0:
            return arr
        delta = points[arr] - points[point]
        return arr[np.linalg.norm(delta, axis=1) <= eps]

    for seed in range(len(points)):
        if labels[seed] != -2:
            continue
        nearby = neighbors_at(seed)
        if len(nearby) < min_points:
            labels[seed] = -1
            continue
        labels[seed] = cluster_id
        queue = list(nearby)
        visited = set(queue)
        while queue:
            point = queue.pop()
            if labels[point] == -1:
                labels[point] = cluster_id
            point_neighbors = neighbors_at(point)
            if len(point_neighbors) >= min_points:
                labels[point] = cluster_id
                for item in point_neighbors:
                    if labels[item] == -2 or labels[item] == -1:
                        labels[item] = cluster_id
                    if item not in visited:
                        visited.add(item)
                        queue.append(item)
        cluster_id += 1
    return labels


def ransac_line(points_xy, min_points_for_line=4, iterations=80, inlier_distance=0.08, seed=7):
    """Fit dominant 2D line and return slope/intercept/num_inliers."""
    points = np.asarray(points_xy, dtype=np.float64)
    if len(points) < 2:
        return None, 0

    rng = np.random.default_rng(seed)
    best_count = 0
    best_model = None
    for _ in range(iterations):
        i, j = rng.choice(len(points), size=2, replace=False)
        p, q = points[i], points[j]
        delta = q - p
        norm = float(np.linalg.norm(delta))
        if norm < 1e-5:
            continue
        # -y*dx + x*dy + c = 0
        a, b = -delta[1] / norm, delta[0] / norm
        c = -(a * p[0] + b * p[1])
        distances = np.abs(points @ np.array([a, b]) + c)
        count = int(np.count_nonzero(distances <= inlier_distance))
        if count > best_count:
            best_count = count
            best_model = (float(a), float(b), float(c))
    if best_model is None or best_count < min_points_for_line:
        return None, 0
    return best_model, best_count
