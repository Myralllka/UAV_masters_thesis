#!/bin/env bash

import numpy as np

import numpy as np
from itertools import product


def find_intersection_loop(Os, Ds):
    assert Os.shape == Ds.shape, "wrong shape"
    ks = Os - Ds
    distances = []
    # compute the direction vector to the closest points between all pairs
    for element in product(ks, repeat=2):
        distances.append(np.cross(element[0], element[1]))
    distances = np.array(distances).reshape(Os.shape[0], -1, Os.shape[1])

    pts = []

    for i in range(Os.shape[0]):
        for j in range(Os.shape[0]):
            if i == j:
                continue
            t1 = (np.cross(Ds[j] - Os[j], distances[i, j]) @ (Os[j] - Os[i])) / (distances[i, j] @ distances[i, j])
            t2 = (np.cross(Ds[i] - Os[i], distances[i, j]) @ (Os[j] - Os[i])) / (distances[i, j] @ distances[i, j])

            p1 = (Ds[i] - Os[i]) * t1 + Os[i]
            p2 = (Ds[j] - Os[j]) * t2 + Os[j]
            pts.append(p1)
            pts.append(p2)

    return np.mean([pts], axis=1)


def find_intersection_svd(Os: np.ndarray, Ds: np.ndarray):
    assert Os.shape == Ds.shape, "wrong shape"
    ks = Os - Ds

    A = np.zeros((3 * len(Os), 3 + len(Os)))
    b = Os.reshape(-1, 1)
    for i in range(0, len(Os) * 3 - 1, 3):
        A[i:i + 3, 0:3] = np.eye(3)
        A[i:i + 3, i // 3 + 3] = ks[i // 3]

    A = np.array(A)
    # x = np.linalg.inv(A.T @ A) @ A.T @ b
    x = np.linalg.lstsq(A, b)[0]
    return x.flatten()[:3]


if __name__ == "__main__":
    # Define camera origins and target points for each line
    Os = np.array([[2, 2, 0],
                   [9, 2, 0],
                   [9, 7, 0],
                   [1, 6, 0]])
    Ds = np.array([[3, 3, 0],
                   [3, 10, 0],
                   [3, 5, 0],
                   [8, 6, 0]])

    # Optimize the intersection
    intersection_point = find_intersection_svd(Os, Ds)

    print("Optimized Intersection Point:", intersection_point)
