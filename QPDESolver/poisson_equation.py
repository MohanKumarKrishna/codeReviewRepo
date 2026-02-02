
"""
Poisson Equation (2D)
--------------------
∇²u = f

Discretized using finite differences.
"""

import numpy as np

def build_poisson_system(n=10):
    size = n * n
    A = np.zeros((size, size))
    b = np.ones(size)

    def idx(i, j):
        return i*n + j

    for i in range(1, n-1):
        for j in range(1, n-1):
            k = idx(i, j)
            A[k, k] = -4
            A[k, idx(i+1, j)] = 1
            A[k, idx(i-1, j)] = 1
            A[k, idx(i, j+1)] = 1
            A[k, idx(i, j-1)] = 1

    return A, b
