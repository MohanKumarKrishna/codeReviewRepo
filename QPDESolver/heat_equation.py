
"""
Heat Equation (1D)
-----------------
∂u/∂t = α ∂²u/∂x²

Provides a discretized linear system suitable for
classical or quantum (HHL) solvers.
"""

import numpy as np

def build_heat_system(nx=50, alpha=1.0, dt=0.01):
    dx = 1.0 / nx
    A = np.zeros((nx, nx))

    for i in range(1, nx-1):
        A[i, i-1] = alpha / dx**2
        A[i, i]   = -2 * alpha / dx**2
        A[i, i+1] = alpha / dx**2

    A = np.eye(nx) - dt * A

    b = np.zeros(nx)
    b[nx//2] = 1.0  # initial heat impulse

    return A, b
