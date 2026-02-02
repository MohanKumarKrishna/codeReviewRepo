
"""
Time-Dependent Schrödinger Equation (1D)
---------------------------------------
i ∂ψ/∂t = H ψ

Returned as a linear Hamiltonian system.
"""

import numpy as np

def build_schrodinger_system(nx=50):
    dx = 10.0 / nx
    x = np.linspace(-5, 5, nx)

    A = np.zeros((nx, nx))
    V = 0.5 * x**2

    for i in range(1, nx-1):
        A[i, i-1] = -0.5 / dx**2
        A[i, i]   = 1.0 / dx**2 + V[i]
        A[i, i+1] = -0.5 / dx**2

    b = np.exp(-x**2)

    return A, b
