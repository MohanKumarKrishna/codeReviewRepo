
"""
Wave Equation (1D)
-----------------
∂²u/∂t² = c² ∂²u/∂x²

Converted to first-order system for linear solving.
"""

import numpy as np

def build_wave_system(nx=50, c=1.0, dt=0.01):
    dx = 1.0 / nx
    A = np.zeros((nx, nx))

    for i in range(1, nx-1):
        A[i, i-1] = c**2 / dx**2
        A[i, i]   = -2 * c**2 / dx**2
        A[i, i+1] = c**2 / dx**2

    A = np.eye(nx) - dt * A

    b = np.sin(np.linspace(0, 2*np.pi, nx))

    return A, b
