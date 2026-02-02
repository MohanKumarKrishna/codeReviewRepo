
"""
Quantum PDE Solver
==================
Central interface that imports multiple PDE formulations,
constructs linear systems, and solves them using HHL.

Author style: modular, explicit, research-grade clarity.
"""

import numpy as np
from qiskit import Aer
from qiskit.algorithms.linear_solvers import HHL
from qiskit.algorithms.linear_solvers.matrices import NumPyMatrix
from qiskit.utils import QuantumInstance

from heat_equation import build_heat_system
from wave_equation import build_wave_system
from laplace_equation import build_laplace_system
from poisson_equation import build_poisson_system
from schrodinger_equation import build_schrodinger_system


class QuantumPDESolver:
    """
    Orchestrates PDE selection, discretization,
    normalization, and quantum linear solving.
    """

    def __init__(self):
        backend = Aer.get_backend("statevector_simulator")
        self.quantum_instance = QuantumInstance(backend)
        self.solver = HHL(quantum_instance=self.quantum_instance)

    def solve(self, A, b):
        """Solve Ax = b using HHL."""
        b = b / np.linalg.norm(b)
        matrix = NumPyMatrix(A)
        result = self.solver.solve(matrix, b)
        return result.state

    def solve_heat(self):
        A, b = build_heat_system()
        return self.solve(A, b)

    def solve_wave(self):
        A, b = build_wave_system()
        return self.solve(A, b)

    def solve_laplace(self):
        A, b = build_laplace_system()
        return self.solve(A, b)

    def solve_poisson(self):
        A, b = build_poisson_system()
        return self.solve(A, b)

    def solve_schrodinger(self):
        A, b = build_schrodinger_system()
        return self.solve(A, b)


if __name__ == "__main__":
    solver = QuantumPDESolver()

    print("Heat Equation Quantum State:")
    print(solver.solve_heat())

    print("Wave Equation Quantum State:")
    print(solver.solve_wave())
