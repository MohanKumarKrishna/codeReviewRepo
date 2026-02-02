"""
Quantum Variational PDE Solver
==============================
Solves discretized PDE systems Ax = b using
a Variational Quantum Linear Solver (VQLS).

No HHL is used. Suitable for NISQ-era devices.
"""

import numpy as np
from qiskit import Aer
from qiskit.circuit import QuantumCircuit
from qiskit.algorithms.optimizers import COBYLA
from qiskit.quantum_info import Statevector
from qiskit.utils import QuantumInstance

from heat_equation import build_heat_system
from wave_equation import build_wave_system
from laplace_equation import build_laplace_system
from poisson_equation import build_poisson_system
from schrodinger_equation import build_schrodinger_system


class VariationalAnsatz:
    """
    Hardware-efficient variational ansatz.
    """

    def __init__(self, num_qubits: int):
        self.num_qubits = num_qubits

    def circuit(self, parameters):
        qc = QuantumCircuit(self.num_qubits)

        idx = 0
        for q in range(self.num_qubits):
            qc.ry(parameters[idx], q)
            idx += 1

        for q in range(self.num_qubits - 1):
            qc.cx(q, q + 1)

        return qc


class QuantumVariationalPDESolver:
    """
    Quantum PDE solver using VQLS-style optimization.
    """

    def __init__(self):
        backend = Aer.get_backend("statevector_simulator")
        self.quantum_instance = QuantumInstance(backend)
        self.optimizer = COBYLA(maxiter=200)

    def _cost_function(self, params, ansatz, A, b):
        qc = ansatz.circuit(params)
        state = Statevector.from_instruction(qc).data

        Ax = A @ state
        cost = np.linalg.norm(Ax - b) ** 2
        return cost

    def _solve(self, A: np.ndarray, b: np.ndarray):
        """
        Variationally minimize ||Ax - b||²
        """
        n = int(np.log2(len(b)))
        A = A[: 2**n, : 2**n]
        b = b[: 2**n]
        b = b / np.linalg.norm(b)

        ansatz = VariationalAnsatz(n)
        initial_params = np.random.rand(n)

        result = self.optimizer.minimize(
            fun=self._cost_function,
            x0=initial_params,
            args=(ansatz, A, b),
        )

        final_circuit = ansatz.circuit(result.x)
        solution_state = Statevector.from_instruction(final_circuit)

        return solution_state


    def solve_heat_equation(self):
        A, b = build_heat_system()
        return self._solve(A, b)

    def solve_wave_equation(self):
        A, b = build_wave_system()
        return self._solve(A, b)

    def solve_laplace_equation(self):
        A, b = build_laplace_system()
        return self._solve(A, b)

    def solve_poisson_equation(self):
        A, b = build_poisson_system()
        return self._solve(A, b)

    def solve_schrodinger_equation(self):
        A, b = build_schrodinger_system()
        return self._solve(A, b)


if __name__ == "__main__":
    solver = QuantumVariationalPDESolver()

    print("Heat Equation (Quantum Variational Solution):")
    print(solver.solve_heat_equation())

    print("\nWave Equation (Quantum Variational Solution):")
    print(solver.solve_wave_equation())
