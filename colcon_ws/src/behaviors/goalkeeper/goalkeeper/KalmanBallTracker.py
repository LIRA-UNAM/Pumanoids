import numpy as np


class KalmanBallTracker:
    def __init__(self, dt=0.033):
        self.dt = dt

        # Estado: [x, y, vx, vy]
        self.x = np.array([
            [0.0],
            [0.0],
            [0.0],
            [0.0]
        ])

        # Matriz de transición de estado
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], dtype=float)

        # Matriz de observación
        # Solo medimos x, y
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=float)

        # Incertidumbre inicial
        self.P = np.eye(4) * 1000.0

        # Ruido de medición
        self.R = np.array([
            [5, 0],
            [0, 5]
        ], dtype=float)

        # Ruido del modelo
        self.Q = np.eye(4) * 0.1

        # Matriz identidad
        self.I = np.eye(4)

        self.initialized = False

    def initialize(self, measured_x, measured_y):
        """
        Inicializa el filtro con la primera medición real.
        """
        self.x = np.array([
            [measured_x],
            [measured_y],
            [0.0],
            [0.0]
        ], dtype=float)

        self.initialized = True

    def predict(self):
        """
        Predicción:
        x = F x
        P = F P F^T + Q
        """
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

        return self.x

    def update(self, measured_x, measured_y):
        """
        Corrección con medición z = [x, y]
        """
        z = np.array([
            [measured_x],
            [measured_y]
        ], dtype=float)

        # Innovación
        y = z - self.H @ self.x

        # Covarianza de la innovación
        S = self.H @ self.P @ self.H.T + self.R

        # Ganancia de Kalman
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # Actualización del estado
        self.x = self.x + K @ y

        # Actualización de la incertidumbre
        self.P = (self.I - K @ self.H) @ self.P

        return self.x

    def step(self, measured_x, measured_y):
        """
        Hace predict + update.
        """
        if not self.initialized:
            self.initialize(measured_x, measured_y)
            return self.get_state()

        self.predict()
        self.update(measured_x, measured_y)

        return self.get_state()

    def get_state(self):
        """
        Regresa x, y, vx, vy como valores escalares.
        """
        return {
            "x": float(self.x[0, 0]),
            "y": float(self.x[1, 0]),
            "vx": float(self.x[2, 0]),
            "vy": float(self.x[3, 0])
        }