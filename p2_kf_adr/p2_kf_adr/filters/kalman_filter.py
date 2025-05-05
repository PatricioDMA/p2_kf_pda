import numpy as np 

from ..motion_models import velocity_motion_model, velocity_motion_model_2
from ..observation_models import odometry_observation_model, odometry_observation_model_2

class KalmanFilter:

    def __init__(self, initial_state, initial_covariance, proc_noise_std = [0.02, 0.02, 0.01], obs_noise_std = [0.02, 0.02, 0.01]): # Ruido bajo
    # def __init__(self, initial_state, initial_covariance, proc_noise_std = [0.02, 0.02, 0.01], obs_noise_std = [0.5, 0.5, 0.2]): # Ruido alto en la medición (Q grande)
    # def __init__(self, initial_state, initial_covariance, proc_noise_std = [0.5, 0.5, 0.2], obs_noise_std = [0.02, 0.02, 0.01]): # Ruido alto en el proceso (R grande)
        self.mu = initial_state # Initial state estimate [x, y, theta]
        self.Sigma = initial_covariance # Initial uncertainty

        self.A_func, self.B_func = velocity_motion_model() # The action model to use. Returns A and B matrices

        # Standard deviations for the noise in x, y, and theta (process or action model noise)
        self.proc_noise_std = np.array(proc_noise_std)
        # Process noise covariance (R)
        self.R = np.diag(self.proc_noise_std ** 2)  # process noise covariance

        # Observation model (C)
        self.C = odometry_observation_model() # The observation model to use

        # Standard deviations for the noise in x, y, theta (observation or sensor model noise)
        self.obs_noise_std = np.array(obs_noise_std)
        # Observation noise covariance (Q)
        self.Q = np.diag(self.obs_noise_std ** 2)
            
    def predict(self, u, dt):
        # TODO: Implement Kalman filter prediction step
        # Predict the new mean (mu) using A, B, and control input u
        # Predict the new covariance (Sigma) using A and R
        A = self.A_func()  # identidad
        B = self.B_func(self.mu, dt)  # usa mu y dt para generar B
        #u = np.reshape(u, (-1, 1))  # Asegura forma (2,1)

        self.mu = A @ self.mu + B @ u # Predicción del estado (3x1)
        self.Sigma = A @ self.Sigma @ A.T + self.R  # Predicción de la covarianza (3x3)

    def update(self, z):
        # TODO: Implement Kalman filter correction step
        # Compute Kalman gain K
        # Update the mean (mu) with the measurement z
        # Update the covariance (Sigma)
        K = self.Sigma @ self.C.T @ np.linalg.inv(self.C @ self.Sigma @ self.C.T + self.Q)  # Ganancia de Kalman (3x3)
        self.mu = self.mu + K @ (z - self.C @ self.mu)  # Corrección del estado (3x1)
        self.Sigma = (np.eye(len(self.Sigma)) - K @ self.C) @ self.Sigma  # Corrección de la covarianza (3x3)

class KalmanFilter_2:
    def __init__(self, initial_state, initial_covariance,
                 proc_noise_std=[0.02]*6, obs_noise_std=[0.02]*6): # Ruido bajo
    # def __init__(self, initial_state, initial_covariance,
    #              proc_noise_std=[0.02]*6, obs_noise_std=[0.5, 0.5, 0.2, 0.5, 0.5, 0.2]): # Ruido alto en la medición (Q grande)
    # def __init__(self, initial_state, initial_covariance,
    #              proc_noise_std=[0.5, 0.5, 0.2, 0.5, 0.5, 0.2], obs_noise_std=[0.02]*6): # Ruido alto en el proceso (R grande)

        self.mu = initial_state  # Initial state estimate [x, y, theta, vx, vy, omega]
        self.Sigma = initial_covariance  # Initial uncertainty

        self.A_func, self.B_func = velocity_motion_model_2()  # Motion model matrices

        self.proc_noise_std = np.array(proc_noise_std)
        self.R = np.diag(self.proc_noise_std ** 2)  # Process noise covariance

        self.C = odometry_observation_model_2()  # Observation matrix
        self.obs_noise_std = np.array(obs_noise_std)
        self.Q = np.diag(self.obs_noise_std ** 2)  # Observation noise covariance

    def predict(self, u=None, dt=1.0):
        # TODO: Implement Kalman prediction step for full state (6D)
        # Pure KF: use only the A matrix to update the state and covariance
        # 2 líneas (2 ecuaciones)
        A = self.A_func(self.mu, dt)
        B = self.B_func(self.mu, dt)
        # u = self.mu[3:6]  # [vx, vy, omega]

        self.mu = A @ self.mu #+ B @ u # Predicción del estado (6x1)
        self.Sigma = A @ self.Sigma @ A.T + self.R  # Predicción de la covarianza (6x6)
        return self.mu, self.Sigma

    def update(self, z):
        # TODO: Implement update step
        # Compute Kalman gain
        # Correct the predicted state with measurement
        # Update covariance
        # 3 líneas (3 ecuaciones)
        K = self.Sigma @ self.C.T @ np.linalg.inv(self.C @ self.Sigma @ self.C.T + self.Q)  # Ganancia de Kalman (6x6)
        self.mu = self.mu + K @ (z - self.C @ self.mu)  # Corrección del estado (6x1)
        self.Sigma = (np.eye(len(self.Sigma)) - K @ self.C) @ self.Sigma  # Corrección de la covarianza (6x6)
        return self.mu, self.Sigma
