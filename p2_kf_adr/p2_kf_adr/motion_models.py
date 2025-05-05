import numpy as np

def velocity_motion_model():
    def state_transition_matrix_A():
        # TODO: Define and return the 3x3 identity matrix A
        A = np.eye(3)

        return A

    def control_input_matrix_B(mu, delta_t):
        # TODO: Define B using current theta and timestep delta_t
        # B should apply linear and angular velocity to position and heading
        theta = float(mu[2, 0])
        print(theta, delta_t)
        B = np.array([
            [delta_t * np.cos(theta), 0],
            [delta_t * np.sin(theta), 0],
            [0, delta_t]
        ])

        return B

    return state_transition_matrix_A, control_input_matrix_B

def velocity_motion_model_2():
    def A(mu, dt):
        # TODO: Define and return the 6x6 constant velocity model transition matrix
        return np.array([
            [1, 0, 0, 1, 0, 0], # x = x + v_x * dt
            [0, 1, 0, 0, 1, 0], # y = y + v_y * dt
            [0, 0, 1, 0, 0, 1], # theta = theta + omega * dt
            [0, 0, 0, 1, 0, 0], # v_x = v_x
            [0, 0, 0, 0, 1, 0], # v_y = v_y
            [0, 0, 0, 0, 0, 1]  # omega = omega
        ])

    def B(mu, dt):
        # TODO: Return 6x2 zero matrix (no control input used in pure KF)
        return np.array([
            [0, 0],
            [0, 0],
            [0, 0],
            [0, 0], # No control input for Kalman filter
            [0, 0],
            [0, 0]
        ])
        

    return A, B
