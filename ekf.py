import numpy as np
from numpy.linalg import inv

# Subscribe to encoders
# Subscribe to detected arucos -> If aruco detected, use the stored position

current_position = [[0], [0], [0]]

estimated_position = current_position

covariance_matrix = [[0, 0, 0],
                     [0, 0, 0],
                     [0, 0, 0]]

estimated_covariance_matrix = covariance_matrix

landmark_position = [[3], [4]]

Q_k = [[0.5, 0.01, 0.01], # Motion model covariance matrix
       [0.01, 0.5, 0.01],
       [0.01, 0.01, 0.2]]

R_k = [[0.1, 0], # Observation model covariance matrix
       [0, 0.02]]

linear_velocity = 1 # mobile robot linar velocity m/s
angular_velocity = 1 # mobile robot angular velocity rad/s
timer_period = 0.1 # Sampling time

# Assumed measurements at each step k using lidar
measurement = {0: [[4.87], [0.8]],
               1: [[4.72], [0.72]],
               2: [[4.69], [0.65]]}

def calculate_estimated_position(current_position, timer_period, linear_velocity, angular_velocity):

    estimated_position = [[current_position[0][0] + timer_period * linear_velocity * np.cos(current_position[2][0])],
                          [current_position[1][0] + timer_period * linear_velocity * np.sin(current_position[2][0])],
                          [current_position[2][0] + timer_period * angular_velocity]]

    return estimated_position

def calculate_linearized_model(timer_period, current_position):

    H = [[1, 0, -timer_period * linear_velocity * np.sin(current_position[2][0])],
         [0, 1, timer_period * linear_velocity * np.cos(current_position[2][0])],
         [0, 0, 1]]

    return H

def calculate_estimated_covariance_matrix(H, covariance_matrix, Q_k):

    H = np.array(H)
    covariance_matrix = np.array(covariance_matrix)
    Q_k = np.array(Q_k)

    estimated_covariance_matrix = H @ covariance_matrix @ H.T + Q_k

    return estimated_covariance_matrix

def calculate_estimated_measurement(landmark_position, estimated_position): # Observation model

    delta_x = landmark_position[0][0] - estimated_position[0][0]
    delta_y = landmark_position[1][0] - estimated_position[1][0]
    p = (delta_x ** 2) + (delta_y ** 2)

    estimated_measurement = [[np.sqrt(p)],
                             [np.arctan2(delta_y, delta_x) - estimated_position[2][0]]]

    return estimated_measurement

def calculate_linearized_observation_model(landmark_position, estimated_position):

    delta_x = landmark_position[0][0] - estimated_position[0][0]
    delta_y = landmark_position[1][0] - estimated_position[1][0]
    p = (delta_x ** 2) + (delta_y ** 2)

    G = [[(-delta_x / np.sqrt(p)), (-delta_y / np.sqrt(p)), 0],
         [(delta_y / p), (-delta_x / p), -1]]

    return G

def calculate_measurement_uncertainty(G, estimated_covariance_matrix, R_k):

    G = np.array(G)
    estimated_covariance_matrix = np.array(estimated_covariance_matrix)
    R_k = np.array(R_k)

    Z = G @ estimated_covariance_matrix @ G.T + R_k

    return Z

def calculate_kalman_gain(estimated_covariance_matrix, G, Z):

    estimated_covariance_matrix = np.array(estimated_covariance_matrix)
    G = np.array(G)
    Z = np.array(Z)

    K = estimated_covariance_matrix @ G.T @ inv(Z)

    return K

def calculate_position(estimated_position, K, estimated_measurement, z_1_1):

    K = np.array(K)
    estimated_measurement = np.array(estimated_measurement)
    z_1_1 = np.array(z_1_1)

    position = estimated_position + K @ (z_1_1 - estimated_measurement)

    return position

def calculate_covariance_matrix(K, G, estimated_covariance_matrix):

    I = np.identity(3)
    K = np.array(K)
    G = np.array(G)
    estimated_covariance_matrix = np.array(estimated_covariance_matrix)

    covariance_matrix = (I - K @ G) @ estimated_covariance_matrix

    return covariance_matrix

for i in range(3):

    estimated_position = calculate_estimated_position(current_position, timer_period, linear_velocity, angular_velocity)
    print("\nEstimated position\n")
    print(estimated_position)
    H = calculate_linearized_model(timer_period, current_position)
    print("\nH\n")
    print(H)
    estimated_covariance_matrix = calculate_estimated_covariance_matrix(H, covariance_matrix, Q_k)
    print("\nEstimated covariance matriz\n")
    print(estimated_covariance_matrix)
    estimated_measurement = calculate_estimated_measurement(landmark_position, estimated_position)
    # If marker detected
    print("\nEstimated measurement\n")
    print(estimated_measurement)
    G = calculate_linearized_observation_model(landmark_position, estimated_position)
    print("\nG\n")
    print(G)
    Z = calculate_measurement_uncertainty(G, estimated_covariance_matrix, R_k)
    print("\nZ\n")
    print(Z)
    K = calculate_kalman_gain(estimated_covariance_matrix, G, Z)
    print("\nK\n")
    print(K)
    current_position = calculate_position(estimated_position, K, estimated_measurement, measurement[i])
    print("\nPosition\n")
    print(current_position)
    covariance_matrix = calculate_covariance_matrix(K, G, estimated_covariance_matrix)
    print("\nCovariance matrix\n")
    print(covariance_matrix)