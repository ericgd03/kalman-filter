#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import numpy as np
from numpy.linalg import inv
import transforms3d

class ExtendedKalmanFilter(Node):

    def __init__(self):

        super().__init__('simple_tf_publisher')

        # Broadcasters y publisher de odometría
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.dynamic_broadcaster = TransformBroadcaster(self)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)

        # Estado inicial
        self.wl = 0.0
        self.wr = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now().nanoseconds / 1e9

        # Parámetros del robot
        self.R = 0.05  # radio de la rueda [m]
        self.L = 0.19  # distancia entre ruedas [m]

        # Suscripciones a velocidades de rueda
        self.create_subscription(Float64, '/wl', self.wl_callback, 10)
        self.create_subscription(Float64, '/wr', self.wr_callback, 10)
        self.create_subscription(Vector3, '/aruco_relative_pos', self.aruco_callback, 10)

        # Publicar transforms estáticos (si aplica)
        self.publish_static_transforms()

        # Timer principal (20 Hz)
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.create_timer(0.01, self.check_aruco_timeout)  # Called every 0.1 seconds

        # Extended Kalman Filter variables
        self.current_position = np.zeros((3, 1))
        self.estimated_position = self.current_position

        self.H = np.zeros((3, 3))

        self.covariance_matrix = np.zeros((3, 3))
        self.estimated_covariance_matrix = self.covariance_matrix

        self.estimated_measurement = np.zeros((2, 1))
        self.measurement = np.zeros((2, 1))

        self.landmark_position = [[-0.5], [1]] # Waypoints
        # self.landmark_position = [[-1.5], [1.5]] # Circle

        self.Q_k = np.zeros((3, 3)) # Motion model covariance matrix

        self.R_k = [[0.1, 0], # Observation model covariance matrix
                    [0, 0.02]]

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        self.aruco = False
        self.last_aruco_time = self.get_clock().now()

    def wl_callback(self, msg):
        self.wl = msg.data

    def wr_callback(self, msg):
        self.wr = msg.data

    def publish_static_transforms(self):
        # Agrega aquí cualquier transformación estática si la necesitas
        self.get_logger().info('Static TFs published')

    def aruco_callback(self, msg):

        self.last_aruco_time = self.get_clock().now()

        self.measurement[0][0] = msg.x
        self.measurement[1][0] = msg.y

        self.aruco = True

        # self.get_logger().info(f'Measured -> Distance: {self.measurement[0][0]:.2f} m | Angle: {self.measurement[1][0]:.2f} rad')
        # self.get_logger().info(f'Estimated -> Distance: {self.estimated_measurement[0][0]:.2f} m | Angle: {self.estimated_measurement[1][0]:.2f} rad')

    def check_aruco_timeout(self):

        # If time since last ArUco > 1 second, mark it as not visible
        now = self.get_clock().now()
        if (now - self.last_aruco_time) > Duration(seconds=self.timer_period):
            self.aruco = False
        # pass

    def normalize_angle(self, angle):
        
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def calculate_estimated_position(self):

        self.estimated_position = [[self.current_position[0][0] + self.timer_period * self.linear_velocity * np.cos(self.current_position[2][0])],
                                   [self.current_position[1][0] + self.timer_period * self.linear_velocity * np.sin(self.current_position[2][0])],
                                   [self.normalize_angle(self.current_position[2][0] + self.timer_period * self.angular_velocity)]]

    def calculate_linearized_model(self):

        self.H = [[1, 0, -self.timer_period * self.linear_velocity * np.sin(self.current_position[2][0])],
                  [0, 1, self.timer_period * self.linear_velocity * np.cos(self.current_position[2][0])],
                  [0, 0, 1]]

    def calculate_Qk(self):

        kr = 0.5
        kl = 0.5

        sigma = [[kr * abs(self.wr), 0],
                 [0, kl * abs(self.wl)]]

        sigma = np.array(sigma)

        grad_h = ((1/2) * self.R * self.timer_period) * np.array([[np.cos(self.current_position[2][0]), np.cos(self.current_position[2][0])], 
                                                                  [np.sin(self.current_position[2][0]), np.sin(self.current_position[2][0])], 
                                                                  [(2/self.L), -(2/self.L)]])

        grad_h = np.array(grad_h)
        
        self.Q_k = grad_h @ sigma @ grad_h.T

    def calculate_estimated_covariance_matrix(self):

        self.H = np.array(self.H)
        self.covariance_matrix = np.array(self.covariance_matrix)
        self.Q_k = np.array(self.Q_k)

        self.estimated_covariance_matrix = self.H @ self.covariance_matrix @ self.H.T + self.Q_k

    def calculate_estimated_measurement(self): # Observation model

        delta_x = self.landmark_position[0][0] - self.estimated_position[0][0]
        delta_y = self.landmark_position[1][0] - self.estimated_position[1][0]
        p = (delta_x ** 2) + (delta_y ** 2)

        # self.get_logger().info(f"Estimated --> dx: {delta_x:.2f} | dy: {delta_y:.2f}")

        self.estimated_measurement = [[np.sqrt(p)],
                                      [np.arctan2(delta_y, delta_x) - self.estimated_position[2][0]]]

    def calculate_linearized_observation_model(self):

        delta_x = self.landmark_position[0][0] - self.estimated_position[0][0]
        delta_y = self.landmark_position[1][0] - self.estimated_position[1][0]
        p = (delta_x ** 2) + (delta_y ** 2)

        self.G = [[(-delta_x / np.sqrt(p)), (-delta_y / np.sqrt(p)), 0],
                  [(delta_y / p), (-delta_x / p), -1]]

    def calculate_measurement_uncertainty(self):

        self.G = np.array(self.G)
        self.estimated_covariance_matrix = np.array(self.estimated_covariance_matrix)
        self.R_k = np.array(self.R_k)

        self.Z = self.G @ self.estimated_covariance_matrix @ self.G.T + self.R_k

    def calculate_kalman_gain(self):

        self.estimated_covariance_matrix = np.array(self.estimated_covariance_matrix)
        self.G = np.array(self.G)
        self.Z = np.array(self.Z)

        self.K = self.estimated_covariance_matrix @ self.G.T @ inv(self.Z)

    def calculate_position(self):

        self.K = np.array(self.K)
        self.estimated_measurement = np.array(self.estimated_measurement)
        self.measurement = np.array(self.measurement)

        self.current_position = self.estimated_position + self.K @ (self.measurement - self.estimated_measurement)

    def calculate_covariance_matrix(self):

        I = np.identity(3)
        self.K = np.array(self.K)
        self.G = np.array(self.G)
        self.estimated_covariance_matrix = np.array(self.estimated_covariance_matrix)

        self.covariance_matrix = (I - self.K @ self.G) @ self.estimated_covariance_matrix

    def ekf_loop(self):

        self.calculate_estimated_position()
        self.calculate_linearized_model()
        self.calculate_Qk()
        self.calculate_estimated_covariance_matrix()

        if self.aruco:

            self.calculate_estimated_measurement()
            self.calculate_linearized_observation_model()
            self.calculate_measurement_uncertainty()
            self.calculate_kalman_gain()
            self.calculate_position()
            self.calculate_covariance_matrix()

        else:

            self.current_position = self.estimated_position
            self.covariance_matrix = self.estimated_covariance_matrix 

    def timer_callback(self):

        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_time
        self.last_time = current_time

        # Calcular velocidades del cuerpo (v, w)
        self.linear_velocity = self.R * (self.wr + self.wl) / 2.0
        self.angular_velocity = self.R * (self.wr - self.wl) / self.L

        # Convertir a cuaternión
        quat = transforms3d.euler.euler2quat(0, 0, self.current_position[2][0])

        self.ekf_loop() # Kalman filter

        # Publicar TF dinámico: odom → base_footprint
        td = TransformStamped()
        td.header.stamp = self.get_clock().now().to_msg()
        td.header.frame_id = 'odom'
        td.child_frame_id = 'base_footprint'
        td.transform.translation.x = self.current_position[0][0]
        td.transform.translation.y = self.current_position[1][0]
        td.transform.translation.z = 0.0
        td.transform.rotation.x = quat[1]
        td.transform.rotation.y = quat[2]
        td.transform.rotation.z = quat[3]
        td.transform.rotation.w = quat[0]
        self.dynamic_broadcaster.sendTransform(td)

        # Publicar mensaje de Odom con covarianza 2D (xy + yaw)
        odom = Odometry()
        odom.header.stamp = td.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.current_position[0][0]
        odom.pose.pose.position.y = self.current_position[1][0]
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = td.transform.rotation
        odom.twist.twist.linear.x = self.linear_velocity
        odom.twist.twist.angular.z = self.angular_velocity

        # Matriz 6×6 de covarianza: solo XY y yaw (rotZ)
        # Se aplana esta matriz dejando solo los valores correspondientes 
        # a XY y yaw para una representación en rviz en 2D
        cov6 = np.zeros((6, 6))
        # Posición XY
        cov6[0, 0] = self.covariance_matrix[0, 0]
        cov6[0, 1] = self.covariance_matrix[0, 1]
        cov6[1, 0] = self.covariance_matrix[1, 0]
        cov6[1, 1] = self.covariance_matrix[1, 1]
        # Yaw (rotación Z)
        cov6[5, 5] = self.covariance_matrix[2, 2]
        odom.pose.covariance = cov6.flatten().tolist()

        self.odom_publisher.publish(odom)

        # Animación de ruedas (aproximada)
        wheel_rotation = self.linear_velocity * current_time / self.R
        quat_wheel = transforms3d.euler.euler2quat(0, wheel_rotation, 0)

        for link, pos in [('wheel_r_link', (0.052, -0.095, -0.0025)),
                          ('wheel_l_link', (0.052,  0.095, -0.0025))]:
            td_w = TransformStamped()
            td_w.header.stamp = td.header.stamp
            td_w.header.frame_id = 'base_link'
            td_w.child_frame_id = link
            td_w.transform.translation.x, td_w.transform.translation.y, td_w.transform.translation.z = pos
            td_w.transform.rotation.x = quat_wheel[1]
            td_w.transform.rotation.y = quat_wheel[2]
            td_w.transform.rotation.z = quat_wheel[3]
            td_w.transform.rotation.w = quat_wheel[0]
            self.dynamic_broadcaster.sendTransform(td_w)

def main(args=None):

    rclpy.init(args=args)
    node = ExtendedKalmanFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()