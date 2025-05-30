#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import numpy as np
import transforms3d


class SimpleTFPublisher(Node):
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
        self.R = 0.05       # radio de la rueda [m]
        self.L = 0.19       # distancia entre ruedas [m]

        # Covarianza inicial y varianza de ruido de ruedas
        self.P = np.zeros((3, 3))
        self.var_wheel = 0.4  # varianza del ruido en cada rueda

        # Suscripciones a velocidades de rueda
        self.create_subscription(Float64, '/wl', self.wl_callback, 10)
        self.create_subscription(Float64, '/wr', self.wr_callback, 10)

        # Publicar transforms estáticos (si aplica)
        self.publish_static_transforms()

        # Timer principal (20 Hz)
        self.timer = self.create_timer(0.05, self.timer_callback)

    def wl_callback(self, msg):
        self.wl = msg.data

    def wr_callback(self, msg):
        self.wr = msg.data

    def publish_static_transforms(self):
        # Agrega aquí cualquier transformación estática si la necesitas
        self.get_logger().info('Static TFs published')

    def timer_callback(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_time
        self.last_time = current_time

        # Calcular velocidades del cuerpo (v, w)
        v = self.R * (self.wr + self.wl) / 2.0
        w = self.R * (self.wr - self.wl) / self.L

        # --- Propagación de covarianza --
        # F (Jacobiana de transición de estado respecto al estado)
        F = np.array([
            [1.0, 0.0, -v * dt * np.sin(self.theta)],
            [0.0, 1.0,  v * dt * np.cos(self.theta)],
            [0.0, 0.0, 1.0]
        ])

        # L (Jacobiana de transición de estado respecto a la entrada de control)
        L_mat = np.array([
            [dt * np.cos(self.theta), 0.0],
            [dt * np.sin(self.theta), 0.0],
            [0.0,                    dt]
        ])
        # J (Jacobiana que mapea ruido en velocidades de rueda (wr,wl)(wr​,wl​) a ruido en (v,ω)(v,ω)
        J_w = np.array([
            [self.R/2.0,          self.R/2.0],
            [self.R/self.L,      -self.R/self.L]
        ])
        Sigma_wheels = np.diag([self.var_wheel, self.var_wheel]) # Covarianza del ruido en cada rueda
        Sigma_u = J_w @ Sigma_wheels @ J_w.T # Covarianza en el espacio de control (v,ω)(v,ω)
        self.P = F @ self.P @ F.T + L_mat @ Sigma_u @ L_mat.T # Estima la incertidumbre del robot
        # --- Fin propagación de covarianza ---

        # Actualizar estado
        self.x += v * np.cos(self.theta) * dt
        self.y += v * np.sin(self.theta) * dt
        self.theta += w * dt

        # Convertir a cuaternión
        quat = transforms3d.euler.euler2quat(0, 0, self.theta)

        # Publicar TF dinámico: odom → base_footprint
        td = TransformStamped()
        td.header.stamp = self.get_clock().now().to_msg()
        td.header.frame_id = 'odom'
        td.child_frame_id = 'base_footprint'
        td.transform.translation.x = self.x
        td.transform.translation.y = self.y
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
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = td.transform.rotation
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        # Matriz 6×6 de covarianza: solo XY y yaw (rotZ)
        # Se aplana esta matriz dejando solo los valores correspondientes 
        # a XY y yaw para una representación en rviz en 2D
        cov6 = np.zeros((6, 6))
        # Posición XY
        cov6[0, 0] = self.P[0, 0]
        cov6[0, 1] = self.P[0, 1]
        cov6[1, 0] = self.P[1, 0]
        cov6[1, 1] = self.P[1, 1]
        # Yaw (rotación Z)
        cov6[5, 5] = self.P[2, 2]
        odom.pose.covariance = cov6.flatten().tolist()

        self.odom_publisher.publish(odom)

        # Animación de ruedas (aproximada)
        wheel_rotation = v * current_time / self.R
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
    node = SimpleTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()