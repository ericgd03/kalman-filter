import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
import math

class SetpointGenerator(Node):
    def __init__(self):
        super().__init__('setpoint_generator')

        # Puntos sin orientación inicial
        base_poses = [
            (0.0, 0.0),
            (1.5, 0.0),
            (1.5, 1.0),
            (0.0, 1.0),
            (0.0, 0.0)
        ]

        # Generar orientación automáticamente hacia el siguiente
        self.poses = []
        for i in range(len(base_poses)):
            x, y = base_poses[i]
            if i < len(base_poses) - 1:
                next_x, next_y = base_poses[i+1]
                theta = math.atan2(next_y - y, next_x - x)
            else:
                theta = 0.0  # último punto, theta no importa
            self.poses.append(Pose2D(x=x, y=y, theta=theta))

        self.current_index = 0
        self.tolerance = 0.07

        self.odom = None
        self.publisher = self.create_publisher(Pose2D, '/setpoint', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.5, self.publish_next_goal)

        self.get_logger().info("SetpointGenerator iniciado: trayectoria cuadrada con orientación automática")

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2)
        theta = math.atan2(siny_cosp, cosy_cosp)
        self.odom = Pose2D(x=position.x, y=position.y, theta=theta)

    def publish_next_goal(self):
        if self.odom is None or self.current_index >= len(self.poses):
            return

        goal = self.poses[self.current_index]
        dx = goal.x - self.odom.x
        dy = goal.y - self.odom.y
        distance = math.hypot(dx, dy)

        if distance < self.tolerance:
            self.get_logger().info(f"Setpoint {self.current_index} alcanzado")
            self.current_index += 1
            if self.current_index >= len(self.poses):
                self.get_logger().info("Recorrido completo")
                return
            goal = self.poses[self.current_index]

        self.publisher.publish(goal)

def main(args=None):
    rclpy.init(args=args)
    node = SetpointGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
