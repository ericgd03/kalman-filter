import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math

class ControlNode(Node):
    def __init__(self):
        super().__init__('control')

        self.setpoint = None
        self.current_pose = None
        self.reached_goal = False
        self.phase = 1  # 1: orientar al punto, 2: avanzar con corrección, 3: giro final, 4: quieto

        # Parámetros de control
        self.Kp_linear = 0.5
        self.Kp_angular = 1.0
        self.linear_tolerance = 0.05
        self.angular_tolerance = 0.05

        # Confirmación de orientación estable
        self.orientation_hold_counter = 0
        self.hold_threshold = 3

        # Subscripciones
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Pose2D, '/setpoint', self.setpoint_callback, 10)

        # Publicadores
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub = self.create_publisher(Bool, '/goal_reached', 10)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Nodo de control secuencial con corrección iniciado")

    def setpoint_callback(self, msg):
        if self.setpoint is None or self.reached_goal:
            self.setpoint = msg
            self.reached_goal = False
            self.phase = 1
            self.orientation_hold_counter = 0
            # self.get_logger().info("Nuevo setpoint recibido.")

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y ** 2 + q.z ** 2)
        theta = math.atan2(siny_cosp, cosy_cosp)
        self.current_pose = Pose2D(x=pos.x, y=pos.y, theta=theta)

    def control_loop(self):
        if self.setpoint is None or self.current_pose is None:
            return

        dx = self.setpoint.x - self.current_pose.x
        dy = self.setpoint.y - self.current_pose.y
        distance = math.hypot(dx, dy)

        angle_to_target = math.atan2(dy, dx)
        error_orientation_to_target = self.normalize_angle(angle_to_target - self.current_pose.theta)
        error_to_final_theta = self.normalize_angle(self.setpoint.theta - self.current_pose.theta)

        twist = Twist()

        # FASE 1: orientar hacia el punto
        if self.phase == 1:
            twist.linear.x = 0.0
            if abs(error_orientation_to_target) > self.angular_tolerance:
                twist.angular.z = self.Kp_angular * error_orientation_to_target
                self.orientation_hold_counter = 0
            else:
                twist.angular.z = 0.0
                self.orientation_hold_counter += 1
                if self.orientation_hold_counter >= self.hold_threshold:
                    self.phase = 2
                    self.orientation_hold_counter = 0
                    # self.get_logger().info("Orientado al objetivo. Pasando a fase de avance con corrección.")

        # FASE 2: avanzar con corrección de rumbo
        elif self.phase == 2:
            if distance > self.linear_tolerance:
                twist.linear.x = self.Kp_linear * distance
                if twist.linear.x < 0.02:
                    twist.linear.x = 0.0
                twist.angular.z = self.Kp_angular * error_orientation_to_target  # Corrección activa
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.phase = 3
                # self.get_logger().info("Posición alcanzada. Pasando a giro final.")

        # FASE 3: girar a orientación final
        elif self.phase == 3:
            twist.linear.x = 0.0
            if abs(error_to_final_theta) > self.angular_tolerance:
                twist.angular.z = self.Kp_angular * error_to_final_theta
            else:
                twist.angular.z = 0.0
                self.phase = 4
                self.reached_goal = True
                self.goal_pub.publish(Bool(data=True))
                # self.get_logger().info("Setpoint final alcanzado.")

        # FASE 4: quieto
        elif self.phase == 4:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # Saturación
        twist.linear.x = max(min(twist.linear.x, 0.3), -0.3)
        twist.angular.z = max(min(twist.angular.z, 1.5), -1.5)

        # # Log
        # self.get_logger().info(
        #     f"[Fase {self.phase}] cmd_vel → linear.x: {twist.linear.x:.3f}, angular.z: {twist.angular.z:.3f}"
        # )

        # twist.linear.x = 0.5
        # twist.angular.z = 0.5

        self.cmd_vel_pub.publish(twist)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
