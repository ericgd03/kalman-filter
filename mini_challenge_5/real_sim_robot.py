import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class RealSimRobot(Node):
    def __init__(self):
        super().__init__('real_sim_robot')

        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.19)

        self.R = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.L = self.get_parameter('wheel_base').get_parameter_value().double_value

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.pub_wr = self.create_publisher(Float64, '/wr', 10)
        self.pub_wl = self.create_publisher(Float64, '/wl', 10)

        self.get_logger().info('Real Sim Robot Node ready. Awaiting /cmd_vel...')

    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z

        wl = (2 * v - w * self.L) / (2 * self.R)
        wr = (2 * v + w * self.L) / (2 * self.R)

        self.pub_wl.publish(Float64(data=wl))
        self.pub_wr.publish(Float64(data=wr))

def main(args=None):
    rclpy.init(args=args)
    node = RealSimRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
