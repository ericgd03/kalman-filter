import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.path_pub = self.create_publisher(Path, '/path', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.path = Path()
        self.path.header.frame_id = 'odom'

    def odom_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)
        self.path_pub.publish(self.path)

def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
