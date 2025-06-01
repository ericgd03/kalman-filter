import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Vector3
import tf2_ros
import numpy as np

class ArucoRelativePose(Node):

    def __init__(self):

        super().__init__('aruco_relative_pose_node')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.pose_publisher = self.create_publisher(Vector3, '/aruco_error', 10)
        self.aruco_number_publisher = self.create_publisher(Int32, '/aruco_id', 10)

        self.aruco_number = 3

    def timer_callback(self):

        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('base_link', ('aruco_' + str(self.aruco_number)), now)

            dx = trans.transform.translation.x
            dy = trans.transform.translation.y

            distance = np.sqrt(dx ** 2 + dy ** 2)  # Distance error (range)
            angle = np.arctan2(dy, dx)  # Angle error (bearing)

            max_distance = 1.0  # meters
            fov_angle = np.radians(60)  # 60° field of view (±30°)

            if distance <= max_distance and abs(angle) <= fov_angle / 2:

                # self.get_logger().info(f"Measured --> dx: {dx:.2f} | dy: {dy:.2f}")

                msg = Vector3()
                msg.x = distance
                msg.y = angle
                msg.z = 0.0 # Unused

                aruco_msg = Int32()
                aruco_msg.data = self.aruco_number

                self.pose_publisher.publish(msg)
                self.aruco_number_publisher.publish(aruco_msg)

        except Exception as e:
            self.get_logger().warn(f'No transform found: {e}')

def main(args=None):

    rclpy.init(args=args)
    node = ArucoRelativePose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()