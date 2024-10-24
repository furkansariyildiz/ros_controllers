#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import tf_transformations

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.publisher_ = self.create_publisher(Path, '/reference_path', 10)
        self.publish_path()

    def publish_path(self):
        path_msg = Path()
        path_msg.header.frame_id = 'map'

        # Define your path poses here
        poses = [
            {'x': 1.0, 'y': 0.0, 'theta': 0.0},
            {'x': 2.0, 'y': 0.0, 'theta': 1.57},
            {'x': 3.0, 'y': 1.0, 'theta': 1.57},
            {'x': 4.0, 'y': 1.0, 'theta': 1.57},
            {'x': 5.0, 'y': 2.0, 'theta': 1.57},
            {'x': 6.0, 'y': 3.0, 'theta': 1.57},
            {'x': 7.0, 'y': 4.0, 'theta': 1.57},
            {'x': 8.0, 'y': 5.0, 'theta': 1.57},
            {'x': 9.0, 'y': 5.0, 'theta': 1.57},
            {'x': 10.0, 'y': 5.0, 'theta': 1.57},
            {'x': 11.0, 'y': 5.0, 'theta': 1.57},
            {'x': 12.0, 'y': 5.0, 'theta': 1.57},
            # Add more waypoints as needed
        ]

        for pose in poses:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.pose.position.x = pose['x']
            pose_stamped.pose.position.y = pose['y']
            pose_stamped.pose.position.z = 0.0

            # Convert yaw to quaternion
            quat = tf_transformations.quaternion_from_euler(0, 0, pose['theta'])
            pose_stamped.pose.orientation.x = quat[0]
            pose_stamped.pose.orientation.y = quat[1]
            pose_stamped.pose.orientation.z = quat[2]
            pose_stamped.pose.orientation.w = quat[3]

            path_msg.poses.append(pose_stamped)

        self.publisher_.publish(path_msg)
        self.get_logger().info('Path published.')

def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
