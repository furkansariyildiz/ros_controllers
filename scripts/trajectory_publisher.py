#!/usr/bin/env python
import rospy
import math
from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint


class TrajectoryPublisher:
    def __init__(self):
        self.trajectory_publisher_ = rospy.Publisher('/planning/trajectory', Trajectory, queue_size=1)
        self.rate_ = rospy.Rate(10)

        self.trajectory_ = Trajectory()
        

    def prepare_trajectory(self):
        amplitude = 2.0
        frequency = 0.2
        number_of_points = 100

        for i in range(number_of_points):
            point = TrajectoryPoint()
            point.pose.position.x = i * 0.5
            point.pose.position.y = amplitude * math.sin(frequency * point.pose.position.x)
            point.pose.position.z = 0.0

            point.pose.orientation.x = 0.0
            point.pose.orientation.y = 0.0
            point.pose.orientation.z = 0.0
            point.pose.orientation.w = 1.0

            self.trajectory_.points.append(point)


    def publish_trajectory(self):
        self.prepare_trajectory()
        while not rospy.is_shutdown():
            self.trajectory_publisher_.publish(self.trajectory_)
            self.rate_.sleep()


if __name__ == '__main__':
    rospy.init_node('path_publisher', disable_signals=True)
    trajectory_publisher = TrajectoryPublisher()
    trajectory_publisher.publish_trajectory()
