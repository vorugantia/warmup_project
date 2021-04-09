#!/usr/bin/env python3
""" This script makes turtlebot follow a "person" (cylinder) at a safe distance. """
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import numpy as np

class Follower:
    """Publishes move directives to '/cmd_vel' topic, and listens to the '/scan' topic for proximity to person."""
    def __init__(self):
        """Initialize node, publisher, subscriber"""
        rospy.init_node('follow_person')
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self._scan)
        # Specify the callback function on ROS interrupt (CTRL+C):
        rospy.on_shutdown(self._stop_robot)

    def _publish_vel(self, linear_x, angular_z):
        """Publish Twist message with given velocity attributes."""
        vel_msg = Twist()
        vel_msg.linear.x = linear_x
        vel_msg.angular.z = angular_z
        self.publisher.publish(vel_msg)

    def _scan(self, data):
        """Callback function for '/scan' messages received. Interpret message and move robot to follow person"""
        # Gather sensor data on person
        person_angle = np.argmin(data.ranges)
        person_dist = data.ranges[person_angle]

        # If person is not in vicinity or person is too close, stop moving
        if person_angle is None or person_dist < 0.8:
            self._publish_vel(0,0)
        else:
            # adjust angular velocity range to go from -180 to 180
            if person_angle > 180:
                person_angle -= 360
            
            # set angular velocity with proportional control
            k = 0.3
            angular_z = k * person_angle * 2 * math.pi / 360

            # only move linearly once robot is facing person
            if abs(person_angle) < 45:
                linear_x = 0.5
            else:
                linear_x = 0

            # Stop micro-adjusting angle if roughly facing person
            if abs(person_angle) < 10:
                angular_z = 0

            self._publish_vel(linear_x, angular_z)

    def run(self):
        """Continuously run turtlebot."""
        rospy.spin()

    def _stop_robot(self):
        """Callback function on ROS interrupt (CTRL+C). Stops robot."""
        stop_msg = Twist()
        self.publisher.publish(stop_msg)


if __name__ == '__main__':
    node = Follower()
    node.run()