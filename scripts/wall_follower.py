#!/usr/bin/env python3
""" This script makes turtlebot follow a wall. """
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import numpy as np



class Follower:
    """Publishes move directives to '/cmd_vel' topic, and listens to the '/scan' topic for proximity to wall."""
    def __init__(self):
        """Initialize node, publisher, subscriber"""
        rospy.init_node('follow_person')
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self._scan)
        # Specify the callback function on ROS interrupt (CTRL+C):
        rospy.on_shutdown(self._stop_robot)
        # implement a stage variable that determines whether to handle moving robot forward or rotation.
        self.stage = 1

    def _publish_vel(self, linear_x, angular_z):
        """Publish Twist message with given velocity attributes."""
        vel_msg = Twist()
        vel_msg.linear.x = linear_x
        vel_msg.angular.z = angular_z
        self.publisher.publish(vel_msg)        
        
    def _scan(self, data):
        """Callback function for '/scan' messages received. Interpret message and move robot along face of a wall"""
        # Gather sensor data at four points: front, right, front right, back right.
        front = data.ranges[0]
        front_side = data.ranges[314]
        side = data.ranges[269]
        back_side = data.ranges[224]


        linear_x = 0
        angular_z = 0
        # Handle moving robot forward
        if self.stage == 1:
            # if no wall in vicinity, move robot straight until it finds a wall
            if front > 0.33:
                linear_x = 0.25
            # stop robot when wall is in front of it
            else:
                linear_x = 0
                self.stage = 2
        # Handle rotating robot
        elif self.stage == 2:
            # turn robot until trigonometry numbers match (see my writeup)
            buffer = 0.025
            lower_bound = side * math.sqrt(2) - buffer
            upper_bound = side * math.sqrt(2) + buffer
            if lower_bound <= front_side <= upper_bound and lower_bound <= back_side <= upper_bound:
                angular_z = 0
                self.stage = 1
            else:
                angular_z = 0.25
        
        self._publish_vel(linear_x, angular_z)
        
    
    def run(self):
        """Spin turtlebot."""
        rospy.spin()


    def _stop_robot(self):
        """Stops robot."""
        stop_msg = Twist()
        self.publisher.publish(stop_msg)


if __name__ == '__main__':
    node = Follower()
    node.run()