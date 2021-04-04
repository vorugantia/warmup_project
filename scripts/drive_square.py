#!/usr/bin/env python3
""" This script makes turtlebot drive along a square path. """
import rospy
from geometry_msgs.msg import Twist
import math


class DriveInSquare(object):
    """This class defines node 'drive_in_square' that publishes on `cmd_vel` topic."""
    def __init__(self):
        rospy.init_node('drive_in_square')
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # Specify the callback function on ROS interrupt (CTRL+C):
        rospy.on_shutdown(self._stop_robot)

    def run(self):
        """Update robot's trajectory such that it moves along a square path."""
        # Define message and rate
        vel_msg = Twist()
        r = rospy.Rate(2)
        
        rospy.sleep(1)
        while not rospy.is_shutdown():
            # Init to drive forward directive, publish it for 5s
            vel_msg.linear.x = 0.1
            vel_msg.angular.z = 0
            for _ in range(10): # 10 / 2Hz = 5s
                self.publisher.publish(vel_msg)
                r.sleep()

            self._pause(r) # Pause 1 second

            # Update vel_msg to turn 90 degrees. Want to turn slowly (5s) to prevent drift.
            vel_msg.linear.x = 0
            vel_msg.angular.z = math.pi / 10  # 18deg/s
            for _ in range(10): # 10 / 2Hz = 5s
                self.publisher.publish(vel_msg)
                r.sleep()

            self._pause(r) # Pause 1 second

    def _pause(self, r, seconds=1):
        """Pause robot trajectory for given number of seconds. To be called between different directives."""
        for _ in range(math.ceil(2*seconds)):
            self.publisher.publish(Twist())
            r.sleep()
    
    def _stop_robot(self):
        """Callback function on ROS interrupt (CTRL+C). Stops robot."""
        stop_msg = Twist()
        self.publisher.publish(stop_msg)


if __name__ == '__main__':
    node = DriveInSquare()
    node.run()