#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class Controller:
    def __init__(self):
        rospy.init_node('point_follower')

        # Publisher for cmd_vel
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # Subscriber for odom
        rospy.Subscriber('odom', Odometry, self.odom_callback)

        # Initialize variables
        self.set_point_x = 4.5
        self.set_point_y = 4.5

        rospy.spin()

    def odom_callback(self, msg):
        # Callback function to handle odometry messages
        current_pose = self.get_current_pose(msg)
        self.move_to_point(current_pose)

    def move_to_point(self, current_pose):
        # Calculate the angle to the desired point
        angle_to_point = math.atan2(self.set_point_y - current_pose[1], self.set_point_x - current_pose[0])

        # Calculate the linear velocity (distance to the point)
        distance_to_point = math.sqrt((self.set_point_x - current_pose[0])**2 + (self.set_point_y - current_pose[1])**2)
        linear_velocity = 40 * distance_to_point  # Adjust the velocity gain as needed

        # Create Twist message
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_velocity
        cmd_vel_msg.angular.z = 45 * angle_to_point  # Adjust the angular velocity gain as needed

        # Publish cmd_vel
        self.cmd_vel_pub.publish(cmd_vel_msg)

    def get_current_pose(self, msg):
        # Function to extract the current pose of the robot from the odometry message
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        _, _, yaw = euler_from_quaternion([msg.pose.pose.orientation.x,
                                            msg.pose.pose.orientation.y,
                                            msg.pose.pose.orientation.z,
                                            msg.pose.pose.orientation.w])
        return current_x, current_y, yaw

if __name__ == '__main__':
    Controller()
