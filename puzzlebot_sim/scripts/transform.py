#!/usr/bin/env python3  
import rospy 

import tf2_ros #ROS package to work with transformations 
from geometry_msgs.msg import TransformStamped 
from nav_msgs.msg import Odometry
 
 
class TfBroadcaster(): 
    def __init__(self): 
        rospy.init_node('CoordinateTransform') 

        rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        print("Node broadcaster initialized!!") 
        print("use rviz to see the rotating transformation") 
        self.dt = 0.02
        r = rospy.Rate(int(1.0/self.dt)) 
        print("Node broadcaster initialized!!") 
        print("use rviz to see the rotating transformation") 
        self.message = TransformStamped()
            
        while not rospy.is_shutdown(): 
            print(self.message)   
            r.sleep() 

    def odom_callback(self, odom_msg):
        # Get the pose from the odometry message
        pose = odom_msg.pose.pose

        # Create a TransformStamped message
        tf_msg = TransformStamped()
        tf_msg.header.stamp = rospy.Time.now()
        tf_msg.header.frame_id = "base_link"
        tf_msg.child_frame_id = "chassis"
        tf_msg.transform.translation = pose.position
        tf_msg.transform.rotation = pose.orientation
        self.message = tf_msg

        # Publish the transform
        self.tf_broadcaster.sendTransform(tf_msg)

        
if __name__ == '__main__': 
    TfBroadcaster()