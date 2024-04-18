#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped 
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


# Declare the output Messages

class joint_wheels():
    def __init__(self):
        rospy.init_node('wheel_joints') 

        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=1)



        self.theta = 0
        self.dt = 0.02
        r = rospy.Rate(int(1.0/self.dt)) 
        
    
        self.message = TransformStamped()

        #q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0.0)
        self.contJoints = JointState()

        self.init_joints()

        while not rospy.is_shutdown(): 
            self.contJoints.header.stamp = rospy.Time.now()
            self.contJoints.position[0] = self.wrap_to_Pi(self.theta)
            self.contJoints.position[1] = self.wrap_to_Pi(self.theta)
           
            
            self.joint_pub.publish(self.contJoints)

            r.sleep()
        


        # Declare the output Messages
    def init_joints(self):
        self.contJoints.header.frame_id = "chassis"
        self.contJoints.header.stamp = rospy.Time.now()
        self.contJoints.name.extend(["joint_base_to_wl", "joint_base_to_wr"])
        self.contJoints.position.extend([0.0, 0.0])
        self.contJoints.velocity.extend([0.0, 0.0])
       
    

    #wrap to pi function
    def wrap_to_Pi(self,theta):
        result = np.fmod((theta + np.pi),(2 * np.pi))
        if(result < 0):
            result += 2 * np.pi
        return result - np.pi
    
    def odom_callback(self, odom_msg):
        # Get the pose from the odometry message
        pose = odom_msg.pose.pose

        self.theta = 0
        pose = odom_msg.pose.pose

        # Extract orientation quaternion
        orientation = pose.orientation

        # Convert quaternion to Euler angles
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        # Roll (x), pitch (y), and yaw (z) angles
        roll, pitch, yaw = euler_from_quaternion([x, y, z, w])

        # Extract the yaw angle (rotation around the Z-axis)
        self.theta = yaw

if __name__ == '__main__': 
    joint_wheels()

        





