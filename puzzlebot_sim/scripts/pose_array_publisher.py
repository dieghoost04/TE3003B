#!/usr/bin/env python3  

# license removed for brevity 

import rospy 

from geometry_msgs.msg import PoseArray, Pose 

import numpy as np 

from tf.transformations import quaternion_from_euler 

 

import numpy as np 

 

class PublishPoseArray():  

    def __init__(self):  

        # first thing, init a node! 

        rospy.init_node('pose_errors')  

        self.pose_array_pub = rospy.Publisher("pose_array_topic", PoseArray, queue_size=1) 

         
        x_data_cm = np.array([9.5, 7.1, 6.4, 8.7, 8.0, 8.0, 8.5, 9.4, 7.0, 7.3, 5.4, 7.0, 6.6, 7.2, 7.0, 7.9, 7.2, 9.6, 7.1, 8.0, 7.9, 5.5, 2.3, 0.8, 5.9, 3.2, 0.0, 5.0, 3.0, 5.8, 7.8, 0.9, 8.0, 8.0, 7.9, 7.7, 8.0, 7.2, 4.0, 8.0, 7.0, 7.8, 7.9, 7.5, 5.2, 6.9, 5.9, 6.7, 7.7, 4.7])
        x_data_m = x_data_cm / 100.0

# Mediciones y en centímetros
        y_data_cm = np.array([4.0, 3.8, 3.0, 3.7, 4.0, 4.1, 3.1, 1.7, 3.0, 3.6, 4.5, 3.3, 2.7, 4.1, 4.4, 2.8, 3.2, 6.0, 3.3, 3.4, 5.3, 2.0, 12.8, 11.6, 9.4, 13.2, 16.4, 11.1, 14.1, 4.0, 4.5, 6.8, 4.6, 4.3, 3.3, 3.9, 2.0, 5.0, 2.9, 3.4, 2.7, 3.7, 5.3, 7.3, 8.7, 5.3, 10.0, 6.1, 7.2, 5.0])
        y_data_m = y_data_cm / 100.0

        theta_deg = np.array([0.0000, 6.0000, 4.7042, 5.7042, 5.7042, 5.7042, 6.7042, 6.7042, 6.7042, 6.7042, 4.7042, 4.7042, 5.7042, 5.7042, 2.7042, 2.7042, 5.7042, 4.7042, 3.7042, 5.7042, 6.7042, 6.7042, 6.7042, 11.7042, 10.7042, 9.7042, 5.7042, -2.2958, -5.2958, -3.2958, 3.7042, 4.7042, -16.2958, 9.7042, 1.7042, 6.7042, 6.7042, 0.7042, 7.7042, 7.7042, 0.7042, 4.7042, 2.7042, 0.7042, 1.7042, 0.7042, 1.7042, 1.7042, -6.2958, 7.7042])

# Convertir los ángulos de grados a radianes
        theta_rad = np.deg2rad(theta_deg)


        # Create a PoseArray message 

        pose_array_msg = PoseArray() 

 

        # Wait for a valid simulated time 

        while rospy.get_time() == 0: 

            print("no simulated time has been received yet") 

        print("Got time") 

         # Add some sample poses to the PoseArray 

        for i in range(50): 

            pose = Pose() 

            # The position will be a random number with normal distribution 

            # NOTE: this is just an example  

            # you will have to fill the pose information (x,y,theta) with the  

            # results from your different experiments.  

            pose.position.x = x_data_m[i] # mean 0 and standard deviation 0.3 

            pose.position.y = y_data_m[i] # mean 0 and standard deviation 0.1 

            pose.position.z = 0 

            #The angle will be a random number from -pi to pi 

            theta = theta_rad[i]

            quat=quaternion_from_euler(0.0, 0.0, theta) 

            pose.orientation.x = quat[0] 

            pose.orientation.y = quat[1] 

            pose.orientation.z = quat[2] 

            pose.orientation.w = quat[3] 

            pose.orientation.w = 1.0 

            pose_array_msg.poses.append(pose) 

        rate = rospy.Rate(20) # The rate of the while loop 

         

        # Set the header information (frame ID and timestamp) 

        pose_array_msg.header.frame_id = 'base_link' 

        pose_array_msg.header.stamp = rospy.Time.now() 

 

        # Publish the PoseArray 

        rate = rospy.Rate(10)  # 10 Hz 

        while not rospy.is_shutdown(): 

            self.pose_array_pub.publish(pose_array_msg) 

            rate.sleep() 

 

 

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  

    PublishPoseArray()