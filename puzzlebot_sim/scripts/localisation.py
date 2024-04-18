#!/usr/bin/env python3 

import rospy  

from geometry_msgs.msg import Twist  

from geometry_msgs.msg import PoseStamped 

from std_msgs.msg import Float32 

from nav_msgs.msg import Odometry

import tf2_ros #ROS package to work with transformations 
from geometry_msgs.msg import TransformStamped 
from tf.transformations import quaternion_from_euler 

import numpy as np 



#This class will do the following: 

#   subscribe to the /wr and /wl topic  

#   publish to /odom   

class OdomClass():  

    def __init__(self):  

        # first thing, init a node! 

        rospy.init_node('localisation') 

        ###******* INIT PUBLISHERS *******###  

        # Create the subscriber to wr and wl topics

        rospy.Subscriber("wl", Float32, self.wl_cb) 
        rospy.Subscriber("wr", Float32, self.wr_cb) 

        

        # Create ROS publishers 

        self.odom_pub = rospy.Publisher('odom', Odometry ,queue_size=1) #Publisher to pose_sim topic 

        
      
         

        ############ ROBOT CONSTANTS ################  

        self.r=0.05 #puzzlebot wheel radius [m] 

        self.L = 0.19 #puzzlebot wheel separation [m] 

        self.dt = 0.02 # Desired time to update the robot's pose [s] 

        ############ Variables ############### 

        self.w = 0.0 # robot's angular speed [rad/s] 

        self.v = 0.0 #robot's linear speed [m/s] 

        self.x = 0.0 # 

        self.y = 0.0  

        self.theta = 0.0  

        self.wr = 0.0

        self.wl = 0.0

 

        self.odom = Odometry() 

                

        rate = rospy.Rate(int(1.0/self.dt)) # The rate of the while loop will be the inverse of the desired delta_t 

        while not rospy.is_shutdown(): 

            ####### Add / modify  your code here ############# 

            [self.v, self.w] = self.get_robot_vel(self.wr,self.wl)

            self.update_robot_pose(self.v, self.w) 

            odom_msg = self.get_odom_stamped(self.x, self.y, self.theta) 

            
            print(odom_msg)

            ######################################### 

 

            ######## Publish the data ################# 
    
            self.odom_pub.publish(odom_msg) 

          

            rate.sleep() 

     

    def wl_cb(self, msg): 

        self.wl = msg.data

    def wr_cb(self, msg): 

        self.wr = msg.data

       
         

    def get_robot_vel(self,wr,wl): 


       #MODIFICAR

        v = ((wl + wr)/2) # Left wheel angular speed in [rad/s] 

        #w = ((wr*2*self.r)-2*v)/self.L  # Right wheel angular speed in [rad/s] 

        w = ((wr - wl) / self.L)*self.r

        return [v, w] 

    

    def get_odom_stamped(self, x, y, yaw): 

        # x, y and yaw are the robot's position (x,y) and orientation (yaw) 

        # Write the data as a ROS PoseStamped message 

        odom_stamped = Odometry() 

        odom_stamped.header.frame_id = "base_link" 
        odom_stamped.child_frame_id = "chassis"
        odom_stamped.header.stamp = rospy.Time.now() 
        odom_stamped.pose.pose.position.x = x
        odom_stamped.pose.pose.position.y = y

        quat = quaternion_from_euler(0,0,yaw) 
        odom_stamped.pose.pose.orientation.x = quat[0]
        odom_stamped.pose.pose.orientation.y = quat[1]
        odom_stamped.pose.pose.orientation.z = quat[2]
        odom_stamped.pose.pose.orientation.w = quat[3]
         

        return odom_stamped 

         

    def update_robot_pose(self, v, w): 

        #This functions receives the robot speed v [m/s] and w [rad/s] 

        # and gets the robot's pose (x, y, theta) where (x,y) are in [m] and (theta) in [rad] 

        # is the orientation,     

        self.x = self.x + v * np.cos(self.theta)*(self.dt)

        self.y = self.y + v * np.sin(self.theta)*(self.dt)

        self.theta = self.theta + w*self.dt

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  

    OdomClass()  