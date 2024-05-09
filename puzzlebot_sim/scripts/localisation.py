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


np.set_printoptions(suppress=True) 

np.set_printoptions(formatter={'float': '{: 0.4f}'.format}) 


#This class will do the following: 

#   subscribe to the /wr and /wl topic  
#   publish to /odom   

class Localisation():  
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
        self.dt = 0.1 # Desired time to update the robot's pose [s] 

        ############ Variables ############### 

        self.w = 0.0 # robot's angular speed [rad/s] 
        self.v = 0.0 #robot's linear speed [m/s] 
        self.x = 0.0 # 
        self.y = 0.0  
        self.theta = 0.0  
        self.wr = 0.0
        self.wl = 0.0

        ########################################### DEAD RECKONING ############################

        ######### CONTROL VARIABLES FOR Q ##########
        #self.wr_k = 0.15

        #self.wl_k = 0.2

        self.wr_k = 0.2

        self.wl_k = 0.8

        ###################

        self.z_covariance = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])

        self.miu = np.array([0, 0, 0])

        self.w_sigma_const = 0.5 * self.dt * self.r
        
        self.odom = Odometry() 

        rate = rospy.Rate(int(1.0/self.dt)) # The rate of the while loop will be the inverse of the desired delta_t 

        while not rospy.is_shutdown(): 

            #####OBTENER VELOCIDADES #####################
            [self.v, self.w] = self.get_robot_vel(self.wr,self.wl)

            ####### DEAD RECKONING ########################

            self.sigma_k = np.array([[self.wr_k*self.wr, 0],
                                     [0,  self.wl_k*self.wl]])
        
    

            self.w_sigma = self.w_sigma_const * (np.array([[np.cos(self.miu[2]),np.cos(self.miu[2])],
                                                                [np.sin(self.miu[2]),np.sin(self.miu[2])],
                                                                [2/self.L, 2/self.L]]))
        
        
            self.Q_k =  self.w_sigma @ self.sigma_k @ self.w_sigma.T


            
            self.H = np.array([[1, 0, -(((self.wl + self.wr) *0.5 *self.r*self.dt )* np.sin(self.miu[2]))],
                                   [0, 1,(((self.wl + self.wr) *0.5 *self.r*self.dt )* np.cos(self.miu[2]))],
                                   [0, 0, 1]])
            


            self.miu = np.array([self.miu[0] + (((self.wl + self.wr) *0.5 *self.r*self.dt )* np.cos(self.miu[2])),
                                     self.miu[1] + (((self.wl + self.wr) *0.5 *self.r*self.dt )* np.sin(self.miu[2])),
                                     self.miu[2] + (((self.wr - self.wl)/self.L)*self.r*self.dt)])
            
            

            self.z_covariance = (self.H @ self.z_covariance @ self.H.T) + self.Q_k



            ####################################################

           
            self.update_robot_pose(self.v, self.w) 
            odom_msg = self.get_odom_stamped(self.x, self.y, self.theta,self.z_covariance,self.v,self.w) 
            odom_msg.twist.twist.linear.x = self.v
            odom_msg.twist.twist.angular.z = self.w

            print(self.wr)
            print(self.wl)

            print(self.miu)
        



            ######################################### 
            ######## Publish the data ################# 
    
            self.odom_pub.publish(odom_msg) 

            rate.sleep() 

     

    def wl_cb(self, msg): 
        self.wl = msg.data

    def wr_cb(self, msg): 
        self.wr = msg.data

    def get_robot_vel(self,wr,wl): 
        v = ((wl + wr)/2)*self.r 
        w = ((wr - wl) / self.L)*self.r

        return [v, w] 


    def get_odom_stamped(self, x, y, yaw,Sigma,Mu_v,Mu_w): 
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

        odom_array = np.array([[Sigma[0][0],Sigma[0][1],0,0,0,Sigma[0][2]],
                       [Sigma[1][0],Sigma[1][1],0,0,0,Sigma[1][2]],
                       [0,0,0,0,0,0],
                       [0,0,0,0,0,0],
                       [0,0,0,0,0,0],
                       [Sigma[2][0],Sigma[2][1],0,0,0,Sigma[2][2]]])
        
        odom_stamped.pose.covariance = odom_array.flatten().tolist()

        odom_stamped.twist.twist.linear.x = Mu_v
        odom_stamped.twist.twist.angular.z = Mu_w
        
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
    Localisation()  
