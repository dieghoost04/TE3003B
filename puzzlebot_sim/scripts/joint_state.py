#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion

class WheelState():
    def __init__(self):
        rospy.init_node('WheelState')
        self.last_odometry = Odometry()
        self.last_time = rospy.get_time()

        rospy.Subscriber('/odom', Odometry, self.odometry_cb)
        pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        pub_v = rospy.Publisher('/vel', Float32, queue_size=10)
        rospy.sleep(1) 

        self.linear_vel = 0
        self.angular_vel = 0

        self.wr_vel = 0
        self.wl_vel = 0


        rate = rospy.Rate(10) 
        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            # Calcula el tiempo transcurrido desde el inicio
            current_time = rospy.Time.now()
            dt = (current_time - start_time).to_sec()

            self.linear_vel = self.last_odometry.twist.twist.linear.x
            self.angular_vel = self.last_odometry.twist.twist.angular.z

            # Calcular la velocidad angular de cada rueda usando el radio de la base del robot
            wheel_base = 0.19 
            self.wr_vel = (self.linear_vel + self.angular_vel * wheel_base / 2 ) / 0.1
            self.wl_vel = (self.linear_vel - self.angular_vel * wheel_base / 2 ) / 0.1

        
            # Crea un mensaje de JointState
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = rospy.Time.now()
            joint_state_msg.name = ['wl_joint', 'wr_joint']  # Nombres de los joints

            # Calcula las posiciones deseadas de las ruedas en función del tiempo transcurrido
            left_wheel_position = -self.wl_vel * dt 
            right_wheel_position = self.wr_vel * dt

            # Agrega las posiciones deseadas al mensaje de JointState
            joint_state_msg.position = [left_wheel_position, right_wheel_position]

            pub_v.publish(self.wr_vel)
            pub.publish(joint_state_msg)

            rate.sleep()


    def odometry_cb(self, msg):
        self.last_odometry = msg

    
if __name__ == '__main__':
    WheelState()

