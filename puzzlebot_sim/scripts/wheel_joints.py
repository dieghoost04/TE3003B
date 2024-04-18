#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState


# Declare the output Messages

#q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0.0)
contJoints = JointState()

# Declare the output Messages
def init_joints():
    contJoints.header.frame_id = "link1"
    contJoints.header.stamp = rospy.Time.now()
    contJoints.name.extend(["joint1", "joint2", "joint3"])
    contJoints.position.extend([0.0, 0.0, 0.0])
    contJoints.velocity.extend([0.0, 0.0, 0.0])
    contJoints.effort.extend([0.0, 0.0, 0.0])

#wrap to pi function
def wrap_to_Pi(theta):
    result = np.fmod((theta + np.pi),(2 * np.pi))
    if(result < 0):
        result += 2 * np.pi
    return result - np.pi

def wrap_prismatic_position(tiempo):
    # Definir los límites del rango de la articulación prismatic
    lower_limit = -2.0
    upper_limit = 2.0
    period = 4.0
    normalized_position = 2 * np.sin(2 * np.pi * tiempo / period)
    
    return normalized_position
    # Envolver la posición dentro del rango definido
    return np.clip(position, lower_limit, upper_limit)

def normalize_revolute_position(tiempo):

    lower_limit = -3.14159
    upper_limit = 3.14159

    period = 4
    normalized_position = np.sin(2 * np.pi * tiempo / period)
    scaled_position = upper_limit * normalized_position
    return scaled_position




#Stop Condition
def stop():
    #Setup the stop message (can be the same as the control message)
    print("Stopping")

if __name__=='__main__':

    #Initialise and Setup node
    rospy.init_node("Puzzlebot_Pose_Estimator")
 
    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("/rate",100))
    rospy.on_shutdown(stop)

    #Init joints
    init_joints()

    #Setup Transform Broadcasters
    joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=1)

    print("The Estimator is Running")
    try:
    #Run the node
        while not rospy.is_shutdown(): 
            t = rospy.Time.now().to_sec()
            contJoints.header.stamp = rospy.Time.now()
            contJoints.position[0] = wrap_to_Pi(t*0.1)
            contJoints.position[1] = normalize_revolute_position(t*0.1)
            contJoints.position[2] = wrap_prismatic_position(t*0.2)
            
            joint_pub.publish(contJoints)

            loop_rate.sleep()
 
    except rospy.ROSInterruptException:
        pass
