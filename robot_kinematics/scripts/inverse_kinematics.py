#!/usr/bin/env python
import rospy
import math
import numpy as np
from robot_kinematics.msg import kinematics_msgs



def states_callback(msg):

    print(msg.transformation_matrix)

if __name__ == '__main__':

    rospy.init_node('inverse_kinematics')
    topic_name = 'robot_kinematics'
    rospy.Subscriber(topic_name, kinematics_msgs, states_callback)
    rospy.spin()


































'''
if __name__ == '__main__':
    
    rospy.init_node('inverse_kinematics')


    theta = np.array([0 ,0, 0 , theta_4 ,theta_3 ,theta_2 , theta_1])

    print (theta)
    pub = rospy.Publisher("/joint_position_example_controller_sim/joint_command", Float64MultiArray, queue_size = 1000)
    init_position = [0,-0.5,0,-2.5,0,2,0]
    joint_move_distance = math.radians(30)
    rospy.sleep(2)

    msg = Float64MultiArray()    
    msg.data = init_position
    pub.publish(msg)
    
    rospy.sleep(2)
    msg = Float64MultiArray()
    msg.data = theta

    pub.publish(msg)
    
'''