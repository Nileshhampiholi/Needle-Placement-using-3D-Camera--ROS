#!/usr/bin/env python

import rospy
import math
import numpy as np
import tf
from geometry_msgs.msg import Transform
from sensor_msgs.msg import JointState
import time



def handle_pose_to_cartesian(msg):

    # DH parameters of Franka_Panda, msg.position are the incoming values of theta from jointstates topic
    m01 = calculate_matrix(0 ,0 ,0.333, msg.position[0]) 
    m12 = calculate_matrix(0 ,-math.radians(90), 0, msg.position[1])
    m23 = calculate_matrix(0 ,math.radians(90), 0.316, msg.position[2])
    m34 = calculate_matrix(0.0825 ,math.radians(90), 0, msg.position[3])
    m45 = calculate_matrix(-0.0825 ,-math.radians(90), 0.384, msg.position[4])
    m56 = calculate_matrix(0 ,math.radians(90), 0, msg.position[5])
    m67 = calculate_matrix(0.088 ,math.radians(90), 0, msg.position[6])
    m78 = calculate_matrix(0, 0, 0.107, 0)

    # Multiplication of transformation matrix
    m02 = np.dot(m01,m12)
    m03 = np.dot(m02,m23)
    m04 = np.dot(m03,m34)
    m05 = np.dot(m04,m45)
    m06 = np.dot(m05,m56)
    m07 = np.dot(m06,m67)
    m08 = np.dot(m07,m78) #The transformation matric from panda_link0 to panda_link8

    Matrices = [m02, m03, m04, m05, m06, m07, m08]
    #print(matrices) # Print transformation matrices 
    make_cartesian(Matrices)



def make_cartesian(matrix):

    Joint_number = 1
    for i in matrix: # Calculating cartesian from every matrices calculated above
        
        transform = Transform()
        # Calculating roll, pitch and yaw angles from rotation matrix
        if i[2][0] != abs(1):
            pitch = -math.asin(i[2][0])

            roll = math.atan2(i[2][1]/math.cos(pitch), i[2][2]/math.cos(pitch))

            yaw = math.atan2(i[1][0]/math.cos(pitch), i[0][0]/math.cos(pitch))

        else:
            yaw = 0
            if i[2][0] == 1:
                pitch = math.pi/2
                roll = math.atan2(i[0][1], i[0][2])

            else:
                pitch = -math.pi/2
                roll = math.atan2(-i[0][1], -i[0][2])

    
        # Getting the translation vector by taking the first 3 values of the last column of every matrix 

        transform.translation.x = i[0][3] 
        transform.translation.y = i[1][3]
        transform.translation.z = i[2][3]
        #quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw) # Calculating queternion from euler
        quaternion = compute_quaternions(i)
        transform.rotation.x = quaternion[0]
        transform.rotation.y = quaternion[1]
        transform.rotation.z = quaternion[2]
        transform.rotation.w = quaternion[3]

        trans.append(transform)


        print("Transformation of joint number " + str(Joint_number) + " is  \n" + str(transform))
        Joint_number += 1

#Standard matrix to calculate transformation with DH parameters
def calculate_matrix(a, alpha, d, theta): 
    matrix =    [[math.cos(theta),             -math.sin(theta),         0,           a ],
                [math.sin(theta)*math.cos(alpha),  math.cos(theta)* math.cos(alpha),  -math.sin(alpha), -math.sin(alpha)*d],
                [math.sin(theta)*math.sin(alpha),  math.cos(theta)* math.sin(alpha) ,  math.cos(alpha),  math.cos(alpha)*d],
                [0  ,                    0    ,                    0     ,      1      ]        ]
    return matrix

def compute_quaternions(R):
      magnitude = [
       math.sqrt(abs ( 1 + R[0][0] + R[1][1] + R[2][2] ) /4 ) ,
       math.sqrt(abs ( 1 + R[0][0] - R[1][1] - R[2][2] ) /4 ) ,
       math.sqrt(abs ( 1 - R[0][0] + R[1][1] - R[2][2] ) /4 ) ,
       math.sqrt(abs ( 1 - R[0][0] - R[1][1] + R[2][2] ) /4 ) ,
        ]
      quaternion = [
        max(magnitude),
        ( R[2][1] - R[1][2] /( 4 * max(magnitude))),
        ( R[0][2] - R[2][0] /( 4 * max(magnitude))),
        ( R[1][0] - R[0][1] /( 4 * max(magnitude)))
        ]
      return quaternion


if __name__ == '__main__':
    
    
    rospy.init_node('fow_kin')


    trans = []

    try:
        sub = rospy.Subscriber('/joint_states', JointState, handle_pose_to_cartesian) #Subscribing to the topic, which gives current joint angles for every joint
        rospy.spin()
    except rospy.ROSException as e:
        rospy.logwarn("Node failed : " + str(e))



