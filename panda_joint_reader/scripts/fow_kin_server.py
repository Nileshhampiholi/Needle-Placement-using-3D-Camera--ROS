#!/usr/bin/env python

import rospy
import math
from franka_msgs.srv import cartesianfrompose
import numpy as np
import tf
from geometry_msgs.msg import Transform

def handle_pose_to_cartesian(req):

    m01 = calculate_matrix(0 ,0 ,0.333, req.position[0]) 
    m12 = calculate_matrix(0 ,-math.radians(90), 0, req.position[1])
    m23 = calculate_matrix(0 ,math.radians(90), 0.316, req.position[2])
    m34 = calculate_matrix(0.0825 ,math.radians(90), 0, req.position[3])
    m45 = calculate_matrix(-0.0825 ,-math.radians(90), 0.384, req.position[4])
    m56 = calculate_matrix(0 ,math.radians(90), 0, req.position[5])
    m67 = calculate_matrix(0.088 ,math.radians(90), 0, req.position[6])
    m78 = calculate_matrix(0, 0, 0.107, 0)

    
    m02 = np.dot(m01,m12)
    m03 = np.dot(m02,m23)
    m04 = np.dot(m03,m34)
    m05 = np.dot(m04,m45)
    m06 = np.dot(m05,m56)
    m07 = np.dot(m06,m67)
    m08 = np.dot(m07,m78)

    if m08[2][0] != abs(1):
        pitch = -math.asin(m08[2][0])

        roll = math.atan2(m08[2][1]/math.cos(pitch), m08[2][2]/math.cos(pitch))

        yaw = math.atan2(m08[1][0]/math.cos(pitch), m08[0][0]/math.cos(pitch))

    else:
        yaw = 0
        if m08[2][0] == 1:
            pitch = math.pi/2
            roll = math.atan2(m08[0][1], m08[0][2])

        else:
            pitch = -math.pi/2
            roll = math.atan2(-m08[0][1], -m08[0][2])


    transform.translation.x = m08[0][3] 
    transform.translation.y = m08[1][3]
    transform.translation.z = m08[2][3]
    quatenion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    transform.rotation.x = quatenion[0]
    transform.rotation.y = quatenion[1]
    transform.rotation.z = quatenion[2]
    transform.rotation.w = quatenion[3]

    return transform



if __name__ == '__main__':
    
    
    rospy.init_node('fow_kin_server')
    transform = Transform()


    def calculate_matrix(a, alpha, d, theta):
        matrix =    [[math.cos(theta),             -math.sin(theta),         0,           a ],
                    [math.sin(theta)*math.cos(alpha),  math.cos(theta)* math.cos(alpha),  -math.sin(alpha), -math.sin(alpha)*d],
                    [math.sin(theta)*math.sin(alpha),  math.cos(theta)* math.sin(alpha) ,  math.cos(alpha),  math.cos(alpha)*d],
                    [0  ,                    0    ,                    0     ,      1      ]        ]
        return matrix  
    
  
  
    

    service = rospy.Service("/pose_to_cartesian", cartesianfrompose, handle_pose_to_cartesian)
    print("Server started")
    rospy.spin()


