#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import math
import numpy as np


def callback(message):
    position  = message.position
    dh_parameters = [
    [position[0],   0.0,       0.333,   0.0    ] ,
    [position[1],  -math.pi/2, 0.0,     0.0    ] ,
    [position[2],   math.pi/2, 0.316,   0.0    ] ,
    [position[3],   math.pi/2, 0.0,     0.0825 ] ,
    [position[4],  -math.pi/2, 0.384,  -0.0825 ] ,
    [position[5],   math.pi/2, 0.0,     0.0    ] ,
    [position[6],   math.pi/2, 0.0,     0.088  ] ,
    [0,             0,         0.107,   0.0    ] ,
    ] 
    rotation_matrix = compute_rotation_matrix(dh_parameters) 
    trasfromation_matrix = compute_joint_postions(rotation_matrix)
    joint_position_quaternions = compute_joint_position_quaternions(trasfromation_matrix)

    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', trasfromation_matrix)  

def forward_kinematics():
    rospy.init_node('forward_kinematics', anonymous=True)
    topic = "/joint_states"
    rospy.Subscriber(topic, JointState, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def rotation_matrix(dh):
    T = [[math.cos(dh[0]),             -math.sin(dh[0]),         0,            dh[3]],
         [math.sin(dh[0])*math.cos(dh[1]),  math.cos(dh[0])* math.cos(dh[1]),  -math.sin(dh[1]), -math.sin(dh[1])*dh[2]],
         [math.sin(dh[0])*math.sin(dh[1]),  math.cos(dh[0])* math.sin(dh[1]) ,  math.cos(dh[1]),  math.cos(dh[1])*dh[2]],
         [0  ,                    0    ,                    0     ,      1      ]        ]
    return T 

def compute_rotation_matrix(parameters):
     R = []
     for i in range(len(parameters)):
        X =  rotation_matrix(parameters[i])  
        R.append(X)
     return R

def compute_joint_postions(rotation_matrices):
    T=[]
    X= np.identity(4)
    for i in range(0,8):
        X = np.dot(X ,rotation_matrices[i])
        #print(X)
        T.append(X)
    return T

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

def compute_joint_position_quaternions(rotation_matrices):
     joint_position_quaternion = []
     q = 0
     for i in range(len(rotation_matrices)):
         q = compute_quaternions(rotation_matrices[i])
         joint_position_quaternion.append(q)
     return joint_position_quaternion

if __name__ == '__main__':
    forward_kinematics()

  