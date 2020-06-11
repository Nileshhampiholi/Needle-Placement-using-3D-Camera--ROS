#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import math
import numpy as np
from geometry_msgs.msg import Transform
from std_msgs.msg import Float64MultiArray

global position

def callback(message):
     abc  = Float64MultiArray()
     global position
     position  = message.position
    
     print('')
     print(position)
     print ('')
     jacobian = xxxxx(position)
    
     
     X = []
     for i in range(len(jacobian)):
          X = np.append(X,jacobian[i])
     print (X)
     
     
     
     abc.lable = 'jacobian'
     abc.data = X
     pub = rospy.Publisher('forward_kinematics', Float64MultiArray, queue_size=1)
     rate = rospy.Rate(1) # 1hz
     pub.publish(abc)

 


def xxxxx(position):
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
     roll_pitch_yaw = compute_roll_pitch_yaw(trasfromation_matrix)
     cartesian_cordinates = get_cartesian_cordinates(trasfromation_matrix)
     rotation_part = compute_rotation_part(trasfromation_matrix)
     jacobian = compute_jacobian(rotation_part,cartesian_cordinates)
     return jacobian


     
def forward_kinematics():
    rospy.init_node('forward_kinematics', anonymous=True)
    topic = "/joint_states"
    rospy.Subscriber(topic, JointState, callback)
    rospy.sleep(1)
    # spin() simply keeps python from exiting until this node is stopped

def join_vectors(cartesian_cordinates, roll_pitch_yaw):
     xyz_roll_pitch_yaw = []
     for i in range(len(cartesian_cordinates)):
          xr = np.append(cartesian_cordinates[i],roll_pitch_yaw[i])
          xyz_roll_pitch_yaw.append(xr)
     return xyz_roll_pitch_yaw

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
      quaternion =np.array( [
        max(magnitude),
        ( R[2][1] - R[1][2] /( 4 * max(magnitude))),
        ( R[0][2] - R[2][0] /( 4 * max(magnitude))),
        ( R[1][0] - R[0][1] /( 4 * max(magnitude)))
        ])
      return quaternion

def compute_joint_position_quaternions(rotation_matrices):
     joint_position_quaternion = []
     q = 0
     for i in range(len(rotation_matrices)):
         q = compute_quaternions(rotation_matrices[i])
         joint_position_quaternion.append(q)
     #joint_position_quaternion = np.array(joint_position_quaternion)
     return joint_position_quaternion

def compute_roll_pitch_yaw(transfromation_matrix):
     roll_pitch_yaw = []
     
     for i in range(len(transfromation_matrix)):
            rpy = []
            if transfromation_matrix[i][2][0] != abs(1):
               pitch = -math.asin(transfromation_matrix[i][2][0])
               roll = math.atan2(transfromation_matrix[i][2][1]/math.cos(pitch), transfromation_matrix[i][2][2]/math.cos(pitch))
               yaw = math.atan2(transfromation_matrix[i][1][0]/math.cos(pitch), transfromation_matrix[i][0][0]/math.cos(pitch))

            else:
               yaw = 0
               if transfromation_matrix[i][2][0] == 1:
                    pitch = math.pi/2
                    roll = math.atan2(transfromation_matrix[i][0][1], transfromation_matrix[i][0][2])

               else:
                    pitch = -math.pi/2
                    roll = math.atan2(-transfromation_matrix[i][0][1], -transfromation_matrix[i][0][2])
            rpy.append(roll)
            rpy.append(pitch)
            rpy.append(yaw)
            roll_pitch_yaw.append(rpy)
             
     return roll_pitch_yaw 

def get_cartesian_cordinates(transfromation_matrix):
       cartesian_cordinates = []
       for i in range(len(transfromation_matrix)):
            c_c = np.array( [
            transfromation_matrix [i][0][3],
            transfromation_matrix [i][1][3],
            transfromation_matrix [i][2][3]
            ])
            cartesian_cordinates.append(c_c)
       return cartesian_cordinates

def compute_rotation_part(transfromation_matrix):
     rotation_part = []
     for i in range(len(transfromation_matrix)):
          temp = np.array(transfromation_matrix[i][:3, :3])
          rotation_part.append(temp)
     rotation_part = np.array(rotation_part)
     return rotation_part

def compute_jacobian(rotation_part, traslation_part):
     jacobian = np.zeros((7,6))
     for i in range (len(rotation_part)-1):
          n = len(rotation_part) - 1
          print ([i ,n ])
          cross_product = np.cross(np.array(rotation_part[i][:,2]),(np.array(np.array(traslation_part[n]) -traslation_part[i])))
          jacobian[i] = np.append(  cross_product,     rotation_part[i][:,2] ,axis =0  )
     jacobian = np.transpose(jacobian)
     jacobian = jacobian.tolist()
     return jacobian

def compute_rotation_part(transfromation_matrix):
     rotation_part = []
     for i in range(len(transfromation_matrix)):
          temp = np.array(transfromation_matrix[i][:3, :3])
          rotation_part.append(temp)
     rotation_part = np.array(rotation_part)
     return rotation_part

if __name__ == '__main__':
    forward_kinematics()
    i = 5
    while (i >1 ):
          
          print ("")
          print (i)
          print (jacobian)
          i = i-1
          rospy.sleep(1)
     

    rospy.spin()



