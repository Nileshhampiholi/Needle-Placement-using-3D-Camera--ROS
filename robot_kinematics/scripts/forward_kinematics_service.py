#!/usr/bin/env python
import rospy
import numpy as np 
import math

from robot_kinematics.srv import forward_kinematics_server
from robot_kinematics.srv import forward_kinematics_serverRequest
from robot_kinematics.srv import forward_kinematics_serverResponse

from robot_kinematics.msg import kinematics_msgs


def handle_compute_kinematics(message):
    position  = message.dh_parameters
    current_position = kinematics_msgs()
    
    dh_parameters =np.array ( [
    [position[0],   0.0,       0.333,   0.0    ] ,
    [position[1],  -math.pi/2, 0.0,     0.0    ] ,
    [position[2],   math.pi/2, 0.316,   0.0    ] ,
    [position[3],   math.pi/2, 0.0,     0.0825 ] ,
    [position[4],  -math.pi/2, 0.384,  -0.0825 ] ,
    [position[5],   math.pi/2, 0.0,     0.0    ] ,
    [position[6],   math.pi/2, 0.0,     0.088  ] ,
    [0,             0,         0.107,   0.0    ] ,
    ] , dtype= 'float64')

    rotation_matrix = compute_rotation_matrix(dh_parameters) 

    transformation_matrix = compute_joint_postions(rotation_matrix)
    
    #quaternions = compute_joint_position_quaternions(transformation_matrix)

    roll_pitch_yaw = compute_roll_pitch_yaw(transformation_matrix)
    
    cartesian_cordinates = get_cartesian_cordinates(transformation_matrix)

    rotation_part = compute_rotation_part(transformation_matrix)

    jacobian = compute_jacobian(rotation_part,cartesian_cordinates)

    xyz_roll_pitch_yaw = join_vectors(cartesian_cordinates, roll_pitch_yaw)
    current_position.transformation_matrix = transformation_matrix
    current_position.xyz_roll_pitch_yaw = xyz_roll_pitch_yaw
    current_position.jacobian  = jacobian

    print ("Returning trasformation_matrices , quaternions, roll_pitch_yaw, and cartesian cordinates in the given order")
  
    return forward_kinematics_serverResponse(current_position)

def join_vectors(cartesian_cordinates, roll_pitch_yaw):
     xyz_roll_pitch_yaw = []
     for i in range(len(cartesian_cordinates)):
          xr = np.append(cartesian_cordinates[i],roll_pitch_yaw[i])
          xyz_roll_pitch_yaw.append(xr)
     return xyz_roll_pitch_yaw

def rotation_matrix(dh):
    T = np.array([[math.cos(dh[0]),                 -math.sin(dh[0]),                    0.0,              dh[3]],
         [math.sin(dh[0])*math.cos(dh[1]),  math.cos(dh[0])* math.cos(dh[1]),  -math.sin(dh[1]), -math.sin(dh[1])*dh[2]],
         [math.sin(dh[0])*math.sin(dh[1]),  math.cos(dh[0])* math.sin(dh[1]) ,  math.cos(dh[1]),  math.cos(dh[1])*dh[2]],
         [0.0 ,                             0.0   ,                             0.0     ,         1.0      ]        ] ,dtype= "float32")
    return T 

def compute_rotation_matrix(parameters):
     R = []
     for i in range(len(parameters)):
        X =  rotation_matrix(parameters[i])  
        R.append(X)
     return R

def compute_joint_postions(rotation_matrices):
    T=[]
    X= np.identity(4,dtype = 'float32')
    for i in range(len(rotation_matrices)):
        X = np.dot(X ,rotation_matrices[i])
        #print(X)
        T.append(X)
        
    return T

'''def compute_quaternions(R):
      magnitude = [
       math.sqrt(abs ( 1.0 + R[0][0] + R[1][1] + R[2][2] ) /4.0 ) ,
       math.sqrt(abs ( 1.0 + R[0][0] - R[1][1] - R[2][2] ) /4.0 ) ,
       math.sqrt(abs ( 1.0 - R[0][0] + R[1][1] - R[2][2] ) /4.0 ) ,
       math.sqrt(abs ( 1.0 - R[0][0] - R[1][1] + R[2][2] ) /4.0 ) ,
        ]
      quaternion = np.array([
        max(magnitude),
        ( R[2][1] - R[1][2] /( 4.0 * max(magnitude))),
        ( R[0][2] - R[2][0] /( 4.0 * max(magnitude))),
        ( R[1][0] - R[0][1] /( 4.0 * max(magnitude)))
        ], dtype= 'float64')
      return quaternion

def compute_joint_position_quaternions(rotation_matrices):
     joint_position_quaternion = []
     for i in range(len(rotation_matrices)):
         q = compute_quaternions(rotation_matrices[i])
         joint_position_quaternion.append(q)
     return joint_position_quaternion
'''
def compute_roll_pitch_yaw(transfromation_matrix):
     roll_pitch_yaw = []
     
     for i in range(len(transfromation_matrix)):
            rpy = []
            if transfromation_matrix[i][2][0] != abs(1):
               pitch = -math.asin(transfromation_matrix[i][2][0])
               roll = math.atan2(transfromation_matrix[i][2][1]/math.cos(pitch), transfromation_matrix[i][2][2]/math.cos(pitch))
               yaw = math.atan2(transfromation_matrix[i][1][0]/math.cos(pitch), transfromation_matrix[i][0][0]/math.cos(pitch))

            else:
               yaw = 0.0
               if transfromation_matrix[i][2][0] == 1.0:
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
            c_c =np.array( [
            transfromation_matrix [i][0][3],
            transfromation_matrix [i][1][3],
            transfromation_matrix [i][2][3]
            ], dtype = 'float64')
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
          cross_product = np.cross(np.array(rotation_part[i][:,2]),(np.array(np.array(traslation_part[n]) -traslation_part[i])))
          jacobian[i] = np.append(  cross_product,     rotation_part[i][:,2] ,axis =0  )
     jacobian = np.transpose(jacobian)
     jacobian = jacobian.tolist()
     return jacobian

def compute_forward_kinematics_service():
    rospy.init_node("forward_kinmatics_service_node")
    s = rospy.Service('compute_forward_kinematics_service',forward_kinematics_server, handle_compute_kinematics)
    print ("Ready to compute forward kinematics.")
    rospy.spin()
    
if __name__ == "__main__":
    compute_forward_kinematics_service()