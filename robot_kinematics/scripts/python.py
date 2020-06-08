import math
from array import *
import numpy as np
import sympy as sym

c1, c2, c3, c4, c5, c6, c7 = sym.symbols("c1, c2, c3, c4, c5, c6, c7")

s1, s2, s3, s4, s5, s6, s7 = sym.symbols("s1, s2, s3, s4, s5, s6, s7")


def matrix(dh):
    T = np.array([[math.cos(dh[0]),             -math.sin(dh[0]),         0,            dh[3]],
         [math.sin(dh[0])*math.cos(dh[1]),  math.cos(dh[0])* math.cos(dh[1]),  -math.sin(dh[1]), -math.sin(dh[1])*dh[2]],
         [math.sin(dh[0])*math.sin(dh[1]),  math.cos(dh[0])* math.sin(dh[1]) ,  math.cos(dh[1]),  math.cos(dh[1])*dh[2]],
         [0  ,                    0    ,                    0     ,      1      ]        ])
    return T  
      
def rotation_matrik(theta, alpha, d, a):
        T = np.array([[math.cos(theta),             -math.sin(theta),         0,           a ],
             [math.sin(theta)*math.cos(alpha),  math.cos(theta)* math.cos(alpha),  -math.sin(alpha), -math.sin(alpha)*d],
            [math.sin(theta)*math.sin(alpha),  math.cos(theta)* math.sin(alpha) ,  math.cos(alpha),  math.cos(alpha)*d],
            [0  ,                    0    ,                    0     ,      1      ]        ])
        return T  

def compute_rotation_matrix(parameters):
     R = []
     for i in range(len(parameters)):
        X =  matrix(parameters[i])  
        R.append(X)
     return R

def compute_homogenous_transfromation_matrix(rotation_matrices):
     T=[]
     X= [ [1,0,0,0], [0,1,0,0],[0,0,1,0] ,[0,0,0,1]]
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
      quaternion = np.array([
        max(magnitude),
        ( R[2][1] - R[1][2] /( 4 * max(magnitude))),
        ( R[0][2] - R[2][0] /( 4 * max(magnitude))),
        ( R[1][0] - R[0][1] /( 4 * max(magnitude)))
        ])
      return quaternion

def joint_position_quaternions(rotation_matrices):
     joint_position_quaternion = []
     q = 0
     for i in range(len(rotation_matrices)):
         q = compute_quaternions(rotation_matrices[i])
         joint_position_quaternion.append(q)
     joint_position_quaternion = np.array(joint_position_quaternion)
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
            c_c = [
            transfromation_matrix [i][0][3],
            transfromation_matrix [i][1][3],
            transfromation_matrix [i][2][3]
            ]
            cartesian_cordinates.append(c_c)
       cartesian_cordinates = np.array(cartesian_cordinates)   
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
     return jacobian
   
def compute_inverse_kinematics(x_current, x_goal, traslation_part , rotation_part,joint_states):
     epsilon = 0.0001
     delta_x = x_goal - x_current
     while(delta_x>=epsilon):
          delta_x = x_goal - x_current
          differential_x = epsilon* delta_x
          jacobian = compute_jacobian(rotation_part,cartesian_cordinates)
          pesudo_inverse = np.linalg.pinv(jacobian, epsilon)
          delta_q = np.dot(pesudo_inverse, joint_states)q_new = joint_states + delta_q
          #poublish Q



x = math.pi/2

dh_parameters = [
    [x,   0.0,       0.333,   0.0    ] ,
    [x,  -math.pi/2, 0.0,     0.0    ] ,
    [x,   math.pi/2, 0.316,   0.0    ] ,
    [x,   math.pi/2, 0.0,     0.0825 ] ,
    [x,  -math.pi/2, 0.384,  -0.0825 ] ,
    [x,   math.pi/2, 0.0,     0.0    ] ,
    [x,   math.pi/2, 0.0,     0.088  ] ,
    [0,   0,         0.107,   0.0    ] ,
   ] 
  
rotation = compute_rotation_matrix(dh_parameters)

joint_postions = compute_homogenous_transfromation_matrix(rotation)

joint_position_quaternions = joint_position_quaternions(joint_postions)

roll_pitch_yaw = compute_roll_pitch_yaw(joint_postions)
 
cartesian_cordinates = get_cartesian_cordinates(joint_postions)

rotation_part = compute_rotation_part(joint_postions)

jacobian = compute_jacobian(rotation_part,cartesian_cordinates)
 
#px = 0.0825*c1*c2*c3 + 0.384*c1*c4*s2 - 0.0825*c1*s2*s4 + 0.316*c1*s2 - 0.0825*c4*(c1*c2*c3 - s1*s3) + 0.088*c6*(c5*(c1*s2*s4 + c4*(c1*c2*c3 - s1*s3)) - s5*(c1*c2*s3 + c3*s1)) - 0.107*c6*(c1*c4*s2 - s4*(c1*c2*c3 - s1*s3)) - 0.0825*s1*s3 - 0.384*s4*(c1*c2*c3 - s1*s3) + 0.107*s6*(c5*(c1*s2*s4 + c4*(c1*c2*c3 - s1*s3)) - s5*(c1*c2*s3 + c3*s1)) + 0.088*s6*(c1*c4*s2 - s4*(c1*c2*c3 - s1*s3))

#py = 0.0825*c1*s3 + 0.0825*c2*c3*s1 + 0.384*c4*s1*s2 - 0.0825*c4*(c1*s3 + c2*c3*s1) + 0.088*c6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - s5*(-c1*c3 + c2*s1*s3)) - 0.107*c6*(c4*s1*s2 - s4*(c1*s3 + c2*c3*s1)) - 0.0825*s1*s2*s4 + 0.316*s1*s2 - 0.384*s4*(c1*s3 + c2*c3*s1) + 0.107*s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - s5*(-c1*c3 + c2*s1*s3)) + 0.088*s6*(c4*s1*s2 - s4*(c1*s3 + c2*c3*s1))

#pz = 0.384*c2*c4 - 0.0825*c2*s4 + 0.316*c2 + 0.0825*c3*c4*s2 + 0.384*c3*s2*s4 - 0.0825*c3*s2 - 0.107*c6*(c2*c4 + c3*s2*s4) + 0.088*c6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + 0.088*s6*(c2*c4 + c3*s2*s4) + 0.107*s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + 0.333]

print (jacobian)
