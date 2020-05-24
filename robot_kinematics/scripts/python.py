import math
from array import *
import numpy as np

def rotation_matrix(dh):
    T = [[math.cos(dh[0]),             -math.sin(dh[0]),         0,            dh[3]],
         [math.sin(dh[0])*math.cos(dh[1]),  math.cos(dh[0])* math.cos(dh[1]),  -math.sin(dh[1]), -math.sin(dh[1])*dh[2]],
         [math.sin(dh[0])*math.sin(dh[1]),  math.cos(dh[0])* math.sin(dh[1]) ,  math.cos(dh[1]),  math.cos(dh[1])*dh[2]],
         [0  ,                    0    ,                    0     ,      1      ]        ]
    return T  
      
def rotation_matrik(theta, alpha, d, a):
        T = [[math.cos(theta),             -math.sin(theta),         0,           a ],
             [math.sin(theta)*math.cos(alpha),  math.cos(theta)* math.cos(alpha),  -math.sin(alpha), -math.sin(alpha)*d],
            [math.sin(theta)*math.sin(alpha),  math.cos(theta)* math.sin(alpha) ,  math.cos(alpha),  math.cos(alpha)*d],
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
      quaternion = [
        max(magnitude),
        ( R[2][1] - R[1][2] /( 4 * max(magnitude))),
        ( R[0][2] - R[2][0] /( 4 * max(magnitude))),
        ( R[1][0] - R[0][1] /( 4 * max(magnitude)))
        ]
      return quaternion

def joint_position_quaternions(rotation_matrices):
     joint_position_quaternion = []
     q = 0
     for i in range(len(rotation_matrices)):
         q = compute_quaternions(rotation_matrices[i])
         joint_position_quaternion.append(q)
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
       return cartesian_cordinates


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

joint_postions = compute_joint_postions(rotation)

joint_position_quaternions = joint_position_quaternions(joint_postions)

roll_pitch_yaw = compute_roll_pitch_yaw(joint_postions)

#for i in range(len(joint_position_quaternions)):
 #   print (joint_position_quaternions[i],"")

 
cartesian_cordinates = get_cartesian_cordinates(joint_postions)


#for i in range(len(roll_pitch_yaw)):
  #  print (roll_pitch_yaw[i],"")

print (joint_postions[0][0][3])
print (joint_postions[0])
print ("")
for i in range(len(cartesian_cordinates)):
     print ("cordinates of frame ",i)
     print (cartesian_cordinates[i],"")
     

