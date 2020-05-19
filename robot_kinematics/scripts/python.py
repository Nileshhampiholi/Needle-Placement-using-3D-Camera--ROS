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
T = compute_joint_postions(rotation)


for i in range(0,8):
     print(rotation[i],'')


for i in range(0,8):
     print(T[i],'')

