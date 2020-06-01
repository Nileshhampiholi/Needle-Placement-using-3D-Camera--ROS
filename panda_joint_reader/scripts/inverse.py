#!/usr/bin/env python

import math
import numpy as np
import sympy as sym
import mpmath as mp
from sympy.physics.mechanics import dynamicsymbols

def simplify_matrix(matrix):
    mbee=       [[matrix[0,0].simplify(), matrix[0,1].simplify(), matrix[0,2].simplify(),matrix[0,3].simplify()],
                 [matrix[1,0].simplify(), matrix[1,1].simplify(), matrix[1,2].simplify(), matrix[1,3].simplify()],
                 [matrix[2,0].simplify(), matrix[2,1].simplify(),matrix[2,2].simplify(), matrix[2,3].simplify()],
                 [0  ,                    0    ,                    0     ,      1      ]]
    return mbee

theta1, theta2, theta3, theta4, theta5, theta6, theta7, theta = dynamicsymbols('theta1 theta2 theta3 theta4 theta5 theta6 theta7 theta ')



T0_1 =  sym.Matrix([[sym.cos(theta1),                 -sym.sin(theta1),                    0,                0    ],
         [sym.sin(theta1)*1,  sym.cos(theta1)* 1,  -0, -0*0.333],
         [sym.sin(theta1)*0,  sym.cos(theta1)* 0,   1,  1*0.333],
         [0  ,                  0    ,                0     ,           1    ]])

T1_2 =   sym.Matrix([[sym.cos(theta2),                 -sym.sin(theta2),                    0,                0    ],
         [sym.sin(theta2)*0,  sym.cos(theta2)* 0,  1, 1*0],
         [sym.sin(theta2)*-1,  sym.cos(theta2)* -1,   0,  0*0],
         [0  ,                  0    ,                0     ,           1    ]])

T2_3 =   sym.Matrix([[sym.cos(theta3),                 -sym.sin(theta3),                    0,                0    ],
         [sym.sin(theta3)*0,  sym.cos(theta3)* 0,  -1, -1*0.316],
         [sym.sin(theta3)*1,  sym.cos(theta3)* 1,   0,  0*0.316],
         [0  ,                  0    ,                0     ,           1    ]])

T3_4 =   sym.Matrix([[sym.cos(theta4),                 -sym.sin(theta4),                    0,                0.0825    ],
         [sym.sin(theta4)*0,  sym.cos(theta4)* 0,  -1, -1*0],
         [sym.sin(theta4)*1,  sym.cos(theta4)* 1,   0,  0*0],
         [0  ,                  0    ,                0     ,           1    ]])

T4_5 =   sym.Matrix([[sym.cos(theta5),                 -sym.sin(theta5),                    0,                -0.0825    ],
         [sym.sin(theta5)*0,  sym.cos(theta5)* 0,  1, 1*0.384],
         [sym.sin(theta5)*-1,  sym.cos(theta5)* -1,   0,  0*0.384],
         [0  ,                  0    ,                0     ,           1    ]])

T5_6 =   sym.Matrix([[sym.cos(theta6),                 -sym.sin(theta6),                    0,               0    ],
         [sym.sin(theta6)*0,  sym.cos(theta6)* 0,  -1, -1*0],
         [sym.sin(theta6)*1,  sym.cos(theta6)* 1,   0,  0*0],
         [0  ,                  0    ,                0     ,           1    ]])

T6_7 =   sym.Matrix([[sym.cos(theta7),                 -sym.sin(theta7),                    0,               0.088    ],
         [sym.sin(theta7)*0,  sym.cos(theta7)* 0,  -1, -1*0],
         [sym.sin(theta7)*1,  sym.cos(theta7)* 1,   0,  0*0],
         [0  ,                  0    ,                0     ,           1    ]])

T7_F =  sym.Matrix([[1,                 0,                    0,                0    ],
         [0*1,  1* 1,  -0, -0*0.333],
         [0*0,  1* 0,   1,  1*0.333],
         [0  ,                  0    ,                0     ,           1    ]])


T0_3 = T0_1*T1_2*T2_3
T0_F = T0_3*T3_4*T4_5*T5_6*T6_7*T7_F

print (T0_3)

print('')

print (simplify_matrix(T0_F))



