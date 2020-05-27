import math
import numpy as np
import sympy as sym
import mpmath as mp
c1, c2, c3, c4, c5, c6, c7 = sym.symbols("c1, c2, c3, c4, c5, c6, c7")

s1, s2, s3, s4, s5, s6, s7 = sym.symbols("s1, s2, s3, s4, s5, s6, s7")

T0_1 =  [[c1,                 -s1,                    0,                0    ],
         [s1*math.cos(0),  c1* math.cos(0),  -math.sin(0), -math.sin(0)*0.333],
         [s1*math.sin(0),  c1* math.sin(0),   math.cos(0),  math.cos(0)*0.333],
         [0  ,                  0    ,                0     ,           1    ]]

T1_2 =  [[c2,                 -s2,                    0,                0    ],
         [s2*math.cos(-math.pi/2),  c2* math.cos(-math.pi/2),  -math.sin(-math.pi/2), -math.sin(-math.pi/2)*0],
         [s2*math.sin(-math.pi/2),  c2* math.sin(-math.pi/2),   math.cos(-math.pi/2),  math.cos(-math.pi/2)*0],
         [0  ,                  0    ,                0     ,           1    ]]

T2_3 =  [[c3,                 -s3,                    0,                0    ],
         [s3*math.cos(math.pi/2),  c3* math.cos(math.pi/2),  -math.sin(math.pi/2), -math.sin(math.pi/2)*0.316],
         [s3*math.sin(math.pi/2),  c3* math.sin(math.pi/2),   math.cos(math.pi/2),  math.cos(math.pi/2)*0.316],
         [0  ,                  0    ,                0     ,           1    ]]

T3_4 =  [[c4,                 -s4,                    0,                0.0825    ],
         [s4*math.cos(math.pi/2),  c4* math.cos(math.pi/2),  -math.sin(math.pi/2), -math.sin(math.pi/2)*0],
         [s4*math.sin(math.pi/2),  c4* math.sin(math.pi/2),   math.cos(math.pi/2),  math.cos(math.pi/2)*0],
         [0  ,                  0    ,                0     ,           1    ]]

T4_5 =  [[c5,                 -s5,                    0,                -0.0825    ],
         [s5*math.cos(-math.pi/2),  c5* math.cos(-math.pi/2),  -math.sin(-math.pi/2), -math.sin(-math.pi/2)*0.384],
         [s5*math.sin(-math.pi/2),  c5* math.sin(-math.pi/2),   math.cos(-math.pi/2),  math.cos(-math.pi/2)*0.384],
         [0  ,                  0    ,                0     ,           1    ]]

T5_6 =  [[c6,                 -s6,                    0,               0    ],
         [s6*math.cos(math.pi/2),  c6* math.cos(math.pi/2),  -math.sin(math.pi/2), -math.sin(math.pi/2)*0],
         [s6*math.sin(math.pi/2),  c6* math.sin(math.pi/2),   math.cos(math.pi/2),  math.cos(math.pi/2)*0],
         [0  ,                  0    ,                0     ,           1    ]]

T6_7 =  [[c7,                 -s7,                    0,               0.088    ],
         [s7*math.cos(math.pi/2),  c7* math.cos(math.pi/2),  -math.sin(math.pi/2), -math.sin(math.pi/2)*0],
         [s7*math.sin(math.pi/2),  c7* math.sin(math.pi/2),   math.cos(math.pi/2),  math.cos(math.pi/2)*0],
         [0  ,                  0    ,                0     ,           1    ]]

T7_F = [[1,                 0,                    0,                0    ],
         [0*math.cos(0),  1* math.cos(0),  -math.sin(0), -math.sin(0)*0.333],
         [0*math.sin(0),  1* math.sin(0),   math.cos(0),  math.cos(0)*0.333],
         [0  ,                  0    ,                0     ,           1    ]]

T0_2 = np.dot(T0_1,T1_2)
T0_3 = np.dot(T0_2,T2_3)
#for i in range(len(T0_3)):
#print(T0_3[i],'')

T0_4 = np.dot(T0_3,T3_4)
T0_5 = np.dot(T0_4,T4_5)
T0_6 = np.dot(T0_5,T5_6)
T0_7 = np.dot(T0_6,T6_7)
T0_F = np.dot(T0_7,T7_F)

for i in range(len(T0_F)):
    for j in range(len(T0_F)):
        print('')
        print(i , j)
        print(T0_F[i][j],'')
        print('')