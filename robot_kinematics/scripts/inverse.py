import math
import numpy as np
import sympy as sym
import mpmath as mp
c1, c2, c3, c4, c5, c6, c7 = sym.symbols("c1, c2, c3, c4, c5, c6, c7")

s1, s2, s3, s4, s5, s6, s7 = sym.symbols("s1, s2, s3, s4, s5, s6, s7")

T0_1 =  sym.Matrix([[c1,                 -s1,                    0,                0    ],
         [s1*math.cos(0),  c1* math.cos(0),  -math.sin(0), -math.sin(0)*0.333],
         [s1*math.sin(0),  c1* math.sin(0),   math.cos(0),  math.cos(0)*0.333],
         [0  ,                  0    ,                0     ,           1    ]])

T1_2 =   sym.Matrix([[c2,                 -s2,                    0,                0    ],
         [s2*math.cos(-math.pi/2),  c2* math.cos(-math.pi/2),  -math.sin(-math.pi/2), -math.sin(-math.pi/2)*0],
         [s2*math.sin(-math.pi/2),  c2* math.sin(-math.pi/2),   math.cos(-math.pi/2),  math.cos(-math.pi/2)*0],
         [0  ,                  0    ,                0     ,           1    ]])

T2_3 =   sym.Matrix([[c3,                 -s3,                    0,                0    ],
         [s3*math.cos(math.pi/2),  c3* math.cos(math.pi/2),  -math.sin(math.pi/2), -math.sin(math.pi/2)*0.316],
         [s3*math.sin(math.pi/2),  c3* math.sin(math.pi/2),   math.cos(math.pi/2),  math.cos(math.pi/2)*0.316],
         [0  ,                  0    ,                0     ,           1    ]])

T3_4 =   sym.Matrix([[c4,                 -s4,                    0,                0.0825    ],
         [s4*math.cos(math.pi/2),  c4* math.cos(math.pi/2),  -math.sin(math.pi/2), -math.sin(math.pi/2)*0],
         [s4*math.sin(math.pi/2),  c4* math.sin(math.pi/2),   math.cos(math.pi/2),  math.cos(math.pi/2)*0],
         [0  ,                  0    ,                0     ,           1    ]])

T4_5 =   sym.Matrix([[c5,                 -s5,                    0,                -0.0825    ],
         [s5*math.cos(-math.pi/2),  c5* math.cos(-math.pi/2),  -math.sin(-math.pi/2), -math.sin(-math.pi/2)*0.384],
         [s5*math.sin(-math.pi/2),  c5* math.sin(-math.pi/2),   math.cos(-math.pi/2),  math.cos(-math.pi/2)*0.384],
         [0  ,                  0    ,                0     ,           1    ]])

T5_6 =   sym.Matrix([[c6,                 -s6,                    0,               0    ],
         [s6*math.cos(math.pi/2),  c6* math.cos(math.pi/2),  -math.sin(math.pi/2), -math.sin(math.pi/2)*0],
         [s6*math.sin(math.pi/2),  c6* math.sin(math.pi/2),   math.cos(math.pi/2),  math.cos(math.pi/2)*0],
         [0  ,                  0    ,                0     ,           1    ]])

T6_7 =   sym.Matrix([[c7,                 -s7,                    0,               0.088    ],
         [s7*math.cos(math.pi/2),  c7* math.cos(math.pi/2),  -math.sin(math.pi/2), -math.sin(math.pi/2)*0],
         [s7*math.sin(math.pi/2),  c7* math.sin(math.pi/2),   math.cos(math.pi/2),  math.cos(math.pi/2)*0],
         [0  ,                  0    ,                0     ,           1    ]])

T7_F =  sym.Matrix([[1,                 0,                    0,                0    ],
         [0*math.cos(0),  1* math.cos(0),  -math.sin(0), -math.sin(0)*0.333],
         [0*math.sin(0),  1* math.sin(0),   math.cos(0),  math.cos(0)*0.333],
         [0  ,                  0    ,                0     ,           1    ]])


T0_3 = T0_1*T1_2*T2_3
T0_F = T0_3*T3_4*T4_5*T5_6*T6_7*T7_F

print (T0_3)

print('------------------------------------------------------------------------------------------------------------------------')

print (T0_F)

