#!/usr/bin/env python
import rospy
import math
import numpy as np
from std_msgs.msg import Float64MultiArray

def rotation_matrix(dh):
    T = np.array([[math.cos(dh[0]),             -math.sin(dh[0]),         0,            dh[3]],
         [math.sin(dh[0])*math.cos(dh[1]),  math.cos(dh[0])* math.cos(dh[1]),  -math.sin(dh[1]), -math.sin(dh[1])*dh[2]],
         [math.sin(dh[0])*math.sin(dh[1]),  math.cos(dh[0])* math.sin(dh[1]) ,  math.cos(dh[1]),  math.cos(dh[1])*dh[2]],
         [0  ,                    0    ,                    0     ,      1      ]        ])
    #T = np.linalg.inv(T)
    return T  


if __name__ == '__main__':
    
    rospy.init_node('inverse_kinematics')
    a_3 = 0.088
    a_4 = 0.088
    d_3 = 0.384
    d_5 = 0.316
    d_7 = -0.333
    delta = 0

    T_E_E =  [[ 0.96280283, -0.26675206,  0.04305865,  0.21021535],
              [-0.22147043, -0.68777444,  0.69131553, -0.12232927],
              [-0.1547952,  -0.67513677 ,-0.72126887,  0.38914927],
              [ 0,           0,          0,             1        ]]
    T_E_E_inv = np.linalg.inv(T_E_E)

    n = np.array(T_E_E_inv[:,0])
    n1 = n.reshape(4,1)

    b = np.array(T_E_E_inv[:,2])
    b1 = b.reshape(4,1)
    p = T_E_E_inv[:,3]
    p = p.reshape(4,1)

    w = p + b1*d_7
    theta_1 = np.unwrap([math.atan2(w[1], w[0])])[0]

    T01 = rotation_matrix([theta_1, math.pi/2, 0.107, 0.088])
    w_d = np.dot(T01,w)

    D = math.sqrt(math.pow(w_d[0],2)+math.pow(w_d[1],2)+math.pow(w_d[3],2))

    hypo_1 = math.hypot(d_3, a_3)
    hypo_2 = math.hypot(d_5, a_4)
    a = math.acos(a_4/hypo_2) 
    b = math.acos(a_3/hypo_1)

    C = math.acos(pow(hypo_1, 2) + pow(hypo_2, 2) - pow(D,2)/ 2 * hypo_1 * hypo_2)


    theta_4 = (2 * math.pi) - (a + b + C) - (math.pi/2)

    angle_b = math.atan2(d_5, a_4)
    angle_c = angle_b + theta_4  
    real_d_4 = np.linalg.norm([a_4, d_5])

    wrist_x = d_3 + (math.cos(angle_c) * real_d_4)
    wrist_y = a_4 + (math.sin(angle_c) * real_d_4)  

    v_d_5 = np.linalg.norm([wrist_x - d_3 , wrist_y])
    wrist_mag_sqr = pow(wrist_x, 2) + pow(wrist_y, 2)
    v_theta_4 = -(math.acos((math.pow(v_d_5, 2)+ math.pow(d_3, 2) - wrist_mag_sqr) / (2 * v_d_5 * d_3)) - math.pi/2) 
    phi= math.atan2(w_d[1],w_d[0])
    theta_2 = phi - math.asin((d_3 + (v_d_5 *math.sin(v_theta_4)))/ math.sqrt(math.pow(w_d[1],2) + math.pow(w_d[0],2)))
    T12 = rotation_matrix([theta_2,-math.pi/2 , 0 , 0])
    w_2_d = np.dot(T12,w_d)

    theta_3 = math.atan2(w_2_d[1], w_2_d[0])

    '''theta5'''

    T23 = rotation_matrix([theta_3 , math.pi/2 , 0.384 ,0.088])
    T34 = rotation_matrix([theta_4+ math.pi/2 , -math.pi/2 , 0 ,0.088])
    T02 = np.dot(T01,T12)
    T03 = np.dot(T02,T23)
    T04 = np.dot(T03,T34)

    p_1 = np.dot(T04, p)
    theta_5 = math.atan2(-p_1[2], p_1[0])

    '''theta6'''

    norm =math.sqrt( math.pow(p_1[0],2) + math.pow(p_1[1],2) +math.pow(p_1[2] ,2 ) )

    theta_6 = math.acos(-p_1[1]/norm )

    '''theta7'''
    T45 = rotation_matrix([theta_5 , math.pi/2 , 0.316 ,0])
    T56 = rotation_matrix([theta_6+ math.pi , math.pi/2 , 0 ,0])
    T05 = np.dot(T04,T45)
    T06 = np.dot(T05,T56)

    T60 = np.linalg.inv(T06)
    unit = np.array([1,0,0,0])
    u = unit.reshape(4,1)
    n_6 = np.dot(T60,u)

    s_p = np.dot(n, n_6)

    theta_7 = math.acos(s_p)

    theta = np.array([0 ,0, 0 , theta_4 ,theta_3 ,theta_2 , theta_1])

    print (theta)
    pub = rospy.Publisher("/joint_position_example_controller_sim/joint_command", Float64MultiArray, queue_size = 1000)
    init_position = [0,-0.5,0,-2.5,0,2,0]
    joint_move_distance = math.radians(30)
    rospy.sleep(2)

    msg = Float64MultiArray()    
    msg.data = init_position
    pub.publish(msg)
    
    rospy.sleep(2)
    msg = Float64MultiArray()
    msg.data = theta

    pub.publish(msg)
    
