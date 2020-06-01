#!/usr/bin/env python

import math
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from sympy.matrices import Matrix
import sympy as sp
from sympy.physics.mechanics import dynamicsymbols


def calculate_inverse_matrix(a, alpha, d, theta): 
    matrix =    [[math.cos(theta),             -math.sin(theta),         0,           a ],
                [math.sin(theta)*math.cos(alpha),  math.cos(theta)* math.cos(alpha),  -math.sin(alpha), -math.sin(alpha)*d],
                [math.sin(theta)*math.sin(alpha),  math.cos(theta)* math.sin(alpha) ,  math.cos(alpha),  math.cos(alpha)*d],
                [0  ,                    0    ,                    0     ,      1      ]]

    inv_matrix = np.linalg.inv(matrix)
    
    return inv_matrix

if __name__ == '__main__':
    
    rospy.init_node('inv_kin')
    goal_matrix =       [[ 0.96280283, -0.26675206,  0.04305865,  0.21021535],
                        [-0.22147043, -0.68777444,  0.69131553, -0.12232927],
                        [-0.1547952,  -0.67513677 ,-0.72126887,  0.38914927],
                        [ 0,          0,          0,          1        ]]
    
    d_7 = -0.333 #Check
    d_3 = 0.384
    d_5 = 0.316
    a_3 = 0.0825
    a_4 = 0.0825

    """ theta_1 """
    inv_goal_matrix = np.linalg.inv(goal_matrix)

    #print(inv_goal_matrix)

    

    #Location of Wrist

    wrist_pos = Matrix([[inv_goal_matrix[0][3], inv_goal_matrix[1][3], inv_goal_matrix[2][3]]]) + d_7 * Matrix([[inv_goal_matrix[0][2], inv_goal_matrix[1][2], inv_goal_matrix[2][2]]])
    
    #print(wrist_pos)
    theta_1 = np.unwrap([math.atan2(wrist_pos[1], wrist_pos[0])])[0]

    print(theta_1)

    """ theta_4 """

    m1_0 = calculate_inverse_matrix(0.088, math.radians(90), 0.107, theta_1)

    wrist = [[wrist_pos[0]], [wrist_pos[1]], [wrist_pos[2]],[0]]
  
    # Distance from wrist to origin
    D_1 = np.dot(m1_0,wrist)
    
    vector = [D_1[0][0], D_1[1][0], D_1[2][0]]
    D = math.sqrt(math.pow(D_1[0][0],2) + math.pow(D_1[1][0],2) + math.pow(D_1[2][0],2))
    
    # Solving for hypoteneus links 
    hypo_1 = math.hypot(d_3, -a_3)
    hypo_2 = math.hypot(d_5, a_4)
    a = math.acos(a_4/hypo_2) #Check
    b = math.acos(a_3/hypo_1) #Check
    

    C = math.acos(pow(hypo_1, 2) + pow(hypo_2, 2) - pow(D,2)/ 2 * hypo_1 * hypo_2)

    theta_4 = (2 * math.pi) - (a + b + C) - (math.pi/2)
    print(theta_4)


    """ Virtual theta 4 """

    angle_b = math.atan2(d_5, a_4)
    angle_c = angle_b + theta_4
    
    real_d_4 = np.linalg.norm([a_4, d_5])

    wrist_x = d_3 + (math.cos(angle_c) * real_d_4)
    wrist_y = a_4 + (math.sin(angle_c) * real_d_4)

    v_d_5 = np.linalg.norm([wrist_x - d_3 , wrist_y])
    wrist_mag_sqr = pow(wrist_x, 2) + pow(wrist_y, 2)
    v_theta_4 = -(math.acos((math.pow(v_d_5, 2)+ math.pow(d_3, 2) - wrist_mag_sqr) / (2 * v_d_5 * d_3)) - math.pi/2) 
    #print(v_theta_4)

    """ theta_2 """
    phi = math.atan2(vector[1], vector[0])

    theta_2 = phi - math.asin((d_3 + (v_d_5 *math.sin(v_theta_4)))/ math.sqrt(math.pow(vector[1],2) + math.pow(vector[0],2)))

    print(theta_2)

    """ theta_3 """

    m2_1 = calculate_inverse_matrix(0, -math.radians(90), 0, theta_2)
    D_2 = np.dot(m2_1, D_1)
    theta_3 = math.atan2(D_2[1][0], D_2[0][0])
    print(theta_3)

    """ theta_5"""

    m3_2 = calculate_inverse_matrix(0.0825, math.radians(90), 0.384, theta_3)
    m4_3 = calculate_inverse_matrix(0.0825, -math.radians(90), 0 ,(theta_4 + math.radians(90)))

    m2_0 = np.dot(m2_1, m1_0)
    m3_0 = np.dot(m3_2, m2_0)
    m4_0 = np.dot(m4_3, m3_0)
    
    theta_5 = dynamicsymbols('theta_5')
    """
    m5_4 = sp.Matrix([[sp.cos(theta_5),0 , sp.sin(theta_5), 0],
                    [-sp.sin(theta_5), 0, sp.cos(theta_5), 0],
                    [0, -1, 0 ,-0.316 ],
                    [0,0,0,1]])
    m5_0 = m5_4 * m4_0
    """
    

    p_vector = sp.Matrix([[inv_goal_matrix[0][3]], [inv_goal_matrix[1][3]], [inv_goal_matrix[2][3]], [0]])
    
    p_vector_1 = m4_0 * p_vector
    
    theta_5 = math.atan2((-p_vector_1[2]), p_vector_1[0])
    print(theta_5)

    """theta_6"""

    theta_6 = math.acos(-(p_vector_1[1])/ math.sqrt(math.pow(p_vector_1[2],2)+ math.pow(p_vector_1[0],2) + math.pow(p_vector_1[1],2)))
    print(theta_6)




    m5_4 = calculate_inverse_matrix(0, math.radians(90), 0.316, theta_5)
    m6_5 = calculate_inverse_matrix(0, math.radians(90), 0, (theta_6 + math.radians(180)))
    m5_0 = np.dot(m5_4,m4_0)
    m6_0 = np.dot(m6_5,m5_0)
    
    vector_2 = np.linalg.inv(m6_0)[:,0][0:3]

    
    dot_product = np.dot(vector_2, [inv_goal_matrix[0][0], inv_goal_matrix[0][1], inv_goal_matrix[0][2]])
    theta_7 = math.acos(dot_product)
    print(theta_7)

    desired_thetas = [theta_7 + math.pi/2, theta_6, theta_5 + math.pi/2, -theta_4, theta_3, theta_2, theta_1]

    print(desired_thetas)
    
    pub = rospy.Publisher("/joint_position_example_controller_sim/joint_command", Float64MultiArray, queue_size = 1000)

    init_position = [0,-0.5,0,-2.5,0,2,0]

    joint_move_distance = math.radians(30)

    rospy.sleep(2)

    msg = Float64MultiArray()    
    msg.data = init_position
    pub.publish(msg)
    
    rospy.sleep(2)
    msg = Float64MultiArray()
    msg.data = desired_thetas

    pub.publish(msg)
    
    """
    while not rospy.is_shutdown():
        goal_position = []
        delta_angle = joint_move_distance * math.sin(counter/10)
        

        for i in range(len(init_position)):
            if i == 4:
                goal_position.append(init_position[i] - delta_angle)
            else:
                goal_position.append(init_position[i] + delta_angle)

        counter = counter + 1    
        msg = Float64MultiArray()
        msg.data = init_position

        rate.sleep()

        pub.publish(msg)
    """
    