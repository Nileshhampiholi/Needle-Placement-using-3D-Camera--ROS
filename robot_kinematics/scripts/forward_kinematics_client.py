#!/usr/bin/env python
import rospy
import numpy as np
from robot_kinematics.srv import forward_kinematics_server
from robot_kinematics.srv import forward_kinematics_serverRequest
from robot_kinematics.srv import forward_kinematics_serverResponse

from sensor_msgs.msg import JointState

def compute_forward_kinematics_client(position):
    try:
        compute_kinematics = rospy.ServiceProxy('compute_forward_kinematics_service', forward_kinematics_server)
        current_jacobian= compute_kinematics(position)
        return current_jacobian

        
    except rospy.ServiceException as e:
        print ("Service call failed: %s" %e)

def callback(message):
    position  = message.position
    current_jacobian =compute_forward_kinematics_client(position)
    jacobian  = np.array(current_jacobian)
    print (jacobian)
    

def get_current_position():
    rospy.init_node('get_current_position', anonymous=True)
    rospy.wait_for_service('compute_forward_kinematics_service')
    topic = "/joint_states"
    rospy.Subscriber(topic, JointState, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    get_current_position()
    
    