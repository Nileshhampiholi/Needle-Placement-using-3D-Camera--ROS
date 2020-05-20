#!/usr/bin/env python

import rospy
import math
from franka_msgs.srv import cartesianfrompose
from sensor_msgs.msg import JointState

def theta_values(msg):

    try:
        transformation = pose_to_cartesian(msg.position)
        print(transformation)
    except rospy.ServiceException as e:
        rospy.logwarn("Service failed: " + str(e))
        pass



   
if __name__ == '__main__':
    
    
    rospy.init_node('fow_kin_client')

    rospy.wait_for_service("/pose_to_cartesian")

    pose_to_cartesian = rospy.ServiceProxy("/pose_to_cartesian", cartesianfrompose)

    try:
        sub = rospy.Subscriber('/joint_states', JointState, theta_values)
        rospy.spin()
    except rospy.ROSException as e:
        rospy.logwarn("Node failed : " + str(e))

