#!/usr/bin/env python
import rospy

from robot_kinematics.srv import forward_kinematics_server
from robot_kinematics.srv import forward_kinematics_serverRequest
from robot_kinematics.srv import forward_kinematics_serverResponse

from sensor_msgs.msg import JointState
from robot_kinematics.msg import kinematics_msgs



def compute_forward_kinematics_client(position):
    rospy.wait_for_service('compute_forward_kinematics_service')
    try:
        compute_kinematics = rospy.ServiceProxy('compute_forward_kinematics_service', forward_kinematics_server)
        current_position = compute_kinematics(position)
        return current_position
    except rospy.ServiceException as e:
        print ("Service call failed: %s" %e)

def callback(message):
    position  = message.position
    #print(position)
    current_position = compute_forward_kinematics_client(position)
    print " Trasnformation matrix   =  %s"%( current_position)



def get_current_position():
    rospy.init_node('get_current_position', anonymous=True)
    topic = "/joint_states"
    rospy.Subscriber(topic, JointState, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    get_current_position()