#!/usr/bin/env python

import rospy 
from sensor_msgs.msg import JointState

def states_callback(msg):

    print(msg.position)

if __name__ == '__main__':

    rospy.init_node('panda_joint_reader')
    topic_name = rospy.get_param("panda_joint_reader/topic_name")
    queue_size = rospy.get_param("panda_joint_reader/queue_size")
    rospy.Subscriber(topic_name, JointState, states_callback)
    rospy.spin()
