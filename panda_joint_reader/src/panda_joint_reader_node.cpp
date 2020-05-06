#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <std_msgs/String.h>
#include <iostream>
#include <array>



void states_callback(const sensor_msgs::JointState::ConstPtr& msg) {

    for(int i=0 ; i < msg->position.size(); i++)
    {
        ROS_INFO_STREAM(" position of joint " << i+1 << " is "<< msg->position[i]);
    }
}

int main(int argc, char **argv)
{
  // ROS objects
  ros::init(argc, argv, "panda_joint_reader_node");
  ros::NodeHandle nh;
  std::string topic_name;
  int queue_size;

  ros::param::get("panda_joint_reader/topic_name", topic_name);
  ros::param::get("panda_joint_reader/queue_size", queue_size);
  

  ros::Subscriber sub = nh.subscribe(topic_name, queue_size, states_callback);

  ros::spin();

}
