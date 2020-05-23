editior Nilesh Hampiholi, Yash Shah

To use this package and the nodes run following commands 

Make sure the python nodes in the robot_kinematics/scripts are make excutables.
Pefrom catkin_make


1. roslaunch franka_example_controllers joint_position_example_controller_sim.launch
2. rosrun franka_example_controllers test_move_node
3. rosrun robot_kinematics forward_kinematics_service.py
4. rosrun robot_kinematics forward_kinematics_client.py 


forward_kinematics_service starts the service to calculate kinematic and return trasfromation matrices, quaternions,
roll_pitch_yaw and cartesian cordinates in the mentiones order. 

forward_kinematics_client subscribes to joint_states topic and receivces 7 joint positions and client calls the service to calculate kinematics 
and publish as a custom message defined in the msg folder. 