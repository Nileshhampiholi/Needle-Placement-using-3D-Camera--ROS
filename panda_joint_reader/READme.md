Editor- Yash Shah, Nilesh Hampiholi

To start forward kinematics. Pull the code from master branch. I would suggest to delete packages of the franka panda robot and pull all the packages once again, because I have not only made changes in panda_joint_reader package but franka_msgs also. Once you setup everything from the master branch

type this commands in terminal

1. cd catkin_ws (or the workspace in which you are working)

2. catkin_make

3. source devel/setup.bash

3.1 chmod +x src/Franka_Panda/panda_joint_reader/scripts/fow_kin_server.py

3.2 chmod +x src/Franka_Panda/panda_joint_reader/scripts/fow_kin_client.py

4. roslaunch franka_example_controllers joint_position_example_controller_sim.launch

    New Terminal
    
5. roslaunch franka_example_controllers test_move_node.launch

    New Terminal
    
6. roslaunch panda_joint_reader fow_kin.launch

You can see in the terminal the cartesian coordinates of the end effector

If you find any difficulties feel free to contact me.
