ur5_RTDE_controller

To start the "server": rosrun ur5_RTDE_controller Ros_node.py

3 functions are exposed through ROS service

- startPath: if a path is already set, start the execution of it.

- stopPath: if a path is being executed, stop the movement of the robot

- setNewPath: set a new path for the robot by defining goto poses in a csv.
setNewPath takes the csv filepath as a parameter

RTDE data can be access through the /UR_RTDE_Info topic

TODO: 
- add the transformation to a specific frame for the data record
- be able to set the speed
- make the program close properly
- Add blending function for path
- Add path planning options
- probably other stuff I forgot...