ur5_RTDE_controller

To start the "server": rosrun ur5_RTDE_controller Ros_node.py

3 functions are exposed through ROS service

- StartPath: if a path is already set, start the execution of it.

- StopPath: if a path is being executed, stop the movement of the robot

- SetNewPath: set a new path for the robot by defining goto poses in a csv.
setNewPath takes the csv filepath as a parameter

- SetTransformPose: set a new pose to transform the actual tcp pose from->to

RTDE data can be access through the /UR_RTDE_Info topic

TODO: 
- add the transformation for speed and force
- be able to set the speed
- Add blending function for path
- Add path planning options
- probably other stuff I forgot...