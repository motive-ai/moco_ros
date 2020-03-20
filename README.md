## Moco Control ##

Use Motive Moco controller as ROS Joint Controller

### Requirements ###
sudo apt install ros-melodic-ros-control ros-melodic-ros-controllers

### Launch RViz demo ###
roslaunch msa6_moveit_config demo.launch

### Launch hardware demo ###

roslaunch moco_control moco_hardware.launch moco_chain:=MSA6_1002    (for example)

roslaunch msa6_moveit_config move_group.launch

rviz    (optional, to visualize the robot)

### To jog a joint
Clone and build the moveit_jog_arm package (https://github.com/ros-planning/moveit/tree/master/moveit_experimental/moveit_jog_arm).

roslaunch moco_jog jog_example.launch

rostopic pub -s -r 250 /jog_server/delta_jog_cmds geometry_msgs/TwistStamped "header: auto
twist: 
  linear: 
    x: 0.0
    y: 0.0
    z: 0.0
  angular: 
    x: 0.0
    y: 0.0
    z: 0.4"
