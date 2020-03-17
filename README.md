## Moco Control ##

Use Motive Moco controller as ROS Joint Controller

### Requirements ###
sudo apt install ros-melodic-ros-control ros-melodic-ros-controllers

### Launch RViz demo ###
roslaunch msa6_moveit_config demo.launch

### Launch hardware demo ###

roslaunch moco_control moco_hardware.launch moco_chain:=MSA6_1002    (for example)

roslaunch msa6_moveit_config move_group.launch
