# robot_programming

These files go in the src folder of your catkin_ws or ros_ws. My folder is currenty called robot_programming.

To launch the robot. Do the following:

`roslaunch rp_assignment_1 garry_mover.launch`

This will call the package and the associated laucnh file. This launch file consists of two simple elements:

1) the node details for the garry_mover.py

2) an <include> which finds the bacchus_gazebo vineyard demo launch
