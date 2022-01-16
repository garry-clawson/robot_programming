# robot_programming

These files go in the src folder of your catkin_ws or ros_ws. My folder is currenty called robot_programming.

To launch the robot, type the following in the terminal (note: using roslaunch instead of rosrun will launch roscore automatically):

`roslaunch rp_assignment_1 garry_mover.launch`

This will call the package and the associated laucnh file. This launch file consists of two simple elements:

1) The node details for the garry_mover.py

2) An <include> which finds the bacchus_gazebo vineyard demo launch

To launch more nodes we can include them in the launch file and call their respective packages and python file.
