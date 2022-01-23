o# robot_programming

These files go in the src folder of your catkin_ws or ros_ws. My folder is currenty called robot_programming.

To launch the robot, type the following in the terminal (note: using roslaunch instead of rosrun will launch roscore automatically):

`roslaunch grape_bunch_counter grape_bunch_counter.launch`

This will call the package and the associated laucnh file. This launch file consists of two simple elements:

1) The node details for the robot_move.py

2) An <include> which finds the bacchus_gazebo vineyard demo launch

To launch more nodes we can include them in the launch file and call their respective packages and python file.


## Update Sensor profiles

The front and back sensor is use the hokuya laser. This is set to min and max capability and gives an uneven range around the robot.
Upodate the sensor_hokuyo_laser.xacro file found in src/bacchus_lcas/bacchus_gazebo/urdf/bacchus_sensors.xaxro and change line 21, 22 and 29,30 to:

`min_angle="0.7854"`
`max_angle="2.3562"`

Also update the sensor_hokuyo_laser.xacro file in src./bacchus_lcas/bacchus_gazebo/urdf/sensors folder and change line 54 to:

`<samples>180</samples>`