## robot_programming

## Set Up Your System

The following denotes the steps required to run and launch this project. You will need a GitHub account for best management of your files and also be familiar with Git command lines tools and the terminal. You will need Ubuntu 18.04 tio run this project.

1. Fork the `LCAS/CMP9767M` project from [https://github.com/LCAS/CMP9767M](https://github.com/LCAS/CMP9767M) as described in the [LCAS Wiki](https://github.com/LCAS/CMP9767M/wiki/Workshop-1---Introduction-and-ROS-Basics)

1. Create a catkin_ws using using the fantastic [ROS catkin tutorials](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

1. Copy the `LCAS/CMP9767M` project fron step into the `/src` folder of your new workspace

1. Fork the contents of the [robot_programming](https://github.com/garry-clawson/robot_programming) repo and clone into the `/src` folder within your `catkin_ws`

1. Now you have all the files you require to run the project. We need to complete `catkin_make` process. To do this `roscd` then `cd ..` out of the `devel` folder. You should now be in the `catkin_ws` folder.  Use `catkin_make` to build your project.


### Update Sensor profiles

The front and back sensor is use the hokuya laser. This is set to min and max capability and gives an uneven range around the robot, esepcially if you devide the sesnor laserscan into zoens for LEFT, LEFTFRONT, FRONT etc.
Update the `sensor_hokuyo_laser.xacro` file found in `src/bacchus_lcas/bacchus_gazebo/urdf/bacchus_sensors.xaxro` and change line 21, 22 and 29,30 to:

`min_angle="-0.7854"`
`max_angle="2.3562"`

This will update the laser to scan across a -180 to 180 field of view. You will still have 720 data points (now more closely aligned) to work with. This could play to our advantage as we get a densor cluster of scans for analysis.

## How to launch the project

To launch the robot, type the following in the terminal (note: using roslaunch instead of rosrun will launch roscore automatically):

`roslaunch grape_bunch_counter grape_bunch_counter.launch`

This will call the package and the associated launch files. This launch file consists of two simple components along with additional launch parameters:

1) The node details for the relevent .py files

2) An <include> which finds the bacchus_gazebo vineyard demo launch (launches the vineyard_small as default)

To launch more nodes we can include them in the launch file or call their respective packages and python files throughout the simulation.


## Counting Process Overview

The grape bunch counting process is very simple. We first get into position, which is provided by the homing beacon. This puts us infront of the vines ready for an image to be taken. The distaance to the vines is correlated to the field of view of the Kinectv2 camera. This infomration can be found in the `src/bacchus_lcas/bacchus_gazebo/urdf/sensors/sensor_kinect_v2_.xaxro` file on line 109. This gives us a value of xxx which is around yyy for the viewing angle. The length of the vine hedge in the 'world_name:=vineyard_small' is 12 meters (found form visula inspection. Taking an image of the full vine hedge therfore requires 12 m / yyy. 

The images are taken form right through left. As part of the process we use `cv2.SimpleBlobDetector`. The simple bob detector ignores keypoints on the boundarty so a broder was created with the right side left off (so anythign on the RHS would nto be counted if it fell on the border). As the robot moves down the vine row anything missed out on the RHS is captured on the LHS. This avoids the issue fo double counting boundary grape bunches; ,however robot distanmce to hedge and camera FoV shoudl be accurate to get sesnible reust without double counting.

