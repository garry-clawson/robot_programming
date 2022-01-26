<!-- 
Author:     Garry Clawson
Date:       26th Jan 2022
Module:     CMP9767M Robotic Programming
Assignment: Assignment 1 - Grapebunch Detection  
Version:    0.1.0  

Comments:
As part of the presentation requirement I will be branching this repo and using the README.md file as a presentation area, with visuals and videos to present my project.  This will be on a FORK called ASSIGN_1_PRESENTATION. I will then merge this into the MAIN afetr the presentation has passed. This will then allow other users to have much more detail about the project. I am noting this because you will see COMMITs to the rep but on a seperate branch.
 -->


## Robot Programming CMP9767M

This repo contains the simuation of a vineyard with the challenge to count all the grape bucnhes across the vines. The specilaistaion in this project is around imaging and coloursegmentation, where a pipeline of tools was used through OpenCV to identy the bunches. 

The navigation for this project uses a homing beacon system, and the BUG2 algorythm to avoid collision and reach certain image points through the vineyard. 

## Set Up Your System

The following are the steps required to run and launch this project. You will need a GitHub account for best management of your files and also be familiar with Git command lines tools and the terminal. You will need Ubuntu 18.04 to run this project.

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


## Route Planning Overview

The path plannnig uses the [BUG2 algorythm](https://automaticaddison.com/the-bug2-algorithm-for-robot-motion-planning/) and a homing beacon system to avoid collisons and get to required points for image taking. 

The algorthym is controlled by being in a series of states `LOOK_TOWARDS`, `GOAL_SEEK`, `WALL_FOLLOW` and `ROTATE_TO_VINES`. 

1. `LOOK_TOWARDS` - This state rotates the robot towards the `HOMING_BEACON` which can be placed anywhere in the vineyard. Rather than use single end point I have used a series of these to act as points to take images at. Once pouinted towards the beacon the robto state will be changed to `GOAL_SEEK`.

1. `GOAL_SEEK` - This moves the robot towards the goal (homing beacon) and if it encounters any obsticle it will change state to `WALL_FOLLOW`. The collison proximity params here are slightly larer than at `WALL_FOLLOW` so we can avoid getting closed into position. 

1. `WALL'_FOLLOW` - This state moves the robot out of a collision area and will keep going until it interscts with the BUG2 `GOAL"_SEEK` line. It will then move to state `LOOK_TOWARDS` to again go to the `HOMING_BEACON`

1. `ROTATE_TO"_VINES` - Once at the goal position, the robot will change state and rotate towards the vines ensuring that the KinectHD camera is facing the vines at 90 degrees (assumes the Vines are infac parrellel to the perimter wall - checks have shown it is)

Once we are at the correct position and angle to take an image the counting process takes over. 


## Counting Process Overview

The grape bunch counting process is achived through an imaging pipeline, using OpenCV, as follows:

- `CV bridge`: This connects OPenCv to ROS
- `cv2.cvtColor(image, cv2.COLOR_BGR2HSV)`: Convert to HSV image, apply thresholds then mask. A useful threshold tool is the [blob_detector.py](https://github.com/tizianofiorenzani/ros_tutorials/blob/master/opencv/include/blob_detector.py) by Tiziano Fiorenzani.
- Repeat tghe above process but remove the green vines aplying a new threshold
- We now have an image with lots of smaller white dots. We need ot rmeove this noise. We used `astype(np.uint8)` to convert to unit8, then `cv2.connectedComponentsWithStats(dummy_image, connectivity=8)` toi build a list of centriods of all white dots, we then remove any that are below 60 pixels. 
- `cv2.dilate(vinemask_updated, np.ones((15, 15)), iterations = 3)` to [increase the size](https://opencv24-python-tutorials.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_morphological_ops/py_morphological_ops.html) of the remaining points. 
- We then use `cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))` to apply an eliptical shape to a following morphologyEx process
-`cv2.morphologyEx(vinemask_updated, cv2.MORPH_OPEN, kernel)` to open the pixels to create larger centroid regions. 

The following stage of the pipeline uses the `cv2.SimpleBlobDetector` to detect the grapes.

- `cv2.copyMakeBorder(grape_bunch_mask, top=1, bottom=1, left=1, right=0, borderType= cv2.BORDER_CONSTANT, value=[255,255,255] )` to add a small border to the top, bottom, left but not the right hand side (see Kinect Caera offset to Vines details for why)
- `cv2.SimpleBlobDetector_Params`, we add the required parameters. Some params are set as default so need ot be adjust to ensure we can identy the shapes of the grape bunches (and not just circles (circularity) for example). These params were disciovered through trial and error across a range of images and lighting conditions, but onlu on 1 x compute reource. Deploying on alternative compute reosurces may require some amendment to these thresholods as well as HSV thresholds). 
- `keypoints = detector.detect(grape_bunch_mask)`, to create a detctor object and identofy keypioints in the image to our previously set params.
- `cv2.drawKeypoints(image, keypoints, np.array([]), (000,000,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)` draws the keypoints onto and image and there is a fucntion withoin the file to save the image to your local directory. 
- The final step is the accumlation of the `keypoints` count. 
- We do this for the images taken across the length fo the vine and sum the total bunches (keypoints identified in each image) found for a total count of grape bunches

### Kinect Camera offset to Vines

The distaance to the vines is correlated to the field of view of the Kinectv2 camera. This infomration can be found in the `src/bacchus_lcas/bacchus_gazebo/urdf/sensors/sensor_kinect_v2_.xaxro` file on line 109. This gives us a value of `<horizontal_fov>${84.1*3.14/180.0}</horizontal_fov>` which is around 1.467 rads for the viewing angle. The length of the vine hedge in the 'world_name:=vineyard_small' is 12 meters (found from visual inspection. At 2m from the hedge we will image 3.60794m of the hedge with the available field of view. Taking an image of the full vine hedge at 2m distance therfore requires 12 / 3.61 = 3.32 images. or rather the last image will only have 1/3rd of the vie hedge on (if we take an image directly at the start of the hedge with the RHS at the border of the image)

The images are taken form right through left. As part of the process we use `cv2.SimpleBlobDetector`. The simple bob detector ignores keypoints on the boundarty so a broder was created with the right side left off (so anythign on the RHS would nto be counted if it fell on the border). As the robot moves down the vine row anything missed out on the RHS is captured on the LHS. This avoids the issue fo double counting boundary grape bunches; ,however robot distanmce to hedge and camera FoV shoudl be accurate to get sesnible reust without double counting.

