#!/usr/bin/env python

# Inspired by --> https://github.com/bipul93/ros-bug2/blob/master/scripts/bot.py

import rospy, sys
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
import enum
import math
import numpy
#import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from datetime import datetime
import cv2
from cv2 import namedWindow, cvtColor, imshow, inRange

from subprocess import Popen


# --------------------------------------------- define variables and initialise --------------------------------------

# Defining Bot States
# We move between these states before we get to our desrired location depending on vineyard and obsticles
class BotState(enum.Enum):
    LOOK_TOWARDS = 0  # rotate bots towards the goal
    GOAL_SEEK = 1  # follow line
    WALL_FOLLOW = 2  # Go around the wall / avoid obstacles
    ROTATE_TO_VINES = 3 # Rotate towards vines to take camera image
    TAKE_IMAGE = 4 # Take images along the vines

# Initialised values
yaw = 0
yaw_threshold = math.pi / 90 #Set at 90 as this gives 2 degrees tolerence I found due to size of vineyard and testing at far edges 4 degs missed homing spots (could moke homing spot larger (line distance value)? - pros and cons)
goal_distance_threshold = 0.5
currentBotState = BotState.LOOK_TOWARDS

# base scan laser range values
maxRange = 3
minRange = 0
bot_pose = None
init_bot_pose = []
beacon_pose = None
bot_motion = None  # cmd_vel publisher
homing_signal = None  # subscriber
init_config_complete = False
wall_hit_point = None
beacon_found = False
taken_image = False
twist = Twist()
distance_moved = 0
front_obs_distance = None
left_obs_distance = None
wall_following = False

#Rotation vairables
#Target for rotation and smoothing speed (kp) used to slow down the rotation the closer we get to our target
target = -90 # Target angle to achive ibn degrees (Note: this is the world view and is directly facuing the grape vines)
kp=0.5 # Slows the angle of rotation the closer you get to the desried angle (stops from overshooting)

# Image couting variables
image_count = 0

# --------------------------------------------- movement functions --------------------------------------


# Looks towards the homing beacon position
def look_towards(des_pos):
    global yaw, yaw_threshold, bot_motion, currentBotState, twist
    quaternion = (
        des_pos.orientation.x,
        des_pos.orientation.y,
        des_pos.orientation.z,
        des_pos.orientation.w)
    euler = euler_from_quaternion(quaternion)
    yaw = euler[2]  # bot's yaw
    beacon_yaw = math.atan2(beacon_pose.position.y - des_pos.position.y, beacon_pose.position.x - des_pos.position.x)
    yaw_diff = normalize(beacon_yaw - yaw)

    # math.fabs = returns absolute value of a number as a float
    if math.fabs(yaw_diff) > yaw_threshold:
        print("Rads to get to beacon", math.fabs(yaw_diff))
        twist.angular.z = -0.2 # clockwise rotation if yaw_diff > 0 else 0.5  # counter-clockwise rotation

    if math.fabs(yaw_diff) <= yaw_threshold:
        twist.angular.z = 0
        currentBotState = BotState.GOAL_SEEK
    bot_motion.publish(twist)

# Seeks out the homing beacon and if comes into contact with an obstrucrtion envokes the wall_follow() function (algroythm used is BUG2)
def goal_seek():
    print("goal seek initilised")
    global zone_F, zone_FL, zone_FR, currentBotState, bot_pose, wall_hit_point, front_obs_distance, left_obs_distance

    # Comment:
    # UIse 1.5 distance here as this allows us to move around the wall with a 2m offset but then gives us change to get closer
    # once we are in a line distance position. 1m is the lowest  and with trials the robtos corners caught on the hedge
    # when the becaon is positiohned very close to it (just to see hwo close we can get and also trying to tune the parameters)
    obstacle_in_front = numpy.any((zone_F < 1.5)) or numpy.any((zone_FR < 1.5)) or numpy.any((zone_FL < 1.5)) #Note that 1.5 is 
    # Or find the minimum value in this zone. or maybe numpy.any would be faster
    print("obsticle in front?", obstacle_in_front)
    if obstacle_in_front:
        twist.linear.x = 0
        wall_hit_point = bot_pose.position
        currentBotState = BotState.WALL_FOLLOW
    else:
        twist.angular.z = 0
        twist.linear.x = 1
    bot_motion.publish(twist)

# Comment:
# Avoids obsticales and follows walls to the BUG2 algorythm using the beacon as the target to get too
# Depending on the rottaion of the vines this maybe useful to spawn the robot perpendicular to the vines and have the target
# point behind them. This way the robot will navigate fully around each vine row 
# After though - BUG1 would have been better here as it would fully encompas each vine hedge befor going to its nearest jump off point
# as it moves ot its target (beacon) point - should have thought about that!! 

def wall_follow():
    print("wall follow initilised")
    global twist, bot_pose, bot_motion, currentBotState, distance_moved, wall_hit_point

    # Comment: 
    # The distance '<2' should be at least as large as if the robot was inscribed in a circle
    # In Thorvalds instance its rectnagluar in shape so the enscribed circle when rotating about its centre point + some tolerance
    obstacle_in_front = numpy.any((zone_F < 2)) or numpy.any((zone_FR < 2)) or numpy.any((zone_FL < 2))
    distance_moved = math.sqrt(pow(bot_pose.position.y - wall_hit_point.y, 2) + pow(bot_pose.position.x - wall_hit_point.x, 2))
    #print(line_distance(), distance_moved, (line_distance() < 0.2 and distance_moved > 0.5))


    if line_distance() < 0.2 and distance_moved > 0.5:
        print("line_hit")
        print(distance_moved)
        # found line point. rotate and move forward via the LOOK_TOWARDS state
        twist.angular.z = 0
        twist.linear.x = 0
        currentBotState = BotState.LOOK_TOWARDS
        return
    elif numpy.any((zone_F < 2)) or numpy.any((zone_FR < 2)):  # turn left
        print("turn right")
        twist.angular.z = 1
        twist.linear.x = 0
    elif numpy.any((zone_F < 2)) or numpy.any((zone_FL < 2)):  # turn right
        print("turn right")
        twist.angular.z = -1
        twist.linear.x = 0
    else:
        print("move forward")
        twist.angular.z = 0  # move forward
        twist.linear.x = 0.5
    bot_motion.publish(twist)
    

# Rotates to face the vines according tpo the world view (-90 degrees). This uses the front IKinect HD camera but couuld have also used left or right 
# hand camera here
def rotate_to_vines():
    print("rotating to vines")
    global bot_motion, twist, bot_pose, target, currentBotState, yaw_threshold, facing_vines
    #bot_motion = rospy.Publisher("/thorvald_001/teleop_joy/cmd_vel", Twist, queue_size=10)
    target_rad = target*math.pi/180 #-90 degrees to face towards vines
    #print("bot pose", bot_pose)
    # Change bot pose from Quarternion to euler to get the yaw and difference to target (-90 degrees)
    quaternion = (
        bot_pose.orientation.x,
        bot_pose.orientation.y,
        bot_pose.orientation.z,
        bot_pose.orientation.w)
    euler = euler_from_quaternion(quaternion)
    yaw_ = euler[2]  # bot's yaw
    print("yaw", yaw_)

    if math.fabs(target_rad-yaw_) > yaw_threshold:
        print("bot pose", bot_pose)
        print("target_rad", target_rad)
        twist.linear.x = 0.0
        twist.angular.z = kp * (target_rad-yaw_)
    else:
        twist.angular.z = 0
        print("VINES TRUE")
        currentBotState = BotState.TAKE_IMAGE
        return

    bot_motion.publish(twist)

    

        

# --------------------------------------------- helper functions --------------------------------------


# Angles are from 180 to -180 so nneed to nrolaise tio this rather than 320 etc
def normalize(angle):
    if math.fabs(angle) > math.pi:
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

# distance between a point and a line - right angles to the line
# When following the line we need ot know the shortest distance to jump ioff from (BUG2 algorythem -> https://automaticaddison.com/the-bug2-algorithm-for-robot-motion-planning/ )
def line_distance():
    global init_bot_pose, beacon_pose, bot_pose
    point_1 = init_bot_pose  # in form of array
    point_2 = beacon_pose.position
    point_k = bot_pose.position
    numerator = math.fabs((point_2.y - point_1[1]) * point_k.x - (point_2.x - point_1[0]) * point_k.y + (point_2.x * point_1[1]) - (point_2.y * point_1[0]))
    denominator = math.sqrt(pow(point_2.y - point_1[1], 2) + pow(point_2.x - point_1[0], 2))
    return numerator / denominator


def callback(msg):
    global beacon_pose
    beacon_pose = msg.pose
    check_init_config()
    rospy.wait_for_message("homing_signal", PoseStamped)

#def callback_2(msg):
#    global beacon_pose
#    beacon_pose = msg.pose
#    check_init_config()
#    rospy.wait_for_message("homing_signal_2", PoseStamped)

# Bot pose position relative to homing beacon
def get_base_truth(bot_data):
    global bot_pose, beacon_found, goal_distance_threshold, currentBotState
    bot_pose = bot_data.pose.pose
    if not init_config_complete:
        check_init_config()

    if beacon_pose is not None:
        goal_distance = math.sqrt(pow(bot_pose.position.y - beacon_pose.position.y, 2) + pow(bot_pose.position.x - beacon_pose.position.x, 2))
        # print(goal_distance)
        if goal_distance <= goal_distance_threshold:
            currentBotState = BotState.ROTATE_TO_VINES
            beacon_found = True
            
# Takes subscriber infor from front scan and assigns field of view zones
def process_sensor_info(data):
    global maxRange, minRange, front_obs_distance, left_obs_distance, zone_R, zone_FR, zone_F, zone_FL, zone_L
    
    # Testing print outs
    #maxRange = data.range_max
    #print("maxRange", maxRange)
    #minRange = data.range_min
    #print("minRange", minRange)


    # Comment:
    # The range split is currently at uneven angles. We could use >> 'zone = numpy.array_split(numpy.array(data.ranges), 5)' and split into 5 equal zones?
    # However, there are 720 data.ranges. This is defined in the sensors URDF folder for the Hokuya camera (bacchus_sensors.xacro file - line 29,30), which
    # holds the args that are pulled through via the launch file. These can be changed to: min_angle="-0.7854" and max_angle="2.3562" respectivly
    # The front camera on the Thorvald is offset by 45 degrees so doesn't detect within a range - I think this is because if you set the range from
    # -180 to 180, you have a gap at between the front and back camera (must be because the cameras are mounted back to back and obviously not ontop of one another
    # For me to use data ranges I will need to amend the front sensor URDF folder to adjust for the 45 degree offset so the front zones are actually the front laser scans
    # A benefit to splitting the laserscan into zones is you can get a narrow field of view if going down tight tunnels with larger forward zone and smaller side zones
    # However, a downside I have seen through testing is that the robot can get caught looping around the same 'avoid' route - never crashing but never taking a risk either! Come on Thorvald!!

    zone = numpy.array(data.ranges) 
    zone_R = zone[0:143] 
    zone_FR = zone[144:287]
    zone_F = zone[288:431]
    zone_FL = zone[432:575]
    zone_L = zone[576:719]

    if front_obs_distance is None and left_obs_distance is None:
        front_obs_distance = 2
        left_obs_distance = 2


# --------------------------------------------- main while loop for robot --------------------------------------

def check_init_config():
    global bot_pose, beacon_pose, init_config_complete, init_bot_pose
    if bot_pose is not None and beacon_pose is not None:
        init_config_complete = True
        init_bot_pose = [bot_pose.position.x, bot_pose.position.y]
        bot_bug2()

# Comment:
# While loop to keep checkuing if we have hit homing position and are facing the vines to takje an image
# Shifts between states depending upon issues we see - mainly managing obsticles via wall following
def bot_bug2():
    global bot_motion, currentBotState, bot_pose
    bot_motion = rospy.Publisher("/thorvald_001/teleop_joy/cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(20)
    while not taken_image:
        if not init_config_complete:
            return
        if currentBotState is BotState.LOOK_TOWARDS:
            print("look towards")
            look_towards(bot_pose)
        elif currentBotState is BotState.GOAL_SEEK:
            print("Goal Seek")
            goal_seek()
        elif currentBotState is BotState.WALL_FOLLOW:
            print("Wall Follow")
            wall_follow()
        elif currentBotState is BotState.ROTATE_TO_VINES:
            print("Rotate to Vines")
            rotate_to_vines()
        elif currentBotState is BotState.TAKE_IMAGE:
            print("Taking Image")
            image_listener()
            return
        rate.sleep()
    print("Image captured")





# --------------------------------------------- ported imaging class and functions --------------------------------------

class image_listener:
    print("entered listener")

    def __init__(self):
        # Enable OpenCV with ROS
        self.bridge = CvBridge()
        # Subscribe to front camera feed
        self.image_sub = rospy.Subscriber("/thorvald_001/kinect2_front_camera/hd/image_color_rect",
                                          Image, self.image_callback)

    def image_callback(self, data):

        global image_count #Counts the number of images taken
        print("Received an image!")

        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print(e)
        else:
            img_noBackground = self.removeBackGround(cv2_img) # remove background on start
            img_removeVines = self.removeVines(img_noBackground)
            image_count = image_count + 1
            print("Count of images", image_count)
            #self.saveImage(img_removeVines)


    def removeBackGround(self, image):
        # Remove background
        HSVimage = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) # convert color space
        #cv2.imshow("initial image", HSVimage)
        #cv2.waitKey(0) 
        #self.saveImage(HSVimage)
        # between values for thresholding
        min = numpy.array([35, 000, 000]) 
        max = numpy.array([180, 253, 255]) 
        mask = cv2.inRange(HSVimage, min, max) # threshold
        bunch_image = cv2.bitwise_and(HSVimage, HSVimage, mask=mask) # obtain threshold result
        im_NoBackground = cv2.cvtColor(bunch_image, cv2.COLOR_HSV2BGR) # reconvert color space for publishing
        #cv2.imshow("Removed background", im_NoBackground)
        #cv2.waitKey(0) 
        return im_NoBackground


    def removeVines(self, image):
        HSVimage = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)   # convert color space for thresholding
        #cv2.imshow("HSVImage", HSVimage)
        #cv2.waitKey(0) 
        # mask out vine - values found by using hsv_range_detector.py
        # inspired by https://www.youtube.com/watch?v=We6CQHhhOFo&t=136s  -> ROS and OpenCv for beginners | Blob Tracking and Ball Chasing with Raspberry Pi by Tiziano Fiorenzani
        min = numpy.array([90, 000, 40])
        max = numpy.array([255, 255, 255]) 
        vinemask = cv2.inRange(HSVimage, min, max) # threshold
        #cv2.imshow("vinemask", vinemask)
        #cv2.waitKey(0) 
        #self.saveImage(vinemask)

        # Remove odd small spots
        # Inspired by -> https://stackoverflow.com/a/42812226
        dummy_image = vinemask.astype(numpy.uint8) # reconvert to uint8
        #find all your connected components (white blobs in your image)
        nb_components, output, stats, centroids = cv2.connectedComponentsWithStats(dummy_image, connectivity=8)
        #connectedComponentswithStats yields every seperated component with information on each of them, such as size
        #the following part is just taking out the background which is also considered a component, but most of the time we don't want that.
        sizes = stats[1:, -1]; nb_components = nb_components - 1
        # minimum size of particles we want to keep (number of pixels)
        #here, it's a fixed value, but you can set it as you want, eg the mean of the sizes or whatever
        min_size = 60 # Value found through trial and error - since we are donig this pre dilation we need ot pick up smaller elements
        #answer image
        vinemask_updated = numpy.zeros((output.shape))
        #for every component in the image, you keep it only if it's above min_size
        for i in range(0, nb_components):
            if sizes[i] >= min_size:
                vinemask_updated[output == i + 1] = 255

        vinemask_updated = vinemask_updated.astype(numpy.uint8) # reconvert to uint8
        #cv2.imshow("vinemask updated", vinemask_updated)
        #cv2.waitKey(0) 

        # Increase size of remaining pixels
        vinemask_updated = cv2.dilate(vinemask_updated, numpy.ones((15, 15)), iterations = 1) # expand mask
        #cv2.imshow("diliated", vinemask_updated)
        #cv2.waitKey(0) 

        # Add kernal to complete the morphEx operation using morph_elispse (simular shape to grapes)
        # Inspired by -> https://www.pyimagesearch.com/2021/04/28/opencv-morphological-operations/
	    # construct a eliptic kernel (same shape as grapes) from the current size and then apply an "opening" operation to close the gaps
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        morph_vinemask = cv2.morphologyEx(vinemask_updated, cv2.MORPH_OPEN, kernel)
        #cv2.imshow("MorphEx vinemask", morph_vinemask)
        #cv2.waitKey(0)

        # obtain threshold result
        grapeBunchImage = cv2.bitwise_and(HSVimage, HSVimage, mask=morph_vinemask) 
        #cv2.imshow("grapeBunchImage with MorphEx ANDED HSV image", grapeBunchImage)
        #cv2.waitKey(0) 

        # Detect the keypoints of the grape bunches in the image and count them
        im_detectGrapes_with_keypoints, keypoints = self.detectGrapes(grapeBunchImage, morph_vinemask)
        #cv2.imshow("Final Grape bunch Image", im_detectGrapes_with_keypoints)
        #cv2.waitKey(0)
        #self.saveImage(im_detectGrapes_with_keypoints)
        
        return im_detectGrapes_with_keypoints


    def detectGrapes(self, image, mask):
        global taken_image

        grape_bunch_mask=cv2.bitwise_not(mask) # invert as blob detector will look for black pixels, ours is white

        # create the small border around the image. As the robot will move forwards down the row then don't catch the right border
        # because the the nexct image the left border will catch any overlap and register it (hopefully!!)
        # If the robot is too close this will work and if far enough away the border wont be required top/bottom
        # Inspired by -> https://stackoverflow.com/questions/53064534/simple-blob-detector-does-not-detect-blobs
        grape_bunch_mask=cv2.copyMakeBorder(grape_bunch_mask, top=1, bottom=1, left=1, right=0, borderType= cv2.BORDER_CONSTANT, value=[255,255,255] ) 

        # FROM -> https://www.learnopencv.com/blob-detection-using-opencv-python-c/
        # Inspired params from -> https://stackoverflow.com/questions/53064534/simple-blob-detector-does-not-detect-blobs 
        # Note: Need the robot to sdand off so that grape bunches are at the image border - this impacts pixel masking params also!!
        params = cv2.SimpleBlobDetector_Params() # initialize detection parameters
        # Inspired by -> https://programmerall.com/article/3089974703/  
        params.maxArea = 100000
        params.minInertiaRatio = 0.05
        params.minConvexity = .60
        # Create a detector with the parameters
        detector = cv2.SimpleBlobDetector_create(params)
        # Detect blobs.
        keypoints = detector.detect(grape_bunch_mask)
        # Draw detected blobs as red circles. cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(image, keypoints, numpy.array([]), (000,000,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        #Counts the number of keypoints (grape bunches) in image
        grape_bunches_in_image = len(keypoints)
        if(grape_bunches_in_image != 0): # If a blob is detected, print out how many
            print('Keypoints = ',grape_bunches_in_image)
        #cv2.imshow("detect keypoints image", im_with_keypoints)
        #cv2.waitKey(0)
        taken_image = True # flags that we have now taken an image
        return im_with_keypoints, keypoints

    def saveImage(self, image):
        # Save your OpenCV2 image as a jpeg 
        time = datetime.now()
        filepath = 'grape_bunches'+str(time)+'.jpg' 
        print('saving to ',filepath)
        cv2.imwrite(filepath, image)
        #imshow("cv2", image)
        rospy.sleep(1)








# --------------------------------------------- main program entry --------------------------------------

def init():
    global homing_signal
    rospy.init_node("navigation")

    homing_signal = rospy.Subscriber('/homing_signal', PoseStamped, callback)

    #homing_signal_2 = rospy.Subscriber('/homing_signal_2', PoseStamped, callback_2)

    rospy.Subscriber('/thorvald_001/front_scan', LaserScan, process_sensor_info)
    rospy.Subscriber('/thorvald_001/odometry/base_raw', Odometry, get_base_truth)

    print('--------- navigation has started -----------')
    rospy.spin()


if __name__ == '__main__':
    try:
        init()
       
    except rospy.ROSInterruptException:
        pass