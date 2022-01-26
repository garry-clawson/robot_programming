#!/usr/bin/env python

# Inspired by --> https://github.com/bipul93/ros-bug2/blob/master/scripts/bot.py

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
import enum
import math
import numpy

# --------------------------------------------- define variables --------------------------------------

# Defining Bot States
# We move between these states before we get to our desrired location depending on vineyard and obsticles
class BotState(enum.Enum):
    LOOK_TOWARDS = 0  # rotate bots towards the goal
    GOAL_SEEK = 1  # follow line
    WALL_FOLLOW = 2  # Go around the wall / avoid obstacles
    ROTATE_TO_VINES = 3 # Rotate towards vines to take camera image

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
facing_vines = False
twist = Twist()
distance_moved = 0
front_obs_distance = None
left_obs_distance = None
wall_following = False
#Target for rotation and smoothing speed (kp) used to slow down the rotation the closer we get to our target
target = -90 # Target angle to achive ibn degrees (Note: this is the world view and is directly facuing the grape vines)
kp=0.5 # Slows the angle of rotation the closer you get to the desried angle (stops from overshooting)


# --------------------------------------------- called and helper functions --------------------------------------


# Angles are from 180 to -180 so nneed to nrolaise tio this rather than 320 etc
def normalize(angle):
    if math.fabs(angle) > math.pi:
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

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
        twist.angular.z = -0.2  # clockwise rotation if yaw_diff > 0 else 0.5  # counter-clockwise rotation

    if math.fabs(yaw_diff) <= yaw_threshold:
        twist.angular.z = 0
        currentBotState = BotState.GOAL_SEEK
    bot_motion.publish(twist)

# Seeks out the homing beacon and if comes into contact with an obstrucrtion envokes the wall_follow() function (algroythm used is BUG2)
def goal_seek():
    global zone_F, zone_FL, zone_FR, currentBotState, bot_pose, wall_hit_point, front_obs_distance, left_obs_distance
    # zone_F = numpy.array(zone_F)
    obstacle_in_front = numpy.any((zone_F < 2))
    # Or find the minimum value in this zone. or maybe numpy.any would be faster
    print(obstacle_in_front, zone_F)
    if obstacle_in_front:
        twist.linear.x = 0
        wall_hit_point = bot_pose.position
        currentBotState = BotState.WALL_FOLLOW
    else:
        twist.angular.z = 0
        twist.linear.x = 0.5
    bot_motion.publish(twist)


# Follows any obstricle in the fashion of BUG2 using the becaon as the target to get too
# Depending on the rottaion of the vines this maybe useful to spawn the robot perpendicular to the vines and have the target
# point behind them. This way the robot will navigate fully around each vine row 
# After though - BUG1 would have been better here as it would fully encompas each vine hedge befor going to its nearest jump off point
# as it moves ot its target (beacon) point - should have thought about that!! 
def wall_follow():
    print("wall follow initilised")
    global twist, bot_pose, bot_motion, currentBotState, distance_moved, wall_hit_point
    # Todo: Tune the parameters.
    # maybe turn right until zone_F is clear
    # Wall follow enter
    obstacle_in_front = numpy.any((zone_F < front_obs_distance))
    #obstacle_in_frontLeft = numpy.any((zone_FL < 2))
    #obstacle_in_frontRight = numpy.any((zone_FR < 2))
    distance_moved = math.sqrt(pow(bot_pose.position.y - wall_hit_point.y, 2) + pow(bot_pose.position.x - wall_hit_point.x, 2))
    print(line_distance(), distance_moved, (line_distance() < 0.2 and distance_moved > 0.5))

    if line_distance() < 0.2 and distance_moved > 0.5:
        print("line_hit")
        print(distance_moved)
        # found line point. rotate and move forward
        twist.angular.z = 0
        twist.linear.x = 0
        currentBotState = BotState.LOOK_TOWARDS
        return
    elif obstacle_in_front:  # turn right
        print("turn right")
        twist.angular.z = -0.5
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
    global bot_motion, twist, bot_pose, target, currentBotState, yaw_threshold
    #bot_motion = rospy.Publisher("/thorvald_001/teleop_joy/cmd_vel", Twist, queue_size=10)
    target_rad = target*math.pi/180 #-90 degrees to face towards vines
    print("bot pose", bot_pose)
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
        facing_vines = True
    bot_motion.publish(twist)
        


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
    # goal_location.unregister()
    rospy.wait_for_message("homing_signal", PoseStamped)

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
            

def process_sensor_info(data):
    global maxRange, minRange, front_obs_distance, left_obs_distance, zone_R, zone_FR, zone_F, zone_FL, zone_L
    maxRange = data.range_max
    print("maxRange", maxRange)
    minRange = data.range_min
    print("minRange", minRange)

    # Note: Configuration  - Breaking at uneven angles
    # we cuold use >> zone = numpy.array_split(numpy.array(data.ranges), 5) and split into 5 equal zones
    zone = numpy.array(data.ranges)
    zone_R = zone[0:50]  # 30deg
    zone_FR = zone[51:140]
    zone_F = zone[141:220]
    zone_FL = zone[221:310]
    zone_L = zone[311:361]

    if front_obs_distance is None and left_obs_distance is None:
        front_obs_distance = 2
        left_obs_distance = 2


def check_init_config():
    global bot_pose, beacon_pose, init_config_complete, init_bot_pose
    if bot_pose is not None and beacon_pose is not None:
        init_config_complete = True
        init_bot_pose = [bot_pose.position.x, bot_pose.position.y]
        bot_bug2()

# While loop to keep checkuing if we have hit homing position and are facing the vines to takje an image
# Shifts between states depending upon issues we see - mainly managing obsticles via wall following
def bot_bug2():
    global bot_motion, currentBotState, bot_pose
    bot_motion = rospy.Publisher("/thorvald_001/teleop_joy/cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(10)
    while not facing_vines:
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
            # return
        rate.sleep()
    print("Facing vines")




# --------------------------------------------- main program entry --------------------------------------

def init():
    global homing_signal
    rospy.init_node("bug2")
    homing_signal = rospy.Subscriber('/homing_signal', PoseStamped, callback)
    rospy.Subscriber('/thorvald_001/front_scan', LaserScan, process_sensor_info)
    rospy.Subscriber('/thorvald_001/odometry/base_raw', Odometry, get_base_truth)
    print('--------- bug2 has started -----------')
    rospy.spin()


if __name__ == '__main__':
    try:
        init()
        
    except rospy.ROSInterruptException:
        pass