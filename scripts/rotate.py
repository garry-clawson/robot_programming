#!/usr/bin/env python

# Credit -> https://www.theconstructsim.com/ros-qa-135-how-to-rotate-a-robot-to-a-desired-heading-using-feedback-from-odometry/

'''
Comments on use of code and reasoning:

This small program will be used to orientate the robot towards the grape vines for image taking.
An alternative was to square up against the real wall measuring 15 degs eaither side and checking distance
the self correcting the pose, however that would also have only got us square and not square to the hedge
The asumption is the hedge is square to the world and the centre of the world is centre of the grid [0,0] - I have tried to measure this but the
laser scanner shoots through as it is not a completly full structure.
April tags could be nice in a real world but not practical here

This program will be called when we reach the first beacon point along the hedge to take our first image
We will then move to the nexct spot for an image, if we have had to avoid an obsticle and our orientaion is off,
we will then again call this to ensure we are facing the hedge.
'''


import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
import math
from std_msgs.msg import String

# Set roll and pitch and yaw to zero
roll = pitch = yaw = 0.0
target = -90 # Target angle to achive ibn degrees (Note: this is the world view and is directly facuing the grape vines)
kp=0.5 # Slows the angle of rotation the closer you get to the desried angle (stops from overshooting)

# recives pose orienatio and converst from quarts to euler so we can translate to roll, pitch and yaw
def get_rotation (msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    #print yaw
 

# Init node 
rospy.init_node('rotate_robot')

# Subscribe to odeomtry for rotation and cmd_vel for movements
sub = rospy.Subscriber ('/thorvald_001/odometry/base_raw', Odometry, get_rotation)
pub = rospy.Publisher('/thorvald_001/teleop_joy/cmd_vel', Twist, queue_size=1)

r = rospy.Rate(10)
command =Twist()
print("command", command)


while not rospy.is_shutdown():
    # Translate degrees (our target) into rads
    target_rad = target*math.pi/180
    command.angular.z = kp * (target_rad-yaw)
    pub.publish(command)
    r.sleep()
    #print("target={}, yaw={}", target_rad, yaw)

    #print("target_rad - yaw", (target_rad-yaw))
    # Will shutdown the node once rotation angle is achieved
    #if(target_rad-yaw) < 0.0002:
        #rospy.signal_shutdown("rotation angle achieved")
        #print("rotation angle achieved")

    




