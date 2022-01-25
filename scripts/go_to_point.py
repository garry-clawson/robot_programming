#! /usr/bin/env python

# Credit -> https://www.theconstructsim.com/exploring-ros-with-a-2-wheeled-robot-10-bug-1/

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

import math

# robot state variables
active_ = False
position_ = Point()
yaw_ = 0
# machine state
state_ = 0
# goal
desired_position_ = Point()
desired_position_.x = 0 #rospy.get_param('des_pos_x')
desired_position_.y = 0 #rospy.get_param('des_pos_y')
desired_position_.z = 0
# parameters
yaw_precision_ = math.pi / 45  # +/- 4 degree allowed.   5 * (math.pi / 180) # 5 degrees
dist_precision_ = 0.3

# service callback
def go_to_point_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

# publishers
pub = None


def change_state(state):
    global state_
    state_ = state
    print 'State changed to [%s]' % state_


def fix_yaw(des_pos):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x) #this is in rads
    err_yaw = desired_yaw - yaw_ #desired yaw is in rads. yaw is in degrees. This causes and error and riobot to just rotate back and forth
    print('yaw', yaw_)
    print('err_yaw', err_yaw)
    print('ya prec', yaw_precision_)
    print('desired ya', desired_yaw)

    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_: #yaw_precsion is in rads
        twist_msg.angular.z = -0.3 if err_yaw > 0 else 0.3
        #print('--------- stuck here -----------')

    pub.publish(twist_msg)

    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_:
        print 'Yaw error: [%s]' % err_yaw
        change_state(1)


def go_straight_ahead(des_pos):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.3
        pub.publish(twist_msg)
    else:
        print 'Position error: [%s]' % err_pos
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        print 'Yaw error: [%s]' % err_yaw
        change_state(0)


def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)
    print('--------- at point -----------')


def clbk_odom(msg):
    global position_
    global yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    #yaw_ = euler[2] #This is in degrees
    yaw_ = euler[2]*math.pi/180 #This is in rads
    #print('test - yaw [2]', yaw_)


def main():
    global pub, active_

    rospy.init_node('go_to_point')
    pub = rospy.Publisher('/thorvald_001/teleop_joy/cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/thorvald_001/odometry/base_raw', Odometry, clbk_odom)
    srv = rospy.Service('go_to_point_switch', SetBool, go_to_point_switch)

    print('--------- go to point has started -----------')

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_:
            continue
        else:
            if state_ == 0:
                print('--------- state 0 yaw-----------')
                fix_yaw(desired_position_)
            elif state_ == 1:
                print('--------- state 1 ahead -----------')
                go_straight_ahead(desired_position_)
            elif state_ == 2:
                print('--------- done -----------')
                done()
            else:
                rospy.logerr('Unknown state!')

        rate.sleep()


if __name__ == '__main__':
    main()
