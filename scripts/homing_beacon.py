#!/usr/bin/env python

# Credit --> https://github.com/bipul93/ros-bug2/blob/master/scripts/homing_beacon.py

import rospy
from geometry_msgs.msg import PoseStamped

def talker():
    pub = rospy.Publisher('homing_signal', PoseStamped, queue_size=10)
    rospy.init_node('homing_beacon', anonymous=False)
    rate = rospy.Rate(10) # 10hz

    print("-------- Homing beacon initialised ----------")

    home_pose = PoseStamped()
    home_pose.header.frame_id = "thorvald_001/odom"

    home_pose.pose.position.x = 0
    home_pose.pose.position.y = -10
    home_pose.pose.position.z = 0

    
    while not rospy.is_shutdown():
        home_pose.header.stamp = rospy.Time.now()

        pub.publish(home_pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass