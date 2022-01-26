#!/usr/bin/env python

# Inspired by the great LCAS repo -> https://github.com/LCAS/CMP9767M/tree/master/uol_cmp9767m_tutorial/scripts 

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy
import random #used for giving different angluar speeds to make the robot explore all areas when avoiding collisions

class Mover:
    """
    A very simple Roamer implementation for Thorvald.
    It simply goes straight until any obstacle is within
    3 m distance and then just simply turns left.
    A purely reactive approach.
    """

    def __init__(self):
        """
        On construction of the object, create a Subscriber
        to listen to lasr scans and a Publisher to control
        the robot
        """
        self.publisher = rospy.Publisher('/thorvald_001/teleop_joy/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber("/thorvald_001/front_scan", LaserScan, self.callback)

    def callback(self, data):
        """
        Callback called any time a new laser scan becomes available
        """
        # Logs the details and prints to console
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.header.seq)

        # Note: The range split is currently at uneven angles. We could use >> 'zone = numpy.array_split(numpy.array(data.ranges), 5)' and split into 5 equal zones?
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


        t = Twist()
        if numpy.any((zone_F < 2)) or numpy.any((zone_FR < 2)) or numpy.any((zone_FL < 2)):
            t.angular.z = 1 
        else:
            t.linear.x = 0.8
        self.publisher.publish(t)



        #Takes min distance from laserscan and if less than 4m issues twist message to rotate
        #min_dist = min(data.ranges)
        #t = Twist()
        #if min_dist < 4:
        #    t.angular.z = 1.0
        #else:
        #    t.linear.x = 0.8
        #self.publisher.publish(t)



if __name__ == '__main__':
    # as usual, initialise the ROS node with a name
    rospy.init_node('robot_move')
    # Create the robot move object 
    # (which in turns registers the subscriber and make the system go)
    Mover()
    # Enable mover to be killed and screens removed
    print('--------- mover has started -----------')
    try:
        rospy.spin() # run mover until stopped 
    except KeyboardInterrupt:
        print "Shutting down"
        print('----- mover exiting -----')
        cv2.destroyAllWindows() # destroy all opencv windows when killing node