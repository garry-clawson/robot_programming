#! /usr/bin/env python

# Credit-> https://www.theconstructsim.com/make-robot-detect-and-avoid-an-obstacle/ 

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist



class Avoider:

    def __init__(self):
        """
        On construction of the object, create a Subscriber
        to listen to lasr scans and a Publisher to control
        the robot
        """
        self.publisher = rospy.Publisher(
            '/thorvald_001/teleop_joy/cmd_vel',
            Twist, queue_size=1)
        # Note: front scan is -1.57 - 1.57 rads (-90 to +90 degress) with 720 laser points (rostopic echo/thorvald_001/front scan -n1) defined in urdf sensors
        # Good for 30mm accuracy as per spec
        rospy.Subscriber("/thorvald_001/front_scan", LaserScan, self.callback)


    def callback(self, msg):

        move = Twist()

        #If the distance to an obstacle in front of the robot is bigger than 1 meter, the robot will move forward
        if msg.ranges[90] < 2:
            move.angular.z = 1
        else:
            move.linear.x = 0.5


        #If the distance to an obstacle in front of the robot is smaller than 1 meter, the robot will stop
        print "Print min range: ", min(msg.ranges)

        print "Reading at position 0:", msg.ranges[0]
        print "Reading at position 90:", msg.ranges[90]
        print "Reading at position 180:", msg.ranges[179]

        # inbuilt function to find the position of minimum 
        minpos = msg.ranges.index(min(msg.ranges))
        # printing the position 
        print "The minimum is at position", minpos + 1

        self.publisher.publish(move)



if __name__ == '__main__':
    # as usual, initialise the ROS node with a name
    rospy.init_node('avoid_obsticle')
    # Create the robot move object 
    # (which in turns registers the subscriber and make the system go)
    Avoider()
    # Enable mover to be killed and screens removed
    print('--------- Avoider has started -----------')
    try:
        rospy.spin() # run mover until stopped 
    except KeyboardInterrupt:
        print "Shutting down"
        print('----- Avoider exiting -----')
        cv2.destroyAllWindows() # destroy all opencv windows when killing node