#! /usr/bin/python
# coding=utf-8

# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
from cv2 import namedWindow, cvtColor, imshow, inRange
# Numpy for image array thresholds
import numpy as np
# Datetime for catagorising images in sequence
from datetime import datetime




class image_listener:

    def __init__(self):

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/thorvald_001/kinect2_front_camera/hd/image_color_rect",
                                          Image, self.image_callback)

    def image_callback(self, data):
        print("Received an image!")

        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print(e)
        else:
            img_noBackground = self.removeBackGround(cv2_img) # remove background on start
            img_removeVines = self.removeVines(img_noBackground)
            #self.saveImage(img_removeVines)


    def removeBackGround(self, image):
        #Remove background
        HSVimage = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) # convert color space
        # between values for thresholding
        min = np.array([043, 000, 000]) 
        max = np.array([180, 253, 255]) 
        mask = cv2.inRange(HSVimage, min, max) # threshold
        cropimage = cv2.bitwise_and(HSVimage, HSVimage, mask=mask) # obtain threshold result
        im_NoBackground = cv2.cvtColor(cropimage, cv2.COLOR_HSV2BGR) # reconvert color space for publishing
        return im_NoBackground


    def removeVines(self, image):
        BLURogimage = cv2.GaussianBlur(image,(35,35),0) # blur to remove details
        HSVimage = cv2.cvtColor(BLURogimage, cv2.COLOR_BGR2HSV)   # convert color space for thresholding
        # mask out vine 
        min = np.array([035, 000, 000]) 
        max = np.array([180, 129, 254]) 
        vinemask = cv2.inRange(HSVimage, min, max) # threshold
        vinemask = cv2.morphologyEx(vinemask, cv2.MORPH_OPEN, np.ones((8, 8))) # remove small white dots
        # obtain threshold result
        grapeBunchImage = cv2.bitwise_and(HSVimage, HSVimage, mask=vinemask) 
        return grapeBunchImage

    def saveImage(self, image):
        # Save your OpenCV2 image as a jpeg 
        time = datetime.now()
        filepath = 'grape_bunches'+str(time)+'.jpg' 
        print('saving to ',filepath)
        cv2.imwrite(filepath, image)
        imshow("cv2", image)
        rospy.sleep(1)


#startWindowThread()
rospy.init_node('image_listener')
il = image_listener()
print('--------- image saver started -----------')
try:
    rospy.spin() # run image saver until stopped 
except KeyboardInterrupt:
    print "Shutting down"
    print('----- imsage saver exiting-----')
    cv2.destroyAllWindows() # destroy all opencv windows when killing node
