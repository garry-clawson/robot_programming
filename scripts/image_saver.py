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
        min = np.array([35, 000, 000]) 
        max = np.array([180, 253, 255]) 
        
        mask = cv2.inRange(HSVimage, min, max) # threshold
        #self.saveImage(mask)

        bunch_image = cv2.bitwise_and(HSVimage, HSVimage, mask=mask) # obtain threshold result
        #self.saveImage(bunch_image)

        im_NoBackground = cv2.cvtColor(bunch_image, cv2.COLOR_HSV2BGR) # reconvert color space for publishing
        
        return im_NoBackground


    def removeVines(self, image):

        HSVimage = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)   # convert color space for thresholding
        cv2.imshow("hasImage", HSVimage)
        cv2.waitKey(0) 
        # mask out vine - values found by using hsv_range_detector.py, inspired by https://www.youtube.com/watch?v=We6CQHhhOFo&t=136s  -> ROS and OpenCv for beginners | Blob Tracking and Ball Chasing with Raspberry Pi by Tiziano Fiorenzani
        min = np.array([95, 000, 46])
        max = np.array([255, 255, 255]) 
        vinemask = cv2.inRange(HSVimage, min, max) # threshold
        cv2.imshow("vinemask", vinemask)
        cv2.waitKey(0) 
        #self.saveImage(vinemask)
        
        vinemask = cv2.dilate(vinemask, np.ones((10, 10))) # expand mask
        cv2.imshow("diliated", vinemask)
        cv2.waitKey(0) 

        # Add kernal to complete the morphEx operation using morph_elispse (simular shape to grapes)
        # Inspired by -> https://www.pyimagesearch.com/2021/04/28/opencv-morphological-operations/
        #kernelSize = [(5, 5)] 
        #kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, kernelSize)
        #vinemask = cv2.morphologyEx(vinemask, cv2.MORPH_OPEN, kernal) # remove small white dots
        #self.saveImage(vinemask)


        # close all windows to cleanup the screen, then initialize a list of
        # of kernels sizes that will be applied to the image
        #kernelSizes = [(1, 1), (2, 2), (3, 3)]
        # loop over the kernels sizes
        #for kernelSize in kernelSizes:
	        #construct a rectangular kernel from the current size and then apply an "opening" operation
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        vinemask = cv2.morphologyEx(vinemask, cv2.MORPH_OPEN, kernel)
        cv2.imshow("final vinemask", vinemask)
        cv2.waitKey(0)

        # close all windows to cleanup the screen
        



        # obtain threshold result
        grapeBunchImage = cv2.bitwise_and(HSVimage, HSVimage, mask=vinemask) 
        cv2.imshow("grapeBunchImage", grapeBunchImage)
        cv2.waitKey(0) 
        #self.saveImage(grapeBunchImage)
        
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
