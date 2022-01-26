#! /usr/bin/python
# coding=utf-8

# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy, sys
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

        # Enable OpenCV with ROS
        self.bridge = CvBridge()
        # Subscribe to front camera feed
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
            self.saveImage(img_removeVines)


    def removeBackGround(self, image):
        # Remove background
        HSVimage = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) # convert color space
        # between values for thresholding
        min = np.array([35, 000, 000]) 
        max = np.array([180, 253, 255]) 
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
        min = np.array([90, 000, 40])
        max = np.array([255, 255, 255]) 
        vinemask = cv2.inRange(HSVimage, min, max) # threshold
        #cv2.imshow("vinemask", vinemask)
        #cv2.waitKey(0) 
        #self.saveImage(vinemask)

        # Remove odd small spots
        # Inspired by -> https://stackoverflow.com/a/42812226
        dummy_image = vinemask.astype(np.uint8) # reconvert to uint8
        #find all your connected components (white blobs in your image)
        nb_components, output, stats, centroids = cv2.connectedComponentsWithStats(dummy_image, connectivity=8)
        #connectedComponentswithStats yields every seperated component with information on each of them, such as size
        #the following part is just taking out the background which is also considered a component, but most of the time we don't want that.
        sizes = stats[1:, -1]; nb_components = nb_components - 1
        # minimum size of particles we want to keep (number of pixels)
        #here, it's a fixed value, but you can set it as you want, eg the mean of the sizes or whatever
        min_size = 60 # Value found through trial and error - since we are donig this pre dilation we need ot pick up smaller elements
        #answer image
        vinemask_updated = np.zeros((output.shape))
        #for every component in the image, you keep it only if it's above min_size
        for i in range(0, nb_components):
            if sizes[i] >= min_size:
                vinemask_updated[output == i + 1] = 255

        vinemask_updated = vinemask_updated.astype(np.uint8) # reconvert to uint8
        #cv2.imshow("vinemask updated", vinemask_updated)
        #cv2.waitKey(0) 

        # Increase size of remaining pixels
        vinemask_updated = cv2.dilate(vinemask_updated, np.ones((15, 15)), iterations = 3) # expand mask
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
        #self.saveImage(grapeBunchImage)
        
        return im_detectGrapes_with_keypoints


    def detectGrapes(self, image, mask):
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
        im_with_keypoints = cv2.drawKeypoints(image, keypoints, np.array([]), (000,000,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        if(len(keypoints) != 0): # If a blob is detected, print out how many
            print('Keypoints = ',len(keypoints))
        #cv2.imshow("detect keypoints image", im_with_keypoints)
        #cv2.waitKey(0)
        return im_with_keypoints, keypoints

    def saveImage(self, image):
        # Save your OpenCV2 image as a jpeg 
        time = datetime.now()
        filepath = 'grape_bunches'+str(time)+'.jpg' 
        print('saving to ',filepath)
        cv2.imwrite(filepath, image)
        #imshow("cv2", image)
        rospy.sleep(1)


#startWindowThread()
rospy.init_node('image_listener')
il = image_listener()
print('--------- image saver started -----------')
try:
    rospy.spin() # run image saver until stopped 
except KeyboardInterrupt:
    print "Shutting down"
    print('----- image saver exiting-----')
    cv2.destroyAllWindows() # destroy all opencv windows when killing node
