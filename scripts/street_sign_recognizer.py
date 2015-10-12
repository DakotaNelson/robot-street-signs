#!/usr/bin/env python

""" This is a script that walks through some of the basics of working with images
    with opencv in ROS. """

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class StreetSignRecognizer(object):
    """ This robot should recognize street signs """

    def __init__(self, image_topic):
        """ Initialize the street sign reocgnizer """
        rospy.init_node('street_sign_recognizer')
        self.cv_image = None                        # the latest image from the camera
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV

        rospy.Subscriber(image_topic, Image, self.process_image)
        cv2.namedWindow('video_window')


        self.use_slider = False
        self.use_mouse_hover = True

        # # # # # # # # # # # # #
        # color params, in HSV  # 
        # # # # # # # # # # # # #
        
        self.COLOR = "yellow"                       # which default color we binarize based on
        self.color_bounds = {}

        # tuned from the green hand ball
        self.color_bounds["green"] = (
              np.array([60,76,2])                   # min
            , np.array([80,255,255])                # max
        )
        # tuned from yellow lturn sign
        self.color_bounds["yellow"] = (
              np.array([23,175,130])                # min
            , np.array([32,255,255])                # max
        )
        
        if self.use_mouse_hover:
            # when mouse hovers over video window
            cv2.setMouseCallback('video_window', self.process_mouse_event)

        if self.use_slider:
            cv2.namedWindow('threshold_image')
            self.hsv_lb = np.array([0, 0, 0])
            cv2.createTrackbar('H lb', 'threshold_image', 0, 255, self.set_h_lb)
            cv2.createTrackbar('S lb', 'threshold_image', 0, 255, self.set_s_lb)
            cv2.createTrackbar('V lb', 'threshold_image', 0, 255, self.set_v_lb)
            self.hsv_ub = np.array([255, 255, 255])
            cv2.createTrackbar('H ub', 'threshold_image', 0, 255, self.set_h_ub)
            cv2.createTrackbar('S ub', 'threshold_image', 0, 255, self.set_s_ub)
            cv2.createTrackbar('V ub', 'threshold_image', 0, 255, self.set_v_ub)

    # # # # # # # # # #
    # color callbacks # 
    # # # # # # # # # #

    def set_h_lb(self, val):
        self.hsv_lb[0] = val

    def set_s_lb(self, val):
        self.hsv_lb[1] = val

    def set_v_lb(self, val):
        self.hsv_lb[2] = val

    def set_h_ub(self, val):
        self.hsv_ub[0] = val

    def set_s_ub(self, val):
        self.hsv_ub[1] = val

    def set_v_ub(self, val):
        self.hsv_ub[2] = val

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

        if self.use_slider:
            # binarize based on the slider values
            self.binarized_image = cv2.inRange(self.hsv_image, self.hsv_lb, self.hsv_ub)
        else:
            # binarize based on preset values
            self.binarized_image = cv2.inRange(self.hsv_image, self.color_bounds[self.COLOR][0], self.color_bounds[self.COLOR][1])

        cv2.imshow('video_window', self.binarized_image)
        cv2.waitKey(5)

    def find_object_center(self, binary_image):
        moments = cv2.moments(binary_image)
        if moments['m00'] != 0:
            self.center_x, self.center_y = moments['m10']/moments['m00'], moments['m01']/moments['m00']
            return True
        return False

    def process_mouse_event(self, event, x,y,flags,param):
        """ Process mouse events so that you can see the color values associated
            with a particular pixel in the camera images """
        image_info_window = 255*np.ones((500,500,3))
        
        # show hsv values
        cv2.putText(image_info_window,
                    'Color (h=%d,s=%d,v=%d)' % (self.hsv_image[y,x,0], self.hsv_image[y,x,1], self.hsv_image[y,x,2]),
                    (5,50), # 5 = x, 50 = y
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0,0,0))

        # show bgr values 
        cv2.putText(image_info_window,
                    'Color (b=%d,g=%d,r=%d)' % (self.cv_image[y,x,0], self.cv_image[y,x,1], self.cv_image[y,x,2]),
                    (5,100),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0,0,0))
        cv2.imshow('image_info', image_info_window)
        cv2.waitKey(5)

    def run(self):
        """ The main run loop, in this node it doesn't do anything """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == '__main__':
    node = StreetSignRecognizer("/camera/image_raw")
    node.run()