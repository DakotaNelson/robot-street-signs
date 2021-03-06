#!/usr/bin/env python

""" This is a script that walks through some of the basics of working with images
    with opencv in ROS. """

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String, Bool
import cv2
import numpy as np
import os

from template_matcher import TemplateMatcher


class StreetSignRecognizer(object):
    """ This robot should recognize street signs """


    curr_dir = os.path.dirname(os.path.realpath(__file__))
    template_images = {
        "lturn":  os.path.join(curr_dir, '../images/leftturn_box_small.png'),
        "rturn": os.path.join(curr_dir, '../images/rightturn_box_small.png'),
        "uturn": os.path.join(curr_dir, '../images/uturn_box_small.png')
    }

    def __init__(self, image_topic, sleep_topic):
        """ Initialize the street sign reocgnizer """
        rospy.init_node('street_sign_recognizer')
        self.cv_image = None                        # the latest image from the camera
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV
        self.saveCounter = 0                        # how many images we've saved to disk


        print "Loading TemplateMatcher"
        self.template_matcher = TemplateMatcher(self.template_images)

        self.pub = rospy.Publisher('predicted_sign', String, queue_size=1)
        cv2.namedWindow('video_window')

        self.sleeping = False

        self.running_predictions = {"lturn": 0, "rturn": 0, "uturn": 0}

        self.sign_pub_mappings = {
            "uturn": 1,
            "lturn": 2,
            "rturn": 3
        }

        # 'use' parameters for quick changes in node functionality
        self.use_slider = False
        self.use_mouse_hover = False
        self.use_saver = False
        self.use_predict = True

        # threshold by which the running confidence summation must achieve to publish a predicted_sign
        # hand tuned
        self.decision_threshold = 35

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

        rospy.Subscriber(image_topic, Image, self.process_image)
        rospy.Subscriber(sleep_topic, Bool, self.process_sleep)

    # # # # # # # # # #
    # color callbacks #
    # # # # # # # # # #

    def set_h_lb(self, val):
        """ set hue lower bound """
        self.hsv_lb[0] = val

    def set_s_lb(self, val):
        """ set saturation lower bound """
        self.hsv_lb[1] = val

    def set_v_lb(self, val):
        """ set value lower bound """
        self.hsv_lb[2] = val

    def set_h_ub(self, val):
        """ set hue upper bound """
        self.hsv_ub[0] = val

    def set_s_ub(self, val):
        """ set saturation upper bound """
        self.hsv_ub[1] = val

    def set_v_ub(self, val):
        """ set value upper bound """
        self.hsv_ub[2] = val

    def process_sleep(self, msg):
        """ Process sleep messages from the navigation node and stash them
            in an attribute called sleeping """
        self.sleeping = msg.data

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

        # compute bounding box points, bounding the "white" pixel grids in a binarized image
        binaryGrid = thresh2binarygrid(self.binarized_image, gridsize=(20, 20), percentage=0.2)
        pt1, pt2 = get_bbox_from_grid(self.binarized_image, binaryGrid, pad=1)

 
        if self.use_predict and pt1 and pt2:
            # crop and gray scale the bounding box region of interest
            cropped_sign = self.cv_image[pt1[1]:pt2[1], pt1[0]:pt2[0]]
            cropped_sign_gray = cv2.cvtColor(cropped_sign, cv2.COLOR_BGR2GRAY)

            # make predictions with confidence for each sign key
            prediction = self.template_matcher.predict(cropped_sign_gray)

            for sign_key in prediction:
                self.running_predictions[sign_key] += prediction[sign_key]

        # draw bounding box rectangle
        cv2.rectangle(self.cv_image, pt1, pt2, color=(0, 0, 255), thickness=5)
        cv2.imshow('video_window', self.cv_image)

        if self.use_saver:
            cv2.imwrite("/tmp/bin_img_{0:0>4}.jpg".format(self.saveCounter), cropped_sign)
            self.saveCounter += 1

        cv2.waitKey(5)

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
            if not self.sleeping:
                print self.running_predictions
                sorted_preds = sorted(self.running_predictions.iterkeys(), key=(lambda key: self.running_predictions[key]), reverse=True)
                best = sorted_preds[0]
                second = sorted_preds[1]

                # If cummulative running predictions > decision threshold
                # AND the confidence for the best sign is greater than the second best by a certain value
                if (sum(self.running_predictions.values()) > self.decision_threshold
                and self.running_predictions[best] - self.running_predictions[second] > self.decision_threshold/5):
                    # publish the best predicted_sign and reset running predictions
                    self.pub.publish(best)
                    self.running_predictions = {"lturn": 0, "rturn": 0, "uturn": 0}
            else:
                # Don't store running predictions while sleeping
                self.running_predictions = {"lturn": 0, "rturn": 0, "uturn": 0}

            r.sleep()


def thresh2binarygrid(img, gridsize=(10,10), percentage=0.20):
    """ Takes a thresholded image (i.e. from a cv2.inRange operation), divides
    the image into a grid of gridsize; if a gridblock has percentage white pixels,
    the gridblock will be "turned on"

    Parameters
    ----------
    img: np.array, shape (image pixel height, image pixel width)
        values should be between 0 - 255.

    gridsize: tuple, default (10, 10)

    percentage: float, between 0 - 1

    Returns
    -------
    grid: binary grid of size gridsize
    """
    gridrows, gridcols = gridsize
    m, n = img.shape

    # initialize binary grid
    grid = np.zeros(gridsize)

    # define the size of each grid block
    block_size = (m/gridrows, n/gridcols)

    for i in range(gridrows):
        for j in range(gridcols):
            block = img[int(block_size[0]*i):int(block_size[0]*(i+1)), int(block_size[1]*j):int(block_size[1]*(j+1))]

            # if percentage of the block was white (inRange)
            threshold = block_size[0] * block_size[1] * 255 * percentage
            if block.sum() >= threshold:
                # turn the gridblock on
                grid[i,j] = 1

    return grid

def get_bbox_from_grid(img, grid, pad=0):
    """ Gets a bounding box in the form of (pt1, pt2), pairs of (x,y)
    coordinates that define the lefttop and rightbottom of the bounding rectangle """
    top, left, bottom, right = (None, ) * 4

    # get top
    for i in range(grid.shape[0]):
        if grid[i].sum() > 0:
            top = i
            break

    # get left
    for j in range(grid.shape[1]):
        if grid[:, j].sum() > 0:
            left = j
            break

    # get bottom
    for i in range(grid.shape[0] - 1, -1, -1):
        if grid[i].sum() > 0:
            bottom = i
            break

    # get right
    for j in range(grid.shape[1] - 1, -1, -1):
        if grid[:, j].sum() > 0:
            right = j
            break

    # if bbox was not found
    if top is None or left is None or bottom is None or right is None:
        return None, None

    # make bbox a little bigger
    top -= pad
    left -= pad
    bottom += pad
    right += pad

    # bounds should not be outside of the grid area
    if top < 0:
        top = 0
    if left < 0:
        left = 0
    if bottom > grid.shape[0]:
        bottom = grid.shape[0]
    if right > grid.shape[1]:
        right = grid.shape[1]

    # (x, y)
    pt1 = (img.shape[1]/grid.shape[1]*left, img.shape[0]/grid.shape[0]*top)

    # (right+1) and (bottom+1) cuz grid only represents the left hand corner of a block
    pt2 = (img.shape[1]/grid.shape[1]*(right+1), img.shape[0]/grid.shape[0]*(bottom+1))

    return pt1, pt2

if __name__ == '__main__':
    node = StreetSignRecognizer("/camera/image_raw","/imSleeping")
    node.run()
