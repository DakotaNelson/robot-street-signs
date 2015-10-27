#!/usr/bin/env python

""" Listens on the street sign prediction topic and uses the resultant
    predictions to obey street signs. """

from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point
from std_msgs.msg import Header
from std_msgs.msg import String, Bool
from time import sleep
import threading
import smach
import smach_ros
import rospy
import tf
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def convert_translation_rotation_to_pose(translation, rotation):
    """ Convert from representation of a pose as translation and rotation (Quaternion) tuples to a geometry_msgs/Pose message """
    return Pose(position=Point(x=translation[0],y=translation[1],z=translation[2]), orientation=Quaternion(x=rotation[0],y=rotation[1],z=rotation[2],w=rotation[3]))

def convert_pose_to_xy_and_theta(pose):
    """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
    orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    angles = euler_from_quaternion(orientation_tuple)
    return (pose.position.x, pose.position.y, angles[2])

###### State Machine States ######

class StateMachine():
    def __init__(self, publisher):
        """ Initialize the StateMachine 

        publisher: the handler to a method that publishes way point goals
        """

        # Goal publisher for the ROS Navstack
        self.publishGoal = publisher

        # Sleep publisher to communicate with the Street Sign Prediction node
        self.imSleeping = rospy.Publisher("/imSleeping", Bool, queue_size=1)

        self.end = False

        # hand tuned way point locations for the size of the T intersection map
        delta_x = 2.5
        delta_y = 1.5
        self.theta = 0
        self.nodes = {
            'start' : (0,0,0),
            'inter' : (delta_x,0,0),
            'left' : (delta_x, delta_y, 0),
            'right' : (delta_x, -delta_y, 0)
        }

        # Robot begins by leaving the start and heading toward the intersection
        self.last_node = 'start'
        self.curr_dest = 'inter'

    def run(self):
        """ Run the StateMachine! """
        while not self.end:
            self.go_robot()
            sleep(.5)

    def sleep(self, time):
        """ Publishes it is sleeping,
            enters a sleeping state (blocking program execution),
            and finally republishes it is not sleeping once is has finished """
        self.imSleeping.publish(True)
        sleep(time)
        self.imSleeping.publish(False)

    def calc_transition(self, sign):
        """
        Determine next destination based off of current destination, previous location
        and the last seen sign
        """
        if sign == 'uturn':
            tmp = self.last_node
            self.last_node = self.curr_dest
            self.curr_dest = tmp
            self.theta = -self.theta
        elif sign == 'rturn':
            if self.curr_dest == 'inter':
                if self.last_node == 'start':
                    self.last_node = self.curr_dest
                    self.curr_dest = 'right'
                    self.theta = -np.pi/2
                elif self.last_node == 'left':
                    self.last_node = self.curr_dest
                    self.curr_dest = 'start'
                    self.theta = np.pi
            else:
                # invalid turn in our set up
                print "Invalid turn"
        elif sign == 'lturn':
            if self.curr_dest == 'inter':
                if self.last_node == 'start':
                    self.last_node = self.curr_dest
                    self.curr_dest = 'left'
                    self.theta = np.pi/2
                elif self.last_node == 'right':
                    self.last_node = self.curr_dest
                    self.curr_dest = 'start'
                    self.theta = np.pi
            else:
                # invalid turn in our set up
                print "Invalid turn"
        self.go_robot()
        self.sleep(3)

    def go_robot(self):
        """
        Publish a navigation goal to the current destination
        """
        self.publishGoal(self.nodes[self.curr_dest], self.theta)


###### Node Class ######

class StreetSignFollower(object):
    """ Issues goals based on a state machine transitioned by
        listening on the street sign topic. """

    def __init__(self, sign_topic):
        """ Initialize the StreetSignFollower 

        sign_topic: str, name of the sign topic, i.e. '/predicted_sign' 
        """

        rospy.init_node('sign_follower')

        # move_base_simple/goal is part of the ROS Navigation Stack
        self.pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

        listener = tf.TransformListener()
        listener.waitForTransform('/map', 'base_link', rospy.Time(), rospy.Duration(3.0))

        # Find the initial start position of the robot in terms of the map frame
        (initial_trans, initial_rot) = listener.lookupTransform('/map', '/base_link', rospy.Time())

        # Convert to x, y, theta for trigonometric convenience
        self.init_x, self.init_y, self.init_th = convert_pose_to_xy_and_theta(convert_translation_rotation_to_pose(initial_trans, initial_rot))
        print self.init_x, self.init_y, self.init_th

        # Subscribe to predicted street sign topic 
        rospy.Subscriber(sign_topic, String, self.process_sign)

    def process_sign(self, msg):
        """ Process sign predictions, use them to transition the state machine. """

        print("The sign says: {}".format(msg.data))

        # Calculate next way point based on the new predicted sign message
        self.sm.calc_transition(msg.data)

    def publishGoal(self, pos_tup=(0.0,0.0,0.0), th=0.0):
        """ Converts the way point position tuple and theta angle in terms
        of the map frame, and publishes a Pose to the ROS goal navigation topic

        pos_tup: a tuple of x, y, z position values
        th: theta or yaw of the robot
        """
        x, y, z = pos_tup
        print("Publishing goal at ({},{},{},{})".format(x,y,z,th))

        # compute final x, y, theta in terms of the map
        final_x = (
            self.init_x +
            x * np.cos(self.init_th) +
            y * np.cos(self.init_th + np.pi / 2)
        )
        final_y = (
            self.init_y +
            x * np.sin(self.init_th) +
            y * np.sin(self.init_th + np.pi / 2)
        )
        final_th = self.init_th + th

        point_msg = Point(final_x, final_y, z)
        quat_msg = Quaternion(*quaternion_from_euler(0,0,final_th))
        pose_msg = Pose(position=point_msg, orientation=quat_msg)

        # in terms of the map frame!
        header_msg = Header(stamp=rospy.Time.now(),
                                    frame_id='map')

        pose_stamped = PoseStamped(header=header_msg, pose=pose_msg)
        self.pub.publish(pose_stamped)

    def run(self):
        """ The main run loop - create a state machine, and set it off """

        # Create a state machine
        self.sm = StateMachine(self.publishGoal)

        # Execute the machine
        # threading required for control-C-ability

        # Create a thread to execute the smach container
        smach_thread = threading.Thread(target=self.sm.run)
        smach_thread.start()

        # Wait for ctrl-c
        rospy.spin()

        # stop the state machine
        self.sm.end = True

        # Block until everything is shut down
        smach_thread.join()

if __name__ == '__main__':
    node = StreetSignFollower("/predicted_sign")
    node.run()
