#!/usr/bin/env python

""" Listens on the street sign prediction topic and uses the resultant
    predictions to obey street signs. """

from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point
from std_msgs.msg import Header
from std_msgs.msg import String
from time import sleep
import threading
import smach
import smach_ros
import rospy

###### State Machine States ######

class StateMachine():
    def __init__(self, publisher, data={}):
        # state initialization goes here
        self.res = 'forward'
        self.data = data
        self.publishGoal = publisher
        self.stop = False

        # if a function returns [this string], execute [this function]
        self.transitions = {
                  'rturn': self.rturn,
                  #'lturn': self.lturn,
                  #'uturn': self.uturn,
                  'forward': self.forward,
                  'stop': self.stop
                }

    def run(self):
        while not self.stop:
            res = self.transitions[self.res]()
            self.res = res
            # the robot should 'latch' slightly once it's made a decision
            sleep(.5)

    """ Each function is a different state. """

    def forward(self):
        print("The sign says: {}".format(self.data['sign']))

        self.publishGoal(0,0,0)

        if self.data['sign'] and self.data['sign'] == 'rturn':
            return 'rturn'
        elif self.data['sign'] == 'lturn':
            return 'lturn'
        elif self.data['sign'] == 'uturn':
            return 'uturn'
        elif self.data['sign'] == 'stop':
            return 'stop'
        else:
            return 'forward'

    def rturn(self):
        sleep(2)

        """if self.data.sign == 'rturn':
            self.certainty += 1
        else:
            self.certainty -= 1

        if self.certainty > 3:"""
        if self.data['sign'] == 'rturn':
            return 'rturn'
        else:
            return 'forward'

    def stop(self):
        # stop the robbit
        pass


###### Node Class ######

class StreetSignFollower(object):
    """ Issues goals based on a state machine transitioned by
        listening on the street sign topic. """

    def __init__(self, sign_topic):
        """ Initialize the state machine """

        rospy.init_node('sign_follower')

        self.pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

        rospy.Subscriber(sign_topic, String, self.process_sign)

    def process_sign(self, msg):
        """ Process sign predictions, use them to transition the state machine. """

        self.sm.data['sign'] = msg.data

    @staticmethod
    def publishGoal(x=0.0, y=0.0, z=0.0):
        """point_msg = Point(x, y, z)
        quat_msg = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        pose_msg = Pose(position=point_msg, orientation=quat_msg)

        header_msg = Header(stamp=rospy.Time.now(),
                                    frame_id='base_link')

        pose_stamped = PoseStamped(header=header_msg, pose=pose_msg)

        self.pub.Publish(pose_stamped)"""

        print("IT WORKS")

    def run(self):
        """ The main run loop - create a state machine, and set it off """

        # Create a state machine
        self.sm = StateMachine(self.publishGoal, {'sign': None})

        # Execute the machine
        # threading required for control-C-ability

        # Create a thread to execute the smach container
        smach_thread = threading.Thread(target=self.sm.run)
        smach_thread.start()

        # Wait for ctrl-c
        rospy.spin()

        # stop the state machine
        self.sm.stop = True

        # Block until everything is shut down
        smach_thread.join()

if __name__ == '__main__':
    node = StreetSignFollower("/sign_predictions")
    node.run()
