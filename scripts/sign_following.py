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

class Forward(smach.State):
    #def __init__(self, outcomes=['r_turn', 'l_turn', 'u_turn', 'stop']):
    def __init__(self, outcomes=['r_turn', 'forward', 'preempted']):
        # state initialization goes here
        smach.State.__init__(self, outcomes=outcomes,
                             input_keys = ['sign'])

    def execute(self, data):
        # Check for preempt
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        if data.sign:
            print(data.sign.data)
            print(data.sign.data == 'rturn')

        if data.sign and data.sign.data == 'rturn':
            return 'r_turn'
        else:
            return 'forward'
        """elif sign == 'lturn':
            return 'l_turn'
        elif sign == 'uturn':
            return 'u_turn'
        elif sign == 'stop':
            return 'stop'"""

class R_turn(smach.State):
    def __init__(self, outcomes=['forward', 'preempted']):
        # state initialization goes here
        smach.State.__init__(self, outcomes=outcomes,
                             input_keys = ['sign'])
        self.certainty = 0

    def execute(self, data):
        # Check for preempt
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        sleep(5)

        if data.sign == 'rturn':
            self.certainty += 1
        else:
            self.certainty -= 1

        if self.certainty > 3:
            return 'r_turn'
        else:
            return 'forward'

class Stop(smach.State):
    def __init__(self, outcomes=[]):
        # state initialization goes here
        smach.State.__init__(self, outcomes=outcomes,
                             input_keys = ['sign'])

    def execute(self, data):
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

        self.sm.userdata.sign = msg

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

        # Create a SMACH state machine
        self.sm = smach.StateMachine(outcomes=[])

        self.sm.userdata.sign = None

        # Open the container
        with self.sm:
            # Add states to the container
            smach.StateMachine.add('forward', Forward(),
                                   transitions={'r_turn':'r_turn',
                                   'forward':'forward',
                                   'preempted': 'stop'})

            smach.StateMachine.add('r_turn', R_turn(),
                                   transitions={'forward':'forward',
                                   'preempted': 'stop'})

            smach.StateMachine.add('stop', Stop(),
                                   transitions={})

        # Execute SMACH plan
        # threading required for control-C-ability

        # Create a thread to execute the smach container
        smach_thread = threading.Thread(target=self.sm.execute)
        smach_thread.start()

        # Wait for ctrl-c
        rospy.spin()

        # Request the container to preempt
        self.sm.request_preempt()

        # Block until everything is preempted
        # (you could do something more complicated to get the execution outcome if you want it)
        smach_thread.join()
        #outcome = sm.execute()

if __name__ == '__main__':
    node = StreetSignFollower("/sign_predictions")
    node.run()
