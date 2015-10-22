#!/usr/bin/env python

""" Listens on the street sign prediction topic and uses the resultant
    predictions to obey street signs. """

from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point
from std_msgs.msg import Header
from std_msgs.msg import String
import threading
import smach
import smach_ros
import rospy

###### State Machine States ######

class Forward(smach.State):
    #def __init__(self, outcomes=['r_turn', 'l_turn', 'u_turn', 'stop']):
    def __init__(self, outcomes=['r_turn', 'forward', 'preempted']):
        # state initialization goes here
        smach.State.__init__(self, outcomes=outcomes)
        pass

    def execute(self, sign):
        # Check for preempt
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        if sign == 'rturn':
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
        smach.State.__init__(self, outcomes=outcomes)
        self.certainty = 0

    def execute(self, sign):
        # Check for preempt
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        if sign == 'rturn':
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
        smach.State.__init__(self, outcomes=outcomes)

    def execute(self, sign):
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

        point_msg = Point(x=1.0, y=2.0, z=0.0)
        quat_msg = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        pose_msg = Pose(position=point_msg, orientation=quat_msg)

        header_msg = Header(stamp=rospy.Time.now(),
                                    frame_id=0)

        pose_stamped = PoseStamped(header=header_msg, pose=pose_msg)

        self.pub.Publish(pose_stamped)

    def run(self):
        """ The main run loop - create a state machine, and set it off """
        """r = rospy.Rate(5)
        while not rospy.is_shutdown():
            r.sleep()"""

        # Create a SMACH state machine
        sm = smach.StateMachine(outcomes=[])

        # Open the container
        with sm:
            # Add states to the container
            smach.StateMachine.add('Forward', Forward(),
                                   transitions={'r_turn':'R_turn',
                                   'forward':'Forward',
                                   'preempted': 'Stop'})
            smach.StateMachine.add('R_turn', R_turn(),
                                   transitions={'forward':'Forward',
                                   'preempted': 'Stop'})

            smach.StateMachine.add('Stop', Stop(),
                                   transitions={})

        # Execute SMACH plan
        # threading required for control-C-ability

        # Create a thread to execute the smach container
        smach_thread = threading.Thread(target=sm.execute)
        smach_thread.start()

        # Wait for ctrl-c
        rospy.spin()

        # Request the container to preempt
        sm.request_preempt()

        # Block until everything is preempted
        # (you could do something more complicated to get the execution outcome if you want it)
        smach_thread.join()
        #outcome = sm.execute()

if __name__ == '__main__':
    node = StreetSignFollower("/sign_predictions")
    node.run()
