#!/usr/bin/env python

import numpy as np
import rospy
import tf
from geometry_msgs.msg import TransformStamped, Point
from turtlesim.msg import Pose
from std_msgs.msg import String
#from tf2_msgs.msg import TFMessage
import tf2_msgs.msg
import sys
import signal

from pycrazyswarm import *
import uav_trajectory
import actionlib
import actionlib_tutorials.msg

from math import pow, atan2, sqrt

def signal_handler(signal, frame):
	sys.exit(0)

class ArenaFlyer(object):

    def __init__(self, name):

        self.swarm = Crazyswarm()
        #self.timeHelper = swarm.timeHelper
        self.allcfs = self.swarm.allcfs

        # when a message of type Pose is received.
        #self.ttl1_subscriber = rospy.Subscriber('/turtle1/pose',
        #                                                Pose, self.update_pose1)

        self.cf2_subscriber = rospy.Subscriber('/tf', tf2_msgs.msg.TFMessage, self.cf2_callback)
        self.cf2_pose = Point()

        self.success = False

        self._feedback = actionlib_tutorials.msg.FibonacciFeedback()
        self._result = actionlib_tutorials.msg.FibonacciResult()
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, actionlib_tutorials.msg.FibonacciAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    # def update_pose1(self, data):
    #     """Callback function which is called when a new message of type Pose is
    #     received by the subscriber."""
    #     #print("UPDATE TT1")
    #     self.turtle1_pose = data
    #     self.turtle1_pose.x = round(self.turtle1_pose.x, 4)
    #     self.turtle1_pose.y = round(self.turtle1_pose.y, 4)
	#     #print("turtle1:")

    def cf2_callback(self, cf2):

    	#cf2 is a : cf2=tf2_msgs.msg.TFMessage([t])
    	#with t : t=geometry_msgs.msg.TransformStamped()

    	#create cf to be a TransformStamped:
    	cf = cf2.transforms[0]

    	self.cf2_pose.x = cf.transform.translation.x
    	self.cf2_pose.y = cf.transform.translation.y
    	self.cf2_pose.z = cf.transform.translation.z
    	#print("cf2_pose:", self.cf2_pose)


    def execute_cb(self, goal):
        # helper variables
        #r = rospy.Rate(10)
        rospy.wait_for_message('/tf', tf2_msgs.msg.TFMessage, timeout=None)

        # append the seeds for the fibonacci sequence
        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)
        self.success == False

        # publish info to the console for the user
        #rospy.loginfo('%s: Now with tolerance %i with current pose [%s]' % (self._action_name, goal.order, ','.join(map(str,self._feedback.sequence))))

        # check that preempt has not been requested by the client


        # start executing the action
        # x = TurtleBot()

        for cf in self.allcfs.crazyflies:
            print(cf.id)
            cf.takeoff(0.5, 5.0)
        #     Z = 0.0
	    # while Z < 1.0:
		#    pos = np.array([0, 0, Z])
	    #        cf.goTo(pos, yaw=0, duration=0.5)
		#    #timeHelper.sleep(0.1)
		#    Z += 0.05
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break


            #now we test if he has reached the desired point.
        self.takeoff_transition()


        if self.success == False:

            print ("Not yet...")

            try:

                self._feedback.sequence.append(currentPose)
                # publish the feedback
                self._as.publish_feedback(self._feedback)
                #rospy.loginfo('%s: Now with tolerance %i with current pose [%s]' % (self._action_name, goal.order, ','.join(map(str,self._feedback.sequence))))

            except rospy.ROSInterruptException:
                print("except clause opened")
                pass


        if self.success == True:
            for cf in self.allcfs.crazyflies:
                #print(cf.id)
                #print("press button to continue")
                #self.swarm.input.waitUntilButtonPressed()
                cf.land(0.04, 2.5)
            print("Reached the perimeter!!")
            self._result.sequence = self._feedback.sequence
            rospy.loginfo('My feedback: %s' % self._feedback)
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        euclidean_distance= sqrt(pow((goal_pose.x - self.cf2_pose.x), 2) + pow((goal_pose.y - self.cf2_pose.y), 2) + pow((goal_pose.z - self.cf2_pose.z), 2))
    	#print("distance to goal is", round(euclidean_distance, 4))
    	return euclidean_distance

    def takeoff_transition(self):
        distance_tolerance = 0.1
        goal = Point()
        goal.x = 0.0
        goal.y = 0.0
        goal.z = 0.5

        while self.euclidean_distance(goal) >= distance_tolerance:
            self.success = False

        if self.euclidean_distance(goal) < distance_tolerance:
            self.success = True
            print("success is", self.success)
            #return success

signal.signal(signal.SIGINT, signal_handler)

if __name__ == "__main__":
    #rospy.init_node('detect_perimeter') # this is a maybe

    #swarm = Crazyswarm()
    #timeHelper = swarm.timeHelper
    #allcfs = swarm.allcfs
    drone_server = ArenaFlyer('detect_perimeter')
    #server = FibonacciAction('detect_perimeter')
    rospy.spin()
