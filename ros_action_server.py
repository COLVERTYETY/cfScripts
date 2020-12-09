#!/usr/bin/python

import numpy as np
import rospy
from rospy import Duration

import tf
from geometry_msgs.msg import TransformStamped, Point, Pose
import time
import turtlesim.msg
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
#import tf2_msgs.msg
import sys
import signal

from pycrazyswarm import *
import uav_trajectory
import actionlib
import actionlib_tutorials.msg
import actionlib_tutorials.srv

from math import pow, atan2, sqrt
# from gtts import gTTS
# import playsound
# try:
#     from urllib.parse import urlparse
# except ImportError:
#      from urlparse import urlparse

import pyttsx3
global engine
engine = pyttsx3.init()


#from six.moves.urllib import parse

def signal_handler(signal, frame):
	sys.exit(0)

def speak(engine, text):


    # tts = gTTS(text=text, lang="en")
    # filename = "voice.mp3"
    # tts.save(filename)
    # playsound.playsound(filename)
    engine.say(text)
    engine.runAndWait()

class PerimeterMonitor(object):

    def __init__(self, name):
        print("name is", name)
        #self._action_name = name
        self.swarm = Crazyswarm()
        self.allcfs = self.swarm.allcfs
        #speak (engine, "Action server activated.")
        self.cf2_pose = Point()
        self.cf3_pose = Point()

        self.success = False

        self._feedback2 = actionlib_tutorials.msg.doTrajFeedback()
        self._result2 = actionlib_tutorials.msg.doTrajResult()
        self._action_name2 = 'trajectory_action'
        print (self._action_name2)
        self._as2 = actionlib.SimpleActionServer(self._action_name2, actionlib_tutorials.msg.doTrajAction, execute_cb=self.execute_cb2, auto_start = False)
        self._as2.start()

        self._feedback_cf3 = actionlib_tutorials.msg.my_newFeedback()
        self._result_cf3 = actionlib_tutorials.msg.my_newResult()
        self._action_name_cf3 = 'detect_perimeter1'
        print (self._action_name_cf3)
        self._as_cf3 = actionlib.SimpleActionServer(self._action_name_cf3, actionlib_tutorials.msg.my_newAction, execute_cb=self.execute_cb_cf3, auto_start = False)
        self._as_cf3.start()
        print("Ready to move _cf3.")

        self._feedback_cf2 = actionlib_tutorials.msg.my_newFeedback()
        self._result_cf2 = actionlib_tutorials.msg.my_newResult()
        self._action_name_cf2 = 'detect_perimeter'
        print (self._action_name_cf2)
        self._as_cf2 = actionlib.SimpleActionServer(self._action_name_cf2, actionlib_tutorials.msg.my_newAction, execute_cb=self.execute_cb_cf2, auto_start = False)
        self._as_cf2.start()
        print("Ready to move _cf2.")

        #SERVICES: Check if Services work on Action Server (ROS2 functionality)
        #self.setupKillService()


    """MUST BE UPDATED"""
    def handleKillService(self, req):
        for cf in self.allcfs.crazyflies:
            if cf.id == 2:
                print(cf.id)
                cf.cmdStop()
        #print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
        return 1.0




    def PoseListener(self):
        # A subscriber to the topic '/tf'. self.update_pose is called
        # when a message of type TFMessage is received.

		#self.cf2_subscriber = rospy.Subscriber('/tf', tf2_msgs.msg.TFMessage, self.cf2_callback)

        cf2_pose = rospy.wait_for_message("/tf", TFMessage, timeout=None)
        self.cf2_callback(cf2_pose)

        # while cf2_pose.transforms[0].child_frame_id != 'cf2':
        #     cf2_pose = rospy.wait_for_message("/tf", TFMessage, timeout=None)


        # while cf2_pose.transforms[0].child_frame_id != 'cf3':
        #     cf2_pose = rospy.wait_for_message("/tf", TFMessage, timeout=None)
        # self.cf2_callback(cf2_pose)


    def setupKillService(self):
        s = rospy.Service('kill_service', actionlib_tutorials.srv.killMotors(), self.handleKillService)
        self._as.start()
        print("ready to kill.")

    def cf2_callback(self, data):
    	#create cf to be a TransformStamped:
    	cf = data.transforms[0]
        if cf.child_frame_id == 'cf2':
            self.cf2_pose.x = cf.transform.translation.x
            self.cf2_pose.y = cf.transform.translation.y
            self.cf2_pose.z = cf.transform.translation.z
            print("cf2_pose: received")
        if cf.child_frame_id == 'cf3':
            self.cf3_pose.x = cf.transform.translation.x
            self.cf3_pose.y = cf.transform.translation.y
            self.cf3_pose.z = cf.transform.translation.z
            print("cf3_pose: received")


    def execute_cb2(self, goal):
        speak (engine, "Spiral trajectory.")
        traj1 = uav_trajectory.Trajectory()
        traj1.loadcsv("helicoidale.csv")
       # helper variables
        #r = rospy.Rate(10)
        rospy.wait_for_message('/tf', tf2_msgs.msg.TFMessage, timeout=None)

        # append the seeds for the fibonacci sequence
        self._feedback2.time_elapsed = Duration(5)
        self.success == False

        TRIALS = 1
        TIMESCALE = 0.5
        for cf in self.allcfs.crazyflies:
            cf.takeoff(targetHeight=0.6, duration=3.0)
            cf.uploadTrajectory(0, 0, traj1)
            #timeHelper.sleep(2.5)
            self.allcfs.crazyflies[0].startTrajectory(0, timescale=TIMESCALE)
            #timeHelper.sleep(1.0)
            print("press button to continue...")
            self.swarm.input.waitUntilButtonPressed()

            self.allcfs.crazyflies[0].land(targetHeight=0.06, duration=2.0)
            #timeHelper.sleep(3.0)

            if self._as2.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name2)
                self._as2.set_preempted()
                success = False
                break
            #now we test if he has reached the desired point.
        #self.takeoff_transition()


        if self.success == False:

            print ("Not yet...")

            try:

                ###self._feedback.sequence.append(currentPose)
                # publish the feedback
                self._as2.publish_feedback(self._feedback2)
                #rospy.loginfo('%s: Now with tolerance %i with current pose [%s]' % (self._action_name, goal.order, ','.join(map(str,self._feedback.sequence))))

            except rospy.ROSInterruptException:
                print("except clause opened")
                rospy.loginfo('Feedback did not go through.')
                pass


        if self.success == True:
            for cf in self.allcfs.crazyflies:
                cf.land(0.04, 2.5)
            print("Reached the perimeter!!")

            self._result2.time_elapsed = Duration(5)
            self._result2.updates_n = 1
            rospy.loginfo('My feedback: %s' % self._feedback2)
            rospy.loginfo('%s: Succeeded' % self._action_name2)
            self._as2.set_succeeded(self._result2)



    def execute_cb_cf2(self, goal):
        """togoal ACTION SERVER [name='togoal', variable:'_as_cf2', drone: cf2]"""

        #speak (engine, "Moving to point.")

        self.PoseListener()

        print ("CF2 Action Server callback")
        print ("point is", goal.point)
        print("id is " + str(goal.id))

        self._feedback_cf2.position = Pose()
        self._feedback_cf2.time_elapsed = Duration(5)

        self.success_cf2 = False
        self.waypoint = np.array([goal.point.x, goal.point.y, goal.point.z])

        #speak (engine, "YO")
        for cf in self.allcfs.crazyflies:
            if cf.id == goal.id:             
                print("send COMMANDS to cf...", goal.id)
                self._feedback_cf2.position.position.x = cf.position()[0]
                self._feedback_cf2.position.position.y = cf.position()[1]
                self._feedback_cf2.position.position.z = cf.position()[2]
                cf.takeoff(0.5, 5.0)
                cf.goTo(self.waypoint, yaw=0, duration=5.0)

        while self.success_cf2 == False:
            self.PoseListener()


            if self._as_cf2.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name_cf2)
                self._as_cf2.set_preempted()
                for cf in self.allcfs.crazyflies:
                    if cf.id == goal.id:             
                        print("LANDING cf...", goal.id)
                        cf.land(0.04, 2.5)
                break

            print ("Not yet...")
            #now we test if he has reached the desired point.
            self.takeoff_transition(goal.id, goal.point)

            try:
                # publish the feedback
                self._as_cf2.publish_feedback(self._feedback_cf2)

            except rospy.ROSInterruptException:
                rospy.loginfo("except clause opened")
                pass


        if self.success_cf2 == True:
            #for cf in self.allcfs.crazyflies:
                #print(cf.id)
                #print("press button to continue")
                #self.swarm.input.waitUntilButtonPressed()
                #cf.land(0.04, 2.5)
            print("Reached the perimeter!!")
            self.success_cf2 = False
            self._result_cf2.time_elapsed = Duration(5)
            self._result_cf2.updates_n = 1
            rospy.loginfo('My feedback: %s' % self._feedback_cf2)
            rospy.loginfo('%s: Succeeded' % self._action_name_cf2)
            self._as_cf2.set_succeeded(self._result_cf2)



    def execute_cb_cf3(self, goal):
        """togoal ACTION SERVER [name='togoal', variable:'_as_cf2', drone: cf2]"""
        #speak (engine, "Moving to point.")

        #rospy.wait_for_message('/tf', tf2_msgs.msg.TFMessage, timeout=None)
        self.PoseListener()

        print ("CF2 Action Server callback")
        print ("point is", goal.point)
        print("id is " + str(goal.id))

        self._feedback_cf3.position = Pose()
        self._feedback_cf3.time_elapsed = Duration(5)

        self.success_cf3 = False
        self.waypoint = np.array([goal.point.x, goal.point.y, goal.point.z])

        #speak (engine, "YO")
        for cf in self.allcfs.crazyflies:
            if cf.id == goal.id:             
                print("send COMMANDS to cf...", goal.id)
                self._feedback_cf3.position.position.x = cf.position()[0]
                self._feedback_cf3.position.position.y = cf.position()[1]
                self._feedback_cf3.position.position.z = cf.position()[2]
                cf.takeoff(0.5, 5.0)
                cf.goTo(self.waypoint, yaw=0, duration=5.0)

        while self.success_cf3 == False:
            self.PoseListener()


            if self._as_cf3.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name_cf3)
                self._as_cf3.set_preempted()
                for cf in self.allcfs.crazyflies:
                    if cf.id == goal.id:             
                        print("LANDING cf...", goal.id)
                        cf.land(0.04, 2.5)
                break

            print ("Not yet...")
            #now we test if he has reached the desired point.
            self.takeoff_transition(goal.id, goal.point)

            try:
                # publish the feedback
                self._as_cf3.publish_feedback(self._feedback_cf3)

            except rospy.ROSInterruptException:
                rospy.loginfo("except clause opened")
                pass


        if self.success_cf3 == True:
            #for cf in self.allcfs.crazyflies:
                #print(cf.id)
                #print("press button to continue")
                #self.swarm.input.waitUntilButtonPressed()
                #cf.land(0.04, 2.5)
            print("Reached the perimeter!!")
            self.success_cf3 = False
            self._result_cf3.time_elapsed = Duration(5)
            self._result_cf3.updates_n = 1
            rospy.loginfo('My feedback: %s' % self._feedback_cf3)
            rospy.loginfo('%s: Succeeded' % self._action_name_cf3)
            self._as_cf3.set_succeeded(self._result_cf3)




    """ CALCULATION FUNCTIONS """

    def euclidean_distance(self, goal_pose, id):
        """Euclidean distance between current pose and the goal."""
        if id == 2:
            print("distance for id 2")
            euclidean_distance= sqrt(pow((goal_pose.x - self.cf2_pose.x), 2) + pow((goal_pose.y - self.cf2_pose.y), 2) + pow((goal_pose.z - self.cf2_pose.z), 2))
        elif id == 3:
            print("distance for id 3")
            euclidean_distance= sqrt(pow((goal_pose.x - self.cf3_pose.x), 2) + pow((goal_pose.y - self.cf3_pose.y), 2) + pow((goal_pose.z - self.cf3_pose.z), 2))
        else:
            print ("no id detected... aborting?", id)
        #euclidean_distance= sqrt(pow((goal_pose.x - self.cf2_pose.x), 2) + pow((goal_pose.y - self.cf2_pose.y), 2) + pow((goal_pose.z - self.cf2_pose.z), 2))
        print("distance to goal is", round(euclidean_distance, 4))
        return euclidean_distance

    def takeoff_transition(self, id, goal):
        distance_tolerance = 0.2

        if self.euclidean_distance(goal, id) < distance_tolerance:
            if id == 2:
                self.success_cf2 = True
            if id == 3:
                self.success_cf3 = True

signal.signal(signal.SIGINT, signal_handler)

if __name__ == "__main__":
    #rospy.init_node('hi') # CANNOT do this: conflicts with CrazyflieAPI.

    print ("Server name is:", rospy.get_name())
    perimeter_server = PerimeterMonitor(str(rospy.get_name())+str(1))
    rospy.spin()
