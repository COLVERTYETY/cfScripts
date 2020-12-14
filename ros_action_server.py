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
        self.timeHelper = self.swarm.timeHelper
        #speak (engine, "Action server activated.")
        self.cf1_pose = Point()
        self.cf2_pose = Point()
        self.cf3_pose = Point()
        self.desired_position = Point()
        #Arm the active drones that you wish to fly.
        self.enable_cf1 = False
        self.enable_cf2 = False
        self.enable_cf3 = False
        #Quick position verification.




        self.success = False
        self.collision_tolerance = 0.2
        self.sleeptime = 4.0
        self.justenoughsleeptime = 0.005

        for cf in self.allcfs.crazyflies:
            if cf.id == 3 and self.enable_cf3 == True:            
                cf.takeoff(0.5, 5.0)
            if cf.id == 2 and self.enable_cf2 == True:            
                cf.takeoff(0.5, 5.0)

        self._feedback2 = actionlib_tutorials.msg.doTrajFeedback()
        self._result2 = actionlib_tutorials.msg.doTrajResult()
        self._action_name2 = 'trajectory_action'
        print (self._action_name2)
        self._as2 = actionlib.SimpleActionServer(self._action_name2, actionlib_tutorials.msg.doTrajAction, execute_cb=self.execute_cb2, auto_start = False)
        self._as2.start()

        self._feedback_cf3_go = actionlib_tutorials.msg.my_newFeedback()
        self._result_cf3_go = actionlib_tutorials.msg.my_newResult()
        self._action_name_cf3_go = 'detect_perimeter1'
        print (self._action_name_cf3_go)
        self._as_cf3_go = actionlib.SimpleActionServer(self._action_name_cf3_go, actionlib_tutorials.msg.my_newAction, execute_cb=self.execute_cb_cf3_go, auto_start = False)
        self._as_cf3_go.start()
        print("Ready to move _cf3.")

        self._feedback_cf2 = actionlib_tutorials.msg.my_newFeedback()
        self._result_cf2 = actionlib_tutorials.msg.my_newResult()
        self._action_name_cf2 = 'detect_perimeter'
        print (self._action_name_cf2)
        self._as_cf2 = actionlib.SimpleActionServer(self._action_name_cf2, actionlib_tutorials.msg.my_newAction, execute_cb=self.execute_cb_cf2, auto_start = False)
        self._as_cf2.start()
        print("Ready to move _cf2.")

        self._feedback_cf3 = actionlib_tutorials.msg.my_newFeedback()
        self._result_cf3 = actionlib_tutorials.msg.my_newResult()
        self._action_name_cf3 = 'cf3_follow_cf2'
        print (self._action_name_cf3)
        self._as_cf3 = actionlib.SimpleActionServer(self._action_name_cf3, actionlib_tutorials.msg.my_newAction, execute_cb=self.execute_cb_cf3_follow_cf2, auto_start = False)
        self._as_cf3.start()
        print("Ready to follow cf3_cf2.")



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

        #cf2_pose = rospy.wait_for_message("/tf", TFMessage, timeout=None)
        self.cf2_callback()



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
    	# cf = data.transforms[0]
        # if cf.child_frame_id == 'cf2':
        #     self.cf2_pose.x = cf.transform.translation.x
        #     self.cf2_pose.y = cf.transform.translation.y
        #     self.cf2_pose.z = cf.transform.translation.z
        #     print("cf2_pose: received")
        # if cf.child_frame_id == 'cf3':
        #     self.cf3_pose.x = cf.transform.translation.x
        #     self.cf3_pose.y = cf.transform.translation.y
        #     self.cf3_pose.z = cf.transform.translation.z
        #     print("cf3_pose: received")
        """GET POSE FROM CF API"""
        for cf in self.allcfs.crazyflies:
            #print(cf.id, cf.position()[0])
            if cf.id == 1:
                self.cf1_pose.x = cf.position()[0]
                self.cf1_pose.y = cf.position()[1]
                self.cf1_pose.z = cf.position()[2]
                #print("cf1_pose: received")
            if cf.id == 2:
                self.cf2_pose.x = cf.position()[0]
                self.cf2_pose.y = cf.position()[1]
                self.cf2_pose.z = cf.position()[2]
                #print("cf2_pose: received")
            if cf.id == 3:
                self.cf3_pose.x = cf.position()[0]
                self.cf3_pose.y = cf.position()[1]
                self.cf3_pose.z = cf.position()[2]
                #print("cf3_pose: received")

        #self.waypoint = np.array([self.cf3_pose.x, self.cf3_pose.y + 0.3, self.cf3_pose.z])
        #print("waypoint is", self.waypoint)

        """CHECK FOR COLLISIONS BETWEEN CF2 AND CF3"""
        collision_distance = self.euclidean_distance(self.cf1_pose, 2)
        print ("DISTANCE IS", collision_distance)
        if collision_distance < self.collision_tolerance:
            print ("COLLISION AT", collision_distance)
            collision_publisher.publish("home")
            # for cf in self.allcfs.crazyflies:
            #     cf.land(0.04, 2.5)
            # rospy.sleep(self.sleeptime)

        #and id 2 exists...
        #rospy.spin()
        







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
        self.enable_cf2 = False
        self.waypoint = np.array([goal.point.x, goal.point.y, goal.point.z])

        #speak (engine, "YO")
        for cf in self.allcfs.crazyflies:
            if cf.id == goal.id:             
                print("send COMMANDS to cf...", goal.id)
                self._feedback_cf2.position.position.x = cf.position()[0]
                self._feedback_cf2.position.position.y = cf.position()[1]
                self._feedback_cf2.position.position.z = cf.position()[2]
                #ENABLE:
                if self.enable_cf2 == True:
                    cf.goTo(self.waypoint, yaw=0, duration=5.0)

        while self.success_cf2 == False:
            self.PoseListener()


            if self._as_cf2.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name_cf2)
                self._as_cf2.set_preempted()
                # for cf in self.allcfs.crazyflies:
                #     if cf.id == goal.id:             
                #         print("LANDING cf...", goal.id)
                #         cf.land(0.04, 2.5)
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



    def execute_cb_cf3_go(self, goal):
        """togoal ACTION SERVER [name='togoal', variable:'_as_cf2', drone: cf2]"""
        #speak (engine, "Moving to point.")

        #rospy.wait_for_message('/tf', tf2_msgs.msg.TFMessage, timeout=None)
        self.PoseListener()

        print ("CF2 Action Server callback")
        print ("point is", goal.point)
        print("id is " + str(goal.id))

        self._feedback_cf3_go.position = Pose()
        self._feedback_cf3_go.time_elapsed = Duration(5)

        self.success_cf3 = False
        self.waypoint = np.array([goal.point.x, goal.point.y, goal.point.z])

        #speak (engine, "YO")
        for cf in self.allcfs.crazyflies:
            if cf.id == goal.id:             
                print("send COMMANDS to cf...", goal.id)
                self._feedback_cf3_go.position.position.x = cf.position()[0]
                self._feedback_cf3_go.position.position.y = cf.position()[1]
                self._feedback_cf3_go.position.position.z = cf.position()[2]
                cf.takeoff(0.5, 5.0)
                cf.goTo(self.waypoint, yaw=0, duration=5.0)

        while self.success_cf3 == False:
            self.PoseListener()


            if self._as_cf3_go.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name_cf3_go)
                self._as_cf3_go.set_preempted()
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
                self._as_cf3_go.publish_feedback(self._feedback_cf3_go)

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
            self._result_cf3_go.time_elapsed = Duration(5)
            self._result_cf3_go.updates_n = 1
            rospy.loginfo('My feedback: %s' % self._feedback_cf3_go)
            rospy.loginfo('%s: Succeeded' % self._action_name_cf3_go)
            self._as_cf3_go.set_succeeded(self._result_cf3_go)




    def execute_cb_cf3_follow_cf2(self, goal):
        """togoal ACTION SERVER [name='togoal', variable:'_as_cf2', drone: cf2]"""
        #speak (engine, "Moving to point.")

        #rospy.wait_for_message('/tf', tf2_msgs.msg.TFMessage, timeout=None)
        self.PoseListener()

        print ("FOLLOW callback")
        print ("point is", goal.point)
        print("id is " + str(goal.id))

        self._feedback_cf3.position = Pose()
        self._feedback_cf3.time_elapsed = Duration(5)

        self.success_cf3 = False
        y_offset = 0
        x_offset = -0.3



        speak (engine, "YO")
        # for cf in self.allcfs.crazyflies:
        #     if cf.id == goal.id:             
        #         print("send COMMANDS to cf...", goal.id)
        #         #cf.takeoff(0.5, 5.0)
        #         #cf.goTo(self.waypoint, yaw=0, duration=2.0)

        while self.success_cf3 == False:
            self.PoseListener()
            print("in false loop...")


            if self._as_cf3.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name_cf3)
                self._as_cf3.set_preempted()
                for cf in self.allcfs.crazyflies:
                    if cf.id == goal.id:             
                        print("LANDING cf...", goal.id)
                        cf.land(0.04, 2.5)
                break

            print ("FOLLOW Not yet...")

            self.waypoint = np.array([self.cf2_pose.x + x_offset, self.cf2_pose.y + y_offset, self.cf2_pose.z])
            print("waypoint is", self.waypoint)

            for cf in self.allcfs.crazyflies:
                if cf.id == goal.id:             
                    #print("TRYING TO REACH GOAL...", goal.id)
                    #cf.takeoff(0.5, 5.0)
                    cf.goTo(self.waypoint, yaw=0, duration=self.justenoughsleeptime)
                    #rospy.sleep(self.justenoughsleeptime)


            self.stayclose_transition() #STOP WHEN HAND SIGN APPEARS.

            # try:
            #     # publish the feedback
            #     self._as_cf3.publish_feedback(self._feedback_cf3)

            # except rospy.ROSInterruptException:

            #     speak (engine, "EXCEPTION")

            #     rospy.loginfo("except clause opened")
            #     pass


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
            #print("distance for id 2")
            euclidean_distance= sqrt(pow((goal_pose.x - self.cf2_pose.x), 2) + pow((goal_pose.y - self.cf2_pose.y), 2) + pow((goal_pose.z - self.cf2_pose.z), 2))
        elif id == 3:
            #print("distance for id 3")
            euclidean_distance= sqrt(pow((goal_pose.x - self.cf3_pose.x), 2) + pow((goal_pose.y - self.cf3_pose.y), 2) + pow((goal_pose.z - self.cf3_pose.z), 2))
        else:
            print ("no id detected... aborting?", id)
        #euclidean_distance= sqrt(pow((goal_pose.x - self.cf2_pose.x), 2) + pow((goal_pose.y - self.cf2_pose.y), 2) + pow((goal_pose.z - self.cf2_pose.z), 2))
        print("offset_x:", goal_pose.x - self.cf2_pose.x)
        print("offset_y:", goal_pose.y - self.cf2_pose.y)
        print("offset_z:", goal_pose.z - self.cf2_pose.z)
        return euclidean_distance

    def takeoff_transition(self, id, goal):
        distance_tolerance = 0.2

        if self.euclidean_distance(goal, id) < distance_tolerance:
            if id == 2:
                self.success_cf2 = True
            if id == 3:
                self.success_cf3 = True

    def stayclose_transition(self):
        # distance_tolerance = 0.5
        # if self.euclidean_distance(goal, id) > distance_tolerance:
        #     if id == 2:
        #         self.success_cf2 = True
        #     if id == 3:
        #         self.success_cf3 = True  
        try:
            follow_trigger = rospy.wait_for_message("/swarmfollow", String, timeout=0.3)
            if follow_trigger.data == "STOP_FOLLOW_ME":
                success_cf3 = True
        except:
            pass

signal.signal(signal.SIGINT, signal_handler)

if __name__ == "__main__":
    #rospy.init_node('hi') # CANNOT do this: conflicts with CrazyflieAPI.

    print ("Server name is:", rospy.get_name())
    collision_publisher = rospy.Publisher('/collision', String, queue_size=10)
    perimeter_server = PerimeterMonitor(str(rospy.get_name())+str(1))
    #trying a quick pos_verification.
    position_subscriber = rospy.Subscriber('/tf', TFMessage, perimeter_server.cf2_callback)
    #Adding this to callback CONTINUOUSLY... 
    rospy.spin()
