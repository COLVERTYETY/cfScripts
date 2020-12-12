#!/usr/bin/env python

import logging
from pynput import keyboard


import numpy as np
from pycrazyswarm import *
import sys
import signal
import uav_trajectory
import rospy
from std_msgs.msg import String

from collections import deque
import statistics
from statistics import mode#, multimode
import math
import pyttsx3
global engine 




engine = pyttsx3.init()
Z = 0.3
sleepRate = 30

def signal_handler(signal, frame):
	sys.exit(0)

def speak(engine, text):
    

    # tts = gTTS(text=text, lang="en")
    # filename = "voice.mp3"
    # tts.save(filename)
    # playsound.playsound(filename)
    engine.say(text)
    engine.runAndWait()


class GestureDrone:

    def __init__(self, cfs):
        speak (engine, "Hand control mode activated.")
        #self.cf = cf
        

        self.velocity = 0.2
        self.velocity_base = 0.5
        self.velocity_filtered=1
        self.angle=0.0
        self.velocity_X=0
        self.velocity_Y=0
        self.velocity_Z=0
        self.releasevelocity = 1.0
        self.ang_velocity = 120
        self.takeoff_height = 0.5

        self.sleeptime = 3
        self.releasetime = 0.5
        self.msg= ''
        self.goToDuration=1.0
        self.followMode=False
        #self.max_hight = 0.8
        #self.hight = 0.0
        print ('SPIDERMAN Figure8!')
        print ('THUMBDOWN to land!')
        print ('UP to move up!')
        print ('DOWN to move down!')
        print ('RIGHT to move right!')
        print ('LEFT to move left!')
        print ('FIST emergency STOP')
        print('PEACE to go on follow mode')

        for cf in cfs:
            if cf.id == 1:
                    self.cf1 = cf
                    print("cf1 found")
                    self.cf1.takeoff(targetHeight=self.takeoff_height, duration=3.0)

            if cf.id == 2:
                    self.cf2 = cf
                    print("cf2 found")
                    self.cf2.takeoff(targetHeight=self.takeoff_height, duration=3.0)

            if cf.id == 3:
                    self.cf3 = cf
                    print("cf3 found")
                    self.cf3.takeoff(targetHeight=self.takeoff_height, duration=3.0)


        self.listener()

    def cf2_callback(self, msg):


        global lastGesture, lastSlide, timeHelper

        print(msg.data)
        if msg.data[0] == '1':
            print ('flying cf1. works.')
            self.cf = self.cf1

        if msg.data[0] == '2':
            print ('flying cf2. works.')
            self.cf = self.cf2


        if msg.data[0] == '3':
            print ('flying cf3. works.')
            self.cf = self.cf3

        msg.data = msg.data[1:]
        print(msg.data)
        
 
        if msg.data == 'GRAB' : #Activate followMODE
            #speak (engine, "Activating Hand following mode.")
            #print ("followMode ACTIVATED")
            self.followMode=True

        if msg.data == 'PEACE' : #Activate SignalMode: 
            #speak (engine, "Activating signal mode.")
            #Deactivate followMODE
            #print ("SignalMode ACTIVATED")
            self.followMode=False
            

        if self.followMode == False :

            if msg.data == 'SPIDERMAN':#start_up
                print("signalmode, spiderman.")
                for cf in allcfs.crazyflies:
                        cf.uploadTrajectory(0, 0, fig8)
                #self.cf1.takeoff(targetHeight=self.takeoff_height, duration=3.0)
                #self.cf2.takeoff(targetHeight=self.takeoff_height, duration=3.0)
                #self.cf3.takeoff(targetHeight=self.takeoff_height, duration=3.0)

            if msg.data == 'INDEX':#start_up
                print("signalmode, index.")
                for cf in allcfs.crazyflies:
                        cf.uploadTrajectory(0, 0, helicoidale)

            if msg.data == 'FUCK':
                
                speak (engine, "Fuck you bitch")
                
            if msg.data == 'FIST':#start_up
                print("FIST")
                self.cf.land(0.05, duration = 3.0)
                timeHelper.sleep(self.sleeptime)

            if msg.data == 'UP':#start_up
                self.velocity_Z=self.velocity
                self.velocity_X=0
                self.velocity_Y=0

            if msg.data == 'DOWN': #start_down
                self.velocity_Z=-self.velocity
                self.velocity_X=0
                self.velocity_Y=0

            if msg.data == 'RIGHT': #start_right
                self.velocity_X=self.velocity
                self.velocity_Z=0
                self.velocity_Y=0

            if msg.data == 'LEFT': #start_right
                self.velocity_X=-self.velocity
                self.velocity_Z=0
                self.velocity_Y=0

            if (msg.data == '' or msg.data == 'UNKNOWN'):
                print("UNKNOWN")

                
        if self.followMode == True:
            print("in FOLLOW mode")

            if msg.data == 'FIST':#start_up
                print("FIST")
                self.cf.land(0.05, duration = 3.0)
                timeHelper.sleep(self.sleeptime)

            else:
                
                if msg.data[0]=="A" :
                    self.angle=float(msg.data[1:])

                elif msg.data[0]=="V":
                    self.velocity_filtered=float(msg.data[1:])
   

                self.velocity_X=math.cos(self.angle)*self.velocity_base*self.velocity_filtered
                self.velocity_Z=math.sin(self.angle)*self.velocity_base*self.velocity_filtered
                self.velocity_Y=0

            self.cf.cmdVelocityWorld(np.array([-self.velocity_X, self.velocity_Y, self.velocity_Z]), yawRate=0)


        
    def listener(self):
        #rospy.init_node('drone_RTcommands', anonymous=True)
        handsignal_subscriber_cf3 = rospy.Subscriber('/cf2/filtredsignal', String, self.cf2_callback)
        
        #self.cf2_callback(cf2_pose)
        #handsignal_subscriber_cf3 = rospy.Subscriber('/cf3/signal', String, self.cf2_callback)

        #handslide_subscriber = rospy.Subscriber('/hand/direction', String, self.slide_callback)

        #cf.cmdVelocityWorld(np.array([self.velocity, 0, 0]), yawRate=0)
        
        rospy.spin()


signal.signal(signal.SIGINT, signal_handler)

if __name__ == '__main__':

    global d_slide, lastGesture, timeHelper
    d_slide = deque([], 10)
    lastGesture="None"

    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    #drone = KeyboardDrone(allcfs.crazyflies[0])
    #with keyboard.Listener(on_press=drone.on_press, on_release=drone.on_release) as listener:
    #     listener.join()
    drone = GestureDrone(allcfs.crazyflies)

####################################__Loading trajectories__################################################################

    fig8 = uav_trajectory.Trajectory()
    fig8.loadcsv("figure8.csv")


    traj2 = uav_trajectory.Trajectory()
    traj2.loadcsv("trajectory2.csv")

    #traj2 = uav_trajectory.Trajectory()
    #traj2.loadcsv("timed_waypoints_circle1.csv")

    cirlce = uav_trajectory.Trajectory()
    circle.loadcsv("circle_join_longer.csv")

    helicoidale = uav_trajectory.Trajectory()
    helicoidale.loadcsv("helicoidale.csv")
 
    rdev18_traj = uav_trajectory.Trajectory()
    rdev18_traj.loadcsv("demo_shapes/rdev_18deg.csv")

    ldev18_traj = uav_trajectory.Trajectory()
    ldev18_traj.loadcsv("demo_shapes/ldev_18deg.csv")

    f8xz_traj = uav_trajectory.Trajectory()
    f8xz_traj.loadcsv("demo_shapes/figure8_xz.csv")
    #try:
        #Testing our function
        #rospy.init_node('drone_RTcommands', anonymous=True)
        #handsignal_subscriber = rospy.Subscriber('/hand/signal', String, signal_callback())
        #handslide_subscriber = rospy.Publisher('/hand/direction', String, queue_size=10)
        #handforward_publisher = rospy.Publisher('/hand/forward', String, queue_size=10)

    #    execute()

    #except rospy.ROSInterruptException: pass
