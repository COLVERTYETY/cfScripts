#!/usr/bin/env python

import logging
from pynput import keyboard


import numpy as np
from pycrazyswarm import *
import sys
import signal

import rospy
from std_msgs.msg import String

from collections import deque
import statistics
from statistics import mode#, multimode

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
        self.velocity2 = 0.5
        self.releasevelocity = 1.0
        self.ang_velocity = 120
        self.takeoff_height = 0.5

        self.sleeptime = 0.5
        self.releasetime = 0.5
        self.msg= ''
        self.goToDuration=1.0
        self.followMode=False
        #self.max_hight = 0.8
        #self.hight = 0.0
        print ('SPIDERMAN to take off!')
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


        global lastGesture, lastSlide

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
        
        # while abs(self.cf.position()[0])>1.0 or abs(self.cf.position()[1])>1.0 or abs(self.cf.position()[0])>1.0:
        #     print ("go home now")

        #print (self.cf.position())
        if msg.data == 'GRAB' : #Activate followMODE
            #speak (engine, "Activating Hand following mode.")
            #print ("followMode ACTIVATED")
            self.followMode=True

        if msg.data == 'PEACE' : #Activate SignalMode: 
            #speak (engine, "Activating signal mode.")
            #Deactivate followMODE
            #print ("SignalMode ACTIVATED")
            self.followMode=False
            

        #print ("FollowMODE is", self.followMode)
        
        #if self.followMode==False:
            # if signal == 'w': #start_forward
            #     self.cf.cmdVelocityWorld(np.array([self.velocity, 0, 0]), yawRate=0)
            # if msg.data == 'THREE' :#start_back
            #     self.cf.cmdVelocityWorld(np.array([-self.velocity, 0, 0]), yawRate=0)

            # if msg.data == 'TWO': #start_forward
            #     self.cf.cmdVelocityWorld(np.array([self.velocity, 0, 0]), yawRate=0)

        #self.followMode="TT"
        if self.followMode == False :


            if msg.data == 'SPIDERMAN':#start_up
                print("signalmode, spiderman.")
                self.cf1.takeoff(targetHeight=self.takeoff_height, duration=3.0)
                #self.cf2.takeoff(targetHeight=self.takeoff_height, duration=3.0)
                #self.cf3.takeoff(targetHeight=self.takeoff_height, duration=3.0)

                
            if msg.data == 'FIST':#start_up
                print("FIST")
                #for cf in self.cf:
                self.cf.land(0.05, duration = 3.0)
                timeHelper.sleep(self.sleeptime)

            if msg.data == 'UP':#start_up
                print(".")
                self.cf.cmdVelocityWorld(np.array([0, 0, self.velocity]), yawRate=0)


            if msg.data == 'DOWN': #start_down
                print(".")
                self.cf.cmdVelocityWorld(np.array([0, 0, -self.velocity]), yawRate=0)


            if msg.data == 'RIGHT': #start_right
                print(".")
                self.cf.cmdVelocityWorld(np.array([0, -self.velocity, 0]), yawRate=0)


            if msg.data == 'LEFT': #start_right
                print(".")
                self.cf.cmdVelocityWorld(np.array([0, self.velocity, 0]), yawRate=0)


            # if signal == 'c': #start_down
            #     self.cf.cmdVelocityWorld(np.array([0, 0, -self.velocity]), yawRate=0)
            # if signal == 'z': #start_up
            #     self.cf.cmdVelocityWorld(np.array([0, 0, self.velocity]), yawRate=0)
            if (msg.data == '' or msg.data == 'UNKNOWN' or msg.data == 'FIST'):
                print(".")

                

                    #if key.char == 'q':
                    #    self.cf.start_turn_left(self.ang_velocity)
                    #if key.char == 'e':
                    #    self.cf.start_turn_right(self.ang_velocity)

                # def on_release (self, key):
                #     self.cf.cmdVelocityWorld(np.array([0, 0, 0]), yawRate=0)

                # def slide_callback(self, msg):
                #     #print(msg.data)
                #     #print (self.cf.position()[0])

                #     new = self.addToQueueAndAverage(d_slide, msg.data)
                #     #print("HI", new)


        if self.followMode == True:
            print("in FOLLOW mode")

            if msg.data == 'FIST':#start_up
                print("FIST")
                self.cf.land(0.05, duration = 3.0)
                timeHelper.sleep(self.sleeptime)

            # elif msg.data == 'UP' and lastGesture == 'GRAB':
            #     print("!!!!!!! RELEASE !!!!!!!!.")
            #     print("MOVING IN DIRECTION:", lastSlide)
            #     if lastSlide == 'LEFT SLIDE':
            #         self.cf.cmdVelocityWorld(np.array([self.releasevelocity, 0, 0]), yawRate=0)
            #         timeHelper.sleep(self.releasetime)
            #         #stop
            #         self.cf.cmdVelocityWorld(np.array([0, 0, 0]), yawRate=0)
            #     if lastSlide == 'RIGHT SLIDE':
            #         self.cf.cmdVelocityWorld(np.array([-self.releasevelocity, 0, 0]), yawRate=0)
            #         timeHelper.sleep(self.releasetime)
            #         #stop
            #         self.cf.cmdVelocityWorld(np.array([0, 0, 0]), yawRate=0)
            #     if lastSlide == 'UP SLIDE':
            #         self.cf.cmdVelocityWorld(np.array([0, 0, self.releasevelocity]), yawRate=0)
            #         timeHelper.sleep(self.releasetime)
            #         #stop
            #         self.cf.cmdVelocityWorld(np.array([0, 0, 0]), yawRate=0)
            #     if lastSlide == 'DOWN SLIDE':
            #         self.cf.cmdVelocityWorld(np.array([0, 0, -self.releasevelocity]), yawRate=0)
            #         timeHelper.sleep(self.releasetime)
            #         #stop
            #         self.cf.cmdVelocityWorld(np.array([0, 0, 0]), yawRate=0)
            
            else:
                try:     
                    if float(msg.data) < 1.1 :#start_right
                        #speed_msg = rospy.wait_for_message("/cf2/speed", String, timeout=0.5)
                        #speed = float(speed_msg.data)
                        # #print(speed_msg, type(speed))
                        self.smoothvelocity = float(msg.data)*self.velocity2
                        # #print("velocity is", self.smoothvelocity)
                        #self.smoothvelocity= 0.3
                except:
                    if msg.data == 'RIGHT SLIDE' :#start_right
                        self.cf.cmdVelocityWorld(np.array([0, -self.smoothvelocity, 0]), yawRate=0)
                        print("slide right, follow.")


                    if msg.data == 'LEFT SLIDE' :#start_left
                        print("slide left")
                        self.cf.cmdVelocityWorld(np.array([0, self.smoothvelocity, 0]), yawRate=0)

                    
                    if msg.data == 'UP SLIDE': #start_up
                        print("slide up")
                        self.cf.cmdVelocityWorld(np.array([0, 0, self.smoothvelocity]), yawRate=0)
                        
                    if msg.data == 'DOWN SLIDE': #start_down
                        print("slide down")
                        self.cf.cmdVelocityWorld(np.array([0, 0, -self.smoothvelocity]), yawRate=0)


                    # if msg.data == 'ZOOM IN': #start_down
                    #     print("slide down")
                    #     self.cf.cmdVelocityWorld(np.array([0, -self.smoothvelocity, 0]), yawRate=0)


                    # if msg.data == 'ZOOM OUT': #start_down
                    #     print("slide down")
                        
                    #     self.cf.cmdVelocityWorld(np.array([0, self.smoothvelocity, 0]), yawRate=0)

            if msg.data != "" and msg.data != "ZOOM IN" and msg.data != "ZOOM OUT" and msg.data != 'LEFT SLIDE' and msg.data != 'RIGHT SLIDE' and msg.data != 'UP SLIDE' and msg.data != 'DOWN SLIDE':
                lastGesture = msg.data
                #print("lastGesture:", lastGesture)

            if 'SLIDE' in msg.data:
                lastSlide = msg.data
                #print("lastSlide:", lastGesture)



    def addToQueueAndAverage(self, d, image):
        d.append(image)
        print("queue", d)
        #print ("len", len(d))
        if len(d) == 10:
        #print ("getting rid of ", d.popleft())
            try:
                return(mode(d))
            except:
                return('')
        else:
            return('')

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

    global d_slide, lastGesture
    d_slide = deque([], 10)
    lastGesture="None"

    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    #drone = KeyboardDrone(allcfs.crazyflies[0])
    #with keyboard.Listener(on_press=drone.on_press, on_release=drone.on_release) as listener:
    #     listener.join()
    drone = GestureDrone(allcfs.crazyflies)

    #try:
        #Testing our function
        #rospy.init_node('drone_RTcommands', anonymous=True)
        #handsignal_subscriber = rospy.Subscriber('/hand/signal', String, signal_callback())
        #handslide_subscriber = rospy.Publisher('/hand/direction', String, queue_size=10)
        #handforward_publisher = rospy.Publisher('/hand/forward', String, queue_size=10)

    #    execute()

    #except rospy.ROSInterruptException: pass
