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

    def __init__(self, cf):
        speak (engine, "Hand control mode activated.")
        self.cf = cf
        self.velocity = 0.1
        self.velocity2 = 0.3
        self.ang_velocity = 120
        self.takeoff_height = 0.5

        self.sleeptime = 0.5
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
        self.cf.takeoff(targetHeight=self.takeoff_height, duration=3.0)
        
        self.listener()




    def signal_callback(self, msg):
        print(msg.data)

        
        while abs(self.cf.position()[0])>1.0 or abs(self.cf.position()[1])>1.0 or abs(self.cf.position()[0])>1.0:
            print ("go home now")

        #print (self.cf.position())
        if msg.data == 'PEACE': #Activate followMODE
            speak (engine, "Activating Hand following mode.")
            #print ("followMode ACTIVATED")
            self.followMode=True
        if msg.data == 'INDEX': #Activate SignalMode: 
            speak (engine, "Activating signal mode.")
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
        if self.followMode == False :


            if msg.data == 'SPIDERMAN':#start_up
                #print("signalmode, spiderman.")
                self.cf.takeoff(targetHeight=self.takeoff_height, duration=3.0)

                
            if msg.data == 'THUMBDOWN':#start_up
                #print(".")
                self.cf.land(0.05, duration=1.0)


            if msg.data == 'UP':#start_up
                #print(".")
                self.cf.cmdVelocityWorld(np.array([0, 0, self.velocity]), yawRate=0)
                #rospy.sleep()

            if msg.data == 'DOWN': #start_down
                #print(".")
                self.cf.cmdVelocityWorld(np.array([0, 0, -self.velocity]), yawRate=0)
                #rospy.sleep()

            if msg.data == 'RIGHT': #start_right
                #print(".")
                self.cf.cmdVelocityWorld(np.array([0, -self.velocity, 0]), yawRate=0)
                #rospy.sleep()

            if msg.data == 'LEFT': #start_right
                #print(".")
                self.cf.cmdVelocityWorld(np.array([0, self.velocity, 0]), yawRate=0)
                #rospy.sleep()

            # if signal == 'c': #start_down
            #     self.cf.cmdVelocityWorld(np.array([0, 0, -self.velocity]), yawRate=0)
            # if signal == 'z': #start_up
            #     self.cf.cmdVelocityWorld(np.array([0, 0, self.velocity]), yawRate=0)
            if (msg.data == '' or msg.data == 'UNKNOWN' or msg.data == 'FIST'):
                print(".")
                #self.cf.cmdVelocityWorld(np.array([0, 0, 0]), yawRate=0)
                

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


        if self.followMode == True :
            #print("in mode", msg.data)

            
            if msg.data == 'RIGHT SLIDE' :#start_right
                self.cf.cmdVelocityWorld(np.array([0, self.velocity2, 0]), yawRate=0)
                print("slide right, follow.")
                #pos = self.cf.position() + np.array([0, -0.05, 0])
                #self.cf.goTo(pos, yawRate=0)
                #self.cf.goTo(np.array([0, self.velocity, 0]), yawRate=0)
                #pos = self.cf.position() + np.array([0, -self.velocity, 0])

            if msg.data == 'LEFT SLIDE' :#start_left
                print("slide left")
                self.cf.cmdVelocityWorld(np.array([0, -self.velocity2, 0]), yawRate=0)
                #pos = self.cf.position() + np.array([0, self.velocity, 0])                
                #self.cf.goTo(np.array([0, self.velocity, 0]), yawRate=0)
            
            if msg.data == 'UP SLIDE': #start_up
                print("slide up")
                #pos = self.cf.position() + np.array([0, 0, self.velocity])               
                self.cf.cmdVelocityWorld(np.array([0, 0, self.velocity2]), yawRate=0)
                
            if msg.data == 'DOWN SLIDE': #start_down
                print("slide down")
                #pos = self.cf.position() + np.array([0, 0, -self.velocity])
                self.cf.cmdVelocityWorld(np.array([0, 0, -self.velocity2]), yawRate=0)
                #self.cf.land(0.05, duration=1.0)

            #if msg.data == 'FIST' or msg.data == '':
                #print("stop")
                #self.cf.cmdVelocityWorld(np.array([0, 0, 0]), yawRate=0)
                #pos = self.cf.position() + np.array([0, 0, 0])

            #else:
                #self.cf.cmdVelocityWorld(np.array([0, 0, 0]), yawRate=0)

                #pos = self.cf.position()



            #self.cf.goTo(pos, yaw=0, duration=self.goToDuration)


    def addToQueueAndAverage(self, d, image):
        d.append(image)
        #print("queue", d)
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
        handsignal_subscriber = rospy.Subscriber('/hand/signal', String, self.signal_callback)

        #handslide_subscriber = rospy.Subscriber('/hand/direction', String, self.slide_callback)

        #cf.cmdVelocityWorld(np.array([self.velocity, 0, 0]), yawRate=0)
        
        rospy.spin()


signal.signal(signal.SIGINT, signal_handler)

if __name__ == '__main__':

    global d_slide
    d_slide = deque([], 10)

    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    #drone = KeyboardDrone(allcfs.crazyflies[0])
    #with keyboard.Listener(on_press=drone.on_press, on_release=drone.on_release) as listener:
    #     listener.join()
    drone = GestureDrone(allcfs.crazyflies[0])

    #try:
        #Testing our function
        #rospy.init_node('drone_RTcommands', anonymous=True)
        #handsignal_subscriber = rospy.Subscriber('/hand/signal', String, signal_callback())
        #handslide_subscriber = rospy.Publisher('/hand/direction', String, queue_size=10)
        #handforward_publisher = rospy.Publisher('/hand/forward', String, queue_size=10)

    #    execute()

    #except rospy.ROSInterruptException: pass
