#!/usr/bin/env python

import logging
from pynput import keyboard


import numpy as np
from pycrazyswarm import *
import sys
import signal

import rospy
from std_msgs.msg import String



Z = 0.3
sleepRate = 30

def signal_handler(signal, frame):
	sys.exit(0)

class GestureDrone:

    def __init__(self, cf):
        self.cf = cf
        self.velocity = 0.75
        self.ang_velocity = 120
        self.takeoff_height = 0.5

        self.sleeptime = 0.5
        self.msg= ''
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
        print('THUMBUP to go on follow mode')
        self.cf.takeoff(targetHeight=self.takeoff_height, duration=3.0)
        
        self.listener()




    def signal_callback(self, msg):
        print(msg.data)
        if msg.data == 'THUMBUP': #Activate followMODE
            print ("followMode ACTIVATED")
            self.followMode=True

        else : #Deactivate followMODE
            print ("SignalMode ACTIVATED")
            self.followMode=False
        
        #if self.followMode==False:
            # if signal == 'w': #start_forward
            #     self.cf.cmdVelocityWorld(np.array([self.velocity, 0, 0]), yawRate=0)
            # if msg.data == 'THREE' :#start_back
            #     self.cf.cmdVelocityWorld(np.array([-self.velocity, 0, 0]), yawRate=0)

            # if msg.data == 'TWO': #start_forward
            #     self.cf.cmdVelocityWorld(np.array([self.velocity, 0, 0]), yawRate=0)
            if msg.data == 'SPIDERMAN' :#start_up
                self.cf.takeoff(targetHeight=self.takeoff_height, duration=3.0)
            
            if msg.data == 'THUMBDOWN' :#start_up
                self.cf.land(0.05, duration=1.0)

            if msg.data == 'UP' :#start_up
                self.cf.cmdVelocityWorld(np.array([0, 0, self.velocity]), yawRate=0)

            if msg.data == 'DOWN': #start_down
                self.cf.cmdVelocityWorld(np.array([0, 0, -self.velocity]), yawRate=0)

            if msg.data == 'RIGHT': #start_right
                 self.cf.cmdVelocityWorld(np.array([0, -self.velocity, 0]), yawRate=0)

            if msg.data == 'LEFT': #start_right
                 self.cf.cmdVelocityWorld(np.array([0, self.velocity, 0]), yawRate=0)

            # if signal == 'c': #start_down
            #     self.cf.cmdVelocityWorld(np.array([0, 0, -self.velocity]), yawRate=0)
            # if signal == 'z': #start_up
            #     self.cf.cmdVelocityWorld(np.array([0, 0, self.velocity]), yawRate=0)
            if msg.data == '' or msg.data == 'UNKNOWN' or msg.data == 'FIST':
                print("fixed")
                self.cf.cmdVelocityWorld(np.array([0, 0, 0]), yawRate=0)
                

        #if key.char == 'q':
        #    self.cf.start_turn_left(self.ang_velocity)
        #if key.char == 'e':
        #    self.cf.start_turn_right(self.ang_velocity)

    # def on_release (self, key):
    #     self.cf.cmdVelocityWorld(np.array([0, 0, 0]), yawRate=0)

    def slide_callback(self, msg):
        print(msg.data)
        if self.followMode == True :

            if msg.data == 'RIGHT SLIDE' :#start_right
                self.cf.cmdVelocityWorld(np.array([0, -self.velocity, 0]), yawRate=0)

            if msg.data == 'LEFT SLIDE' :#start_left
                self.cf.cmdVelocityWorld(np.array([0, self.velocity, 0]), yawRate=0)
            
            if msg.data == 'UP SLIDE': #start_up
               
                self.cf.cmdVelocityWorld(np.array([0, 0, self.velocity]), yawRate=0)
                

            if msg.data == 'DOWN SLIDE': #start_down
                
                self.cf.cmdVelocityWorld(np.array([0, 0, -self.velocity]), yawRate=0)
                self.cf.land(0.05, duration=1.0)

            if msg.data == 'FIST':
                self.cf.cmdVelocityWorld(np.array([0, 0, 0]), yawRate=0)

    def listener(self):
        #rospy.init_node('drone_RTcommands', anonymous=True)
        handsignal_subscriber = rospy.Subscriber('/hand/signal', String, self.signal_callback)
        handslide_subscriber = rospy.Subscriber('/hand/direction', String, self.slide_callback)
        #cf.cmdVelocityWorld(np.array([self.velocity, 0, 0]), yawRate=0)
        rospy.spin()


signal.signal(signal.SIGINT, signal_handler)

if __name__ == '__main__':

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
