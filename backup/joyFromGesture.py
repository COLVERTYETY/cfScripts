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
        self.takeoff_height = 0.3

        self.sleeptime = 0.5
        self.msg= ''
        #self.max_hight = 0.8
        #self.hight = 0.0
        print ('Press u to take off!')
        self.listener()




    def signal_callback(self, msg):

        # if signal == 'm': #fix_position
        #     self.cf.goTo(self.cf.position(), yaw=0, duration=0.5)

        # if signal == 'w': #start_forward
        #     self.cf.cmdVelocityWorld(np.array([self.velocity, 0, 0]), yawRate=0)
        if msg.data == 'THREE' :#start_back
            self.cf.cmdVelocityWorld(np.array([-self.velocity, 0, 0]), yawRate=0)
            
        if msg.data == 'TWO': #start_forward
            self.cf.cmdVelocityWorld(np.array([self.velocity, 0, 0]), yawRate=0)
            
        #if msg.data == 'FIVE' :#start_up
        #    self.cf.cmdVelocityWorld(np.array([0, 0, self.velocity]), yawRate=0)

        if msg.data == 'FOUR': #start_down
            self.cf.cmdVelocityWorld(np.array([0, 0, -self.velocity]), yawRate=0)

        # if signal == 'd': #start_right
        #     self.cf.cmdVelocityWorld(np.array([0, -self.velocity, 0]), yawRate=0)
        # if signal == 'c': #start_down
        #     self.cf.cmdVelocityWorld(np.array([0, 0, -self.velocity]), yawRate=0)
        # if signal == 'z': #start_up
        #     self.cf.cmdVelocityWorld(np.array([0, 0, self.velocity]), yawRate=0)


        if msg.data == '':
            print("fixed")
            #print('Kill engines')
	    #cf.cmdStop()
            self.cf.cmdVelocityWorld(np.array([0, 0, 0]), yawRate=0)
            #return False

        #if key.char == 'q':
        #    self.cf.start_turn_left(self.ang_velocity)
        #if key.char == 'e':
        #    self.cf.start_turn_right(self.ang_velocity)

    # def on_release (self, key):
    #     self.cf.cmdVelocityWorld(np.array([0, 0, 0]), yawRate=0)

    def slide_callback(self, msg):

        if msg.data == 'SLIDE RIGHT' :#start_right
            self.cf.cmdVelocityWorld(np.array([0, -self.velocity, 0]), yawRate=0)

        if msg.data == 'SLIDE LEFT' :#start_left
            self.cf.cmdVelocityWorld(np.array([0, self.velocity, 0]), yawRate=0)
        
        if msg.data == 'SLIDE UP': #take_off
            print ("takeoff")
            self.cf.takeoff(targetHeight=self.takeoff_height, duration=1.0)

        if msg.data == 'SLIDE DOWN': #land
            print ("land")
            self.cf.land(0.05, duration=1.0)

        if msg.data == '':
            print("fixed")
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
