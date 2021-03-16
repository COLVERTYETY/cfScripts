#!/usr/bin/env python

import logging
from pynput import keyboard


import numpy as np
from pycrazyswarm import *
import sys
import signal

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String

Z = 0.3
sleepRate = 30

def signal_handler(signal, frame):
	sys.exit(0)

class KeyboardDrone:

    def __init__(self, cf):
        self.cf = cf
        self.velocity = 0.75
        self.ang_velocity = 120
        self.takeoff_height = 0.5

        self.sleeptime = 0.5
        #self.max_hight = 0.8
        #self.hight = 0.0
        #print ('Press u to take off!')

        self.cf.takeoff(targetHeight=self.takeoff_height, duration=1.0)
        # self.cf.cmdVelocityWorld(np.array([0, 0, 0]), yawRate=0.1)



    def on_press(self, key):
        # A subscriber to the topic '/joy'. self.on_press is called
        # when a message of type Joy is received.

        if key.data =='LEFT':
            print('LEFT')
            self.cf.goTo(np.array([0.5, 0, 0.5]), 0, duration=1.0, groupMask=0)

        if key.data =='RIGHT':
            print('RIGHT')
            self.cf.goTo(np.array([-0.5, 0, 0.5]), 0, duration=1.0, groupMask=0)

        # if key.axes[0]== 1:
        #     print('LEFT')
        #     #self.cf.cmdVelocityWorld(np.array([0, 0, 0]), yawRate=0.1)
        # # if key.axes[0]== 0:
        # #     print('NOTHING')

        # if key.axes[0]== -1:
        #     print('RIGHT')
        #     #self.cf.cmdVelocityWorld(np.array([0, 0, 0]), yawRate=0.1)

        # if key.axes[1]== 1:
        #     print('UP')#start_forward
        #     #self.cf.cmdVelocityWorld(np.array([self.velocity, 0, 0]), yawRate=0)

        # # if key.axes[1]== 0:
        # #     print('NOTHING')

        # if key.axes[1]== -1:
        #     print('DOWN')



        # if key.buttons[0]== 1:
        #     print('TAKEOFF') #take_off
        #     #self.cf.takeoff(targetHeight=self.takeoff_height, duration=1.0)

        # if key.buttons[1]== 1:
        #     print('LAND')

        # if key.buttons[0]== 0:
        #     print('DOWN')



        #     cf2_pose = rospy.wait_for_message("/tf", TFMessage, timeout=None)

        # if key.char == 'm': #fix_position
        #     self.cf.goTo(self.cf.position(), yaw=0, duration=0.5)
        # if key.char == 'w': #start_forward
        #     self.cf.cmdVelocityWorld(np.array([self.velocity, 0, 0]), yawRate=0)
        # if key.char == 'u': #take_off
        #     self.cf.takeoff(targetHeight=self.takeoff_height, duration=1.0)
        # if key.char == 's': #start_back
        #     self.cf.cmdVelocityWorld(np.array([-self.velocity, 0, 0]), yawRate=0)
        # if key.char == 'a': #start_left
        #     self.cf.cmdVelocityWorld(np.array([0, self.velocity, 0]), yawRate=0)
        # if key.char == 'd': #start_right
        #     self.cf.cmdVelocityWorld(np.array([0, -self.velocity, 0]), yawRate=0)
        # if key.char == 'c': #start_down
        #     self.cf.cmdVelocityWorld(np.array([0, 0, -self.velocity]), yawRate=0)
        # if key.char == 'z': #start_up
        #     self.cf.cmdVelocityWorld(np.array([0, 0, self.velocity]), yawRate=0)

        # if key.char == 'l':
        #     #print('Kill engines')
	    # #cf.cmdStop()
        #     self.cf.land(0.05, duration=1.0)

            #return False

        #if key.char == 'q':
        #    self.cf.start_turn_left(self.ang_velocity)
        #if key.char == 'e':
        #    self.cf.start_turn_right(self.ang_velocity)

    # def on_release (self, key):
    #     self.cf.cmdVelocityWorld(np.array([0, 0, 0]), yawRate=0)

signal.signal(signal.SIGINT, signal_handler)

if __name__ == '__main__':
    #rospy.init_node('joystick_to_drone')
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    drone = KeyboardDrone(allcfs.crazyflies[0])
    #with keyboard.Listener(on_press=drone.on_press, on_release=drone.on_release) as listener:
    #     listener.join()
    #collision_publisher = rospy.Publisher('/collision', String, queue_size=10)
    #perimeter_server = PerimeterMonitor(str(rospy.get_name())+str(1))

    #position_subscriber = rospy.Subscriber('/joy', Joy, drone.on_press)
    neuro_subscriber = rospy.Subscriber("/neuro_output",
            String, drone.on_press,  queue_size = 1)

    #Remove if you know what you're doing. Currrently used to callback CONTINUOUSLY...
    rospy.spin()