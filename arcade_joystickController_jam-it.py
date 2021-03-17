#!/usr/bin/env python

import logging
from pynput import keyboard


import numpy as np
from pycrazyswarm import *
import sys
import signal

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String, Float64MultiArray
Z = 0.3
sleepRate = 30


import uav_trajectory

import dvic_demo #ALL TRAJECTORIES...

def signal_handler(signal, frame):
	sys.exit(0)

class KeyboardDrone:

    def __init__(self, cfs):
        self.ids = (1, 2)
        self.cfs = cfs
        self.velocity = [0, 0, 0, 0, 0, 0] #CF API DOES NOT TAKE FLOATS // CURRENTLY FOR SIX DRONES
        self.direction = np.array([0, 0, 0, 0, 0, 0]) #ENABLE DIRECTIONS // CURRENTLY FOR TWO DRONES
        self.arm = True
        self.base_velocity = 0.3

        self.ang_velocity = 120
        self.takeoff_height = 0.5

        self.sleeptime = 0.5

        self.control_from_joystick = False


        self.heli = uav_trajectory.Trajectory()
        self.heli.loadcsv("helicoidale.csv")


        self.traj0 = uav_trajectory.Trajectory()
        self.traj0.loadcsv("figure8.csv")


        self.rdev18_traj = uav_trajectory.Trajectory()
        self.rdev18_traj.loadcsv("demo_shapes/rdev_18deg.csv")

        self.ldev18_traj = uav_trajectory.Trajectory()
        self.ldev18_traj.loadcsv("demo_shapes/ldev_18deg.csv")

        self.joystick = Joy()
        self.joystick.axes = [-0.0, -0.0, -0.0, -0.0, 0.0, -0.0]
        self.joystick.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        print ('Press B0 to take off!')

        for cf in self.cfs:
            if cf.id == 1:
                    self.cf1 = cf
                    print("cf1 found")
                    #self.cf1.takeoff(targetHeight=self.takeoff_height, duration=3.0)

            if cf.id == 2:
                    self.cf2 = cf
                    print("cf2 found")
                    #self.cf2.takeoff(targetHeight=self.takeoff_height, duration=3.0)

            if cf.id == 3:
                    self.cf3 = cf
                    print("cf3 found")
                    #self.cf3.takeoff(targetHeight=self.takeoff_height, duration=3.0)


    def read_joystick(self, key):
        # self.joystick.axes = key.axes
        # self.joystick.buttons = key.buttons
        if key != None:
            self.joystick = key
        # axes: [-0.0, -0.0, -0.0, -0.0, -1.0, -0.0]
        # buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    def on_press(self, key):
        #print(key)



        # A subscriber to the topic '/joy'. self.on_press is called
        # when a message of type Joy is received.

        """MSG HEADER INDICATES THE DRONE ID"""
        #     print("hi")
        #     if key.data[0] == '1':
        #         #print ('update v1.')
        #         velocity_cf1 = float(key.data[:1])
        #         self.velocity[0] = self.base_velocity * velocity_cf1

        #     if key.data[0] == '2':
        #         #print ('update v2.')
        #         velocity_cf2 = float(key.data[:1])
        #         self.velocity[1] = self.base_velocity * velocity_cf2


        #     if key.data[0] == '3':
        #         #print ('update v3')
        #         velocity_cf3 = float(key.data[:1])
        #         self.velocity[2] *= self.base_velocity * velocity_cf3

        """SEPARATE SPEED COEFFICIENT UPDATES THE VELOCITY: SECURITYSPEED[6]*SELF.BASE_VELOCITY = SELF.VELOCITY[6]"""
        msg = rospy.wait_for_message('/cf2/filtredsignal', Float64MultiArray)

        update_drone1 = msg.data[self.ids[0]-1]
        #print("update {:.3f}".format(update)) #FOR DEBUGGING.
        self.velocity[self.ids[0]-1] = update_drone1 * self.base_velocity

        update_drone2 = msg.data[self.ids[1]-1]
        #print("update {:.3f}".format(update)) #FOR DEBUGGING.
        self.velocity[self.ids[1]-1] = update_drone2 * self.base_velocity


        ## MAKING A SELF.DIRECTION STRUCTURE: indices [0, 1, 2] refers to /j0 JOYSTICK, ordered as [+up/-down, +left/-right, {empty because plane for now}]
        if key.axes[0]== 1:
            print('LEFT')
            self.direction[0] = 1

        if key.axes[0]== -1:
            print('RIGHT')
            self.direction[0] = -1

        if key.axes[0]== 0:
            #print('LR FIXED')
            self.direction[0] = 0



        if key.axes[1]== 1:
            print('UP')
            self.direction[1] = 1

        if key.axes[1]== -1:
            print('DOWN')
            self.direction[1] = -1

        if key.axes[1]== 0:
            #print('UD FIXED')
            self.direction[1] = 0



        if key.buttons[0]== 1:
            print('B0: TAKEOFF') #take_off
            if self.arm == True:
                for cf in self.cfs:
                    if cf.id == self.ids[0]:
                        print("Takeoff.")
                        self.direction[0] = 0
                        self.direction[1] = 0
                        cf.takeoff(targetHeight=self.takeoff_height, duration=3.0)

        if key.buttons[1]== 1:
            print('B1: LAND')
            if self.arm == True:
                for cf in self.cfs:
                    if cf.id == self.ids[0]:
                        print("Land.")
                        self.direction[0] = 0
                        self.direction[1] = 0
                        cf.land(0.05, duration = 3.0)

        if key.buttons[2]== 1:
            print('B2: JOYSTICK MODE ACTIVATED')
            self.control_from_joystick = True

        if key.buttons[3]== 1:
            print('B3: HELI, ONE DRONE')
            self.control_from_joystick=False
            self.trajectory = "1 heli"
            # for cf in allcfs.crazyflies:
            #     if cf.id == self.ids[0]:
            #         cf.uploadTrajectory(0, 0, self.heli)
            # TRIALS = 1
            # TIMESCALE = 1
            # dvic_demo.fly(self.ids[0], allcfs, swarm, timeHelper, TRIALS, TIMESCALE)

        if key.buttons[4]==1:
            print('B4: RIGHT DEV, DRONE 1')
            TRIALS = 1
            TIMESCALE = 0.75
            #for i in range(TRIALS):
            # for cf in allcfs.crazyflies:
            #         cf.uploadTrajectory(0, 0, self.rdev18_traj)
            # dvic_demo.fly(self.ids[0], allcfs, swarm, timeHelper, TRIALS, TIMESCALE)

        if key.buttons[5]== 1:
            print('B5: LEFT DEV, DRONE 1')
            TRIALS = 1
            TIMESCALE = 0.75
            #for i in range(TRIALS):
            # for cf in allcfs.crazyflies:
            #         cf.uploadTrajectory(0, 0, self.ldev18_traj)
            # dvic_demo.fly(self.ids[0], allcfs, swarm, timeHelper, TRIALS, TIMESCALE)

        ## MAKING A SELF.DIRECTION STRUCTURE: indices [3, 4, 5] refers to /j0 JOYSTICK, ordered as [+up/-down, +left/-right, {empty because plane for now}]
        if key.axes[3]== 1:
            print('LEFT')
            self.direction[3] = 1

        if key.axes[3]== -1:
            print('RIGHT')
            self.direction[3] = -1

        if key.axes[3]== 0:
            #print('LR FIXED')
            self.direction[3] = 0



        if key.axes[4]== 1:
            print('UP')
            self.direction[4] = 1

        if key.axes[4]== -1:
            print('DOWN')
            self.direction[4] = -1

        if key.axes[4]== 0:
            #print('UD FIXED')
            self.direction[4] = 0



        if key.buttons[6]== 1:
            print('B6: TAKEOFF') #take_off
            #self.velocity_streaming = False
            if self.arm == True:
                for cf in self.cfs:
                    if cf.id == self.ids[1]:
                        print("Takeoff.")
                        self.direction[3] = 0
                        self.direction[4] = 0
                        cf.takeoff(targetHeight=self.takeoff_height, duration=3.0)

        if key.buttons[7]== 1:
            print('B7: LAND')
            #self.velocity_streaming = False
            if self.arm == True:
                for cf in self.cfs:
                    if cf.id == self.ids[1]:
                        print("Land.")
                        self.direction[3] = 0
                        self.direction[4] = 0
                        cf.land(0.05, duration = 3.0)

        if key.buttons[8]== 1:
            print('B8: FIGURE OF 8, BOTH')
            self.control_from_joystick = False
            # for cf in allcfs.crazyflies:
            #         cf.uploadTrajectory(0, 0, self.traj0)
            # TRIALS = 1
            # TIMESCALE = 1
            # dvic_demo.fly_two_drones(allcfs, swarm, timeHelper, TRIALS, TIMESCALE)

        if key.buttons[9] == 1:
            print('B9: HELI, DRONE 2')
            self.control_from_joystick=False

            # for cf in allcfs.crazyflies:
            #     if cf.id == self.ids[1]:
            #         cf.uploadTrajectory(0, 0, self.heli)
            # TRIALS = 1
            # TIMESCALE = 1
            # dvic_demo.fly(self.ids[1], allcfs, swarm, timeHelper, TRIALS, TIMESCALE)


        if key.buttons[10]==1:
            print('B10: RIGHT DEV, DRONE 2')
            TRIALS = 1
            TIMESCALE = 0.75
            #for i in range(TRIALS):
            # for cf in allcfs.crazyflies:
            #         cf.uploadTrajectory(0, 0, self.rdev18_traj)
            # dvic_demo.fly(self.ids[1], allcfs, swarm, timeHelper, TRIALS, TIMESCALE)


        if key.buttons[11]== 1:
            print('B11')
            TRIALS = 1
            TIMESCALE = 0.75
            #for i in range(TRIALS):
            # for cf in allcfs.crazyflies:
            #         cf.uploadTrajectory(0, 0, self.ldev18_traj)
            # dvic_demo.fly(self.ids[1], allcfs, swarm, timeHelper, TRIALS, TIMESCALE)

        if self.arm == True: #and self.velocity_streaming == True:
            for cf in self.cfs:
                if cf.id == self.ids[0]:
                    cf.cmdVelocityWorld(np.array([self.velocity[self.ids[0]-1]*self.direction[0], self.velocity[self.ids[0]-1]*self.direction[1], 0]), yawRate=0)

                if cf.id == self.ids[1]:
                    cf.cmdVelocityWorld(np.array([self.velocity[self.ids[1]-1]*self.direction[3], self.velocity[self.ids[1]-1]*self.direction[4], 0]), yawRate=0)

signal.signal(signal.SIGINT, signal_handler)

if __name__ == '__main__':
    #rospy.init_node('joystick_to_drone')
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs


    drone = KeyboardDrone(allcfs.crazyflies)
    joy_sub = rospy.Subscriber('/joy_unified', Joy, drone.on_press)

    rospy.spin()


