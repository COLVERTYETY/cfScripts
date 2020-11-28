#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *
import sys
import signal

Z = 0.4
sleepRate = 30


def signal_handler(signal, frame):
	sys.exit(0)

def goCircle(timeHelper, cf, totalTime, radius, kPosition):
        startTime = timeHelper.time()
        pos = cf.position()
        startPos = pos + np.array([0, 0, Z])
        center_circle = startPos - np.array([radius, 0, 0])
        while timeHelper.time()-startTime<10 :
            time = timeHelper.time() - startTime
            omega = 2 * np.pi / totalTime
            vx = -radius * omega * np.sin(omega * time)  
            vy = radius * omega * np.cos(omega * time)
            desiredPos = center_circle + radius * np.array(
                [np.cos(omega * time), np.sin(omega * time), 0])
            errorX = desiredPos - cf.position() 
            allcfs.crazyflies[0].cmdVelocityWorld(np.array([vx, 0, vy] + kPosition * errorX), yawRate=0)
            allcfs.crazyflies[1].cmdVelocityWorld(np.array([vx, vy, 0] + kPosition * errorX), yawRate=0)

            timeHelper.sleepForRate(sleepRate)

        cf.cmdVelocityWorld(np.array([0, 0, 0]), yawRate=0)


def goCircleReverse(timeHelper, cf, totalTime, radius, kPosition):
        startTime = timeHelper.time()
        pos = cf.position()
        startPos = pos + np.array([0, 0, Z])
        center_circle = startPos - np.array([radius, 0, 0])
        while timeHelper.time()-startTime<10 :
            time = timeHelper.time() - startTime
            omega = 2 * np.pi / totalTime
            vx = -radius * omega * np.sin(omega * time)  
            vy = radius * omega * np.cos(omega * time)
            desiredPos = center_circle + radius * np.array(
                [np.cos(omega * time), np.sin(omega * time), 0])
            errorX = desiredPos - cf.position() 
            allcfs.crazyflies[0].cmdVelocityWorld(-1*(np.array([vx, vy, 0] + kPosition * errorX)), yawRate=0)
            allcfs.crazyflies[1].cmdVelocityWorld(np.array([vx, vy, 0] + kPosition * errorX), yawRate=0)
            timeHelper.sleepForRate(sleepRate)

        cf.cmdVelocityWorld(np.array([0, 0, 0]), yawRate=0)


signal.signal(signal.SIGINT, signal_handler)

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.crazyflies[0].takeoff(targetHeight=Z, duration=2.0+Z)
    allcfs.crazyflies[1].takeoff(targetHeight=Z, duration=2.0+Z)

    timeHelper.sleep(4)
    goCircle(timeHelper, allcfs.crazyflies[0], totalTime=4, radius=0.3, kPosition=1)
    #goCircleReverse(timeHelper, allcfs.crazyflies[1], totalTime=4, radius=0.3, kPosition=1)
	
    #print("press button to continue...")
    #swarm.input.waitUntilButtonPressed()

   # while Z>0:
       # print(Z)
    #	allcfs.crazyflies[0].cmdPosition(allcfs.crazyflies[0].position()-np.array([0,0,0.05]))
#	timeHelper.sleep(0.3)
#	Z-=0.05
    allcfs.crazyflies[0].land(targetHeight=0.06, duration=2.0)
    allcfs.crazyflies[1].land(targetHeight=0.06, duration=2.0)

    print("press button to continue...")
    swarm.input.waitUntilButtonPressed()

    #allcfs.crazyflies[0].cmdStop()

    #print("press button to continue...")
    #swarm.input.waitUntilButtonPressed()

