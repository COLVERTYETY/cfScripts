#!/usr/bin/env python

import numpy as np
import rospy
import tf
from geometry_msgs.msg import TransformStamped
#from tf2_msgs.msg import TFMessage
import tf2_msgs.msg
from pycrazyswarm import *
import uav_trajectory
import matplotlib.pyplot as plt


def callback(cf_of_interest):

    #cf2 is a : cf2=tf2_msgs.msg.TFMessage([t])
    #with t : t=geometry_msgs.msg.TransformStamped()

    #create cf to be a TransformStamped:
    cft = cf_of_interest.transforms[0]
    print(cft)

    x = cft.transform.translation.x
    y = cft.transform.translation.y
    z = cft.transform.translation.z
    print(z)


    if (z < 0.15):
        z = 0.15


   # for cf in allcfs.crazyflies:
   #     pos = np.array([x, y, z+0.3])
   #     print("calculated pos: ", pos)
   #     print ("real pos: ", cf.position())
   #     #cf.cmdPosition(pos, 0)
   #     cf.cmdVelocityWorld(np.array([1,0,0]), 0)
   #     #timeHelper.sleep(1.5+2.0)

    pos = np.array([x, y, z+0.3])
    print("calculated pos: ", pos)
    print ("real pos: ", allcfs.crazyflies[0].position())
    #cf.cmdPosition(pos, 0)
    allcfs.crazyflies[0].cmdPosition(pos, 0)
    #timeHelper.sleep(1.5)

    #plt.show(block=False)


    #print("press button to continue...")
    #swarm.input.waitUntilButtonPressed()


def listener():
    rospy.init_node('follow_me_now', anonymous=True) # this is a maybe
    rospy.Subscriber('/tf', tf2_msgs.msg.TFMessage, callback)
    rospy.spin()
    #timeHelper.sleep(1.5)
	#plt.show(block=False)


if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    Z = 0.3
    allcfs.crazyflies[0].takeoff(targetHeight=Z, duration=1.4+Z)
    timeHelper.sleep(1.5+Z)
    while (1):
        listener()

    allcfs.crazyflies[0].land(targetHeight=0.06, duration=1.0+Z)
    timeHelper.sleep(1.0)

