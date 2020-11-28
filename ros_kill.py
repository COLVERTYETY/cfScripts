#!/usr/bin/env python

from __future__ import print_function
import sys
import signal

from pycrazyswarm import *
import uav_trajectory
import actionlib
import actionlib_tutorials.msg
# from beginner_tutorials.srv import AddTwoInts,AddTwoIntsResponse
# import rospy

# def handle_add_two_ints(req):
#     print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
#     return AddTwoIntsResponse(req.a + req.b)

# def add_two_ints_server():
#     rospy.init_node('add_two_ints_server')
#     s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
#     print("Ready to add two ints.")
#     rospy.spin()

# if __name__ == "__main__":
#     add_two_ints_server()


from actionlib_tutorials.srv import killMotors, killMotorsResponse
import rospy

def signal_handler(signal, frame):
	sys.exit(0)




def handleKillService(req):
    for cf in self.allcfs.crazyflies:
        if cf.id == 2.0:
            print(cf.id)
            cf.cmdStop()
    #print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
    return 1.0

def setupKillService():
    #rospy.init_node('setupKillService')
    s = rospy.Service('add_two_ints', AddTwoInts, handleKillService)
    print("Ready to add two ints.")
    rospy.spin()

signal.signal(signal.SIGINT, signal_handler)

if __name__ == "__main__":

    self.swarm = Crazyswarm()
    #self.timeHelper = swarm.timeHelper
    self.allcfs = self.swarm.allcfs
    setupKillService()