#!/usr/bin/env python

from __future__ import print_function

from pycrazyswarm import *
import numpy as np
def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    for cf in allcfs.crazyflies:
        print(cf.id)
        cf.takeoff(0.5, 2.5)
        # if cf.id == 2:
        #     cf.takeoff(0.5, 2.5)

        
    # YAW RATE INCREASE TEST
    # print("press button to continue")
    # swarm.input.waitUntilButtonPressed()

    # for cf in allcfs.crazyflies:
    #     cf.cmdVelocityWorld(np.array([0, 0, 0]), yawRate=0.1)


    print("press button to continue")
    swarm.input.waitUntilButtonPressed()

    for cf in allcfs.crazyflies:
        cf.land(0.04, 2.5)


if __name__ == "__main__":
    main()
