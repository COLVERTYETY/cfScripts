#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *

Z = 0.2

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(3.5)
    for cf in allcfs.crazyflies:
        pos = cf.position() + np.array([1.5, 0, 0])
        cf.goTo(pos, 0, 5.0)

    print(pos)
    print("press button to continue...")
    swarm.input.waitUntilButtonPressed()

    for cf in allcfs.crazyflies:
        pos2 = cf.position() + np.array([-1.5, 0, 0])
        cf.goTo(pos2, 0, 5.0)

    print(pos2)
    print("press button to continue...")
    swarm.input.waitUntilButtonPressed()


    allcfs.land(targetHeight=0.02, duration=2.5+Z)
    timeHelper.sleep(1.0+Z)
