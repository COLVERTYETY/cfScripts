
#!/usr/bin/env python

import numpy as np

from pycrazyswarm import *
import uav_trajectory

import keyboard

def fly():
    for i in range(TRIALS):
        #for cf in allcfs.crazyflies:
        #cf.takeoff(targetHeight=0.5, duration=3.0)
	allcfs.crazyflies[0].takeoff(targetHeight=0.3, duration=3.0)

	timeHelper.sleep(2.5)
	    #for cf in allcfs.crazyflies:
	    #    pos = cf.position() + np.array([0, 0, 1.0])
	    #    cf.goTo(pos, 0, 2.0)
	    #timeHelper.sleep(2.5)
        allcfs.crazyflies[0].startTrajectory(0, timescale=TIMESCALE)
	    #allcfs.crazyflies[0].startTrajectory(0, timescale=TIMESCALE, reverse=False)
	    #timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)
        timeHelper.sleep(1.0)

	print("press button to continue...")
	swarm.input.waitUntilButtonPressed()

	allcfs.crazyflies[0].land(targetHeight=0.06, duration=2.0)
	timeHelper.sleep(3.0)

def fly_two_drones():
    for i in range(TRIALS):
        allcfs.crazyflies[0].takeoff(targetHeight=0.5, duration=3.0)
        allcfs.crazyflies[1].takeoff(targetHeight=0.2, duration=3.0)

        timeHelper.sleep(2.5)
            #for cf in allcfs.crazyflies:
            #    pos = cf.position() + np.array([0, 0, 1.0])
            #    cf.goTo(pos, 0, 2.0)
            #timeHelper.sleep(2.5)
        #allcfs.crazyflies[0].startTrajectory(0, timescale=TIMESCALE)
        allcfs.crazyflies[1].startTrajectory(0, timescale=TIMESCALE)

        allcfs.crazyflies[0].startTrajectory(0, timescale=TIMESCALE, reverse=True)
            #timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)
        timeHelper.sleep(1.0)

        print("press button to continue...")
        swarm.input.waitUntilButtonPressed()

        allcfs.land(targetHeight=0.06, duration=2.0)
        timeHelper.sleep(3.0)


def fly_nostops():
    for i in range(TRIALS):
        for cf in allcfs.crazyflies:
            #cf.takeoff(targetHeight=0.5, duration=3.0)
        #allcfs.crazyflies[1].takeoff(targetHeight=0.3, duration=3.0)

            timeHelper.sleep(2.5)
            #for cf in allcfs.crazyflies:
            #    pos = cf.position() + np.array([0, 0, 1.0])
            #    cf.goTo(pos, 0, 2.0)
            #timeHelper.sleep(2.5)
            cf.startTrajectory(0, timescale=TIMESCALE)
            #allcfs.crazyflies[0].startTrajectory(0, timescale=TIMESCALE, reverse=False)
            #timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)
            #timeHelper.sleep(1.0)

            #print("press button to continue...")
            #swarm.input.waitUntilButtonPressed()

        #allcfs.land(targetHeight=0.06, duration=2.0)
        timeHelper.sleep(3.0)


if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    traj0 = uav_trajectory.Trajectory()
    traj0.loadcsv("figure8.csv")


    traj1 = uav_trajectory.Trajectory()
    traj1.loadcsv("trajectory2.csv")

    #traj2 = uav_trajectory.Trajectory()
    #traj2.loadcsv("timed_waypoints_circle1.csv")

    traj3 = uav_trajectory.Trajectory()
    traj3.loadcsv("circle_join_longer.csv")

    traj4 = uav_trajectory.Trajectory()
    traj4.loadcsv("helicoidale.csv")
 
    rdev18_traj = uav_trajectory.Trajectory()
    rdev18_traj.loadcsv("demo_shapes/rdev_18deg.csv")

    ldev18_traj = uav_trajectory.Trajectory()
    ldev18_traj.loadcsv("demo_shapes/ldev_18deg.csv")

    f8xz_traj = uav_trajectory.Trajectory()
    f8xz_traj.loadcsv("demo_shapes/figure8_xz.csv")

    #TRIALS = 1
    #TIMESCALE = 0.5
    nb_drones = raw_input ("nb of drones: ")
    demo = raw_input ("choose a demo: \n 0: xy figure of 8+circle \n 1: zx figure of 8 \n 2: semicircle(issues) \n 3: helicoidale\n 4: rdev18\n 5:ldev18\n 6:f8(xz)\n 7: keyboard ctrl\n 8: demo of the death \n demo:")
    #TRIALS = int(raw_input ("nb of repeats: ")
    #TIMESCALE = raw_input ("speed factor where 1 is max 1m/s, 1m/s2: ")
  
    if demo == '0':
                TRIALS = 1
                TIMESCALE = 1
        #for i in range(TRIALS):
                for cf in allcfs.crazyflies:
                        cf.uploadTrajectory(0, 0, traj0)
                if nb_drones == "2":
                	fly_two_drones()
                if nb_drones == "1":
                        fly()

                #fly()

    if demo == '1':

                TRIALS = 1
                TIMESCALE = 0.5
        #for i in range(TRIALS):
                for cf in allcfs.crazyflies:
                        cf.uploadTrajectory(0, 0, traj1)
                fly()

    if demo == '3':
                TRIALS = 1
                TIMESCALE = 0.5
        #for i in range(TRIALS):
                for cf in allcfs.crazyflies:
                        cf.uploadTrajectory(0, 0, traj4)
                if nb_drones == "2":
                        fly_two_drones()
                if nb_drones == "1":
                        fly()



    if demo == '4':
                TRIALS = 1
                TIMESCALE = 0.75
        #for i in range(TRIALS):
                for cf in allcfs.crazyflies:
                        cf.uploadTrajectory(0, 0, rdev18_traj)
                fly()

    if demo == '5':
                TRIALS = 1
                TIMESCALE = 0.75
        #for i in range(TRIALS):
                for cf in allcfs.crazyflies:
                        cf.uploadTrajectory(0, 0, ldev18_traj)
                fly()

    if demo == '6':
                TRIALS = 1
                TIMESCALE = 0.5
        #for i in range(TRIALS):
                for cf in allcfs.crazyflies:
                        cf.uploadTrajectory(0, 0, f8xz_traj)
                fly()


    if demo == '7':
        TRIALS = 1
        TIMESCALE = 0.75
        execfile('keyboard.py')

    if demo == '8':
        TRIALS = 1
        TIMESCALE = 0.5
        for cf in allcfs.crazyflies:
              cf.takeoff(targetHeight=0.6, duration=3.0)
              cf.uploadTrajectory(0, 0, traj1)
        fly_nostops()

        TRIALS = 1
        TIMESCALE = 1
        for cf in allcfs.crazyflies:
              cf.uploadTrajectory(0, 0, traj0)
        fly_nostops()

        TRIALS = 1
        TIMESCALE = 0.5
        for cf in allcfs.crazyflies:
              cf.uploadTrajectory(0, 0, traj4)
        fly_nostops()




#execfile('keyboard.py')
