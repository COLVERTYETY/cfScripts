# -*- coding: utf-8 -*-
"""
Trial script: setting up a state machine with the Flight Arena :)
"""

#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *
import uav_trajectory

import time
from enum import Enum

#from udacidrone import Drone
#from udacidrone.messaging import MsgID
#from udacidrone.connection import CrazyflieConnection


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5

class MsgID():
    LOCAL_POSITION = allcfs.crazyflies[0].position()
    LOCAL_VELOCITY = allcfs.crazyflies[0].velocity()
    LOCAL_ACCELERATION = allcfs.crazyflies[0].acceleration()
    STATE = drone.flight_state


class ArenaFlyer():

    def __init__(self):
        #super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        self.swarm = Crazyswarm()
        self.timeHelper = swarm.timeHelper
        self.allcfs = swarm.allcfs

        """ INITIAL STATE """
        self.flight_state = States.MANUAL

        """ CALLBACKS """

        
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

        # m_subscribeCmdVel = n.subscribe(tf_prefix + "/cmd_vel", 1, &CrazyflieROS::cmdVelChanged, this);
        # m_subscribeCmdPosition = n.subscribe(tf_prefix + "/cmd_position", 1, &CrazyflieROS::cmdPositionSetpoint, this);
        # m_subscribeCmdFullState = n.subscribe(tf_prefix + "/cmd_full_state", 1, &CrazyflieROS::cmdFullStateSetpoint, this);
        # m_subscribeCmdVelocityWorld = n.subscribe(tf_prefix + "/cmd_velocity_world", 1, &CrazyflieROS::cmdVelocityWorldSetpoint, this);
        # m_subscribeCmdStop = n.subscribe(m_tf_prefix + "/cmd_stop", 1, &CrazyflieROS::cmdStop, this);

    def local_position_callback(self):

        """ CALLBACK IN MANUAL:
        >ALWAYS> transition(start taking off.) """
        if self.flight_state == States.MANUAL:
            self.takeoff_transition()

        """ CALLBACKS IN TAKEOFF:
        >ALWAYS> have we reached the starting altitude?
        >THEN> setup all waypoints and waypoint transition (moving to waypoints) """
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.all_waypoints = self.calculate_box()
                self.waypoint_transition()

        """ CALLBACKS IN WAYPOINT:
        >ALWAYS> have we reached waypoint x ?
        >REACHED> Are there more waypoints ?
        >MORE WAYPOINTS>start transition (moving to waypoints)
        >NO WAYPOINTS> Are we slow enough to land ?  """
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 0.1:
                if len(self.all_waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):

        """ CALLBACKS IN LANDING:
        >ALWAYS> have we reached altitude
        >REACHED> manual transition (stop and turn off mission) """
        if self.flight_state == States.LANDING:
            if abs(self.local_position[2] < 0.01): #######Probably not correct.
                self.manual_transition()

    # def velocity_callback(self):
    #     if self.flight_state == States.LANDING:
    #         if self.global_position[2] - self.global_home[2] < 0.1:
    #             if abs(self.local_position[2]) < 0.01:
    #                 self.disarming_transition()

    def state_callback(self):

        """ CALLBACKS WHEN CHANGING STATE:
        >IF> mission is activated (not finished by final manual state.)
        >MANUAL> arming transition(arm and set home position)
        >ARMING>>ARMED>> takeoff transition (start taking off.)
        >DISARMING>>NOTARMED>>NOTGUIDED>> manual transition (stop and turn off mission) """

        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    # def calculate_box(self):
    #     print("Setting Home")
    #     local_waypoints = [[10.0, 0.0, 3.0], [10.0, 10.0, 3.0], [0.0, 10.0, 3.0], [0.0, 0.0, 3.0]]
    #     return local_waypoints

    def calculate_box(self):
        cp = self.local_position
        cp[2] = 0
        local_waypoints = [cp + [0.5, 0.0, 0.3], cp + [0.5, 0.5, 0.3], cp + [0.0, 0.5, 0.3], cp + [0.0, 0.0, 0.3]]
        return local_waypoints

    def arming_transition(self):
        print("arming transition")
        self.take_control()
        self.arm()
        self.set_home_position(self.global_position[0], self.global_position[1],
                               self.global_position[2])  # set the current location to be the home position

        self.flight_state = States.ARMING

    def takeoff_transition(self):
        print("takeoff transition")
        """ MOVE TO ALTITUDE
        STATE: TRIGGER POSITION CALLBACK  """
        # self.global_home = np.copy(self.global_position)  # can't write to this variable!
        target_altitude = 0.3
        self.target_position[2] = target_altitude

        self.allcfs.takeoff(targetHeight=target_altitude, duration=1.0+target_altitude)

        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        print("waypoint transition")
        """ POP NEXT TARGET
        > GET TO TARGET
        STATE: TRIGGER POSITION CALLBACK """
        self.target_position = self.all_waypoints.pop(0)
        print('target position', self.target_position)
        #self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], 0.0)
        #######REALTIME POSITIONAL CHANGE!
        self.flight_state = States.WAYPOINT

    def landing_transition(self):
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        print("disarm transition")
        #####self.disarm()
        #####self.release_control()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        print("manual transition")
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """ Start the log """
        #self.start_log("Logs", "NavLog.txt")
        #self.connect()

        print("starting connection")
        # self.connection.start()

        super().start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        """ Stop the log """
        #self.stop_log()


if __name__ == "__main__":

    traj0 = uav_trajectory.Trajectory()
    traj0.loadcsv("figure8.csv")
    #conn = CrazyflieConnection('radio://0/80/2M')
    drone = ArenaFlyer()
    time.sleep(2)
    drone.start()
