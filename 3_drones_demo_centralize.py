#! /usr/bin/env python
from __future__ import print_function
import rospy

import threading
from math import sqrt, pow
import smach_ros
import smach

import actionlib
from actionlib_tutorials.msg import my_newAction, my_newGoal, MachineAction, FibonacciAction, FibonacciGoal
from geometry_msgs.msg import Point
from smach import StateMachine, Concurrence
from smach_ros import ActionServerWrapper, ServiceState, SimpleActionState, MonitorState, IntrospectionServer
import std_srvs.srv
import turtlesim.srv
import turtlesim.msg
import turtle_actionlib.msg
import sys

def polygonial():
    ids = [4, 5, 6]
    #define the differents points
    my_points = [Point(), Point(), Point(), Point()]

    my_points[0].x = 0.9 -0.4
    my_points[0].y = 0.0 -0.4
    my_points[0].z = 0.9

    my_points[1].x = 0.9 -0.4
    my_points[1].y = 0.9-0.4
    my_points[1].z = 0.9

    my_points[2].x = 0.0-0.4
    my_points[2].y = 0.9-0.4
    my_points[2].z = 0.9

    my_points[3].x = 0.0-0.4
    my_points[3].y = 0.0-0.4
    my_points[3].z = 0.9

    # Create a SMACH state machine
    sm0 = StateMachine(outcomes=['succeeded','aborted','preempted'])
    #progressively add drones
    with sm0:
        # StateMachine.add('drone1-1',
        #                  SimpleActionState('drone1detect_perimeter',
        #                                     my_newAction, goal = my_newGoal(point = my_points[0], id = ids[0] )),
        #                 transitions={'succeeded' : 'drone1-2'})
        # StateMachine.add('drone1-2',
        #                  SimpleActionState('drone1detect_perimeter',
        #                                     my_newAction, goal = my_newGoal(point = my_points[1], id = ids[0] )),
        #                 transitions={'succeeded' : 'drone2-1'})
        # StateMachine.add('drone2-1',
        #                  SimpleActionState('drone2detect_perimeter',
        #                                     my_newAction, goal = my_newGoal(point = my_points[0], id = ids[1] )),
        #                 transitions={'succeeded' : 'drone1-3'})

        StateMachine.add('drone1-3',
                         SimpleActionState('drone1detect_perimeter',
                                            my_newAction, goal = my_newGoal(point = my_points[2], id = ids[0] )),
                        transitions={'succeeded' : 'drone2-2', 'aborted' : 'land_all'})
        StateMachine.add('drone2-2',
                         SimpleActionState('drone2detect_perimeter',
                                            my_newAction, goal = my_newGoal(point = my_points[1], id = ids[1] )),
                        transitions={'succeeded' : 'drone3-1', 'aborted' : 'land_all'})
        StateMachine.add('drone3-1',
                         SimpleActionState('drone3detect_perimeter',
                                            my_newAction, goal = my_newGoal(point = my_points[0], id = ids[2] )),
                        transitions={'succeeded' : 'infinit_loop', 'aborted' : 'land_all'})


        land_sm =  Concurrence(['succeeded', 'aborted', 'preempted'], 'succeeded')

        StateMachine.add('land_all', land_sm)

        with land_sm:



            # DUPLICATA FOR HIGH LEVEL
            #  #Land Drone If Aborted
            Concurrence.add('LAND_DRONE1',
                            SimpleActionState('land_drone1',
                                                my_newAction, goal = my_newGoal(point = my_points[3], id = ids[0])))

            # #Land Drone If Aborted
            Concurrence.add('LAND_DRONE2',
                            SimpleActionState('land_drone2',
                                                my_newAction, goal = my_newGoal(point = my_points[3], id = ids[1])))

            # #Land Drone If Aborted
            Concurrence.add('LAND_DRONE3',
                            SimpleActionState('land_drone3',
                                                my_newAction, goal = my_newGoal(point = my_points[3], id = ids[2])))

            ############################################

        sm1 = Concurrence(['succeeded', 'aborted', 'preempted'],
                'succeeded',
                #child_termination_cb = lambda so: True,
                #outcome_map = {
                    #'succeeded':{'WAIT_FOR_CLEAR':'valid'},
	                #'aborted':{'DRONE1':'aborted'}}
                )

        StateMachine.add('infinit_loop', sm1)

        with sm1:
            drone1 = StateMachine(outcomes=['succeeded','aborted','preempted'])  # ['succeeded','aborted','preempted']


            Concurrence.add('DRONE1', drone1)
            # Open the container
            with drone1:
                #add each state
                order = (3, 0, 1, 2)
                for i in range(3):
                    StateMachine.add('DRONE1-' + str(order[i]),
                                     SimpleActionState('drone1detect_perimeter',
                                                          my_newAction, goal = my_newGoal(point = my_points[order[i]], id = ids[0])),
                                       transitions={'succeeded' : 'DRONE1-' + str(order[i+1]), 'aborted' : 'LAND_DRONE1'})

                    #make it infinit
                smach.StateMachine.add('DRONE1-' + str(2),
                               SimpleActionState('drone1detect_perimeter',
                                                    my_newAction, goal = my_newGoal(point = my_points[order[3]], id = ids[0])),
                              transitions={'succeeded' : 'DRONE1-'  + str(order[0]), 'aborted' : 'LAND_DRONE1'})


                # #Land Drone If Aborted
                smach.StateMachine.add('LAND_DRONE1',
                               SimpleActionState('land_drone1',
                                                    my_newAction, goal = my_newGoal(point = my_points[3], id = ids[0])),
                              transitions={'succeeded' : 'LAND_DRONE1'})



            drone2 = StateMachine(outcomes=['succeeded','aborted','preempted'])  # ['succeeded','aborted','preempted']


            Concurrence.add('DRONE2', drone2)
            # Open the container
            with drone2:
                #add each state
                order = (2, 3, 0, 1)
                for i in range(3):
                    StateMachine.add('DRONE2-' + str(order[i]),
                                     SimpleActionState('drone2detect_perimeter',
                                                          my_newAction, goal = my_newGoal(point = my_points[order[i]], id = ids[1])),
                                       transitions={'succeeded' : 'DRONE2-' + str(order[i+1]), 'aborted' : 'LAND_DRONE2'})

                #make it infinit
                smach.StateMachine.add('DRONE2-' + str(1),
                             SimpleActionState('drone2detect_perimeter',
                                                     my_newAction, goal = my_newGoal(point = my_points[order[3]], id = ids[1])),
                             transitions={'succeeded' : 'DRONE2-' + str(order[0]), 'aborted' : 'LAND_DRONE2'})


                # #Land Drone If Aborted
                smach.StateMachine.add('LAND_DRONE2',
                               SimpleActionState('land_drone2',
                                                    my_newAction, goal = my_newGoal(point = my_points[3], id = ids[1])),
                              transitions={'succeeded' : 'LAND_DRONE2'})


            drone3 = StateMachine(outcomes=['succeeded','aborted','preempted'])  # ['succeeded','aborted','preempted']


            Concurrence.add('DRONE3-', drone3)
            # Open the container
            with drone3:
                #add each state
                order = (1, 2, 3, 0)
                for i in range(3):
                    StateMachine.add('DRONE3-' + str(order[i]),
                                     SimpleActionState('drone3detect_perimeter',
                                                          my_newAction, goal = my_newGoal(point = my_points[order[i]], id = ids[2])),
                                       transitions={'succeeded' : 'DRONE3-' + str(order[i+1]), 'aborted' : 'LAND_DRONE3'})

                #make it infinit
                smach.StateMachine.add('DRONE3-' + str(0),
                                   SimpleActionState('drone3detect_perimeter',
                                                   my_newAction, goal = my_newGoal(point = my_points[order[3]], id = ids[2])),
                                   transitions={'succeeded' : 'DRONE3-'  + str(order[0]), 'aborted' : 'LAND_DRONE3'})


                # #Land Drone If Aborted
                smach.StateMachine.add('LAND_DRONE3',
                               SimpleActionState('land_drone3',
                                                    my_newAction, goal = my_newGoal(point = my_points[3], id = ids[2])),
                              transitions={'succeeded' : 'LAND_DRONE3'})




    # Attach a SMACH introspection server
    sis = IntrospectionServer('smach_usecase_01', sm0, '/USE_CASE')
    sis.start()

    # Set preempt handler
    smach_ros.set_preempt_handler(sm0)

    # Execute SMACH tree in a separate thread so that we can ctrl-c the script
    smach_thread = threading.Thread(target = sm0.execute)
    smach_thread.start()



if __name__ == '__main__':
    rospy.init_node('smach_usecase_step_06')
    t1 = threading.Thread(target=polygonial)
    t1.start()
    rospy.spin()
