#! /usr/bin/env python
from __future__ import print_function
import rospy

import threading
from math import sqrt, pow
import smach_ros
import smach

import actionlib
from actionlib_tutorials.msg import my_newAction, my_newGoal, MachineAction, FibonacciAction, FibonacciGoal, doTrajAction, doTrajGoal
from geometry_msgs.msg import Point
from smach import StateMachine, Concurrence
from smach_ros import ActionServerWrapper, ServiceState, SimpleActionState, MonitorState, IntrospectionServer
import std_srvs.srv
import turtlesim.srv
import turtlesim.msg
import turtle_actionlib.msg
import sys

def polygonial():
    ids = [5, 2, 3]
    #define the differents points
    test_point= Point()
    test_point.x = 0
    test_point.y = 0
    test_point.z = 0.5

    home_points_fo8 = [Point(), Point(), Point()]

    home_points_fo8[0].x = 0.0
    home_points_fo8[0].y = 0.0
    home_points_fo8[0].z = 0.3

    home_points_fo8[1].x = 0.3
    home_points_fo8[1].y = 0.0
    home_points_fo8[1].z = 0.3

    home_points_fo8[2].x = -0.3
    home_points_fo8[2].y = 0.0
    home_points_fo8[2].z = 0.3


    home_points_heli = [Point(), Point(), Point()]

    home_points_heli[0].x = 0.0
    home_points_heli[0].y = 0.0
    home_points_heli[0].z = 0.3

    home_points_heli[1].x = 0.0
    home_points_heli[1].y = 0.9-0.4
    home_points_heli[1].z = 0.9

    home_points_heli[2].x = -0.3
    home_points_heli[2].y = 0.0
    home_points_heli[2].z = 0.3

    my_points = [Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point()]

    my_points[0].x = 0.5
    my_points[0].y = -0.5
    my_points[0].z = 0.5

    my_points[1].x = 0.7
    my_points[1].y = 0.0
    my_points[1].z = 0.5

    my_points[2].x = 0.5
    my_points[2].y = 0.5
    my_points[2].z = 0.5

    my_points[3].x = 0
    my_points[3].y = 0.7
    my_points[3].z = 0.5

    my_points[4].x = -0.5
    my_points[4].y = 0.5
    my_points[4].z = 0.5

    my_points[5].x =  -0.7
    my_points[5].y = 0
    my_points[5].z = 0.5

    my_points[6].x = -0.5
    my_points[6].y = -0.5
    my_points[6].z = 0.5

    my_points[7].x = 0
    my_points[7].y = -0.7
    my_points[7].z = 0.5





    # Create a SMACH state machine
    sm0 = StateMachine(outcomes=['succeeded','aborted','preempted'])
    #progressively add drones
    with sm0:

        """
        StateMachine.add('HELI_START',
                         SimpleActionState('drone1detect_perimeter',
                                            my_newAction, goal = my_newGoal(point = home_points_heli[0], id = ids[0] )),
                        transitions={'succeeded' : 'HELI_EXECUTE', 'aborted' : 'land_all', 'preempted' : 'land_all'})

        StateMachine.add('HELI_EXECUTE',
                         SimpleActionState('trajectory_action',
                                            doTrajAction, goal = doTrajGoal(shape = 1, id = ids[0] )),
                         transitions={'succeeded' : 'HELI_END', 'aborted' : 'land_all', 'preempted' : 'land_all'})



        StateMachine.add('HELI_END',
                        SimpleActionState('land_drone1',
                                             my_newAction, goal = my_newGoal(point = my_points[3], id = ids[0]) ),
                                             transitions={'succeeded' : 'drone2-FIG8', 'aborted' : 'land_all', 'preempted' : 'land_all'})

        """
        StateMachine.add('drone2-FIG8',
                         SimpleActionState('drone2detect_perimeter',
                                            my_newAction, goal = my_newGoal(point = home_points_fo8[2], id = ids[1] )),
                        transitions={'succeeded' : 'drone3-FIG8', 'aborted' : 'land_all', 'preempted' : 'land_all'})
        StateMachine.add('drone3-FIG8',
                         SimpleActionState('drone3detect_perimeter',
                                            my_newAction, goal = my_newGoal(point = home_points_fo8[1], id = ids[2] )),
                        transitions={'succeeded' : 'FIG8_EXECUTE', 'aborted' : 'land_all', 'preempted' : 'land_all'})


        # fig8_sm = Concurrence(['drone2-2', 'land_all'], 'land_all', 
        #                                 outcome_map={'drone2-2' : {'FIG8_EXECUTE_drone3' : 'succeeded', 'FIG8_EXECUTE_drone2' : 'succeeded'}})

        fig8_sm = Concurrence(['succeeded', 'aborted', 'preempted'], 'succeeded')#,
                                        #outcome_map={'succeeded' : 'drone2-2', 'preempted' : 'land_all', 'aborted' : 'land_all' })


        StateMachine.add('FIG8_EXECUTE', fig8_sm, transitions={'succeeded' : 'drone2-2', 'aborted' : 'land_all', 'preempted' : 'land_all'})
        ## Bug BBL: DO NOT try to decide on outcomes in a concurrence, do it in the SM.add!!
        with fig8_sm:
            Concurrence.add('FIG8_EXECUTE_drone2',
            SimpleActionState('fig8_drone2',
                            doTrajAction, goal = doTrajGoal(shape = 8, id = ids[1])))

            Concurrence.add('FIG8_EXECUTE_drone3',
            SimpleActionState('fig8_drone3',
                                doTrajAction, goal = doTrajGoal(shape = 8, id = ids[2])))


        heli_sm = Concurrence(['succeeded', 'aborted', 'preempted'], 'succeeded')#,
                                        #outcome_map={'succeeded' : 'drone2-2', 'preempted' : 'land_all', 'aborted' : 'land_all' })


        StateMachine.add('HELI_EXECUTE', heli_sm, transitions={'succeeded' : 'drone2-2', 'aborted' : 'land_all', 'preempted' : 'land_all'})
        ## Bug BBL: DO NOT try to decide on outcomes in a concurrence, do it in the SM.add!!
        with heli_sm:
            Concurrence.add('HELI_EXECUTE_drone2',
            SimpleActionState('heli_drone2',
                            doTrajAction, goal = doTrajGoal(shape = 0, id = ids[1])))

            Concurrence.add('HELI_EXECUTE_drone3',
            SimpleActionState('heli_drone3',
                                doTrajAction, goal = doTrajGoal(shape = 0, id = ids[2])))





        fig8_end = Concurrence(['succeeded', 'aborted', 'preempted'], 'succeeded')#,
                                        #outcome_map={'succeeded' : 'drone2-2', 'preempted' : 'land_all', 'aborted' : 'land_all' })


        StateMachine.add('FIG8_END', fig8_end, transitions={'succeeded' : 'drone2-2', 'aborted' : 'land_all', 'preempted' : 'land_all'})
        ## Bug BBL: DO NOT try to decide on outcomes in a concurrence, do it in the SM.add!!





        with fig8_end:
            Concurrence.add('FIG8_END_drone2',
                SimpleActionState('land_drone2',
                                    my_newAction, goal = my_newGoal(point = my_points[3], id = ids[1]) ))

            Concurrence.add('FIG8_END_drone3',
                SimpleActionState('land_drone3',
                                    my_newAction, goal = my_newGoal(point = my_points[3], id = ids[2]) ))

                                    



        StateMachine.add('drone2-2',
                         SimpleActionState('drone2detect_perimeter',
                                            my_newAction, goal = my_newGoal(point = my_points[2], id = ids[1] )),
                        transitions={'succeeded' : 'drone3-1', 'aborted' : 'land_all', 'preempted' : 'land_all'})

        StateMachine.add('drone3-1',
                         SimpleActionState('drone3detect_perimeter',
                                            my_newAction, goal = my_newGoal(point = my_points[0], id = ids[2] )),
                        transitions={'succeeded' : 'drone1-3', 'aborted' : 'land_all', 'preempted' : 'land_all'})

        StateMachine.add('drone1-3',
                         SimpleActionState('drone1detect_perimeter',
                                            my_newAction, goal = my_newGoal(point = my_points[4], id = ids[0] )),
                        transitions={'succeeded' : 'infinit_loop', 'aborted' : 'land_all', 'preempted' : 'land_all'})

        


        


        land_sm =  Concurrence(['succeeded', 'aborted', 'preempted'], 'succeeded')

        StateMachine.add('land_all', land_sm)

        with land_sm:



            # DUPLICATA FOR HIGH LEVEL
            #  #Land Drone If Aborted
            Concurrence.add('LAND_DRONE1',
                            SimpleActionState('land_drone1',
                                                my_newAction, goal = my_newGoal(point = my_points[3], id = ids[0])  ))

            # #Land Drone If Aborted
            Concurrence.add('LAND_DRONE2',
                            SimpleActionState('land_drone2',
                                                my_newAction, goal = my_newGoal(point = my_points[3], id = ids[1])  ))

            # #Land Drone If Aborted
            Concurrence.add('LAND_DRONE3',
                            SimpleActionState('land_drone3',
                                                my_newAction, goal = my_newGoal(point = my_points[3], id = ids[2])  ))

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

            test_drone = StateMachine(outcomes=['succeeded','aborted','preempted'])  # ['succeeded','aborted','preempted']


            #Concurrence.add('DRONE1', test_drone)
            with test_drone:
                
                order = (5, 6, 7, 0, 1, 2, 3, 4)
                for i in range(7):
                    point_for_state = Point()
                    point_for_state.x = test_point.x
                    point_for_state.y = test_point.y
                    if i <= 4 :
                        point_for_state.z = test_point.z + (i*0.1)
                    else :
                        point_for_state.z = test_point.z - (i*0.1) + 0.8 
                    StateMachine.add('DRONE1-' + str(order[i]),
                                     SimpleActionState('drone1detect_perimeter',
                                                          my_newAction, goal = my_newGoal(point = point_for_state, id = ids[0])),
                                       transitions={'succeeded' : 'DRONE1-' + str(order[i+1]), 'aborted' : 'LAND_DRONE1', 'preempted' : 'LAND_DRONE1'})

                #make it infinit
                smach.StateMachine.add('DRONE1-' + str(order[-1]),
                               SimpleActionState('drone1detect_perimeter',
                                                    my_newAction, goal = my_newGoal(point = test_point, id = ids[0])),
                              transitions={'succeeded' : 'DRONE1-'  + str(order[0]), 'aborted' : 'LAND_DRONE1', 'preempted' : 'LAND_DRONE1'})

            Concurrence.add('DRONE1', drone1)
            # Open the container
            with drone1:
                #add each state
                order = (5, 6, 7, 0, 1, 2, 3, 4)
                for i in range(7):
                    point_for_state = Point()
                    point_for_state.x = my_points[order[i]].x
                    point_for_state.y = my_points[order[i]].y
                    if i <= 4 :
                        point_for_state.z = my_points[order[i]].z + (i*0.1)
                    else :
                        point_for_state.z = my_points[order[i]].z - (i*0.1) + 0.8 
                    StateMachine.add('DRONE1-' + str(order[i]),
                                     SimpleActionState('drone1detect_perimeter',
                                                          my_newAction, goal = my_newGoal(point = point_for_state, id = ids[0])),
                                       transitions={'succeeded' : 'DRONE1-' + str(order[i+1]), 'aborted' : 'LAND_DRONE1', 'preempted' : 'LAND_DRONE1'})

                #make it infinit
                smach.StateMachine.add('DRONE1-' + str(order[-1]),
                               SimpleActionState('drone1detect_perimeter',
                                                    my_newAction, goal = my_newGoal(point = my_points[order[-1]], id = ids[0])),
                              transitions={'succeeded' : 'DRONE1-'  + str(order[0]), 'aborted' : 'LAND_DRONE1', 'preempted' : 'LAND_DRONE1'})


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
                order = (3, 4, 5, 6, 7, 0, 1, 2 )
                for i in range(7):
                    point_for_state = Point()
                    point_for_state.x = my_points[order[i]].x
                    point_for_state.y = my_points[order[i]].y
                    if i <= 4 :
                        point_for_state.z = my_points[order[i]].z + (i*0.1)
                    else :
                        point_for_state.z = my_points[order[i]].z - (i*0.1) + 0.8 
                    StateMachine.add('DRONE2-' + str(order[i]),
                                     SimpleActionState('drone2detect_perimeter',
                                                          my_newAction, goal = my_newGoal(point = point_for_state, id = ids[1])),
                                       transitions={'succeeded' : 'DRONE2-' + str(order[i+1]), 'aborted' : 'LAND_DRONE2', 'preempted' : 'LAND_DRONE2'})

                #make it infinit
                smach.StateMachine.add('DRONE2-' + str(order[-1]),
                             SimpleActionState('drone2detect_perimeter',
                                                     my_newAction, goal = my_newGoal(point = my_points[order[-1]], id = ids[1])),
                             transitions={'succeeded' : 'DRONE2-' + str(order[0]), 'aborted' : 'LAND_DRONE2', 'preempted' : 'LAND_DRONE2'})


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
                order = (1, 2, 3, 4, 5, 6, 7, 0)
                for i in range(7):
                    point_for_state = Point()
                    point_for_state.x = my_points[order[i]].x
                    point_for_state.y = my_points[order[i]].y
                    if i <= 4 :
                        point_for_state.z = my_points[order[i]].z + (i*0.1)
                    else :
                        point_for_state.z = my_points[order[i]].z -(i*0.1) + 0.8 
                    StateMachine.add('DRONE3-' + str(order[i]),
                                     SimpleActionState('drone3detect_perimeter',
                                                          my_newAction, goal = my_newGoal(point =point_for_state, id = ids[2])),
                                       transitions={'succeeded' : 'DRONE3-' + str(order[i+1]), 'aborted' : 'LAND_DRONE3', 'preempted' : 'LAND_DRONE3'})

                #make it infinit
                smach.StateMachine.add('DRONE3-' + str(order[-1]),
                                   SimpleActionState('drone3detect_perimeter',
                                                   my_newAction, goal = my_newGoal(point = my_points[order[-1]], id = ids[2])),
                                   transitions={'succeeded' : 'DRONE3-'  + str(order[0]), 'aborted' : 'LAND_DRONE3', 'preempted' : 'LAND_DRONE3'})


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
