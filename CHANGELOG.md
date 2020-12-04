# CHANGELOG
(also accessible in folder on worktable)

## 15 October 2020
## Fly two drones with simplified parameters.

Approach: Motion capture
-         Simplification: initial location constraint removed (less constraints possibly more reliable)
-         Simplification: motion capture with identical marker arrangements (may not intersect paths)
-         “Duration” parameter: 3s (slower response time)
-         “Z” parameter: 0.3m (higher chance of localization?)
Tests: 
 1 hovers -> 1 moves in x direction
 2 hover -> 2 move in x direction (battery failure?)
Error and fix: (documented) Multicast does not allow wired connection. Fixed by stopping avahi-daemon.service
Error and fix: Drone wobbles. Vibration due to unbalanced propellers. Replaced
 

## 16 October 2020
## Title: Drone balancing and first intersections

Changes: Motion capture approach
-         After each test, reposition real markers + recreate Motive rigid body
-         When changing the real number of drones: edit the two .yaml files.
-         Only then turn on the drones.
-         Note: niceHover.py script has been edited for multiple tests.
Tests: 
 CF3 hovers -> CF3 moves along full x direction -> CF3 does Figure of 8
CF2 hovers -> CF2 does Figure of 8
CF1/2 do simult Figure of 8 -> CF1/2 hover different altitudes -> CF1/2 do simult opp Figures of 8
Error and fix: CF2 wobbles. Due to ball on thin guards. Ball repositioned (guards to be printed)
Error: CF1 unflyable (to be inspected)
Error: cmdVelocityWorld() does not work (probably pull latest version https://github.com/USC-ACTLab/crazyswarm/pull/202 )
Warning: CFs wobble on takeoff (probably imbalanced)
 
 
## 21 October 2020
## Objective: Update&Maintenance
Logitech camera functional, records with ros launchfile as start-recording.sh
Vid+Twitch streaming works from live-demo.sh.
Lin-Win problem debugged, documentation updated. Suggestion: upload as a readme to github, start automating start scripts.
[Advanced on:                ]
 
Crazyswarm repo updated, –sim debugged (recompiled), real flight debugged (new cpp).

Objective: Running custom shapes.
Tests: CF1 in Arena >> helicoidal shape successful
For full effect: a little slow. Maybe two drones for full effect.


[Advanced on:                ]
 
 
## 22 October 2020
## Objective: Need to use lowlevel commands for behaviour planning.
Hardware: camera 4 repositioned, immediate positive effects. Drone batteries charged, problems have disappeared.
Tests: CF1 in Arena >> cmdVelocityCircle.py
Error1+experimentation: yesterday’s updated repo allows for cmdVelocityWorld() but CF dies shortly thereafter. Suggestion: to use NotifySetpointStop(). 
[Advanced on:                ]
 
Objective: Running custom shapes.
Tests: CF1 in Arena >> ldev / rdev / f8xz shapes. Parameters tweaked and all added to demo script.
Error2: some trajectories too slow/could be extended to different usecases.
-         (verified.) Should fix each shape speed using TIMESCALE
-         (to do.) Should change size of ldev/rdev for different figures
                                                                                                                                             [Advanced on:                ]
 
Error3+experimentation: Should loadTrajectory() on key press, however it currently takes too long to load to drone. Suggestion: to combine trajectories in advance. 
                                                                                                                                              [Advanced on:                ]
 
 
 
## 23 October 2020
## Objective: functional position callbacks
Behaviours must be configured from position of drone, new script to be developed with ROS callbacks. If it doesn’t work, might have to use python callbacks. 
                                                                                                                                     [Advanced on:                 ]
Objective: Keyboard controller has been developed using CF API.
Error1+fix: Couldn’t find position information. Now successfully extracted from /tf in callback.
 
Error2+experimentation: Low level commands are pre-empted after 2-3s of idle time, cf flies back to takeoff home position. Tested using:
-         NotifySetpointStop(): normally used to end low-level commands. Currently doesn’t have an effect. Maybe this functionality works in coordination with something else?? 
-         cmdPosition(): at button press, much effort needed to reach point, reminiscent of a misbehaving controller.
-         stopEngines() correctly kills the engines, permanently only when takeoff() is removed from the code (to be verified).
                                                                                                                                             [Advanced on:                 ]
 
Error 3+experimentation: issue explained here - https://github.com/USC-ACTLab/crazyswarm/issues/266
                                                                                                                               [Advanced on:                 ]
 
Error4: only CF rigidbodies are detected. Solution available: set others up for OT in cs_server.cpp. https://github.com/USC-ACTLab/crazyswarm/issues/263
                                                                                                                                     [Advanced on:                 ]
 
## 24 October 2020
## Objective: setting up Gazebo simulator
Approach: followed instructions from wuwushrek/sim_cf . If it doesn’t work, might have to use CrazyS.                                                                  [Advanced on:                 ]
Tests: open gazebo and spawn 7 CFs >> CF1/CF2 given a handler >> Gazebo running >> fancy_traj.py
Setting up: sim_session will open upon $./demo-layout.sh, also should $nano sim-setup.txt
Error1: crashing and erratic behaviour.
Note: The ROS topic /cf2/local_position not available to correct behaviour.
Recommendation: custom subscribe/publisher node finds topic from Gazebo’s API and adds to cf2/local_position (/tf for general use)                                                
                                                                                                                                     [Advanced on:                 ]
 
Error2: 1 core over 100%. Suggestion: turn it off and on again. Might need a separate computer.
                                                                           [Advanced on:                 ]
 


