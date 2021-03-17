import math as m
import rospy
import tf
from geometry_msgs.msg import TransformStamped, Point, Pose
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String, Float64MultiArray
from pycrazyswarm import *

a=1.35
b=0.85
c=1.1

########################################__creation des fonctions__########################################################
sigmoid = lambda v : 1/(1+m.exp(-v))                                       #fonction sigmoid entre 0-1 avec une evolution exponentielle proche de 0
distance_ellipsoid= lambda x,y,z : (x**2/a**2)+(y**2/b**2)+(z**2/c**2)     #distance avant sortie de l'arene normalise entre 0-1
closeness = lambda xa, xb, ya, yb, za, zb : ((xa-xb)**2+(ya-yb)**2+(za-zb)**2)**0.5                        #distance avant sortie de l'arene normalise entre 0-1

#TODO: ADEM- CHANGE ARENA BOUNDARIES AND ADD FUNCTION FOR DRONE COLLISIONS.

def cf_callback(data):
        global securitySpeed, msg, x1, x2, x3, y1, y2, y3, z1, z2, z3

        collision_distance = 0.6

        #According to the drone position assign a value beetween 0-1 following a sigmoid function --> tend to 0 when the drone is close to the net
        cf = data.transforms[0]

        if cf.child_frame_id == 'cf1':

            x1 = cf.transform.translation.x
            y1 = cf.transform.translation.y
            z1 = cf.transform.translation.z

            securitySpeed1 = sigmoid(18*((1-distance_ellipsoid(x1,y1,z1))-0.3))
            securitySpeed[0]=securitySpeed1
            print("CF1:", distance_ellipsoid(x1,y1,z1))

            distance1_2 = closeness(x1, x2, y1, y2, z1, z2)
            distance1_3 = closeness(x1, x3, y1, y3, z1, z3)
            if distance1_2 < collision_distance or distance1_3 < collision_distance:
                print("CF1 slowing down.")
                securitySpeed[0]= 0.2


        if cf.child_frame_id == 'cf2':

            x2 = cf.transform.translation.x
            y2 = cf.transform.translation.y
            z2 = cf.transform.translation.z

            securitySpeed2 = sigmoid(18*((1-distance_ellipsoid(x2,y2,z2))-0.3))
            securitySpeed[1]=securitySpeed2
            print("CF2:", distance_ellipsoid(x2,y2,z2))

            distance2_1 = closeness(x1, x2, y1, y2, z1, z2)
            distance2_3 = closeness(x2, x3, y2, y3, z2, z3)
            if distance2_1 < collision_distance or distance2_3 < collision_distance:
                print("CF2 slowing down.")
                securitySpeed[1]= 0.2

        if cf.child_frame_id == 'cf3': 

            x3 = cf.transform.translation.x
            y3 = cf.transform.translation.y
            z3 = cf.transform.translation.z
        
            securitySpeed3 = sigmoid(18*((1-distance_ellipsoid(x3,y3,z3))-0.3))
            securitySpeed[2]=securitySpeed3
            print("CF3:", distance_ellipsoid(x3,y3,z3))

            distance3_1 = closeness(x1, x3, y1, y3, z1, z3)
            distance3_2 = closeness(x2, x3, y2, y3, z2, z3)
            if distance3_1 < collision_distance or distance3_2 < collision_distance:
                print("CF3 slowing down.")
                securitySpeed[2]= 0.2

        msg.data = securitySpeed
        securitySpeed_publisher.publish(msg)



if __name__ == '__main__':
    global securitySpeed, msg, x1, x2, x3, y1, y2, y3, z1, z2, z3

    x1, x2, x3, y1, y2, y3, z1, z2, z3 = 1000, 100, 10, 1000, 100, 10, 1000, 100, 10
    securitySpeed= [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    msg = Float64MultiArray()

    rospy.init_node('security', anonymous=True)

    securitySpeed_publisher = rospy.Publisher('/cf2/filtredsignal', Float64MultiArray, queue_size=10)
    cf2_subscriber = rospy.Subscriber('/tf', TFMessage, cf_callback)
    rospy.spin()