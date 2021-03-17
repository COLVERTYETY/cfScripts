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


x = {}
y = {}
z = {}

all_drones = ['cf1', 'cf2', 'cf3', 'cf4', 'cf5', 'cf6']

def cf_callback(data):
        global securitySpeed, msg, x1, x2, x3, y1, y2, y3, z1, z2, z3, x4, x5, x6, y4, y5, y6, z4, z5, z6

        collision_distance = 0.6

        #According to the drone position assign a value beetween 0-1 following a sigmoid function --> tend to 0 when the drone is close to the net
        cf = data.transforms[0]

        x[cf.child_frame_id]=cf.transform.translation.x
        y[cf.child_frame_id]=cf.transform.translation.y
        z[cf.child_frame_id]=cf.transform.translation.z

        securitySpeed[cf.child_frame_id] = sigmoid(18*((1-distance_ellipsoid(x1,y1,z1))-0.3))

        print(str(cf.child_frame_id) + ' ' + str(distance_ellipsoid(x[cf.child_frame_id],y[cf.child_frame_id],z[cf.child_frame_id])))

        for drone in all_drones:
            if drone != cf.child_frame_id:
                if closeness(x[cf.child_frame_id], y[cf.child_frame_id], z[cf.child_frame_id], x[drone], y[drone], z[drone]):
                    print(str(cf.child_frame_id) + 'slowing down')
                    securitySpeed[cf.child_frame_id] = 0.2



        if cf.child_frame_id == 'cf4': 

            x4 = cf.transform.translation.x
            y4 = cf.transform.translation.y
            z4 = cf.transform.translation.z
        
            securitySpeed4 = sigmoid(18*((1-distance_ellipsoid(x4,y4,z4))-0.3))
            securitySpeed[3]=securitySpeed4
            print("CF4:", distance_ellipsoid(x4,y4,z4))

            # distance3_1 = closeness(x1, x3, y1, y3, z1, z3)
            # distance3_2 = closeness(x2, x3, y2, y3, z2, z3)
            # if distance3_1 < collision_distance or distance3_2 < collision_distance:
            #     print("CF3 slowing down.")
            #     securitySpeed[2]= 0.2

        if cf.child_frame_id == 'cf5': 

            x5 = cf.transform.translation.x
            y5 = cf.transform.translation.y
            z5 = cf.transform.translation.z
        
            securitySpeed5 = sigmoid(18*((1-distance_ellipsoid(x5,y5,z5))-0.3))
            securitySpeed[4]=securitySpeed5
            print("CF5:", distance_ellipsoid(x5,y5,z5))

            # distance3_1 = closeness(x1, x3, y1, y3, z1, z3)
            # distance3_2 = closeness(x2, x3, y2, y3, z2, z3)
            # if distance3_1 < collision_distance or distance3_2 < collision_distance:
            #     print("CF3 slowing down.")
            #     securitySpeed[2]= 0.2

        if cf.child_frame_id == 'cf6': 

            x6 = cf.transform.translation.x
            y6 = cf.transform.translation.y
            z6 = cf.transform.translation.z
        
            securitySpeed6 = sigmoid(18*((1-distance_ellipsoid(x6,y6,z6))-0.3))
            securitySpeed[5]=securitySpeed6
            print("CF6:", distance_ellipsoid(x6,y6,z6))

            # distance3_1 = closeness(x1, x3, y1, y3, z1, z3)
            # distance3_2 = closeness(x2, x3, y2, y3, z2, z3)
            # if distance3_1 < collision_distance or distance3_2 < collision_distance:
            #     print("CF3 slowing down.")
            #     securitySpeed[2]= 0.2


        msg.data = securitySpeed
        securitySpeed_publisher.publish(msg)



if __name__ == '__main__':
    global securitySpeed, msg, x1, x2, x3, y1, y2, y3, z1, z2, z3, x4, x5, x6, y4, y5, y6, z4, z5, z6

    x1, x2, x3, y1, y2, y3, z1, z2, z3 = 1000, 100, 10, 1000, 100, 10, 1000, 100, 10
    x4, x5, x6, y4, y5, y6, z4, z5, z6 = 1000, 100, 10, 1000, 100, 10, 1000, 100, 10
    securitySpeed= [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    msg = Float64MultiArray()

    rospy.init_node('security', anonymous=True)

    securitySpeed_publisher = rospy.Publisher('/cf2/filtredsignal', Float64MultiArray, queue_size=10)
    cf2_subscriber = rospy.Subscriber('/tf', TFMessage, cf_callback)
    rospy.spin()