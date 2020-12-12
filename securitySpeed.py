import math as m
import rospy
import tf
from geometry_msgs.msg import TransformStamped, Point, Pose
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String

a=1.35
b=0.85
c=1.1

########################################__creation des fonctions__########################################################
sigmoid = lambda v : 1/(1+m.exp(-v))
distance_ellipsoid= lambda x,y,z : (x**2/a**2)+(y**2/b**2)+(z**2/c**2)     #distance avant le limite (entre 0-1)

def cf_callback(data):
        global securitySpeed1,securitySpeed2,securitySpeed3
        try:
            cf_id=data.data[0]
            speed=data.data[1:]
            print(speed)
            try:
                speed=float(speed)
                if cf_id == "1":
                    finalspeed=speed*securitySpeed1
                    
                if cf_id == "2":
                    finalspeed=speed*securitySpeed2
                    
                if cf_id == "3":
                    finalspeed=speed*securitySpeed3
                    
                
                print("FINAAAAAAAAAAAAAAL: ",securitySpeed1)
                securitySpeed_publisher.publish(cf_id+str(finalspeed))

                
            except:
                securitySpeed_publisher.publish(data.data)

                if data.data[1:] == "THREE":
                    swarmFollow_publisher.publish("FOLLOW_ME")
                if data.data[1:] == "INDEX":
                    swarmFollow_publisher.publish("STOP_FOLLOW_ME")
            

        except:
            cf = data.transforms[0]
            if cf.child_frame_id == 'cf1':
                x = cf.transform.translation.x
                y = cf.transform.translation.y
                z = cf.transform.translation.z
                #print("cf1_pose: received")

                securitySpeed1 = sigmoid(18*((1-distance_ellipsoid(x,y,z))-0.3))
                
                print("cf1_speed: "+str(securitySpeed1))

            if cf.child_frame_id == 'cf2':
                x = cf.transform.translation.x
                y = cf.transform.translation.y
                z = cf.transform.translation.z
                #print("cf2_pose: received")

                securitySpeed2 = sigmoid(18*((1-distance_ellipsoid(x,y,z))-0.3))
                print("cf2_speed: "+str(securitySpeed2))


            if cf.child_frame_id == 'cf3':
                x = cf.transform.translation.x
                y = cf.transform.translation.y
                z = cf.transform.translation.z
                #print("cf3_pose: received")
                
                securitySpeed3 = sigmoid(18*((1-distance_ellipsoid(x,y,z))-0.3))
                print("cf3_speed: "+str(securitySpeed3))






if __name__ == '__main__':
    global securitySpeed1,securitySpeed2,securitySpeed3,speed,cf_id
    securitySpeed1=1
    securitySpeed2=1
    securitySpeed3=1
    rospy.init_node('security', anonymous=True)
    securitySpeed_publisher = rospy.Publisher('/cf2/filtredsignal', String, queue_size=10)
    swarmFollow_publisher = rospy.Publisher('/swarmfollow', String, queue_size=10)
    cf2_subscriber = rospy.Subscriber('/tf', TFMessage, cf_callback)
    cf2_subscriber2 = rospy.Subscriber('/cf2/signal', String, cf_callback)
    rospy.spin()



