from __future__ import print_function
import numpy as np
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import time


rospy.init_node('PositionCommande', anonymous=True)
twist_pub = rospy.Publisher("robot_base_velocity_controller/cmd_vel", Twist, queue_size=1000)
rate = rospy.Rate(10)

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image,queue_size=1000)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/cf2/feed",Image,self.callback)
    


  def callback(self,data):
    
    print ("hello ")
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    kernel = np.ones((5,5), np.uint8)
    width=800
    height=800 
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    l_b1 = np.array([0, 150, 25])
    u_b1= np.array([10, 255, 255])
    l_b2 = np.array([245, 150, 70])
    u_b2= np.array([255, 255, 255])

    mask1 = cv2.inRange(hsv, l_b1, u_b1)

    mask2 = cv2.inRange(hsv, l_b2, u_b2)

    mask = mask1 + mask2

    opening_masked_img = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    opening_masked_img = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    (contours, hierarchy) = cv2.findContours(opening_masked_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    cv2.drawContours(opening_masked_img, contours, -1, (255,255,255), 2)
    masked_img = cv2.bitwise_and(cv_image, cv_image, mask=mask)

    

    for c in contours:
      
      (x, y, w, h) = cv2.boundingRect(c)
      
      vel_x=0
      vel_y=0
      vel_z=0.2

      if cv2.contourArea(c) < 3500:  # supprimer les petits contours, supprimer le bruit
          continue

      x2 = x + (w/2)      
      y2 = y + (h/2)
      x2 = int(round(x2))             #On trouve le centre de notre portail et on place un point au sol pour que le robot se déplace vers celui ci
      y2 = int(round(y2))             #pareil pour la hauteur pour que le drone se déplace vers celui ci
      
      cv2.line(cv_image,(width//2,height),(x2,y2),(0,255,0),1)

      font = cv2.FONT_HERSHEY_COMPLEX
      

      text = 'Target'
      cv2.putText(cv_image, text, (x2, y2), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

      cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

      if x2< width//2 -5:
          vel_x=0.2
      elif x2 > width//2 +5:
          vel_x=-0.2
      
      if y2< height//2 -5:
        vel_y=0.2
      elif y2> height//2 +5:
        vel_y=-0.2

      else:
          vel_z=vel_z+0.2

      move_drone(vel_x,vel_y,vel_z)
      break

    cv2.imshow("Image window", cv_image)
    #cv2.imshow("masked", masked_img)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def move_drone(x,y,z):
       
        vel_msg=Twist()
        vel_msg.linear.x=x
        vel_msg.linear.y=y
        vel_msg.linear.z=z

        twist_pub.publish(vel_msg)

            

def main(args):
  
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)