import cv2
import mediapipe as mp
from std_msgs.msg import String
import rospy
mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

# # For static images:
# hands = mp_hands.Hands(
#     static_image_mode=True,
#     max_num_hands=2,
#     min_detection_confidence=0.7)
# for idx, file in enumerate(file_list):
#   # Read an image, flip it around y-axis for correct handedness output (see
#   # above).
#   image = cv2.flip(cv2.imread(file), 1)
#   # Convert the BGR image to RGB before processing.
#   results = hands.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

#   # Print handedness and draw hand landmarks on the image.
#   print('handedness:', results.multi_handedness)
#   if not results.multi_hand_landmarks:
#     continue
#   annotated_image = image.copy()
#   for hand_landmarks in results.multi_hand_landmarks:
#     print('hand_landmarks:', hand_landmarks)
#     mp_drawing.draw_landmarks(
#         annotated_image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
#   cv2.imwrite(
#       '/tmp/annotated_image' + str(idx) + '.png', cv2.flip(image, 1))
# hands.close()

# For webcam input:



# The landmarks array has the following structur: [x0, y0, x1, y1, ....., x20, y20]
# with for example x0 and y0 the x and y values of the landmark at index 0.
test_landmarks_data = [
  0.499651,0.849638,0.614354,0.796254,
  0.686660,0.692482, 0.743792,0.606666,
  0.809362,0.512337,0.538779,0.499517,
  0.513829,0.361394,0.484049,0.260214,
  0.452508,0.173999, 0.445565,0.512067,
  0.396448,0.358399,0.355494,0.245083,0.318670,
  0.157915,0.355069,0.562040, 0.278774,
  0.435983,0.221781,0.345394,0.178977,0.273430,
  0.288238,0.631016,0.219506, 0.544787,
  0.162939,0.483343,0.110222,0.422808] # true label: 5



### Functions
def recognizeHandGesture(landmarks):
  thumbState = 'UNKNOW'
  indexFingerState = 'UNKNOW'
  middleFingerState = 'UNKNOW'
  ringFingerState = 'UNKNOW'
  littleFingerState = 'UNKNOW'
  recognizedHandGesture = None
  pseudoFixKeyPoint = landmarks.landmark[2].x
  if (landmarks.landmark[3].x < pseudoFixKeyPoint and landmarks.landmark[4].x  < landmarks.landmark[3].x ):
    thumbState = 'OPEN'    
  elif (pseudoFixKeyPoint < landmarks.landmark[3].x  and landmarks.landmark[3].x  < landmarks.landmark[4].x ):
    thumbState = 'CLOSE'    

  pseudoFixKeyPoint = landmarks.landmark[6].y
  if (landmarks.landmark[7].y < pseudoFixKeyPoint and landmarks.landmark[8].y < landmarks.landmark[7].y):
    indexFingerState = 'OPEN'    
  elif (pseudoFixKeyPoint < landmarks.landmark[7].y and landmarks.landmark[7].y < landmarks.landmark[8].y):
    indexFingerState = 'CLOSE'    

  pseudoFixKeyPoint = landmarks.landmark[10].y
  if (landmarks.landmark[11].y < pseudoFixKeyPoint and landmarks.landmark[12].y < landmarks.landmark[11].y):
    middleFingerState = 'OPEN'    
  elif (pseudoFixKeyPoint < landmarks.landmark[11].y and landmarks.landmark[11].y < landmarks.landmark[12].y):
    middleFingerState = 'CLOSE'

  pseudoFixKeyPoint = landmarks.landmark[14].y
  if (landmarks.landmark[15].y < pseudoFixKeyPoint and landmarks.landmark[16].y < landmarks.landmark[15].y):
    ringFingerState = 'OPEN'    
  elif (pseudoFixKeyPoint < landmarks.landmark[15].y and landmarks.landmark[15].y < landmarks.landmark[16].y):
    ringFingerState = 'CLOSE'
  
  pseudoFixKeyPoint = landmarks.landmark[18].y
  if (landmarks.landmark[19].y < pseudoFixKeyPoint and landmarks.landmark[20].y < landmarks.landmark[19].y):
    littleFingerState = 'OPEN'    
  elif (pseudoFixKeyPoint < landmarks.landmark[19].y and landmarks.landmark[19].y < landmarks.landmark[20].y):
    littleFingerState = 'CLOSE'
    
  if (thumbState == 'OPEN' and indexFingerState == 'OPEN' and middleFingerState == 'OPEN' and ringFingerState == 'OPEN' and littleFingerState == 'OPEN'):
    recognizedHandGesture = 'FIVE'   
  elif (thumbState == 'CLOSE' and indexFingerState == 'OPEN' and middleFingerState == 'OPEN' and ringFingerState == 'OPEN' and littleFingerState == 'OPEN'):
    recognizedHandGesture = 'FOUR'  
  elif (thumbState == 'OPEN' and indexFingerState == 'OPEN' and middleFingerState == 'OPEN' and ringFingerState == 'CLOSE' and littleFingerState == 'CLOSE'):
    recognizedHandGesture = 'THREE'   
  elif (thumbState == 'OPEN' and indexFingerState == 'OPEN' and middleFingerState == 'CLOSE' and ringFingerState == 'CLOSE' and littleFingerState == 'CLOSE'):
    recognizedHandGesture = 'TWO'   
  elif (thumbState == 'CLOSE' and indexFingerState == 'CLOSE' and middleFingerState == 'CLOSE' and ringFingerState == 'CLOSE' and littleFingerState == 'CLOSE'):
    recognizedHandGesture = 'FIST'
  elif (thumbState == 'CLOSE' and indexFingerState == 'CLOSE' and middleFingerState == 'OPEN' and ringFingerState == 'CLOSE' and littleFingerState == 'CLOSE'):
    recognizedHandGesture = 'FUCK DEVO'
  else:
    recognizedHandGesture = 'UNKNOWN'

  return recognizedHandGesture



def isSliding(landmarks,memo):

    actualPosition_x = landmarks.landmark[0].x
    actualPosition_y = landmarks.landmark[0].y
    actualPosition_z = landmarks.landmark[0].z

    if memo == None:
        memo=[landmarks.landmark[0].x,landmarks.landmark[0].y,landmarks.landmark[0].z]

    lastPosition_x,lastPosition_y,lastPosition_z = memo[0],memo[1],memo[2]

    slide = ""


    # if actualPosition_z - lastPosition_z > 0.000007:
    #     slide = "ZOOM OUT"
    # elif actualPosition_z - lastPosition_z < -0.000007:
    #     slide = "ZOOM IN"
    if actualPosition_x - lastPosition_x > 0.015:
        slide = 'RIGHT SLIDE'
    elif actualPosition_x - lastPosition_x < -0.015:
        slide = 'LEFT SLIDE'
    elif actualPosition_y - lastPosition_y > 0.015:
        slide = "DOWN SLIDE"
    elif actualPosition_y - lastPosition_y < -0.015:
        slide = "UP SLIDE"
   
    if slide == "":
        memory=[lastPosition_x,lastPosition_y,lastPosition_z]

    else :
        memory= [actualPosition_x, actualPosition_y,actualPosition_z]

    return [slide,memory] 

def getStructuredLandmarks(landmarks):
  structuredLandmarks = []
  for j in range(42):
    if( j %2 == 1):
      structuredLandmarks.append({ 'x': landmarks[j - 1], 'y': landmarks[j] })
  return structuredLandmarks

#recognizedHandGesture = recognizeHandGesture(getStructuredLandmarks(test_landmarks_data))
#print("recognized hand gesture: ", recognizedHandGesture) # print: "recognized hand gesture: 5"


def execute():
    hands = mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.5,max_num_hands=1)
    cap = cv2.VideoCapture(0)

    memo = None

    while cap.isOpened():
      success, image = cap.read()
      if not success:
        break

      # Flip the image horizontally for a later selfie-view display, and convert
      # the BGR image to RGB.
      image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
      # To improve performance, optionally mark the image as not writeable to
      # pass by reference.
      image.flags.writeable = False
      results = hands.process(image)


      # Draw the hand annotations on the image.
      image.flags.writeable = True
      image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
      font = cv2.FONT_HERSHEY_COMPLEX
  

      if results.multi_hand_landmarks:
  
        for hand_landmarks in results.multi_hand_landmarks: 
          mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

        #Get Signal
        text = recognizeHandGesture(results.multi_hand_landmarks[0])
        #handsignal = String()
        handsignal = text
        handsignal_publisher.publish(handsignal)  

        #Get Slide
        text2,memo = isSliding(results.multi_hand_landmarks[0],memo)[0],isSliding(results.multi_hand_landmarks[0],memo)[1]
        #handslide = String()
        handslide = text2
        handslide_publisher.publish(handslide) 

        print("z : "+str(results.multi_hand_landmarks[0].landmark[0].z))
        
        cv2.putText(image, text, (360,360), font, 1, (0, 0, 255), 2, cv2.LINE_4)

        cv2.putText(image, text2, (360,460), font, 1, (0, 0, 255), 2, cv2.LINE_4)    

      cv2.imshow('MediaPipe Hands', image)
      
      if cv2.waitKey(5) & 0xFF == 27:
        break

    hands.close()
    cap.release()


if __name__ == '__main__':
    try:
        #Testing our function
        rospy.init_node('hand', anonymous=True)
        handsignal_publisher = rospy.Publisher('/hand/signal', String, queue_size=10)
        handslide_publisher = rospy.Publisher('/hand/direction', String, queue_size=10)
        handforward_publisher = rospy.Publisher('/hand/forward', String, queue_size=10)

        execute()

    except rospy.ROSInterruptException: pass

