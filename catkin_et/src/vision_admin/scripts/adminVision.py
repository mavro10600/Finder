#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_msjs.msg import nodesVflags
from vision_msjs.srv import imgServTest
from vision_msjs.srv import imgServTestResponse

cont = 0

def giveResult(req):
  global fImg, srvTest

  cv2.putText(fImg,"Holoo XD",(20, 30),cv2.FONT_HERSHEY_SIMPLEX,0.43,(255,0,0),2)
  msg2 = CvBridge()
  #cv2.imshow('result',fImg)
  #cv2.waitKey(1)

  return imgServTestResponse(msg2.cv2_to_imgmsg(fImg, "bgr8"))



# Filtro previo para la busqueda de colores, aplicado a la roi
def prevColorFilter(img):
  # Ventana para aplicar la el filtro de dilatcion y erosion
  wKernel = np.ones((9,9),np.uint8)
  # Aplicacion del filtros morfologicos, Dilate y posteriormente erosiona
  fImg = cv2.morphologyEx(img, cv2.MORPH_CLOSE, wKernel)
  # Filtro mediana de suavizamiento de imagen
  fImg = cv2.medianBlur(fImg, 5)
  #print('filtrado')
  #cv2.imshow('alrs',fImg)
  return fImg

def callback(data):
  global cont, flagPub, fImg

  msg = nodesVflags()
  bridge = CvBridge()
  
  cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
  cv_image = cv2.flip(cv_image,1)
  cv2.imshow("Original_Image", cv_image)

  fImg = prevColorFilter(cv_image)
  cv2.imshow("Process_Image", fImg)
  cv2.waitKey(1)
  
  cont+=1

  if ( cont > 10 ):
    msg.qrflag = True
    msg.cflag = True
    flagPub.publish(msg)
    cont = 0
  else:
    msg.qrflag = False
    msg.cflag = False

def main(args):
  global cont, flagPub, srvTest

  rospy.init_node('admin_vision', anonymous=False)
  flagPub = rospy.Publisher('VisionFlagTest', nodesVflags, queue_size=10)
  srvTest = rospy.Service('ResVision1', imgServTest, giveResult)
  image_sub = rospy.Subscriber("camera/image",Image,callback)
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)