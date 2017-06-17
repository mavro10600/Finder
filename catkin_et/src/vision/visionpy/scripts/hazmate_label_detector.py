#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Perfiles de colores basicos de las etiquetas
# naranja [yl, yh, Crl, Crh, Cbl, Cbh]
orang = np.array([0, 135, 0, 170, 0, 115])

# rojo [yl, yh, Crl, Crh, Cbl, Cbh]
red = np.array([0, 255, 170, 255, 0, 255])

# amarillo [yl, yh, Crl, Crh, Cbl, Cbh]
yellow = np.array([120, 255, 0, 170, 0, 115])

# verde [yl, yh, Crl, Crh, Cbl, Cbh]
green = np.array([0, 150, 0, 125, 0, 138])

# azul [yl, yh, Crl, Crh, Cbl, Cbh]
blue = np.array([0, 255, 0, 158, 158, 255])

# area de referencia
areaRef = np.array([1000, 90000])


def prevFilter(img):
  # Ventana para aplicar la el filtro de dilatcion y erosion
  wKernel = np.ones((5,5),np.uint8)
  # Aplicacion del filtros morfologicos, Dilate y posteriormente erosiona
  fImg = cv2.morphologyEx(img, cv2.MORPH_CLOSE, wKernel)
  # Filtro mediana de suavizamiento de imagen
  fImg = cv2.medianBlur(fImg, 5)
  #print('filtrado')
  #cv2.imshow('alrs',fImg)
  return fImg


def getMask(img, pfColor):
  # Define el conjunto de valores para obtener la mascara binaria
  lowLimits = (pfColor[0],pfColor[2],pfColor[4])
  highLimits = (pfColor[1],pfColor[3],pfColor[5])
  
  # Calculo de la mascara binaria, en funcion de los limites definidos
  mask = cv2.inRange(img, lowLimits, highLimits)
  return mask


def getCentroids(img, areaRef):
  # Inicializa lista para guardar los centros que se calcularan
  centers = []

  # Obtencion de segmentos, basado en la imagen binaria
  contours, hierarchy = cv2.findContours(img,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
  # Ordenamiento de mayor a menor de todos los segmentos encontrados
  # considerando como criterio su area. Solo se conserva el mayor
  # el cual se controla, por el [:1] al final de la instruccion
  contours = sorted(contours, key=cv2.contourArea, reverse = True)[:1]

  for c in contours:
    # Calculo del area de un segmento
    blobArea = cv2.contourArea(c)
    #perAreaFind = int( blobArea/(areaRef[1] - areaRef[0]) )*100

    # Se busca que el area del segmento en estudio, este dentro del intervalo
    # establecido
    if ( (blobArea > areaRef[0]) and (blobArea < areaRef[1]) ): 
      # Calculo del momento del segmento encontrado
      M = cv2.moments(c)
      #print('Area: %d'%(blobArea))

      if M["m00"] != 0:
        cX = int(M["m10"]/ M["m00"])
        cY = int(M["m01"]/ M["m00"])
        #print('Cx: %d\t Cy: %d'%(cX,cY))
        # Bandera para decir que el dato es util
        use = 1
        x,y,w,h = cv2.boundingRect(c)
        centers.append((use,cX,cY,x,y,w,h))
        #centers.append((use,cX,cY, perAreaFind))
    else:
      cX = 0
      cY = 0
      # Bandera para decir que el dato es util
      use = 0
      centers.append((use,cX,cY))
      #centers.append((use,cX,cY, perAreaFind))
  return centers

def labelDetector(imgSrc):
  # Aplicacion de filtro para suavizar bordes
  fImg = prevFilter(imgSrc)
  
  # Conversion de espacio de color BGR -> YCrCb
  yuvImg = cv2.cvtColor(fImg, cv2.COLOR_BGR2YCR_CB)
  #cv2.imshow("window_tester_2", yuvImg)

  # Obtencion de imagenes binarias en funcion del perfil de color
  # Obtencion de la mascara para el color azul
  blueMask = getMask(yuvImg, blue)
  #cv2.imshow("window_tester_2", blueMask)
  blueCenters = getCentroids(blueMask, areaRef)

  # Obtencion de la mascara para el color verde
  greenMask = getMask(yuvImg, green)
  greenCenters = getCentroids(greenMask, areaRef)
 # cv2.imshow("window_tester_1", greenMask)
  # Obtencion de la mascara para el color naranja
  orangMask = getMask(yuvImg, orang)
  orangCenters = getCentroids(orangMask, areaRef)

  # Imagenes binarias que corresponden a mas de una hazmate label
  # Obtencion de la mascara para el color rojo
  redMask = getMask(yuvImg, red)
  redCenters = getCentroids(redMask, areaRef)  
  #cv2.imshow("window_tester_1", redMask)
  # Obtencion de la mascara para el color amarillo
  yellowMask = getMask(yuvImg, yellow)
  yellowCenters = getCentroids(yellowMask, areaRef)

  if( (len(blueCenters) != 0) and (blueCenters[0][0] == 1) ):
    cv2.rectangle(imgSrc,(blueCenters[0][3], blueCenters[0][4]),(blueCenters[0][3]+blueCenters[0][5],blueCenters[0][4]+blueCenters[0][6]),(255,0,0),2)
    cv2.putText(imgSrc,"Dangerous when wet",(blueCenters[0][3]-5, blueCenters[0][4]-5),cv2.FONT_HERSHEY_SIMPLEX,0.43,(255,0,0),2)
    cv2.imshow("window_tester_1", imgSrc)

  if( (len(greenCenters) != 0) and (greenCenters[0][0] == 1) ):
    cv2.rectangle(imgSrc,(greenCenters[0][3], greenCenters[0][4]),(greenCenters[0][3]+greenCenters[0][5],greenCenters[0][4]+greenCenters[0][6]),(0,255,0),2)
    cv2.putText(imgSrc,"Non-flammable gas",(greenCenters[0][3]-5, greenCenters[0][4]-5),cv2.FONT_HERSHEY_SIMPLEX,0.43,(0,255,0),2)
    cv2.imshow("window_tester_1", imgSrc)

  if( (len(orangCenters) != 0) and (orangCenters[0][0] == 1) ):
    cv2.rectangle(imgSrc,(orangCenters[0][3], orangCenters[0][4]),(orangCenters[0][3]+orangCenters[0][5],orangCenters[0][4]+orangCenters[0][6]),(0,69,255),2)
    cv2.putText(imgSrc,"Explosive",(orangCenters[0][3]-5, orangCenters[0][4]-5),cv2.FONT_HERSHEY_SIMPLEX,0.43,(0,69,255),2)
    cv2.imshow("window_tester_1", imgSrc)

  if( (len(redCenters) != 0) and (redCenters[0][0] == 1) ):
    cv2.rectangle(imgSrc,(redCenters[0][3], redCenters[0][4]),(redCenters[0][3]+redCenters[0][5],redCenters[0][4]+redCenters[0][6]),(0,0,255),2)
    cv2.putText(imgSrc,"Fammable Liquids",(redCenters[0][3]-5, redCenters[0][4]-5),cv2.FONT_HERSHEY_SIMPLEX,0.43,(0,0,255),2)
    cv2.imshow("window_tester_1", imgSrc)

  if( (len(yellowCenters) != 0) and (yellowCenters[0][0] == 1) ):
    cv2.rectangle(imgSrc,(yellowCenters[0][3], yellowCenters[0][4]),(yellowCenters[0][3]+yellowCenters[0][5],yellowCenters[0][4]+yellowCenters[0][6]),(0,255,255),2)
    cv2.putText(imgSrc,"Oxidizer",(yellowCenters[0][3]-5, yellowCenters[0][4]-5),cv2.FONT_HERSHEY_SIMPLEX,0.43,(0,255,255),2)
    cv2.imshow("window_tester_1", imgSrc)  


def callback(data):
  global i

  bridge = CvBridge()
  imgSrc = bridge.imgmsg_to_cv2(data, "bgr8")
  #imgSrc = cv2.flip(imgSrc,1)
  cv2.imshow("window_tester_1", imgSrc)

  labelDetector(imgSrc)
  key = cv2.waitKey(1)


def main(args):
  global i

  i = 0
  rospy.init_node('hazmate_label_detector', anonymous=False)
  #image_sub2 = rospy.Subscriber("camera/image",Image,callback)
  image_sub2 = rospy.Subscriber("usb_cam1/image_raw",Image,callback)
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)