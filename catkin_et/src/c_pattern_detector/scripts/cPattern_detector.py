#!/usr/bin/env python

'''
Este programa se puede dividir en cuatro partes fundamentales 
A. Filtrado por color, se detecta negro
B. Segmentacion
C. Deteccion de contornos para detectar circulos
D. Deteccion de apertura
'''

import roslib
import cv2.cv as cv
import cv2
import numpy as np
import math
import sys
import rospy

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from vision_msjs.msg import cPatternDetect
from vision_msjs.srv import imgCpattern
from vision_msjs.srv import imgCpatternResponse


# Perfil de color negro mapeado en el espacio de colores YCrCb
# Dichos valores cambian ligeramente acorde a las condiciones de trabajo
yl =66
crl =37
cbl =53

yh =255
crh =255
cbh = 250


# Funcion que aplica un suavizado de bordes a la mascara obtenida
# por el perfil de color de interes. Dicha mascara se dilata y erosiona previo a suavizar
# bordes
def prevFilter(img):
	# Ventana para aplicar la el filtro de dilatcion y erosion
	wKernel = np.ones((5,5),np.uint8)
	# Aplicacion del filtros morfologicos, Dilate y posteriormente erosiona
	fImg = cv2.morphologyEx(img, cv2.MORPH_CLOSE, wKernel)
	# Filtro mediana de suavizamiento de imagen
	fImg = cv2.medianBlur(fImg, 5)
	#cv2.imshow('ImgFiltering',fImg)
	return fImg


#  Funcion para obtener la mascara binaria, a partir del color de interes especificado
def maskf( img ):
	# Define el conjunto de valores para obtener la mascara binaria
	lowLimits = (yl,crl,cbl)
	highLimits = (yh,crh,cbh)
	# Calculo de la mascara binaria, en funcion de los limites definidos
	mask = cv2.inRange(img, lowLimits, highLimits)
	# Ajuste de mascara
	# Se define la ventana a utilizar en el filtro morfologico
	wKernel = np.ones((3,3),np.uint8)
	# Filtro de dilatacion y erosion a la mascara
	mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, wKernel)

	return mask


# Funcion que calcula las coordenadas del punto medio entre dos puntos
#def pmedio(x1,x2,y1,y2):
def pmedio(cod_point1, cod_point2):
  if ( cod_point1[0] < cod_point2[0] ):
    xc = cod_point1[0] + ( cod_point2[0] - cod_point1[0] )*0.5
  else:
    xc = cod_point2[0] + ( cod_point1[0] - cod_point2[0] )*0.5
  
  if ( cod_point1[1] < cod_point2[1] ):
    yc = cod_point1[1] + ( cod_point2[1] - cod_point1[1] )*0.5
  else:
    yc = cod_point2[1] + ( cod_point1[1] - cod_point2[1] )*0.5
  
  return xc,yc


# Funcion para el calculo de contornos de una imagen con un solo canal. El proposito
# de esta imagen es encontrar el contorno del patron C
def canny_detector(image, sigma=0.33):
  # Calcula la intensidad media de los pixeles de la imagen de entrada
  v = np.median(image)
  # Calculo de los umbrales para el detector canny, en funcion de la media y un delta
  lower = int(max(0, (1.0 - sigma) * v))
  upper = int(min(255, (1.0 + sigma) * v))
  # Obtencion de los contornos
  edged = cv2.Canny(image, lower, upper)
  
  return edged


##  Funcion que determina la abertura, los parametros son nombre de la ventana que se abrira, la imagen a la cual
##  se le aplicara el algoritmo, factor de calidad, minima distancia permitida
def recAber(colororig, original, imagen, mind, (x0,y0)):
  # Numero de esquinas deseadas a obtener
  Nesq = 3
  #print '----'
  # parametro del operador canny
  cal = 32
  cal=cal/255.0
  #mind = 0.1
        
  # numero de esquinas por buscar
  #cv2.bitwise_not ( imagen,imagen )
  #cv2.imshow('imagen',imagen)

  # Detecta las esquinas en imagen
  corners = cv2.goodFeaturesToTrack(imagen, Nesq, cal, mind)
  # Conversion del vector de coordenadas flotantes a enteros de 8 bits
  corners = np.int0(corners)
  #print corners
  
  # Inicializacion de variables
  x,y=0,0
  coor=[]

  # coloca las coordenadas de las esquinas en un solo arreglo
  for i in corners:
    # Separa los datos de cada elemento en el vector de corners
    x,y = i.ravel()
    # Junta las coordenadas de cada esquina en una lista
    coor.append((x,y))

    ## Esta parte se encarga de ver cual segmento de recta entre los tres puntos tiene la maxima
    ## distancia y almacena el valor del punto medio de la recta mayor en xc yc
    if  1<len(coor)<4:
      # Calculo de la distancia entre las esquinas detectadas de la apertura del patron
      d1 = math.sqrt( (coor[0][0] -coor[1][0] )**2 + (coor[0][1] -coor[1][1] )**2 )    #1,2
      
      # Calculo del punto medio de la recta que une a las dos esquinas detectadas
      xc,yc = pmedio( coor[0], coor[1] )

      ##  Dibuja en la imagen las esquinas localizadas, y el punto donde
      ##  se encuentre la abertura
      #imagenC = cv2.cvtColor(imagen, cv2.COLOR_GRAY2BGR)
      #cv2.circle(imagenC,(coor[0][0],coor[0][1]),2,(0,0,255),-1)
      #cv2.circle(imagenC,(coor[1][0],coor[1][1]),2,(0,255,0),-1)
      #cv2.circle(imagenC,(coor[2][0],coor[2][1]),2,(255,0,0),-1)

      #cv2.circle(imagenC,(int(xc),int(yc)),5,(100,100,100),-1)
      cv2.circle(colororig,(int(xc), int(yc)), 5, (0,0,255), -1)
      #cv2.circle(colororig,(int(x0),int(y0)),5,(255,0,0),-1)

      x1 = float(xc)
      y1 = float(yc)
      x0 = float(x0)
      y0 = float(y0)
      sx = abs(x0-x1)
      sy = abs(y0-y1)
      
      # dependiendo del cuadrante donde se encuentre la apertura, se suma o resta angulo
      if x1<x0:
        argum=sy/sx
        angulo=math.atan(argum)
        texto=str(angulo)

        if y1<y0:
          angulo=math.pi-angulo
          texto=str(angulo)
        elif y1>y0:
          angulo=math.pi+angulo
          texto=str(angulo)
        elif y1==y0:
          angulo=math.pi
          texto=str(angulo)

      elif x1>x0:
        argum=sy/sx
        angulo=math.atan(argum)
        texto=str(angulo)

        if y1>y0:
          angulo=2*math.pi-angulo
          texto=str(angulo)

      elif x1==x0:
        if y1<y0:
          angulo=(math.pi)/2
          texto=str(angulo)

        if y1>y0:
          angulo=(math.pi/2)
          texto=str(angulo)

        else:
          texto='c'
      
      font = cv2.FONT_HERSHEY_SIMPLEX
      #coloca el texto en la imagen original
      cv2.putText(colororig,texto, (int(xc)+3,int(yc)+3), font, 1, (0,255,0))

  return (angulo, colororig)



# Funcion que envia los resultados por medio de la invocacion del servicio
# especificado en el main
def sendResult(req):
  global ang, imgRes
  
  msgim = CvBridge()

  aux = msgim.cv2_to_imgmsg(imgRes, "bgr8")

  cv2.imwrite("/home/finder-remoto/Finder/catkin_et/src/roswww/www/img/cPatternDetected"+str(ang)+".jpg",imgRes)

  return imgCpatternResponse(aux, str(ang))



def callback(data):
  global flagC, ang, imgRes

  bridge = CvBridge()
  sendFlag = cPatternDetect()

  try:
    imgSrc = bridge.imgmsg_to_cv2(data, "bgr8")
  except CvBridgeError as e:
    print(e)

  '''
  inicia
  A. Filtrado por color, se detecta negro
  '''
  # Conversion de espacio de colores RGB a YCrCb
  yuvImg = cv2.cvtColor(imgSrc.copy(), cv2.COLOR_BGR2YCR_CB)
  
  # Ecualizacion de la imagen en el espacio de color YCrCb
  clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
  yuvImg[:,:,0] = clahe.apply( yuvImg[:,:,0] )
  yuvImg[:,:,1] = clahe.apply( yuvImg[:,:,1] )
  yuvImg[:,:,2] = clahe.apply( yuvImg[:,:,2] )

  # filtrado inicial, dilata y erosiona
  fImg = prevFilter(yuvImg)
  # aplica mascara para obtener los objetos color negro
  yuvMask = maskf( fImg )
  #cv2.imshow('Mascara YUV', yuvMask)
  '''
  final A
  '''

  '''
  inicia
  B. Segmentacion
  '''

  # Deteccion de circulos con base en la macara binaria obtenida del color negro
  circlei = cv2.HoughCircles(yuvMask,cv.CV_HOUGH_GRADIENT,1,20,param1=100,param2=30,minRadius=0,maxRadius=0)
  imguno = yuvMask

  # cuando detecta circulos en la mascara YUV
  if circlei!=None:
    circlei=sorted(circlei[0],key=lambda x:x[2],reverse=True)   # ordena los circulos encontrados de mayor a menor radio 
        
    mask0 = np.ones(yuvMask.shape,np.uint8)                    # crea una mascara
    #mask0 = cv2.bitwise_not( mask0 )                               # niega a la mascara
    
    radio = int(1.1*circlei[0][2])                                # obtiene el radio del circulo mayor
    cv2.circle(mask0,(circlei[0][0],circlei[0][1]),radio,(0,0,0),-1)  # dibuja el circulo mayor en la mascara
    fg1 = cv2.bitwise_or( yuvMask, mask0 )                           # segmenta la imagen YUV con la mascara
    #fg2=cv2.bitwise_or(equ,mask0)                               # segmenta la imagen ecualizada con la mascara
    #cv2.imshow('segmenta equ',fg2)                              
    #cv2.imshow('segmenta yuvMask',fg1)                          
    '''
    finaliza B
    '''

    '''
    inicia
    C. Deteccion de contornos para detectar circulos
    '''

    # busca contornos en la imagen YUV segmentada
    contours, hier = cv2.findContours(fg1,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)     
        
    ## cuando los contornos encontrados son menores de 9 entonces empieza el proceso
    if len(contours)<9:
      for h,cnt in enumerate(contours):                       # para cada contorno encontrado se le aplica el cdigo
        mask = np.zeros(imguno.shape,np.uint8)
        mask = cv2.bitwise_not(mask )
        cv2.drawContours(mask,[cnt],0,0,-1)                   # dibuja en mask el contorno actual
        circlei = cv2.HoughCircles(mask,cv.CV_HOUGH_GRADIENT,1,20,param1=50,param2=30,minRadius=0,maxRadius=0)

        # si el contorno detectado tiene la forma circular, entonces busca la abertura
        if circlei!=None:
          circles=sorted(circlei[0],key=lambda x:x[2],reverse=True)
          rmax=500
          for i in circles:
            if i[2]<rmax:
              rmax=i[2]
              mind= rmax/10
              '''
              inicia
              D. Deteccion de apertura

              '''
              # Deteccion de la apertura y calculo del angulo en donde se encuentra
              (ang, imgRes) = recAber(imgSrc.copy() ,fg1, mask, mind, (i[0],i[1]))
              cv2.imshow('Gap_C_Detection', imgRes)
              
              sendFlag.cPatternFlag = True
              flagC.publish(sendFlag)

  cv2.imshow('Video_search',imgSrc)
  cv2.waitKey(1)


def main(args):
  global flagC

  # Iniciacion del nodo
  rospy.init_node('c_pattern_recognition', anonymous=False)
  
  # Iniiacion del publicador de a bandera de deteccion
  flagC = rospy.Publisher('flag_c_pattern', cPatternDetect, queue_size=2)
  
  # Iniciacion del servicio que enviara la imagen resultado
  srvLabel = rospy.Service('img_cpattern_result', imgCpattern, sendResult)
  
  # Iniciacion del nodo suscriptor al bus que transporta los frames de la camara
  sus = rospy.Subscriber("usb_cam1/image_raw", Image, callback)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
