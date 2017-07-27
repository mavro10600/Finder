#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import rospkg
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from avision_msjs.msg import labelDetect
from avision_msjs.srv import imgLabels
from avision_msjs.srv import imgLabelsResponse

#umbrales del detector de bordes
thhc = 110
thlc = 70

#Porcentaje de diferencia permitido en la comparacion de lados que forman
#la region de interes candidata
perDifh = 38
perDifv = 36

# Area de referencia para validar deteccion de ROI
areaRoiRef = 1000

# Representacion vectorial de cada uno de los colores de interes en el espacio
# de color YCrCb
# naranja [yl, yh, Crl, Crh, Cbl, Cbh]
#orang = np.array([100, 253, 151, 203, 0, 112])
orang = np.array([0, 135, 0, 189, 0, 105])

# rojo [yl, yh, Crl, Crh, Cbl, Cbh]
#red = np.array([7, 162, 163, 255, 111, 255])
red = np.array([0, 255, 148, 255, 0, 255])

# amarillo [yl, yh, Crl, Crh, Cbl, Cbh]
yellow = np.array([120, 255, 0, 170, 0, 115])

# verde [yl, yh, Crl, Crh, Cbl, Cbh]
#green = np.array([41, 130, 0, 125, 0, 160])
green = np.array([0, 154, 0, 125, 0, 161])

# azul [yl, yh, Crl, Crh, Cbl, Cbh]
#blue = np.array([0, 255, 0, 158, 158, 255])
blue = np.array([0, 255, 0, 157, 156, 255])

# negro [yl, yh, Crl, Crh, Cbl, Cbh]
#black = np.array([0, 57, 0, 143, 0, 164])
#black = np.array([0, 72, 0, 255, 113, 128])
#black = np.array([0, 72, 117, 138, 118, 140])
black = np.array([0, 72, 117, 138, 118, 145])

# Listas de etiquetas patrones, acorde a su color
#Etiquetas unicas
pkgPath = rospkg.RosPack()
patPath = pkgPath.get_path('labels_hazmate_detector')+'/scripts/'

blueLabel = [patPath+'patternsOnly/pat1.jpg']
orangLabel = [patPath+'patternsOnly/pat2.jpg']
greenLabel = [patPath+'patternsOnly/pat3.jpg']

redLabels = [patPath+'patternsRed/pat1.jpg', patPath+'patternsRed/pat2.jpg',patPath+'patternsRed/pat3.jpg',patPath+'patternsRed/pat4.jpg']
yellowLabels = [patPath+'patternsYel/pat1.jpg', patPath+'patternsYel/pat2.jpg', patPath+'patternsYel/pat3.jpg']
blackLabels = [patPath+'patternsBl/pat1.jpg', patPath+'patternsBl/pat2.jpg', patPath+'patternsBl/pat3.jpg']

# Listas de nombres de las etiquetas patron a detectar
#Etiquetas unicas
bLabelNames = ['Dangerous_when_wet']
gLabelNames = ['Non-flammable_gas']
oLabelNames = ['Explosive']

rLabelNames = ['Flammable_gas','Spontaneously_combustible','Flammable_solid','Organic_peroxid']
yLabelNames = ['Oxidizer', 'Radioactive', 'Organic_peroxid']
blLabelNames = ['Corrosive', 'Inhalation_hazard', 'Infectious_substance']

# Parametros del extractor de caracteristicas SURF
hessian_threshold = 800

# Lista que contiene el nombre de las etiquetas a publicar en el servicio
sendNames = []

global imgPlot

#Configuracion de ventanas para desplegar resultados
#cv2.namedWindow('Video search',1)
#cv2.moveWindow('Video search',0,0)

#cv2.namedWindow('Roi',1)
#cv2.moveWindow('Roi',550,0)

#cv2.namedWindow('Contours detected',0)
#cv2.moveWindow('Contours detected',0,450)

#cv2.namedWindow('Label Founded',1)
#cv2.moveWindow('Label Founded',550,450)


# Funcion para calcular distancia euclidiana entre dos puntos
def rectLong(a, b):
  # Calculo de distancia euclidiana entre dos puntos
  dist = np.linalg.norm(a-b)
  return dist



# Comprobacion de similitud con rectangulos o cuadrados de cualquier
# poligono que este formado por 4 contornos. La funcion recibe como datos
# de entrada los cuatro puntos de aproximacion del poligono
def isaRect(points):
  # Rectas verticales del contorno
  dp1p0 = abs( rectLong(points[1],points[0]) )
  dp2p3 = abs( rectLong(points[2],points[3]) )
  # Rectas horizontales del contorno
  dp2p1 = abs( rectLong(points[2],points[1]) )
  dp3p0 = abs( rectLong(points[3],points[0]) )
  # Calculo de diferencias entre el tamano de los segmentos de recta
  difh = abs (dp2p1 - dp3p0)
  difv = abs (dp2p3 - dp1p0)

  if( difh <= perDifh and difv <= perDifv):
    square = True
  else:
    square = False

  return square



# Busqueda de la region de interes, para aplicar metodos de reconocimiento.
# La funcion consiste en un buscador de contornos similares a rectangulos y
# cuadrados, recibiendo como entrada la matriz que define los contornos "img"
# y la imagen original para indicar la deteccion. Regresa una lista cuyos datos
# son [utilidad del conjunto, centro del poligono, coordenadas del boundingBox
# x,y,w,h, area del poligono detectado]
def findRoi(img, imgt):
  # Inicializa lista para guardar los centros que se calcularan
  centers = []
  # Inicializacion de variables
  cX = 0
  cY = 0
  totR = 0
  squaref = False

  # Obtencion de segmentos, basado en la imagen binaria
  #contour, hierarchy = cv2.findContours(img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
  contour, hierarchy = cv2.findContours(img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
  # Ordenamiento de mayor a menor de todos los segmentos encontrados
  # considerando como criterio su area.
  contours = sorted(contour, key=cv2.contourArea, reverse = True)[:1]

  for c in contours:
    # Calculo del perimetro
    per = cv2.arcLength(c, True)
    # Calculo de los segmentos que forman el perimetro
    approx = cv2.approxPolyDP(c, 0.06*per, True)
    # Calculo del area comprendida por el contorno
    cntArea = cv2.contourArea(c)

    # Analisis de proporcion de los lados que forman el contorno
    if(len (approx) == 4):
      squaref = isaRect(approx)

    if( squaref and (cntArea >= areaRoiRef) ):
      M = cv2.moments(c)
      cX = int(M["m10"]/ M["m00"])
      cY = int(M["m01"]/ M["m00"])
      use = 1
      x,y,w,h = cv2.boundingRect(c)
      #cv2.drawContours(imgt, [approx], -1, (0,255,0), 4)
      centers.append((use,cX,cY,x,y,w,h,cntArea))
      totR += 1
  
  #print ('No. of rects: %d'%(totR))
  return centers



# Filtro previo para la busqueda de colores, aplicado a la roi. La funcion aplica
# un filtro de dilatacion y erosion sobre la roi y despues hace un suavizado por
# algoritmo de mediana. Regresa la imagen procesada por todo el filtro
def prevColorFilter(img):
  # Ventana para aplicar la el filtro de dilatcion y erosion
  wKernel = np.ones((3,3),np.uint8)
  # Aplicacion del filtros morfologicos, Dilate y posteriormente erosiona
  fImg = cv2.morphologyEx(img, cv2.MORPH_CLOSE, wKernel)
  # Filtro mediana de suavizamiento de imagen
  fImg = cv2.medianBlur(fImg, 5)
  #print('filtrado')
  #cv2.imshow('alrs',fImg)
  return fImg


# Funcion filtro de color, utiliza umbrales caracteristicos de cada color
# que son de interes, mapeados en el espacio de colores YCrCb para obtener las
# regiones que son de dicho color dentro de la roi. La funcion tiene como entradas
# la roi = img y el vector que contiene los umbrales que definen el color de interes
# regresa una imagen binaria
def getMask(img, pfColor):
  # Define el conjunto de valores para obtener la mascara binaria
  lowLimits = (pfColor[0],pfColor[2],pfColor[4])
  highLimits = (pfColor[1],pfColor[3],pfColor[5])
  
  # Calculo de la mascara binaria, en funcion de los limites definidos
  mask = cv2.inRange(img, lowLimits, highLimits)
  return mask



# Calculo de centrodes de las regiones de imagen obtenidas por el filtro
# de color. La funcion recibe la imagen binaria "img", el area de la roi, para tener una
# referencia del tamano de las segmentaciones y poder validarlas, y la cantidad de rois
# que se desean obtener, lo cual es util para los perfiles de color que pueden definir
# mas de una etiqueta. Como dato de salida, se tiene una lista cuyos datos
# son [validacion del contorno, centro del poligono, coordenadas del boundingBox
# x,y,w,h]
def getColorCenters(img, roiArea, nRoi):
  # Inicializa lista para guardar los centros que se calcularan
  centers = []

  # Obtencion de segmentos, basado en la imagen binaria
  contours, hierarchy = cv2.findContours(img,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
  # Ordenamiento de mayor a menor de todos los segmentos encontrados
  # considerando como criterio su area. Solo se conserva el mayor
  # el cual se controla, por el [:1] al final de la instruccion
  contours = sorted(contours, key=cv2.contourArea, reverse = True)[:nRoi]

  for c in contours:
    # Calculo del area de un segmento
    blobArea = cv2.contourArea(c)
    
    if ( blobArea >= 0.1*roiArea):
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
        x = 0
        y = 0
        w = 0
        h = 0
        # Bandera para decir que el dato es util
        use = 0
        centers.append((use,cX,cY,x,y,w,h))
        #centers.append((use,cX,cY, perAreaFind))
  return centers



# Funcion para graficar los resultados de la deteccion de colores
def plotSubRois(centers, imgPlot, color):
  globals()['imgPlot'] = imgPlot
  aux = len(centers)
  nLabels = 0

  if( (aux == 1) and (centers[0][0] == 1) ):
    cv2.rectangle(imgPlot,(centers[0][3], centers[0][4]),(centers[0][3]+centers[0][5],centers[0][4]+centers[0][6]),color,2)
    #cv2.putText(imgSrc,"Dangerous when wet",(blueCenters[0][3]-5, blueCenters[0][4]-5),cv2.FONT_HERSHEY_SIMPLEX,0.43,(255,0,0),2)
    #cv2.imshow("Label Founded", imgPlot)
    nLabels = 1

  elif( (aux == 2) and (centers[1][0] == 1) ):
    cv2.rectangle(imgPlot,(centers[0][3], centers[0][4]),(centers[0][3]+centers[0][5],centers[0][4]+centers[0][6]),color,2)
    cv2.rectangle(imgPlot,(centers[1][3], centers[1][4]),(centers[1][3]+centers[1][5],centers[1][4]+centers[1][6]),color,1)
    #cv2.imshow("Label Founded", imgPlot)
    nLabels = 2

  elif( (aux == 3) and (centers[2][0] == 1) ):
    cv2.rectangle(imgPlot,(centers[0][3], centers[0][4]),(centers[0][3]+centers[0][5],centers[0][4]+centers[0][6]),color,3)
    cv2.rectangle(imgPlot,(centers[1][3], centers[1][4]),(centers[1][3]+centers[1][5],centers[1][4]+centers[1][6]),color,2)
    cv2.rectangle(imgPlot,(centers[2][3], centers[2][4]),(centers[2][3]+centers[2][5],centers[2][4]+centers[2][6]),color,1)
    #cv2.imshow("Label Founded", imgPlot)
    nLabels = 3

  return nLabels



# Comparacion de caracteristicas pictograficas, utilizando el algoritmo SURF
def surfAlgorit(imgpat, imgsear):
  # Conversion de las imagenes a escala de grises
  patgrey = cv2.cvtColor(imgpat, cv2.COLOR_BGR2GRAY)
  seagrey = cv2.cvtColor(imgsear, cv2.COLOR_BGR2GRAY)
  
  # Iniciacion del metodo, bajo el limite del determinante de la matriz de Harris
  detector = cv2.SURF(hessian_threshold)
  
  # Extraciion caracteristicas de la imagen candidato y la patron
  skeypoints,sdescriptors = detector.detectAndCompute(seagrey,None)
  pkeypoints,pdescriptors = detector.detectAndCompute(patgrey,None)

  # calculo del numero de filas que ocupan los vectores de caracteristicas de la
  # imagen en analisis
  if( sdescriptors != 'None' ):
    if( len(skeypoints) ):
      rowsize = len(sdescriptors) / len(skeypoints)
      cont = True
    else:
      cont = False
  else:
    cont = False

  if( cont ):
    # Re-dimensionamiento de los vectores de caracteristicas
    if rowsize > 1:
      srows = np.array(sdescriptors, dtype = np.float32).reshape((-1, rowsize))
      prows = np.array(pdescriptors, dtype = np.float32).reshape((-1, rowsize))
      #print hrows.shape, nrows.shape
    else:
      srows = np.array(sdescriptors, dtype = np.float32)
      prows = np.array(pdescriptors, dtype = np.float32)
      rowsize = len(srows[0])

    samples = srows

    # Iniciacion del vector para guardar als respuestas del matching
    responses = np.arange(len(skeypoints), dtype = np.float32)
    # Iniciacion del metodo de kNearest para realizar la comparacion de imagenes
    knn = cv2.KNearest()
    # Entrenemaiento del metodo
    knn.train(samples,responses)
    # Inicio del contador de puntos conicidentes
    nPoints = 0

    # Ciclo de comparacion de los puntos patron y de la imagen en analisis
    for i, descriptor in enumerate(prows):
      descriptor = np.array(descriptor, dtype = np.float32).reshape((1, rowsize))
      # Calculo de las distancias entre los puntos clave de la iamgen patron y los de la
      # imagen en analisis
      retval, results, neigh_resp, dists = knn.find_nearest(descriptor, 1)
      dist =  dists[0][0]

      # Calificacion de aceptacion de los puntos de coincidencia encontrados
      if dist < 0.1:
        #color = (0, 255, 0)
        nPoints += 1
      #else:
        #color = (0, 0, 255)

    if (nPoints >= 3):
      validID = 1
    else:
      validID = 0
    # Regreso de la cantidad de puntos encontrados y bandera de reconocimiento
    #return (validID, nPoints)
    return validID
  else:
    return 0



# Funcion que aplica el algoritmo SURF para la identificacion pictografica de las etiquetas
# de interes a reconocer
def surfLabelID(centers, roi, pattern, labelNames):
  # Lista que guarda las banderas de coincidencia con cada uno de los patrones con los que se busca
  # la conicidencia de la subroi
  patMatch = []
  # Lista que guarda los patrones detectados y sus coordenadas
  detectRes = []
  #nRegs = len(centers)
  #nPatterns = len(pats)

  # Ciclo para buscar la coincidencia de cada subroi candidata a ser etiqueta. 
  for cent in centers:
    if ( cent[0] ):
      # Segmentacion del area donde se buscara el patron
      subRoi = roi[ ( cent[4] ):( cent[4]+cent[6] ), ( cent[3] ):( cent[3]+cent[5] ) ]

      # Ciclo que busca el match de la subroi con los patrones disponibles
      for pat in pattern:
        imgp = cv2.imread(pat, -1)
        aux = surfAlgorit(imgp, subRoi)
        patMatch.append(aux)

      try:
        nPat = patMatch.index(1)
        flag = True
      except ValueError:
        flag = False

      if ( flag ):
        if( nPat <= (len(labelNames)-1) ):
          detectRes.append( (1, cent[1], cent[2], cent[3], cent[4], cent[5], cent[6], labelNames[nPat]) )
        #else:
        #  detectRes.append( (1, cent[1], cent[2], cent[3], cent[4], cent[5], cent[6], labelNames[0]) )
        #cv2.rectangle(roi,(cent[3], cent[4]),(cent[3]+cent[5],cent[4]+cent[6]),(255,0,0),2)
        #cv2.imshow('SURFResult',roi)
      else:
        detectRes.append( (0, cent[1], cent[2], cent[3], cent[4], cent[5], cent[6], 'nothing') )

  return detectRes

# Funcion que atiende el servicio para proporcionar la imagen de las etiquetas que se encontraron
# y mensages con el nombre de dichas etiquetas
def sendResult(req):
  global sendNames, imgPlot

  msgim = CvBridge()
  aux = msgim.cv2_to_imgmsg(imgPlot, "bgr8")

  aux2 = len(sendNames)
  #print(sendNames)

  if( aux2 == 1 ):
    #print(sendNames[0][0][7])
    label1 = sendNames[0][0][7]
    label2 = 'nothing'
    label3 = 'nothing'
    label4 = 'nothing'
  elif( aux2 == 2 ):
    #print(sendNames[1][0][7])
    label1 = sendNames[0][0][7]
    label2 = sendNames[1][0][7]
    label3 = 'nothing'
    label4 = 'nothing'
  elif( aux2 == 3 ):
    #print(sendNames[2][0][7])
    label1 = sendNames[0][0][7]
    label2 = sendNames[1][0][7]
    label3 = sendNames[2][0][7]
    label4 = 'nothing'

  elif( aux2 == 4 ):
    #print(sendNames[3][0][7])
    label1 = sendNames[0][0][7]
    label2 = sendNames[1][0][7]
    label3 = sendNames[2][0][7]
    label4 = sendNames[3][0][7]
  else:
    label1 = 'nothing'
    label2 = 'nothing'
    label3 = 'nothing'
    label4 = 'nothing'
  cv2.imwrite("/home/finder-remoto/Finder/catkin_et/src/roswww/www/img/labelDetected"+label1+label2+label3+label4+".jpg",imgPlot)

  return imgLabelsResponse(aux,label1,label2,label3,label4)


# Funcion para la deteccion especifica de cada hazmate label
def labelDetector(imgSrc, roiArea):
  global flagLab, sendNames, imgPlot
  sendNames = []
  # Mensaje para avisar que ha encontrado etiquetas Hazmate
  sendFlag = labelDetect()

  # Conversion de espacio de color BGR -> YCrCb
  yuvImg = cv2.cvtColor(imgSrc, cv2.COLOR_BGR2YCR_CB)
  #cv2.imshow("window_tester_2", yuvImg)

  # Aplicacion de filtro para suavizar bordes
  yuvImg = prevColorFilter(yuvImg)

  # Obtencion de imagenes binarias en funcion del perfil de color
  # Obtencion de la mascara para el color azul
  blueMask = getMask(yuvImg.copy(), blue)
  #cv2.imshow("window_tester_2", blueMask)
  blueCenters = getColorCenters(blueMask, roiArea, 1)

  # Obtencion de la mascara para el color verde
  greenMask = getMask(yuvImg.copy(), green)
  greenCenters = getColorCenters(greenMask, roiArea, 1)
 # cv2.imshow("Label Founded", greenMask)
  # Obtencion de la mascara para el color naranja
  orangMask = getMask(yuvImg.copy(), orang)
  orangCenters = getColorCenters(orangMask, roiArea, 1)

  # Imagenes binarias que corresponden a mas de una hazmate label
  # Obtencion de la mascara para el color rojo
  redMask = getMask(yuvImg.copy(), red)
  #cv2.imshow("Label mask", redMask)
  redCenters = getColorCenters(redMask, roiArea, 3)  
  #cv2.imshow("Label mask", redMask)

  # Obtencion de la mascara para el color amarillo
  yellowMask = getMask(yuvImg.copy(), yellow)
  yellowCenters = getColorCenters(yellowMask, roiArea, 3)
  
  # Obtencion de la mascara para el color negro
  blackMask = getMask(yuvImg.copy(), black)
  blackCenters = getColorCenters(blackMask, roiArea, 3)

  ## Busqueda de etiquetas mediante comportamientos pictograficos aplicando el algoritmo SURF

  # Busqueda de etiqueta patron azul
  blueLabCenter = surfLabelID(blueCenters, imgSrc, blueLabel, bLabelNames)
  #if ( blueLabCenter[0][0] and len(blueLabCenter[0]) == 8 ):
  #  print(blueLabCenter[0][7])
  #print(len(blueLabCenter))
  blueLab = plotSubRois(blueLabCenter, imgSrc, [255,0,0])

  # Busqueda de etiqueta patron verde
  greenLabCenter = surfLabelID(greenCenters, imgSrc, greenLabel, gLabelNames)
  greenLab = plotSubRois(greenLabCenter, imgSrc, [0,255,0])

  # Busqueda de etiqueta patron naranja
  orangLabCenter = surfLabelID(orangCenters, imgSrc, orangLabel, oLabelNames)
  orangLab = plotSubRois(orangLabCenter, imgSrc, [0,69,255])

  # Segmentacion de color rojo
  redLabCenters = surfLabelID(redCenters, imgSrc, redLabels, rLabelNames)
  redLab = plotSubRois(redLabCenters, imgSrc, [0,0,255])

  # Segmentacion de color amarillo
  yelLabCenters = surfLabelID(yellowCenters, imgSrc, yellowLabels, yLabelNames)
  yelLab = plotSubRois(yelLabCenters, imgSrc, [0,255,255])

  #Segmentacion de color negro
  blLabCenters = surfLabelID(blackCenters, imgSrc, blackLabels, blLabelNames)
  blackLab = plotSubRois(blLabCenters, imgSrc, [255,255,255])

  # Vector que contiene la cantidad de etiquetas por color encontradas
  labMsgs = [blueLabCenter, greenLabCenter, orangLabCenter, redLabCenters, yelLabCenters, blLabCenters]
  # Vector que contiene los centroides de las etiquetas detectadas
  labNum = np.array([blueLab, greenLab, orangLab, redLab, yelLab, blackLab])

  # Encuentra las etiquetas que fueron encontradas
  labFinded = labNum.nonzero()

  labIndex = len(labFinded[0])
  prevMsgs = labFinded[0]

  if( labIndex == 1 ):
    sendNames.append( labMsgs[prevMsgs[0]] )
    sendFlag.labelflag = True
    flagLab.publish(sendFlag)
  
  elif( labIndex == 2 ):
    sendNames.append( labMsgs[prevMsgs[0]] )
    sendNames.append( labMsgs[prevMsgs[1]] )
    sendFlag.labelflag = True
    flagLab.publish(sendFlag)
  
  elif( labIndex == 3 ):
    sendNames.append( labMsgs[prevMsgs[0]] )
    sendNames.append( labMsgs[prevMsgs[1]] )
    sendNames.append( labMsgs[prevMsgs[2]] )
    sendFlag.labelflag = True
    flagLab.publish(sendFlag)
  
  elif( labIndex == 4 ):
    sendNames.append( labMsgs[prevMsgs[0]] )
    sendNames.append( labMsgs[prevMsgs[1]] )
    sendNames.append( labMsgs[prevMsgs[2]] )
    sendNames.append( labMsgs[prevMsgs[3]] )
    sendFlag.labelflag = True
    flagLab.publish(sendFlag)
  
  else:
    sendFlag.labelflag = False
  
  #print( 'ScoreLabels; Blue %d, \t Green %d, \t Orang %d, \t Red %d, \t Yellow %d, \t Black %d \t'%(blueLab, greenLab, orangLab, redLab, yelLab, blackLab) )


def callback(data):

  bridge = CvBridge()
  imgSrc = bridge.imgmsg_to_cv2(data, "bgr8")
  # Conversion a gris de la imagen
  imgwg = cv2.cvtColor(imgSrc, cv2.COLOR_BGR2GRAY)
  # Ecualizacion de la imagen en escala de grises
  clahe = cv2.createCLAHE(clipLimit=4.0, tileGridSize=(8,8))
  imgwg = clahe.apply(imgwg)
  # Suaviazado con ventana tipo Gaussiana de bordes en la imagen
  #imgwg = cv2.GaussianBlur(imgwg, (5,5), 0)
  imgwg = cv2.bilateralFilter(imgwg, 5, 150, 150)
  #cv2.imshow('GrayImageWork', imgwg)
  # Calculo de contornos por el algoritmo de Canny
  edges = cv2.Canny(imgwg, thlc, thhc)
  edges = cv2.GaussianBlur(edges, (5,5), 0)
  #cv2.imshow('Contours detected', edges)
  
  # Busqueda de la region de interes a analizar. La lista almacena los datos
  # de la bandera que aprueba el uso, el centroide de la roi detectada y los puntos
  # cartesianos que definen el boundingBox de la roi
  cent = findRoi(edges.copy(), imgSrc)

  if( len(cent) ):
    #cv2.rectangle(imgSrc,(cent[0][3], cent[0][4]),(cent[0][3]+cent[0][5],cent[0][4]+cent[0][6]),(255,0,0),2)
    #cv2.circle(imgSrc, (cent[0][1],cent[0][2]), 5, (0,0,255), -1)
    
    # Segmentacion de roi
    # imgSrc[Ya:Yb,Xa:Xb]
    roi = imgSrc[ ( cent[0][4] ):( cent[0][4]+cent[0][6] ), ( cent[0][3] ):( cent[0][3]+cent[0][5] ) ]
    # Metodo de identificacion de etiquetas
    labelDetector( roi.copy(), cent[0][7] )

    #cv2.imshow('Roi', roi)

  #cv2.imshow('Video search', imgSrc)
  cv2.waitKey(1)


def main(args):
  global flagLab, srvLabel, imgPlot

  rospy.init_node('hazmateLabelDetector', anonymous=False)

  # Iniiacion del publicador de a bandera de deteccion
  flagLab = rospy.Publisher('flag_labels_hazmate', labelDetect, queue_size=2)
  # Iniciacion del servicio que enviara la imagen resultado
  srvLabel = rospy.Service('img_labels_result', imgLabels, sendResult)
  # #Iniciacion del subscriptor al bus de la camara
  image_cam = rospy.Subscriber("usb_cam1/image_raw",Image,callback)
  #image_sub2 = rospy.Subscriber("usb_cam1/image_raw",Image,callback)
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
