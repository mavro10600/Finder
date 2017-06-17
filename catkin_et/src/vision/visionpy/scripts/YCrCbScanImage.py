#!/usr/bin/python
import cv2
import sys
import numpy as np

# Parametros de control del programa
camera_device = 1
width = 320
height = 240
fps = 30
src = 0

#image = 'hlabel/explos.jpg'
image = 'hlabel/hazard_labels2.png'
#image = 'hlabel/testHaz.jpg'
#image = 'hlabel/flammLiquTest2.png'
#image = 'hlabel/explostest2.png'
#image = 'hlabel/flamm_liqu.jpg'
#image = 'hlabel/oxidizer.jpg'

# Funcion necesaria para los trackVars
def nothing(x):
	pass

# Iniciacion de ventana de trackVars
cv2.namedWindow("tracksBar",0)
cv2.moveWindow("tracksBar",850,0)

# Iniciacion de ventanas
cv2.namedWindow("original",0)
cv2.moveWindow("original",450,0)

cv2.namedWindow("Mask",0)
cv2.moveWindow("Mask",850,350)


# Creacion de los objetos trackVar para variar umbrales
# Se crean dos trackVars para cada canal, para fijar min y max
cv2.createTrackbar('Yh', 'tracksBar',1,255, nothing)
cv2.createTrackbar('Yl', 'tracksBar',1,255, nothing)

cv2.createTrackbar('Crh', 'tracksBar',1,255, nothing)
cv2.createTrackbar('Crl', 'tracksBar',1,255, nothing)

cv2.createTrackbar('Cbh', 'tracksBar',1,255, nothing)
cv2.createTrackbar('Cbl', 'tracksBar',1,255, nothing)

def prevFilter(img):
	# Ventana para aplicar la el filtro de dilatcion y erosion
	wKernel = np.ones((5,5),np.uint8)
	# Aplicacion del filtros morfologicos, Dilate y posteriormente erosiona
	fImg = cv2.morphologyEx(img, cv2.MORPH_CLOSE, wKernel)
	# Filtro mediana de suavizamiento de imagen
	fImg = cv2.medianBlur(fImg, 5)
	#cv2.imshow('ImgFiltering',fImg)
	return fImg


# Funcion para obtener la mascara binaria
def mask(img,yl,crl,cbl,yh,crh,cbh):
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

# Funcion para obtener el area de la mascara binaria
def getArea(img):
	# Obtencion de segmentos, basado en la imagen binaria
	contours, hierarchy = cv2.findContours(img,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
	# Ordenamiento de mayor a menor de todos los segmentos encontrados
	# considerando como criterio su area. Solo se conserva el mayor
	# el cual se controla, por el [:1] al final de la instruccion
	contours = sorted(contours, key=cv2.contourArea, reverse = True)[:1]
	
	if ( len(contours) != 0 ):
		area = cv2.contourArea(contours[0])
	else:
		area = 0

	return area

def main():

	if ( src == 0 ):
		# Iniciacion de la camara especificada con el ID
		vSrc = cv2.VideoCapture(camera_device)
		# Confiuracion del tamano de la imagen del video
		vSrc.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH,width)
		vSrc.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT,height)
		# Configuracion de los cuadros por segundo
		vSrc.set(cv2.cv.CV_CAP_PROP_FPS,fps)

		while ( vSrc.isOpened() ):
			# Lectura de cuadros de la camara
			ret,imgSrc = vSrc.read()
			
			if (ret == True):
				cv2.imshow('original',imgSrc)
				yuvImg = cv2.cvtColor(imgSrc, cv2.COLOR_BGR2YCR_CB)

				fImg = prevFilter(yuvImg)

				# Extraccion de valores de los trackVar
				yl = cv2.getTrackbarPos('Yl','tracksBar')
				crl = cv2.getTrackbarPos('Crl','tracksBar')
				cbl = cv2.getTrackbarPos('Cbl','tracksBar')

				yh = cv2.getTrackbarPos('Yh','tracksBar')
				crh = cv2.getTrackbarPos('Crh','tracksBar')
				cbh = cv2.getTrackbarPos('Cbh','tracksBar')

				# Obtencion de la mascara binaria a partir de la imagen en Yuv
				yuvMask = mask(fImg, yl, crl, cbl, yh, crh, cbh)
				cv2.imshow('Mask', yuvMask)
				AreaFind = getArea(yuvMask)

				#print('Area: %d'%(AreaFind))
							
				if cv2.waitKey(5) == 27:
	   				break

		vSrc.release()
	   	cv2.destroyAllWindows()

	else:
		imgSrc = cv2.imread(image,-1)
		while (True):
			cv2.imshow('original',imgSrc)
			yuvImg = cv2.cvtColor(imgSrc, cv2.COLOR_BGR2YCR_CB)
			fImg = prevFilter(yuvImg)
			# Extraccion de valores de los trackVar
			yl = cv2.getTrackbarPos('Yl','tracksBar')
			crl = cv2.getTrackbarPos('Crl','tracksBar')
			cbl = cv2.getTrackbarPos('Cbl','tracksBar')

			yh = cv2.getTrackbarPos('Yh','tracksBar')
			crh = cv2.getTrackbarPos('Crh','tracksBar')
			cbh = cv2.getTrackbarPos('Cbh','tracksBar')

				# Obtencion de la mascara binaria a partir de la imagen en Yuv
			yuvMask = mask(fImg, yl, crl, cbl, yh, crh, cbh)
			cv2.imshow('Mask', yuvMask)
			AreaFind = getArea(yuvMask)

			#print('Area: %d'%(AreaFind))
							
			if cv2.waitKey(5) == 27:
   				break

	   	cv2.destroyAllWindows()


if __name__ == '__main__':
	main()
