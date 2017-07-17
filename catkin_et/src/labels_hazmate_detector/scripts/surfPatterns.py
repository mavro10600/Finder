#!usr/bin/python
import cv2
import numpy as np

camera_device = 0
width = 320
height = 240
fps = 30

def nothing(x):
	pass

# Iniciacion de ventana de trackVars
cv2.namedWindow("WindowSizetoCrop",1)
cv2.moveWindow("WindowSizetoCrop",850,0)

cv2.createTrackbar('Width', 'WindowSizetoCrop',1,width, nothing)
cv2.createTrackbar('Heigth', 'WindowSizetoCrop',1,height, nothing)

def select_objective(event, x, y, flags, img):
	global refPt, cropping, cropimg

	w = cv2.getTrackbarPos('Width','WindowSizetoCrop')
	h = cv2.getTrackbarPos('Heigth','WindowSizetoCrop')

	if event == cv2.EVENT_LBUTTONDOWN:
		refPt = [(x, y)]
		cropping = True
	
	elif event == cv2.EVENT_LBUTTONUP:
		cropping = False
		aux = img.copy()
		cropimg = aux[ refPt[0][1]:(refPt[0][1]+h), refPt[0][0]:(refPt[0][0]+w)  ]
		cv2.rectangle(img, refPt[0], (refPt[0][0]+w, refPt[0][1]+h), (0, 0, 255), 2)
		cv2.imshow('WindowSizetoCrop', cropimg)


def main():
	global cropimg
	# Iniciacion de la camara especificada con el ID
	vSrc = cv2.VideoCapture(camera_device)
	# Confiuracion del tamano de la imagen del video
	vSrc.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH,width)
	vSrc.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT,height)
	# Configuracion de los cuadros por segundo
	vSrc.set(cv2.cv.CV_CAP_PROP_FPS,fps)
	i = 0

	while ( vSrc.isOpened() ):
		# Lectura de cuadros de la camara
		ret,imgSrc = vSrc.read()

		if (ret == True):
			cv2.imshow('VideoSource',imgSrc)
			cv2.setMouseCallback('VideoSource', select_objective, imgSrc)
		
		key = cv2.waitKey(5) & 0xFF
		
		if key == ord("q"):
	   		break
	   	elif key == ord("s"):
	   		i+=1
	   		cv2.imwrite('patternsBl/pat%d.jpg'%(i),cropimg)
	   		#cv2.imwrite('searchLabels/sLabel%d.jpg'%(i),cropimg)
	
	vSrc.release()
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main()	