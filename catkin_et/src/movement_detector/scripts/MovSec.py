# -*- coding: utf-8 -*-
#Created on Wed Jul 15 11:18:02 2015

#@author: yolo

#camara1.py:		captura video de la camara de la pc y lo despliega

#segunda prueba
#- segmentación por color
#- detección de contornos
#- determinación por área
#- etiquetar
#- detector de movimiento


import numpy as np
import cv2
import time

cap = cv2.VideoCapture(1)

thresd=127
thresh=255
ret, frame = cap.read()                                          # captura imagen
prevgray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)               # aplica ventana de emborronamiento
grayimg = cv2.GaussianBlur(prevgray,(3,3),0)                    # convierte imagen a color a escala de grises
fondo=None
fondos=[]
cx=[0,300,600,900,1200,100,200,300,0,100,200,300]
cy=[0,0,0,0,0,500,200,200,200,200,300,300,300]
size=[200,300,400,500,600,1500]
secciones=[]
nsec=4
h,w=grayimg.shape
tsec=w/nsec
flag=False

def draw_flow(img, flow, step=16):
    h, w = img.shape[:2]
    y, x = np.mgrid[step/2:h:step, step/2:w:step].reshape(2,-1)
    fx, fy = flow[y,x].T
    lines = np.vstack([x, y, x+fx, y+fy]).T.reshape(-1, 2, 2)
    lines = np.int32(lines + 0.5)
    vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    cv2.polylines(vis, lines, 0, (0, 255, 0))
    for (x1, y1), (x2, y2) in lines:
        cv2.circle(vis, (x1, y1), 1, (0, 255, 0), -1)
    return vis

while(True):
    ret, img = cap.read()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray=cv2.GaussianBlur(gray,(5,5),0)
    cv2.imshow('captura',gray)
    imguno = gray  
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
   
    for i in range(0,nsec):
        lim=tsec*i
        sec=imguno[0:h,lim:lim+tsec]
        secciones.append(sec)        
        
        if flag:
            gris=secciones[i]
            fondo=fondos[i]
            resta = cv2.absdiff(fondo, gris)
            frame=fondo
            # Aplicamos un umbral
            umbral = cv2.threshold(resta, 25, 255, cv2.THRESH_BINARY)[1]
            # Dilatamos el umbral para tapar agujeros
            umbral = cv2.dilate(umbral, None, iterations=2)
            # Copiamos el umbral para detectar los contornos
            contornosimg = umbral.copy()
            # Buscamos contorno en la imagen
            contornos, hier = cv2.findContours(contornosimg,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)    # encuentra contornos
            flow = cv2.calcOpticalFlowFarneback(gris,fondo , 0.5, 3, 15, 3, 5, 1.2, 0) # aplica el algoritmo de flujo optico entre la imagen actual y la anterior
                                
            #cv2.imshow('umbral%i'%i,umbral)
            #print 'b'
            
            # Recorremos todos los contornos encontrados
            for c in contornos:
            		# Eliminamos los contornos más pequeños
                if cv2.contourArea(c) < 500:
                    continue
                #print len(c)
                # Obtenemos el bounds del contorno, el rectángulo mayor que engloba al contorno
                (x, y, we, hi) = cv2.boundingRect(c)
                # Dibujamos el rectángulo del bounds
                cv2.rectangle(frame, (x, y), (x + we, y + hi), (0, 255, 0), 2)
                indice=1+i
                cv2.imshow('flow%i'%i, draw_flow(frame, flow))
                print 'movimiento en la sección: '+str(i)

            cv2.namedWindow('mov%i'%i,cv2.WINDOW_NORMAL)
            cv2.imshow('mov%i'%i,frame)
            cv2.moveWindow('mov%i'%i,cx[i],cy[i])
        
    flag=True
    fondos=secciones
    secciones=[]

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()