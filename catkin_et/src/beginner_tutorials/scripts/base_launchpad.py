#!/usr/bin/env python
'''
launchpad node , recibimos valores de sensores de la tarjeta stellaris, y 
los publicamos como topicos
el codigo es de chefbot
'''
#Python client library for ROS

import rospy
import sys
import time
import math

from SerialDataGateway import SerialDataGateway
from std_msgs.msg import Int16,Int32,Int64,Float32,String,Header,UInt64
#from sensor_msgs.msg import Imu

#Clase para manejar datos seriales y convertirla en topicos de ROS

class Launchpad_Class(object):
	
	def __init__(self):
		print "Iniciando clase launchpad"
		
######################################
#Variables de los sensores		
		self._Counter=0
		self._reset_flipper1_value=0;
		self._reset_flipper2_value=0;
		self._reset_flipper3_value=0;
		self._reset_flipper4_value=0;
		
		self._left_encoder_value=0
		self._left_wheel_speed=0
		
		self._right_encoder_value=0
		self._right_wheel_speed=0
		
		self._flipper1_encoder_value=0
		self._flipper1_wheel_speed=0		
		
		self._flipper2_encoder_value=0
		self._flipper2_wheel_speed=0		
		
		self._flipper3_encoder_value=0
		self._flipper3_wheel_speed=0		
		
		self._flipper4_encoder_value=0
		self._flipper4_wheel_speed=0		

		self._LastUpdate_Microsec=0
		self._Second_Since_Last_Update=0
#########################################
#Asignamos valores del puerto y baudios de la stellaris
#		port=rospy.get_param("~port","/dev/ttyACM0")
		port=rospy.get_param("~port","/dev/stellaris-base")
		baudRate=int(rospy.get_param("~baudRate",115200))

#########################################
		rospy.loginfo("starting with serialport:"+port+"baudrate:"+str(baudRate))
		self._SerialDataGateway=SerialDataGateway(port,baudRate,self._HandleReceivedLine)
		rospy.loginfo("started serial communication")
		
###########################################
#publisher y suscribers
		self._Left_Encoder=rospy.Publisher('left_lec',Int16,queue_size=10)
		
		self._SerialPublisher=rospy.Publisher('serial_base',String,queue_size=10)
		self.deltat=0
		self.lastUpdate=0
		self._Right_Encoder=rospy.Publisher('right_lec',Int16,queue_size=10)
		
		self._Flipper1_Reset=rospy.Publisher('flipper1_reset',Int16,queue_size=10)
		
		self._Flipper2_Reset=rospy.Publisher('flipper2_reset',Int16,queue_size=10)
		
		self._Flipper3_Reset=rospy.Publisher('flipper3_reset',Int16,queue_size=10)
		
		self._Flipper4_Reset=rospy.Publisher('flipper4_reset',Int16,queue_size=10)
		
		self._Flipper1_Encoder=rospy.Publisher('flip1_lec',Int16,queue_size=10)
		self._Flipper2_Encoder=rospy.Publisher('flip2_lec',Int16,queue_size=10)
		self._Flipper3_Encoder=rospy.Publisher('flip3_lec',Int16,queue_size=10)
		self._Flipper4_Encoder=rospy.Publisher('flip4_lec',Int16,queue_size=10)
		
		self._left_motor_speed=rospy.Subscriber('left_out',Int16,self._Update_Left_Speed)
		self._right_motor_speed=rospy.Subscriber('right_out',Int16,self._Update_Right_Speed)
		self._flipper1_speed=rospy.Subscriber('flipper1_out',Int16,self._Update_Flipper1_Speed)
		self._flipper2_speed=rospy.Subscriber('flipper2_out',Int16,self._Update_Flipper2_Speed)
		self._flipper3_speed=rospy.Subscriber('flipper3_out',Int16,self._Update_Flipper3_Speed)
		self._flipper4_speed=rospy.Subscriber('flipper4_out',Int16,self._Update_Flipper4_Speed)
		
	def _Update_Left_Speed(self,left_speed):
		self._left_wheel_speed=left_speed.data
		#rospy.loginfo(left_speed.data)
		speed_message='s %d %d %d %d %d %d\r' %(int(self._left_wheel_speed),int(self._right_wheel_speed),int(self._flipper1_wheel_speed),int(self._flipper2_wheel_speed),int(self._flipper3_wheel_speed),int(self._flipper4_wheel_speed))	
		self._WriteSerial(speed_message)
		
	def _Update_Right_Speed(self,right_speed):
		self._right_wheel_speed=right_speed.data
		#rospy.loginfo(right_speed.data)
		speed_message='s %d %d %d %d %d %d\r' %(int(self._left_wheel_speed),int(self._right_wheel_speed),int(self._flipper1_wheel_speed),int(self._flipper2_wheel_speed),int(self._flipper3_wheel_speed),int(self._flipper4_wheel_speed))	
		self._WriteSerial(speed_message)
		

	def _Update_Flipper1_Speed(self,flipper1_speed):
                self._flipper1_wheel_speed=flipper1_speed.data
                #rospy.loginfo(flipper1_speed.data)
                speed_message='s %d %d %d %d %d %d\r' %(int(self._left_wheel_speed),int(self._right_wheel_speed),int(self._flipper1_wheel_speed),int(self._flipper2_wheel_speed),int(self._flipper3_wheel_speed),int(self._flipper4_wheel_speed))
                self._WriteSerial(speed_message)

	def _Update_Flipper2_Speed(self,flipper2_speed):
                self._flipper2_wheel_speed=flipper2_speed.data
                #rospy.loginfo(flipper2_speed.data)
                speed_message='s %d %d %d %d %d %d\r' %(int(self._left_wheel_speed),int(self._right_wheel_speed),int(self._flipper1_wheel_speed),int(self._flipper2_wheel_speed),int(self._flipper3_wheel_speed),int(self._flipper4_wheel_speed))
                self._WriteSerial(speed_message)

	def _Update_Flipper3_Speed(self,flipper3_speed):
                self._flipper3_wheel_speed=flipper3_speed.data
                #rospy.loginfo(flipper3_speed.data)
                speed_message='s %d %d %d %d %d %d\r' %(int(self._left_wheel_speed),int(self._right_wheel_speed),int(self._flipper1_wheel_speed),int(self._flipper2_wheel_speed),int(self._flipper3_wheel_speed),int(self._flipper4_wheel_speed))
                self._WriteSerial(speed_message)

	def _Update_Flipper4_Speed(self,flipper4_speed):
                self._flipper4_wheel_speed=flipper4_speed.data
                #rospy.loginfo(flipper4_speed.data)
                speed_message='s %d %d %d %d %d %d\r' %(int(self._left_wheel_speed),int(self._right_wheel_speed),int(self._flipper1_wheel_speed),int(self._flipper2_wheel_speed),int(self._flipper3_wheel_speed),int(self._flipper4_wheel_speed))
                self._WriteSerial(speed_message)


	def _HandleReceivedLine(self,line):	
		self._Counter=self._Counter+1
		self._SerialPublisher.publish(String(str(self._Counter)+", in:"+line))
		
		if(len(line)>0):
			lineParts=line.split('\t')
			try:
				if(lineParts[0]=='e'):
					self._left_encoder_value=long(lineParts[1])
					self._right_encoder_value=long(lineParts[2])					
					self._flipper1_encoder_value=long(lineParts[3])
					self._flipper2_encoder_value=long(lineParts[4])					
					self._flipper3_encoder_value=long(lineParts[5])
					self._flipper4_encoder_value=long(lineParts[6])					

					self._Left_Encoder.publish(self._left_encoder_value)
					self._Right_Encoder.publish(self._right_encoder_value)
					self._Flipper1_Encoder.publish(self._flipper1_encoder_value)
					self._Flipper2_Encoder.publish(self._flipper2_encoder_value)
					self._Flipper3_Encoder.publish(self._flipper3_encoder_value)
					self._Flipper4_Encoder.publish(self._flipper4_encoder_value)
			
				if(lineParts[0]=='n'):
					self.reset_flipper1_value=int(lineParts[1])
					self.reset_flipper2_value=int(lineParts[2])
					self.reset_flipper3_value=int(lineParts[3])
					self.reset_flipper4_value=int(lineParts[4])							
					
					self._Flipper1_Reset.publish(self.reset_flipper1_value)
					self._Flipper2_Reset.publish(self.reset_flipper2_value)
					self._Flipper3_Reset.publish(self.reset_flipper3_value)
					self._Flipper4_Reset.publish(self.reset_flipper4_value)
					
			except:
				rospy.logwarn("Error in sensor values")
				rospy.logwarn(lineParts)
				pass

	def _WriteSerial(self,message):
		self._SerialPublisher.publish(String(str(self._Counter)+", out:"+message))
		self._SerialDataGateway.Write(message)
		
	def Start(self):
		rospy.logdebug("Starting")
		self._SerialDataGateway.Start()

	def Stop(self):
		rospy.logdebug("Stoping")
		self._SerialDataGateway.Stop()
		
	def Reset_Launchpad(self):
		print "reset"
		reset='r\r'
		self._WriteSerial(reset)
		time.sleep(1)
		self._WriteSerial(reset)
		time.sleep(2)
		
	def SubscribeSpeed(self):
		a=1
	def SendSpeed(self):
		a=3
	
if __name__=='__main__':
	rospy.init_node('launchpad_ros',anonymous=True)
	launchpad=Launchpad_Class()
	try:
		launchpad.Start()
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.logwarn("error in main function")
	
	launchpad.Reset_Launchpad()
	launchpad.Stop()
		
		
