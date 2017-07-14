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
		self.battery_value=0
		
		self._reset_base1_value=0;
		self._reset_base2_value=0;
		self._reset_shoulder1_value=0;
		self._reset_shoulder2_value=0;
		self._reset_elbow1_value=0;
		self._reset_elbow2_value=0;
		self._reset_roll1_value=0;
		self._reset_roll2_value=0;
		self._reset_pitch1_value=0;
		self._reset_pit1ch2_value=0;
		self._reset_yaw1_value=0;
		self._reset_yaw2_value=0;
		
		self.base_pos=0
		self.shoulder_pos=0
		self.elbow_pos=0
		self.roll_pos=0
		self.pitch_pos=0
		self.yaw_pos=0
		self.gripper_pos=0
		
		self.base=0
		self.shoulder=0
		self.elbow=0
		self.roll=0
		self.pitch=0
		self.yaw=0		
		self.gripper=0

		self._LastUpdate_Microsec=0
		self._Second_Since_Last_Update=0
#########################################
#Asignamos valores del puerto y baudios de la stellaris
		port=rospy.get_param("~port","/dev/stellaris-brazo")
#		port=rospy.get_param("~port","/dev/ttyACM0")
		baudRate=int(rospy.get_param("~baudRate",115200))

#########################################
		rospy.loginfo("starting with serialport:"+port+"baudrate:"+str(baudRate))
		self._SerialDataGateway=SerialDataGateway(port,baudRate,self._HandleReceivedLine)
		rospy.loginfo("started serial communication")
		
###########################################
#publisher y suscribers

		
		self._SerialPublisher=rospy.Publisher('serial_arm',String,queue_size=10)
		self.deltat=0
		self.lastUpdate=0
		
		self._Base1_Reset=rospy.Publisher('base1_reset',Int16,queue_size=10)
		self._Base2_Reset=rospy.Publisher('base2_reset',Int16,queue_size=10)
		self._Shoulder1_Reset=rospy.Publisher('shoulder1_reset',Int16,queue_size=10)
		self._Shoulder2_Reset=rospy.Publisher('shoulder2_reset',Int16,queue_size=10)
		self._Elbow1_Reset=rospy.Publisher('elbow1_reset',Int16,queue_size=10)
		self._Elbow2_Reset=rospy.Publisher('elbow2_reset',Int16,queue_size=10)
		self._Roll1_Reset=rospy.Publisher('roll1_reset',Int16,queue_size=10)
		self._Roll2_Reset=rospy.Publisher('roll2_reset',Int16,queue_size=10)
		self._Pitch1_Reset=rospy.Publisher('pitch1_reset',Int16,queue_size=10)
		self._Pitch2_Reset=rospy.Publisher('pitch2_reset',Int16,queue_size=10)
		self._Yaw1_Reset=rospy.Publisher('yaw1_reset',Int16,queue_size=10)
		self._Yaw2_Reset=rospy.Publisher('yaw2_reset',Int16,queue_size=10)
		
		self._Shoulder_Encoder=rospy.Publisher('shoulder',Int64,queue_size=5)
		self._Base_Encoder=rospy.Publisher('base',Int64,queue_size=10)
		self._Elbow_Encoder=rospy.Publisher('elbow',Int64,queue_size=5)
		self._Roll_Encoder=rospy.Publisher('roll',Int64,queue_size=5)
		self._Pitch_Encoder=rospy.Publisher('pitch',Int64,queue_size=5)
		self._Yaw_Encoder=rospy.Publisher('yaw',Int64,queue_size=10)
		self._Gripper_Encoder=rospy.Publisher('gripper',Int64,queue_size=10)
		self._Battery_Level=rospy.Publisher('battery_level',Int64,queue_size=10)		
								

		
		self.base_out=rospy.Subscriber('base_out',Int16,self._Update_Base)
		self.shoulder_out=rospy.Subscriber('shoulder_out',Int16,self._Update_Shoulder)
		self.elbow_out=rospy.Subscriber('elbow_out',Int16,self._Update_Elbow)
		self.roll_out=rospy.Subscriber('roll_out',Int16,self._Update_Roll)
		self.pitch_out=rospy.Subscriber('pitch_out',Int16,self._Update_Pitch)
		self.yaw_out=rospy.Subscriber('yaw_out',Int16,self._Update_Yaw)
		self.gripper_out=rospy.Subscriber('gripper_out',Int16,self._Update_Gripper)		
		
		
##############################################
#defino las rutinas de actulizacion del publicador

	def _Update_Base(self,base_out):
		self.base_pos=base_out.data
		rospy.loginfo(base_out.data)
		speed_message='s %d %d %d %d %d %d \r' %(int(self.base_pos),int(self.shoulder_pos),int(self.elbow_pos),int(self.roll_pos),int(self.pitch_pos),int(self.yaw_pos))	
		self._WriteSerial(speed_message)
		
	def _Update_Shoulder(self,shoulder_out):
		self.shoulder_pos=shoulder_out.data
		rospy.loginfo(shoulder_out.data)
		speed_message='s %d %d %d %d %d %d \r' %(int(self.base_pos),int(self.shoulder_pos),int(self.elbow_pos),int(self.roll_pos),int(self.pitch_pos),int(self.yaw_pos))	
		self._WriteSerial(speed_message)
		
	def _Update_Elbow(self,elbow_out):
		self.elbow_pos=elbow_out.data
		rospy.loginfo(elbow_out.data)
		speed_message='s %d %d %d %d %d %d \r' %(int(self.base_pos),int(self.shoulder_pos),int(self.elbow_pos),int(self.roll_pos),int(self.pitch_pos),int(self.yaw_pos))	
		self._WriteSerial(speed_message)
		
	def _Update_Roll(self,roll_out):
		self.roll_pos=roll_out.data
		rospy.loginfo(roll_out.data)
		speed_message='s %d %d %d %d %d %d \r' %(int(self.base_pos),int(self.shoulder_pos),int(self.elbow_pos),int(self.roll_pos),int(self.pitch_pos),int(self.yaw_pos))	
		self._WriteSerial(speed_message)
		
	def _Update_Pitch(self,pitch_out):
		self.pitch_pos=pitch_out.data
		rospy.loginfo(pitch_out.data)
		speed_message='s %d %d %d %d %d %d \r' %(int(self.base_pos),int(self.shoulder_pos),int(self.elbow_pos),int(self.roll_pos),int(self.pitch_pos),int(self.yaw_pos))	
		self._WriteSerial(speed_message)
		
	def _Update_Yaw(self,yaw_out):
		self.yaw_pos=yaw_out.data
		rospy.loginfo(yaw_out.data)
		speed_message='s %d %d %d %d %d %d \r' %(int(self.base_pos),int(self.shoulder_pos),int(self.elbow_pos),int(self.roll_pos),int(self.pitch_pos),int(self.yaw_pos))	
		self._WriteSerial(speed_message)
	
	def _Update_Gripper(self,gripper_out):
		self.gripper_pos=gripper_out.data
		rospy.loginfo(gripper_out.data)
		speed_message='s %d %d %d %d %d %d \r' %(int(self.base_pos),int(self.shoulder_pos),int(self.elbow_pos),int(self.roll_pos),int(self.pitch_pos),int(self.yaw_pos))	
		self._WriteSerial(speed_message)
	
#################################################
#la rutina _HandleRececivedLine es el que toma los valores de los encoders dela stellaris y lo convierte en los topicos a publicar 
	
	def _HandleReceivedLine(self,line):	
		self._Counter=self._Counter+1
		self._SerialPublisher.publish(String(str(self._Counter)+", in:"+line))
		
		if(len(line)>0):
			lineParts=line.split('\t')
			try:
				if(lineParts[0]=='e'):
					self.base=long(lineParts[1])
					self.shoulder=long(lineParts[2])
					self.elbow=long(lineParts[3])
					self.roll=long(lineParts[4])
					self.pitch=long(lineParts[5])
					self.yaw=long(lineParts[6])
																									
					
					self._Base_Encoder.publish(self.base)
					self._Shoulder_Encoder.publish(self.shoulder)
					self._Elbow_Encoder.publish(self.elbow)
					self._Roll_Encoder.publish(self.roll)
					self._Pitch_Encoder.publish(self.pitch)
					self._Yaw_Encoder.publish(self.yaw)
				
				if(lineParts[0]=='b'):
					
					self.battery_value=int(lineParts[1])
					self._Battery_Level.publish(self.battery_value)
				
				if (lineParts[0]=='n'):
					self.reset_base1_value=int(lineParts[1])
					self.reset_base2_value=int(lineParts[2])
					self.reset_shoulder1_value=int(lineParts[3])
					self.reset_shoulder2_value=int(lineParts[4])
					self.reset_elbow1_value=int(lineParts[5])
					self.reset_elbow2_value=int(lineParts[6])
					self.reset_roll1_value=int(lineParts[7])
					self.reset_roll2_value=int(lineParts[8])
					self.reset_pitch1_value=int(lineParts[9])
					self.reset_pitch2_value=int(lineParts[10])
					self.reset_yaw1_value=int(lineParts[11])
					self.reset_yaw2_value=int(lineParts[12])
					
					self._Base1_Reset.publish(self.reset_base1_value)
					self._Base2_Reset.publish(self.reset_base2_value)
					self._Shoulder1_Reset.publish(self.reset_shoulder1_value)
					self._Shoulder2_Reset.publish(self.reset_shoulder2_value)
					self._Elbow1_Reset.publish(self.reset_elbow1_value)
					self._Elbow2_Reset.publish(self.reset_elbow2_value)
					self._Roll1_Reset.publish(self.reset_roll1_value)
					self._Roll2_Reset.publish(self.reset_roll2_value)
					self._Pitch1_Reset.publish(self.reset_pitch1_value)
					self._Pitch2_Reset.publish(self.reset_pitch2_value)
					self._Yaw1_Reset.publish(self.reset_yaw1_value)
					self._Yaw2_Reset.publish(self.reset_yaw2_value)
					
				
			except:
				rospy.logwarn("Error in sensor values")
				rospy.logwarn(lineParts)
				pass

###################################################
#
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

#######################################################
###funcion principal "main" 
	
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
		
		
