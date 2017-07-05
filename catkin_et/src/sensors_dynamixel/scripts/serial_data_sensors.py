#!/usr/bin/env python
import sys
import rospy
import time
from SerialDataGateway import SerialDataGateway
from std_msgs.msg import Int16,Int32,Int64,Float32,String,Header,UInt64
from termcolor import colored

class Launchpad_Class(object):
	
	def __init__(self):
		print "Iniciando clase launchpad sensores y dynamixel"
######################################
#Variables de los sensores			
		self.gripper = 0.0
		self.roll = 0.0
		self.pitch = 0.0
		self.yaw = 0.0
		self.co2=0
		self.dynamixel_pos = 0.0

		self._LastUpdate_Microsec=0
		self._Second_Since_Last_Update=0
#########################################
#Asignamos valores del puerto y baudios de la stellaris
		#port=rospy.get_param("~port","/dev/stellaris-dynamixel")
		port=rospy.get_param("~port","/dev/ttyACM0")
		baudRate=int(rospy.get_param("~baudRate",115200))

#########################################
		rospy.loginfo("starting with serialport:"+port+" baudrate: "+str(baudRate))
		self._SerialDataGateway=SerialDataGateway(port,baudRate,self._HandleReceivedLine)
		rospy.loginfo("started serial communication")
		
###########################################
#publisher y suscribers
		self.deltat=0
		self.lastUpdate=0
		self._Imu_Roll_Publisher = rospy.Publisher("/imu/roll",Float32,queue_size=5)
		self._Imu_Pitch_Publisher = rospy.Publisher("/imu/pitch",Float32,queue_size=5)
		self._Imu_Yaw_Publisher = rospy.Publisher("/imu/yaw",Float32,queue_size=5)
		self.co2_level_publisher = rospy.Publisher("/co2/level",Int32,queue_size=5)
		self.gripper_pos_publisher = rospy.Publisher("gripper",Int32,queue_size=5)

		self.gripper_out=rospy.Subscriber('gripper_out',Int16,self._Update_Dynamixel)	
		
		
##############################################
#defino las rutinas de actulizacion del publicador
	def _Update_Dynamixel(self,dynamixel_out):
		self.gripper_pos=dynamixel_out.data
		rospy.loginfo(dynamixel_out.data)
		dynamixel_message='s %d \r' %(int(self.gripper_pos))
		self._WriteSerial(dynamixel_message)
	
#################################################
#la rutina _HandleRececivedLine es el que toma los valores de los encoders dela stellaris y lo convierte en los topicos a publicar 
	def _HandleReceivedLine(self,line):	
		if(len(line)>0):

			lineParts=line.split('\t')
			try:
				if(lineParts[0]=='e'):
					self.roll = float(lineParts[1])
					self.pitch = float(lineParts[2])
					self.yaw = float(lineParts[3])
					self.co2 = int(lineParts[4])
					self.dynamixel_pos  = int(lineParts[5])

					self._Imu_Roll_Publisher.publish(self.roll)
					self._Imu_Pitch_Publisher.publish(self.pitch)
					self._Imu_Yaw_Publisher.publish(self.yaw)
					self.co2_level_publisher.publish(self.co2)
					self.gripper_pos_publisher.publish(self.dynamixel_pos)
					#time.sleep(0.01)
					

			except:
				rospy.logwarn("Error in sensor values")
				rospy.logwarn(lineParts)
				print "Error:", sys.exc_info()[0]
				pass

###################################################
#
	def _WriteSerial(self,message):
		#self._SerialPublisher.publish(String(str(self._Counter)+", out:"+message))
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
		

#######################################################
###funcion principal "main" 
	
if __name__=='__main__':
	rospy.init_node('servo_imu_dynamixel_node',anonymous=True)
	#rate = rospy.Rate(10) # 10hz
	launchpad = Launchpad_Class()
	try:
		rate = rospy.Rate(10) # 10hz
		launchpad.Start()
		print colored("Dynamixel and imu node successfully started",'green')

		print colored("Topics:'\n- /imu/roll'\n- '/imu/pitch'\n- '/imu/yaw'\n- '/co2/level',\n- /gripper_rotation/","blue")
		while not rospy.is_shutdown():
			rate.sleep()
		#rospy.spin()

	except rospy.ROSInterruptException:
		rospy.logwarn("error in main function")
	
	launchpad.Reset_Launchpad()
	launchpad.Stop()
