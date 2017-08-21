#!/usr/bin/env python
import sys
import rospy
import time
from SerialDataGateway import SerialDataGateway
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from termcolor import colored

class Launchpad_Class(object):
	
	def __init__(self):
		print colored("Staring Launchpad Class with arduino-imu","yellow")
###############################################################################
#Variables de los sensores			
		self.q_x = 0.0
		self.q_y = 0.0
		self.q_z = 0.0
		self.q_w = 0.0

		self._LastUpdate_Microsec=0.0
		self._Second_Since_Last_Update=0.0
		self.deltat=0.0
		self.lastUpdate=0.0
###############################################################################
#Asignamos valores del puerto y baudios de la stellaris
		#port=rospy.get_param("~port","/dev/arduino-imu")
		port=rospy.get_param("~port","/dev/ttyUSB1")
		baudRate=int(rospy.get_param("~baudRate",115200))

###############################################################################
		rospy.loginfo("starting with serialport:"+port+" baudrate: "+str(baudRate))
		self._SerialDataGateway=SerialDataGateway(port,baudRate,self._HandleReceivedLine)
		rospy.loginfo("started serial communication")
		
###############################################################################
#publisher y suscribers
		
		self.frame_id = '/imu'

		self.imu_pub = rospy.Publisher("thumper_imu",Imu,queue_size=10);

#Rutinas de actulizacion del publicador


################################################################################
#la rutina _HandleRececivedLine es el que toma los valores de los encoders dela stellaris y lo convierte en los topicos a publicar 
	def _HandleReceivedLine(self,line):	
		if(len(line)>0):

			lineParts=line.split('\t')
			try:
				if(lineParts[0]=='e'):
					self.q_x = float(lineParts[1])
					self.q_y = float(lineParts[2])
					self.q_z = float(lineParts[3])
					self.q_w = float(lineParts[4])

					imu_msg = Imu()
					h = Header()
					h.stamp = rospy.Time.now()
					h.frame_id = self.frame_id

					imu_msg.header = h
					imu_msg.orientation_covariance = (-1., )*9	
					imu_msg.angular_velocity_covariance = (-1., )*9
					imu_msg.linear_acceleration_covariance = (-1., )*9

					imu_msg.orientation.x = self.q_x
					imu_msg.orientation.y = self.q_y
					imu_msg.orientation.z = self.q_z
					imu_msg.orientation.w = self.q_w
					self.imu_pub.publish(imu_msg)


			except:
				rospy.logwarn("Error in sensor values")
				rospy.logwarn(lineParts)
				print "Error:", sys.exc_info()[0]
				pass

################################################################################
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
if __name__=='__main__':
	rospy.init_node('imu_node',anonymous=True)
	#rate = rospy.Rate(10) # 10hz
	launchpad = Launchpad_Class()
	try:
		rate = rospy.Rate(10) # 10hz
		launchpad.Start()
		print colored("MPU6050 node successfully started",'green')

		print colored("Topic:'\n- /thumper_imu', Type:sensor_msgs/Imu","blue")
		while not rospy.is_shutdown():
			rate.sleep()
		#rospy.spin()

	except rospy.ROSInterruptException:
		rospy.logwarn("error in main function")
	
	launchpad.Reset_Launchpad()
	launchpad.Stop()
