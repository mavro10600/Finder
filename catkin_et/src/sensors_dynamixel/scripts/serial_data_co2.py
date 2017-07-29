
#!/usr/bin/env python
import sys
import rospy
import time
from SerialDataGateway import SerialDataGateway
from std_msgs.msg import Header 

from std_msgs.msg import Int32 
from sensor_msgs.msg import Imu
from termcolor import colored

class Launchpad_Class(object):
	
	def __init__(self):
		print colored("Staring Launchpad Class with arduino-thermal","yellow")
###############################################################################
#Variables de los sensores			
		self.co2 = 0.0

		self._LastUpdate_Microsec=0.0
		self._Second_Since_Last_Update=0.0
		self.deltat=0.0
		self.lastUpdate=0.0
###############################################################################
#Asignamos valores del puerto y baudios de la stellaris
		port=rospy.get_param("~port","/dev/arduino-thermal")
		#port=rospy.get_param("~port","/dev/ttyUSB0")
		baudRate=int(rospy.get_param("~baudRate",115200))

###############################################################################
		rospy.loginfo("starting with serialport:"+port+" baudrate: "+str(baudRate))
		self._SerialDataGateway=SerialDataGateway(port,baudRate,self._HandleReceivedLine)
		rospy.loginfo("started serial communication")
		
###############################################################################
#publisher y suscribers

		self.co2_level_publisher = rospy.Publisher("/hardware/sensors/co2",Int32,queue_size=5)

#Rutinas de actulizacion del publicador


################################################################################
#la rutina _HandleRececivedLine es el que toma los valores de los encoders dela stellaris y lo convierte en los topicos a publicar 
	def _HandleReceivedLine(self,line):	
		if(len(line)>0):

			lineParts=line.split('\t')
			try:
				if(lineParts[0]=='e'):
					self.co2 = int(lineParts[1])
					self.co2_level_publisher.publish(self.co2)


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
	rospy.init_node('co2_node',anonymous=True)
	#rate = rospy.Rate(10) # 10hz
	launchpad = Launchpad_Class()
	try:
		rate = rospy.Rate(10) # 10hz
		launchpad.Start()
		print colored("co2 sensor started",'green')

		print colored("Topic:'\n- /hardware/sensors/co2', Type:Int16 ","blue")
		while not rospy.is_shutdown():
			rate.sleep()
		#rospy.spin()

	except rospy.ROSInterruptException:
		rospy.logwarn("error in main function")
	
	launchpad.Reset_Launchpad()
	launchpad.Stop()
