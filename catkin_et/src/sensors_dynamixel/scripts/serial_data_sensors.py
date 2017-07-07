#!/usr/bin/env python
import sys
import rospy
import time
from SerialDataGateway import SerialDataGateway
from std_msgs.msg import Int16,Int32,Int64,Float32,String,Header,UInt64
from sensor_msgs.msg import Imu
from termcolor import colored

class Launchpad_Class(object):
	
	def __init__(self):
		print "Iniciando clase launchpad sensores y dynamixel"
######################################
#Variables de los sensores			
		self.gripper = 0.0
		self.q_x = 0.0
		self.q_y = 0.0
		self.q_z = 0.0
		self.q_w = 0.0
		self.co2=0
		self.dynamixel_pos = 0.0

		self._LastUpdate_Microsec=0
		self._Second_Since_Last_Update=0
#########################################
#Asignamos valores del puerto y baudios de la stellaris
		port=rospy.get_param("~port","/dev/stellaris-dynamixel")
		#port=rospy.get_param("~port","/dev/ttyACM0")
		baudRate=int(rospy.get_param("~baudRate",115200))

#########################################
		rospy.loginfo("starting with serialport:"+port+" baudrate: "+str(baudRate))
		self._SerialDataGateway=SerialDataGateway(port,baudRate,self._HandleReceivedLine)
		rospy.loginfo("started serial communication")
		
###########################################
#publisher y suscribers
		self.deltat=0
		self.lastUpdate=0
		'''
		self._Imu_Roll_Publisher = rospy.Publisher("hardware/sensors/imu/roll",Float32,queue_size=5)
		self._Imu_Pitch_Publisher = rospy.Publisher("hardware/sensors/imu/pitch",Float32,queue_size=5)
		self._Imu_Yaw_Publisher = rospy.Publisher("hardware/sensors/imu/yaw",Float32,queue_size=5)
		'''
		'''
		self._Imu_Qx_Publisher = rospy.Publisher("hardware/sensors/imu/q_x",Float32,queue_size=5)
		self._Imu_Qy_Publisher = rospy.Publisher("hardware/sensors/imu/q_y",Float32,queue_size=5)
		self._Imu_Qz_Publisher = rospy.Publisher("hardware/sensors/imu/q_z",Float32,queue_size=5)
		self._Imu_Qw_Publisher = rospy.Publisher("hardware/sensors/imu/q_w",Float32,queue_size=5)
		'''
		
		self.frame_id = '/imu'
		'''
	    self.cal_offset = 0.0
        self.orientation = 0.0
        self.cal_buffer =[]
        self.cal_buffer_length = 1000
        self.imu_data = Imu(header=rospy.Header(frame_id="base_link"))
        self.imu_data.orientation_covariance = [1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6]
	    self.imu_data.angular_velocity_covariance = [1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6]
        self.imu_data.linear_acceleration_covariance = [-1,0,0,0,0,0,0,0,0]
        self.gyro_measurement_range = 150.0 
        self.gyro_scale_correction = 1.35
        '''
		self.imu_pub = rospy.Publisher("thumper_imu",Imu,queue_size=10);
		
		self.co2_level_publisher = rospy.Publisher("/hardware/sensors/co2",Int32,queue_size=5)
		self.gripper_pos_publisher = rospy.Publisher("hardware/gripper",Int32,queue_size=5)

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

					self.co2 = int(lineParts[5])
					self.dynamixel_pos  = int(lineParts[6])

					'''
					self._Imu_Qx_Publisher.publish(self.q_x)
					self._Imu_Qy_Publisher.publish(self.q_y)
					self._Imu_Qz_Publisher.publish(self.q_z)
					self._Imu_Qw_Publisher.publish(self.q_w)
					'''

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

		print colored("Topics:'\n- /thumper_imu', \n- '/co2/level',\n- /gripper_rotation/","blue")
		while not rospy.is_shutdown():
			rate.sleep()
		#rospy.spin()

	except rospy.ROSInterruptException:
		rospy.logwarn("error in main function")
	
	launchpad.Reset_Launchpad()
	launchpad.Stop()
