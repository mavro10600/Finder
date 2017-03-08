#!/usr/bin/env python

import rospy
import roslib
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class TwistToMotors():
	def __init__(self):
		rospy.init_node("twist_to motors")
		nodename=rospy.get_name()
		rospy.loginfo("%s started" % nodename)
		
		self.w=rospy.get_param("~base_width",0.2)
		
		self.pub_lmotor=rospy.Publisher('lwheel_vtarget',Float32,queue_size=10)
		self.pub_rmotor=rospy.Publisher('rwheel_vtarget',Float32,queue_size=10)
		rospy.loginfo("%s started" % nodename)
		
		self.w=rospy.get_param("~base_width",0.2)
		
		self.pub_lmotor=rospy.Publisher('lwheel_vtarget',Float32,queue_size=10)
		self.pub_rmotor=rospy.Publisher('rwheel_vtarget',Float32,queue_size=10)
		rospy.Subscriber('chefbot_teleop/cmd_vel',Twist,self.twistCallback)
		
		self.rate=rospy.get_param("~rate",50)
		self.timeout_ticks=rospy.get_param("~timeout_ticks",2)	
		self.left=0
		self.right=0
		
		
	def spin(self):
	
		r=rospy.Rate(self.rate)
		idle=rospy.Rate(10)
		then=rospy.Time.now()
		self.ticks_since_target=self.timeout_ticks
		
		while not rospy.is_shutdown():
			while not rospy.is_shutdown() and self.ticks_since_target < self.timeout_ticks:
				self.spinOnce()
				r.sleep()
			idle.sleep()

	def spinOnce(self):
		self.right=1.0*self.dx+self.dr*self.w/2
		self.left=1.0*self.dx-self.dr*self.w/2
		
		self.pub_lmotor.publish(self.left)
		self.pub_rmotor.publish(self.right)
		
		self.ticks_since_target+=1
		
	def twistCallback(self,msg):
		self.ticks_since_target=0
		self.dx=msg.linear.x
		self.dr=msg.angular.z
		self.dy=msg.linear.y
		
if __name__=='__main__':
	"""main"""
	twistToMotors=TwistToMotors()
	twistToMotors.spin()
	
		
