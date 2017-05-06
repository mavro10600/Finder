#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy

class Control_interface:
    
    def __init__(self, node_name_override = 'control_interface'):
		
        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("control_interface starting with name %s", self.nodename) 
        self.rate = rospy.get_param("param_global_rate", 10)

        self.axes_names     = {'left_stick_hor':0, 'left_stick_ver':1, 'LT':2, 'right_stick_hor':3, 
                            'right_stick_ver':4, 'RT':5, 'cross_hor':6, 'cross_ver':7}
        self.buttons_names  = {'A':0, 'B':1, 'X':2, 'Y':3, 'LB':4, 'RB':5, 'back':6, 'start':7, 'power':8, 
                            'btn_stick_left':9, 'btn_stick_right':10}

        self.basePub    = rospy.Publisher("base_des", Float32)
        #self.baseDebug  = rospy.Publisher("base_debug", Int16)
        self.armPub     = rospy.Publisher("arm_des", Float32)
        self.forearmPub = rospy.Publisher("forearm_des", Float32)
        self.wristPub   = rospy.Publisher("wrist_des", Float32)
        self.palmPub    = rospy.Publisher("palm_des", Float32)
        self.gripperPub = rospy.Publisher("gripper_des", Float32)
        self.leftPub    = rospy.Publisher("left_des", Float32)
        self.rightPub   = rospy.Publisher("right_des", Float32)
        self.frPub      = rospy.Publisher("fr_des", Float32)
        self.flPub      = rospy.Publisher("fl_des", Float32)
        self.brPub      = rospy.Publisher("br_des", Float32)
        self.blPub      = rospy.Publisher("bl_des", Float32)
        self.frResetPub = rospy.Publisher("fr_reset", Int16)
        self.flResetPub = rospy.Publisher("fl_reset", Int16)
        self.brResetPub = rospy.Publisher("br_reset", Int16)
        self.blResetPub = rospy.Publisher("bl_reset", Int16)
        self.operationModePub = rospy.Publisher("operation_mode", Int16)

	self.gripperResetPub = rospy.Publisher("gripper_reset", Int16)

        self.offset_val = 0
        self.offsetPub = rospy.Publisher("offset", Int16)

        self.base_des = 0    
        self.arm_des = 0
        self.forearm_des = 0
        self.wrist_des = 0
        self.palm_des = 0
        self.gripper_des = 0
        
        # Variables internas
        self.angular_rate = 0
        self.linear_rate = 0
        self.base_rate = 0

        # Activation variables for the fr, fl, br and bl
        self.bl_active = 0
        self.fr_active = 0
        self.br_active = 0
        self.fl_active = 0

        # Rate vars for all litlle arms
        self.little_arm_rate = 0
        self.fr_rate = 0
        self.fl_rate = 0
        self.br_rate = 0
        self.bl_rate = 0

        self.little_arm_reset = 0
        self.fr_reset = 0
        self.fl_reset = 0
        self.br_reset = 0
        self.bl_reset = 0   

        self.lock = False
        self.fast_arms = False
        self.fast_traction = False

        self.arm_out = 0
        self.forearm_out = 0
        # self.armOutPub = rospy.Publisher("arm_out", Int16)
        # self.forearmOutPub = rospy.Publisher("forearm_out", Int16)

        self.operation_mode = "Navigation"

        # THIS MUST BE AT THE END!!!
        self.joySub = rospy.Subscriber("joy", Joy, self.joyCb)        

    def joyCb(self, data):
		
        if data.buttons[self.buttons_names['A']] == 1:
            self.operation_mode = "ArmControl"
        
        if data.buttons[self.buttons_names['X']] == 1:
            self.operation_mode = "Navigation"

        self.angular_rate   = data.axes[self.axes_names['left_stick_hor']] * 100
        self.linear_rate    = data.axes[self.axes_names['left_stick_ver']] * 100

        # El lock existe para evitar que el presionar el eje cross_hor y otro boton a eje
        # al mismo tiempo "desborde" el base_rate
        # if not self.lock:
        #     if data.axes[self.axes_names['cross_hor']] != 0:
        #         self.base_rate += data.axes[self.axes_names['cross_hor']] * -.1
        #         self.lock = True
        # if data.axes[self.axes_names['cross_hor']] == 0:
        #     self.lock = False;
        # if data.axes[self.axes_names['RT']] != 1:
        #     self.base_des = ((data.axes[self.axes_names['RT']] - 1) / 2) 
        # else:
        #     self.base_des = ((data.axes[self.axes_names['LT']] - 1) / 2) * -1

        # Se reemplazaron por comparaciones para Booleanos (0 es False y 1 es True)
        self.bl_active = data.buttons[self.buttons_names['LB']]
        self.fr_active = data.axes[self.axes_names['RT']] < 0
        self.br_active = data.buttons[self.buttons_names['RB']]
        self.fl_active = data.axes[self.axes_names['LT']] < 0

        # El valor de little arm rate es fijo, y puede ser -10 o + 10 como maximo
        self.little_arm_rate = data.axes[self.axes_names['right_stick_ver']] * 10.
        # self.little_arm_reset = data.buttons[self.buttons_names['B']]

        self.fast_traction = data.buttons[self.buttons_names['btn_stick_left']]
        self.fast_arms = data.buttons[self.buttons_names['btn_stick_right']]

        self.offset_val = data.buttons[self.buttons_names['Y']]

        self.base_des = data.axes[self.axes_names['left_stick_hor']] * 1

        if data.buttons[self.buttons_names['B']] == 0:        
            self.arm_des = data.axes[self.axes_names['left_stick_ver']] * 100

        if data.buttons[self.buttons_names['B']] == 1:
            self.forearm_des = data.axes[self.axes_names['left_stick_ver']] * 100

        self.wrist_des = data.axes[self.axes_names['right_stick_ver']] * -1
        self.palm_des = data.axes[self.axes_names['right_stick_hor']] * 0.7 

        if data.axes[self.axes_names['RT']] != 1:
            self.gripper_des = ((data.axes[self.axes_names['RT']] - 1) / 2) 
        else:
            self.gripper_des = ((data.axes[self.axes_names['LT']] - 1) / 2) * -1

	if data.axes[self.axes_names['RT']] == -1:
            if data.axes[self.axes_names['LT']] == -1:
                self.gripperResetPub.publish(1);

        # self.arm_out   = data.axes[self.axes_names['left_stick_ver']] * 60
        # self.forearm_out   = data.axes[self.axes_names['right_stick_ver']] * -60
        
    def motorTractionUpdate(self):
		       
        left_des = self.linear_rate - self.angular_rate
        right_des = self.linear_rate + self.angular_rate

        if not self.fast_traction:
            left_des = left_des / 2
            right_des = right_des / 2

        left_des = self.constrain(left_des,-100,100)
        right_des = self.constrain(right_des,-100,100)

        self.leftPub.publish(left_des)
        self.rightPub.publish(right_des)

        # self.testpub1.publish(self.testdata1)
        # self.testpub2.publish(self.testdata2)

        # print "left: " + str(left_des)
        # print "right: " + str(right_des)


    def motorArmUpdate(self):

        self.basePub.publish(self.base_des)
        self.armPub.publish(self.arm_des)
        self.forearmPub.publish(self.forearm_des)
        self.wristPub.publish(self.wrist_des)
        self.palmPub.publish(self.palm_des)
        self.gripperPub.publish(self.gripper_des)
        # self.baseDebug.publish(int(self.base_rate * 20))
        # print "base: " + str(self.base_rate)


    def motorTractionArmUpdate(self):

        if self.fast_arms:
            self.tmp_rate = self.little_arm_rate * 2
        else:
            self.tmp_rate = self.little_arm_rate 

        if self.fr_active:
            self.fr_rate = self.tmp_rate
        else:
            self.fr_rate = 0
        if self.fl_active:
            self.fl_rate = self.tmp_rate
        else:
            self.fl_rate = 0
        if self.br_active:
            self.br_rate = self.tmp_rate
        else:
            self.br_rate = 0
        if self.bl_active:
            self.bl_rate = self.tmp_rate
        else:
            self.bl_rate = 0

        # if self.little_arm_reset == 1:
        #     if self.fr_active:
        #         self.fr_rate = 0
        #         self.frResetPub.publish(1)  
        #     if self.fl_active:
        #         self.fl_rate = 0
        #         self.flResetPub.publish(1)
        #     if self.br_active:
        #         self.br_rate = 0
        #         self.brResetPub.publish(1)
        #     if self.bl_active:
        #         self.bl_rate = 0
        #         self.blResetPub.publish(1)
        # else:
        #     self.frResetPub.publish(0)
        #     self.flResetPub.publish(0)   
        #     self.brResetPub.publish(0)       
        #     self.blResetPub.publish(0)      

        self.frPub.publish(self.fr_rate * -1)
        self.flPub.publish(self.fl_rate * -1)
        self.brPub.publish(self.br_rate)
        self.blPub.publish(self.bl_rate * -1)                          

        # print "fr: " + str(self.fr_rate)
        # print "fl: " + str(self.fl_rate)
        # print "br: " + str(self.br_rate)
        # print "bl: " + str(self.bl_rate)
        
        
    def spin(self):
		
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.operation_mode == 'ArmControl':
                # self.armPub.publish(self.arm_des)
                # self.forearmPub.publish(self.forearm_des)
                # self.armOutPub.publish(self.arm_out)
                # self.forearmOutPub.publish(self.forearm_out)
                # self.basePub.publish(self.base_des)
                self.motorArmUpdate()
                self.operationModePub.publish(1);
            else:
                self.motorTractionUpdate()
                self.motorTractionArmUpdate()
                self.offsetPub.publish(self.offset_val)
                self.operationModePub.publish(0);

            r.sleep()
            

    def constrain(self, value, lower_limit, higher_limit):
        
        if value > higher_limit:
            value = higher_limit
        elif value < lower_limit:
            value = lower_limit
        return value


if __name__ == '__main__':
    """ main """
    control_interface = Control_interface()
    control_interface.spin()
