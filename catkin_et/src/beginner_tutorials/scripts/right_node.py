#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from math import *

class Right_node:

    def __init__(self, node_name_override = 'right_node'):

        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("right_node starting with name %s", self.nodename) 
        self.rate = rospy.get_param("param_global_rate", 10)

        self.rightOutPub = rospy.Publisher("right_out", Int16)
        self.rightAngPub = rospy.Publisher("right_ang", Float32)
        self.rightVelPub = rospy.Publisher("right_vel", Float32)

        
        #TODO cambiar por parametros
        self.right_enc_ana = False
        self.right_enc_max = 1023
        self.right_offset = 0
        #self.right_enc_lim = 100
        #self.right_lec_min = 485
        #self.right_lec_max = 1004
        #self.right_lec_dir = -100

        # PID control parameters
        self.kp_pos = 30.
        self.ki_pos = 0.
        self.kd_pos = 0.
        self.km_pos = 20.
        self.umbral_pos = 0.1
        self.range_pos = 100. # Maximo pwm permitido
        self.kierr_pos = 1.2
        self.kimax_pos = 100.
        self.kisum_pos = 0.
        self.error_pos = 0.

        self.kp_vel = 1. #80
        self.ki_vel = 1. #5.
        self.kd_vel = 0.
        self.km_vel = 0.
        self.umbral_vel = 0.1
        self.range_vel = 80. # Maximo pwm permitido  ##100
        self.kierr_vel = 2
        self.kimax_vel = 25.
        self.kisum_vel = 0.
        self.error_vel = 0.
        
        # topic variables
        self.right_lec = 0
        self.right_des = 0
        self.right_ang = 0
        self.right_ang_des = 0
        self.right_vel = 0
        self.right_vel_des = 0
        self.right_out = 0

        # helper variables
        self.right_ang_tmp = 0
        self.right_ang_rng = 0
        self.right_lec_dst = 0
        self.right_ang_lst = 0
        self.right_ang_chg = 0
        self.right_ang_abs = 0
        self.right_ang_lap = 0

        self.right_offset_internal = 0.
        self.right_ang_tmp_internal = 0      
        self.right_ang_lst_internal = 0
        self.right_ang_abs_internal = 0

        self.times = 0
        self.init_time = rospy.get_time()

        self.rightResetSub = rospy.Subscriber("right_reset", Int16, self.rightResetCb)
        self.rightLecSub = rospy.Subscriber("right_lec", Int16, self.rightLecCb)
        self.rightDesSub = rospy.Subscriber("right_des", Float32, self.rightDesCb)
        self.offsetSub = rospy.Subscriber("offset", Int16, self.offsetCb)


    def offsetCb(self, data):

        if (data.data == 1):

            self.right_offset  = self.right_lec
            
            self.right_ang_tmp = 0
            self.right_ang_lst = 0
            self.right_ang_abs = 0

            self.right_ang = 0
            self.right_ang_lap =  0
            self.right_ang_lap_lst = 0


    def map(self, x, in_min, in_max, out_min, out_max):

        return (x - in_min) * (out_max - out_min + 0.) / (in_max - in_min + 0.) + out_min


    def millis(self):

        return int(1000 * (self.rospy.get_time() - self.init_time))


    def constrain(self, x, min, max):

        if (x > max):
            return max
        if (x < min):
            return min
        return x

    """
    def pid(self):

        if (abs(self.right_des - self.right_ang) < self.umbral_pos):
           self.error_pos = 0.
        else:
            self.error_pos = self.right_des - self.right_ang

        if (abs(self.right_des - self.right_ang) < self.kierr_pos):
            if (abs(self.right_des - self.right_ang) < self.umbral_pos):
                #pass
                self.kisum_pos = 0.;
            else:
                self.kisum_pos += self.ki_pos * self.error_pos
                self.kisum_pos = self.constrain(self.kisum_pos, -self.kimax_pos, self.kimax_pos)
        else:
            self.kisum_pos = 0.

        self.right_out = -self.constrain(self.kp_pos * self.error_pos + self.kisum_pos - self.kd_pos * (self.right_ang - self.right_ang_lst) + self.km_pos * self.right_des, -self.range_pos, self.range_pos)
    """


    def pid_pos(self):

        if (abs(self.right_ang_des - self.right_ang) < self.umbral_pos):
           self.error_pos = 0.
        else:
            self.error_pos = self.right_ang_des - self.right_ang
            #print "error " + str(self.error)

        if (abs(self.right_ang_des - self.right_ang) < self.kierr_pos):
            if (abs(self.right_ang_des - self.right_ang) < self.umbral_pos):
                self.kisum_pos = 0.;
            else:
                self.kisum_pos += self.ki_pos * self.error_pos
                self.kisum_pos = self.constrain(self.kisum_pos, -self.kimax_pos, self.kimax_pos)
            #print "kisum " + str(self.kisum)
        else:
            self.kisum_pos = 0.

        self.right_out = self.constrain(self.kp_pos * self.error_pos + self.kisum_pos - self.kd_pos * (self.right_ang - self.right_ang_lst) + self.km_pos * self.right_ang_des, -self.range_pos, self.range_pos)


    def pid_vel(self):

        if (abs(self.right_vel_des - self.right_vel) < self.umbral_vel):
           self.error_vel = 0.
        else:
            self.error_vel = self.right_vel_des - self.right_vel + 0.
            #print "error " + str(self.error)

        if (abs(self.right_vel_des - self.right_vel) < self.kierr_vel):
            if (abs(self.right_vel_des - self.right_vel) < self.umbral_vel):
                self.kisum_vel = 0.;
            else:
                self.kisum_vel += self.ki_vel * self.error_vel
                self.kisum_vel = self.constrain(self.kisum_vel, -self.kimax_vel, self.kimax_vel)
            #print "kisum " + str(self.kisum)
        else:
            self.kisum_vel = 0.

        self.right_out = self.constrain(self.kp_vel * self.error_vel + self.kisum_vel - self.kd_pos * (self.right_ang - self.right_ang_lst) + self.km_vel * self.right_vel_des, -self.range_vel, self.range_vel)


    def angCalc(self):

        self.times += 1

        self.right_offset_internal = self.map(self.right_offset, 0., self.right_enc_max, 0, 2 * pi)
        self.right_ang_tmp_internal = self.map(self.right_lec, 0., self.right_enc_max, 0, 2 * pi)
        self.right_ang_lst_internal = self.right_ang_abs_internal
        self.right_ang_abs_internal = self.right_ang_tmp_internal

        if (self.times > 4):
            # encuentra si el cambio fue de 0 a 2pi
            if (self.right_ang_abs_internal > 1.7 * pi and self.right_ang_lst_internal < 0.3 * pi):
                self.right_ang_lap -= 1
            # encuentra si el cambio fue de 2pi a 0
            if (self.right_ang_abs_internal < 0.3 * pi and self.right_ang_lst_internal > 1.7 * pi):
                self.right_ang_lap += 1

        self.right_ang_lap_lst = self.right_ang
        self.right_ang = 2 * pi * self.right_ang_lap + self.right_ang_abs_internal - self.right_offset_internal 
        self.right_vel = 10 * (self.right_ang - self.right_ang_lap_lst)


    """
    def angCalc(self):

        ###MAP FIRST###
        ###
        if self.right_lec < self.right_offset:
            self.right_ang_tmp = self.right_lec + 1024 - self.right_offset
        else:
            self.right_ang_tmp = self.right_lec - self.right_offset
        ###
        self.right_ang_tmp = self.right_lec        
        self.right_ang_tmp = self.map(self.right_ang_tmp, 0., 1023., 0, 2*pi)
        self.right_ang_lst = self.right_ang_abs
        self.right_ang_abs = self.right_ang_tmp
        ###MAP FIRST###

        ###LAP CALCULATE###
        # encuentra si el cambio fue de 0 a 2pi
        if (self.right_ang_abs > 1.8 * pi and self.right_ang_lst < 0.2 * pi):
            self.lap -= 1
        # encuetra si el cambio due de 2pi a 0
        if (self.right_ang_abs < 0.2 * pi and self.right_ang_lst > 1.8 * pi):
            self.lap += 1
        
        self.right_ang_lap_lst = self.right_ang
        self.right_ang = 2 * pi * self.lap + self.right_ang_abs
        ###LAP CALCULATE###

        ###MAP VEL OUT###
        self.right_vel = 10. * (self.right_ang - self.right_ang_lap_lst)
        ###MAP VEL OUT###
    """


    def rightResetCb(self, data):
        
        if (data.data == 1):
            self.right_ang_des = 0


    """
    def rightLecCb(self, data):
        self.right_lec = data.data
        self.angCalc()
        self.pid()


    def rightDesCb(self, data):
        self.right_des = self.constrain(data.data, -80, 80)
        self.angCalc()
        self.pid()


    def update(self):
        self.rightOutPub.publish(self.right_des)
        self.rightAngPub.publish(self.right_ang)
        self.rightVelPub.publish(self.right_vel)
    """


    def rightLecCb(self, data):

        self.right_lec = data.data
        self.angCalc()

        # if (abs(self.right_vel_des) < 0.1):
        #     # self.right_ang_des = self.constrain(self.right_ang_des, 0, 1000)
        #     self.pid_pos()
        #     # print "angdes " + str(self.right_ang_des)
        # else:
        #     self.right_ang_des = self.right_ang
        #     self.pid_vel()


    def rightDesCb(self, data):

        self.right_des = self.constrain(data.data, -80, 80)

        self.right_vel_des = data.data
        self.angCalc()
        self.pid_vel()

        # if (abs(self.right_vel_des) < 0.1):
        #     # self.right_ang_des = self.constrain(self.right_ang_des, 0, 1000)
        #     self.pid_pos()
        #     # print "angdes " + str(self.right_ang_des)
        # else:
        #     self.right_ang_des = self.right_ang
        #     self.pid_vel()


    def update(self):

        self.rightOutPub.publish(self.right_des)
        #self.rightOutPub.publish(self.right_out)
        self.rightAngPub.publish(self.right_ang)
        self.rightVelPub.publish(self.right_vel)


    def spin(self):

        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()


if __name__ == '__main__':

    """ main """
    right_node = Right_node()
    right_node.spin() 
