#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from math import *

class Left_node:

    def __init__(self, node_name_override = 'left_node'):

        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("left_node starting with name %s", self.nodename) 
        self.rate = rospy.get_param("param_global_rate", 10)

        self.leftOutPub = rospy.Publisher("left_out", Int16)
        self.leftAngPub = rospy.Publisher("left_ang", Float32)
        self.leftVelPub = rospy.Publisher("left_vel", Float32)

        
        #TODO cambiar por parametros
        self.left_enc_ana = False
        self.left_enc_max = 1023
        self.left_offset = 0
        #self.left_enc_lim = 100
        #self.left_lec_min = 485
        #self.left_lec_max = 1004
        #self.left_lec_dir = -100

        # PID control parameters
        self.kp_pos = 20.
        self.ki_pos = 0.
        self.kd_pos = 0.
        self.km_pos = 20.
        self.umbral_pos = 0.1
        self.range_pos = 100. # Maximo pwm permitido
        self.kierr_pos = 1.2
        self.kimax_pos = 100.
        self.kisum_pos = 0.
        self.error_pos = 0.

        self.kp_vel = 1.
        self.ki_vel = 1.
        self.kd_vel = 0.
        self.km_vel = 0.
        self.umbral_vel = 0.1
        self.range_vel = 80. # Maximo pwm permitido
        self.kierr_vel = 2
        self.kimax_vel = 25.
        self.kisum_vel = 0.
        self.error_vel = 0.
        
        # topic variables
        self.left_lec = 0
        self.left_des = 0
        self.left_ang = 0
        self.left_ang_des = 0
        self.left_vel = 0
        self.left_vel_des = 0
        self.left_out = 0

        # helper variables
        self.left_ang_tmp = 0
        self.left_ang_rng = 0
        self.left_lec_dst = 0
        self.left_ang_lst = 0
        self.left_ang_chg = 0
        self.left_ang_abs = 0
        self.left_ang_lap = 0

        self.left_offset_internal = 0.
        self.left_ang_tmp_internal = 0      
        self.left_ang_lst_internal = 0
        self.left_ang_abs_internal = 0

        self.times = 0
        self.init_time = rospy.get_time()

        self.leftResetSub = rospy.Subscriber("left_reset", Int16, self.leftResetCb)
        self.leftLecSub = rospy.Subscriber("left_lec", Int16, self.leftLecCb)
        self.leftDesSub = rospy.Subscriber("left_des", Float32, self.leftDesCb)
        self.offsetSub = rospy.Subscriber("offset", Int16, self.offsetCb)


    def offsetCb(self, data):

        if (data.data == 1):

            self.left_offset  = self.left_lec
            
            self.left_ang_tmp = 0
            self.left_ang_lst = 0
            self.left_ang_abs = 0

            self.left_ang = 0
            self.left_ang_lap =  0
            self.left_ang_lap_lst = 0


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

        if (abs(self.left_des - self.left_ang) < self.umbral_pos):
           self.error_pos = 0.
        else:
            self.error_pos = self.left_des - self.left_ang

        if (abs(self.left_des - self.left_ang) < self.kierr_pos):
            if (abs(self.left_des - self.left_ang) < self.umbral_pos):
                #pass
                self.kisum_pos = 0.;
            else:
                self.kisum_pos += self.ki_pos * self.error_pos
                self.kisum_pos = self.constrain(self.kisum_pos, -self.kimax_pos, self.kimax_pos)
        else:
            self.kisum_pos = 0.

        self.left_out = -self.constrain(self.kp_pos * self.error_pos + self.kisum_pos - self.kd_pos * (self.left_ang - self.left_ang_lst) + self.km_pos * self.left_des, -self.range_pos, self.range_pos)
    """


    def pid_pos(self):

        if (abs(self.left_ang_des - self.left_ang) < self.umbral_pos):
           self.error_pos = 0.
        else:
            self.error_pos = self.left_ang_des - self.left_ang
            #print "error " + str(self.error)

        if (abs(self.left_ang_des - self.left_ang) < self.kierr_pos):
            if (abs(self.left_ang_des - self.left_ang) < self.umbral_pos):
                self.kisum_pos = 0.;
            else:
                self.kisum_pos += self.ki_pos * self.error_pos
                self.kisum_pos = self.constrain(self.kisum_pos, -self.kimax_pos, self.kimax_pos)
            #print "kisum " + str(self.kisum)
        else:
            self.kisum_pos = 0.

        self.left_out = self.constrain(self.kp_pos * self.error_pos + self.kisum_pos - self.kd_pos * (self.left_ang - self.left_ang_lst) + self.km_pos * self.left_ang_des, -self.range_pos, self.range_pos)


    def pid_vel(self):

        if (abs(self.left_vel_des - self.left_vel) < self.umbral_vel):
           self.error_vel = 0.
        else:
            self.error_vel = self.left_vel_des - self.left_vel + 0.
            #print "error " + str(self.error)

        if (abs(self.left_vel_des - self.left_vel) < self.kierr_vel):
            if (abs(self.left_vel_des - self.left_vel) < self.umbral_vel):
                self.kisum_vel = 0.;
            else:
                self.kisum_vel += self.ki_vel * self.error_vel
                self.kisum_vel = self.constrain(self.kisum_vel, -self.kimax_vel, self.kimax_vel)
            #print "kisum " + str(self.kisum)
        else:
            self.kisum_vel = 0.

        self.left_out = self.constrain(self.kp_vel * self.error_vel + self.kisum_vel - self.kd_pos * (self.left_ang - self.left_ang_lst) + self.km_vel * self.left_vel_des, -self.range_vel, self.range_vel)


    def angCalc(self):

        self.times += 1

        self.left_offset_internal = self.map(self.left_offset, 0., self.left_enc_max, 2 * pi, 0)
        self.left_ang_tmp_internal = self.map(self.left_lec, 0., self.left_enc_max, 2 * pi, 0)
        self.left_ang_lst_internal = self.left_ang_abs_internal
        self.left_ang_abs_internal = self.left_ang_tmp_internal

        if (self.times > 4):
            # encuentra si el cambio fue de 0 a 2pi
            if (self.left_ang_abs_internal > 1.7 * pi and self.left_ang_lst_internal < 0.3 * pi):
                self.left_ang_lap -= 1
            # encuentra si el cambio fue de 2pi a 0
            if (self.left_ang_abs_internal < 0.3 * pi and self.left_ang_lst_internal > 1.7 * pi):
                self.left_ang_lap += 1

        self.left_ang_lap_lst = self.left_ang
        self.left_ang = 2 * pi * self.left_ang_lap + self.left_ang_abs_internal - self.left_offset_internal 
        self.left_vel = 10 * (self.left_ang - self.left_ang_lap_lst)


    """
    def angCalc(self):

        ###MAP FIRST#rosto##
        ###
        if self.left_lec < self.left_offset:
            self.left_ang_tmp = self.left_lec + 1024 - self.left_offset
        else:
            self.left_ang_tmp = self.left_lec - self.left_offset
        ###
        self.left_ang_tmp = self.left_lec        
        self.left_ang_tmp = self.map(self.left_ang_tmp, 0., 1023., 2*pi, 0.)
        self.left_ang_lst = self.left_ang_abs
        self.left_ang_abs = self.left_ang_tmp
        ###MAP FIRST###

        ###LAP CALCULATE###
        # encuentra si el cambio fue de 0 a 2pi
        if (self.left_ang_abs > 1.8 * pi and self.left_ang_lst < 0.2 * pi):
            self.lap -= 1
        # encuetra si el cambio due de 2pi a 0
        if (self.left_ang_abs < 0.2 * pi and self.left_ang_lst > 1.8 * pi):
            self.lap += 1
        
        self.left_ang_lap_lst = self.left_ang
        self.left_ang = 2 * pi * self.lap + self.left_ang_abs
        ###LAP CALCULATE###

        ###MAP VEL OUT###
        self.left_vel = 10. * (self.left_ang - self.left_ang_lap_lst)
        ###MAP VEL OUT###
    """


    def leftResetCb(self, data):
        
        if (data.data == 1):
            self.left_ang_des = 0


    """
    def leftLecCb(self, data):
        self.left_lec = data.data
        self.angCalc()
        self.pid()


    def leftDesCb(self, data):
        self.left_des = self.constrain(data.data, -80, 80)
        self.angCalc()
        self.pid()


    def update(self):
        self.leftOutPub.publish(self.left_des)
        self.leftAngPub.publish(self.left_ang)
        self.leftVelPub.publish(self.left_vel)
    """


    def leftLecCb(self, data):

        self.left_lec = data.data
        self.angCalc()

        # if (abs(self.left_vel_des) < 0.1):
        #     # self.left_ang_des = self.constrain(self.left_ang_des, 0, 1000)
        #     self.pid_pos()
        #     # print "angdes " + str(self.left_ang_des)
        # else:
        #     self.left_ang_des = self.left_ang
        #     self.pid_vel()


    def leftDesCb(self, data):

        self.left_des = self.constrain(data.data, -80, 80)

        self.left_vel_des = data.data
        self.angCalc()
        self.pid_vel()

        # if (abs(self.left_vel_des) < 0.1):
        #     # self.left_ang_des = self.constrain(self.left_ang_des, 0, 1000)
        #     self.pid_pos()
        #     # print "angdes " + str(self.left_ang_des)
        # else:
        #     self.left_ang_des = self.left_ang
        #     self.pid_vel()


    def update(self):

        #self.leftOutPub.publish(self.left_des)
        self.leftOutPub.publish(self.left_out)
        self.leftAngPub.publish(self.left_ang)
        self.leftVelPub.publish(self.left_vel)


    def spin(self):

        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()


if __name__ == '__main__':

    """ main """
    left_node = Left_node()
    left_node.spin()
