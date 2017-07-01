#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from math import *

class Base_node:

    def __init__(self, node_name_override = 'base_node'):

        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("base_node starting with name %s", self.nodename) 
        self.rate = rospy.get_param("param_global_rate", 10)

        self.baseOutPub = rospy.Publisher("base_out", Int16)
        self.baseAngPub = rospy.Publisher("base_ang", Float32)
        self.baseVelPub = rospy.Publisher("base_vel", Float32)

        #TODO cambiar por parametros
        self.base_enc_ana = False
        self.base_enc_max = 1023
        self.base_offset = 245
        #self.base_enc_lim = 100
        #self.base_lec_min = 485
        #self.base_lec_max = 1004
        #self.base_lec_dir = -100

        # PID control parameters
        self.kp_pos = 70
        self.ki_pos = 5.
        self.kd_pos = 0.
        self.km_pos = 0.
        self.umbral_pos = 0.1
        self.range_pos = 80. # Maximo pwm permitido
        self.kierr_pos = 2
        self.kimax_pos = 25.
        self.kisum_pos = 0.
        self.error_pos = 0.

        self.kp_vel = 70
        self.ki_vel = 5.
        self.kd_vel = 0.
        self.km_vel = 0.
        self.umbral_vel = 0.1
        self.range_vel = 80. # Maximo pwm permitido
        self.kierr_vel = 2
        self.kimax_vel = 25.
        self.kisum_vel = 0.
        self.error_vel = 0.

        # topic variables
        self.base_lec = 0
        self.base_des = 0
        self.base_ang = 0
        self.base_ang_des = 0
        self.base_vel = 0
        self.base_vel_des = 0
        self.base_out = 0

        # helper variables
        self.base_ang_tmp = 0
        self.base_ang_rng = 0
        self.base_lec_dst = 0
        self.base_ang_lst = 0
        self.base_ang_chg = 0
        self.base_ang_abs = 0
        self.base_ang_lap = 0

        self.base_offset_internal = 0.
        self.base_ang_tmp_internal = 0      
        self.base_ang_lst_internal = 0
        self.base_ang_abs_internal = 0

        self.times = 0
        self.times_lec = 0
        self.init_time = rospy.get_time()

        self.baseResetSub = rospy.Subscriber("base_reset", Int16, self.baseResetCb)
        self.baseLecSub = rospy.Subscriber("base_lec", Int16, self.baseLecCb)
        self.baseDesSub = rospy.Subscriber("base_des", Float32, self.baseDesCb)
        self.offsetSub = rospy.Subscriber("offset", Int16, self.offsetCb)


    def offsetCb(self, data):

        if (data.data == 1):

            self.base_offset  = self.base_lec
            
            self.base_ang_tmp = 0
            self.base_ang_lst = 0
            self.base_ang_abs = 0

            self.base_ang = 0
            self.base_ang_lap =  0
            self.base_ang_lap_lst = 0


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

        if (abs(self.base_des - self.base_ang) < self.umbral_pos):
           self.error_pos = 0.
        else:
            self.error_pos = self.base_des - self.base_ang

        if (abs(self.base_des - self.base_ang) < self.kierr_pos):
            if (abs(self.base_des - self.base_ang) < self.umbral_pos):
                #pass
                self.kisum_pos = 0.;
            else:
                self.kisum_pos += self.ki_pos * self.error_pos
                self.kisum_pos = self.constrain(self.kisum_pos, -self.kimax_pos, self.kimax_pos)
        else:
            self.kisum_pos = 0.

        self.base_out = -self.constrain(self.kp_pos * self.error_pos + self.kisum_pos - self.kd_pos * (self.base_ang - self.base_ang_lst) + self.km_pos * self.base_des, -self.range_pos, self.range_pos)
    """


    def pid_pos(self):

        if (abs(self.base_ang_des - self.base_ang) < self.umbral_pos):
           self.error_pos = 0.
        else:
            self.error_pos = self.base_ang_des - self.base_ang
            #print "error " + str(self.error)

        if (abs(self.base_ang_des - self.base_ang) < self.kierr_pos):
            if (abs(self.base_ang_des - self.base_ang) < self.umbral_pos):
                self.kisum_pos = 0.;
            else:
                self.kisum_pos += self.ki_pos * self.error_pos
                self.kisum_pos = self.constrain(self.kisum_pos, -self.kimax_pos, self.kimax_pos)
            #print "kisum " + str(self.kisum)
        else:
            self.kisum_pos = 0.

        self.base_out = self.constrain(self.kp_pos * self.error_pos + self.kisum_pos - self.kd_pos * (self.base_ang - self.base_ang_lst) + self.km_pos * self.base_ang_des, -self.range_pos, self.range_pos)


    def pid_vel(self):

        if (abs(self.base_vel_des - self.base_vel) < self.umbral_vel):
           self.error_vel = 0.
        else:
            self.error_vel = self.base_vel_des - self.base_vel + 0.
            #print "error " + str(self.error)

        if (abs(self.base_vel_des - self.base_vel) < self.kierr_vel):
            if (abs(self.base_vel_des - self.base_vel) < self.umbral_vel):
                self.kisum_vel = 0.;
            else:
                self.kisum_vel += self.ki_vel * self.error_vel
                self.kisum_vel = self.constrain(self.kisum_vel, -self.kimax_vel, self.kimax_vel)
            #print "kisum " + str(self.kisum)
        else:
            self.kisum_vel = 0.

        self.base_out = self.constrain(self.kp_vel * self.error_vel + self.kisum_vel - self.kd_pos * (self.base_ang - self.base_ang_lst) + self.km_vel * self.base_vel_des, -self.range_vel, self.range_vel)


    def angCalc(self):

        self.times += 1

        self.base_offset_internal = self.map(self.base_offset, 0., self.base_enc_max, 2 * pi, 0)
        self.base_ang_tmp_internal = self.map(self.base_lec, 0., self.base_enc_max, 2 * pi, 0)
        self.base_ang_lst_internal = self.base_ang_abs_internal
        self.base_ang_abs_internal = self.base_ang_tmp_internal

        if (self.times > 4):
            # encuentra si el cambio fue de 0 a 2pi
            if (self.base_ang_abs_internal > 1.7 * pi and self.base_ang_lst_internal < 0.3 * pi):
                self.base_ang_lap -= 1
            # encuentra si el cambio fue de 2pi a 0
            if (self.base_ang_abs_internal < 0.3 * pi and self.base_ang_lst_internal > 1.7 * pi):
                self.base_ang_lap += 1

        self.base_ang_lap_lst = self.base_ang
        self.base_ang = 2 * pi * self.base_ang_lap + self.base_ang_abs_internal - self.base_offset_internal 
        self.base_vel = 10 * (self.base_ang - self.base_ang_lap_lst)


    """
    def angCalc(self):

        self.base_ang_tmp = self.base_lec        
        self.base_ang_tmp = self.map(self.base_ang_tmp, 0., 1023., 2*pi, 0.)
        self.base_ang_lst = self.base_ang_abs
        self.base_ang_abs = self.base_ang_tmp

        # encuentra si el cambio fue de 0 a 2pi
        if (self.base_ang_abs > 1.8 * pi and self.base_ang_lst < 0.2 * pi):
            self.lap -= 1
        # encuetra si el cambio due de 2pi a 0
        if (self.base_ang_abs < 0.2 * pi and self.base_ang_lst > 1.8 * pi):
            self.lap += 1
        
        self.base_ang_lap_lst = self.base_ang
        self.base_ang = 2 * pi * self.lap + self.base_ang_abs

        self.base_vel = 10. * (self.base_ang - self.base_ang_lap_lst)
    """


    def baseResetCb(self, data):
        
        if (data.data == 1):
            self.base_ang_des = 0


    """
    def baseLecCb(self, data):
        
        self.base_lec = data.data
        self.angCalc()
        self.pid()


    def baseDesCb(self, data):

        self.base_des = data.data
        self.angCalc()
        self.pid()
    """


    def baseLecCb(self, data):

        self.times_lec += 1
        if (self.times_lec < 4):
            self.offset = data.data

        self.base_lec = data.data
        self.angCalc()

        if (abs(self.base_vel_des) < 0.1):
            # self.base_ang_des = self.constrain(self.base_ang_des, 0, 1000)
            self.pid_pos()
            # print "angdes " + str(self.base_ang_des)
        else:
            self.base_ang_des = self.base_ang
            self.pid_vel()


    def baseDesCb(self, data):

        self.base_vel_des = data.data
        self.angCalc()

        if (abs(self.base_vel_des) < 0.1):
            # self.base_ang_des = self.constrain(self.base_ang_des, 0, 1000)
            self.pid_pos()
            # print "angdes " + str(self.base_ang_des)
        else:
            self.base_ang_des = self.base_ang
            self.pid_vel()


    def update(self):

        self.baseOutPub.publish(self.base_out)
        self.baseAngPub.publish(self.base_ang)
        self.baseVelPub.publish(self.base_vel)


    def spin(self):

        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()


if __name__ == '__main__':

    """ main """
    base_node = Base_node()
    base_node.spin() 
