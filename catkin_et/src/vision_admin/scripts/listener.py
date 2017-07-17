#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_msjs.msg import nodesVflags
from vision_msjs.srv import imgServTest

from vision_msjs.msg import labelDetect
from vision_msjs.srv import imgLabels

from vision_msjs.msg import cPatternDetect
from vision_msjs.srv import imgCpattern

from vision_msjs.msg import qrDetect
from vision_msjs.srv import imgQr

def callback(data):
    bridge = CvBridge()
    #rospy.loginfo(rospy.get_caller_id() + "I heard QR: %r \t C: %r"%(data.qrflag,data.cflag))
    #rospy.wait_for_service('img_labels_result')
    #rospy.wait_for_service('img_cpattern_result')
    #rospy.wait_for_service('ResVision1')
    rospy.wait_for_service('qr_code_img')
    
    try:
        #res = rospy.ServiceProxy('img_cpattern_result', imgCpattern)
        #res = rospy.ServiceProxy('img_labels_result', imgLabels)
        #res = rospy.ServiceProxy('ResVision1', imgServTest)
        res = rospy.ServiceProxy('qr_code_img', imgQr)
        prevImg = res()
        #rospy.loginfo("I heard ... Lab1: %s \t Lab2: %s \t Lab3: %s \t Lab4: %s"%(prevImg.label1, prevImg.label2, prevImg.label3, prevImg.label4))
        #rospy.loginfo("I heard ... Angle in radians; %s"%(prevImg.gap_angle))
        cv_image = bridge.imgmsg_to_cv2(prevImg.imgqr, "bgr8")
        type(cv_image)
        #cv_image = bridge.imgmsg_to_cv2(prevImg.imgcpat, "bgr8")
        #cv_image = bridge.imgmsg_to_cv2(prevImg.imglab, "bgr8")
        #cv_image = bridge.imgmsg_to_cv2(prevImg.img, "bgr8")
        #cv_image = cv2.flip(cv_image,1)
        #cv2.imshow("Service_Image", cv_image)
        cv2.waitKey(1)
    except rospy.ServiceException:
        print "Not Service"

def listener():
    rospy.init_node('listener_tester', anonymous = False)

    #rospy.Subscriber("chatter", String, callback)
    #rospy.Subscriber("VisionFlagTest", nodesVflags, callback)
    
    #rospy.Subscriber("flag_labels_hazmate", labelDetect, callback)
    #rospy.Subscriber("flag_c_pattern", cPatternDetect, callback)
    rospy.Subscriber("qrflag", qrDetect, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
