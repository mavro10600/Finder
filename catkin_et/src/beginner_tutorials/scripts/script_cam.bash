#!/bin/bash

#roscore

indexright=$(udevadm info --query=all /dev/PSEYERIGHT | grep DEVNAME | grep -oP "[0-9]*")
echo $indexright

indexleft=$(udevadm info --query=all /dev/PSEYELEFT | grep DEVNAME | grep -oP "[0-9]*")
echo $indexleft

indexpan=$(udevadm info --query=all /dev/PSEYEPAN | grep DEVNAME | grep -oP "[0-9]*")
echo $indexpan

indexfront=$(udevadm info --query=all /dev/LOGITECHCAM | grep DEVNAME | grep -oP "[0-9]*")
echo $indexfront

rosrun video_web usbCam1 $indexfront &  
rosrun video_web usbCam2 $indexright &
rosrun video_web usbCam3 $indexleft & 
sleep 1
rosrun video_web usbCam4 $indexpan &
#rosrun video_web usbCam4 $indexpan &
