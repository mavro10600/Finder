#include "ros/ros.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sstream>
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <zbar.h>

#include "vision_msjs/Marker.h"
#include "vision_msjs/qrDetect.h"
#include "vision_msjs/imgQr.h"
#include "vision_msjs/imgQrResponse.h"

using namespace std;
using namespace zbar;
namespace enc = sensor_msgs::image_encodings;

bool SHOW_CV_WINDOW;
//SHOW_CV_WINDOW = 1;
string camera;

cv_bridge::CvImagePtr bridge;
IplImage * frame = 0;
ImageScanner scanner;
ros::Publisher code;

ros::Publisher qrFlag;
ros::ServiceServer qrsrv;
image_transport::Publisher pub;

cv::Mat cv_matrix;

uint message_sequence = 0;

bool giveImgRes(vision_msjs::imgQr::Request &req, vision_msjs::imgQr::Response &res){
  vision_msjs::imgQr::Response resp_;
  sensor_msgs::ImagePtr msg2;

  msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_matrix).toImageMsg();
  res = resp_;

  res.imgqr = *msg2;
  return true;
}

void imageReceiver(const sensor_msgs::ImageConstPtr &image) {

  //ros::NodeHandle nh;
  //image_transport::ImageTransport it(nh);
  //image_transport::Publisher pub = it.advertise("usb_cam1/QRcodeResult", 1);
  //sensor_msgs::ImagePtr res;
  vision_msjs::qrDetect fQr;


  try {
    bridge = cv_bridge::toCvCopy(image, enc::MONO8);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }  
  
  cv_matrix = bridge->image;
  int width = cv_matrix.cols; 
  int height = cv_matrix.rows;

	uchar* raw = cv_matrix.ptr<uchar>(0);
	Image scan_image(width, height, "Y800", raw, width * height);
	int n = scanner.scan(scan_image); 

  if (n < 0) {
   	ROS_ERROR("Error occured while finding barcode");
   	return;
  }
  fQr.qrDetectFlag = false;

  // extract results
  for(SymbolIterator symbol = scan_image.symbol_begin();
            symbol != scan_image.symbol_end();
            ++symbol) {
    std::stringstream ss;
  	//Publish msg on zbar topic 
	  vision_msjs::Marker msg;
	  ss << symbol->get_data();
    msg.header.seq = message_sequence++;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = image->header.frame_id;
	  msg.data = ss.str();

    int x1 = width, y1 = height, x2 = 0, y2 = 0;

    for (int i = 0; i < symbol->get_location_size(); i++) {
      x1 = MIN(x1, symbol->get_location_x(i));
      y1 = MIN(y1, symbol->get_location_y(i));
      x2 = MAX(x2, symbol->get_location_x(i));
      y2 = MAX(y2, symbol->get_location_y(i));
    }

    msg.center_x = (x1 + x2) / 2; 
    msg.center_y = (y1 + y2) / 2; 
    msg.width = (x2 - x1); 
    msg.height = (y2 + y1);

    cv::Mat cv_bgr(cv_matrix.size(), CV_8UC3);
    cvtColor(cv_matrix, cv_bgr, CV_GRAY2BGR);
    fQr.qrDetectFlag = true;

    if(1) {
      cv::rectangle(cv_matrix, cv::Point(x1,y1), cv::Point(x2,y2), cv::Scalar(255), 2);
      cv::rectangle(cv_bgr, cv::Point(x1,y1), cv::Point(x2,y2), cv::Scalar(0,255,0), 2);
    }
    qrFlag.publish(fQr);
	  code.publish(msg);
  }

  //leaqrFlag.publish(fQr);
  //Show image in CV window
  if(1) {
  //frame = new IplImage(bridge->image);
  	cv::imshow("zbar", cv_matrix);
  	cv::waitKey(1);
	//cvReleaseImage
  }

 // bridge.release();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "barcode_detector");
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);

  //Set param
  n.param("barcode_detector/show_cv_window", true);
  n.param("barcode_detector/camera/image", camera, string("/camera/rgb/image_color"));

  if(true)
  	cv::namedWindow("zbar", CV_WINDOW_AUTOSIZE); 
  
  scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1); 

  code = n.advertise<vision_msjs::Marker>("markers", 1000);
  
  ros::Subscriber sub = n.subscribe("camera/image", 10, imageReceiver);

  qrFlag = n.advertise<vision_msjs::qrDetect>("qrflag",10);

  qrsrv = n.advertiseService("qr_code_img", giveImgRes);

  ros::spin();

  return 0;
}
