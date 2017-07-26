cmake_minimum_required(VERSION 2.8.3)
project(zbar_detector)

find_package(catkin REQUIRED COMPONENTS image_transport cv_bridge std_msgs genmsg message_generation sensor_msgs avision_msjs)

find_package(OpenCV)

#add_message_files(FILES Marker.msg qrDetect.msg)

#add_service_files(FILES imgQr.srv)

#generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(CATKIN_DEPENDS roscpp cv_bridge image_transport sensor_msgs avision_msjs)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(barcode_detector src/main.cpp) 

add_dependencies(barcode_detector {${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(barcode_detector ${catkin_LIBRARIES} zbar ${OpenCV_LIBRARIES})

install(TARGETS barcode_detector
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


