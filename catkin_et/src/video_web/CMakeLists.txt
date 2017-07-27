cmake_minimum_required(VERSION 2.8)
project(video_web)

find_package(catkin REQUIRED cv_bridge genmsg image_transport sensor_msgs)

# add the resized image message
add_message_files(DIRECTORY msg
   FILES ResizedImage.msg
)
generate_messages(DEPENDENCIES sensor_msgs)

catkin_package()

find_package(OpenCV)

include_directories(include ${OpenCV_INCLUDE_DIRS})

# add the publisher example
add_executable(usbCam1 src/myCam1.cpp)
target_link_libraries(usbCam1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
# add the publisher example
add_executable(usbCam2 src/myCam2.cpp)
target_link_libraries(usbCam2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
# add the publisher example
add_executable(usbCam3 src/myCam3.cpp)
target_link_libraries(usbCam3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
# add the publisher example
add_executable(usbCam4 src/myCam4.cpp)
target_link_libraries(usbCam4 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
# add the publisher example
add_executable(usbCam5 src/myCam5.cpp)
target_link_libraries(usbCam5 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})



# add the publisher example
add_executable(my_publisher_2 src/my_publisher_2.cpp)
target_link_libraries(my_publisher_2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# add the subscriber example
add_executable(my_subscriber src/my_subscriber.cpp)
target_link_libraries(my_subscriber ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# add the my_subscriber_2 example
add_executable(my_subscriber_2 src/my_subscriber_2.cpp)
target_link_libraries(my_subscriber_2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


# add the plugin examples
#add_library(resized_publisher src/manifest.cpp src/resized_publisher.cpp src/resized_subscriber.cpp)
#target_link_libraries(resized_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
