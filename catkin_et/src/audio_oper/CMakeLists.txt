cmake_minimum_required(VERSION 2.8.3)
project(audio_oper)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  audio_common_msgs
  roscpp
)

find_package(PkgConfig)
pkg_check_modules(GST1.0 gstreamer-1.0 REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${GST1.0_INCLUDE_DIRS})

catkin_package()

add_executable(audio_capture_oper src/audio_capture_oper.cpp)
target_link_libraries(audio_capture_oper ${catkin_LIBRARIES} ${GST1.0_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(audio_capture_oper ${catkin_EXPORTED_TARGETS})

add_executable(audio_oper_play src/audio_oper_play.cpp)
target_link_libraries(audio_oper_play ${catkin_LIBRARIES} ${GST1.0_LIBRARIES} ${Boost_LIBRARIES}) 
add_dependencies(audio_oper_play ${catkin_EXPORTED_TARGETS})

install(TARGETS audio_oper_play 
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS audio_capture_oper
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
