cmake_minimum_required(VERSION 2.8.3)
project(youbot-arm-dk-control)

add_compile_options(-std=c++11 -I /usr/include/eigen3/)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  brics_actuator
  youbot_arm_kinematics
)

#find_package(VISP REQUIRED visp_core visp_robot visp_io visp_gui)


#generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS youbot_arm_kinematics message_runtime std_msgs)

include_directories(include/youbot-arm-dk-control ${catkin_INCLUDE_DIRS})

add_executable(velocity_controller_node src/velocity_controller.cpp)
target_link_libraries(velocity_controller_node ${catkin_LIBRARIES})
add_dependencies(velocity_controller_node ${catkin_EXPORTED_TARGETS})
