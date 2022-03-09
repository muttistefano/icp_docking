cmake_minimum_required(VERSION 3.2)
project(icp_docking)



find_package(PCL REQUIRED COMPONENTS common io registration)


find_package (Eigen3 3.3 REQUIRED NO_MODULE)


## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  roslib
  message_generation
)

add_service_files(
  FILES
  save_pcl.srv
  move_to_pcl.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES icp_docking
#  CATKIN_DEPENDS roscpp sensor_msgs
#  DEPENDS system_lib
)





include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)



add_executable(${PROJECT_NAME}_node src/icp_docking_node.cpp)


 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${PCL_LIBRARY_DIRS}
 )
