cmake_minimum_required(VERSION 3.0.2)
project(rbl_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  mrs_msgs
  mrs_lib
  visualization_msgs
  pcl_ros
  pcl_msgs
  pcl_conversions
  message_generation
)

find_package(Eigen3 REQUIRED)
find_package(pcl_ros REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${EIGEN_LIBRARIES})

add_service_files(
  FILES
  ActivateParams.srv
)

add_message_files(
  FILES
  PoseVelocityArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  LIBRARIES nodelet_rbl_controller 
  CATKIN_DEPENDS roscpp mrs_lib mrs_msgs geometry_msgs nav_msgs pcl_ros pcl_msgs pcl_conversions message_runtime
  DEPENDS Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${pcl_ros_INCLUDE_DIRS}
)

add_library(nodelet_rbl_controller src/nodelet.cpp src/rbl_controller.cpp src/rbl_replanner.cpp src/ciri.cpp)
add_dependencies(nodelet_rbl_controller
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(nodelet_rbl_controller
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${pcl_ros_LIBRARIES}
)

install(TARGETS nodelet_rbl_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
