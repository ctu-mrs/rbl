cmake_minimum_required(VERSION 3.8)
project(rbl_controller_node)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 1 == use rclcpp::Timer implementation
# 0 == use MRS's thread timer implementation
set(USE_ROS_TIMER 0)

if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[example_waypoint_flier]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options("-Wall" "-Wextra" "-Wpedantic")
endif()

if(ENABLE_COVERAGE)
  message(WARNING "building for coverage, the performance might be limited")

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs")

  add_definitions("-O1")
  add_definitions("-g")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(mrs_lib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(mrs_uav_testing REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mrs_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rbl_replanner REQUIRED)
find_package(ciri REQUIRED)
find_package(rbl_controller_core REQUIRED)
find_package(filter_reflective_uavs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PoseVelocityArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# Define the shared library created by this package
set(RBL_CONTROLLER_NODE_TARGET ${PROJECT_NAME}_component)

add_library(${RBL_CONTROLLER_NODE_TARGET} SHARED)

target_sources(${RBL_CONTROLLER_NODE_TARGET}
  PRIVATE
    "src/nodelet.cpp"
)

ament_target_dependencies(
  ${RBL_CONTROLLER_NODE_TARGET}
  rclcpp
  rclcpp_components
  mrs_lib
  octomap_msgs
  pcl_conversions
  pcl_ros
  geometry_msgs
  nav_msgs
  mrs_msgs
  std_srvs
  std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${RBL_CONTROLLER_NODE_TARGET} "${cpp_typesupport_target}")

set(FILTER_REFLECTIVE_UAVS_LINK_TARGETS)
if(TARGET filter_reflective_uavs::filter_reflective_uavs)
  list(APPEND FILTER_REFLECTIVE_UAVS_LINK_TARGETS filter_reflective_uavs::filter_reflective_uavs)
elseif(TARGET filter_reflective_uavs)
  list(APPEND FILTER_REFLECTIVE_UAVS_LINK_TARGETS filter_reflective_uavs)
elseif(DEFINED filter_reflective_uavs_TARGETS)
  list(APPEND FILTER_REFLECTIVE_UAVS_LINK_TARGETS ${filter_reflective_uavs_TARGETS})
else()
  message(FATAL_ERROR "filter_reflective_uavs was found, but it did not export a usable CMake target")
endif()

target_link_libraries(
  ${RBL_CONTROLLER_NODE_TARGET}
  Eigen3::Eigen
  rbl_replanner::rbl_replanner
  ciri::ciri
  rbl_controller_core::rbl_controller_core
  ${FILTER_REFLECTIVE_UAVS_LINK_TARGETS}
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
)

# target_link_libraries(rbl_nodelet
#   PRIVATE
#     rclcpp::rclcpp
#     rclcpp_components::component
#     mrs_lib::mrs_lib
#     Eigen3::Eigen
#     rbl_replanner::rbl_replanner
#     ciri::ciri
#     rbl_controller_core::rbl_controller_core
#     ${geometry_msgs_TARGETS}
#     ${nav_msgs_TARGETS}
#     ${mrs_msgs_TARGETS}
#     ${std_srvs_TARGETS}
#     ${PCL_LIBRARIES}
# )

# Register components defined in this library.
# rclcpp_components_register_nodes(<library> <names of components>...)
rclcpp_components_register_nodes(${RBL_CONTROLLER_NODE_TARGET}
  "rbl_controller::WrapperRosRBL"
)

## --------------------------------------------------------------
## |                           testing                          |
## --------------------------------------------------------------

if(ENABLE_TESTS)

  add_subdirectory(test)

  find_package(ament_cmake_clang_format REQUIRED)
  ament_clang_format(CONFIG_FILE "${PROJECT_SOURCE_DIR}/.clang-format")
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()

endif()

install(
  TARGETS ${RBL_CONTROLLER_NODE_TARGET}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch
  DESTINATION "share/${PROJECT_NAME}"
)

install(
  DIRECTORY config
  DESTINATION "share/${PROJECT_NAME}"
)

# install(
#   DIRECTORY tmux
#   USE_SOURCE_PERMISSIONS
#   DESTINATION "share/${PROJECT_NAME}"
# )

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
