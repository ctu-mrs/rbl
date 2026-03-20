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
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(mrs_lib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(mrs_uav_testing REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mrs_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rbl_replanner REQUIRED)
find_package(ciri REQUIRED)
find_package(rbl_controller_core REQUIRED)

# Define the shared library created by this package
add_library(rbl_controller_node SHARED)

target_sources(rbl_controller_node
  PRIVATE
    "src/nodelet.cpp"
)

ament_target_dependencies(
  rbl_controller_node
  rclcpp
  rclcpp_components
  mrs_lib
  octomap_msgs
  pcl_conversions
  geometry_msgs
  nav_msgs
  mrs_msgs
  std_srvs
)

target_link_libraries(
  rbl_controller_node
  Eigen3::Eigen
  rbl_replanner::rbl_replanner
  ciri::ciri
  rbl_controller_core::rbl_controller_core
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
rclcpp_components_register_nodes(rbl_controller_node
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
  TARGETS rbl_controller_node
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

ament_package()
