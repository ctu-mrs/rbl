cmake_minimum_required(VERSION 3.8)
project(rbl_controller_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(rbl_replanner REQUIRED)
find_package(ciri REQUIRED)

# find_package(geometry_msgs REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(visualization_msgs REQUIRED)
# find_package(std_srvs REQUIRED)

# find_package(mrs_msgs REQUIRED)
# find_package(mrs_lib REQUIRED)

add_library(rbl_controller_core
  src/rbl_controller.cpp
)

target_include_directories(rbl_controller_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
)


target_link_libraries(rbl_controller_core
  PUBLIC
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    rbl_replanner::rbl_replanner
    ciri::ciri
)


# target_link_directories(rbl_controller_core
#   PUBLIC
#     ${mrs_lib_LIBRARY_DIRS}
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
