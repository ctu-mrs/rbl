cmake_minimum_required(VERSION 3.8)
project(rbl_replanner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


include_directories(
  ${PCL_INCLUDE_DIRS}
)

add_library(rbl_replanner 
  src/replanner.cpp
)

add_library(rbl_replanner::rbl_replanner
  ALIAS rbl_replanner
)

target_include_directories(rbl_replanner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(rbl_replanner
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)

install(
  TARGETS rbl_replanner
  EXPORT export_rbl_replanner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

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

ament_export_targets(export_rbl_replanner HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3 PCL)

ament_package()
