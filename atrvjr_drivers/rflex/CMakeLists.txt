cmake_minimum_required(VERSION 3.8)
project(rflex)

set(PROJECTED_NAME rflex_ros2)
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_lifecycle
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  angles
  #rflex
  #atrvjr
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(angles REQUIRED)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/
  )


add_executable(${PROJECTED_NAME} src/atvrjr_node.cc src/atrvjr_driver.cc src/rflex_driver.cc)


target_include_directories(${PROJECTED_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/rflex>
  PRIVATE
  include
  
)

ament_target_dependencies(${PROJECTED_NAME} PUBLIC
 ${THIS_PACKAGE_INCLUDE_DEPENDS}
)


#add_library(atrvjr src/atrvjr_driver.cc)
#target_link_libraries(${PROJECTED_NAME} PUBLIC atrvjr)


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
