cmake_minimum_required(VERSION 3.8)
project(cyberdog_tracking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(protocol REQUIRED)
find_package(nav2_util REQUIRED)
find_package(OpenCV 4 REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(cyberdog_tracking
  src/main.cpp
  src/transform.cpp
  src/distance_filter.cpp
  src/object_tracking.cpp
)
ament_target_dependencies(cyberdog_tracking
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  cv_bridge
  tf2_ros
  protocol
  nav2_util
  OpenCV
)

add_executable(sub
  src/subscribe.cpp
)
ament_target_dependencies(sub
  rclcpp
  sensor_msgs
)

add_executable(set_object
  src/set_object.cpp
)
ament_target_dependencies(set_object
  rclcpp
  sensor_msgs
  protocol
)

install(TARGETS
  cyberdog_tracking
  sub
  set_object
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
