cmake_minimum_required(VERSION 3.8)
project(lynxarm_srvcli)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
#find_package(serial_driver REQUIRED)
#find_package(example_interfaces REQUIRED)
find_package(lynxarm_interface REQUIRED)

#include_directories(${PROJECT_NAME} /opt/ros/humble/include/serial_driver)


add_executable(server src/HighLevelDriverServer.cpp)
ament_target_dependencies(server
  rclcpp lynxarm_interface rclcpp_action)                      # CHANGE

add_executable(client src/UserClient.cpp)
ament_target_dependencies(client
  rclcpp lynxarm_interface)                      # CHANGE

install(TARGETS
  server
  client
  DESTINATION lib/${PROJECT_NAME})


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
