cmake_minimum_required(VERSION 3.5)
project(cart_pole_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(controller src/controller.cpp)
ament_target_dependencies(controller rclcpp std_msgs sensor_msgs geometry_msgs)

install(TARGETS
  controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()

