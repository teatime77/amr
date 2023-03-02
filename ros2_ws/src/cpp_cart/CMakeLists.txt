cmake_minimum_required(VERSION 3.8)
project(cpp_cart)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cart_interfaces REQUIRED)
find_package(Qt5Widgets REQUIRED)

add_executable(talker src/publisher.cpp src/cart_qt.cpp)
ament_target_dependencies(talker rclcpp tf2 tf2_geometry_msgs sensor_msgs nav_msgs cart_interfaces Qt5Widgets)

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})

ament_package()
