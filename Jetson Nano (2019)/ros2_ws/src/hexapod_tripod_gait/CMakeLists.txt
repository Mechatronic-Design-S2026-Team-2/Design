cmake_minimum_required(VERSION 3.8)
project(hexapod_tripod_gait)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dsy_motor_msgs REQUIRED)
find_package(hexapod_control_interfaces REQUIRED)

add_executable(tripod_gait_node
  src/main.cpp
  src/tripod_gait_node.cpp
)

target_include_directories(tripod_gait_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(tripod_gait_node PUBLIC cxx_std_17)
ament_target_dependencies(tripod_gait_node
  rclcpp
  geometry_msgs
  sensor_msgs
  dsy_motor_msgs
  hexapod_control_interfaces
)

install(TARGETS tripod_gait_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
