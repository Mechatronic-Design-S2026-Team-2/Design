cmake_minimum_required(VERSION 3.8)
project(hexapod_hardware_cpp)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(dsy_motor_msgs REQUIRED)
find_package(hexapod_control_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_library(${PROJECT_NAME}_core
  src/serial_port.cpp
  src/wt901_protocol.cpp
  src/klann_geometry.cpp
  src/lcd1602_i2c.cpp
  src/path_utils.cpp
)
set_target_properties(${PROJECT_NAME}_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core yaml-cpp Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME}_core ament_index_cpp)

add_executable(wt901_imu_node src/wt901_imu_node.cpp)
target_link_libraries(wt901_imu_node ${PROJECT_NAME}_core)
ament_target_dependencies(wt901_imu_node rclcpp sensor_msgs geometry_msgs std_msgs)

add_executable(motor_state_aggregator_node src/motor_state_aggregator_node.cpp)
target_link_libraries(motor_state_aggregator_node ${PROJECT_NAME}_core)
ament_target_dependencies(motor_state_aggregator_node rclcpp dsy_motor_msgs hexapod_control_interfaces)

add_executable(phase_command_router_node src/phase_command_router_node.cpp)
target_link_libraries(phase_command_router_node ${PROJECT_NAME}_core)
ament_target_dependencies(phase_command_router_node rclcpp dsy_motor_msgs hexapod_control_interfaces)

add_executable(body_state_estimator_node src/body_state_estimator_node.cpp)
target_link_libraries(body_state_estimator_node ${PROJECT_NAME}_core)
ament_target_dependencies(body_state_estimator_node rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs tf2 tf2_ros hexapod_control_interfaces ament_index_cpp)

add_executable(weighted_odom_fusion_node src/weighted_odom_fusion_node.cpp)
target_link_libraries(weighted_odom_fusion_node ${PROJECT_NAME}_core)
ament_target_dependencies(weighted_odom_fusion_node rclcpp sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros)

target_link_libraries(body_state_estimator_node ${PROJECT_NAME}_core)
ament_target_dependencies(body_state_estimator_node rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs tf2 tf2_ros hexapod_control_interfaces ament_index_cpp)

add_executable(lcd1602_status_node src/lcd1602_status_node.cpp)
target_link_libraries(lcd1602_status_node ${PROJECT_NAME}_core)
ament_target_dependencies(lcd1602_status_node rclcpp nav_msgs hexapod_control_interfaces)


add_executable(klann_visualizer_node src/klann_visualizer_node.cpp)
target_link_libraries(klann_visualizer_node ${PROJECT_NAME}_core)
ament_target_dependencies(klann_visualizer_node rclcpp visualization_msgs hexapod_control_interfaces dsy_motor_msgs)


add_executable(wt901_config_tool src/wt901_config_tool.cpp)
target_link_libraries(wt901_config_tool ${PROJECT_NAME}_core)
ament_target_dependencies(wt901_config_tool yaml_cpp_vendor ament_index_cpp)

add_executable(klann_lut_generator src/klann_lut_generator.cpp)
target_link_libraries(klann_lut_generator ${PROJECT_NAME}_core)
ament_target_dependencies(klann_lut_generator yaml_cpp_vendor ament_index_cpp)

install(TARGETS
  ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(TARGETS
  wt901_imu_node
  motor_state_aggregator_node
  phase_command_router_node
  body_state_estimator_node
  weighted_odom_fusion_node
  lcd1602_status_node
  klann_visualizer_node
  wt901_config_tool
  klann_lut_generator
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(EXPORT export_${PROJECT_NAME}
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(yaml_cpp_vendor Eigen3 visualization_msgs ament_index_cpp)
ament_package()
