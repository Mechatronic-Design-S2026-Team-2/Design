cmake_minimum_required(VERSION 3.8)
project(hexapod_nav_cpp)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(hexapod_control_interfaces REQUIRED)
find_package(hexapod_hardware_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_library(${PROJECT_NAME}_core
  src/phase_gait_generator.cpp
  src/klann_controller_core.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core Eigen3::Eigen hexapod_hardware_cpp::hexapod_hardware_cpp_core)
ament_target_dependencies(${PROJECT_NAME}_core rclcpp geometry_msgs nav_msgs hexapod_control_interfaces)

add_library(nav2_klann_controller SHARED src/nav2_klann_controller_plugin.cpp)
target_include_directories(nav2_klann_controller PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(nav2_klann_controller ${PROJECT_NAME}_core)
ament_target_dependencies(nav2_klann_controller
  rclcpp geometry_msgs nav_msgs std_msgs pluginlib nav2_core nav2_costmap_2d tf2 tf2_ros hexapod_control_interfaces)

pluginlib_export_plugin_description_file(nav2_core plugin.xml)

add_executable(klann_mppi_controller_node src/klann_mppi_controller_node.cpp)
target_include_directories(klann_mppi_controller_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(klann_mppi_controller_node ${PROJECT_NAME}_core)
ament_target_dependencies(klann_mppi_controller_node rclcpp geometry_msgs nav_msgs std_msgs hexapod_control_interfaces ament_index_cpp)

install(TARGETS
  ${PROJECT_NAME}_core
  nav2_klann_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(TARGETS
  klann_mppi_controller_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(EXPORT export_${PROJECT_NAME}
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

install(DIRECTORY include/ DESTINATION include)
install(FILES plugin.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(pluginlib nav2_core nav2_costmap_2d tf2 tf2_ros hexapod_control_interfaces hexapod_hardware_cpp Eigen3 ament_index_cpp)
ament_package()
