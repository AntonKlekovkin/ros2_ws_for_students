cmake_minimum_required(VERSION 3.8)
project(sphero_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Зависимости ROS 2 ──
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rviz2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ── Gazebo Harmonic через vendor-пакеты ──
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim8 REQUIRED)

find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)

find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport13 REQUIRED)

find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs10 REQUIRED)

find_package(gz_math_vendor REQUIRED)
find_package(gz-math7 REQUIRED)

# ═══════════════════════════════════════════════════════════
# Plugin library
# ═══════════════════════════════════════════════════════════
add_library(sphero_plugin SHARED
  plugins/sphero_plugin.cpp
)

# Include directories для своих заголовков
target_include_directories(sphero_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Линковка с ROS 2 (modern CMake стиль)
ament_target_dependencies(sphero_plugin PUBLIC
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_geometry_msgs
)

# Линковка с Gazebo
target_link_libraries(sphero_plugin PUBLIC
  gz-sim8::gz-sim8
  gz-plugin2::register
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  gz-math7::gz-math7
)

# Установка плагина
install(TARGETS sphero_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


set_target_properties(sphero_plugin PROPERTIES
    INSTALL_RPATH_USE_LINK_PATH TRUE
)


# ═══════════════════════════════════════════════════════════
# Export
# ═══════════════════════════════════════════════════════════
ament_export_include_directories(include)
ament_export_libraries(sphero_plugin)
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

ament_package()

install(DIRECTORY launch urdf meshes src plugins worlds config scripts DESTINATION share/${PROJECT_NAME})

add_executable(platform_control src/platform_control.cpp)
ament_target_dependencies(platform_control rclcpp geometry_msgs)
install(TARGETS platform_control DESTINATION lib/${PROJECT_NAME})

add_executable(pendulum_control src/pendulum_control.cpp)
ament_target_dependencies(pendulum_control rclcpp std_msgs sensor_msgs)
install(TARGETS pendulum_control DESTINATION lib/${PROJECT_NAME})

add_executable(rotor_vel_control src/rotor_vel_control.cpp)
ament_target_dependencies(rotor_vel_control rclcpp std_msgs sensor_msgs)
install(TARGETS rotor_vel_control DESTINATION lib/${PROJECT_NAME})

add_executable(sphero_yaw_control src/sphero_yaw_control.cpp)
ament_target_dependencies(sphero_yaw_control rclcpp std_msgs sensor_msgs)
install(TARGETS sphero_yaw_control DESTINATION lib/${PROJECT_NAME})

add_executable(sphero_ang_vel_control src/sphero_ang_vel_control.cpp)
ament_target_dependencies(sphero_ang_vel_control rclcpp std_msgs sensor_msgs)
install(TARGETS sphero_ang_vel_control DESTINATION lib/${PROJECT_NAME})

add_executable(sphero_lin_vel_control src/sphero_lin_vel_control.cpp)
ament_target_dependencies(sphero_lin_vel_control rclcpp std_msgs sensor_msgs)
install(TARGETS sphero_lin_vel_control DESTINATION lib/${PROJECT_NAME})

add_executable(sphero_vw_control src/sphero_vw_control.cpp)
ament_target_dependencies(sphero_vw_control rclcpp std_msgs sensor_msgs)
install(TARGETS sphero_vw_control DESTINATION lib/${PROJECT_NAME})

add_executable(sphero_const_torque src/sphero_const_torque.cpp)
ament_target_dependencies(sphero_const_torque rclcpp std_msgs sensor_msgs)
install(TARGETS sphero_const_torque DESTINATION lib/${PROJECT_NAME})