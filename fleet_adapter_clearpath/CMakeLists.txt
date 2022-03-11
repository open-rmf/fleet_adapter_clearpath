cmake_minimum_required(VERSION 3.5)
project(fleet_adapter_clearpath)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rmf_fleet_adapter REQUIRED)
find_package(rmf_utils REQUIRED)
find_package(rmf_task_msgs REQUIRED)

find_package(clearpath_api REQUIRED)
find_package(command_control_msgs REQUIRED)
find_package(configuration_msgs REQUIRED)
find_package(data_msgs REQUIRED)
find_package(diagnostic_extra_msgs REQUIRED)
find_package(platform_sensor_msgs REQUIRED)
find_package(geometry_extra_msgs REQUIRED)
find_package(maintenance_msgs REQUIRED)
find_package(onboard_systems_msgs REQUIRED)
find_package(perception_navigation_msgs REQUIRED)
find_package(sensor_extra_msgs REQUIRED)
find_package(appliance_control_msgs REQUIRED)
find_package(std_extra_srvs REQUIRED)
find_package(command_control_msgs_utils REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)

include(GNUInstallDirs)

add_executable(fleet_adapter
  src/fleet_adapter_clearpath/main.cpp
  src/fleet_adapter_clearpath/ClearpathConnections.cpp
)

target_link_libraries(fleet_adapter
  PRIVATE
    rmf_fleet_adapter::rmf_fleet_adapter
    yaml-cpp
    ${rclcpp_LIBARRIES}
    ${rclcpp_action_LIBRARIES}
    ${appliance_control_msgs_LIBRARIES}
    ${command_control_msgs_LIBRARIES}
    ${configuration_msgs_LIBRARIES}
    ${data_msgs_LIBRARIES}
    ${diagnostic_extra_msgs_LIBRARIES}
    ${geometry_extra_msgs_LIBRARIES}
    ${platform_sensor_msgs_LIBRARIES}
    ${maintenance_msgs_LIBRARIES}
    ${onboard_systems_msgs_LIBRARIES}
    ${perception_navigation_msgs_LIBRARIES}
    ${sensor_extra_msgs_LIBRARIES}
    ${std_extra_srvs_LIBRARIES}
    ${command_control_msgs_utils_LIBRARIES}
    ${rmf_task_msgs_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
)

target_include_directories(fleet_adapter
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${rclcpp_INCLUDE_DIRS}
    ${rclcpp_action_INCLUDE_DIRS}
    ${clearpath_api_INCLUDE_DIRS}
    ${appliance_control_msgs_INCLUDE_DIRS}
    ${command_control_msgs_INCLUDE_DIRS}
    ${configuration_msgs_INCLUDE_DIRS}
    ${data_msgs_INCLUDE_DIRS}
    ${diagnostic_extra_msgs_INCLUDE_DIRS}
    ${geometry_extra_msgs_INCLUDE_DIRS}
    ${platform_sensor_msgs_INCLUDE_DIRS}
    ${maintenance_msgs_INCLUDE_DIRS}
    ${onboard_systems_msgs_INCLUDE_DIRS}
    ${perception_navigation_msgs_INCLUDE_DIRS}
    ${sensor_extra_msgs_INCLUDE_DIRS}
    ${std_extra_srvs_INCLUDE_DIRS}
    ${command_control_msgs_utils_INCLUDE_DIRS}
    ${rmf_task_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
)

if(BUILD_TESTING)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_file(uncrustify_config_file
    NAMES "rmf_code_style.cfg"
    PATHS "${rmf_utils_DIR}/../../../share/rmf_utils/")

  ament_uncrustify(
    include src
    CONFIG_FILE ${uncrustify_config_file}
    MAX_LINE_LENGTH 80
  )
endif()

install(
  TARGETS fleet_adapter
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
