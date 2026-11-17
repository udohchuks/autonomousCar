cmake_minimum_required(VERSION 3.8)
project(arduino_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

find_library(LIBSERIAL_LIBRARY NAMES serial REQUIRED)

include_directories(include)

set(HW_LIB arduino_diff_drive_hardware)

add_library(
  ${HW_LIB} 
  SHARED
  src/arduino_hardware_interface.cpp 
  src/wheel.cpp 
  src/arduino_comms.cpp
)

ament_target_dependencies(${HW_LIB} 
rclcpp rclcpp_lifecycle hardware_interface pluginlib
)

target_link_libraries(${HW_LIB}
  ${LIBSERIAL_LIBRARY}
)

pluginlib_export_plugin_description_file(hardware_interface arduino_diff_drive_hardware_interface.xml)

install(
  TARGETS ${HW_LIB}
  DESTINATION lib
)

install(
  DIRECTORY include
  DESTINATION include
)

ament_export_libraries(
  ${HW_LIB}
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
