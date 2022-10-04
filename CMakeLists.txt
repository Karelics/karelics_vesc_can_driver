cmake_minimum_required(VERSION 3.5)
project(karelics_vesc_can_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  karelics_vesc_can_driver/vesc_can_driver.py
  karelics_vesc_can_driver/battery_status.py
  DESTINATION lib/${PROJECT_NAME}
)

set(msg_files
  "msg/VescStatus.msg"
  "msg/VescStatus2.msg"
  "msg/VescStatus3.msg"
  "msg/VescStatus4.msg"
  "msg/VescStatus5.msg"
)

set(dependencies
    rclcpp
    rclpy
    can_msgs
    sensor_msgs
    std_msgs
    std_srvs
)

ament_export_dependencies(${PROJECT_NAME}
  rclcpp, rclpy, can_msgs, sensor_msgs, std_msgs, std_srvs
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES ${dependencies}
)

if(BUILD_TESTING)
    add_subdirectory(test)
endif()

ament_package()
