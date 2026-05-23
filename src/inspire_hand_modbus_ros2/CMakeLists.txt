cmake_minimum_required(VERSION 3.8)
project(inspire_hand_modbus_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(service_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(std_msgs REQUIRED)  # std_msgs
find_package(rosidl_default_generators REQUIRED)

# modbus
find_library(MODBUS_LIBRARIES NAMES modbus REQUIRED)

# modbus
include_directories(/usr/include/modbus)

include_directories(include ${ament_INCLUDE_DIRS})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

add_executable(hand_modbus_control_node 
  src/hand_control.cpp
)


ament_target_dependencies(hand_modbus_control_node
  rclcpp
  service_interfaces
  std_msgs
)

# modbus
target_link_libraries(hand_modbus_control_node ${MODBUS_LIBRARIES})

install(TARGETS
  hand_modbus_control_node
  # hand_control_client_modbus_node
  # handcontroltopicpublisher_modbus_node
  # handcontroltopicsubscriber_modbus_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

