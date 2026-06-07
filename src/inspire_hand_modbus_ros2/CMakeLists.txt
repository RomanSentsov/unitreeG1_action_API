cmake_minimum_required(VERSION 3.8)
project(inspire_hand_modbus_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(service_interfaces REQUIRED)

# 查找 modbus 库
find_library(MODBUS_LIBRARIES NAMES modbus REQUIRED)

# 添加 modbus 的头文件路径
include_directories(/usr/include/modbus)

include_directories(include ${ament_INCLUDE_DIRS})

add_executable(hand_modbus_control_node 
  src/hand_control.cpp
)

ament_target_dependencies(hand_modbus_control_node
  rclcpp
  service_interfaces
)

target_link_libraries(hand_modbus_control_node ${MODBUS_LIBRARIES})

install(TARGETS
  hand_modbus_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
