cmake_minimum_required(VERSION 3.8)
project(g1_manip_server_test1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Устанавливаем ABI для совместимости с PyTorch
add_compile_definitions(_GLIBCXX_USE_CXX11_ABI=1)

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(g1_manip_interf_test1 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(dftp_hand_control REQUIRED)
find_package(service_interfaces REQUIRED)

find_package(moveit_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

find_package(unitree_hg REQUIRED)
find_package(action_msgs REQUIRED)

# Путь к PyTorch, установленному через pip (ARM64)
set(TORCH_PATH "/home/unitree/.local/lib/python3.10/site-packages/torch")
include_directories(${TORCH_PATH}/include)
include_directories(${TORCH_PATH}/include/torch/csrc/api/include)

include_directories(include include/common include/nlohmann)

add_executable(arm_exec_action_server
  src/arm_exec_action_server.cpp
  src/motor_crc_hg.cpp
)

target_link_libraries(arm_exec_action_server
  ${unitree_hg_LIBRARIES}
  dftp_hand_control::hand_controller_lib
  ${TORCH_PATH}/lib/libtorch.so
  ${TORCH_PATH}/lib/libc10.so
)

ament_target_dependencies(arm_exec_action_server
  g1_manip_interf_test1
  rclcpp
  rclcpp_action
  moveit_msgs
  trajectory_msgs
  unitree_hg
  dftp_hand_control
  service_interfaces
)

install(TARGETS arm_exec_action_server DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()