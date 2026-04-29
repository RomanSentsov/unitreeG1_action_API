cmake_minimum_required(VERSION 3.8)
project(g1_manip_server_test1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# интерфейс нашего сервера
find_package(g1_manip_interf_test1 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

# dftp hand control
find_package(dftp_hand_control REQUIRED)
find_package(service_interfaces REQUIRED)

# Структура приходящих запросов основана на этих пакетах:
find_package(moveit_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# msg файлы, которые мы отправляем как команды в топик /lowcmd и читаем из топика /lowstate, описаны в этом пакете,
# созданном unitree. Мы берем его из unitree_ros2
find_package(unitree_hg REQUIRED)

find_package(action_msgs REQUIRED)

include_directories(include include/common include/nlohmann)


# Включаем в исполняемые файлы src/motor_crc_hg.cpp, так как оттуда используется функция в нашей основной программе.
add_executable(arm_exec_action_server
  src/arm_exec_action_server.cpp
  src/motor_crc_hg.cpp
)


# Не нужно, забыл убрать. Ничего не ломает
target_link_libraries(arm_exec_action_server
  ${unitree_hg_LIBRARIES}
  dftp_hand_control::hand_controller_lib
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

install(TARGETS
  arm_exec_action_server
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
