cmake_minimum_required(VERSION 3.8)
project(dftp_hand_control)

# Стандарт C++14 или 17 (для ROS2 Humble рекомендуется 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- 1. Поиск зависимостей ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# Самое важное: находим твой пакет с сервисами
find_package(service_interfaces REQUIRED) 

# --- 2. Создание библиотеки (логика управления) ---
add_library(hand_controller_lib
  src/hand_controller.cpp
)

# Указываем пути к заголовочным файлам (.hpp)
target_include_directories(hand_controller_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Линкуем библиотеку с ROS2 и твоими сервисами
ament_target_dependencies(hand_controller_lib
  rclcpp
  service_interfaces
)

# --- 3. Создание исполняемого файла (тестовая нода) ---
add_executable(test_gestures_node src/test_gestures.cpp)

# Связываем исполняемый файл с нашей библиотекой
target_link_libraries(test_gestures_node hand_controller_lib)

# --- 4. Установка (Install) ---
# Чтобы команды ros2 run видели твои ноды после сборки

# Устанавливаем заголовки
install(DIRECTORY include/
  DESTINATION include/
)

# Устанавливаем библиотеку
install(TARGETS hand_controller_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Устанавливаем исполняемые файлы в lib/dftp_hand_control/
install(TARGETS test_gestures_node
  DESTINATION lib/${PROJECT_NAME}
)

# --- 5. Экспорт зависимостей ---
# Это критически важно! Если другой пакет будет зависеть от dftp_hand_control,
# он должен автоматически знать про service_interfaces и rclcpp.
ament_export_include_directories(include)
ament_export_libraries(hand_controller_lib)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rclcpp service_interfaces)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(test_hand_controller test/test_hand_controller.cpp)
  if(TARGET test_hand_controller)
    target_link_libraries(test_hand_controller hand_controller_lib)
    ament_target_dependencies(test_hand_controller
      rclcpp
      service_interfaces
    )
  endif()

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
