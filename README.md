# unitreeG1_action_API

Репозиторий ros2 интеграции задачи по действиям робота

Требования:

ros2 humble

moveit

unitree_ros2

## Быстрый запуск (3 терминала)

```
ros2 launch g1_moveit_desc_v3 move_group.launch.py

ros2 run inspire_hand_modbus_ros2 hand_modbus_control_node

ros2 run g1_manip_server_test1 arm_exec_action_server

```

Отправка команды:

```
ros2 action send_goal /r_arm_plan_exec g1_manip_interf_test1/action/TestServerExec "{action_code: 0}" --feedback
```


## 📋 Обзор манипуляции с кистями

Данный модуль реализует интеграцию библиотеки `dftp_hand_control` в низкоуровневый сервер манипуляции `g1_manip_server_test1` для выполнения комплексных действий вида:

Переместить руку → Выполнить жест кистью → Вернуться в безопасное состояние

## 🏗 Архитектура
```text
┌────────────────────────────────────────┐
│  g1_manip_server_test1                 │
│  ┌─────────────────────────────────┐   │
│  │ ManipulationActionServer        │   │
│  │                                 │   │
│  │ • enable()                      │   │
│  │ • move_to_pose(x,y,z,q)         │   │
│  │ • emergency()                   │   │
│  │ • execute_hand_gesture() ◄──────┼───┐
│  │ • execute_parallel_gestures()   │   │
│  │                                 │   │
│  │ • hand_controller_              │   │
│  │   (unique_ptr<HandController>)  │   │
│  └─────────────────────────────────┘   │
└────────────────┬───────────────────────┘
                 │ std::shared_ptr<rclcpp::Node>
                 ▼
┌────────────────────────────────────────┐
│  dftp_hand_control                     │
│  ┌─────────────────────────────────┐   │
│  │ HandController                  │   │
│  │                                 │   │
│  │ • executeGesture(side, gesture) │   │
│  │ • executeGestureParallel(...)   │   │
│  │ • enum Gesture                  │   │
│  │ • enum HandSide                 │   │
│  └─────────────────────────────────┘   │
└────────────────┬───────────────────────┘
                 │ ROS 2 Service Client
                 ▼
┌─────────────────────────────────────────┐
│  inspire_hand_modbus_ros2               │
│  (Modbus TCP → Inspire Hand)            │
└─────────────────────────────────────────┘
```
### Ключевые принципы
- **Инкапсуляция**: Сервер манипуляции не работает с топиками/сервисами кистей напрямую — всё абстрагировано через `HandController`.
- **Синхронность**: Все методы `execute_*` являются **блокирующими** и возвращают управление только после подтверждения от драйвера.
- **Потокобезопасность**: Вызовы жестов защищены мьютексом `state_mutex_` и проверяют флаг `enable_control_`.

---

## 📦 Зависимости

### Системные
- ROS 2 Humble Hawksbill
- C++17 или новее
- `colcon` build system

### ROS 2 пакеты
```xml
<!-- package.xml -->
<depend>dftp_hand_control</depend>
<depend>inspire_hand_modbus_ros2</depend>
<depend>service_interfaces</depend>
<depend>unitree_hg</depend>
<depend>moveit_msgs</depend>
<depend>trajectory_msgs</depend>
```

### CMake конфигурация
```cmake
# CMakeLists.txt
find_package(dftp_hand_control REQUIRED)
find_package(service_interfaces REQUIRED)

ament_target_dependencies(arm_exec_action_server
  # ... остальные зависимости ...
  dftp_hand_control
  service_interfaces
)

target_link_libraries(arm_exec_action_server
  ${unitree_hg_LIBRARIES}
  dftp_hand_control::hand_controller_lib
)
```

---

## 💻 API для разработчика

### Методы класса `ManipulationActionServer`

#### Выполнение жеста на одной кисти
```cpp
/**
 * @brief Выполняет жест на указанной кисти синхронно
 * @param side Левая или правая кисть (dftp_hand::HandSide)
 * @param gesture Тип жеста из enum dftp_hand::Gesture
 * @param stabilization_ms Задержка после выполнения для стабилизации пальцев (по умолчанию 600 мс)
 * @return true при успешном выполнении, false при ошибке
 */
bool execute_hand_gesture(
    dftp_hand::HandSide side, 
    dftp_hand::Gesture gesture, 
    int stabilization_ms = 600);
```

#### Параллельное выполнение жестов на двух кистях
```cpp
/**
 * @brief Выполняет жесты на обеих кистях одновременно
 * @param left_side Сторона левой кисти
 * @param left_gesture Жест для левой кисти
 * @param right_side Сторона правой кисти
 * @param right_gesture Жест для правой кисти
 * @param stabilization_ms Задержка стабилизации (по умолчанию 600 мс)
 * @return true при успешном выполнении обоих жестов
 */
bool execute_parallel_gestures(
    dftp_hand::HandSide left_side, dftp_hand::Gesture left_gesture,
    dftp_hand::HandSide right_side, dftp_hand::Gesture right_gesture,
    int stabilization_ms = 600);
```

### Доступные жесты (`enum Gesture`)
| Жест | Описание | Применение |
|------|----------|------------|
| `FREE_WRIST` | Разжатая кисть, свободное запястье | Исходное состояние, отпускание объекта |
| `POINT_BUTTON` | Вытянутый указательный палец | Нажатие кнопок, указание |
| `GRASP_HANDLE_PREP` | Подготовка к захвату ручки | Пре-граб дверной ручки |
| `GRASP_DOOR_HANDLE` | Захват дверной ручки | Поворот ручки двери |
| `GRASP_PASS_PREP` | Подготовка к передаче объекта | Пре-граб для передачи |
| `GRASP_PASS` | Захват для передачи | Фиксация объекта при передаче |

### Доступные стороны (`enum HandSide`)
```cpp
dftp_hand::HandSide::LEFT   // Левая кисть
dftp_hand::HandSide::RIGHT  // Правая кисть
```

---

## 🔁 Синхронизация и потокобезопасность

### Почему вызовы являются блокирующими
1. `HandController::executeGesture()` отправляет ROS 2 service request и ожидает ответа.
2. Драйвер `inspire_hand_modbus_ros2` выполняет Modbus-команду и ждёт подтверждения от кисти.
3. Только после получения `success==true` метод возвращает управление.

### Дополнительная стабилизация
Параметр `stabilization_ms` добавляет задержку после подтверждения драйвером для учёта:
- Механической инерции сервоприводов пальцев
- Времени на демпфирование вибраций
- Гарантии статичного положения перед следующей командой руки

### Пример безопасной последовательности
```cpp
void execute_complex_action() {
    // 1. Включение
    enable();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 2. Подход к точке (блокирующий)
    if (!move_to_pose(0.33, -0.31, 0.4)) return;

    // 3. Жест (блокирующий + стабилизация 800 мс)
    if (!execute_hand_gesture(
            dftp_hand::HandSide::RIGHT, 
            dftp_hand::Gesture::POINT_BUTTON, 
            800)) return;  // emergency() вызван внутри

    // 4. Движение с активным жестом (только после полной фиксации пальцев!)
    move_to_pose(0.44, -0.24, 0.4);

    // ... дальнейшие действия
}
```

---

## 🚀 Запуск

### Предварительные требования
1. Сеть: Убедитесь, что Робот имеет доступ к IP-адресам кистей по Modbus TCP.

<!-- ### Сборка проекта
```bash
cd ~/your_ws
colcon build --packages-select g1_manip_server_test1 --event-handlers console_direct+
source install/setup.bash
``` -->

### Сборка проекта
```bash
cd ~/unitreeG1_action_API-main
colcon build --packages-up-to g1_manip_server_test1
source install/setup.bash
```

### Порядок запуска (4 терминала)

**Терминал 1: Инициализация окружения и планировщик**
```bash
cd ~/g1_test_packages_ws
source install/setup.bash

# Запуск MoveIt (предоставляет сервис /plan_kinematic_path для планирования траекторий)
ros2 launch g1_moveit_desc_v3 move_group.launch.py
```

**Терминал 2: Низкоуровневый драйвер кистей**
```bash
cd ~/unitreeG1_action_API-main
source install/setup.bash
ros2 run inspire_hand_modbus_ros2 hand_modbus_control_node
```

**Терминал 3: Сервер манипуляции**
```bash
cd ~/unitreeG1_action_API-main
source install/setup.bash
ros2 run g1_manip_server_test1 arm_exec_action_server
```

**Терминал 4: Отправка комплексного действия**
```bash
cd ~/g1_test_packages_ws
source install/setup.bash

# Запуск последовательности: Подход → Жест (POINT_BUTTON) → Отход
ros2 action send_goal /r_arm_plan_exec g1_manip_interf_test1/action/TestServerExec "{action_code: 0}" --feedback
```

---

<!-- ### Логирование
Уровень логирования можно изменить для отладки:
```bash
# Подробные логи кистей
ros2 run g1_manip_server_test1 arm_exec_action_server --ros-args --log-level dftp_hand_control:=DEBUG

# Только ошибки
ros2 run g1_manip_server_test1 arm_exec_action_server --ros-args --log-level error
``` -->

---

## 📁 Структура файлов интеграции

```
g1_manip_server_test1/
├── package.xml                          # Зависимости: dftp_hand_control, service_interfaces
├── CMakeLists.txt                       # find_package + ament_target_dependencies + target_link_libraries
└── src/
    └── arm_exec_action_server.cpp       # Основной код:
                                         # • #include "dftp_hand_control/hand_controller.hpp"
                                         # • std::unique_ptr<HandController> hand_controller_
                                         # • Инициализация в конструкторе
                                         # • Методы execute_hand_gesture(), execute_parallel_gestures()
                                         # • Обновлённые execute_* действия с жестами
```

---

## 🔄 Расширение: добавление нового комплексного действия

1. Добавьте новый `action_code` в `TestServerExec.action`:
```action
# robot_interfaces/action/Manipulate.action
int32 BUTTON_PRESS_WITH_GESTURE=2
```

2. Реализуйте обработчик в `handle_accepted`:
```cpp
case 2:  // BUTTON_PRESS_WITH_GESTURE
    std::thread([this]() { execute_button_press_with_gesture(); }).detach();
    break;
```

3. Создайте метод с нужной последовательностью:
```cpp
void execute_button_press_with_gesture() {
    enable();
    move_to_pose_safe(0.33, -0.31, 0.4);
    execute_hand_gesture(dftp_hand::HandSide::RIGHT, dftp_hand::Gesture::POINT_BUTTON, 800);
    move_to_pose_safe(0.44, -0.24, 0.4);
    // ... ваша логика
    disable();
}
```

---

## 📄 Ссылки и демонстрации

- `dftp_hand_control, inspire_hand_modbus_ros2, работа кистей`: [[ссылка на гугл диск](https://drive.google.com/drive/folders/1yF2wz_ClhV5SBzENKPAhYA1ctDXb72rt?usp=drive_link)]
- `TODO`: [TODO?]

> ⚠️ **Важно**: Данный модуль требует физического подключения кистей Inspire Hand (DFTP) и корректной настройки Modbus TCP сети. Тестирование без кистей возможно только с запуском ноды драйвера.
