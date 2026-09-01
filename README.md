# Сервер манипуляции для робота Unitree G1 (ROS2)

Репозиторий содержит ROS2-пакет `g1_manip_server_test1`, реализующий:

- Планирование движений для **обеих рук** (левой и правой) через MoveIt (декартово и суставное пространство).
- Инференс **RL-политик** (В частности, `Button_Press_UB.pt`) через интеграцию с LibTorch.
- **Управление кистями** (Inspire Hand) через библиотеку `dftp_hand_control` (жесты: `POINT_BUTTON`, `FREE_WRIST`, `GRASP_DOOR_HANDLE` и др.).
- **Динамическое обновление цели** из топика `/detected_handle/xyz` (CV-модуль).

---

## Требования

- **ROS2 Humble**
- **MoveIt 2** (с конфигурацией для G1)
- **Unitree ROS2** (`unitree_hg`, `unitree_go` и др.)
- **LibTorch** (C++ библиотека PyTorch) – для инференса нейросетей
- **dftp_hand_control** и **inspire_hand_modbus_ros2** – для управления кистями
- **colcon** (система сборки)

---

## Сборка на чистом роботе

### 1. Установите системные зависимости
```bash
sudo apt update
sudo apt install ros-humble-desktop ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers
```

### 2. Клонируйте репозиторий
```bash
mkdir -p ~/unitree_ws/src
cd ~/unitree_ws/src
git clone https://github.com/RomanSentsov/unitreeG1_action_API.git
cd ..
```

### 3. Установите недостающие пакеты
```bash
# unitree_ros2
git clone https://github.com/unitreerobotics/unitree_ros2.git src/unitree_ros2

# dftp_hand_control и inspire_hand_modbus_ros2
git clone https://github.com/your-org/dftp_hand_control.git src/dftp_hand_control
git clone https://github.com/your-org/inspire_hand_modbus_ros2.git src/inspire_hand_modbus_ros2
```

### 4. Установите LibTorch для ARM64
Скачайте сборку LibTorch для ARM64 (например, с сайта PyTorch) и распакуйте в `/home/unitree/libtorch_arm64` (или укажите другой путь, затем обновите `LD_LIBRARY_PATH`).

### 5. Соберите рабочее пространство
```bash
cd ~/unitree_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to g1_manip_server_test1 --executor sequential --parallel-workers 1
source install/setup.bash
```

---

## Запуск

Откройте **4 терминала** и выполните:

### Терминал 1: MoveIt (планировщик)
```bash
ros2 launch g1_moveit_desc_v3 move_group.launch.py
```
> Если используются кисти Inspire Hand, замените на:  
> `ros2 launch g1_moveit_desc_ftp move_group.launch.py`

### Терминал 2: Драйвер кистей (Modbus)
```bash
ros2 run inspire_hand_modbus_ros2 hand_modbus_control_node
```

### Терминал 3: Сервер манипуляции
```bash
cd ~/unitree_ws
source install/setup.bash
export LD_LIBRARY_PATH=/home/unitree/libtorch_arm64/lib:$LD_LIBRARY_PATH
ros2 run g1_manip_server_test1 arm_exec_action_server
```

### Терминал 4: Отправка команды (goal)
```bash
cd ~/unitree_ws
source install/setup.bash

# Пример: выполнить последовательность с жестами (action_code 0)
ros2 action send_goal /r_arm_plan_exec g1_manip_interf_test1/action/TestServerExec "{action_code: 0}" --feedback

# Пример: запустить RL-политику "нажатие кнопки" (action_code 1)
ros2 action send_goal /r_arm_plan_exec g1_manip_interf_test1/action/TestServerExec "{action_code: 1}"
```

---

## API для скриптования действий

**Управление состоянием**

*enable()* – включить управление руками (перейти в состояние ENABLED).

*disable()* – отключить управление (DISABLED), остановить текущую траекторию.

*emergency()* – аварийный режим: обнулить усилия, перейти в демпфирование.

**Движение руки (все методы блокирующие)**

move_to_pose_left(x, y, z, roll=0, pitch=0, yaw=0) – спланировать и выполнить декартово движение левой руки.

move_to_pose_right(x, y, z, roll=0, pitch=0, yaw=0) – аналогично для правой руки.

move_to_pose_angles_left(target_joint_angles) – движение левой руки в суставном пространстве (вектор из 7 углов в радианах).

move_to_pose_angles_right(target_joint_angles) – аналогично для правой руки.

move_to_default_pose() – перевести обе руки в дефолтную позу (DEFAULT_ANGLES_ACTIVE) через MoveIt.

soft_disable() – вернуть обе руки в HOME_LEFT/HOME_RIGHT и выключить управление.

Пример простой последовательности:

cpp
enable();
move_to_pose_left(0.3, -0.2, 0.5);   // подойти к точке
execute_hand_gesture(HandSide::LEFT, Gesture::POINT_BUTTON, 800);
move_to_pose_left(0.4, -0.2, 0.5);   // нажать
soft_disable();

**Управление кистями**

Библиотека dftp_hand_control предоставляет следующие методы:

execute_hand_gesture(HandSide side, Gesture gesture, int stabilization_ms = 600) – выполнить жест на одной кисти.

execute_parallel_gestures(HandSide left_side, Gesture left_gesture, HandSide right_side, Gesture right_gesture, int stabilization_ms = 600) – выполнить жесты на обеих кистях одновременно.

Перечисления:

HandSide: LEFT, RIGHT

Gesture:

FREE_WRIST – разжатая кисть, свободное запястье

POINT_BUTTON – вытянутый указательный палец

PREP_PREP_GRASP – подготовка к захвату (пре-граб)

GRASP_DOOR_HANDLE – захват дверной ручки

GRASP_PASS_PREP – подготовка к передаче объекта

GRASP_PASS – захват для передачи

Важно: Все вызовы блокирующие – метод возвращает управление только после подтверждения от драйвера.
Параметр stabilization_ms добавляет задержку для учёта инерции пальцев.

**Запуск RL-политик (нейроуправление)**

Action code 1 активирует режим NEURO_DRIVEN.

Последовательность выполнения:

enable()

move_to_default_pose() – переход в дефолтную позу (обе руки)

Выполняется жест POINT_BUTTON на левой кисти

Запуск NeuralActionManager с таймаутом 10 с

В цикле (каждые 2 мс) генерируются команды:

getCurrentCommand → инференс сети (раз в 20 мс) → целевые углы q_target

принудительное обнуление пояса (waist)

асинхронная проверка коллизий через /check_state_validity (с сохранением последней безопасной команды)

отправка MotorCommand в буфер

По окончании таймаута: stop(), жест FREE_WRIST, soft_disable()

Настройка политики:
Путь к модели и параметры (OBS_DIM=57, ACT_DIM=17, масштабы) заданы в коде (планируется вынести в .yaml).

**Использование CV (Computer Vision)**

Сервер подписан на топик /detected_handle/xyz (geometry_msgs/PointStamped).
Колбэк door_handle_callback:

обновляет current_handle_xyz_

вызывает neural_manager_->setGoalXYZ(current_handle_xyz_) – динамически обновляет цель для нейросети

Это позволяет политике в реальном времени корректировать движение под меняющуюся позицию кнопки/ручки.


*Unitree ROS2*