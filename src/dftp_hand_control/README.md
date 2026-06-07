# dftp_hand_control

`dftp_hand_control` - это C++-обертка для высокоуровневых жестов кистей
Inspire/DFTP. Пакет сам не ходит в Modbus. Он вызывает ROS 2 сервисы драйвера
`inspire_hand_modbus_ros2`, а драйвер уже пишет и читает регистры физической
кисти.

## Общая схема

```text
g1_manip_server_test1
  -> dftp_hand::HandController
    -> /Setforce, /Setpos, /Getposact
      -> inspire_hand_modbus_ros2/hand_modbus_control_node
        -> Modbus TCP
          -> физическая кисть
```

ID кистей задается прямо в service request:

```text
HandSide::LEFT  -> id = 1
HandSide::RIGHT -> id = 2
```

## Что запускать

Сначала нужен Modbus-драйвер кистей:

```bash
ros2 run inspire_hand_modbus_ros2 hand_modbus_control_node
```

При необходимости IP можно переопределить параметрами:

```bash
ros2 run inspire_hand_modbus_ros2 hand_modbus_control_node --ros-args \
  -p left_ip:=192.168.123.210 \
  -p right_ip:=192.168.123.211 \
  -p modbus_port:=6000
```

После этого можно запускать сервер манипуляции. Он создает `HandController`
внутри себя и дергает жесты по мере выполнения action-сценариев.

Для ручной проверки есть тестовая нода:

```bash
ros2 run dftp_hand_control test_gestures_node
```

`test_gestures_node` прогоняет набор жестов для обеих кистей. Это именно
демо/ручная проверка, а не безопасный production-сценарий: перед запуском на
роботе стоит посмотреть список жестов в `src/test_gestures.cpp`.

## Поведение при отсутствующих кистях

Драйвер кистей больше не завершает работу, если левая, правая или обе кисти не
подключены. Он оставляет сервисы живыми, но запросы к отсутствующей кисти
отклоняются.

Типовые логи при старте драйвера:

```text
Initializing Modbus connections...
Left Hand IP: 192.168.123.210
Right Hand IP: 192.168.123.211
Connected to Left Hand.
Connected to Right Hand.
Services started. Use id=1 for Left, id=2 for Right.
```

Если кисти нет:

```text
Left hand is not connected. Requests with id=1 will be rejected.
Right hand is not connected. Requests with id=2 will be rejected.
No hands connected. Services stay alive and will reject hand commands.
```

При конкретном запросе к отсутствующей кисти драйвер пишет throttled warning:

```text
Left hand (ID 1) is disconnected!
Right hand (ID 2) is disconnected!
```

Отсутствие кисти не должно останавливать сервер манипуляции. В текущей логике
жест будет считаться неуспешным, но action-сценарий может продолжить работу,
если вызывающий код решил, что это допустимо.

## Поддержанные сервисы драйвера

Текущий `hand_modbus_control_node` реально реализует:

```text
/Setforce
/Setpos
/Setspeed
/Getposact
```

`HandController` использует только:

```text
/Setforce
/Setpos
/Getposact
```

Остальные service-типы в `service_interfaces` считаются reserved/vendor API,
пока в драйвере нет callbacks с реальной Modbus-логикой.

Диапазоны, которые сейчас проверяет драйвер:

```text
Setpos:   0..2000
Setforce: 0..3000
Setspeed: 0..1000
```

## Где описаны жесты

Жесты сейчас описаны в C++, в функции:

```text
src/dftp_hand_control/src/hand_controller.cpp
dftp_hand::HandController::getGestureData(...)
```

YAML-файла с жестами сейчас нет и он не используется. Это сделано намеренно:
на текущем этапе проще держать значения рядом с логикой `HandController`, пока
не откалиброваны реальные `POS_ACT` для всех жестов.

Поддержанные enum-жесты лежат в:

```text
src/dftp_hand_control/include/dftp_hand_control/hand_controller.hpp
```

## Логика выполнения одного жеста

`executeGesture(side, gesture)` блокирующий. Он возвращается только после
успеха, ошибки или timeout.

Последовательность такая:

```text
1. Отправить /Setforce.
2. Проверить response->force_accepted.
3. Подождать 50 ms.
4. Отправить /Setpos.
5. Проверить response->pos_accepted && response->success.
6. Читать /Getposact, пока actual не попадет в expected_actual +- tolerance.
7. После успеха подождать settle_ms.
8. Вернуть GestureResult.
```

То есть принятого `/Setpos` недостаточно. Жест считается успешным только если
драйвер принял force/position команды и фактическая позиция по `/Getposact`
дошла до ожидаемых значений.

## Параметры жеста

Каждый жест задается структурой `GestureData`:

```cpp
struct GestureData {
    std::array<int, 6> command_positions;
    std::array<int, 6> expected_actual;
    std::array<int, 6> tolerance;
    std::array<int, 6> forces;
    int wait_timeout_ms = 1500;
    int poll_interval_ms = 50;
    int settle_ms = 100;
};
```

### command_positions

Значения, которые отправляются в `/Setpos`:

```text
Setpos.pos0..pos5 = command_positions
```

Это команда для кисти, а не показания датчиков.

### expected_actual

Значения, которые ожидаются от `/Getposact` после движения:

```text
Getposact.curpos ~= expected_actual
```

`expected_actual` может отличаться от `command_positions`. Например, для
`FREE_WRIST` мы командуем все пальцы в `0`, но реально открытая кисть сейчас
читает примерно такие значения:

```cpp
command_positions = {0, 0, 0, 0, 0, 0};
expected_actual   = {157, 150, 135, 85, 246, 51};
```

Это нормальная ситуация: команда и фактическое значение датчика не обязаны быть
одинаковыми.

### tolerance

Допустимая абсолютная ошибка для каждого пальца:

```text
abs(actual[i] - expected_actual[i]) <= tolerance[i]
```

Режима `max_threshold` сейчас нет. Используется только `tolerance`.

Для большинства жестов `expected_actual` пока равен `command_positions`, а
`tolerance` стоит около `200`. Эти значения нужно будет подогнать по реальному
роботу, как уже сделано для `FREE_WRIST`.

### forces

Значения, которые перед жестом отправляются в `/Setforce`:

```text
Setforce.force0..force5 = forces
```

Смысл текущей логики: сначала настроить force threshold, потом отправить
позицию.

### wait_timeout_ms

Максимальное время ожидания, пока `/Getposact` попадет в `expected_actual` с
учетом `tolerance`.

Текущее значение по умолчанию:

```text
1500 ms
```

Если кисть физически двигается медленно или фактические значения плохо
откалиброваны, жест будет ждать до этого timeout и потом вернет
`actual_position_ok = false`.

### poll_interval_ms

Интервал между чтениями `/Getposact` во время проверки позиции.

Текущее значение по умолчанию:

```text
50 ms
```

При `wait_timeout_ms = 1500` и `poll_interval_ms = 50` контроллер пытается
прочитать позицию примерно до 30 раз, если сервисы отвечают быстро.

### settle_ms

Небольшая пауза после того, как позиция уже попала в tolerance.

Текущее значение по умолчанию:

```text
100 ms
```

Это не основное время движения. Основное ожидание идет через polling
`/Getposact`.

## ROS-параметры HandController

Значения жестов хардкодятся в C++, но имена сервисов и service timeouts можно
переопределить параметрами node, на котором создается `HandController`.

Значения по умолчанию:

```text
setpos_service: /Setpos
setforce_service: /Setforce
getposact_service: /Getposact
service_wait_timeout_ms: 500
service_call_timeout_ms: 2000
```

`service_wait_timeout_ms` используется при ожидании доступности сервиса.

`service_call_timeout_ms` используется при ожидании ответа на один service call.

Важный нюанс: `wait_timeout_ms` ограничивает общее ожидание попадания позиции в
tolerance, но каждый отдельный service call может ждать до
`service_call_timeout_ms`. При проблемах связи фактическая задержка может быть
больше, чем просто `1500 ms`.

## GestureResult

`executeGesture()` возвращает:

```cpp
struct GestureResult {
    bool force_ok;
    bool pos_ok;
    bool service_timeout;
    bool driver_rejected;
    bool actual_position_ok;
    std::array<int16_t, 6> actual_positions;
    std::string message;

    bool ok() const;
};
```

`ok()` возвращает `true` только если:

```text
force_ok == true
pos_ok == true
service_timeout == false
driver_rejected == false
actual_position_ok == true
```

Типовые причины ошибки:

```text
Setforce service is unavailable
Setforce service timed out
Setforce was rejected by driver
Setpos service is unavailable
Setpos service timed out
Setpos was rejected by driver
Getposact service is unavailable
Getposact service timed out
Getposact returned invalid finger positions
Actual hand position did not reach expected values
```

## Логи при выполнении жестов

`HandController` при создании пишет:

```text
Waiting for hand services...
Service /Setpos is not available yet.
Service /Setforce is not available yet.
Service /Getposact is not available yet.
HandController initialized.
```

Warnings по сервисам появляются только если сервис не успел стать доступным за
`service_wait_timeout_ms`. Это не останавливает создание контроллера: следующая
попытка будет уже во время самого жеста.

Сервер манипуляции пишет краткий INFO на успешный жест:

```text
Executing gesture POINT_BUTTON on RIGHT hand...
Gesture POINT_BUTTON on RIGHT hand completed: actual=[...]
```

При ошибке:

```text
Gesture failed on RIGHT hand: Actual hand position did not reach expected values
```

Подробный polling `/Getposact` выведен только на DEBUG:

```text
Getposact id=2 actual=[...] expected=[...] tolerance=[...]
```

Чтобы увидеть DEBUG-логи, можно запускать node с уровнем debug, например:

```bash
ros2 run g1_manip_server_test1 arm_exec_action_server --ros-args --log-level debug
```

## Поведение в сервере манипуляции

В `g1_manip_server_test1` есть две обертки:

```cpp
bool execute_hand_gesture(HandSide side, Gesture gesture);
bool execute_parallel_gestures(...);
```

Они:

```text
1. Коротко проверяют состояние сервера под state_mutex_.
2. Отпускают mutex.
3. Вызывают blocking executeGesture / executeGestureParallel.
4. Логируют success/failure.
5. Возвращают bool вызывающему сценарию.
```

Долгий `spin_until_future_complete`, polling `/Getposact` и `sleep_for` не
держат `state_mutex_`.

Текущие сценарии, где сервер использует кисти:

```text
action_code = 0 -> button press
action_code = 2 -> neural action с .pt-политикой
```

### Button press

В сценарии button press правая кисть делает:

```text
RIGHT POINT_BUTTON
RIGHT FREE_WRIST
```

Если жест не сработал, сервер пишет warning и продолжает сценарий. Сейчас ошибка
кисти не abort-ит action и не вызывает `soft_disable()` сама по себе.

### Neural action

В сценарии нейроуправления левая кисть готовится перед запуском политики:

```text
enable()
подготовка дефолтной позы
LEFT POINT_BUTTON
neural_manager_->start(...)
...
LEFT FREE_WRIST
soft_disable()
```

`executeGesture()` блокирующий, поэтому `.pt`-политика стартует только после
того, как подготовительный жест завершился, упал или дошел до timeout. Если
жест неуспешен, сервер пишет warning и все равно запускает нейрополитику.

## Параллельные жесты

`executeGestureParallel(...)` предназначен для одновременной команды двум
кистям. Он отправляет force-запросы для обеих кистей, затем position-запросы
для обеих кистей, а после этого проверяет фактические позиции.

Это полезно для тестов обеих кистей, но важно понимать ограничение: внутри все
равно используются ROS service calls и `spin_until_future_complete()` на том же
node. Это не полноценный независимый realtime-поток управления каждой кистью.

## Тонкие места

1. Проверка успеха идет только по `/Getposact`.

   Мы пока не проверяем фактическую силу, ток, error/status регистры и наличие
   удерживаемого объекта. Поэтому `actual_position_ok = true` означает только
   "позиция похожа на ожидаемую".

2. Значения `expected_actual` откалиброваны не для всех жестов.

   Для `FREE_WRIST` уже используются реальные показания открытой кисти. Для
   остальных жестов пока в основном стоит `expected_actual = command_positions`,
   поэтому их нужно подгонять на роботе.

3. `HandController` сам вызывает `rclcpp::spin_until_future_complete(node_, ...)`.

   Сейчас это нормально, потому что `hand_control_proxy` node в сервере
   манипуляции не добавлен в другой executor. Если в будущем этот node добавить
   в общий executor, `HandController` лучше переписать на модель без внутреннего
   spin.

4. Modbus-драйвер защищает операции per-hand mutex.

   Это не дает двум service callback одновременно писать/читать один и тот же
   `modbus_t*` для одной кисти. Левая и правая кисти имеют разные mutex и могут
   обслуживаться независимо.

5. При плохой связи задержки могут быть больше ожидаемых.

   У драйвера Modbus timeout сейчас 500 ms. `/Getposact` читает 6 регистров
   последовательно, поэтому при проблемной связи один запрос может быть
   заметно дольше обычного.

## Тесты

Для `HandController` добавлены unit tests с mock-сервисами:

```bash
colcon test --packages-select dftp_hand_control
```

Тестируются:

```text
успешный FREE_WRIST с реалистичными POS_ACT
отклоненный Setforce
отклоненный Setpos
timeout по фактической позиции
недоступные сервисы
```

Тесты нужно запускать на машине с установленным ROS 2 Humble и зависимостями
workspace.

## Как калибровать жесты

1. Задать `command_positions` для нужного жеста.
2. Запустить драйвер и выполнить жест.
3. Считать реальные `POS_ACT` по всем 6 пальцам.
4. Записать эти значения в `expected_actual`.
5. Подобрать `tolerance`, чтобы жест проходил стабильно, но не принимал явно
   неправильное положение.
6. Проверить несколько повторов на левой и правой кисти отдельно.

Для `FREE_WRIST` уже использован такой подход:

```text
команда:  [0, 0, 0, 0, 0, 0]
POS_ACT:  [157, 150, 135, 85, 246, 51]
```
