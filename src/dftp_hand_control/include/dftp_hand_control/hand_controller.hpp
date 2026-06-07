#ifndef DFTP_HAND_CONTROLLER_HPP
#define DFTP_HAND_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <memory>
#include <array>
#include <future>
#include <chrono>
#include <cstdint>

// Подключаем типы сервисов
#include "service_interfaces/srv/setpos.hpp"
#include "service_interfaces/srv/setforce.hpp"
#include "service_interfaces/srv/getposact.hpp"

namespace dftp_hand {

// Enum для выбора руки
enum class HandSide {
    LEFT = 1,
    RIGHT = 2
};

// Enum для жестов
enum class Gesture {
    FREE_WRIST,           // Исходное
    POINT_BUTTON,         // Нажатие кнопки
    GRASP_HANDLE_PREP,    // Подготовка к ручке
    GRASP_DOOR_HANDLE,    // Сжатие ручки
    GRASP_PASS_PREP,      // Подготовка к пропуску
    GRASP_PASS,            // Сжатие пропуска
    FUCK_YOU
};

// Структура данных жеста (объявлена здесь один раз)
struct GestureData {
    std::array<int, 6> command_positions;
    std::array<int, 6> expected_actual;
    std::array<int, 6> tolerance;
    std::array<int, 6> forces;
    int wait_timeout_ms = 1500;
    int poll_interval_ms = 50;
    int settle_ms = 100;
};

struct GestureResult {
    bool force_ok = false;
    bool pos_ok = false;
    bool service_timeout = false;
    bool driver_rejected = false;
    bool actual_position_ok = false;
    std::array<int16_t, 6> actual_positions = {-1, -1, -1, -1, -1, -1};
    std::string message;

    bool ok() const {
        return force_ok && pos_ok && !service_timeout && !driver_rejected && actual_position_ok;
    }
};

struct ParallelGestureResult {
    GestureResult first;
    GestureResult second;

    bool ok() const {
        return first.ok() && second.ok();
    }
};

class HandController {
public:
    explicit HandController(rclcpp::Node::SharedPtr node);

    // 1. Синхронное выполнение (одна рука)
    GestureResult executeGesture(HandSide side, Gesture gesture);

    // 2. Асинхронное выполнение (две руки одновременно)
    ParallelGestureResult executeGestureParallel(HandSide side1, Gesture gesture1,
                                                 HandSide side2, Gesture gesture2);

private:
    rclcpp::Node::SharedPtr node_;

    // Клиенты сервисов
    rclcpp::Client<service_interfaces::srv::Setpos>::SharedPtr client_setpos_;
    rclcpp::Client<service_interfaces::srv::Setforce>::SharedPtr client_setforce_;
    rclcpp::Client<service_interfaces::srv::Getposact>::SharedPtr client_getposact_;

    std::string setpos_service_name_;
    std::string setforce_service_name_;
    std::string getposact_service_name_;
    std::chrono::milliseconds service_wait_timeout_;
    std::chrono::milliseconds service_call_timeout_;

    // --- Приватные методы ---
    
    // Установка позиций (синхронно)
    bool setFingerPositions(int id, const std::array<int, 6>& positions, GestureResult& result);
    
    // Установка силы (синхронно)
    bool setForceThresholds(int id, const std::array<int, 6>& forces, GestureResult& result);

    bool waitForActualPosition(int id, const GestureData& data, GestureResult& result);
    bool readActualPositions(int id, GestureResult& result);
    bool isActualPositionAccepted(const GestureData& data, const std::array<int16_t, 6>& actual) const;

    // Получение данных жеста по его типу
    GestureData getGestureData(Gesture gesture);
    int getIntParam(const std::string& name, int default_value);
    std::string getStringParam(const std::string& name, const std::string& default_value);

    // Хелперы для создания запросов
    std::shared_ptr<service_interfaces::srv::Setpos::Request> createPosRequest(int id, const std::array<int, 6>& pos);
    std::shared_ptr<service_interfaces::srv::Setforce::Request> createForceRequest(int id, const std::array<int, 6>& force);
    std::shared_ptr<service_interfaces::srv::Getposact::Request> createGetPosActRequest(int id);
};

} // namespace dftp_hand

#endif // DFTP_HAND_CONTROLLER_HPP
