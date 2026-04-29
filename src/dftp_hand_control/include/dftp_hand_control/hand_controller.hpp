#ifndef DFTP_HAND_CONTROLLER_HPP
#define DFTP_HAND_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <memory>
#include <array>
#include <future>

// Подключаем типы сервисов
#include "service_interfaces/srv/setpos.hpp"
#include "service_interfaces/srv/setforce.hpp"

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
    std::array<int, 6> positions;
    std::array<int, 6> forces;
};

class HandController {
public:
    explicit HandController(rclcpp::Node::SharedPtr node);

    // 1. Синхронное выполнение (одна рука)
    void executeGesture(HandSide side, Gesture gesture);

    // 2. Асинхронное выполнение (две руки одновременно)
    void executeGestureParallel(HandSide side1, Gesture gesture1, 
                                HandSide side2, Gesture gesture2);

private:
    rclcpp::Node::SharedPtr node_;

    // Клиенты сервисов
    rclcpp::Client<service_interfaces::srv::Setpos>::SharedPtr client_setpos_;
    rclcpp::Client<service_interfaces::srv::Setforce>::SharedPtr client_setforce_;

    // --- Приватные методы ---
    
    // Установка позиций (синхронно)
    void setFingerPositions(int id, const std::array<int, 6>& positions);
    
    // Установка силы (синхронно)
    void setForceThresholds(int id, const std::array<int, 6>& forces);

    // Получение данных жеста по его типу
    GestureData getGestureData(Gesture gesture);

    // Хелперы для создания запросов
    std::shared_ptr<service_interfaces::srv::Setpos::Request> createPosRequest(int id, const std::array<int, 6>& pos);
    std::shared_ptr<service_interfaces::srv::Setforce::Request> createForceRequest(int id, const std::array<int, 6>& force);
};

} // namespace dftp_hand

#endif // DFTP_HAND_CONTROLLER_HPP
