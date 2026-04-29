#include "rclcpp/rclcpp.hpp"
#include "dftp_hand_control/hand_controller.hpp"
#include <chrono>
#include <thread>
#include <iostream>

using namespace std::chrono_literals;

void wait_and_log(rclcpp::Logger logger, const std::string& msg, int seconds) {
    RCLCPP_INFO(logger, "%s (Ждем %d сек...)", msg.c_str(), seconds);
    std::this_thread::sleep_for(std::chrono::seconds(seconds));
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("test_gestures_node");
    
    // Инициализируем контроллер
    dftp_hand::HandController hand_ctrl(node);

    RCLCPP_INFO(node->get_logger(), "=== ЗАПУСК СИНХРОННОГО ТЕСТА ВСЕХ ЖЕСТОВ ===");
    RCLCPP_INFO(node->get_logger(), "Обе руки будут выполнять одинаковые действия одновременно.");

    // --- 0. Исходное положение ---
    RCLCPP_INFO(node->get_logger(), "--- 0. FREE_WRIST (Исходное) ---");
    hand_ctrl.executeGestureParallel(
        dftp_hand::HandSide::LEFT, dftp_hand::Gesture::FREE_WRIST,
        dftp_hand::HandSide::RIGHT, dftp_hand::Gesture::FREE_WRIST
    );
    wait_and_log(node->get_logger(), "Руки в исходном положении", 2);

    // --- 1. Кнопка (POINT_BUTTON) ---
    // Примечание: В твоем enum нет POINT_BUTTON_PREP, поэтому сразу переходим к нажатию.
    // Если захочешь добавить подготовку, нужно будет расширить enum.
    RCLCPP_INFO(node->get_logger(), "--- 1. POINT_BUTTON (Нажатие кнопки) ---");
    hand_ctrl.executeGestureParallel(
        dftp_hand::HandSide::LEFT, dftp_hand::Gesture::POINT_BUTTON,
        dftp_hand::HandSide::RIGHT, dftp_hand::Gesture::POINT_BUTTON
    );
    wait_and_log(node->get_logger(), "Кнопка нажата (указательные выпрямлены)", 3);
    
    // Возврат в исходное перед следующим действием
    hand_ctrl.executeGestureParallel(
        dftp_hand::HandSide::LEFT, dftp_hand::Gesture::FREE_WRIST,
        dftp_hand::HandSide::RIGHT, dftp_hand::Gesture::FREE_WRIST
    );
    wait_and_log(node->get_logger(), "Сброс", 2);

    // --- 2. Дверная ручка (С подготовкой) ---
    
    // Шаг А: Подготовка (GRASP_HANDLE_PREP)
    RCLCPP_INFO(node->get_logger(), "--- 2a. GRASP_HANDLE_PREP (Подготовка к ручке) ---");
    hand_ctrl.executeGestureParallel(
        dftp_hand::HandSide::LEFT, dftp_hand::Gesture::GRASP_HANDLE_PREP,
        dftp_hand::HandSide::RIGHT, dftp_hand::Gesture::GRASP_HANDLE_PREP
    );
    wait_and_log(node->get_logger(), "Руки готовы к обхвату (широкий ковш)", 3);

    // Шаг Б: Схват (GRASP_DOOR_HANDLE)
    RCLCPP_INFO(node->get_logger(), "--- 2b. GRASP_DOOR_HANDLE (Сжатие ручки) ---");
    hand_ctrl.executeGestureParallel(
        dftp_hand::HandSide::LEFT, dftp_hand::Gesture::GRASP_DOOR_HANDLE,
        dftp_hand::HandSide::RIGHT, dftp_hand::Gesture::GRASP_DOOR_HANDLE
    );
    wait_and_log(node->get_logger(), "Ручка схвачена", 3);

    // Возврат в исходное
    hand_ctrl.executeGestureParallel(
        dftp_hand::HandSide::LEFT, dftp_hand::Gesture::FREE_WRIST,
        dftp_hand::HandSide::RIGHT, dftp_hand::Gesture::FREE_WRIST
    );
    wait_and_log(node->get_logger(), "Сброс", 2);

    // --- 3. Пропуск (С подготовкой) ---

    // Шаг А: Подготовка (GRASP_PASS_PREP)
    RCLCPP_INFO(node->get_logger(), "--- 3a. GRASP_PASS_PREP (Подготовка к пропуску) ---");
    hand_ctrl.executeGestureParallel(
        dftp_hand::HandSide::LEFT, dftp_hand::Gesture::GRASP_PASS_PREP,
        dftp_hand::HandSide::RIGHT, dftp_hand::Gesture::GRASP_PASS_PREP
    );
    wait_and_log(node->get_logger(), "Клешня открыта, ждет карту", 3);

    // Шаг Б: Схват (GRASP_PASS)
    RCLCPP_INFO(node->get_logger(), "--- 3b. GRASP_PASS (Сжатие пропуска) ---");
    hand_ctrl.executeGestureParallel(
        dftp_hand::HandSide::LEFT, dftp_hand::Gesture::GRASP_PASS,
        dftp_hand::HandSide::RIGHT, dftp_hand::Gesture::GRASP_PASS
    );
    wait_and_log(node->get_logger(), "Пропуск схвачен аккуратно", 3);

    // Шаг O: иди нафиг
    RCLCPP_INFO(node->get_logger(), "--- 3o (Нехороший жест) ---");
    hand_ctrl.executeGestureParallel(
        dftp_hand::HandSide::LEFT, dftp_hand::Gesture::FUCK_YOU,
        dftp_hand::HandSide::RIGHT, dftp_hand::Gesture::FUCK_YOU
    );
    wait_and_log(node->get_logger(), "Пропуск схвачен аккуратно", 3);

    // --- Финальный сброс ---
    RCLCPP_INFO(node->get_logger(), "--- Финальный сброс ---");
    hand_ctrl.executeGestureParallel(
        dftp_hand::HandSide::LEFT, dftp_hand::Gesture::FREE_WRIST,
        dftp_hand::HandSide::RIGHT, dftp_hand::Gesture::FREE_WRIST
    );

    RCLCPP_INFO(node->get_logger(), "=== ТЕСТ УСПЕШНО ЗАВЕРШЕН ===");

    rclcpp::shutdown();
    return 0;
}
