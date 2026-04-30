#include "dftp_hand_control/hand_controller.hpp"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

namespace dftp_hand {

HandController::HandController(rclcpp::Node::SharedPtr node) : node_(node) {
    client_setpos_ = node_->create_client<service_interfaces::srv::Setpos>("/Setpos");
    client_setforce_ = node_->create_client<service_interfaces::srv::Setforce>("/Setforce");

    RCLCPP_INFO(node_->get_logger(), "Waiting for services...");
    if (!client_setpos_->wait_for_service(5s)) {
        RCLCPP_ERROR(node_->get_logger(), "Service /Setpos not available!");
    }
    if (!client_setforce_->wait_for_service(5s)) {
        RCLCPP_ERROR(node_->get_logger(), "Service /Setforce not available!");
    }
    RCLCPP_INFO(node_->get_logger(), "HandController initialized.");
}

// --- Реализация приватных хелперов ---

GestureData HandController::getGestureData(Gesture gesture) {
    std::array<int, 6> positions = {0, 0, 0, 0, 0, 0};
    std::array<int, 6> forces = {500, 500, 500, 500, 500, 500};

    switch (gesture) {
        case Gesture::POINT_BUTTON:
            positions = {2000, 2000, 2000, 0, 2000, 2000}; 
            forces = {600, 600, 600, 600, 600, 600};
            break;
        case Gesture::GRASP_HANDLE_PREP:
            positions = {600, 600, 600, 600, 550, 2000}; 
            forces = {600, 600, 600, 600, 600, 600};
            break;
        case Gesture::GRASP_DOOR_HANDLE:
            positions = {2000, 2000, 2000, 2000, 500, 2000};
            forces = {900, 900, 900, 900, 900, 900};
            break;
        case Gesture::GRASP_PASS_PREP:
            positions = {0, 0, 0, 900, 600, 2000}; 
            forces = {600, 600, 600, 600, 600, 600};
            break;
        case Gesture::GRASP_PASS:
            positions = {0, 0, 0, 1200, 850, 2000}; 
            forces = {300, 300, 300, 300, 300, 300};
            break;
        case Gesture::FUCK_YOU:
            positions = {2000, 2000, 0, 2000, 0, 0}; 
            forces = {600, 600, 600, 600, 600, 600};
            break;
        case Gesture::FREE_WRIST:
        default:
            positions = {0, 0, 0, 0, 0, 0}; 
            forces = {500, 500, 500, 500, 500, 500};
            break;
    }
    return {positions, forces};
}

std::shared_ptr<service_interfaces::srv::Setpos::Request> HandController::createPosRequest(int id, const std::array<int, 6>& pos) {
    auto request = std::make_shared<service_interfaces::srv::Setpos::Request>();
    request->id = static_cast<int16_t>(id);
    request->status = "set_pos";
    request->pos0 = pos[0]; request->pos1 = pos[1]; request->pos2 = pos[2];
    request->pos3 = pos[3]; request->pos4 = pos[4]; request->pos5 = pos[5];
    return request;
}

std::shared_ptr<service_interfaces::srv::Setforce::Request> HandController::createForceRequest(int id, const std::array<int, 6>& force) {
    auto request = std::make_shared<service_interfaces::srv::Setforce::Request>();
    request->id = static_cast<int16_t>(id);
    request->status = "set_force";
    request->force0 = force[0]; request->force1 = force[1]; request->force2 = force[2];
    request->force3 = force[3]; request->force4 = force[4]; request->force5 = force[5];
    return request;
}

void HandController::setFingerPositions(int id, const std::array<int, 6>& positions) {
    auto request = createPosRequest(id, positions);
    auto future = client_setpos_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future, 2s) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_WARN(node_->get_logger(), "Failed to set position for ID %d", id);
    }
}

void HandController::setForceThresholds(int id, const std::array<int, 6>& forces) {
    auto request = createForceRequest(id, forces);
    auto future = client_setforce_->async_send_request(request);
    rclcpp::spin_until_future_complete(node_, future, 2s);
}

// --- Публичные методы ---

void HandController::executeGesture(HandSide side, Gesture gesture) {
    int hand_id = static_cast<int>(side);
    auto data = getGestureData(gesture);
    
    setForceThresholds(hand_id, data.forces);
    std::this_thread::sleep_for(50ms); 
    setFingerPositions(hand_id, data.positions);
}

void HandController::executeGestureParallel(HandSide side1, Gesture gesture1, 
                                            HandSide side2, Gesture gesture2) {
    int id1 = static_cast<int>(side1);
    int id2 = static_cast<int>(side2);

    auto data1 = getGestureData(gesture1);
    auto data2 = getGestureData(gesture2);

    RCLCPP_INFO(node_->get_logger(), "Sending parallel commands to ID %d and ID %d...", id1, id2);

    // 1. Отправляем силу для обеих рук (асинхронно)
    auto req_f1 = createForceRequest(id1, data1.forces);
    auto req_f2 = createForceRequest(id2, data2.forces);
    
    auto fut_f1 = client_setforce_->async_send_request(req_f1);
    auto fut_f2 = client_setforce_->async_send_request(req_f2);

    // Ждем завершения установки силы
    rclcpp::spin_until_future_complete(node_, fut_f1, 1s);
    rclcpp::spin_until_future_complete(node_, fut_f2, 1s);

    std::this_thread::sleep_for(50ms);

    // 2. Отправляем позицию для обеих рук (асинхронно)
    auto req_p1 = createPosRequest(id1, data1.positions);
    auto req_p2 = createPosRequest(id2, data2.positions);

    auto fut_p1 = client_setpos_->async_send_request(req_p1);
    auto fut_p2 = client_setpos_->async_send_request(req_p2);

    // Ждем завершения движения (или отправки команды)
    rclcpp::spin_until_future_complete(node_, fut_p1, 2s);
    rclcpp::spin_until_future_complete(node_, fut_p2, 2s);
    
    RCLCPP_INFO(node_->get_logger(), "Parallel execution complete.");
}

} // namespace dftp_hand
