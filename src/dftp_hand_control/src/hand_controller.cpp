#include "dftp_hand_control/hand_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

namespace dftp_hand {
namespace {

void appendMessage(GestureResult& result, const std::string& message) {
    if (result.message.empty()) {
        result.message = message;
    } else {
        result.message += "; " + message;
    }
}

template <typename T, size_t N>
std::string arrayToString(const std::array<T, N>& values) {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0) {
            oss << ", ";
        }
        oss << static_cast<int>(values[i]);
    }
    oss << "]";
    return oss.str();
}

} // namespace

HandController::HandController(rclcpp::Node::SharedPtr node) : node_(node) {
    setpos_service_name_ = getStringParam("setpos_service", "/Setpos");
    setforce_service_name_ = getStringParam("setforce_service", "/Setforce");
    getposact_service_name_ = getStringParam("getposact_service", "/Getposact");
    service_wait_timeout_ = std::chrono::milliseconds(getIntParam("service_wait_timeout_ms", 500));
    service_call_timeout_ = std::chrono::milliseconds(getIntParam("service_call_timeout_ms", 2000));

    client_setpos_ = node_->create_client<service_interfaces::srv::Setpos>(setpos_service_name_);
    client_setforce_ = node_->create_client<service_interfaces::srv::Setforce>(setforce_service_name_);
    client_getposact_ = node_->create_client<service_interfaces::srv::Getposact>(getposact_service_name_);

    RCLCPP_INFO(node_->get_logger(), "Waiting for hand services...");
    if (!client_setpos_->wait_for_service(service_wait_timeout_)) {
        RCLCPP_WARN(node_->get_logger(), "Service %s is not available yet.", setpos_service_name_.c_str());
    }
    if (!client_setforce_->wait_for_service(service_wait_timeout_)) {
        RCLCPP_WARN(node_->get_logger(), "Service %s is not available yet.", setforce_service_name_.c_str());
    }
    if (!client_getposact_->wait_for_service(service_wait_timeout_)) {
        RCLCPP_WARN(node_->get_logger(), "Service %s is not available yet.", getposact_service_name_.c_str());
    }
    RCLCPP_INFO(node_->get_logger(), "HandController initialized.");
}

std::string HandController::getStringParam(const std::string& name, const std::string& default_value) {
    if (!node_->has_parameter(name)) {
        return node_->declare_parameter<std::string>(name, default_value);
    }
    return node_->get_parameter(name).as_string();
}

int HandController::getIntParam(const std::string& name, int default_value) {
    if (!node_->has_parameter(name)) {
        return node_->declare_parameter<int>(name, default_value);
    }
    return static_cast<int>(node_->get_parameter(name).as_int());
}

GestureData HandController::getGestureData(Gesture gesture) {
    GestureData data;
    data.command_positions = {0, 0, 0, 0, 0, 0};
    data.expected_actual = data.command_positions;
    data.tolerance = {200, 200, 200, 200, 200, 200};
    data.forces = {500, 500, 500, 500, 500, 500};

    switch (gesture) {
        case Gesture::POINT_BUTTON:
            data.command_positions = {2000, 2000, 2000, 0, 2000, 2000};
            data.expected_actual = data.command_positions;
            data.forces = {600, 600, 600, 600, 600, 600};
            break;
        case Gesture::GRASP_HANDLE_PREP:
            data.command_positions = {600, 600, 600, 600, 550, 2000};
            data.expected_actual = data.command_positions;
            data.forces = {600, 600, 600, 600, 600, 600};
            break;
        case Gesture::GRASP_DOOR_HANDLE:
            data.command_positions = {2000, 2000, 2000, 2000, 500, 2000};
            data.expected_actual = data.command_positions;
            data.forces = {900, 900, 900, 900, 900, 900};
            break;
        case Gesture::GRASP_PASS_PREP:
            data.command_positions = {0, 0, 0, 900, 600, 2000};
            data.expected_actual = data.command_positions;
            data.forces = {600, 600, 600, 600, 600, 600};
            break;
        case Gesture::GRASP_PASS:
            data.command_positions = {0, 0, 0, 1200, 850, 2000};
            data.expected_actual = data.command_positions;
            data.forces = {300, 300, 300, 300, 300, 300};
            break;
        case Gesture::FUCK_YOU:
            data.command_positions = {2000, 2000, 0, 2000, 0, 0};
            data.expected_actual = data.command_positions;
            data.forces = {600, 600, 600, 600, 600, 600};
            break;
        case Gesture::FREE_WRIST:
        default:
            data.command_positions = {0, 0, 0, 0, 0, 0};
            data.expected_actual = {157, 150, 135, 85, 246, 51};
            data.tolerance = {120, 120, 120, 100, 150, 80};
            data.forces = {500, 500, 500, 500, 500, 500};
            break;
    }

    return data;
}

std::shared_ptr<service_interfaces::srv::Setpos::Request> HandController::createPosRequest(
    int id,
    const std::array<int, 6>& pos)
{
    auto request = std::make_shared<service_interfaces::srv::Setpos::Request>();
    request->id = static_cast<int16_t>(id);
    request->status = "set_pos";
    request->pos0 = pos[0];
    request->pos1 = pos[1];
    request->pos2 = pos[2];
    request->pos3 = pos[3];
    request->pos4 = pos[4];
    request->pos5 = pos[5];
    return request;
}

std::shared_ptr<service_interfaces::srv::Setforce::Request> HandController::createForceRequest(
    int id,
    const std::array<int, 6>& force)
{
    auto request = std::make_shared<service_interfaces::srv::Setforce::Request>();
    request->id = static_cast<int16_t>(id);
    request->status = "set_force";
    request->force0 = force[0];
    request->force1 = force[1];
    request->force2 = force[2];
    request->force3 = force[3];
    request->force4 = force[4];
    request->force5 = force[5];
    return request;
}

std::shared_ptr<service_interfaces::srv::Getposact::Request> HandController::createGetPosActRequest(int id) {
    auto request = std::make_shared<service_interfaces::srv::Getposact::Request>();
    request->id = static_cast<int16_t>(id);
    request->status = "get_posact";
    return request;
}

bool HandController::setForceThresholds(
    int id,
    const std::array<int, 6>& forces,
    GestureResult& result)
{
    if (!client_setforce_->wait_for_service(service_wait_timeout_)) {
        result.service_timeout = true;
        appendMessage(result, "Setforce service is unavailable");
        return false;
    }

    auto future = client_setforce_->async_send_request(createForceRequest(id, forces));
    if (rclcpp::spin_until_future_complete(node_, future, service_call_timeout_) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        result.service_timeout = true;
        appendMessage(result, "Setforce service timed out");
        return false;
    }

    auto response = future.get();
    result.force_ok = response->force_accepted;
    if (!result.force_ok) {
        result.driver_rejected = true;
        appendMessage(result, "Setforce was rejected by driver");
    }
    return result.force_ok;
}

bool HandController::setFingerPositions(
    int id,
    const std::array<int, 6>& positions,
    GestureResult& result)
{
    if (!client_setpos_->wait_for_service(service_wait_timeout_)) {
        result.service_timeout = true;
        appendMessage(result, "Setpos service is unavailable");
        return false;
    }

    auto future = client_setpos_->async_send_request(createPosRequest(id, positions));
    if (rclcpp::spin_until_future_complete(node_, future, service_call_timeout_) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        result.service_timeout = true;
        appendMessage(result, "Setpos service timed out");
        return false;
    }

    auto response = future.get();
    result.pos_ok = response->pos_accepted && response->success;
    if (!result.pos_ok) {
        result.driver_rejected = true;
        appendMessage(result, "Setpos was rejected by driver");
    }
    return result.pos_ok;
}

bool HandController::readActualPositions(int id, GestureResult& result) {
    if (!client_getposact_->wait_for_service(service_wait_timeout_)) {
        result.service_timeout = true;
        appendMessage(result, "Getposact service is unavailable");
        return false;
    }

    auto future = client_getposact_->async_send_request(createGetPosActRequest(id));
    if (rclcpp::spin_until_future_complete(node_, future, service_call_timeout_) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        result.service_timeout = true;
        appendMessage(result, "Getposact service timed out");
        return false;
    }

    auto response = future.get();
    bool valid = true;
    for (size_t i = 0; i < result.actual_positions.size(); ++i) {
        result.actual_positions[i] = response->curpos[i];
        if (response->curpos[i] < 0) {
            valid = false;
        }
    }

    if (!valid) {
        appendMessage(result, "Getposact returned invalid finger positions");
    }
    return valid;
}

bool HandController::isActualPositionAccepted(
    const GestureData& data,
    const std::array<int16_t, 6>& actual) const
{
    for (size_t i = 0; i < actual.size(); ++i) {
        const int error = std::abs(static_cast<int>(actual[i]) - data.expected_actual[i]);
        if (error > data.tolerance[i]) {
            return false;
        }
    }
    return true;
}

bool HandController::waitForActualPosition(int id, const GestureData& data, GestureResult& result) {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(data.wait_timeout_ms);
    const auto poll_interval = std::chrono::milliseconds(std::max(1, data.poll_interval_ms));

    GestureResult last_read;
    while (rclcpp::ok() && std::chrono::steady_clock::now() <= deadline) {
        GestureResult read_result;
        if (readActualPositions(id, read_result)) {
            result.actual_positions = read_result.actual_positions;
            last_read = read_result;
            RCLCPP_DEBUG(
                node_->get_logger(),
                "Getposact id=%d actual=%s expected=%s tolerance=%s",
                id,
                arrayToString(result.actual_positions).c_str(),
                arrayToString(data.expected_actual).c_str(),
                arrayToString(data.tolerance).c_str());
            if (isActualPositionAccepted(data, result.actual_positions)) {
                result.actual_position_ok = true;
                if (data.settle_ms > 0) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(data.settle_ms));
                }
                return true;
            }
        } else {
            last_read = read_result;
        }

        std::this_thread::sleep_for(poll_interval);
    }

    result.actual_positions = last_read.actual_positions;
    if (last_read.service_timeout) {
        result.service_timeout = true;
    }
    result.actual_position_ok = false;

    std::ostringstream oss;
    oss << "Actual hand position did not reach expected values";
    appendMessage(result, oss.str());
    return false;
}

GestureResult HandController::executeGesture(HandSide side, Gesture gesture) {
    GestureResult result;
    const int hand_id = static_cast<int>(side);
    const auto data = getGestureData(gesture);

    if (!setForceThresholds(hand_id, data.forces, result)) {
        return result;
    }

    std::this_thread::sleep_for(50ms);

    if (!setFingerPositions(hand_id, data.command_positions, result)) {
        return result;
    }

    waitForActualPosition(hand_id, data, result);
    return result;
}

ParallelGestureResult HandController::executeGestureParallel(
    HandSide side1,
    Gesture gesture1,
    HandSide side2,
    Gesture gesture2)
{
    ParallelGestureResult result;
    const int id1 = static_cast<int>(side1);
    const int id2 = static_cast<int>(side2);
    const auto data1 = getGestureData(gesture1);
    const auto data2 = getGestureData(gesture2);

    RCLCPP_INFO(node_->get_logger(), "Sending parallel commands to ID %d and ID %d...", id1, id2);

    if (!client_setforce_->wait_for_service(service_wait_timeout_)) {
        result.first.service_timeout = true;
        result.second.service_timeout = true;
        appendMessage(result.first, "Setforce service is unavailable");
        appendMessage(result.second, "Setforce service is unavailable");
        return result;
    }

    auto fut_f1 = client_setforce_->async_send_request(createForceRequest(id1, data1.forces));
    auto fut_f2 = client_setforce_->async_send_request(createForceRequest(id2, data2.forces));

    auto wait_force = [this](auto& future, GestureResult& single_result) {
        if (rclcpp::spin_until_future_complete(node_, future, service_call_timeout_) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            single_result.service_timeout = true;
            appendMessage(single_result, "Setforce service timed out");
            return;
        }
        auto response = future.get();
        single_result.force_ok = response->force_accepted;
        if (!single_result.force_ok) {
            single_result.driver_rejected = true;
            appendMessage(single_result, "Setforce was rejected by driver");
        }
    };

    wait_force(fut_f1, result.first);
    wait_force(fut_f2, result.second);

    std::this_thread::sleep_for(50ms);

    if (!client_setpos_->wait_for_service(service_wait_timeout_)) {
        result.first.service_timeout = true;
        result.second.service_timeout = true;
        appendMessage(result.first, "Setpos service is unavailable");
        appendMessage(result.second, "Setpos service is unavailable");
        return result;
    }

    if (result.first.force_ok && result.second.force_ok) {
        auto fut_p1 = client_setpos_->async_send_request(createPosRequest(id1, data1.command_positions));
        auto fut_p2 = client_setpos_->async_send_request(createPosRequest(id2, data2.command_positions));

        auto wait_pos = [this](auto& future, GestureResult& single_result) {
            if (rclcpp::spin_until_future_complete(node_, future, service_call_timeout_) !=
                rclcpp::FutureReturnCode::SUCCESS)
            {
                single_result.service_timeout = true;
                appendMessage(single_result, "Setpos service timed out");
                return;
            }
            auto response = future.get();
            single_result.pos_ok = response->pos_accepted && response->success;
            if (!single_result.pos_ok) {
                single_result.driver_rejected = true;
                appendMessage(single_result, "Setpos was rejected by driver");
            }
        };

        wait_pos(fut_p1, result.first);
        wait_pos(fut_p2, result.second);
    } else {
        if (result.first.force_ok) {
            setFingerPositions(id1, data1.command_positions, result.first);
        }
        if (result.second.force_ok) {
            setFingerPositions(id2, data2.command_positions, result.second);
        }
    }

    if (result.first.pos_ok) {
        waitForActualPosition(id1, data1, result.first);
    }
    if (result.second.pos_ok) {
        waitForActualPosition(id2, data2, result.second);
    }

    RCLCPP_INFO(node_->get_logger(), "Parallel execution complete.");
    return result;
}

} // namespace dftp_hand
