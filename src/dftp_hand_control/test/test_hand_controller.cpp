#include <gtest/gtest.h>

#include <array>
#include <memory>
#include <thread>
#include <utility>

#include "dftp_hand_control/hand_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "service_interfaces/srv/getposact.hpp"
#include "service_interfaces/srv/setforce.hpp"
#include "service_interfaces/srv/setpos.hpp"

namespace {

class MockHandServices : public rclcpp::Node {
public:
    MockHandServices()
        : Node("mock_hand_services")
    {
        setforce_service_ = create_service<service_interfaces::srv::Setforce>(
            "/Setforce",
            [this](
                const service_interfaces::srv::Setforce::Request::SharedPtr,
                const service_interfaces::srv::Setforce::Response::SharedPtr response)
            {
                response->force_accepted = force_accepted;
            });

        setpos_service_ = create_service<service_interfaces::srv::Setpos>(
            "/Setpos",
            [this](
                const service_interfaces::srv::Setpos::Request::SharedPtr,
                const service_interfaces::srv::Setpos::Response::SharedPtr response)
            {
                response->pos_accepted = pos_accepted;
                response->success = pos_success;
            });

        getposact_service_ = create_service<service_interfaces::srv::Getposact>(
            "/Getposact",
            [this](
                const service_interfaces::srv::Getposact::Request::SharedPtr,
                const service_interfaces::srv::Getposact::Response::SharedPtr response)
            {
                for (size_t i = 0; i < response->curpos.size(); ++i) {
                    response->curpos[i] = actual_positions[i];
                }
            });
    }

    bool force_accepted = true;
    bool pos_accepted = true;
    bool pos_success = true;
    std::array<int16_t, 6> actual_positions = {157, 150, 135, 85, 246, 51};

private:
    rclcpp::Service<service_interfaces::srv::Setforce>::SharedPtr setforce_service_;
    rclcpp::Service<service_interfaces::srv::Setpos>::SharedPtr setpos_service_;
    rclcpp::Service<service_interfaces::srv::Getposact>::SharedPtr getposact_service_;
};

class MockServicesRunner {
public:
    explicit MockServicesRunner(std::shared_ptr<MockHandServices> node)
        : node_(std::move(node))
    {
        executor_.add_node(node_);
        spin_thread_ = std::thread([this]() { executor_.spin(); });
    }

    ~MockServicesRunner() {
        executor_.cancel();
        if (spin_thread_.joinable()) {
            spin_thread_.join();
        }
        executor_.remove_node(node_);
    }

private:
    std::shared_ptr<MockHandServices> node_;
    rclcpp::executors::MultiThreadedExecutor executor_;
    std::thread spin_thread_;
};

rclcpp::NodeOptions fastOptions() {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("service_wait_timeout_ms", 50),
        rclcpp::Parameter("service_call_timeout_ms", 200),
    });
    return options;
}

} // namespace

TEST(HandControllerTest, FreeWristSucceedsWithRealisticOpenPositions) {
    auto mock = std::make_shared<MockHandServices>();
    MockServicesRunner runner(mock);
    auto client_node = std::make_shared<rclcpp::Node>("hand_controller_success_test", fastOptions());
    dftp_hand::HandController controller(client_node);

    auto result = controller.executeGesture(dftp_hand::HandSide::LEFT, dftp_hand::Gesture::FREE_WRIST);

    EXPECT_TRUE(result.force_ok);
    EXPECT_TRUE(result.pos_ok);
    EXPECT_TRUE(result.actual_position_ok);
    EXPECT_TRUE(result.ok()) << result.message;
}

TEST(HandControllerTest, ReportsForceRejected) {
    auto mock = std::make_shared<MockHandServices>();
    mock->force_accepted = false;
    MockServicesRunner runner(mock);
    auto client_node = std::make_shared<rclcpp::Node>("hand_controller_force_rejected_test", fastOptions());
    dftp_hand::HandController controller(client_node);

    auto result = controller.executeGesture(dftp_hand::HandSide::LEFT, dftp_hand::Gesture::FREE_WRIST);

    EXPECT_FALSE(result.ok());
    EXPECT_FALSE(result.force_ok);
    EXPECT_TRUE(result.driver_rejected);
}

TEST(HandControllerTest, ReportsPositionRejected) {
    auto mock = std::make_shared<MockHandServices>();
    mock->pos_accepted = false;
    mock->pos_success = false;
    MockServicesRunner runner(mock);
    auto client_node = std::make_shared<rclcpp::Node>("hand_controller_pos_rejected_test", fastOptions());
    dftp_hand::HandController controller(client_node);

    auto result = controller.executeGesture(dftp_hand::HandSide::LEFT, dftp_hand::Gesture::FREE_WRIST);

    EXPECT_FALSE(result.ok());
    EXPECT_TRUE(result.force_ok);
    EXPECT_FALSE(result.pos_ok);
    EXPECT_TRUE(result.driver_rejected);
}

TEST(HandControllerTest, ReportsActualPositionTimeout) {
    auto mock = std::make_shared<MockHandServices>();
    mock->actual_positions = {1000, 1000, 1000, 1000, 1000, 1000};
    MockServicesRunner runner(mock);
    auto client_node = std::make_shared<rclcpp::Node>("hand_controller_actual_timeout_test", fastOptions());
    dftp_hand::HandController controller(client_node);

    auto result = controller.executeGesture(dftp_hand::HandSide::LEFT, dftp_hand::Gesture::FREE_WRIST);

    EXPECT_FALSE(result.ok());
    EXPECT_TRUE(result.force_ok);
    EXPECT_TRUE(result.pos_ok);
    EXPECT_FALSE(result.actual_position_ok);
}

TEST(HandControllerTest, ReportsServiceUnavailable) {
    auto client_node = std::make_shared<rclcpp::Node>("hand_controller_unavailable_test", fastOptions());
    dftp_hand::HandController controller(client_node);

    auto result = controller.executeGesture(dftp_hand::HandSide::LEFT, dftp_hand::Gesture::FREE_WRIST);

    EXPECT_FALSE(result.ok());
    EXPECT_TRUE(result.service_timeout);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
