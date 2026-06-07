#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <chrono>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <modbus/modbus.h>

// Подключаем все необходимые сервисы
#include "service_interfaces/srv/getangleact.hpp"
#include "service_interfaces/srv/getangleset.hpp"
#include "service_interfaces/srv/getposact.hpp"
#include "service_interfaces/srv/getposset.hpp"
#include "service_interfaces/srv/getspeedset.hpp"
#include "service_interfaces/srv/getforceact.hpp"
#include "service_interfaces/srv/getforceset.hpp"
#include "service_interfaces/srv/getcurrentact.hpp"
#include "service_interfaces/srv/gettemp.hpp"
#include "service_interfaces/srv/geterror.hpp"
#include "service_interfaces/srv/getstatus.hpp"
#include "service_interfaces/srv/setangle.hpp"
#include "service_interfaces/srv/setpos.hpp"
#include "service_interfaces/srv/setspeed.hpp"
#include "service_interfaces/srv/setforce.hpp"
#include "service_interfaces/srv/setforceclb.hpp"
#include "service_interfaces/srv/setclearerror.hpp"
#include "service_interfaces/srv/setcurrentlimit.hpp"
#include "service_interfaces/srv/setdefaultforce.hpp"
#include "service_interfaces/srv/setdefaultspeed.hpp"
#include "service_interfaces/srv/setid.hpp"
#include "service_interfaces/srv/setreduratio.hpp"
#include "service_interfaces/srv/setresetpara.hpp"
#include "service_interfaces/srv/setsaveflash.hpp"
#include "service_interfaces/srv/setgestureno.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class HandControl : public rclcpp::Node
{
public:
    HandControl()
        : Node("hand_modbus_control_node"), 
          ctx_left_(nullptr), 
          ctx_right_(nullptr),
          left_connected_(false),
          right_connected_(false)
    {
        // 1. Получаем IP адреса из параметров ROS
        this->declare_parameter<std::string>("left_ip", "192.168.123.210");
        this->declare_parameter<std::string>("right_ip", "192.168.123.211");
        this->declare_parameter<int>("modbus_port", 6000);

        std::string ip_left = this->get_parameter("left_ip").as_string();
        std::string ip_right = this->get_parameter("right_ip").as_string();
        int port = this->get_parameter("modbus_port").as_int();

        RCLCPP_INFO(this->get_logger(), "Initializing Modbus connections...");
        RCLCPP_INFO(this->get_logger(), "Left Hand IP: %s", ip_left.c_str());
        RCLCPP_INFO(this->get_logger(), "Right Hand IP: %s", ip_right.c_str());

        // 2. Инициализация левой руки (ID 1)
        ctx_left_ = modbus_new_tcp(ip_left.c_str(), port);
        if (ctx_left_) {
            modbus_set_response_timeout(ctx_left_, 0, 500000); // 500ms timeout
            if (modbus_connect(ctx_left_) == -1) {
                RCLCPP_WARN(this->get_logger(), "Could not connect to Left Hand (%s): %s", ip_left.c_str(), modbus_strerror(errno));
                modbus_free(ctx_left_);
                ctx_left_ = nullptr;
            } else {
                left_connected_ = true;
                RCLCPP_INFO(this->get_logger(), "Connected to Left Hand.");
            }
        }

        // 3. Инициализация правой руки (ID 2)
        ctx_right_ = modbus_new_tcp(ip_right.c_str(), port);
        if (ctx_right_) {
            modbus_set_response_timeout(ctx_right_, 0, 500000); // 500ms timeout
            if (modbus_connect(ctx_right_) == -1) {
                RCLCPP_WARN(this->get_logger(), "Could not connect to Right Hand (%s): %s", ip_right.c_str(), modbus_strerror(errno));
                modbus_free(ctx_right_);
                ctx_right_ = nullptr;
            } else {
                right_connected_ = true;
                RCLCPP_INFO(this->get_logger(), "Connected to Right Hand.");
            }
        }

        if (!left_connected_) {
            RCLCPP_WARN(this->get_logger(), "Left hand is not connected. Requests with id=1 will be rejected.");
        }
        if (!right_connected_) {
            RCLCPP_WARN(this->get_logger(), "Right hand is not connected. Requests with id=2 will be rejected.");
        }
        if (!left_connected_ && !right_connected_) {
            RCLCPP_WARN(this->get_logger(), "No hands connected. Services stay alive and will reject hand commands.");
        }

        // 4. Регистрация сервисов
        // Имена сервисов остаются прежними. Выбор руки идет через request->id
        
        Setpos_Server = this->create_service<service_interfaces::srv::Setpos>("Setpos",
            std::bind(&HandControl::setpos_callback, this, _1, _2));
        
        Setforce_Server = this->create_service<service_interfaces::srv::Setforce>("Setforce",
            std::bind(&HandControl::setforce_callback, this, _1, _2));

        Setspeed_Server = this->create_service<service_interfaces::srv::Setspeed>("Setspeed",
            std::bind(&HandControl::setspeed_callback, this, _1, _2));

        Getposact_Server = this->create_service<service_interfaces::srv::Getposact>("Getposact",
            std::bind(&HandControl::getposact_callback, this, _1, _2));
            
        // ... Регистрируем остальные сервисы по аналогии ...
        // Для краткости я оставил основные. Если нужны другие, добавь их здесь по образцу.
        
        RCLCPP_INFO(this->get_logger(), "Services started. Use id=1 for Left, id=2 for Right.");
    }

    ~HandControl() {
        if (ctx_left_) {
            modbus_close(ctx_left_);
            modbus_free(ctx_left_);
        }
        if (ctx_right_) {
            modbus_close(ctx_right_);
            modbus_free(ctx_right_);
        }
    }

private:
    modbus_t *ctx_left_;
    modbus_t *ctx_right_;
    bool left_connected_;
    bool right_connected_;
    std::mutex left_ctx_mutex_;
    std::mutex right_ctx_mutex_;

    // Сервисы
    rclcpp::Service<service_interfaces::srv::Setpos>::SharedPtr Setpos_Server;
    rclcpp::Service<service_interfaces::srv::Setforce>::SharedPtr Setforce_Server;
    rclcpp::Service<service_interfaces::srv::Setspeed>::SharedPtr Setspeed_Server;
    rclcpp::Service<service_interfaces::srv::Getposact>::SharedPtr Getposact_Server;
    // ... объяви остальные сервисы если будешь их реализовывать ...

    struct HandContext {
        modbus_t* ctx;
        std::mutex* mutex;
    };

    // --- Вспомогательная функция выбора контекста ---
    HandContext get_ctx(int id) {
        if (id == 1) {
            if (!left_connected_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Left hand (ID 1) is disconnected!");
                return {nullptr, nullptr};
            }
            return {ctx_left_, &left_ctx_mutex_};
        } else if (id == 2) {
            if (!right_connected_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Right hand (ID 2) is disconnected!");
                return {nullptr, nullptr};
            }
            return {ctx_right_, &right_ctx_mutex_};
        } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid Hand ID: %d. Use 1 (Left) or 2 (Right).", id);
            return {nullptr, nullptr};
        }
    }

    // --- Реализация Callbacks ---

    void setpos_callback(const service_interfaces::srv::Setpos::Request::SharedPtr request,
                         const service_interfaces::srv::Setpos::Response::SharedPtr response)
    {
        HandContext hand = get_ctx(request->id);
        if (!hand.ctx) {
            response->pos_accepted = false;
            response->success = false;
            return;
        }

        // Проверка диапазона
        if ((request->pos0 >= 0 && request->pos0 <= 2000) &&
            (request->pos1 >= 0 && request->pos1 <= 2000) &&
            (request->pos2 >= 0 && request->pos2 <= 2000) &&
            (request->pos3 >= 0 && request->pos3 <= 2000) &&
            (request->pos4 >= 0 && request->pos4 <= 2000) &&
            (request->pos5 >= 0 && request->pos5 <= 2000))
        {
            uint16_t positions[6] = {
                static_cast<uint16_t>(request->pos0),
                static_cast<uint16_t>(request->pos1),
                static_cast<uint16_t>(request->pos2),
                static_cast<uint16_t>(request->pos3),
                static_cast<uint16_t>(request->pos4),
                static_cast<uint16_t>(request->pos5)
            };

            std::lock_guard<std::mutex> lock(*hand.mutex);
            int rc = modbus_write_registers(hand.ctx, 1474, 6, positions);
            // modbus_write_registers возвращает количество записанных регистров при успехе
            response->pos_accepted = (rc == 6);
            response->success = (rc == 6);
            
            if (!response->pos_accepted) {
                RCLCPP_ERROR(this->get_logger(), "Modbus write error (ID %d): %s", request->id, modbus_strerror(errno));
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Hand ID %d: Position out of range!", request->id);
            response->pos_accepted = false;
            response->success = false;
        }
    }

    void setforce_callback(const service_interfaces::srv::Setforce::Request::SharedPtr request,
                           const service_interfaces::srv::Setforce::Response::SharedPtr response)
    {
        HandContext hand = get_ctx(request->id);
        if (!hand.ctx) {
            response->force_accepted = false;
            return;
        }

        if ((request->force0 >= 0 && request->force0 <= 3000) &&
            (request->force1 >= 0 && request->force1 <= 3000) &&
            (request->force2 >= 0 && request->force2 <= 3000) &&
            (request->force3 >= 0 && request->force3 <= 3000) &&
            (request->force4 >= 0 && request->force4 <= 3000) &&
            (request->force5 >= 0 && request->force5 <= 3000))
        {
            uint16_t forces[6] = {
                static_cast<uint16_t>(request->force0),
                static_cast<uint16_t>(request->force1),
                static_cast<uint16_t>(request->force2),
                static_cast<uint16_t>(request->force3),
                static_cast<uint16_t>(request->force4),
                static_cast<uint16_t>(request->force5)
            };

            std::lock_guard<std::mutex> lock(*hand.mutex);
            int rc = modbus_write_registers(hand.ctx, 1498, 6, forces);
            response->force_accepted = (rc == 6);
             if (!response->force_accepted) {
                RCLCPP_ERROR(this->get_logger(), "Modbus write force error (ID %d): %s", request->id, modbus_strerror(errno));
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Hand ID %d: Force out of range!", request->id);
            response->force_accepted = false;
        }
    }

    void setspeed_callback(const service_interfaces::srv::Setspeed::Request::SharedPtr request,
                           const service_interfaces::srv::Setspeed::Response::SharedPtr response)
    {
        HandContext hand = get_ctx(request->id);
        if (!hand.ctx) {
            response->speed_accepted = false;
            return;
        }

        if ((request->speed0 >= 0 && request->speed0 <= 1000) &&
            (request->speed1 >= 0 && request->speed1 <= 1000) &&
            (request->speed2 >= 0 && request->speed2 <= 1000) &&
            (request->speed3 >= 0 && request->speed3 <= 1000) &&
            (request->speed4 >= 0 && request->speed4 <= 1000) &&
            (request->speed5 >= 0 && request->speed5 <= 1000))
        {
            uint16_t speeds[6] = {
                static_cast<uint16_t>(request->speed0),
                static_cast<uint16_t>(request->speed1),
                static_cast<uint16_t>(request->speed2),
                static_cast<uint16_t>(request->speed3),
                static_cast<uint16_t>(request->speed4),
                static_cast<uint16_t>(request->speed5)
            };

            std::lock_guard<std::mutex> lock(*hand.mutex);
            int rc = modbus_write_registers(hand.ctx, 1522, 6, speeds);
            response->speed_accepted = (rc == 6);
             if (!response->speed_accepted) {
                RCLCPP_ERROR(this->get_logger(), "Modbus write speed error (ID %d): %s", request->id, modbus_strerror(errno));
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Hand ID %d: Speed out of range!", request->id);
            response->speed_accepted = false;
        }
    }

    void getposact_callback(const service_interfaces::srv::Getposact::Request::SharedPtr request,
                            const service_interfaces::srv::Getposact::Response::SharedPtr response)
    {
        // ВАЖНО: Убедись, что в Getposact.srv тоже есть поле int16 id!
        // Если нет, добавь его, иначе request->id не скомпилируется или будет мусором.
        HandContext hand = get_ctx(request->id);
        
        if (!hand.ctx) {
            for(auto& v : response->curpos) v = -1;
            return;
        }

        int register_addresses[6] = {1534, 1536, 1538, 1540, 1542, 1544};
        uint16_t tab_reg[1];

        std::lock_guard<std::mutex> lock(*hand.mutex);
        for (int i = 0; i < 6; ++i) {
            int rc = modbus_read_registers(hand.ctx, register_addresses[i], 1, tab_reg);
            if (rc == -1) {
                response->curpos[i] = -1;
            } else {
                response->curpos[i] = static_cast<int16_t>(tab_reg[0]);
            }
        }
    }
    
    // TODO: Здесь нужно добавить остальные callback'и (GetAngle, SetAngle и т.д.)
    // по тому же принципу:
    // 1. modbus_t* ctx = get_ctx(request->id);
    // 2. if (!ctx) return;
    // 3. Использовать ctx вместо старого ctx_
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HandControl>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
