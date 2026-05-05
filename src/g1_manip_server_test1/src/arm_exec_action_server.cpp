// TODO: мягкий переход в нулевые положения

#include <functional>
#include <memory>
#include <thread>
#include <chrono>
#include <cmath>
#include <array>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <optional>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "moveit_msgs/srv/get_motion_plan.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"
#include "common/motor_crc_hg.h"

#include "dftp_hand_control/hand_controller.hpp"

#include <geometry_msgs/msg/point_stamped.hpp>

#include "g1_manip_interf_test1/action/test_server_exec.hpp"

#include <torch/script.h>
#include <moveit_msgs/srv/get_state_validity.hpp>

using namespace std::placeholders;

// ====================================================================
// Константы и индексы суставов (без изменений)
// ====================================================================
constexpr int G1_NUM_MOTOR = 29;
constexpr int NOT_USED_JOINT = 29;

enum G1JointIndex {
    LEFT_HIP_PITCH = 0, LEFT_HIP_ROLL = 1, LEFT_HIP_YAW = 2,
    LEFT_KNEE = 3, LEFT_ANKLE_PITCH = 4, LEFT_ANKLE_B = 4,
    LEFT_ANKLE_ROLL = 5, LEFT_ANKLE_A = 5,
    RIGHT_HIP_PITCH = 6, RIGHT_HIP_ROLL = 7, RIGHT_HIP_YAW = 8,
    RIGHT_KNEE = 9, RIGHT_ANKLE_PITCH = 10, RIGHT_ANKLE_B = 10,
    RIGHT_ANKLE_ROLL = 11, RIGHT_ANKLE_A = 11,
    WAIST_YAW = 12, WAIST_ROLL = 13, WAIST_A = 13,
    WAIST_PITCH = 14, WAIST_B = 14,
    LEFT_SHOULDER_PITCH = 15, LEFT_SHOULDER_ROLL = 16,
    LEFT_SHOULDER_YAW = 17, LEFT_ELBOW = 18,
    LEFT_WRIST_ROLL = 19, LEFT_WRIST_PITCH = 20, LEFT_WRIST_YAW = 21,
    RIGHT_SHOULDER_PITCH = 22, RIGHT_SHOULDER_ROLL = 23,
    RIGHT_SHOULDER_YAW = 24, RIGHT_ELBOW = 25,
    RIGHT_WRIST_ROLL = 26, RIGHT_WRIST_PITCH = 27, RIGHT_WRIST_YAW = 28
};

const std::vector<int> LEFT_ARM_MOTORS = {
    LEFT_SHOULDER_PITCH, LEFT_SHOULDER_ROLL, LEFT_SHOULDER_YAW,
    LEFT_ELBOW, LEFT_WRIST_ROLL, LEFT_WRIST_PITCH, LEFT_WRIST_YAW
};
const std::vector<int> RIGHT_ARM_MOTORS = {
    RIGHT_SHOULDER_PITCH, RIGHT_SHOULDER_ROLL, RIGHT_SHOULDER_YAW,
    RIGHT_ELBOW, RIGHT_WRIST_ROLL, RIGHT_WRIST_PITCH, RIGHT_WRIST_YAW
};
const std::vector<int> ALL_ARM_MOTORS = {
    LEFT_SHOULDER_PITCH, LEFT_SHOULDER_ROLL, LEFT_SHOULDER_YAW, LEFT_ELBOW,
    LEFT_WRIST_ROLL, LEFT_WRIST_PITCH, LEFT_WRIST_YAW,
    RIGHT_SHOULDER_PITCH, RIGHT_SHOULDER_ROLL, RIGHT_SHOULDER_YAW, RIGHT_ELBOW,
    RIGHT_WRIST_ROLL, RIGHT_WRIST_PITCH, RIGHT_WRIST_YAW
};


// TODO: Настроить
const std::array<float, G1_NUM_MOTOR> Kp = {
    60, 60, 60, 100, 40, 40, 60, 60, 60, 100, 40, 40, 400, 240, 240,
    90, 90, 80, 80, 40, 40, 40, 90, 90, 80, 80, 40, 40, 40
};
const std::array<float, G1_NUM_MOTOR> Kd = {
    1, 1, 1, 2, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 1.5, 1.5, 1.5
};

// Позиции для хоуминга рук soft_disable
const std::array<double, 7> HOME_RIGHT = {
    0.25275930762290955,   // right_shoulder_pitch  (14.48°)
    -0.28710609674453735,  // right_shoulder_roll   (-16.45°)
    0.08768857270479202,   // right_shoulder_yaw    (5.02°)
    0.7852064371109009,    // right_elbow           (44.99°)
    -0.008892294950783253, // right_wrist_roll      (-0.51°)
    0.003798999357968569,  // right_wrist_pitch     (0.22°)
    0.0020852552261203527  // right_wrist_yaw       (0.12°)
};

const std::array<double, 7> HOME_LEFT = {
    0.2559950351715088,    // left_shoulder_pitch   (14.67°)
    0.28693830966949463,   // left_shoulder_roll    (16.44°)
    -0.07502125203609467,  // left_shoulder_yaw     (-4.30°)
    0.8284095525741577,    // left_elbow            (47.47°)
    0.0029001825023442507, // left_wrist_roll       (0.17°)
    0.004685832187533379,  // left_wrist_pitch      (0.27°)
    0.0015939020086079836  // left_wrist_yaw        (0.09°)
};


// TODO why?
enum class Mode : uint8_t { PR = 0, AB = 1 };

// ====================================================================
// Класс для загрузки и инференса нейросети
// ====================================================================
class NeuralPolicy {
public:
    NeuralPolicy(const std::string& model_path) {
        try {
            module_ = torch::jit::load(model_path);
            module_.eval();
            std::cout << "NeuralPolicy: model loaded from " << model_path << std::endl;
        } catch (const c10::Error& e) {
            std::cerr << "NeuralPolicy: load failed - " << e.what() << std::endl;
            throw;
        }
    }

    std::vector<float> act(const std::vector<float>& observation) {
        torch::Tensor input = torch::tensor(observation).view({1, (int64_t)observation.size()});
        torch::NoGradGuard no_grad;
        torch::Tensor output = module_.forward({input}).toTensor();
        output = output.squeeze().to(torch::kCPU);
        std::vector<float> result(output.data_ptr<float>(), output.data_ptr<float>() + output.numel());
        return result;
    }

private:
    torch::jit::script::Module module_;
};

// ====================================================================
// Буфер данных
// ====================================================================
template <typename T>
class DataBuffer {
public:
    void SetData(const T &new_data) {
        std::lock_guard<std::mutex> lock(mutex_);
        data_ = std::make_shared<T>(new_data);
    }
    std::shared_ptr<const T> GetData() {
        std::lock_guard<std::mutex> lock(mutex_);
        return data_ ? data_ : nullptr;
    }
    void Clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        data_ = nullptr;
    }
private:
    std::shared_ptr<T> data_;
    std::mutex mutex_;
};

struct MotorCommand {
    std::array<float, G1_NUM_MOTOR> q_target{};
    std::array<float, G1_NUM_MOTOR> dq_target{};
    std::array<float, G1_NUM_MOTOR> kp{};
    std::array<float, G1_NUM_MOTOR> kd{};
    std::array<float, G1_NUM_MOTOR> tau_ff{};
};

struct MotorState {
    std::array<float, G1_NUM_MOTOR> q{};
    std::array<float, G1_NUM_MOTOR> dq{};
};

// ====================================================================
// Менеджер нейросетевых действий
// Отвечает за:
// - загрузку модели (start)
// - обновление нейросети с частотой 50 Гц (раз в 20 мс)
// - сборку вектора наблюдений из low_state (включая предыдущие действия)
// - хранение последней вычисленной команды (current_action_)
// - выдачу команды по запросу (getCurrentCommand)
// ====================================================================
class NeuralActionManager {
public:
    NeuralActionManager(rclcpp::Node* node) : node_(node), active_(false),
        prev_action_(29, 0.0f), current_action_(29, 0.0f) {}

    bool start(const std::string& action_name, const std::string& model_path, double timeout_sec = 30.0) {
        if (active_) {
            RCLCPP_WARN(node_->get_logger(), "NeuralActionManager already active, stopping previous");
            stop();
        }
        try {
            policy_ = std::make_unique<NeuralPolicy>(model_path);
            RCLCPP_INFO(node_->get_logger(), "NeuralActionManager: started for action '%s' with timeout %.1f sec",
                        action_name.c_str(), timeout_sec);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to load model: %s", e.what());
            return false;
        }
        active_ = true;
        start_time_ = std::chrono::steady_clock::now();
        last_update_time_ = start_time_ - std::chrono::milliseconds(20); // чтобы первое обновление произошло сразу
        timeout_sec_ = timeout_sec;
        
        std::fill(current_action_.begin(), current_action_.end(), 0.0f);
        std::fill(prev_action_.begin(), prev_action_.end(), 0.0f);
        return true;
    }

    void stop() {
        active_ = false;
        policy_.reset();
        RCLCPP_INFO(node_->get_logger(), "NeuralActionManager stopped");
    }

    bool isActive() const { return active_; }

    // Проверка таймаута (используется в execute_neural_action для цикла ожидания)
    bool isTimeoutExpired() const {
        if (!active_) return true;
        double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();
        return elapsed > timeout_sec_;
    }

    // Главный метод, вызываемый из command_generator_callback (500 Гц).
    // Он решает, когда обновить нейросеть (не чаще 20 мс) и возвращает команду.
    bool getCurrentCommand(std::vector<float>& cmd_out, const unitree_hg::msg::LowState::SharedPtr low_state) {
        if (!active_) return false;

        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - last_update_time_).count();
        if (dt >= 0.02) {
            std::vector<float> obs = buildObservation(low_state);
            updateNeural(obs);
            last_update_time_ = now;
        }

        cmd_out = current_action_;
        return true;
    }

private:
    // Сборка вектора наблюдений размером 633 из low_state
    std::vector<float> buildObservation(const unitree_hg::msg::LowState::SharedPtr low_state) {
        std::vector<float> obs(633, 0.0f);
        if (!low_state) return obs;

        const auto& imu = low_state->imu_state;

        obs[3] = imu.gyroscope[0];
        obs[4] = imu.gyroscope[1];
        obs[5] = imu.gyroscope[2];

        float ax = imu.accelerometer[0];
        float ay = imu.accelerometer[1];
        float az = imu.accelerometer[2];
        float norm = std::sqrt(ax*ax + ay*ay + az*az);
        if (norm > 0.001f) {
            obs[6] = ax / norm;
            obs[7] = ay / norm;
            obs[8] = az / norm;
        }

        for (int i = 0; i < 27; ++i) {
            obs[9 + i] = low_state->motor_state[i].q;
        }

        for (int i = 0; i < 27; ++i) {
            obs[36 + i] = low_state->motor_state[i].dq;
        }

        for (int i = 0; i < 29; ++i) {
            obs[63 + i] = prev_action_[i];
        }

        // Остальные 541 число (target_ref, rel_pose_target) пока нули.
        return obs;
    }

    void updateNeural(const std::vector<float>& obs) {
        if (!policy_) return;
        std::vector<float> action = policy_->act(obs);
        if (action.size() >= 29) {
            current_action_ = action;
            // Сохраняем текущее действие как предыдущее для следующего вызова
            prev_action_ = current_action_;
        } else {
            RCLCPP_WARN(node_->get_logger(), "Policy returned wrong size: %zu, expected 29", action.size());
        }
    }

    rclcpp::Node* node_;
    std::unique_ptr<NeuralPolicy> policy_;
    bool active_;
    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point last_update_time_;
    double timeout_sec_;
    std::vector<float> prev_action_;      // предыдущее действие (29)
    std::vector<float> current_action_;   // последнее действие от нейросети (29)
};

// ====================================================================
// Менеджер траекторий: содержит поле с траекторией, началом выполнения, и методы для работы с ним
// ====================================================================
class TrajectoryManager {
public:
    enum class State { IDLE, EXECUTING };
    
    void startTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory,
                        const std::unordered_map<std::string, int>& joint_map) {
        std::lock_guard<std::mutex> lock(mutex_);
        current_trajectory_ = trajectory;
        //mapping trajectory
        joint_mapping_.clear();
        for (const auto& joint_name : trajectory.joint_trajectory.joint_names) {
            auto it = joint_map.find(joint_name);
            joint_mapping_.push_back({joint_name, (it != joint_map.end()) ? it->second : -1});
        }
        state_ = State::EXECUTING;
        start_time_ = std::chrono::steady_clock::now();
    }
    
    bool getCurrentCommand(MotorCommand& cmd, const MotorState& current_state,
                          const std::unordered_map<std::string, int>& joint_map) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (state_ != State::EXECUTING) return false;
        const auto& points = current_trajectory_.joint_trajectory.points;
        if (points.empty()) {
            state_ = State::IDLE;
            return false;
        }
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - start_time_).count();
        size_t current_idx = 0;
        for (size_t i = 0; i < points.size(); ++i) {
            double t = points[i].time_from_start.sec + points[i].time_from_start.nanosec * 1e-9;
            if (t <= elapsed) current_idx = i;
            else break;
        }
        const auto& point = points[current_idx];
        // По умолчанию – удержание текущей позы
        for (int idx : ALL_ARM_MOTORS) {
            cmd.q_target[idx] = current_state.q[idx];
            cmd.dq_target[idx] = 0.0f;
        }
        for (size_t j = 0; j < point.positions.size() && j < joint_mapping_.size(); ++j) {
            int motor_idx = joint_mapping_[j].second;
            if (motor_idx >= 0) {
                cmd.q_target[motor_idx] = static_cast<float>(point.positions[j]);
                if (j < point.velocities.size())
                    cmd.dq_target[motor_idx] = static_cast<float>(point.velocities[j]);
            }
        }
        double total_duration = points.back().time_from_start.sec + points.back().time_from_start.nanosec * 1e-9;
        if (elapsed > total_duration + 0.1) {
            state_ = State::IDLE;
            current_trajectory_ = moveit_msgs::msg::RobotTrajectory();
            joint_mapping_.clear();
        }
        return true;
    }
    
    bool isExecuting() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return state_ == State::EXECUTING;
    }
    void stop() {
        std::lock_guard<std::mutex> lock(mutex_);
        state_ = State::IDLE;
        current_trajectory_ = moveit_msgs::msg::RobotTrajectory();
        joint_mapping_.clear();
    }
private:
    mutable std::mutex mutex_;
    State state_{State::IDLE};
    moveit_msgs::msg::RobotTrajectory current_trajectory_;
    std::vector<std::pair<std::string, int>> joint_mapping_;
    std::chrono::steady_clock::time_point start_time_;
};

// ====================================================================
// Главный класс сервера манипуляции
// ====================================================================
class ManipulationActionServer : public rclcpp::Node {
public:
    // Изменить назвааниа TODO
    using MotionPlan = g1_manip_interf_test1::action::TestServerExec;
    using GoalHandleMotionPlan = rclcpp_action::ServerGoalHandle<MotionPlan>;
    using MotionPlanService = moveit_msgs::srv::GetMotionPlan;

    // Состояния машины (EnableControl)
    enum class EnableControl { ENABLED, DISABLED, EMERGENCY };
    // Состояния действия (StateOfAction)
    enum class StateOfAction { HOLDING_POSE, EXECUTING_TRAJ, NEURO_DRIVEN };

    ManipulationActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("manipulation_action_server", options),
          enable_control_(EnableControl::DISABLED),
          state_of_action_(StateOfAction::HOLDING_POSE) {

        // Action server
        action_server_ = rclcpp_action::create_server<MotionPlan>(
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "r_arm_plan_exec",
            std::bind(&ManipulationActionServer::handle_goal, this, _1, _2),
            std::bind(&ManipulationActionServer::handle_cancel, this, _1),
            std::bind(&ManipulationActionServer::handle_accepted, this, _1));

        // MoveIt client
        planning_client_ = this->create_client<MotionPlanService>("/plan_kinematic_path");
        validity_client_ = this->create_client<moveit_msgs::srv::GetStateValidity>("/check_state_validity");

        // LowState subscriber
        lowstate_subscriber_ = this->create_subscription<unitree_hg::msg::LowState>(
            "lowstate", 10, std::bind(&ManipulationActionServer::low_state_callback, this, _1));

        // LowCmd publisher (/arm_sdk)
        lowcmd_publisher_ = this->create_publisher<unitree_hg::msg::LowCmd>("/arm_sdk", 10);

        // Command Generator (500 Гц) – формирует команду в буфер
        command_generator_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2),   // 500 Hz
            std::bind(&ManipulationActionServer::command_generator_callback, this));

        // Commander (500 Гц) – отправляет команду в arm_sdk
        commander_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2),
            std::bind(&ManipulationActionServer::commander_callback, this));

        
        // ====== ИНИЦИАЛИЗАЦИЯ КИСТЕЙ (Вариант 2: прокси-узел БЕЗ явного spin) ======
        try {
            // Создаём отдельный узел-прокси ТОЛЬКО для кистей
            // Важно: этот узел НЕ добавляется в главный executor и НЕ спинится явно
            auto hand_node = rclcpp::Node::make_shared("hand_control_proxy");
        
            // HandController будет использовать этот узел для сервис-клиентов
            // При вызове executeGesture() библиотека сама создаст временный executor
            // через spin_until_future_complete, поэтому конфликт не возникнет
            hand_controller_ = std::make_unique<dftp_hand::HandController>(hand_node);
        
            // Сохраняем узел как член класса, чтобы он не уничтожился
            hand_proxy_node_ = hand_node;
        
            RCLCPP_INFO(this->get_logger(), "Hand controller initialized successfully.");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Hand controller init failed: %s", e.what());
        }
        // ============================================================================
        

        // CV test
        subscription_cv_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/detected_handle/xyz",
            10,  // QoS (queue size)
            std::bind(&ManipulationActionServer::door_handle_callback, this, _1)
        );
        neural_manager_ = std::make_unique<NeuralActionManager>(this);
        RCLCPP_INFO(this->get_logger(), "Manipulation Action Server started");
        RCLCPP_INFO(this->get_logger(), "Initial state: DISABLED / HOLDING_POSE");

        is_waiting_for_collision_result_ = false;
        last_safe_neural_cmd_.assign(29, 0.0f);
    }

private:
    // CV test
    void door_handle_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        // Обновляем вектор: [x, y, z]
        current_handle_xyz_.clear();
        current_handle_xyz_.push_back(msg->point.x);
        current_handle_xyz_.push_back(msg->point.y);
        current_handle_xyz_.push_back(msg->point.z);
        
        // Выводим информацию
        RCLCPP_INFO(this->get_logger(), 
                   "Updated handle position - x: %.3f, y: %.3f, z: %.3f",
                   current_handle_xyz_[0], current_handle_xyz_[1], current_handle_xyz_[2]);
    }


    // ====================================================================
    // Публичные методы машины состояний (API)
    // ====================================================================


    void enable() {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (enable_control_ == EnableControl::EMERGENCY) {
            RCLCPP_WARN(this->get_logger(), "Cannot enable from EMERGENCY, call emergency() again to reset");
            return;
        }
        enable_control_ = EnableControl::ENABLED;
        RCLCPP_INFO(this->get_logger(), "EnableControl -> ENABLED");
    }

    void disable() {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (enable_control_ == EnableControl::EMERGENCY) {
            RCLCPP_WARN(this->get_logger(), "Cannot disable from EMERGENCY, call emergency() again to reset");
            return;
        }
        enable_control_ = EnableControl::DISABLED;
        // Останавливаем любую траекторию
        trajectory_manager_.stop();
        state_of_action_ = StateOfAction::HOLDING_POSE;
        RCLCPP_INFO(this->get_logger(), "EnableControl -> DISABLED");
    }

    void emergency() {
        std::lock_guard<std::mutex> lock(state_mutex_);
        enable_control_ = EnableControl::EMERGENCY;
        trajectory_manager_.stop();
        if (neural_manager_) neural_manager_->stop();
        state_of_action_ = StateOfAction::HOLDING_POSE;
        RCLCPP_WARN(this->get_logger(), "EMERGENCY mode activated");
        is_waiting_for_collision_result_ = false;
        collision_future_ = std::shared_future<bool>();
    }

    void execute_traj(const moveit_msgs::msg::RobotTrajectory& traj) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (enable_control_ != EnableControl::ENABLED) {
            RCLCPP_ERROR(this->get_logger(), "execute_traj rejected: not in ENABLED state");
            return;
        }
        if (trajectory_manager_.isExecuting()) {
            RCLCPP_ERROR(this->get_logger(), "Another trajectory is currently being executed");
            return;
        }
        trajectory_manager_.startTrajectory(traj, JOINT_NAME_TO_INDEX);
        state_of_action_ = StateOfAction::EXECUTING_TRAJ;
        RCLCPP_INFO(this->get_logger(), "Trajectory execution started");
    }

    moveit_msgs::msg::RobotTrajectory plan_from_current(
        double x, double y, double z,
        double qx = 0.0, double qy = 0.0, double qz = 0.0, double qw = 1.0) {
        // Оригинальная функция планирования
        moveit_msgs::msg::RobotTrajectory result;
        auto current_state = motor_state_buffer_.GetData();
        if (!current_state) {
            RCLCPP_ERROR(this->get_logger(), "Cannot plan: no robot state");
            return result;
        }
        std::vector<double> start_positions = {
            current_state->q[RIGHT_SHOULDER_PITCH],
            current_state->q[RIGHT_SHOULDER_ROLL],
            current_state->q[RIGHT_SHOULDER_YAW],
            current_state->q[RIGHT_ELBOW],
            current_state->q[RIGHT_WRIST_ROLL],
            current_state->q[RIGHT_WRIST_PITCH],
            current_state->q[RIGHT_WRIST_YAW]
        };
        auto request = std::make_shared<MotionPlanService::Request>();
        request->motion_plan_request.group_name = "arm_r";
        request->motion_plan_request.start_state.is_diff = true;
        request->motion_plan_request.start_state.joint_state.name = {
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
            "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"
        };
        request->motion_plan_request.start_state.joint_state.position = start_positions;
        request->motion_plan_request.start_state.joint_state.velocity = std::vector<double>(7, 0.0);
        request->motion_plan_request.start_state.joint_state.effort = std::vector<double>(7, 0.0);

        moveit_msgs::msg::Constraints goal_constraint;
        moveit_msgs::msg::PositionConstraint pos_constraint;
        pos_constraint.header.frame_id = "pelvis";
        pos_constraint.link_name = "right_hand_middle_ee";
        shape_msgs::msg::SolidPrimitive sphere;
        sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
        sphere.dimensions = {0.01};
        pos_constraint.constraint_region.primitives.push_back(sphere);
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = x; target_pose.position.y = y; target_pose.position.z = z;
        target_pose.orientation.x = qx; target_pose.orientation.y = qy;
        target_pose.orientation.z = qz; target_pose.orientation.w = qw;
        pos_constraint.constraint_region.primitive_poses.push_back(target_pose);
        pos_constraint.weight = 1.0;
        goal_constraint.position_constraints.push_back(pos_constraint);

        moveit_msgs::msg::OrientationConstraint ori_constraint;
        ori_constraint.header.frame_id = "pelvis";
        ori_constraint.link_name = "right_hand_middle_ee";
        ori_constraint.orientation = target_pose.orientation;
        ori_constraint.absolute_x_axis_tolerance = 0.1;
        ori_constraint.absolute_y_axis_tolerance = 0.1;
        ori_constraint.absolute_z_axis_tolerance = 0.1;
        ori_constraint.weight = 1.0;
        goal_constraint.orientation_constraints.push_back(ori_constraint);

        request->motion_plan_request.goal_constraints.push_back(goal_constraint);
        request->motion_plan_request.allowed_planning_time = 5.0;
        request->motion_plan_request.max_velocity_scaling_factor = 0.5;
        request->motion_plan_request.max_acceleration_scaling_factor = 0.5;

        auto future = planning_client_->async_send_request(request);
        auto status = future.wait_for(std::chrono::seconds(10));
        if (status == std::future_status::ready) {
            auto response = future.get();
            if (response->motion_plan_response.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
                result = response->motion_plan_response.trajectory;
                RCLCPP_INFO(this->get_logger(), "Planning successful");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Planning failed, error code: %d",
                             response->motion_plan_response.error_code.val);
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning service timeout");
        }
        return result;
    }

    moveit_msgs::msg::RobotTrajectory plan_to_joint_angles(
        const std::vector<double>& target_joint_angles) {
        // Validate input
        if (target_joint_angles.size() != 7) {
            RCLCPP_ERROR(this->get_logger(), 
                        "Invalid number of joint angles. Expected 7, got %zu",
                        target_joint_angles.size());
            return moveit_msgs::msg::RobotTrajectory();
        }
        
        moveit_msgs::msg::RobotTrajectory result;
        auto current_state = motor_state_buffer_.GetData();
        if (!current_state) {
            RCLCPP_ERROR(this->get_logger(), "Cannot plan: no robot state");
            return result;
        }
        
        // Get current joint positions
        std::vector<double> start_positions = {
            current_state->q[RIGHT_SHOULDER_PITCH],
            current_state->q[RIGHT_SHOULDER_ROLL],
            current_state->q[RIGHT_SHOULDER_YAW],
            current_state->q[RIGHT_ELBOW],
            current_state->q[RIGHT_WRIST_ROLL],
            current_state->q[RIGHT_WRIST_PITCH],
            current_state->q[RIGHT_WRIST_YAW]
        };
        
        // Joint names in the same order
        std::vector<std::string> joint_names = {
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
            "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"
        };
        
        auto request = std::make_shared<MotionPlanService::Request>();
        request->motion_plan_request.group_name = "arm_r";
        request->motion_plan_request.start_state.is_diff = true;
        request->motion_plan_request.start_state.joint_state.name = joint_names;
        request->motion_plan_request.start_state.joint_state.position = start_positions;
        request->motion_plan_request.start_state.joint_state.velocity = std::vector<double>(7, 0.0);
        request->motion_plan_request.start_state.joint_state.effort = std::vector<double>(7, 0.0);
        
        // Create goal constraints for joint angles
        moveit_msgs::msg::Constraints goal_constraint;
        moveit_msgs::msg::JointConstraint joint_constraint;
        
        for (size_t i = 0; i < joint_names.size(); ++i) {
            joint_constraint.joint_name = joint_names[i];
            joint_constraint.position = target_joint_angles[i];
            joint_constraint.tolerance_above = 0.01;  // 0.01 rad tolerance
            joint_constraint.tolerance_below = 0.01;
            joint_constraint.weight = 1.0;
            goal_constraint.joint_constraints.push_back(joint_constraint);
        }
        
        request->motion_plan_request.goal_constraints.push_back(goal_constraint);
        request->motion_plan_request.allowed_planning_time = 5.0;
        request->motion_plan_request.max_velocity_scaling_factor = 0.5;
        request->motion_plan_request.max_acceleration_scaling_factor = 0.5;
        
        auto future = planning_client_->async_send_request(request);
        auto status = future.wait_for(std::chrono::seconds(10));
        
        if (status == std::future_status::ready) {
            auto response = future.get();
            if (response->motion_plan_response.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
                result = response->motion_plan_response.trajectory;
                RCLCPP_INFO(this->get_logger(), "Joint angle planning successful");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Joint angle planning failed, error code: %d",
                            response->motion_plan_response.error_code.val);
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Joint angle planning service timeout");
        }
        
        return result;
    }

    moveit_msgs::msg::RobotTrajectory interpolate_trajectory(
        const moveit_msgs::msg::RobotTrajectory& trajectory, int factor = 8) {
        
        if (trajectory.joint_trajectory.points.empty() || factor <= 1) {
            return trajectory;
        }
        
        moveit_msgs::msg::RobotTrajectory interpolated = trajectory;
        interpolated.joint_trajectory.points.clear();
        
        const auto& points = trajectory.joint_trajectory.points;
        size_t num_joints = points[0].positions.size();
        
        for (size_t i = 0; i < points.size() - 1; ++i) {
            const auto& p1 = points[i];
            const auto& p2 = points[i + 1];
            
            double t1 = p1.time_from_start.sec + p1.time_from_start.nanosec * 1e-9;
            double t2 = p2.time_from_start.sec + p2.time_from_start.nanosec * 1e-9;
            double dt = (t2 - t1) / factor;
            
            for (int step = 0; step < factor; ++step) {
                double ratio = static_cast<double>(step) / factor;
                double current_time = t1 + step * dt;
                
                trajectory_msgs::msg::JointTrajectoryPoint new_point;
                new_point.time_from_start = rclcpp::Duration::from_seconds(current_time);
                new_point.positions.resize(num_joints);
                
                // Линейная интерполяция позиций
                for (size_t j = 0; j < num_joints; ++j) {
                    new_point.positions[j] = p1.positions[j] + ratio * (p2.positions[j] - p1.positions[j]);
                }
                
                // Интерполяция скоростей (если есть)
                if (!p1.velocities.empty() && !p2.velocities.empty()) {
                    new_point.velocities.resize(num_joints);
                    for (size_t j = 0; j < num_joints; ++j) {
                        new_point.velocities[j] = p1.velocities[j] + ratio * (p2.velocities[j] - p1.velocities[j]);
                    }
                }
                
                interpolated.joint_trajectory.points.push_back(new_point);
            }
        }
    
        // Добавляем последнюю точку
        interpolated.joint_trajectory.points.push_back(points.back());
        
        return interpolated;
    }

    // Заглушка проверки на самопересечения
    bool check_validity(const MotorState& /*joint_states*/) {
        // TODO: реализовать проверку коллизий
        return true;
    }

    void wait_for_trajectory_completion() {
        while (trajectory_manager_.isExecuting() && rclcpp::ok()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void move_to_pose(double x, double y, double z,
    double qx = 0.0, double qy = 0.0, double qz = 0.0, double qw = 1.0){

        auto traj_planned = plan_from_current(x, y, z, qx, qy, qz, qw);
        if (!traj_planned.joint_trajectory.points.empty()) {
            auto traj_smooth = interpolate_trajectory(traj_planned);
            execute_traj(traj_smooth);
            wait_for_trajectory_completion();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan approach");
            if (current_goal_handle_) {
                auto result = std::make_shared<MotionPlan::Result>();
                result->success = false;
                current_goal_handle_->abort(result);
                current_goal_handle_.reset();
            }
            return;
        }
    }

    void move_to_pose_angles(const std::vector<double>& target_joint_angles){

        auto traj_planned = plan_to_joint_angles(target_joint_angles);
        if (!traj_planned.joint_trajectory.points.empty()) {
            auto traj_smooth = interpolate_trajectory(traj_planned);
            execute_traj(traj_smooth);
            wait_for_trajectory_completion();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan approach");
            if (current_goal_handle_) {
                auto result = std::make_shared<MotionPlan::Result>();
                result->success = false;
                current_goal_handle_->abort(result);
                current_goal_handle_.reset();
            }
            return;
        }

    }

    bool isValid(const std::vector<std::string>& joint_names,
                const std::vector<double>& positions,
                double timeout_sec = 0.5) {
        // Проверяем, что списки одинаковой длины
        if (joint_names.size() != positions.size()) {
            RCLCPP_ERROR(get_logger(), "isValid: joint names and positions size mismatch");
            return false;
        }
        // Ждём, пока сервис станет доступен 
        if (!validity_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(get_logger(), "Validity service not available");
            return false;  
        }
        auto request = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
        request->robot_state.joint_state.name = joint_names;
        request->robot_state.joint_state.position = positions;
        request->robot_state.joint_state.velocity.resize(joint_names.size(), 0.0);
        request->robot_state.joint_state.effort.resize(joint_names.size(), 0.0);
        auto future = validity_client_->async_send_request(request);
        if (future.wait_for(std::chrono::duration<double>(timeout_sec)) == std::future_status::ready) {
            return future.get()->valid;
        } else {
            RCLCPP_ERROR(get_logger(), "Validity service timeout after %.2f sec", timeout_sec);
            return false;
        }
    }

    void soft_disable(){
        std::vector<double> home_right_vec(HOME_RIGHT.begin(), HOME_RIGHT.end());
        move_to_pose_angles(home_right_vec);

        std::this_thread::sleep_for(std::chrono::milliseconds(500));      
        disable();
        return;
    }

    // ====================================================================
    // Command Generator (500 Hz) – формирует MotorCommand на основе состояния
    // ====================================================================
    void command_generator_callback() {
        auto current_state = motor_state_buffer_.GetData();
        if (!current_state) return;

        auto prev_command = current_command_buffer_.GetData();

        MotorCommand cmd;
        // Инициализация коэффициентов и feedforward
        for (int i = 0; i < G1_NUM_MOTOR; ++i) {
            cmd.kp[i] = Kp[i];
            cmd.kd[i] = Kd[i];
            cmd.tau_ff[i] = 0.0f;
            cmd.dq_target[i] = 0.0f;
            // q_target пока не заполняем
        }
        // По умолчанию q_target 
        if (!prev_command) {
            for (int idx : ALL_ARM_MOTORS) {
                cmd.q_target[idx] = current_state->q[idx];
            }
            cmd.q_target[WAIST_YAW] = current_state->q[WAIST_YAW];
            cmd.q_target[WAIST_ROLL] = current_state->q[WAIST_ROLL];
            cmd.q_target[WAIST_PITCH] = current_state->q[WAIST_PITCH];
        }
        else {
            for (int idx : ALL_ARM_MOTORS) {
                cmd.q_target[idx] = prev_command->q_target[idx];

            }
            // По умолчанию удержание позы талии
            cmd.q_target[WAIST_YAW] = prev_command->q_target[WAIST_YAW];
            cmd.q_target[WAIST_ROLL] = prev_command->q_target[WAIST_ROLL];
            cmd.q_target[WAIST_PITCH] = prev_command->q_target[WAIST_PITCH];
        }




        EnableControl ec;
        StateOfAction soa;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            ec = enable_control_;
            soa = state_of_action_;
        }

        switch (ec) {
            case EnableControl::EMERGENCY:
                //Заполняем текущими (не обязательно)
                for (int idx : ALL_ARM_MOTORS) {
                    cmd.kp[idx] = 0.0;
                }
                break;

            case EnableControl::DISABLED:
                // Удержание текущей позы (q_target = current)
                for (int idx : ALL_ARM_MOTORS) {
                    cmd.q_target[idx] = current_state->q[idx];
                }
                break;

            case EnableControl::ENABLED:
                switch (soa) {
                    case StateOfAction::HOLDING_POSE:

                        if (!prev_command) {
                            for (int idx : ALL_ARM_MOTORS) {
                                cmd.q_target[idx] = current_state->q[idx];
                            }
                        }
                        else {
                            for (int idx : ALL_ARM_MOTORS) {
                                cmd.q_target[idx] = prev_command->q_target[idx];
                            }
                        }
                        break;

                    case StateOfAction::EXECUTING_TRAJ:
                        {
                            bool active = trajectory_manager_.getCurrentCommand(cmd, *current_state, JOINT_NAME_TO_INDEX);
                            if (!active) {
                                // Траектория завершена – переключаемся в HOLDING_POSE
                                // TODO отщелкивание автомата убрать. Улучшить логику
                                std::lock_guard<std::mutex> lock(state_mutex_);
                                if (state_of_action_ == StateOfAction::EXECUTING_TRAJ) {
                                    state_of_action_ = StateOfAction::HOLDING_POSE;
                                    RCLCPP_INFO(this->get_logger(), "Trajectory finished, -> HOLDING_POSE");
                                    // TODO перенести в конец execute, это неверно
                                    if (current_goal_handle_ && !current_goal_handle_->is_canceling()) {
                                        auto result = std::make_shared<MotionPlan::Result>();
                                        result->success = true;
                                        current_goal_handle_->succeed(result);
                                        current_goal_handle_.reset();
                                    }
                                }
                                
                                if (!prev_command) {
                                    for (int idx : ALL_ARM_MOTORS) {
                                        cmd.q_target[idx] = current_state->q[idx];
                                    }
                                }
                                else {
                                    for (int idx : ALL_ARM_MOTORS) {
                                        cmd.q_target[idx] = prev_command->q_target[idx];
                                    }
                                }


                            }
                        }
                        break;

                    case StateOfAction::NEURO_DRIVEN:
                    {
                        // Проверка, что менеджер активен и есть свежий low_state
                        if (!neural_manager_ || !neural_manager_->isActive() || !last_low_state_) {
                            std::lock_guard<std::mutex> lock(state_mutex_);
                            state_of_action_ = StateOfAction::HOLDING_POSE;
                            break;
                        }

                        std::vector<float> neural_cmd;
                        if (neural_manager_->getCurrentCommand(neural_cmd, last_low_state_)) {
                            // ----- Асинхронная проверка коллизий для рук (14 суставов) -----
                            // Имена суставов рук в порядке, соответствующем индексам G1JointIndex
                            static const std::vector<std::string> arm_joint_names = {
                                "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
                                "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
                                "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
                                "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"
                            };
                            const int left_start = LEFT_SHOULDER_PITCH;   
                            const int right_start = RIGHT_SHOULDER_PITCH; 

                            // Если нет ожидающего запроса, отправляем новый асинхронный запрос
                            if (!is_waiting_for_collision_result_) {
                                // Извлекаем позиции только для рук из neural_cmd
                                std::vector<double> arm_positions(arm_joint_names.size());
                                for (size_t i = 0; i < 7; ++i) {
                                    arm_positions[i] = neural_cmd[left_start + i];
                                    arm_positions[i + 7] = neural_cmd[right_start + i];
                                }
                                // Создаём запрос
                                auto request = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
                                request->robot_state.joint_state.name = arm_joint_names;
                                request->robot_state.joint_state.position = arm_positions;
                                request->robot_state.joint_state.velocity.resize(arm_joint_names.size(), 0.0);
                                request->robot_state.joint_state.effort.resize(arm_joint_names.size(), 0.0);
                                // Асинхронная отправка
                                auto future_raw = validity_client_->async_send_request(request);
                                collision_future_ = future_raw.share();
                                is_waiting_for_collision_result_ = true;
                            }

                            // Проверяем, не готов ли результат предыдущего запроса
                            bool use_new_cmd = false;
                            if (is_waiting_for_collision_result_) {
                                auto status = collision_future_.wait_for(std::chrono::milliseconds(1));
                                if (status == std::future_status::ready) {
                                    bool coll_free = collision_future_.get();
                                    is_waiting_for_collision_result_ = false;
                                    if (coll_free) {
                                        last_safe_neural_cmd_ = neural_cmd;
                                        use_new_cmd = true;
                                    } else {
                                        RCLCPP_WARN(get_logger(), "Collision detected! Using last safe command.");
                                    }
                                } else {
                                    // Ответ ещё не пришёл – используем последнюю безопасную команду
                                    use_new_cmd = false;
                                }
                            } else {
                                // Нет ожидающих запросов – все команды уже проверены и безопасны
                                use_new_cmd = true;
                            }

                            // Выбираем, какую команду отправить на моторы
                            const std::vector<float>& cmd_to_use = use_new_cmd ? neural_cmd : last_safe_neural_cmd_;
                            for (int i = 0; i < G1_NUM_MOTOR; ++i) {
                                if (i < (int)cmd_to_use.size())
                                    cmd.q_target[i] = cmd_to_use[i];
                                else
                                    cmd.q_target[i] = current_state->q[i];
                            }
                        } else {
                            // Если getCurrentCommand вернул false – нейрорежим завершён
                            std::lock_guard<std::mutex> lock(state_mutex_);
                            state_of_action_ = StateOfAction::HOLDING_POSE;
                            RCLCPP_INFO(get_logger(), "Neural action finished, switching to HOLDING_POSE");
                        }
                        break;
                    }
                }
                break;
        }

        // Отправляем сгенерированную команду в буфер текущих команд
        current_command_buffer_.SetData(cmd);
    }

    // ====================================================================
    // Commander (500 Hz) – формирует LowCmd и публикует в /arm_sdk
    // ====================================================================
    void commander_callback() {
        unitree_hg::msg::LowCmd low_cmd;
        low_cmd.mode_pr = static_cast<uint8_t>(Mode::PR);
        low_cmd.mode_machine = 0;

        // По умолчанию все моторы в mode=0 (демпфирование)
        for (int i = 0; i < G1_NUM_MOTOR; ++i) {
            low_cmd.motor_cmd[i].mode = 0;
            low_cmd.motor_cmd[i].tau = 0.0;
            low_cmd.motor_cmd[i].q = 0.0;
            low_cmd.motor_cmd[i].dq = 0.0;
            low_cmd.motor_cmd[i].kp = 0.0;
            low_cmd.motor_cmd[i].kd = 0.0;
        }

        EnableControl ec;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            ec = enable_control_;
        }


        // TODO переместить логику в coomand generator
        // Специальная обработка EMERGENCY: все моторы mode=0, NOT_USED_JOINT q = 1
        if (ec == EnableControl::EMERGENCY) {
            low_cmd.motor_cmd[NOT_USED_JOINT].mode = 1;
            low_cmd.motor_cmd[NOT_USED_JOINT].q = 1.0f; // Server controls the joints, NOT robots_alghos. q=0 would make its hands shoot into default walking pose of LocoClient
            get_crc(low_cmd);
            lowcmd_publisher_->publish(low_cmd);
            return;
        }

        // Включаем управление для waist и рук (mode=1)
        for (int idx : {WAIST_YAW, WAIST_ROLL, WAIST_PITCH}) {
            low_cmd.motor_cmd[idx].mode = 1;
        }
        for (int idx : ALL_ARM_MOTORS) {
            low_cmd.motor_cmd[idx].mode = 1;
        }

        // Устанавливаем NOT_USED_JOINT в зависимости от ENABLED/DISABLED
        low_cmd.motor_cmd[NOT_USED_JOINT].mode = 1;
        low_cmd.motor_cmd[NOT_USED_JOINT].q = (ec == EnableControl::ENABLED) ? 1.0f : 0.0f;

        // Берем последнюю команду из буфера (сгенерированную CommandGenerator)
        auto cmd = current_command_buffer_.GetData();
        if (cmd) {
            for (int idx : ALL_ARM_MOTORS) {
                low_cmd.motor_cmd[idx].tau = cmd->tau_ff[idx];
                low_cmd.motor_cmd[idx].q   = cmd->q_target[idx];
                low_cmd.motor_cmd[idx].dq  = cmd->dq_target[idx];
                low_cmd.motor_cmd[idx].kp  = cmd->kp[idx];
                low_cmd.motor_cmd[idx].kd  = cmd->kd[idx];
            }
            // Также для waist (по желанию можно брать из команды, пока удержание)
            low_cmd.motor_cmd[WAIST_YAW].q   = cmd->q_target[WAIST_YAW];
            low_cmd.motor_cmd[WAIST_ROLL].q  = cmd->q_target[WAIST_ROLL];
            low_cmd.motor_cmd[WAIST_PITCH].q = cmd->q_target[WAIST_PITCH];
            // Коэффициенты для waist
            low_cmd.motor_cmd[WAIST_YAW].kp   = Kp[WAIST_YAW];
            low_cmd.motor_cmd[WAIST_YAW].kd   = Kd[WAIST_YAW];
            low_cmd.motor_cmd[WAIST_ROLL].kp  = Kp[WAIST_ROLL];
            low_cmd.motor_cmd[WAIST_ROLL].kd  = Kd[WAIST_ROLL];
            low_cmd.motor_cmd[WAIST_PITCH].kp = Kp[WAIST_PITCH];
            low_cmd.motor_cmd[WAIST_PITCH].kd = Kd[WAIST_PITCH];
        }

        get_crc(low_cmd);
        lowcmd_publisher_->publish(low_cmd);
    }

    // ====================================================================
    // DFTP_hand_control – обёртки для жестов
    // ====================================================================

    bool execute_hand_gesture(dftp_hand::HandSide side, dftp_hand::Gesture gesture, int stabilization_ms = 600) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (enable_control_ != EnableControl::ENABLED) {
            RCLCPP_ERROR(this->get_logger(), "Gesture rejected: robot not ENABLED");
            return false;
        }
        if (!hand_controller_) {
            RCLCPP_ERROR(this->get_logger(), "Hand controller not initialized");
            return false;
        }

        try {
            std::string side_str = (side == dftp_hand::HandSide::LEFT) ? "LEFT" : "RIGHT";
            RCLCPP_INFO(this->get_logger(), "Executing command on %s hand...", side_str.c_str());
            
            // Библиотека dftp_hand_control блокирует поток до получения ответа от драйвера
            hand_controller_->executeGesture(side, gesture);
            
            // Дополнительная синхронизация: ждём стабилизации пальцев перед следующей командой руки
            std::this_thread::sleep_for(std::chrono::milliseconds(stabilization_ms));
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Gesture execution failed: %s", e.what());
            // emergency();
            return false;
        }
    }

    // Для управления обеими кистями одновременно
    bool execute_parallel_gestures(
        dftp_hand::HandSide left_side, dftp_hand::Gesture left_gesture,
        dftp_hand::HandSide right_side, dftp_hand::Gesture right_gesture,
        int stabilization_ms = 600) {
        
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (enable_control_ != EnableControl::ENABLED || !hand_controller_) return false;
        
        try {
            hand_controller_->executeGestureParallel(left_side, left_gesture, right_side, right_gesture);
            std::this_thread::sleep_for(std::chrono::milliseconds(stabilization_ms));
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Parallel gesture failed: %s", e.what());
            emergency();
            return false;
        }
    }

    // ====================================================================
    // LowState callback – обновляет буфер состояния
    // ====================================================================
    void low_state_callback(const unitree_hg::msg::LowState::SharedPtr msg) {
        last_low_state_ = msg;
        MotorState ms;
        for (int i = 0; i < G1_NUM_MOTOR; ++i) {
            ms.q[i] = msg->motor_state[i].q;
            ms.dq[i] = msg->motor_state[i].dq;
        }
        motor_state_buffer_.SetData(ms);
    }

    // ====================================================================
    // Action server callbacks
    // ====================================================================
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const MotionPlan::Goal> goal) {
        (void)uuid;
        // Поддерживаем коды: 0 – button press, 1 – custom traj (заглушка),
        // 2 – enable, 3 – disable, 4 – emergency, 5 – move to pose (заглушка)
        if (goal->action_code >= 0 && goal->action_code <= 5) {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }
        RCLCPP_ERROR(this->get_logger(), "Unknown action code: %d", goal->action_code);
        return rclcpp_action::GoalResponse::REJECT;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMotionPlan> goal_handle) {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Cancel requested -> stopping trajectory, moving to HOLDING_POSE");
        trajectory_manager_.stop();
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            state_of_action_ = StateOfAction::HOLDING_POSE;
            is_waiting_for_collision_result_ = false;
            collision_future_ = std::shared_future<bool>();
        }
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMotionPlan> goal_handle) {
        current_goal_handle_ = goal_handle;
        const auto goal = goal_handle->get_goal();

        switch (goal->action_code) {
            case 0:  // Button press sequence
                std::thread([this]() { execute_button_press(); }).detach();
                break;
            case 2:
            {
                // Пока путь жёсткий, потом вынесем в конфиг
                std::string model_path = "/home/egorlekomtsev/Downloads/2026-03-31_15-52-55-Button-Press-a(1)/2026-03-31_15-52-55-Button-Press-a/exported/policy.pt";
                std::thread([this, model_path]() { execute_neural_action(model_path); }).detach();
                break;
            }
        }
    }

    // Пример высокоуровневого действия (button press)
    void execute_button_press() {
        RCLCPP_INFO(this->get_logger(), "Starting button press sequence");

        // Включаем управление руками
        enable();
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        
        move_to_pose(0.33, -0.31, 0.4);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000)); 

        execute_hand_gesture(dftp_hand::HandSide::RIGHT, dftp_hand::Gesture::POINT_BUTTON, 800);

        move_to_pose(0.44, -0.24, 0.4);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        execute_hand_gesture(dftp_hand::HandSide::RIGHT, dftp_hand::Gesture::FREE_WRIST, 800);
        
        move_to_pose(0.33, -0.31, 0.4);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));   

        // move_to_pose(current_handle_xyz_[0] - 0.1, current_handle_xyz_[1], current_handle_xyz_[2]);

        // std::this_thread::sleep_for(std::chrono::milliseconds(5000));      

        // Планируем траекторию и двигаем руку в точку
        // move_to_pose(current_handle_xyz_[0], current_handle_xyz_[1], current_handle_xyz_[2]);

        std::this_thread::sleep_for(std::chrono::milliseconds(5000));

        soft_disable();

        //std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }

    void execute_neural_action(const std::string& model_path, double timeout = 30.0) {
        RCLCPP_INFO(get_logger(), "Starting neural action with model: %s", model_path.c_str());

        enable();

        if (!neural_manager_->start("button_press", model_path, timeout)) {
            RCLCPP_ERROR(get_logger(), "Failed to start neural manager");
            if (current_goal_handle_) {
                auto result = std::make_shared<MotionPlan::Result>();
                result->success = false;
                current_goal_handle_->abort(result);
                current_goal_handle_.reset();
            }
            return;
        }

        // Переключаем состояние сервера в NEURO_DRIVEN
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            state_of_action_ = StateOfAction::NEURO_DRIVEN;
        }

        // Ждём, пока нейрорежим активен и не истёк таймаут
        while (rclcpp::ok() && neural_manager_->isActive() && !neural_manager_->isTimeoutExpired()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        soft_disable();

        if (current_goal_handle_) {
            auto result = std::make_shared<MotionPlan::Result>();
            result->success = true;
            current_goal_handle_->succeed(result);
            current_goal_handle_.reset();
        }
        RCLCPP_INFO(get_logger(), "Neural action finished");
    }

    // ====================================================================
    // Члены класса
    // ====================================================================
    rclcpp_action::Server<MotionPlan>::SharedPtr action_server_;
    std::shared_ptr<GoalHandleMotionPlan> current_goal_handle_;
    rclcpp::Client<MotionPlanService>::SharedPtr planning_client_;
    rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr validity_client_;

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_cv_;

    rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr lowcmd_publisher_;
    rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowstate_subscriber_;
    rclcpp::TimerBase::SharedPtr command_generator_timer_;
    rclcpp::TimerBase::SharedPtr commander_timer_;

    DataBuffer<MotorCommand> current_command_buffer_;   // Буфер текущих команд
    DataBuffer<MotorState> motor_state_buffer_;

    std::vector<double> current_handle_xyz_;
    TrajectoryManager trajectory_manager_;              // Буфер траекторий

    // Управление нейросетевыми действиями
    std::unique_ptr<NeuralActionManager> neural_manager_;
    std::shared_ptr<unitree_hg::msg::LowState> last_low_state_;
    
    // Для асинхронной проверки коллизий
    std::shared_future<bool> collision_future_;
    std::vector<float> last_safe_neural_cmd_;
    bool is_waiting_for_collision_result_;

    std::unique_ptr<dftp_hand::HandController> hand_controller_; 
    // Прокси-узел для кистей (чтобы не уничтожился после конструктора)
    rclcpp::Node::SharedPtr hand_proxy_node_;

    std::mutex state_mutex_;
    EnableControl enable_control_;
    StateOfAction state_of_action_;

    const std::unordered_map<std::string, int> JOINT_NAME_TO_INDEX = {
        {"left_shoulder_pitch_joint", LEFT_SHOULDER_PITCH},
        {"left_shoulder_roll_joint", LEFT_SHOULDER_ROLL},
        {"left_shoulder_yaw_joint", LEFT_SHOULDER_YAW},
        {"left_elbow_joint", LEFT_ELBOW},
        {"left_wrist_roll_joint", LEFT_WRIST_ROLL},
        {"left_wrist_pitch_joint", LEFT_WRIST_PITCH},
        {"left_wrist_yaw_joint", LEFT_WRIST_YAW},
        {"right_shoulder_pitch_joint", RIGHT_SHOULDER_PITCH},
        {"right_shoulder_roll_joint", RIGHT_SHOULDER_ROLL},
        {"right_shoulder_yaw_joint", RIGHT_SHOULDER_YAW},
        {"right_elbow_joint", RIGHT_ELBOW},
        {"right_wrist_roll_joint", RIGHT_WRIST_ROLL},
        {"right_wrist_pitch_joint", RIGHT_WRIST_PITCH},
        {"right_wrist_yaw_joint", RIGHT_WRIST_YAW}
    };
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto server = std::make_shared<ManipulationActionServer>();
    rclcpp::spin(server);
    rclcpp::shutdown();
    return 0;
}



