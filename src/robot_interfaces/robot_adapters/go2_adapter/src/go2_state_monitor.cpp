/**
 * @file   go2_state_monitor.cpp
 * @brief  Go2机器人状态监控器实现文件
 * @author Yang Nan
 * @date   2025-09-15
 *
 * @details
 * 本文件提供了 `Go2StateMonitor` 类的完整实现。该类负责订阅Go2机器人的
 * 各种状态话题（如运动状态、底层状态、电池状态等），对这些原始数据进行
 * 处理、转换和分析，最终生成统一的、结构化的机器人状态、健康状况和诊断信息。
 *
 * 主要功能包括：
 * - 订阅ROS2话题以接收来自Go2机器人的实时状态数据。
 * - 将特定于Go2的消息格式转换为通用的 `robot_base_interfaces` 格式。
 * - 基于接收到的数据，评估机器人的总体健康分数和健康等级。
 * - 根据预设规则（如低电量）生成并管理告警信息。
 * - 提供一系列接口，供上层应用查询机器人的当前状态、详细信息、诊断结果和性能统计。
 * - 通过回调函数机制，在状态发生变化时主动通知其他模块。
 */

#include "robot_adapters/go2_adapter/go2_state_monitor.hpp" // 引入对应的头文件

#include <rclcpp/rclcpp.hpp>    // ROS2 C++核心库
#include <algorithm>            // C++标准算法库
#include <chrono>               // 时间相关功能
#include <fstream>              // 文件流操作
#include <iomanip>              // 格式化输出 

namespace robot_adapters {
namespace go2_adapter {

// ============= 构造函数与析构函数 (Constructor & Destructor) =============

/**
 * @brief Go2StateMonitor类的构造函数
 * @param node_name ROS节点的名称。
 *
 * @details
 * 初始化ROS节点，并设置所有成员变量的初始值。
 * - 初始化状态标志 (`is_initialized_`, `is_monitoring_`)。
 * - 设置默认的机器人状态、健康等级和分数。
 * - 设置默认的监控频率和健康分数阈值。
 * - 初始化所有模块为启用状态。
 * - 将所有回调函数指针设置为空。
 */
Go2StateMonitor::Go2StateMonitor(const std::string& node_name)
    : rclcpp::Node(node_name),
      is_initialized_(false),
      is_monitoring_(false),
      current_state_(robot_base_interfaces::state_interface::RobotState::UNKNOWN),
      current_health_level_(robot_base_interfaces::state_interface::HealthLevel::UNKNOWN),
      current_health_score_(0.0f),
      current_error_code_(0),
      monitoring_frequency_(DEFAULT_MONITORING_FREQUENCY),
      excellent_threshold_(DEFAULT_EXCELLENT_THRESHOLD),
      good_threshold_(DEFAULT_GOOD_THRESHOLD),
      fair_threshold_(DEFAULT_FAIR_THRESHOLD),
      poor_threshold_(DEFAULT_POOR_THRESHOLD),
      start_time_(std::chrono::steady_clock::now()),
      next_alert_id_(1),
      is_recording_(false),
      recording_duration_(0)
{
    RCLCPP_INFO(this->get_logger(), "Go2 State Monitor node is initializing...");

    // 初始化详细状态结构体的时间戳
    detailed_state_.timestamp_ns = this->get_clock()->now().nanoseconds();

    // 默认启用所有可监控的模块
    module_enabled_[robot_base_interfaces::state_interface::SystemModule::MOTION_CONTROL] = true;
    module_enabled_[robot_base_interfaces::state_interface::SystemModule::SENSOR_SYSTEM] = true;
    module_enabled_[robot_base_interfaces::state_interface::SystemModule::POWER_MANAGEMENT] = true;
    module_enabled_[robot_base_interfaces::state_interface::SystemModule::COMMUNICATION] = true;
    module_enabled_[robot_base_interfaces::state_interface::SystemModule::NAVIGATION] = true;

    // 将所有回调函数初始化为空指针
    state_change_callback_ = nullptr;
    health_change_callback_ = nullptr;
    alert_callback_ = nullptr;
    error_callback_ = nullptr;
    detailed_state_callback_ = nullptr;

    // 初始化Go2专用组件
    try {
        message_converter_ = std::make_shared<Go2MessageConverter>();
        // Go2通信管理器将在initialize()方法中创建，因为需要shared_from_this()
        RCLCPP_INFO(this->get_logger(), "Go2 specialized components initialized successfully.");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize Go2 components: %s", e.what());
    }
}

// ============= IStateMonitor接口实现 (IStateMonitor Interface Implementation) =============

/**
 * @brief 初始化状态监控器
 * @return 如果初始化成功，返回true。
 *
 * @details
 * 创建所有必需的ROS2订阅者来接收Go2的状态消息，并重置性能统计。
 * 如果已经初始化，则直接返回true。
 */
bool Go2StateMonitor::initialize() {
    if (is_initialized_) {
        RCLCPP_WARN(this->get_logger(), "State monitor is already initialized, skipping.");
        return true;
    }

    RCLCPP_INFO(this->get_logger(), "Initializing Go2 state monitor...");
    try {
        // 创建Go2通信管理器
        // 使用rclcpp::Node的shared_from_this()获取节点shared_ptr
        auto node_ptr = rclcpp::Node::shared_from_this();
        go2_communication_ = std::make_shared<Go2Communication>(node_ptr);
        
        // 初始化Go2通信管理器
        if (!go2_communication_->initialize()) {
            throw std::runtime_error("Go2通信管理器初始化失败");
        }

        // 初始化ROS2通信组件（订阅者）
        initializeROS2Communications();

        is_initialized_ = true;
        current_error_code_ = 0;

        // 重置性能统计计数器
        resetPerformanceStats();

        RCLCPP_INFO(this->get_logger(), "Go2 state monitor initialized successfully.");
        return true;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize Go2 state monitor: %s", e.what());
        current_error_code_ = 2001; // 定义一个初始化失败的错误码
        triggerErrorCallback(current_error_code_, "Initialization failed: " + std::string(e.what()));
        return false;
    }
}

/**
 * @brief 关闭状态监控器
 * @return 如果关闭成功，返回true。
 *
 * @details
 * 停止监控，释放所有资源，包括定时器、ROS2订阅者和回调函数。
 */
bool Go2StateMonitor::shutdown() {
    RCLCPP_INFO(this->get_logger(), "Shutting down Go2 state monitor...");
    try {
        stopMonitoring();
        stopDataRecording(); // 停止任何可能在进行的数据记录

        // 停止Go2通信管理器
        if (go2_communication_) {
            go2_communication_->stopCommunication();
            go2_communication_->shutdown();
        }

        // 重置（销毁）所有ROS2实体
        monitoring_timer_.reset();

        // 清理所有回调函数
        state_change_callback_ = nullptr;
        health_change_callback_ = nullptr;
        alert_callback_ = nullptr;
        error_callback_ = nullptr;
        detailed_state_callback_ = nullptr;

        // 清理活动告警列表
        {
            std::lock_guard<std::mutex> lock(alerts_mutex_);
            active_alerts_.clear();
        }

        // 重置状态标志
        is_initialized_ = false;
        is_monitoring_ = false;
        current_error_code_ = 0;

        RCLCPP_INFO(this->get_logger(), "Go2 state monitor shut down safely.");
        return true;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error during Go2 state monitor shutdown: %s", e.what());
        return false;
    }
}

/**
 * @brief 开始状态监控
 * @return 如果成功启动，返回true。
 *
 * @details
 * 创建并启动一个定时器，该定时器以预设的频率 (`monitoring_frequency_`)
 * 周期性地调用 `monitoringTimerCallback` 函数来评估机器人健康状况。
 */
bool Go2StateMonitor::startMonitoring() {
    if (!is_initialized_) {
        RCLCPP_ERROR(this->get_logger(), "State monitor is not initialized. Cannot start monitoring.");
        return false;
    }
    if (is_monitoring_) {
        RCLCPP_WARN(this->get_logger(), "State monitor is already running.");
        return true;
    }

    RCLCPP_INFO(this->get_logger(), "Starting Go2 state monitoring at %.1f Hz", monitoring_frequency_);

    // 根据频率计算定时器周期
    auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0f / monitoring_frequency_));
    monitoring_timer_ = this->create_wall_timer(
        timer_period,
        std::bind(&Go2StateMonitor::monitoringTimerCallback, this)
    );

    is_monitoring_ = true;
    start_time_ = std::chrono::steady_clock::now(); // 重置启动时间
    return true;
}

/**
 * @brief 停止状态监控
 * @return 总是返回true。
 *
 * @details
 * 停止并销毁监控定时器，并将 `is_monitoring_` 标志设置为false。
 */
bool Go2StateMonitor::stopMonitoring() {
    if (!is_monitoring_) {
        return true;
    }
    RCLCPP_INFO(this->get_logger(), "Stopping Go2 state monitoring.");
    if (monitoring_timer_) {
        monitoring_timer_->cancel();
        monitoring_timer_.reset();
    }
    is_monitoring_ = false;
    return true;
}

/**
 * @brief 检查监控器是否正在运行
 * @return 如果正在监控，返回true。
 */
bool Go2StateMonitor::isMonitoring() const {
    return is_monitoring_;
}

// ============= 基础状态查询实现 (Basic State Query Implementation) =============

/**
 * @brief 获取机器人当前的主要状态
 */
robot_base_interfaces::state_interface::RobotState Go2StateMonitor::getRobotState() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return current_state_;
}

/**
 * @brief 获取机器人当前的健康等级
 */
robot_base_interfaces::state_interface::HealthLevel Go2StateMonitor::getHealthStatus() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return current_health_level_;
}

/**
 * @brief 获取机器人当前的健康分数（0.0 - 1.0）
 */
float Go2StateMonitor::getHealthScore() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return current_health_score_;
}

/**
 * @brief 判断机器人是否处于可操作状态
 * @details 综合考虑错误码、健康等级和机器人状态来判断。
 */
bool Go2StateMonitor::isOperational() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return (current_error_code_ == 0 &&
            current_health_level_ != robot_base_interfaces::state_interface::HealthLevel::CRITICAL &&
            current_state_ != robot_base_interfaces::state_interface::RobotState::ERROR &&
            current_state_ != robot_base_interfaces::state_interface::RobotState::EMERGENCY_STOP);
}

/**
 * @brief 获取当前的错误码
 */
uint32_t Go2StateMonitor::getErrorCode() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return current_error_code_;
}

// ============= 详细状态查询实现 (Detailed State Query Implementation) =============

/**
 * @brief 获取包含所有详细信息的机器人状态结构体
 */
robot_base_interfaces::state_interface::DetailedRobotState Go2StateMonitor::getDetailedState() const {
    std::lock_guard<std::mutex> lock(detailed_state_mutex_);
    return detailed_state_;
}

/**
 * @brief 获取单个电机的信息
 * @param motor_id 电机ID (0-11)。
 */
robot_base_interfaces::state_interface::MotorInfo Go2StateMonitor::getMotorInfo(uint8_t motor_id) const {
    if (motor_id >= GO2_MOTOR_COUNT) {
        RCLCPP_WARN(this->get_logger(), "Motor ID %d is out of range [0-%d].", motor_id, GO2_MOTOR_COUNT - 1);
        return {}; // 返回一个默认构造的空对象
    }
    std::lock_guard<std::mutex> lock(detailed_state_mutex_);
    if (motor_id < detailed_state_.motors.size()) {
        return detailed_state_.motors[motor_id];
    }
    return {};
}

/**
 * @brief 获取所有电机的信息
 */
std::vector<robot_base_interfaces::state_interface::MotorInfo> Go2StateMonitor::getAllMotorInfo() const {
    std::lock_guard<std::mutex> lock(detailed_state_mutex_);
    return detailed_state_.motors;
}

/**
 * @brief 获取单个足端的信息
 * @param foot_id 足端ID (0-3)。
 */
robot_base_interfaces::state_interface::FootInfo Go2StateMonitor::getFootInfo(uint8_t foot_id) const {
    if (foot_id >= GO2_FOOT_COUNT) {
        RCLCPP_WARN(this->get_logger(), "Foot ID %d is out of range [0-%d].", foot_id, GO2_FOOT_COUNT - 1);
        return {};
    }
    std::lock_guard<std::mutex> lock(detailed_state_mutex_);
    if (foot_id < detailed_state_.feet.size()) {
        return detailed_state_.feet[foot_id];
    }
    return {};
}

/**
 * @brief 获取所有足端的信息
 */
std::vector<robot_base_interfaces::state_interface::FootInfo> Go2StateMonitor::getAllFootInfo() const {
    std::lock_guard<std::mutex> lock(detailed_state_mutex_);
    return detailed_state_.feet;
}

// ============= 系统诊断实现 (System Diagnostics Implementation) =============

/**
 * @brief 获取所有支持模块的诊断信息
 */
std::vector<robot_base_interfaces::state_interface::DiagnosticInfo> Go2StateMonitor::getSystemDiagnostics() const {
    std::vector<robot_base_interfaces::state_interface::DiagnosticInfo> diagnostics;
    auto supported_modules = getSupportedModules();
    for (const auto& module : supported_modules) {
        diagnostics.push_back(getModuleDiagnostic(module));
    }
    return diagnostics;
}

/**
 * @brief 获取单个模块的诊断信息
 * @details 这是一个简化的实现，主要基于 `checkModuleHealth` 的结果。
 */
robot_base_interfaces::state_interface::DiagnosticInfo Go2StateMonitor::getModuleDiagnostic(
    robot_base_interfaces::state_interface::SystemModule module) const {

    robot_base_interfaces::state_interface::DiagnosticInfo diagnostic;
    diagnostic.module = module;
    bool is_healthy = checkModuleHealth(module);

    // 根据模块类型设置描述信息
    switch (module) {
        case robot_base_interfaces::state_interface::SystemModule::MOTION_CONTROL:
            diagnostic.status_message = "Go2 Motion Control System Status";
            break;
        case robot_base_interfaces::state_interface::SystemModule::SENSOR_SYSTEM:
            diagnostic.status_message = "Lidar, IMU, and other sensor status";
            break;
        case robot_base_interfaces::state_interface::SystemModule::POWER_MANAGEMENT:
            diagnostic.status_message = "Battery and power system status";
            break;
        case robot_base_interfaces::state_interface::SystemModule::COMMUNICATION:
            diagnostic.status_message = "DDS and network communication status";
            break;
        case robot_base_interfaces::state_interface::SystemModule::NAVIGATION:
            diagnostic.status_message = "SLAM and path planning status";
            break;
        default:
            diagnostic.status_message = "Unrecognized system module";
            is_healthy = false;
    }

    // 根据健康状况设置健康等级和分数
    if (is_healthy) {
        diagnostic.health_level = robot_base_interfaces::state_interface::HealthLevel::EXCELLENT;
        diagnostic.health_score = 1.0f;
    } else {
        diagnostic.health_level = robot_base_interfaces::state_interface::HealthLevel::CRITICAL;
        diagnostic.health_score = 0.0f;
    }
    diagnostic.last_update_ns = 0; // 在const方法中无法获取时钟，用0占位
    return diagnostic;
}

/**
 * @brief 执行一次完整的系统自检
 * @return 如果所有启用的模块都健康，返回true。
 */
bool Go2StateMonitor::performSystemCheck() {
    RCLCPP_INFO(this->get_logger(), "Performing system self-check...");
    std::lock_guard<std::mutex> lock(diagnostic_mutex_);
    bool all_systems_ok = true;
    auto supported_modules = getSupportedModules();

    for (const auto& module : supported_modules) {
        if (module_enabled_.at(module)) {
            bool module_ok = checkModuleHealth(module);
            system_check_results_[module] = module_ok;
            all_systems_ok &= module_ok;
            RCLCPP_DEBUG(this->get_logger(), "Module %d self-check result: %s",
                        static_cast<int>(module), module_ok ? "OK" : "FAIL");
        }
    }
    RCLCPP_INFO(this->get_logger(), "System self-check completed. Result: %s", all_systems_ok ? "OK" : "Found issues");
    return all_systems_ok;
}

/**
 * @brief 获取最近一次系统自检的结果
 */
std::map<robot_base_interfaces::state_interface::SystemModule, bool> Go2StateMonitor::getSystemCheckResults() const {
    std::lock_guard<std::mutex> lock(diagnostic_mutex_);
    return system_check_results_;
}

// ============= 私有方法实现 (Private Methods Implementation) =============

/**
 * @brief 初始化ROS2的订阅者
 */
void Go2StateMonitor::initializeROS2Communications() {
    RCLCPP_INFO(this->get_logger(), "Initializing ROS2 communication components...");

    if (!go2_communication_) {
        RCLCPP_ERROR(this->get_logger(), "Go2 communication manager is not initialized");
        return;
    }

    // 使用封装的通信管理器设置回调函数
    go2_communication_->setSportModeStateCallback(
        std::bind(&Go2StateMonitor::sportModeStateCallback, this, std::placeholders::_1)
    );

    go2_communication_->setLowStateCallback(
        std::bind(&Go2StateMonitor::lowStateCallback, this, std::placeholders::_1)
    );

    // 启动Go2通信
    if (!go2_communication_->startCommunication()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start Go2 communication");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "ROS2 communication components initialized.");
}

/**
 * @brief `/sportmodestate` 话题的回调函数
 * @details 处理运动状态消息，更新机器人的主要状态、错误码以及足端的位置和速度。
 */
void Go2StateMonitor::sportModeStateCallback(const unitree_go::msg::SportModeState::SharedPtr msg) {
    // 使用消息转换器转换运动状态
    robot_base_interfaces::motion_interface::MotionState motion_state;
    auto result = message_converter_->convertSportModeState(*msg, motion_state);

    if (result == go2_adapter::ConversionResult::SUCCESS) {
        // 更新详细状态中的运动信息
        {
            std::lock_guard<std::mutex> lock(detailed_state_mutex_);
            detailed_state_.timestamp_ns = this->get_clock()->now().nanoseconds();
            // 将转换后的运动状态映射到详细状态
            detailed_state_.motion.mode = static_cast<uint8_t>(motion_state.current_mode);
            detailed_state_.motion.gait_type = static_cast<uint8_t>(motion_state.current_gait);
            detailed_state_.motion.progress = motion_state.motion_progress;
            detailed_state_.motion.position.x = motion_state.position.x;
            detailed_state_.motion.position.y = motion_state.position.y;
            detailed_state_.motion.position.z = motion_state.position.z;
            detailed_state_.motion.velocity.x = motion_state.velocity.linear_x;
            detailed_state_.motion.velocity.y = motion_state.velocity.linear_y;
            detailed_state_.motion.velocity.z = motion_state.velocity.linear_z;
            detailed_state_.motion.yaw_speed = motion_state.velocity.angular_z;
            detailed_state_.motion.body_height = motion_state.posture.body_height;
            detailed_state_.motion.foot_raise_height = motion_state.foot_raise_height;
        }
    }

    // 转换并更新机器人状态
    auto new_robot_state = convertGo2StateToRobotState(msg);
    updateRobotState(new_robot_state);

    // 更新错误码（根据Go2的mode判断是否有异常）
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        // 根据Go2的mode判断是否有错误
        if (msg->mode == 6 || msg->mode == 7) { // jointLock或damping模式通常表示异常
            current_error_code_ = 2001; // 异常模式错误码
        } else if (msg->mode < 0 || msg->mode > 13) {
            current_error_code_ = 1001; // 未知模式错误
        } else {
            current_error_code_ = 0; // 正常状态
        }
    }

    updateFootPositionAndVelocity(msg);

    // 基于运动状态评估健康等级
    evaluateHealthFromSportState(msg);

    RCLCPP_DEBUG(this->get_logger(), "Received Go2 sport state update. Mode: %d, Current Mode Code: %d",
                msg->mode, msg->error_code);
}

/**
 * @brief `/lowstate` 话题的回调函数
 * @details 处理底层状态消息，更新所有电机的详细信息（温度、位置、速度、力矩）和足端的接触力信息。
 */
void Go2StateMonitor::lowStateCallback(const unitree_go::msg::LowState::SharedPtr msg) {
    // 使用消息转换器转换电机信息
    std::vector<robot_base_interfaces::state_interface::MotorInfo> motors;
    std::vector<unitree_go::msg::MotorState> first_12_motors(
        msg->motor_state.begin(),
        msg->motor_state.begin() + std::min(static_cast<size_t>(12), msg->motor_state.size())
    );
    auto motor_result = message_converter_->convertMotorInfo(first_12_motors, motors);

    // 使用消息转换器转换IMU信息
    decltype(robot_base_interfaces::state_interface::DetailedRobotState{}.imu) imu_info;
    auto imu_result = message_converter_->convertIMUInfo(msg->imu_state, imu_info);

    {
        std::lock_guard<std::mutex> lock(detailed_state_mutex_);
        detailed_state_.timestamp_ns = this->get_clock()->now().nanoseconds();

        // 如果转换成功，更新相应信息
        if (motor_result == go2_adapter::ConversionResult::SUCCESS) {
            detailed_state_.motors = motors;
        } else {
            // 失败时使用原有方法
            detailed_state_.motors = convertGo2MotorInfo(msg);
        }

        if (imu_result == go2_adapter::ConversionResult::SUCCESS) {
            detailed_state_.imu = imu_info;
        }

        // 更新系统电压电流信息
        detailed_state_.system_v = msg->power_v;
        detailed_state_.system_a = msg->power_a;

        updateFootForceInfo(msg);
    }

    // 基于底层状态评估健康等级
    evaluateHealthFromLowState(msg);

    RCLCPP_DEBUG(this->get_logger(), "Received Go2 low state update.");
}


/**
 * @brief 监控定时器的回调函数
 * @details 定期调用 `calculateHealthScore` 来评估并更新机器人的健康状态。
 */
void Go2StateMonitor::monitoringTimerCallback() {
    float new_health_score = calculateHealthScore();
    robot_base_interfaces::state_interface::HealthLevel new_health_level;

    // 根据健康分数确定健康等级
    if (new_health_score >= excellent_threshold_) {
        new_health_level = robot_base_interfaces::state_interface::HealthLevel::EXCELLENT;
    } else if (new_health_score >= good_threshold_) {
        new_health_level = robot_base_interfaces::state_interface::HealthLevel::GOOD;
    } else if (new_health_score >= fair_threshold_) {
        new_health_level = robot_base_interfaces::state_interface::HealthLevel::FAIR;
    } else if (new_health_score >= poor_threshold_) {
        new_health_level = robot_base_interfaces::state_interface::HealthLevel::POOR;
    } else {
        new_health_level = robot_base_interfaces::state_interface::HealthLevel::CRITICAL;
    }

    updateHealthStatus(new_health_level, new_health_score);
}

// ============= 告警管理实现 (Alert Management Implementation) =============

std::vector<robot_base_interfaces::state_interface::AlertInfo> Go2StateMonitor::getActiveAlerts() const {
    std::lock_guard<std::mutex> lock(alerts_mutex_);
    return active_alerts_;
}

std::vector<robot_base_interfaces::state_interface::AlertInfo> Go2StateMonitor::getAlertsByType(
    robot_base_interfaces::state_interface::AlertType type) const {
    std::lock_guard<std::mutex> lock(alerts_mutex_);
    std::vector<robot_base_interfaces::state_interface::AlertInfo> filtered_alerts;
    for (const auto& alert : active_alerts_) {
        if (alert.type == type) {
            filtered_alerts.push_back(alert);
        }
    }
    return filtered_alerts;
}

std::vector<robot_base_interfaces::state_interface::AlertInfo> Go2StateMonitor::getAlertsByModule(
    robot_base_interfaces::state_interface::SystemModule module) const {
    std::lock_guard<std::mutex> lock(alerts_mutex_);
    std::vector<robot_base_interfaces::state_interface::AlertInfo> filtered_alerts;
    for (const auto& alert : active_alerts_) {
        if (alert.module == module) {
            filtered_alerts.push_back(alert);
        }
    }
    return filtered_alerts;
}

/**
 * @brief 清除已解决的告警
 * @param alert_code 要清除的告警码。如果为0，则清除所有已解决的告警。
 * @return 清除的告警数量。
 */
int Go2StateMonitor::clearResolvedAlerts(uint32_t alert_code) {
    std::lock_guard<std::mutex> lock(alerts_mutex_);
    int cleared_count = 0;
    auto it = active_alerts_.begin();
    while (it != active_alerts_.end()) {
        bool should_remove = (alert_code == 0) ? (!it->is_active) : (it->code == alert_code && !it->is_active);
        if (should_remove) {
            it = active_alerts_.erase(it);
            cleared_count++;
        } else {
            ++it;
        }
    }
    return cleared_count;
}

/**
 * @brief 确认一个告警
 * @param alert_code 要确认的告警码。
 * @return 如果找到并确认了告警，返回true。
 */
bool Go2StateMonitor::acknowledgeAlert(uint32_t alert_code) {
    std::lock_guard<std::mutex> lock(alerts_mutex_);
    for (auto& alert : active_alerts_) {
        if (alert.code == alert_code) {
            alert.string_data["status"] = "acknowledged";
            alert.string_data["acknowledged_time"] = std::to_string(this->get_clock()->now().nanoseconds());
            RCLCPP_INFO(this->get_logger(), "Alert %u has been acknowledged.", alert_code);
            return true;
        }
    }
    return false;
}

// ============= 性能与配置实现 (Performance & Configuration Implementation) =============

robot_base_interfaces::state_interface::PerformanceStats Go2StateMonitor::getPerformanceStats() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return performance_stats_;
}

bool Go2StateMonitor::resetPerformanceStats() {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    performance_stats_ = {}; // 重置为默认值
    start_time_ = std::chrono::steady_clock::now();
    return true;
}

uint64_t Go2StateMonitor::getUptimeSeconds() const {
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - start_time_);
    return static_cast<uint64_t>(duration.count());
}

// ============= 回调注册实现 (Callback Registration Implementation) =============

void Go2StateMonitor::setStateChangeCallback(
    std::function<void(robot_base_interfaces::state_interface::RobotState,
                      robot_base_interfaces::state_interface::RobotState)> callback) {
    state_change_callback_ = callback;
}

void Go2StateMonitor::setHealthChangeCallback(
    std::function<void(robot_base_interfaces::state_interface::HealthLevel,
                      robot_base_interfaces::state_interface::HealthLevel,
                      float)> callback) {
    health_change_callback_ = callback;
}

void Go2StateMonitor::setAlertCallback(
    std::function<void(const robot_base_interfaces::state_interface::AlertInfo&)> callback) {
    alert_callback_ = callback;
}

void Go2StateMonitor::setErrorCallback(
    std::function<void(uint32_t, const std::string&)> callback) {
    error_callback_ = callback;
}

void Go2StateMonitor::setDetailedStateCallback(
    std::function<void(const robot_base_interfaces::state_interface::DetailedRobotState&)> callback) {
    detailed_state_callback_ = callback;
}

// ============= 配置管理实现 (Configuration Management Implementation) =============

/**
 * @brief 设置监控频率
 * @details 如果监控器正在运行，会重置定时器以应用新的频率。
 */
bool Go2StateMonitor::setMonitoringFrequency(float frequency) {
    if (frequency <= 0.0f || frequency > 100.0f) {
        return false;
    }
    monitoring_frequency_ = frequency;
    if (is_monitoring_) {
        // 重新启动定时器以应用新频率
        stopMonitoring();
        startMonitoring();
    }
    return true;
}

/**
 * @brief 设置健康分数阈值
 */
bool Go2StateMonitor::setHealthThresholds(float excellent, float good, float fair, float poor) {
    if (excellent <= good || good <= fair || fair <= poor || poor < 0.0f || excellent > 1.0f) {
        return false;
    }
    excellent_threshold_ = excellent;
    good_threshold_ = good;
    fair_threshold_ = fair;
    poor_threshold_ = poor;
    return true;
}

/**
 * @brief 启用或禁用特定模块的监控
 */
bool Go2StateMonitor::setModuleMonitoring(
    robot_base_interfaces::state_interface::SystemModule module, bool enabled) {
    std::lock_guard<std::mutex> lock(diagnostic_mutex_);
    module_enabled_[module] = enabled;
    return true;
}

// ============= 数据记录与导出实现 (Data Recording & Export Implementation) =============
// (以下为占位符实现)

bool Go2StateMonitor::startDataRecording(uint32_t duration_seconds) {
    std::lock_guard<std::mutex> lock(recording_mutex_);

    if (is_recording_) {
        RCLCPP_WARN(this->get_logger(), "数据记录已经在进行中，无法启动新的记录。");
        return false;
    }

    if (!is_monitoring_) {
        RCLCPP_ERROR(this->get_logger(), "状态监控器未启动，无法开始数据记录。");
        return false;
    }

    // 清空之前的记录数据
    recorded_states_.clear();

    // 设置记录参数
    is_recording_ = true;
    recording_duration_ = duration_seconds;
    recording_start_time_ = std::chrono::steady_clock::now();

    // 创建数据记录定时器
    recording_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0f / RECORDING_FREQUENCY)),
        std::bind(&Go2StateMonitor::recordingTimerCallback, this)
    );

    if (duration_seconds == 0) {
        RCLCPP_INFO(this->get_logger(), "开始持续数据记录 (频率: %.1f Hz)", RECORDING_FREQUENCY);
    } else {
        RCLCPP_INFO(this->get_logger(), "开始数据记录，持续时间: %d 秒 (频率: %.1f Hz)",
                   duration_seconds, RECORDING_FREQUENCY);
    }

    return true;
}

bool Go2StateMonitor::stopDataRecording() {
    std::lock_guard<std::mutex> lock(recording_mutex_);

    if (!is_recording_) {
        RCLCPP_WARN(this->get_logger(), "数据记录未在进行中，无需停止。");
        return false;
    }

    // 停止记录定时器
    if (recording_timer_) {
        recording_timer_->cancel();
        recording_timer_.reset();
    }

    // 更新记录状态
    is_recording_ = false;

    // 计算记录时长
    auto recording_end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(
        recording_end_time - recording_start_time_).count();

    RCLCPP_INFO(this->get_logger(), "数据记录已停止。记录时长: %ld 秒，总计记录 %zu 条状态数据。",
               duration, recorded_states_.size());

    return true;
}

bool Go2StateMonitor::exportStateData(const std::string& file_path, const std::string& format) {
    std::lock_guard<std::mutex> lock(recording_mutex_);

    if (recorded_states_.empty()) {
        RCLCPP_WARN(this->get_logger(), "没有可导出的状态数据。");
        return false;
    }

    try {
        if (format == "json") {
            return exportToJSON(file_path);
        } else if (format == "csv") {
            return exportToCSV(file_path);
        } else if (format == "binary") {
            return exportToBinary(file_path);
        } else {
            RCLCPP_ERROR(this->get_logger(), "不支持的导出格式: %s。支持的格式: json, csv, binary",
                        format.c_str());
            return false;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "导出数据时发生错误: %s", e.what());
        return false;
    }
}

std::vector<robot_base_interfaces::state_interface::SystemModule> Go2StateMonitor::getSupportedModules() const {
    return {
        robot_base_interfaces::state_interface::SystemModule::MOTION_CONTROL,
        robot_base_interfaces::state_interface::SystemModule::SENSOR_SYSTEM,
        robot_base_interfaces::state_interface::SystemModule::POWER_MANAGEMENT,
        robot_base_interfaces::state_interface::SystemModule::COMMUNICATION,
        robot_base_interfaces::state_interface::SystemModule::NAVIGATION
    };
}

// ============= 受保护的内部方法 (Protected Internal Methods) =============

/**
 * @brief 更新机器人状态并触发回调
 */
void Go2StateMonitor::updateRobotState(robot_base_interfaces::state_interface::RobotState new_state) {
    robot_base_interfaces::state_interface::RobotState old_state;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (current_state_ == new_state) return; // 状态未变，无需更新
        old_state = current_state_;
        current_state_ = new_state;
    }
    triggerStateChangeCallback(old_state, new_state);
}

/**
 * @brief 更新健康状态并触发回调
 */
void Go2StateMonitor::updateHealthStatus(robot_base_interfaces::state_interface::HealthLevel new_level, float score) {
    robot_base_interfaces::state_interface::HealthLevel old_level;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (current_health_level_ == new_level && current_health_score_ == score) return;
        old_level = current_health_level_;
        current_health_level_ = new_level;
        current_health_score_ = score;
    }
    triggerHealthChangeCallback(old_level, new_level, score);
}

/**
 * @brief 添加一个新告警并触发回调
 * @details 如果已存在相同代码的告警，则不会重复添加。
 */
void Go2StateMonitor::addAlert(const robot_base_interfaces::state_interface::AlertInfo& alert) {
    {
        std::lock_guard<std::mutex> lock(alerts_mutex_);
        // 检查是否存在相同代码的活动告警
        for (const auto& existing_alert : active_alerts_) {
            if (existing_alert.code == alert.code && existing_alert.is_active) {
                return; // 如果已存在，则不重复添加
            }
        }
        active_alerts_.push_back(alert);
    }
    triggerAlertCallback(alert);
}

/**
 * @brief 计算机器人的综合健康分数
 * @return 0.0到1.0之间的浮点数。
 * @details
 * 这是一个简化的健康评估模型，它根据以下几个方面加权计算分数：
 * - 基础系统错误码 (权重 0.3)
 * - 机器人运行状态 (权重 0.2)
 * - 当前活动的告警数量和严重性 (权重 0.3)
 * - 各子模块的健康状况 (权重 0.2)
 */
float Go2StateMonitor::calculateHealthScore() const {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    std::lock_guard<std::mutex> alerts_lock(alerts_mutex_);

    float total_score = 0.0f;
    // 总权重为1.0，由各个分量的权重组成

    // 1. 基于错误码的分数 (权重 30%)
    float system_score = (current_error_code_ == 0) ? 1.0f : 0.0f;
    total_score += system_score * 0.3f;

    // 2. 基于机器人状态的分数 (权重 20%)
    float robot_state_score = 1.0f;
    switch (current_state_) {
        case robot_base_interfaces::state_interface::RobotState::ERROR: robot_state_score = 0.2f; break;
        case robot_base_interfaces::state_interface::RobotState::EMERGENCY_STOP: robot_state_score = 0.0f; break;
        case robot_base_interfaces::state_interface::RobotState::LOW_POWER: robot_state_score = 0.4f; break;
        default: break;
    }
    total_score += robot_state_score * 0.2f;

    // 3. 基于告警的分数 (权重 30%)
    float alert_score = 1.0f;
    for (const auto& alert : active_alerts_) {
        if (!alert.is_active) continue;
        if (alert.type == robot_base_interfaces::state_interface::AlertType::CRITICAL) alert_score -= 0.5f;
        if (alert.type == robot_base_interfaces::state_interface::AlertType::ERROR) alert_score -= 0.25f;
        if (alert.type == robot_base_interfaces::state_interface::AlertType::WARNING) alert_score -= 0.1f;
    }
    total_score += std::max(0.0f, alert_score) * 0.3f;

    // 4. 基于模块健康的分数 (权重 20%)
    float module_score = 0.0f;
    int enabled_modules = 0;
    int healthy_modules = 0;
    for (const auto& [module, enabled] : module_enabled_) {
        if (enabled) {
            enabled_modules++;
            if (checkModuleHealth(module)) healthy_modules++;
        }
    }
    module_score = (enabled_modules > 0) ? static_cast<float>(healthy_modules) / enabled_modules : 1.0f;
    total_score += module_score * 0.2f;

    return std::clamp(total_score, 0.0f, 1.0f);
}

// ============= 辅助方法实现 (Helper Methods Implementation) =============

/**
 * @brief 检查单个模块的健康状况（简化实现）
 */
bool Go2StateMonitor::checkModuleHealth(robot_base_interfaces::state_interface::SystemModule module) const {
    if (!module_enabled_.at(module)) {
        return true; // 如果模块被禁用，则认为它是健康的
    }
    switch (module) {
        case robot_base_interfaces::state_interface::SystemModule::MOTION_CONTROL:
            return (current_error_code_ == 0); // 运动控制的健康依赖于无错误码
        case robot_base_interfaces::state_interface::SystemModule::POWER_MANAGEMENT:
            // 检查是否有严重的电源告警
            for(const auto& alert : active_alerts_) {
                if(alert.module == module && alert.type == robot_base_interfaces::state_interface::AlertType::CRITICAL) return false;
            }
            return true;
        // 其他模块暂时简化为总是健康
        case robot_base_interfaces::state_interface::SystemModule::SENSOR_SYSTEM:
        case robot_base_interfaces::state_interface::SystemModule::COMMUNICATION:
        case robot_base_interfaces::state_interface::SystemModule::NAVIGATION:
            return true;
        default:
            return false;
    }
}

// --- 触发回调的辅助函数，增加了异常捕获 ---

void Go2StateMonitor::triggerStateChangeCallback(
    robot_base_interfaces::state_interface::RobotState old_state,
    robot_base_interfaces::state_interface::RobotState new_state) {
    if (state_change_callback_) {
        try { state_change_callback_(old_state, new_state); }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in state change callback: %s", e.what());
        }
    }
}

void Go2StateMonitor::triggerHealthChangeCallback(
    robot_base_interfaces::state_interface::HealthLevel old_level,
    robot_base_interfaces::state_interface::HealthLevel new_level,
    float score) {
    if (health_change_callback_) {
        try { health_change_callback_(old_level, new_level, score); }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in health change callback: %s", e.what());
        }
    }
}

void Go2StateMonitor::triggerAlertCallback(const robot_base_interfaces::state_interface::AlertInfo& alert) {
    if (alert_callback_) {
        try { alert_callback_(alert); }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in alert callback: %s", e.what());
        }
    }
}

void Go2StateMonitor::triggerErrorCallback(uint32_t error_code, const std::string& error_msg) {
    if (error_callback_) {
        try { error_callback_(error_code, error_msg); }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in error callback: %s", e.what());
        }
    }
}

// --- 数据转换函数 ---

/**
 * @brief 将Go2的运动模式转换为统一的机器人状态枚举
 */
robot_base_interfaces::state_interface::RobotState Go2StateMonitor::convertGo2StateToRobotState(
    const unitree_go::msg::SportModeState::SharedPtr& sport_state) const {
    if (!sport_state) return robot_base_interfaces::state_interface::RobotState::UNKNOWN;

    switch (sport_state->mode) {
        case 0:  // idle - 默认站立状态
        case 1:  // balanceStand - 平衡站立
            return robot_base_interfaces::state_interface::RobotState::STANDBY;
        case 2:  // pose - 姿态控制
            return robot_base_interfaces::state_interface::RobotState::ACTIVE;
        case 3:  // locomotion - 运动模式
            return robot_base_interfaces::state_interface::RobotState::MOVING;
        case 5:  // lieDown - 趴下
            return robot_base_interfaces::state_interface::RobotState::STANDBY;
        case 6:  // jointLock - 关节锁定
        case 7:  // damping - 阻尼模式
            return robot_base_interfaces::state_interface::RobotState::ERROR;
        case 8:  // recoveryStand - 恢复站立
            return robot_base_interfaces::state_interface::RobotState::ACTIVE;
        case 10: // sit - 坐下
            return robot_base_interfaces::state_interface::RobotState::STANDBY;
        case 11: // frontFlip - 前空翻
        case 12: // frontJump - 前跳
        case 13: // frontPounce - 前扑
            return robot_base_interfaces::state_interface::RobotState::ACTIVE;
        case 4:  // reserve - 预留
        case 9:  // reserve - 预留
        default:
            return robot_base_interfaces::state_interface::RobotState::UNKNOWN;
    }
}

/**
 * @brief 将Go2的底层状态消息中的电机信息转换为统一的电机信息结构体向量
 */
std::vector<robot_base_interfaces::state_interface::MotorInfo> Go2StateMonitor::convertGo2MotorInfo(
    const unitree_go::msg::LowState::SharedPtr& low_state) const {
    std::vector<robot_base_interfaces::state_interface::MotorInfo> motor_infos;
    if (!low_state) return motor_infos;

    size_t motor_count = std::min(static_cast<size_t>(GO2_MOTOR_COUNT), low_state->motor_state.size());
    motor_infos.reserve(motor_count);

    for (size_t i = 0; i < motor_count; ++i) {
        const auto& go2_motor = low_state->motor_state[i];
        robot_base_interfaces::state_interface::MotorInfo motor_info;
        motor_info.motor_id = static_cast<uint8_t>(i);
        motor_info.temperature = go2_motor.temperature;
        motor_info.position = go2_motor.q;
        motor_info.velocity = go2_motor.dq;
        motor_info.torque_estimated = go2_motor.tau_est;
        motor_info.is_online = true; // 假设只要收到消息就是在线的
        motor_infos.push_back(motor_info);
    }
    return motor_infos;
}

/**
 * @brief 从底层状态消息中更新足端的接触力信息
 */
void Go2StateMonitor::updateFootForceInfo(const unitree_go::msg::LowState::SharedPtr& low_state) {
    // 确保足端信息数组已初始化
    if (detailed_state_.feet.size() != GO2_FOOT_COUNT) {
        detailed_state_.feet.resize(GO2_FOOT_COUNT);
        for (uint8_t i = 0; i < GO2_FOOT_COUNT; ++i) detailed_state_.feet[i].foot_id = i;
    }
    if (!low_state) return;

    // 更新每个足端的接触力和接触状态
    for (uint8_t i = 0; i < GO2_FOOT_COUNT && i < low_state->foot_force.size(); ++i) {
        detailed_state_.feet[i].force = low_state->foot_force[i];
        detailed_state_.feet[i].in_contact = (low_state->foot_force[i] > 10.0f); // 假设力大于10N为接触
    }
}

/**
 * @brief 从运动状态消息中更新足端的位置和速度信息
 */
void Go2StateMonitor::updateFootPositionAndVelocity(const unitree_go::msg::SportModeState::SharedPtr& sport_state) {
    if (!sport_state) return;
    std::lock_guard<std::mutex> lock(detailed_state_mutex_);

    if (detailed_state_.feet.size() != GO2_FOOT_COUNT) {
        detailed_state_.feet.resize(GO2_FOOT_COUNT);
        for (uint8_t i = 0; i < GO2_FOOT_COUNT; ++i) detailed_state_.feet[i].foot_id = i;
    }

    // Go2消息将4个足端的x,y,z分量平铺在一个数组中
    if (sport_state->foot_position_body.size() >= 12) {
        for (uint8_t i = 0; i < GO2_FOOT_COUNT; ++i) {
            size_t idx = i * 3;
            detailed_state_.feet[i].position.x = sport_state->foot_position_body[idx];
            detailed_state_.feet[i].position.y = sport_state->foot_position_body[idx + 1];
            detailed_state_.feet[i].position.z = sport_state->foot_position_body[idx + 2];
        }
    }
    if (sport_state->foot_speed_body.size() >= 12) {
        for (uint8_t i = 0; i < GO2_FOOT_COUNT; ++i) {
            size_t idx = i * 3;
            detailed_state_.feet[i].velocity.x = sport_state->foot_speed_body[idx];
            detailed_state_.feet[i].velocity.y = sport_state->foot_speed_body[idx + 1];
            detailed_state_.feet[i].velocity.z = sport_state->foot_speed_body[idx + 2];
        }
    }
}

/**
 * @brief 基于运动状态消息评估健康等级
 */
void Go2StateMonitor::evaluateHealthFromSportState(const unitree_go::msg::SportModeState::SharedPtr& msg) {
    if (!msg) return;

    std::vector<robot_base_interfaces::state_interface::AlertInfo> new_alerts;

    // 检查是否处于异常模式
    if (msg->mode == 6) { // jointLock - 关节锁定
        robot_base_interfaces::state_interface::AlertInfo alert;
        alert.code = 3001;
        alert.type = robot_base_interfaces::state_interface::AlertType::CRITICAL;
        alert.module = robot_base_interfaces::state_interface::SystemModule::MOTION_CONTROL;
        alert.message = "机器人处于关节锁定状态";
        alert.is_active = true;
        alert.timestamp_ns = this->get_clock()->now().nanoseconds();
        new_alerts.push_back(alert);
    } else if (msg->mode == 7) { // damping - 阻尼模式
        robot_base_interfaces::state_interface::AlertInfo alert;
        alert.code = 3002;
        alert.type = robot_base_interfaces::state_interface::AlertType::ERROR;
        alert.module = robot_base_interfaces::state_interface::SystemModule::MOTION_CONTROL;
        alert.message = "机器人处于阻尼模式";
        alert.is_active = true;
        alert.timestamp_ns = this->get_clock()->now().nanoseconds();
        new_alerts.push_back(alert);
    } else if (msg->mode < 0 || msg->mode > 13) { // 未知模式
        robot_base_interfaces::state_interface::AlertInfo alert;
        alert.code = 3003;
        alert.type = robot_base_interfaces::state_interface::AlertType::ERROR;
        alert.module = robot_base_interfaces::state_interface::SystemModule::MOTION_CONTROL;
        alert.message = "机器人处于未知运动模式: " + std::to_string(msg->mode);
        alert.is_active = true;
        alert.timestamp_ns = this->get_clock()->now().nanoseconds();
        new_alerts.push_back(alert);
    }

    // 检查足端位置是否异常（如果数据可用）
    if (msg->foot_position_body.size() >= 12) {
        bool foot_pos_abnormal = false;
        for (size_t i = 0; i < 12; i += 3) {
            float x = msg->foot_position_body[i];
            float y = msg->foot_position_body[i + 1];
            float z = msg->foot_position_body[i + 2];
            // 检查足端位置是否超出合理范围（根据Go2机器人的物理尺寸）
            if (std::abs(x) > 0.5f || std::abs(y) > 0.3f || std::abs(z) > 0.4f) {
                foot_pos_abnormal = true;
                break;
            }
        }
        if (foot_pos_abnormal) {
            robot_base_interfaces::state_interface::AlertInfo alert;
            alert.code = 3004;
            alert.type = robot_base_interfaces::state_interface::AlertType::WARNING;
            alert.module = robot_base_interfaces::state_interface::SystemModule::MOTION_CONTROL;
            alert.message = "足端位置异常";
            alert.is_active = true;
            alert.timestamp_ns = this->get_clock()->now().nanoseconds();
            new_alerts.push_back(alert);
        }
    }

    // 添加新的告警
    for (const auto& alert : new_alerts) {
        addAlert(alert);
    }
}

/**
 * @brief 基于底层状态消息评估健康等级
 */
void Go2StateMonitor::evaluateHealthFromLowState(const unitree_go::msg::LowState::SharedPtr& msg) {
    if (!msg) return;

    std::vector<robot_base_interfaces::state_interface::AlertInfo> new_alerts;

    // 检查电池电压
    float battery_voltage = msg->power_v;
    if (battery_voltage < 20.0f) { // 低电量临界值
        robot_base_interfaces::state_interface::AlertInfo alert;
        alert.code = 4001;
        alert.type = robot_base_interfaces::state_interface::AlertType::CRITICAL;
        alert.module = robot_base_interfaces::state_interface::SystemModule::POWER_MANAGEMENT;
        alert.message = "电量严重不足: " + std::to_string(battery_voltage) + "V";
        alert.is_active = true;
        alert.timestamp_ns = this->get_clock()->now().nanoseconds();
        new_alerts.push_back(alert);
    } else if (battery_voltage < 22.0f) { // 低电量警告值
        robot_base_interfaces::state_interface::AlertInfo alert;
        alert.code = 4002;
        alert.type = robot_base_interfaces::state_interface::AlertType::WARNING;
        alert.module = robot_base_interfaces::state_interface::SystemModule::POWER_MANAGEMENT;
        alert.message = "电量较低: " + std::to_string(battery_voltage) + "V";
        alert.is_active = true;
        alert.timestamp_ns = this->get_clock()->now().nanoseconds();
        new_alerts.push_back(alert);
    }

    // 检查电机温度
    const float MAX_MOTOR_TEMP = 90.0f;   // 电机过热阈值
    const float HIGH_MOTOR_TEMP = 80.0f;  // 电机温度偏高阈值

    for (size_t i = 0; i < std::min(static_cast<size_t>(12), msg->motor_state.size()); ++i) {
        float temp = msg->motor_state[i].temperature;
        if (temp > MAX_MOTOR_TEMP) {
            robot_base_interfaces::state_interface::AlertInfo alert;
            alert.code = 4100 + i; // 4100-4111 代表各个电机过热
            alert.type = robot_base_interfaces::state_interface::AlertType::CRITICAL;
            alert.module = robot_base_interfaces::state_interface::SystemModule::MOTION_CONTROL;
            alert.message = "电机" + std::to_string(i) + "过热: " + std::to_string(temp) + "°C";
            alert.is_active = true;
            alert.timestamp_ns = this->get_clock()->now().nanoseconds();
            new_alerts.push_back(alert);
        } else if (temp > HIGH_MOTOR_TEMP) {
            robot_base_interfaces::state_interface::AlertInfo alert;
            alert.code = 4200 + i; // 4200-4211 代表各个电机温度高
            alert.type = robot_base_interfaces::state_interface::AlertType::WARNING;
            alert.module = robot_base_interfaces::state_interface::SystemModule::MOTION_CONTROL;
            alert.message = "电机" + std::to_string(i) + "温度偏高: " + std::to_string(temp) + "°C";
            alert.is_active = true;
            alert.timestamp_ns = this->get_clock()->now().nanoseconds();
            new_alerts.push_back(alert);
        }
    }

    // 检查电流是否过大
    float current = msg->power_a;
    if (current > 15.0f) { // 过大电流阈值（Go2最大电流约20A）
        robot_base_interfaces::state_interface::AlertInfo alert;
        alert.code = 4003;
        alert.type = robot_base_interfaces::state_interface::AlertType::WARNING;
        alert.module = robot_base_interfaces::state_interface::SystemModule::POWER_MANAGEMENT;
        alert.message = "电流过大: " + std::to_string(current) + "A";
        alert.is_active = true;
        alert.timestamp_ns = this->get_clock()->now().nanoseconds();
        new_alerts.push_back(alert);
    }

    // 添加新的告警
    for (const auto& alert : new_alerts) {
        addAlert(alert);
    }
}

// ============= 数据记录私有助手函数实现 =============

void Go2StateMonitor::recordingTimerCallback() {
    // 检查是否应该停止记录
    if (shouldStopRecording()) {
        stopDataRecording();
        return;
    }

    // 获取当前详细状态
    auto current_detailed_state = getDetailedState();

    // 确保基本状态字段是最新的
    {
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        current_detailed_state.state = current_state_;
        current_detailed_state.health_level = current_health_level_;
        current_detailed_state.health_score = current_health_score_;
        current_detailed_state.error_code = current_error_code_;
    }

    // 更新时间戳为当前时间
    current_detailed_state.timestamp_ns = this->get_clock()->now().nanoseconds();

    // 记录状态数据
    {
        std::lock_guard<std::mutex> lock(recording_mutex_);
        recorded_states_.push_back(current_detailed_state);
    }
}

bool Go2StateMonitor::shouldStopRecording() const {
    if (recording_duration_ == 0) {
        return false; // 持续记录，不自动停止
    }

    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        now - recording_start_time_).count();

    return elapsed >= recording_duration_;
}

bool Go2StateMonitor::exportToJSON(const std::string& file_path) const {
    std::ofstream file(file_path);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "无法创建文件: %s", file_path.c_str());
        return false;
    }

    file << "{\n";
    file << "  \"metadata\": {\n";
    file << "    \"export_time\": " << std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count() << ",\n";
    file << "    \"robot_type\": \"Go2\",\n";
    file << "    \"data_count\": " << recorded_states_.size() << ",\n";
    file << "    \"recording_frequency\": " << RECORDING_FREQUENCY << "\n";
    file << "  },\n";
    file << "  \"data\": [\n";

    for (size_t i = 0; i < recorded_states_.size(); ++i) {
        const auto& state = recorded_states_[i];
        file << "    {\n";
        file << "      \"timestamp_ns\": " << state.timestamp_ns << ",\n";
        file << "      \"robot_state\": " << static_cast<int>(state.state) << ",\n";
        file << "      \"health_level\": " << static_cast<int>(state.health_level) << ",\n";
        file << "      \"health_score\": " << state.health_score << ",\n";
        file << "      \"position\": [" << state.motion.position.x << ", "
             << state.motion.position.y << ", " << state.motion.position.z << "],\n";
        file << "      \"velocity\": [" << state.motion.velocity.x << ", "
             << state.motion.velocity.y << ", " << state.motion.velocity.z << "],\n";
        file << "      \"system_voltage\": " << state.system_v << ",\n";
        file << "      \"system_current\": " << state.system_a << ",\n";
        file << "      \"motor_count\": " << state.motors.size() << ",\n";
        file << "      \"foot_count\": " << state.feet.size() << "\n";
        file << "    }";
        if (i < recorded_states_.size() - 1) {
            file << ",";
        }
        file << "\n";
    }

    file << "  ]\n";
    file << "}\n";
    file.close();

    RCLCPP_INFO(this->get_logger(), "已成功导出 %zu 条状态数据到JSON文件: %s",
               recorded_states_.size(), file_path.c_str());
    return true;
}

bool Go2StateMonitor::exportToCSV(const std::string& file_path) const {
    std::ofstream file(file_path);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "无法创建文件: %s", file_path.c_str());
        return false;
    }

    // 写入CSV头部
    file << "timestamp_ns,robot_state,health_level,health_score,pos_x,pos_y,pos_z,"
         << "vel_x,vel_y,vel_z,system_v,system_a,motor_count,foot_count\n";

    // 写入数据行
    for (const auto& state : recorded_states_) {
        file << state.timestamp_ns << ","
             << static_cast<int>(state.state) << ","
             << static_cast<int>(state.health_level) << ","
             << std::fixed << std::setprecision(3) << state.health_score << ","
             << state.motion.position.x << ","
             << state.motion.position.y << ","
             << state.motion.position.z << ","
             << state.motion.velocity.x << ","
             << state.motion.velocity.y << ","
             << state.motion.velocity.z << ","
             << state.system_v << ","
             << state.system_a << ","
             << state.motors.size() << ","
             << state.feet.size() << "\n";
    }

    file.close();
    RCLCPP_INFO(this->get_logger(), "已成功导出 %zu 条状态数据到CSV文件: %s",
               recorded_states_.size(), file_path.c_str());
    return true;
}

bool Go2StateMonitor::exportToBinary(const std::string& file_path) const {
    std::ofstream file(file_path, std::ios::binary);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "无法创建文件: %s", file_path.c_str());
        return false;
    }

    // 写入文件头信息
    uint32_t magic_number = 0x47324442; // "G2DB" Go2 Data Binary
    uint32_t version = 1;
    uint32_t data_count = static_cast<uint32_t>(recorded_states_.size());
    float frequency = RECORDING_FREQUENCY;

    file.write(reinterpret_cast<const char*>(&magic_number), sizeof(magic_number));
    file.write(reinterpret_cast<const char*>(&version), sizeof(version));
    file.write(reinterpret_cast<const char*>(&data_count), sizeof(data_count));
    file.write(reinterpret_cast<const char*>(&frequency), sizeof(frequency));

    // 写入状态数据
    for (const auto& state : recorded_states_) {
        // 简化的二进制格式，只写入核心数据
        file.write(reinterpret_cast<const char*>(&state.timestamp_ns), sizeof(state.timestamp_ns));
        file.write(reinterpret_cast<const char*>(&state.state), sizeof(state.state));
        file.write(reinterpret_cast<const char*>(&state.health_level), sizeof(state.health_level));
        file.write(reinterpret_cast<const char*>(&state.health_score), sizeof(state.health_score));
        file.write(reinterpret_cast<const char*>(&state.motion.position), sizeof(state.motion.position));
        file.write(reinterpret_cast<const char*>(&state.motion.velocity), sizeof(state.motion.velocity));
        file.write(reinterpret_cast<const char*>(&state.system_v), sizeof(state.system_v));
        file.write(reinterpret_cast<const char*>(&state.system_a), sizeof(state.system_a));
    }

    file.close();
    RCLCPP_INFO(this->get_logger(), "已成功导出 %zu 条状态数据到二进制文件: %s",
               recorded_states_.size(), file_path.c_str());
    return true;
}

} // namespace go2_adapter
} // namespace robot_adapters
