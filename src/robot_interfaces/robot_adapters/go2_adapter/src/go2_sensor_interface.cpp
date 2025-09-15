/**
 * @file   go2_sensor_interface.cpp
 * @brief  Go2机器人传感器接口实现文件
 * @author Yang Nan
 * @date   2025-09-14
 *
 * 基于go2_communication和go2_message_converter重新实现的Go2传感器接口
 * 提供统一的传感器数据访问和消息转换功能
 */

#include "robot_adapters/go2_adapter/go2_sensor_interface.hpp"

// Go2原生消息类型（在实现文件中包含具体头文件）
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/bms_state.hpp"
#include "unitree_go/msg/motor_state.hpp"

// 标准ROS2传感器消息
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <robot_base_interfaces/state_interface/state_types.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>

namespace robot_adapters {
namespace go2_adapter {

using ConversionResult = robot_adapters::go2_adapter::ConversionResult;

Go2SensorInterface::Go2SensorInterface(const std::string& node_name,
                                       std::shared_ptr<Go2Communication> communication_ptr)
    : rclcpp::Node(node_name), communication_(communication_ptr), owns_communication_(false) {

    RCLCPP_INFO(this->get_logger(), "Go2 Sensor Interface initializing...");

    // 初始化传感器状态映射
    initializeSensorMappings();

    // 标记通信管理器所有权
    if (!communication_) {
        owns_communication_ = true;
    }
}

Go2SensorInterface::~Go2SensorInterface() {
    shutdown();
    if (owns_communication_ && communication_) {
        communication_->shutdown();
    }
}

// ========== ISensorInterface 核心接口实现 ==========

bool Go2SensorInterface::initialize() {
    RCLCPP_INFO(this->get_logger(), "Initializing Go2 sensor interface...");

    try {
        // 1. 初始化通信管理器
        if (!initializeCommunication()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize communication");
            return false;
        }

        // 2. 初始化消息转换器
        initializeMessageConverter();

        // 3. 设置Go2原生消息回调
        setupGo2MessageCallbacks();

        // 4. 启动数据验证定时器
        data_validation_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&Go2SensorInterface::dataValidationTimerCallback, this));

        // 5. 验证通信连接
        if (!validateCommunication()) {
            RCLCPP_WARN(this->get_logger(), "Communication validation failed, but continuing");
        }

        RCLCPP_INFO(this->get_logger(), "Go2 sensor interface initialized successfully");
        return true;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception during initialization: %s", e.what());
        return false;
    }
}

bool Go2SensorInterface::shutdown() {
    RCLCPP_INFO(this->get_logger(), "Shutting down Go2 sensor interface...");

    // 停止定时器
    if (data_validation_timer_) {
        data_validation_timer_->cancel();
        data_validation_timer_.reset();
    }

    // 清空数据缓存
    {
        std::lock_guard<std::mutex> lock1(sensor_data_mutex_);
        std::lock_guard<std::mutex> lock2(converted_data_mutex_);

        latest_sport_mode_state_.reset();
        latest_low_state_.reset();
        latest_bms_state_.reset();
        latest_point_cloud_.reset();
        latest_standard_imu_.reset();
        latest_odometry_.reset();

        cached_point_cloud_.reset();
        cached_fused_imu_.reset();
        cached_motor_data_.clear();
        cached_foot_data_.clear();
    }

    // 停止通信
    if (communication_ && !owns_communication_) {
        communication_->stopCommunication();
    }

    communication_active_.store(false);
    RCLCPP_INFO(this->get_logger(), "Go2 sensor interface shut down successfully");
    return true;
}

std::vector<robot_base_interfaces::sensor_interface::SensorInfo>
Go2SensorInterface::getAvailableSensors() const {
    using SensorType = robot_base_interfaces::sensor_interface::SensorType;
    using SensorInfo = robot_base_interfaces::sensor_interface::SensorInfo;
    using SensorStatus = robot_base_interfaces::sensor_interface::SensorStatus;

    std::vector<SensorInfo> sensors;

    // Livox Mid360 激光雷达
    SensorInfo lidar_info(SensorType::LIDAR_3D, "Livox Mid360", "/utlidar/cloud");
    lidar_info.frame_id = "utlidar_lidar";
    lidar_info.frequency = LIDAR_FREQUENCY;
    lidar_info.status = point_cloud_available_.load() ?
                       SensorStatus::ACTIVE : SensorStatus::UNKNOWN;
    lidar_info.parameters["max_range"] = LIDAR_MAX_RANGE;
    lidar_info.parameters["min_range"] = LIDAR_MIN_RANGE;
    lidar_info.parameters["resolution"] = LIDAR_RESOLUTION;
    sensors.push_back(lidar_info);

    // IMU传感器（基于SportModeState + 标准IMU融合）
    SensorInfo imu_info(SensorType::IMU, "Go2 Fused IMU", "/imu/data");
    imu_info.frame_id = "imu_link";
    imu_info.frequency = IMU_FREQUENCY;
    imu_info.status = sport_state_available_.load() ?
                     SensorStatus::ACTIVE : SensorStatus::UNKNOWN;
    sensors.push_back(imu_info);

    return sensors;
}

bool Go2SensorInterface::isSensorAvailable(robot_base_interfaces::sensor_interface::SensorType sensor_type) const {
    switch (sensor_type) {
        case robot_base_interfaces::sensor_interface::SensorType::LIDAR_3D:
            return point_cloud_available_.load();
        case robot_base_interfaces::sensor_interface::SensorType::IMU:
            return sport_state_available_.load();
        default:
            return false;
    }
}

bool Go2SensorInterface::startSensor(robot_base_interfaces::sensor_interface::SensorType sensor_type) {
    RCLCPP_INFO(this->get_logger(), "Starting sensor: %d", static_cast<int>(sensor_type));

    if (!communication_active_.load()) {
        RCLCPP_WARN(this->get_logger(), "Communication not active, cannot start sensor");
        return false;
    }

    // Go2的传感器通常是自动启动的，这里主要是状态管理
    return isSensorAvailable(sensor_type);
}

bool Go2SensorInterface::stopSensor(robot_base_interfaces::sensor_interface::SensorType sensor_type) {
    RCLCPP_INFO(this->get_logger(), "Stopping sensor: %d", static_cast<int>(sensor_type));
    // Go2的传感器通常不能手动停止，这里主要是状态管理
    return true;
}

robot_base_interfaces::sensor_interface::SensorStatus
Go2SensorInterface::getSensorStatus(robot_base_interfaces::sensor_interface::SensorType sensor_type) const {
    using SensorStatus = robot_base_interfaces::sensor_interface::SensorStatus;

    if (!isSensorAvailable(sensor_type)) {
        return SensorStatus::DISCONNECTED;
    }

    // 检查数据新鲜度
    if (isSensorDataFresh(sensor_type)) {
        return SensorStatus::ACTIVE;
    } else {
        return SensorStatus::ERROR;
    }
}

std::shared_ptr<robot_base_interfaces::sensor_interface::SensorData>
Go2SensorInterface::getLatestData(robot_base_interfaces::sensor_interface::SensorType sensor_type) const {
    using SensorType = robot_base_interfaces::sensor_interface::SensorType;
    using SensorData = robot_base_interfaces::sensor_interface::SensorData;

    auto data = std::make_shared<SensorData>(sensor_type);

    switch (sensor_type) {
        case SensorType::LIDAR_3D:
            data->point_cloud = getLatestPointCloud();
            if (data->point_cloud) {
                data->timestamp_ns = data->point_cloud->timestamp_ns;
            }
            break;

        case SensorType::IMU:
            data->imu = getLatestIMU();
            if (data->imu) {
                data->timestamp_ns = data->imu->timestamp_ns;
            }
            break;

        default:
            return nullptr;
    }

    return data->hasData() ? data : nullptr;
}

std::shared_ptr<robot_base_interfaces::sensor_interface::PointCloudData>
Go2SensorInterface::getLatestPointCloud() const {
    std::lock_guard<std::mutex> lock(converted_data_mutex_);
    return cached_point_cloud_;
}

std::shared_ptr<robot_base_interfaces::sensor_interface::IMUData>
Go2SensorInterface::getLatestIMU() const {
    std::lock_guard<std::mutex> lock(converted_data_mutex_);
    return cached_fused_imu_;
}

std::shared_ptr<robot_base_interfaces::sensor_interface::IMUData>
Go2SensorInterface::getFusedIMUData() const {
    std::lock_guard<std::mutex> lock(converted_data_mutex_);
    return cached_fused_imu_;
}

// ========== Go2特有接口实现 ==========

std::shared_ptr<unitree_go::msg::SportModeState>
Go2SensorInterface::getLatestSportModeState() const {
    std::lock_guard<std::mutex> lock(sensor_data_mutex_);
    return latest_sport_mode_state_;
}

std::shared_ptr<unitree_go::msg::LowState>
Go2SensorInterface::getLatestLowState() const {
    std::lock_guard<std::mutex> lock(sensor_data_mutex_);
    return latest_low_state_;
}

std::shared_ptr<unitree_go::msg::BmsState>
Go2SensorInterface::getLatestBmsState() const {
    std::lock_guard<std::mutex> lock(sensor_data_mutex_);
    return latest_bms_state_;
}

robot_base_interfaces::state_interface::MotorInfo Go2SensorInterface::getMotorData(int motor_id) const {
    std::lock_guard<std::mutex> lock(converted_data_mutex_);

    auto it = cached_motor_data_.find(motor_id);
    if (it != cached_motor_data_.end()) {
        return it->second;
    }

    // 返回默认电机信息
    robot_base_interfaces::state_interface::MotorInfo motor_info;
    motor_info.motor_id = static_cast<uint8_t>(motor_id);
    return motor_info;
}

robot_base_interfaces::state_interface::FootInfo Go2SensorInterface::getFootData(int foot_id) const {
    std::lock_guard<std::mutex> lock(converted_data_mutex_);

    auto it = cached_foot_data_.find(foot_id);
    if (it != cached_foot_data_.end()) {
        return it->second;
    }

    // 返回默认足端信息
    robot_base_interfaces::state_interface::FootInfo foot_info;
    foot_info.foot_id = static_cast<uint8_t>(foot_id);
    return foot_info;
}

void Go2SensorInterface::setGo2Callbacks(
    std::function<void(const std::shared_ptr<unitree_go::msg::SportModeState>&)> sport_callback,
    std::function<void(const std::shared_ptr<unitree_go::msg::LowState>&)> low_state_callback,
    std::function<void(const std::shared_ptr<unitree_go::msg::BmsState>&)> bms_callback) {

    user_sport_callback_     = sport_callback;
    user_low_state_callback_ = low_state_callback;
    user_bms_callback_       = bms_callback;
}

std::map<std::string, float> Go2SensorInterface::getCommunicationStatistics() const {
    std::map<std::string, float> stats;

    if (communication_) {
        auto comm_stats = communication_->getStatistics();
        stats["messages_received"] = static_cast<float>(comm_stats.messages_received);
        stats["messages_sent"] = static_cast<float>(comm_stats.messages_sent);
        stats["total_bytes_received"] = static_cast<float>(comm_stats.total_bytes_received);
        stats["message_rate_hz"] = comm_stats.message_rate_hz;
    }

    return stats;
}

void Go2SensorInterface::refreshAllSensorData() {
    if (!communication_active_.load()) {
        return;
    }

    // 主动从通信管理器获取最新数据
    if (communication_) {
        // 更新SportModeState
        if (auto sport_state = communication_->getLatestSportModeState()) {
            onSportModeStateReceived(sport_state);
        }

        // 更新LowState
        if (auto low_state = communication_->getLatestLowState()) {
            onLowStateReceived(low_state);
        }

        // 更新BmsState
        if (auto bms_state = communication_->getLatestBmsState()) {
            onBmsStateReceived(bms_state);
        }

        // 更新PointCloud
        if (auto point_cloud = communication_->getLatestPointCloud()) {
            onPointCloudReceived(point_cloud);
        }

        // 更新IMU
        if (auto imu_data = communication_->getLatestImu()) {
            onStandardImuReceived(imu_data);
        }
    }
}

// ========== 私有方法实现 ==========

bool Go2SensorInterface::initializeCommunication() {
    if (!communication_) {
        // 创建新的通信管理器
        communication_ = std::make_shared<Go2Communication>(
            std::static_pointer_cast<rclcpp::Node>(shared_from_this()));
        owns_communication_ = true;

        if (!communication_->initialize()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize Go2 communication");
            return false;
        }

        if (!communication_->startCommunication()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to start Go2 communication");
            return false;
        }
    }

    communication_active_.store(true);
    return true;
}

void Go2SensorInterface::initializeMessageConverter() {
    message_converter_ = std::make_unique<Go2MessageConverter>();
    RCLCPP_INFO(this->get_logger(), "Go2 message converter initialized");
}

void Go2SensorInterface::setupGo2MessageCallbacks() {
    if (!communication_) {
        return;
    }

    // 设置Go2原生消息回调
    communication_->setSportModeStateCallback(
        std::bind(&Go2SensorInterface::onSportModeStateReceived, this, std::placeholders::_1));

    communication_->setLowStateCallback(
        std::bind(&Go2SensorInterface::onLowStateReceived, this, std::placeholders::_1));

    communication_->setBmsStateCallback(
        std::bind(&Go2SensorInterface::onBmsStateReceived, this, std::placeholders::_1));

    // 设置标准ROS2消息回调
    communication_->setPointCloudCallback(
        std::bind(&Go2SensorInterface::onPointCloudReceived, this, std::placeholders::_1));

    communication_->setImuCallback(
        std::bind(&Go2SensorInterface::onStandardImuReceived, this, std::placeholders::_1));

    communication_->setOdometryCallback(
        std::bind(&Go2SensorInterface::onOdometryReceived, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Go2 message callbacks set up");
}

void Go2SensorInterface::initializeSensorMappings() {
    // 初始化传感器类型映射
    go2_to_unified_sensor_map_["lidar"] = robot_base_interfaces::sensor_interface::SensorType::LIDAR_3D;
    go2_to_unified_sensor_map_["imu"] = robot_base_interfaces::sensor_interface::SensorType::IMU;

    // 初始化传感器状态
    sensor_initialized_[robot_base_interfaces::sensor_interface::SensorType::LIDAR_3D] = false;
    sensor_initialized_[robot_base_interfaces::sensor_interface::SensorType::IMU] = false;

    RCLCPP_DEBUG(this->get_logger(), "Sensor mappings initialized");
}

bool Go2SensorInterface::validateCommunication() const {
    if (!communication_) {
        return false;
    }

    return communication_->isConnected() && communication_->isCommunicating();
}

// ========== Go2原生数据回调函数实现 ==========

void Go2SensorInterface::onSportModeStateReceived(const unitree_go::msg::SportModeState::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lock(sensor_data_mutex_);
        latest_sport_mode_state_ = msg;
    }

    sport_state_available_.store(true);
    updateSensorTimestamp(robot_base_interfaces::sensor_interface::SensorType::IMU,
                         this->get_clock()->now());

    // 从SportModeState提取并缓存IMU数据
    if (message_converter_) {
        std::lock_guard<std::mutex> lock(converted_data_mutex_);
        cached_fused_imu_ = std::make_shared<robot_base_interfaces::sensor_interface::IMUData>(
            extractIMUFromSportState(*msg));
        
        // 同时更新足端数据缓存，因为extractFootData现在依赖SportModeState
        // 创建一个临时的LowState消息来调用extractFootData
        unitree_go::msg::LowState temp_low_state;
        temp_low_state.foot_force = {static_cast<int16_t>(msg->foot_force[0]), 
                                    static_cast<int16_t>(msg->foot_force[1]), 
                                    static_cast<int16_t>(msg->foot_force[2]), 
                                    static_cast<int16_t>(msg->foot_force[3])};
        temp_low_state.foot_force_est = temp_low_state.foot_force; // 使用相同的力数据
        
        for (int i = 0; i < NUM_FEET; ++i) {
            cached_foot_data_[i] = extractFootData(temp_low_state, i);
        }
    }

    // 调用用户回调
    if (user_sport_callback_) {
        user_sport_callback_(msg);
    }

    RCLCPP_DEBUG(this->get_logger(), "Received Go2 SportModeState");
}

void Go2SensorInterface::onLowStateReceived(const unitree_go::msg::LowState::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lock(sensor_data_mutex_);
        latest_low_state_ = msg;
    }

    low_state_available_.store(true);

    // 提取并缓存电机数据
    if (message_converter_) {
        std::lock_guard<std::mutex> lock(converted_data_mutex_);

        // 更新所有电机数据
        for (int i = 0; i < NUM_JOINTS && i < static_cast<int>(msg->motor_state.size()); ++i) {
            cached_motor_data_[i] = extractMotorData(*msg, i);
        }

        // 更新足端数据
        for (int i = 0; i < NUM_FEET; ++i) {
            cached_foot_data_[i] = extractFootData(*msg, i);
        }
    }

    // 调用用户回调
    if (user_low_state_callback_) {
        user_low_state_callback_(msg);
    }

    RCLCPP_DEBUG(this->get_logger(), "Received Go2 LowState with %zu motors",
                 msg->motor_state.size());
}

void Go2SensorInterface::onBmsStateReceived(const unitree_go::msg::BmsState::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lock(sensor_data_mutex_);
        latest_bms_state_ = msg;
    }

    // 调用用户回调
    if (user_bms_callback_) {
        user_bms_callback_(msg);
    }

    RCLCPP_DEBUG(this->get_logger(), "Received Go2 BmsState");
}

void Go2SensorInterface::onPointCloudReceived(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lock(sensor_data_mutex_);
        latest_point_cloud_ = msg;
    }

    point_cloud_available_.store(true);
    updateSensorTimestamp(robot_base_interfaces::sensor_interface::SensorType::LIDAR_3D,
                         this->get_clock()->now());

    // 转换并缓存点云数据
    if (message_converter_) {
        std::lock_guard<std::mutex> lock(converted_data_mutex_);
        // 使用消息转换器进行点云增强处理
        auto enhanced_pc = std::make_shared<robot_base_interfaces::sensor_interface::PointCloudData>();
        if (message_converter_->enhancePointCloudData(*msg, *enhanced_pc) ==
            ConversionResult::SUCCESS) {
            cached_point_cloud_ = enhanced_pc;
        }
    }

    RCLCPP_DEBUG(this->get_logger(), "Received PointCloud with %d points",
                 msg->width * msg->height);
}

void Go2SensorInterface::onStandardImuReceived(const sensor_msgs::msg::Imu::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lock(sensor_data_mutex_);
        latest_standard_imu_ = msg;
    }

    // 如果有SportModeState数据，进行IMU数据融合
    if (latest_sport_mode_state_ && message_converter_) {
        std::lock_guard<std::mutex> lock(converted_data_mutex_);

        auto go2_imu = extractIMUFromSportState(*latest_sport_mode_state_);
        robot_base_interfaces::sensor_interface::IMUData standard_imu_data;

        // 使用消息转换器将ROS IMU转换为统一格式
        if (message_converter_->convertRosImuToUnified(*msg, standard_imu_data) ==
            ConversionResult::SUCCESS) {

            // 进行数据融合
            cached_fused_imu_ = std::make_shared<robot_base_interfaces::sensor_interface::IMUData>(
                fuseIMUData(go2_imu, standard_imu_data));
        }
    }

    RCLCPP_DEBUG(this->get_logger(), "Received standard IMU data");
}

void Go2SensorInterface::onOdometryReceived(const nav_msgs::msg::Odometry::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lock(sensor_data_mutex_);
        latest_odometry_ = msg;
    }

    RCLCPP_DEBUG(this->get_logger(), "Received Odometry data");
}

// ========== 数据处理与转换方法实现 ==========

robot_base_interfaces::sensor_interface::IMUData
Go2SensorInterface::extractIMUFromSportState(const unitree_go::msg::SportModeState& sport_state) const {
    robot_base_interfaces::sensor_interface::IMUData imu_data;

    imu_data.frame_id = "imu_link";
    imu_data.timestamp_ns = sport_state.stamp.sec * 1000000000ULL + sport_state.stamp.nanosec;

    // 从SportModeState提取IMU信息
    if (sport_state.imu_state.quaternion.size() >= 4) {
        imu_data.orientation.w = sport_state.imu_state.quaternion[0];
        imu_data.orientation.x = sport_state.imu_state.quaternion[1];
        imu_data.orientation.y = sport_state.imu_state.quaternion[2];
        imu_data.orientation.z = sport_state.imu_state.quaternion[3];
    }

    if (sport_state.imu_state.gyroscope.size() >= 3) {
        imu_data.angular_velocity.x = sport_state.imu_state.gyroscope[0];
        imu_data.angular_velocity.y = sport_state.imu_state.gyroscope[1];
        imu_data.angular_velocity.z = sport_state.imu_state.gyroscope[2];
    }

    if (sport_state.imu_state.accelerometer.size() >= 3) {
        imu_data.linear_acceleration.x = sport_state.imu_state.accelerometer[0];
        imu_data.linear_acceleration.y = sport_state.imu_state.accelerometer[1];
        imu_data.linear_acceleration.z = sport_state.imu_state.accelerometer[2];
    }

    if (sport_state.imu_state.rpy.size() >= 3) {
        imu_data.rpy.roll = sport_state.imu_state.rpy[0];
        imu_data.rpy.pitch = sport_state.imu_state.rpy[1];
        imu_data.rpy.yaw = sport_state.imu_state.rpy[2];
    }

    imu_data.temperature = sport_state.imu_state.temperature;

    return imu_data;
}

robot_base_interfaces::state_interface::MotorInfo Go2SensorInterface::extractMotorData(const unitree_go::msg::LowState& low_state, int motor_id) const {
    robot_base_interfaces::state_interface::MotorInfo motor_info;
    motor_info.motor_id = static_cast<uint8_t>(motor_id);

    if (motor_id < 0 || motor_id >= static_cast<int>(low_state.motor_state.size())) {
        return motor_info;
    }

    const auto& motor_state = low_state.motor_state[motor_id];

    motor_info.mode = motor_state.mode;
    motor_info.position = motor_state.q;
    motor_info.velocity = motor_state.dq;
    motor_info.acceleration = motor_state.ddq;
    motor_info.torque_estimated = motor_state.tau_est;
    motor_info.position_raw = motor_state.q_raw;
    motor_info.velocity_raw = motor_state.dq_raw;
    motor_info.acceleration_raw = motor_state.ddq_raw;
    motor_info.temperature = motor_state.temperature;
    motor_info.lost_count = 0;  // LowState中没有此字段
    motor_info.is_online = true;  // 默认在线
    motor_info.error_code = 0;   // LowState中没有此字段

    return motor_info;
}

robot_base_interfaces::state_interface::FootInfo Go2SensorInterface::extractFootData(const unitree_go::msg::LowState& low_state, int foot_id) const {
    robot_base_interfaces::state_interface::FootInfo foot_info;
    foot_info.foot_id = static_cast<uint8_t>(foot_id);

    if (foot_id < 0 || foot_id >= NUM_FEET) {
        return foot_info;
    }

    // 从LowState提取足端信息
    if (foot_id < static_cast<int>(low_state.foot_force.size())) {
        foot_info.force = static_cast<float>(low_state.foot_force[foot_id]);
    }

    if (foot_id < static_cast<int>(low_state.foot_force_est.size())) {
        foot_info.force_estimated = static_cast<float>(low_state.foot_force_est[foot_id]);
    }

    // 简单的接触检测
    foot_info.in_contact = (foot_info.force > 10.0f);  // 阈值可调整
    foot_info.contact_probability = foot_info.in_contact ? 1.0f : 0.0f;

    // 尝试从SportModeState获取足端位置和速度数据
    if (latest_sport_mode_state_) {
        // 从SportModeState提取足端位置（身体坐标系）
        // foot_position_body数组：12个元素，每只脚3个坐标(x,y,z)，按脚的顺序排列
        int position_start_idx = foot_id * 3;
        if (position_start_idx + 2 < static_cast<int>(latest_sport_mode_state_->foot_position_body.size())) {
            foot_info.position.x = static_cast<float>(latest_sport_mode_state_->foot_position_body[position_start_idx]);
            foot_info.position.y = static_cast<float>(latest_sport_mode_state_->foot_position_body[position_start_idx + 1]);
            foot_info.position.z = static_cast<float>(latest_sport_mode_state_->foot_position_body[position_start_idx + 2]);
        }

        // 从SportModeState提取足端速度（身体坐标系）
        // foot_speed_body数组：12个元素，每只脚3个坐标(x,y,z)，按脚的顺序排列
        int velocity_start_idx = foot_id * 3;
        if (velocity_start_idx + 2 < static_cast<int>(latest_sport_mode_state_->foot_speed_body.size())) {
            foot_info.velocity.x = static_cast<float>(latest_sport_mode_state_->foot_speed_body[velocity_start_idx]);
            foot_info.velocity.y = static_cast<float>(latest_sport_mode_state_->foot_speed_body[velocity_start_idx + 1]);
            foot_info.velocity.z = static_cast<float>(latest_sport_mode_state_->foot_speed_body[velocity_start_idx + 2]);
        }
    } else {
        // 如果没有SportModeState数据，发出警告并设为默认值
        RCLCPP_WARN(
            this->get_logger(),
            "SportModeState数据不可用，足端%d的位置和速度数据设为默认值(0.0)",
            foot_id
        );
        
        foot_info.position.x = 0.0f;
        foot_info.position.y = 0.0f;
        foot_info.position.z = 0.0f;
        foot_info.velocity.x = 0.0f;
        foot_info.velocity.y = 0.0f;
        foot_info.velocity.z = 0.0f;
    }

    return foot_info;
}

robot_base_interfaces::sensor_interface::IMUData
Go2SensorInterface::fuseIMUData(
    const robot_base_interfaces::sensor_interface::IMUData& go2_imu,
    const robot_base_interfaces::sensor_interface::IMUData& standard_imu) const {

    // 简单的数据融合策略：优先使用Go2内置IMU的姿态信息，
    // 标准IMU的加速度和角速度作为补充
    robot_base_interfaces::sensor_interface::IMUData fused_imu = go2_imu;

    // 如果标准IMU数据更新，使用其加速度和角速度数据
    if (standard_imu.timestamp_ns > 0) {
        fused_imu.linear_acceleration = standard_imu.linear_acceleration;
        fused_imu.angular_velocity = standard_imu.angular_velocity;

        // 使用较新的时间戳
        fused_imu.timestamp_ns = std::max(go2_imu.timestamp_ns, standard_imu.timestamp_ns);
    }

    return fused_imu;
}

void Go2SensorInterface::updateSensorTimestamp(robot_base_interfaces::sensor_interface::SensorType sensor_type,
                                               const rclcpp::Time& timestamp) {
    sensor_timestamps_[sensor_type] = timestamp;
    sensor_data_count_[sensor_type]++;
}

bool Go2SensorInterface::isSensorDataFresh(robot_base_interfaces::sensor_interface::SensorType sensor_type,
                                          int max_age_ms) const {
    auto it = sensor_timestamps_.find(sensor_type);
    if (it == sensor_timestamps_.end()) {
        return false;
    }

    auto now = rclcpp::Clock().now();
    auto age_ms = (now - it->second).seconds() * 1000.0;
    return age_ms <= max_age_ms;
}

// ========== ISensorInterface 其他接口实现 ==========

void Go2SensorInterface::setPointCloudCallback(
    std::function<void(const std::shared_ptr<robot_base_interfaces::sensor_interface::PointCloudData>&)> callback) {
    (void)callback;
    // TODO: 实现点云回调存储和触发
}

void Go2SensorInterface::setIMUCallback(
    std::function<void(const std::shared_ptr<robot_base_interfaces::sensor_interface::IMUData>&)> callback) {
    (void)callback;
    // TODO: 实现IMU回调存储和触发
}

void Go2SensorInterface::setSensorCallback(
    robot_base_interfaces::sensor_interface::SensorType sensor_type,
    std::function<void(const std::shared_ptr<robot_base_interfaces::sensor_interface::SensorData>&)> callback) {
    (void)sensor_type;
    (void)callback;
    // TODO: 实现通用传感器回调
}

bool Go2SensorInterface::setSensorParameter(robot_base_interfaces::sensor_interface::SensorType sensor_type,
                                           const std::string& parameter_name,
                                           float value) {
    (void)sensor_type;
    (void)parameter_name;
    (void)value;
    // Go2的传感器参数通常是固定的
    return false;
}

bool Go2SensorInterface::getSensorParameter(robot_base_interfaces::sensor_interface::SensorType sensor_type,
                                           const std::string& parameter_name,
                                           float& value) const {
    using SensorType = robot_base_interfaces::sensor_interface::SensorType;

    if (sensor_type == SensorType::LIDAR_3D) {
        if (parameter_name == "max_range") {
            value = LIDAR_MAX_RANGE;
            return true;
        } else if (parameter_name == "min_range") {
            value = LIDAR_MIN_RANGE;
            return true;
        } else if (parameter_name == "frequency") {
            value = LIDAR_FREQUENCY;
            return true;
        }
    }

    return false;
}

bool Go2SensorInterface::setSensorFrequency(robot_base_interfaces::sensor_interface::SensorType sensor_type,
                                           float frequency) {
    (void)sensor_type;
    (void)frequency;
    // Go2的传感器频率通常是固定的
    return false;
}

bool Go2SensorInterface::calibrateSensor(robot_base_interfaces::sensor_interface::SensorType sensor_type) {
    (void)sensor_type;
    // Go2的传感器校准通常在出厂时完成
    return false;
}

robot_base_interfaces::sensor_interface::CalibrationData
Go2SensorInterface::getCalibrationData(robot_base_interfaces::sensor_interface::SensorType sensor_type) const {
    robot_base_interfaces::sensor_interface::CalibrationData calib_data;
    calib_data.sensor_type = sensor_type;
    calib_data.is_valid = false;  // 当前版本不提供校准数据
    return calib_data;
}

bool Go2SensorInterface::setCalibrationData(const robot_base_interfaces::sensor_interface::CalibrationData& calibration_data) {
    (void)calibration_data;
    // 当前版本不支持设置校准数据
    return false;
}

std::vector<float> Go2SensorInterface::getSensorTransform(robot_base_interfaces::sensor_interface::SensorType sensor_type) const {
    (void)sensor_type;
    std::vector<float> transform(16, 0.0f);
    // 初始化为单位矩阵
    transform[0] = transform[5] = transform[10] = transform[15] = 1.0f;
    return transform;
}

bool Go2SensorInterface::setSensorTransform(robot_base_interfaces::sensor_interface::SensorType sensor_type,
                                           const std::vector<float>& transform) {
    (void)sensor_type;
    (void)transform;
    // 当前版本不支持动态设置变换矩阵
    return false;
}

std::map<robot_base_interfaces::sensor_interface::SensorType, robot_base_interfaces::sensor_interface::SensorStatus>
Go2SensorInterface::getSensorHealth() const {
    using SensorType = robot_base_interfaces::sensor_interface::SensorType;
    using SensorStatus = robot_base_interfaces::sensor_interface::SensorStatus;

    std::map<SensorType, SensorStatus> health;
    health[SensorType::LIDAR_3D] = getSensorStatus(SensorType::LIDAR_3D);
    health[SensorType::IMU] = getSensorStatus(SensorType::IMU);

    return health;
}

std::string Go2SensorInterface::getSensorError(robot_base_interfaces::sensor_interface::SensorType sensor_type) const {
    auto it = sensor_error_count_.find(sensor_type);
    if (it != sensor_error_count_.end() && it->second > 0) {
        return "Sensor data timeout or communication error";
    }
    return "";
}

std::map<std::string, float> Go2SensorInterface::getSensorStatistics(robot_base_interfaces::sensor_interface::SensorType sensor_type) const {
    std::map<std::string, float> stats;

    auto error_it = sensor_error_count_.find(sensor_type);
    if (error_it != sensor_error_count_.end()) {
        stats["error_count"] = static_cast<float>(error_it->second);
    }

    auto data_it = sensor_data_count_.find(sensor_type);
    if (data_it != sensor_data_count_.end()) {
        stats["data_count"] = static_cast<float>(data_it->second);
    }

    return stats;
}

// ========== 保护接口实现 ==========

bool Go2SensorInterface::validateSensorType(robot_base_interfaces::sensor_interface::SensorType sensor_type) const {
    return isSensorAvailable(sensor_type);
}

void Go2SensorInterface::updateSensorStatus(robot_base_interfaces::sensor_interface::SensorType sensor_type,
                                           robot_base_interfaces::sensor_interface::SensorStatus status) {
    RCLCPP_DEBUG(this->get_logger(), "Sensor %d status updated to %d",
                 static_cast<int>(sensor_type), static_cast<int>(status));
}

void Go2SensorInterface::dataValidationTimerCallback() {
    // 定期检查数据新鲜度和通信状态
    if (!communication_active_.load()) {
        return;
    }

    using SensorType = robot_base_interfaces::sensor_interface::SensorType;

    // 检查点云数据新鲜度
    if (!isSensorDataFresh(SensorType::LIDAR_3D, 2000)) { // 2秒超时
        recordSensorError(SensorType::LIDAR_3D, "PointCloud data timeout");
        point_cloud_available_.store(false);
    }

    // 检查IMU数据新鲜度
    if (!isSensorDataFresh(SensorType::IMU, 1000)) { // 1秒超时
        recordSensorError(SensorType::IMU, "IMU data timeout");
        sport_state_available_.store(false);
    }

    // 主动刷新数据
    refreshAllSensorData();
}

void Go2SensorInterface::recordSensorError(robot_base_interfaces::sensor_interface::SensorType sensor_type,
                                          const std::string& error_message) {
    sensor_error_count_[sensor_type]++;
    RCLCPP_WARN(this->get_logger(), "Sensor %d error: %s",
                static_cast<int>(sensor_type), error_message.c_str());
}

std::string Go2SensorInterface::getMotorName(int motor_id) const {
    // Go2电机命名约定
    static const std::vector<std::string> motor_names = {
        "FR_hip_joint",  "FR_thigh_joint",  "FR_calf_joint",  "FR_foot_joint",
        "FL_hip_joint",  "FL_thigh_joint",  "FL_calf_joint",  "FL_foot_joint",
        "RR_hip_joint",  "RR_thigh_joint",  "RR_calf_joint",  "RR_foot_joint",
        "RL_hip_joint",  "RL_thigh_joint",  "RL_calf_joint",  "RL_foot_joint",
        "waist_yaw",     "waist_pitch",     "waist_roll",     "head_yaw"
    };

    if (motor_id >= 0 && motor_id < static_cast<int>(motor_names.size())) {
        return motor_names[motor_id];
    }
    return "unknown_motor_" + std::to_string(motor_id);
}

std::string Go2SensorInterface::getFootName(int foot_id) const {
    static const std::vector<std::string> foot_names = {
        "front_right", "front_left", "rear_right", "rear_left"
    };

    if (foot_id >= 0 && foot_id < static_cast<int>(foot_names.size())) {
        return foot_names[foot_id];
    }
    return "unknown_foot_" + std::to_string(foot_id);
}

} // namespace go2_adapter
} // namespace robot_adapters