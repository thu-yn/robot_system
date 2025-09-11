/**
 * @file go2_motion_controller.cpp
 * @brief Go2机器人运动控制器完整实现
 * 
 * 本文件实现了Go2机器人的运动控制功能，包括：
 * - 统一运动接口的完整实现
 * - Go2 API命令的封装和发送
 * - 机器人状态的实时监控和转换
 * - 安全限制和错误处理
 * - 回调函数和事件通知
 */

#include "robot_adapters/go2_adapter/go2_motion_controller.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <algorithm>

namespace robot_adapters {
namespace go2_adapter {

// ============= 构造函数和析构函数 =============

Go2MotionController::Go2MotionController(const std::string& node_name)
    : rclcpp::Node(node_name)
    , is_initialized_(false)
    , current_error_code_(0)
    , last_command_time_(std::chrono::steady_clock::now())
{
    // 记录节点启动信息
    RCLCPP_INFO(this->get_logger(), "Go2 Motion Controller节点正在初始化...");
    
    // 初始化运动状态结构体
    current_motion_state_.current_mode = robot_base_interfaces::motion_interface::MotionMode::IDLE;
    current_motion_state_.current_gait = robot_base_interfaces::motion_interface::GaitType::IDLE;
    current_motion_state_.is_moving = false;
    current_motion_state_.is_balanced = false;
    current_motion_state_.motion_progress = 0.0f;
    current_motion_state_.posture.body_height = DEFAULT_BODY_HEIGHT;
    
    // 初始化回调函数为空
    state_callback_ = nullptr;
    error_callback_ = nullptr;
}

// ============= IMotionController接口实现 =============

robot_base_interfaces::motion_interface::MotionResult Go2MotionController::initialize() {
    // 防止重复初始化
    if (is_initialized_) {
        RCLCPP_WARN(this->get_logger(), "运动控制器已经初始化，跳过重复初始化");
        return robot_base_interfaces::motion_interface::MotionResult::SUCCESS;
    }
    
    RCLCPP_INFO(this->get_logger(), "正在初始化Go2运动控制器...");
    
    try {
        // 初始化ROS2通信组件
        initializeROS2Communications();
        
        // 设置初始化标志
        is_initialized_ = true;
        current_error_code_ = 0;
        
        // 记录初始化完成时间
        last_command_time_ = std::chrono::steady_clock::now();
        
        RCLCPP_INFO(this->get_logger(), "Go2运动控制器初始化完成");
        return robot_base_interfaces::motion_interface::MotionResult::SUCCESS;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "初始化Go2运动控制器失败: %s", e.what());
        current_error_code_ = 1001; // 初始化错误代码
        triggerErrorCallback(current_error_code_, std::string("初始化失败: ") + e.what());
        return robot_base_interfaces::motion_interface::MotionResult::UNKNOWN_ERROR;
    }
}

robot_base_interfaces::motion_interface::MotionResult Go2MotionController::shutdown() {
    RCLCPP_INFO(this->get_logger(), "正在关闭Go2运动控制器...");
    
    try {
        // 发送紧急停止命令确保安全
        emergencyStop(robot_base_interfaces::motion_interface::EmergencyStopLevel::SOFT_STOP);
        
        // 重置所有ROS2发布器和订阅器
        api_request_pub_.reset();
        sport_state_sub_.reset();
        cmd_vel_pub_.reset();
        
        // 清理回调函数
        state_callback_ = nullptr;
        error_callback_ = nullptr;
        
        // 重置状态标志
        is_initialized_ = false;
        current_error_code_ = 0;
        
        RCLCPP_INFO(this->get_logger(), "Go2运动控制器已安全关闭");
        return robot_base_interfaces::motion_interface::MotionResult::SUCCESS;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "关闭Go2运动控制器时出错: %s", e.what());
        return robot_base_interfaces::motion_interface::MotionResult::UNKNOWN_ERROR;
    }
}

robot_base_interfaces::motion_interface::MotionCapabilities Go2MotionController::getCapabilities() const {
    robot_base_interfaces::motion_interface::MotionCapabilities capabilities;
    
    // 设置Go2的运动能力参数
    capabilities.max_linear_velocity = MAX_LINEAR_VELOCITY;      // 1.5 m/s
    capabilities.max_angular_velocity = MAX_ANGULAR_VELOCITY;    // 2.0 rad/s
    capabilities.max_lateral_velocity = MAX_LATERAL_VELOCITY;    // 0.8 m/s
    capabilities.max_roll_angle = MAX_ROLL_ANGLE;                // 0.4 rad
    capabilities.max_pitch_angle = MAX_PITCH_ANGLE;              // 0.4 rad
    capabilities.min_body_height = MIN_BODY_HEIGHT;              // 0.08 m
    capabilities.max_body_height = MAX_BODY_HEIGHT;              // 0.42 m
    
    // 设置Go2的特殊能力
    capabilities.can_climb_stairs = true;   // 支持爬楼梯
    capabilities.can_balance = true;        // 支持平衡控制
    capabilities.can_lateral_move = true;   // 支持侧移
    capabilities.can_dance = true;          // 支持舞蹈动作
    capabilities.can_jump = true;           // 支持跳跃
    capabilities.can_flip = true;           // 支持翻滚
    
    // 设置支持的运动模式
    capabilities.supported_modes = {
        robot_base_interfaces::motion_interface::MotionMode::IDLE,
        robot_base_interfaces::motion_interface::MotionMode::BALANCE_STAND,
        robot_base_interfaces::motion_interface::MotionMode::POSE,
        robot_base_interfaces::motion_interface::MotionMode::LOCOMOTION,
        robot_base_interfaces::motion_interface::MotionMode::LIE_DOWN,
        robot_base_interfaces::motion_interface::MotionMode::SIT,
        robot_base_interfaces::motion_interface::MotionMode::RECOVERY_STAND
    };
    
    // 设置支持的步态类型
    capabilities.supported_gaits = {
        robot_base_interfaces::motion_interface::GaitType::IDLE,
        robot_base_interfaces::motion_interface::GaitType::TROT,
        robot_base_interfaces::motion_interface::GaitType::RUN,
        robot_base_interfaces::motion_interface::GaitType::CLIMB_STAIR
    };
    
    return capabilities;
}

robot_base_interfaces::motion_interface::MotionResult Go2MotionController::setVelocity(
    const robot_base_interfaces::motion_interface::Velocity& velocity) {
    
    // 检查控制器是否已初始化
    if (!is_initialized_) {
        RCLCPP_ERROR(this->get_logger(), "运动控制器未初始化，无法设置速度");
        return robot_base_interfaces::motion_interface::MotionResult::COMMUNICATION_ERROR;
    }
    
    // 验证速度参数是否在安全范围内
    if (!validateVelocityLimits(velocity)) {
        RCLCPP_WARN(this->get_logger(), "速度参数超出Go2安全限制: vx=%.2f, vy=%.2f, wz=%.2f", 
                    velocity.linear_x, velocity.linear_y, velocity.angular_z);
        return robot_base_interfaces::motion_interface::MotionResult::CAPABILITY_LIMITED;
    }
    
    // 发送速度控制命令到Go2
    bool success = sendVelocityCommand(velocity.linear_x, velocity.linear_y, velocity.angular_z);
    
    if (success) {
        // 更新当前运动状态
        {
            std::lock_guard<std::mutex> lock(motion_state_mutex_);
            current_motion_state_.velocity = velocity;
            current_motion_state_.is_moving = (std::abs(velocity.linear_x) > 0.01f || 
                                             std::abs(velocity.linear_y) > 0.01f || 
                                             std::abs(velocity.angular_z) > 0.01f);
            current_motion_state_.timestamp_ns = this->get_clock()->now().nanoseconds();
        }
        
        // 触发状态变化回调
        triggerStateCallback(current_motion_state_);
        
        RCLCPP_DEBUG(this->get_logger(), "成功设置速度: vx=%.2f, vy=%.2f, wz=%.2f", 
                     velocity.linear_x, velocity.linear_y, velocity.angular_z);
        
        return robot_base_interfaces::motion_interface::MotionResult::SUCCESS;
    } else {
        RCLCPP_ERROR(this->get_logger(), "发送速度命令失败");
        return robot_base_interfaces::motion_interface::MotionResult::COMMUNICATION_ERROR;
    }
}

robot_base_interfaces::motion_interface::MotionResult Go2MotionController::setPosture(
    const robot_base_interfaces::motion_interface::Posture& posture) {
    
    // 检查控制器是否已初始化
    if (!is_initialized_) {
        RCLCPP_ERROR(this->get_logger(), "运动控制器未初始化，无法设置姿态");
        return robot_base_interfaces::motion_interface::MotionResult::COMMUNICATION_ERROR;
    }
    
    // 验证姿态参数是否在安全范围内
    if (!validatePostureLimits(posture)) {
        RCLCPP_WARN(this->get_logger(), "姿态参数超出Go2安全限制: roll=%.2f, pitch=%.2f, height=%.2f", 
                    posture.roll, posture.pitch, posture.body_height);
        return robot_base_interfaces::motion_interface::MotionResult::CAPABILITY_LIMITED;
    }
    
    // 发送姿态控制命令到Go2
    bool success = sendPostureCommand(posture.roll, posture.pitch, posture.yaw, posture.body_height);
    
    if (success) {
        // 更新当前姿态状态
        {
            std::lock_guard<std::mutex> lock(motion_state_mutex_);
            current_motion_state_.posture = posture;
            current_motion_state_.timestamp_ns = this->get_clock()->now().nanoseconds();
        }
        
        // 触发状态变化回调
        triggerStateCallback(current_motion_state_);
        
        RCLCPP_DEBUG(this->get_logger(), "成功设置姿态: roll=%.2f, pitch=%.2f, yaw=%.2f, height=%.2f", 
                     posture.roll, posture.pitch, posture.yaw, posture.body_height);
        
        return robot_base_interfaces::motion_interface::MotionResult::SUCCESS;
    } else {
        RCLCPP_ERROR(this->get_logger(), "发送姿态命令失败");
        return robot_base_interfaces::motion_interface::MotionResult::COMMUNICATION_ERROR;
    }
}

robot_base_interfaces::motion_interface::MotionResult Go2MotionController::setBodyHeight(float height) {
    // 检查高度是否在Go2允许的范围内
    if (height < MIN_BODY_HEIGHT || height > MAX_BODY_HEIGHT) {
        RCLCPP_WARN(this->get_logger(), "机身高度%.2fm超出Go2限制范围[%.2f, %.2f]", 
                    height, MIN_BODY_HEIGHT, MAX_BODY_HEIGHT);
        return robot_base_interfaces::motion_interface::MotionResult::CAPABILITY_LIMITED;
    }
    
    // 创建姿态结构体，保持当前RPY角度不变
    robot_base_interfaces::motion_interface::Posture posture;
    {
        std::lock_guard<std::mutex> lock(motion_state_mutex_);
        posture = current_motion_state_.posture;
        posture.body_height = height;
    }
    
    // 调用通用姿态设置函数
    return setPosture(posture);
}

robot_base_interfaces::motion_interface::MotionResult Go2MotionController::emergencyStop(
    robot_base_interfaces::motion_interface::EmergencyStopLevel level) {
    
    RCLCPP_WARN(this->get_logger(), "执行紧急停止，级别: %d", static_cast<int>(level));
    
    bool success = false;
    
    switch (level) {
        case robot_base_interfaces::motion_interface::EmergencyStopLevel::SOFT_STOP:
            // 软停止：先设置速度为0，然后切换到阻尼模式
            success = sendVelocityCommand(0.0f, 0.0f, 0.0f);
            if (success) {
                success = sendGo2Command(1007); // Go2 API: 阻尼模式
            }
            break;
            
        case robot_base_interfaces::motion_interface::EmergencyStopLevel::HARD_STOP:
            // 硬停止：立即切换到关节锁定模式
            success = sendGo2Command(1006); // Go2 API: 关节锁定
            break;
            
        case robot_base_interfaces::motion_interface::EmergencyStopLevel::POWER_OFF:
            // 断电停止：这需要硬件级别的控制，这里暂时实现为硬停止
            RCLCPP_WARN(this->get_logger(), "断电停止功能需要硬件支持，执行硬停止代替");
            success = sendGo2Command(1006); // Go2 API: 关节锁定
            break;
    }
    
    if (success) {
        // 更新运动状态
        {
            std::lock_guard<std::mutex> lock(motion_state_mutex_);
            current_motion_state_.velocity = robot_base_interfaces::motion_interface::Velocity();
            current_motion_state_.is_moving = false;
            current_motion_state_.current_mode = robot_base_interfaces::motion_interface::MotionMode::IDLE;
            current_motion_state_.timestamp_ns = this->get_clock()->now().nanoseconds();
        }
        
        triggerStateCallback(current_motion_state_);
        
        RCLCPP_INFO(this->get_logger(), "紧急停止执行成功");
        return robot_base_interfaces::motion_interface::MotionResult::SUCCESS;
    } else {
        RCLCPP_ERROR(this->get_logger(), "紧急停止执行失败");
        return robot_base_interfaces::motion_interface::MotionResult::COMMUNICATION_ERROR;
    }
}

robot_base_interfaces::motion_interface::MotionResult Go2MotionController::switchMode(
    robot_base_interfaces::motion_interface::MotionMode mode) {
    
    // 检查控制器是否已初始化
    if (!is_initialized_) {
        return robot_base_interfaces::motion_interface::MotionResult::COMMUNICATION_ERROR;
    }
    
    uint32_t api_id = 0;
    bool success = false;
    
    // 将统一运动模式转换为Go2 API命令
    switch (mode) {
        case robot_base_interfaces::motion_interface::MotionMode::IDLE:
            api_id = 1001; // Go2 API: 待机模式
            break;
        case robot_base_interfaces::motion_interface::MotionMode::BALANCE_STAND:
            api_id = 1002; // Go2 API: 平衡站立
            break;
        case robot_base_interfaces::motion_interface::MotionMode::LOCOMOTION:
            api_id = 1003; // Go2 API: 运动模式
            break;
        case robot_base_interfaces::motion_interface::MotionMode::LIE_DOWN:
            api_id = 1005; // Go2 API: 趴下
            break;
        case robot_base_interfaces::motion_interface::MotionMode::SIT:
            api_id = 1009; // Go2 API: 坐下
            break;
        case robot_base_interfaces::motion_interface::MotionMode::RECOVERY_STAND:
            api_id = 1006; // Go2 API: 恢复站立
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "不支持的运动模式: %d", static_cast<int>(mode));
            return robot_base_interfaces::motion_interface::MotionResult::CAPABILITY_LIMITED;
    }
    
    // 发送模式切换命令
    success = sendGo2Command(api_id);
    
    if (success) {
        // 更新当前运动模式
        {
            std::lock_guard<std::mutex> lock(motion_state_mutex_);
            current_motion_state_.current_mode = mode;
            current_motion_state_.timestamp_ns = this->get_clock()->now().nanoseconds();
        }
        
        triggerStateCallback(current_motion_state_);
        
        RCLCPP_INFO(this->get_logger(), "成功切换到运动模式: %d", static_cast<int>(mode));
        return robot_base_interfaces::motion_interface::MotionResult::SUCCESS;
    } else {
        RCLCPP_ERROR(this->get_logger(), "切换运动模式失败");
        return robot_base_interfaces::motion_interface::MotionResult::COMMUNICATION_ERROR;
    }
}

robot_base_interfaces::motion_interface::MotionResult Go2MotionController::setGaitType(
    robot_base_interfaces::motion_interface::GaitType gait) {
    
    // Go2的步态类型通过运动参数设置，这里记录当前步态
    {
        std::lock_guard<std::mutex> lock(motion_state_mutex_);
        current_motion_state_.current_gait = gait;
        current_motion_state_.timestamp_ns = this->get_clock()->now().nanoseconds();
    }
    
    triggerStateCallback(current_motion_state_);
    
    RCLCPP_INFO(this->get_logger(), "设置步态类型: %d", static_cast<int>(gait));
    return robot_base_interfaces::motion_interface::MotionResult::SUCCESS;
}

// ============= Go2基本动作API实现 =============

robot_base_interfaces::motion_interface::MotionResult Go2MotionController::balanceStand() {
    RCLCPP_INFO(this->get_logger(), "执行平衡站立动作");
    bool success = sendGo2Command(1002); // Go2 API: 平衡站立
    return success ? robot_base_interfaces::motion_interface::MotionResult::SUCCESS : 
                    robot_base_interfaces::motion_interface::MotionResult::COMMUNICATION_ERROR;
}

robot_base_interfaces::motion_interface::MotionResult Go2MotionController::standUp() {
    RCLCPP_INFO(this->get_logger(), "执行站起动作");
    bool success = sendGo2Command(1004); // Go2 API: 站起
    return success ? robot_base_interfaces::motion_interface::MotionResult::SUCCESS : 
                    robot_base_interfaces::motion_interface::MotionResult::COMMUNICATION_ERROR;
}

robot_base_interfaces::motion_interface::MotionResult Go2MotionController::standDown() {
    RCLCPP_INFO(this->get_logger(), "执行趴下动作");
    bool success = sendGo2Command(1005); // Go2 API: 趴下
    return success ? robot_base_interfaces::motion_interface::MotionResult::SUCCESS : 
                    robot_base_interfaces::motion_interface::MotionResult::COMMUNICATION_ERROR;
}

robot_base_interfaces::motion_interface::MotionResult Go2MotionController::sit() {
    RCLCPP_INFO(this->get_logger(), "执行坐下动作");
    bool success = sendGo2Command(1009); // Go2 API: 坐下
    return success ? robot_base_interfaces::motion_interface::MotionResult::SUCCESS : 
                    robot_base_interfaces::motion_interface::MotionResult::COMMUNICATION_ERROR;
}

robot_base_interfaces::motion_interface::MotionResult Go2MotionController::recoveryStand() {
    RCLCPP_INFO(this->get_logger(), "执行恢复站立动作");
    bool success = sendGo2Command(1006); // Go2 API: 恢复站立
    return success ? robot_base_interfaces::motion_interface::MotionResult::SUCCESS : 
                    robot_base_interfaces::motion_interface::MotionResult::COMMUNICATION_ERROR;
}

// ============= Go2特有高级功能实现 =============

robot_base_interfaces::motion_interface::MotionResult Go2MotionController::performDance(int dance_type) {
    uint32_t api_id;
    switch (dance_type) {
        case 1:
            api_id = 1018; // Go2 API: Dance1
            break;
        case 2:
            api_id = 1019; // Go2 API: Dance2
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "不支持的舞蹈类型: %d", dance_type);
            return robot_base_interfaces::motion_interface::MotionResult::INVALID_PARAMETER;
    }
    
    RCLCPP_INFO(this->get_logger(), "执行舞蹈动作，类型: %d", dance_type);
    bool success = sendGo2Command(api_id);
    return success ? robot_base_interfaces::motion_interface::MotionResult::SUCCESS : 
                    robot_base_interfaces::motion_interface::MotionResult::COMMUNICATION_ERROR;
}

robot_base_interfaces::motion_interface::MotionResult Go2MotionController::frontFlip() {
    RCLCPP_INFO(this->get_logger(), "执行前翻动作");
    bool success = sendGo2Command(1030); // Go2 API: 前翻
    return success ? robot_base_interfaces::motion_interface::MotionResult::SUCCESS : 
                    robot_base_interfaces::motion_interface::MotionResult::COMMUNICATION_ERROR;
}

robot_base_interfaces::motion_interface::MotionResult Go2MotionController::frontJump() {
    RCLCPP_INFO(this->get_logger(), "执行前跳动作");
    bool success = sendGo2Command(1031); // Go2 API: 前跳
    return success ? robot_base_interfaces::motion_interface::MotionResult::SUCCESS : 
                    robot_base_interfaces::motion_interface::MotionResult::COMMUNICATION_ERROR;
}

robot_base_interfaces::motion_interface::MotionResult Go2MotionController::hello() {
    RCLCPP_INFO(this->get_logger(), "执行打招呼动作");
    bool success = sendGo2Command(1016); // Go2 API: 打招呼
    return success ? robot_base_interfaces::motion_interface::MotionResult::SUCCESS : 
                    robot_base_interfaces::motion_interface::MotionResult::COMMUNICATION_ERROR;
}

robot_base_interfaces::motion_interface::MotionResult Go2MotionController::stretch() {
    RCLCPP_INFO(this->get_logger(), "执行伸展动作");
    bool success = sendGo2Command(1017); // Go2 API: 伸展
    return success ? robot_base_interfaces::motion_interface::MotionResult::SUCCESS : 
                    robot_base_interfaces::motion_interface::MotionResult::COMMUNICATION_ERROR;
}

robot_base_interfaces::motion_interface::MotionResult Go2MotionController::setSpeedLevel(int level) {
    // 验证速度等级范围
    if (level < 1 || level > 9) {
        RCLCPP_WARN(this->get_logger(), "速度等级%d超出范围[1-9]", level);
        return robot_base_interfaces::motion_interface::MotionResult::INVALID_PARAMETER;
    }
    
    RCLCPP_INFO(this->get_logger(), "设置速度等级: %d", level);
    
    // 构建速度等级参数字符串
    std::string parameter = std::to_string(level);
    bool success = sendGo2Command(1015, parameter); // Go2 API: 设置速度等级
    
    return success ? robot_base_interfaces::motion_interface::MotionResult::SUCCESS : 
                    robot_base_interfaces::motion_interface::MotionResult::COMMUNICATION_ERROR;
}

// ============= 状态查询接口实现 =============

robot_base_interfaces::motion_interface::MotionState Go2MotionController::getMotionState() const {
    std::lock_guard<std::mutex> lock(motion_state_mutex_);
    return current_motion_state_;
}

bool Go2MotionController::isOperational() const {
    return is_initialized_ && (current_error_code_ == 0);
}

uint32_t Go2MotionController::getErrorCode() const {
    return current_error_code_;
}

bool Go2MotionController::isMotionCompleted() const {
    // 检查最后一次命令是否超过超时时间
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_command_time_);
    
    // 如果超过命令超时时间，认为动作已完成
    return elapsed.count() > COMMAND_TIMEOUT_MS;
}

// ============= 回调函数设置实现 =============

void Go2MotionController::setStateCallback(
    std::function<void(const robot_base_interfaces::motion_interface::MotionState&)> callback) {
    state_callback_ = callback;
    RCLCPP_DEBUG(this->get_logger(), "状态变化回调函数已设置");
}

void Go2MotionController::setErrorCallback(
    std::function<void(uint32_t error_code, const std::string& error_msg)> callback) {
    error_callback_ = callback;
    RCLCPP_DEBUG(this->get_logger(), "错误事件回调函数已设置");
}

robot_base_interfaces::motion_interface::RobotType Go2MotionController::getRobotType() const {
    return robot_base_interfaces::motion_interface::RobotType::GO2;
}

// ============= 私有方法实现 =============

void Go2MotionController::initializeROS2Communications() {
    RCLCPP_INFO(this->get_logger(), "初始化ROS2通信组件...");
    
    // 创建Go2 API请求发布器
    api_request_pub_ = this->create_publisher<unitree_api::msg::Request>(
        "/api/sport/request",  // Go2 API话题
        10
    );
    
    // 创建Go2运动状态订阅器
    sport_state_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
        "/sportmodestate",     // Go2状态话题
        10,
        std::bind(&Go2MotionController::sportModeStateCallback, this, std::placeholders::_1)
    );
    
    // 创建ROS2标准速度命令发布器（可选，用于与其他ROS2节点兼容）
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        10
    );
    
    RCLCPP_INFO(this->get_logger(), "ROS2通信组件初始化完成");
}

void Go2MotionController::sportModeStateCallback(const unitree_go::msg::SportModeState::SharedPtr msg) {
    // 将Go2原生状态转换为统一格式
    auto new_state = convertSportStateToMotionState(msg);
    
    // 更新当前状态
    {
        std::lock_guard<std::mutex> lock(motion_state_mutex_);
        current_motion_state_ = new_state;
        current_error_code_ = msg->error_code;
    }
    
    // 如果有错误，触发错误回调
    if (msg->error_code != 0) {
        std::string error_msg = "Go2系统错误代码: " + std::to_string(msg->error_code);
        triggerErrorCallback(msg->error_code, error_msg);
    }
    
    // 触发状态变化回调
    triggerStateCallback(new_state);
    
    RCLCPP_DEBUG(this->get_logger(), "接收到Go2状态更新，模式: %d, 错误码: %d", 
                 msg->mode, msg->error_code);
}

bool Go2MotionController::sendGo2Command(uint32_t api_id, const std::string& parameter) {
    // 检查发布器是否有效
    if (!api_request_pub_) {
        RCLCPP_ERROR(this->get_logger(), "Go2 API发布器未初始化");
        return false;
    }
    
    // 构造Go2 API请求消息
    auto request_msg = std::make_unique<unitree_api::msg::Request>();
    
    // 设置请求头
    request_msg->header.identity.id = 0;  // 请求ID
    request_msg->header.identity.api_id = api_id;  // API ID
    request_msg->header.lease.id = 0;  // 租约ID
    request_msg->header.policy.priority = 0;  // 优先级
    request_msg->header.policy.noreply = false;  // 需要回复
    
    // 设置API参数
    request_msg->parameter = parameter;
    
    // 发布命令
    api_request_pub_->publish(std::move(request_msg));
    
    // 更新命令发送时间
    last_command_time_ = std::chrono::steady_clock::now();
    
    RCLCPP_DEBUG(this->get_logger(), "发送Go2命令: ID=%d, 参数=%s", api_id, parameter.c_str());
    return true;
}

bool Go2MotionController::sendVelocityCommand(float vx, float vy, float wz) {
    // 同时发送到Go2 API和ROS2标准话题
    
    // 1. 发送ROS2标准Twist消息
    if (cmd_vel_pub_) {
        auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
        twist_msg->linear.x = vx;
        twist_msg->linear.y = vy;
        twist_msg->linear.z = 0.0;
        twist_msg->angular.x = 0.0;
        twist_msg->angular.y = 0.0;
        twist_msg->angular.z = wz;
        
        cmd_vel_pub_->publish(std::move(twist_msg));
    }
    
    // 2. 发送Go2特定的速度控制命令
    // 这里需要根据Go2的实际API格式构造参数字符串
    std::string velocity_params = std::to_string(vx) + "," + 
                                 std::to_string(vy) + "," + 
                                 std::to_string(wz);
    
    return sendGo2Command(1003, velocity_params); // 1003可能是Go2的速度控制API ID
}

bool Go2MotionController::sendPostureCommand(float roll, float pitch, float yaw, float body_height) {
    // 构造姿态控制参数字符串
    std::string posture_params = std::to_string(roll) + "," + 
                                std::to_string(pitch) + "," + 
                                std::to_string(yaw) + "," + 
                                std::to_string(body_height);
    
    return sendGo2Command(1002, posture_params); // 1002可能是Go2的姿态控制API ID
}

bool Go2MotionController::validateVelocityLimits(
    const robot_base_interfaces::motion_interface::Velocity& velocity) const {
    
    return (std::abs(velocity.linear_x) <= MAX_LINEAR_VELOCITY &&
            std::abs(velocity.linear_y) <= MAX_LATERAL_VELOCITY &&
            std::abs(velocity.angular_z) <= MAX_ANGULAR_VELOCITY);
}

bool Go2MotionController::validatePostureLimits(
    const robot_base_interfaces::motion_interface::Posture& posture) const {
    
    return (std::abs(posture.roll) <= MAX_ROLL_ANGLE &&
            std::abs(posture.pitch) <= MAX_PITCH_ANGLE &&
            posture.body_height >= MIN_BODY_HEIGHT &&
            posture.body_height <= MAX_BODY_HEIGHT);
}

robot_base_interfaces::motion_interface::MotionState 
Go2MotionController::convertSportStateToMotionState(
    const unitree_go::msg::SportModeState::SharedPtr& sport_state) const {
    
    robot_base_interfaces::motion_interface::MotionState motion_state;
    
    if (!sport_state) {
        return motion_state;
    }
    
    // 转换时间戳 (SportModeState使用TimeSpec而非标准Header)
    motion_state.timestamp_ns = sport_state->stamp.sec * 1000000000ULL + sport_state->stamp.nanosec;
    motion_state.error_code = sport_state->error_code;
    
    // 转换运动模式（需要根据Go2的实际模式编号调整）
    switch (sport_state->mode) {
        case 0:
            motion_state.current_mode = robot_base_interfaces::motion_interface::MotionMode::IDLE;
            break;
        case 1:
            motion_state.current_mode = robot_base_interfaces::motion_interface::MotionMode::BALANCE_STAND;
            break;
        case 3:
            motion_state.current_mode = robot_base_interfaces::motion_interface::MotionMode::LOCOMOTION;
            break;
        default:
            motion_state.current_mode = robot_base_interfaces::motion_interface::MotionMode::IDLE;
    }
    
    // 转换步态类型
    switch (sport_state->gait_type) {
        case 1:
            motion_state.current_gait = robot_base_interfaces::motion_interface::GaitType::TROT;
            break;
        case 2:
            motion_state.current_gait = robot_base_interfaces::motion_interface::GaitType::RUN;
            break;
        default:
            motion_state.current_gait = robot_base_interfaces::motion_interface::GaitType::IDLE;
    }
    
    // 转换位置信息
    if (sport_state->position.size() >= 3) {
        motion_state.position.x = sport_state->position[0];
        motion_state.position.y = sport_state->position[1];
        motion_state.position.z = sport_state->position[2];
    }
    
    // 转换速度信息
    if (sport_state->velocity.size() >= 3) {
        motion_state.velocity.linear_x = sport_state->velocity[0];
        motion_state.velocity.linear_y = sport_state->velocity[1];
        motion_state.velocity.angular_z = sport_state->yaw_speed;
    }
    
    // 转换姿态信息
    motion_state.posture.body_height = sport_state->body_height;
    // IMU数据包含在sport_state->imu_state中，需要提取RPY角度
    
    // 转换足端信息
    if (sport_state->foot_force.size() >= 4) {
        motion_state.foot_forces.assign(sport_state->foot_force.begin(), sport_state->foot_force.end());
    }
    
    // 转换障碍物信息
    if (sport_state->range_obstacle.size() >= 4) {
        motion_state.range_obstacles.assign(sport_state->range_obstacle.begin(), sport_state->range_obstacle.end());
    }
    
    // 设置运动状态标志
    motion_state.is_moving = (std::abs(motion_state.velocity.linear_x) > 0.01f || 
                             std::abs(motion_state.velocity.linear_y) > 0.01f || 
                             std::abs(motion_state.velocity.angular_z) > 0.01f);
    motion_state.is_balanced = (motion_state.current_mode == 
                               robot_base_interfaces::motion_interface::MotionMode::BALANCE_STAND);
    
    return motion_state;
}

void Go2MotionController::triggerStateCallback(
    const robot_base_interfaces::motion_interface::MotionState& new_state) {
    
    if (state_callback_) {
        try {
            state_callback_(new_state);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "状态回调函数执行出错: %s", e.what());
        }
    }
}

void Go2MotionController::triggerErrorCallback(uint32_t error_code, const std::string& error_msg) {
    if (error_callback_) {
        try {
            error_callback_(error_code, error_msg);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "错误回调函数执行出错: %s", e.what());
        }
    }
}

} // namespace go2_adapter
} // namespace robot_adapters