/**
 * @file go2_message_converter.cpp
 * @brief Go2机器人消息转换器实现
 * @author Claude Code
 * @date 2024
 */

#include "robot_adapters/go2_adapter/go2_message_converter.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <sstream>
#include <iomanip>

namespace robot_adapters {
namespace go2_adapter {

// ============= 构造函数和析构函数 =============

Go2MessageConverter::Go2MessageConverter() {
    // 重置为默认配置
    resetToDefaults();
    // 初始化映射表和变换矩阵
    initializeMappingTables();
    initializeTransformMatrices();
}

Go2MessageConverter::Go2MessageConverter(const ConversionOptions& options) 
    : options_(options) {
    // 使用指定配置初始化映射表和变换矩阵
    initializeMappingTables();
    initializeTransformMatrices();
}

Go2MessageConverter::~Go2MessageConverter() = default;

// ============= 配置管理 =============

void Go2MessageConverter::setConversionOptions(const ConversionOptions& options) {
    options_ = options;
}

void Go2MessageConverter::resetToDefaults() {
    options_ = ConversionOptions{};
    resetStatistics();
}

// ============= 初始化函数 =============

void Go2MessageConverter::initializeMappingTables() {
    // 初始化运动模式映射表
    // Go2运动模式: 0=待机, 1=平衡站立, 2=姿态控制, 3=运动模式, 5=紧急停止
    motion_mode_map_[0] = robot_base_interfaces::motion_interface::MotionMode::IDLE;
    motion_mode_map_[1] = robot_base_interfaces::motion_interface::MotionMode::BALANCE_STAND;
    motion_mode_map_[2] = robot_base_interfaces::motion_interface::MotionMode::POSE;
    motion_mode_map_[3] = robot_base_interfaces::motion_interface::MotionMode::LOCOMOTION;
    motion_mode_map_[5] = robot_base_interfaces::motion_interface::MotionMode::LIE_DOWN;
    
    // 初始化步态类型映射表
    // Go2步态类型: 0=空闲, 1=小跑, 2=跑步, 3=步行
    gait_type_map_[0] = robot_base_interfaces::motion_interface::GaitType::IDLE;
    gait_type_map_[1] = robot_base_interfaces::motion_interface::GaitType::TROT;
    gait_type_map_[2] = robot_base_interfaces::motion_interface::GaitType::RUN;
    gait_type_map_[3] = robot_base_interfaces::motion_interface::GaitType::CLIMB_STAIR;
    gait_type_map_[4] = robot_base_interfaces::motion_interface::GaitType::DOWN_STAIR;
    
    // 初始化电池健康状态映射表
    battery_health_map_[0] = robot_base_interfaces::power_interface::BatteryHealth::GOOD;
    battery_health_map_[1] = robot_base_interfaces::power_interface::BatteryHealth::FAIR;
    battery_health_map_[2] = robot_base_interfaces::power_interface::BatteryHealth::POOR;
    battery_health_map_[3] = robot_base_interfaces::power_interface::BatteryHealth::POOR;
    
    // 初始化充电状态映射表
    charging_state_map_[0] = robot_base_interfaces::power_interface::ChargingState::NOT_CHARGING;
    charging_state_map_[1] = robot_base_interfaces::power_interface::ChargingState::CHARGING;
    charging_state_map_[2] = robot_base_interfaces::power_interface::ChargingState::FULL;
    charging_state_map_[3] = robot_base_interfaces::power_interface::ChargingState::ERROR;
}

void Go2MessageConverter::initializeTransformMatrices() {
    // 初始化Go2到ROS坐标系转换矩阵
    // 默认假设坐标系相同，使用单位矩阵
    transforms_.go2_to_ros = {{1.0f, 0.0f, 0.0f},
                              {0.0f, 1.0f, 0.0f},
                              {0.0f, 0.0f, 1.0f}};
    
    transforms_.ros_to_go2 = {{1.0f, 0.0f, 0.0f},
                              {0.0f, 1.0f, 0.0f},
                              {0.0f, 0.0f, 1.0f}};
}

// ============= 基础转换函数 =============

robot_base_interfaces::motion_interface::MotionMode 
Go2MessageConverter::convertMotionMode(uint8_t go2_mode) const {
    // 根据映射表转换Go2运动模式到统一运动模式
    auto it = motion_mode_map_.find(go2_mode);
    if (it != motion_mode_map_.end()) {
        return it->second;
    }
    
    // 未知模式，设置错误信息并返回默认值
    setError("未知的Go2运动模式: " + std::to_string(go2_mode));
    return robot_base_interfaces::motion_interface::MotionMode::IDLE;
}

robot_base_interfaces::motion_interface::GaitType 
Go2MessageConverter::convertGaitType(uint8_t go2_gait) const {
    // 根据映射表转换Go2步态类型到统一步态类型
    auto it = gait_type_map_.find(go2_gait);
    if (it != gait_type_map_.end()) {
        return it->second;
    }
    
    // 未知步态，设置错误信息并返回默认值
    setError("未知的Go2步态类型: " + std::to_string(go2_gait));
    return robot_base_interfaces::motion_interface::GaitType::IDLE;
}

robot_base_interfaces::power_interface::BatteryHealth 
Go2MessageConverter::convertBatteryHealth(uint32_t go2_status) const {
    // 根据映射表转换Go2电池健康状态
    auto it = battery_health_map_.find(go2_status);
    if (it != battery_health_map_.end()) {
        return it->second;
    }
    
    // 未知状态，设置错误信息并返回默认值
    setError("未知的Go2电池健康状态: " + std::to_string(go2_status));
    return robot_base_interfaces::power_interface::BatteryHealth::UNKNOWN;
}

robot_base_interfaces::power_interface::ChargingState 
Go2MessageConverter::convertChargingState(uint8_t go2_charging_status) const {
    // 根据映射表转换Go2充电状态
    auto it = charging_state_map_.find(go2_charging_status);
    if (it != charging_state_map_.end()) {
        return it->second;
    }
    
    // 未知状态，设置错误信息并返回默认值
    setError("未知的Go2充电状态: " + std::to_string(go2_charging_status));
    return robot_base_interfaces::power_interface::ChargingState::UNKNOWN;
}

// ============= 运动状态转换函数 =============

ConversionResult Go2MessageConverter::convertSportModeState(
    const unitree_go::msg::SportModeState& go2_state,
    robot_base_interfaces::motion_interface::MotionState& unified_state) const {
    
    // 记录转换开始时间，用于性能统计
    auto start_time = std::chrono::high_resolution_clock::now();
    
    try {
        // 转换时间戳：Go2使用TimeSpec结构体，转换为纳秒时间戳
        unified_state.timestamp_ns = go2_state.stamp.sec * 1000000000ULL + go2_state.stamp.nanosec;
        unified_state.error_code = go2_state.error_code;
        
        // 转换运动模式和步态类型
        unified_state.current_mode = convertMotionMode(go2_state.mode);
        unified_state.current_gait = convertGaitType(go2_state.gait_type);
        
        // 转换位置信息：Go2位置是std::array<float, 3>，包含x,y,z三个分量
        unified_state.position.x = go2_state.position[0];
        unified_state.position.y = go2_state.position[1];
        unified_state.position.z = go2_state.position[2];
        
        // 转换速度信息：Go2速度是std::array<float, 3>，线性速度和角速度
        unified_state.velocity.linear_x = go2_state.velocity[0];
        unified_state.velocity.linear_y = go2_state.velocity[1];
        unified_state.velocity.angular_z = go2_state.yaw_speed; // 偏航角速度单独提供
        
        // 转换姿态信息：身体高度
        unified_state.posture.body_height = go2_state.body_height;
        
        // 转换足端力信息：Go2 foot_force是std::array<int16_t, 4>
        unified_state.foot_forces.resize(4);
        for (size_t i = 0; i < 4; ++i) {
            unified_state.foot_forces[i] = static_cast<float>(go2_state.foot_force[i]);
        }
        
        // 转换障碍物检测信息：Go2 range_obstacle是std::array<float, 4>
        unified_state.range_obstacles.resize(4);
        for (size_t i = 0; i < 4; ++i) {
            unified_state.range_obstacles[i] = go2_state.range_obstacle[i];
        }
        
        // 设置运动状态标志位：根据速度判断是否在运动
        const float velocity_threshold = 0.01f; // 速度阈值
        unified_state.is_moving = (std::abs(unified_state.velocity.linear_x) > velocity_threshold || 
                                  std::abs(unified_state.velocity.linear_y) > velocity_threshold || 
                                  std::abs(unified_state.velocity.angular_z) > velocity_threshold);
        
        // 判断是否处于平衡站立状态
        unified_state.is_balanced = (unified_state.current_mode == 
                                   robot_base_interfaces::motion_interface::MotionMode::BALANCE_STAND);
        
        // 记录转换成功的统计信息
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("SportModeState", true, duration.count() / 1000.0);
        
        return ConversionResult::SUCCESS;
        
    } catch (const std::exception& e) {
        // 捕获转换过程中的异常，记录错误并返回转换失败
        setError("SportModeState转换失败: " + std::string(e.what()));
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("SportModeState", false, duration.count() / 1000.0);
        return ConversionResult::CONVERSION_ERROR;
    }
}

ConversionResult Go2MessageConverter::convertMotionState(
    const robot_base_interfaces::motion_interface::MotionState& unified_state,
    unitree_go::msg::SportModeState& go2_state) const {
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    try {
        // 转换时间戳：从纳秒时间戳转换为Go2的TimeSpec结构
        go2_state.stamp.sec = unified_state.timestamp_ns / 1000000000ULL;
        go2_state.stamp.nanosec = unified_state.timestamp_ns % 1000000000ULL;
        go2_state.error_code = unified_state.error_code;
        
        // 反向查找运动模式：从统一模式转换为Go2模式值
        bool mode_found = false;
        for (const auto& pair : motion_mode_map_) {
            if (pair.second == unified_state.current_mode) {
                go2_state.mode = pair.first;
                mode_found = true;
                break;
            }
        }
        if (!mode_found && options_.strict_validation) {
            setError("无法找到对应的Go2运动模式");
            return ConversionResult::UNSUPPORTED_TYPE;
        }
        
        // 反向查找步态类型
        bool gait_found = false;
        for (const auto& pair : gait_type_map_) {
            if (pair.second == unified_state.current_gait) {
                go2_state.gait_type = pair.first;
                gait_found = true;
                break;
            }
        }
        if (!gait_found && options_.strict_validation) {
            setError("无法找到对应的Go2步态类型");
            return ConversionResult::UNSUPPORTED_TYPE;
        }
        
        // 转换位置信息 - Go2 position是std::array<float, 3>
        go2_state.position[0] = unified_state.position.x;
        go2_state.position[1] = unified_state.position.y;
        go2_state.position[2] = unified_state.position.z;
        
        // 转换速度信息 - Go2 velocity是std::array<float, 3>
        go2_state.velocity[0] = unified_state.velocity.linear_x;
        go2_state.velocity[1] = unified_state.velocity.linear_y;
        go2_state.velocity[2] = 0.0f; // Go2的z方向线性速度通常为0
        go2_state.yaw_speed = unified_state.velocity.angular_z;
        
        // 转换姿态信息
        go2_state.body_height = unified_state.posture.body_height;
        
        // 转换足端力信息 - Go2 foot_force是std::array<int16_t, 4>
        for (size_t i = 0; i < 4; ++i) {
            if (i < unified_state.foot_forces.size()) {
                go2_state.foot_force[i] = static_cast<int16_t>(unified_state.foot_forces[i]);
            } else {
                go2_state.foot_force[i] = 0;
            }
        }
        
        // 转换障碍物信息 - Go2 range_obstacle是std::array<float, 4>
        for (size_t i = 0; i < 4; ++i) {
            if (i < unified_state.range_obstacles.size()) {
                go2_state.range_obstacle[i] = unified_state.range_obstacles[i];
            } else {
                go2_state.range_obstacle[i] = 0.0f;
            }
        }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("MotionState", true, duration.count() / 1000.0);
        
        return ConversionResult::SUCCESS;
        
    } catch (const std::exception& e) {
        setError("MotionState转换失败: " + std::string(e.what()));
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("MotionState", false, duration.count() / 1000.0);
        return ConversionResult::CONVERSION_ERROR;
    }
}

// ============= 机器人状态转换函数 =============

ConversionResult Go2MessageConverter::convertLowState(
    const unitree_go::msg::LowState& go2_state,
    robot_base_interfaces::state_interface::DetailedRobotState& unified_state) const {
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    try {
        // Go2 LowState没有stamp字段，使用当前系统时间
        auto now = std::chrono::system_clock::now();
        auto time_duration = now.time_since_epoch();
        unified_state.timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(time_duration).count();
        
        // 转换电机信息：Go2有12个电机（每条腿3个关节）
        unified_state.motors.clear();
        unified_state.motors.reserve(std::min(static_cast<size_t>(12), go2_state.motor_state.size()));
        
        // Go2有20个电机（每条腿5个关节），但我们只使用前12个（每条腿3个）
        for (size_t i = 0; i < 12 && i < go2_state.motor_state.size(); ++i) {
            const auto& go2_motor = go2_state.motor_state[i];
            
            robot_base_interfaces::state_interface::MotorInfo motor_info;
            motor_info.motor_id = static_cast<uint8_t>(i);
            motor_info.temperature = go2_motor.temperature;
            motor_info.position = go2_motor.q;      // 关节位置
            motor_info.velocity = go2_motor.dq;     // 关节速度
            motor_info.torque_estimated = go2_motor.tau_est; // 估计力矩
            motor_info.is_online = (go2_motor.temperature > -100.0f); // 根据温度判断是否在线
            
            // 验证数据范围（如果启用了范围验证）
            if (options_.validate_ranges) {
                if (!validateRange(motor_info.temperature, -50.0f, 100.0f, "motor_temperature")) {
                    motor_info.temperature = 25.0f; // 设置默认温度
                }
                if (!validateRange(motor_info.position, -6.28f, 6.28f, "motor_position")) {
                    motor_info.position = 0.0f;
                }
            }
            
            unified_state.motors.push_back(motor_info);
        }
        
        // IMU数据在DetailedRobotState中没有独立的imu_data字段
        // 可以将IMU信息存储在运动状态部分或者扩展数据中
        // 这里暂时跳过IMU数据处理，或者可以添加到扩展字段中
        
        // 记录转换成功统计
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("LowState", true, duration.count() / 1000.0);
        
        return ConversionResult::SUCCESS;
        
    } catch (const std::exception& e) {
        setError("LowState转换失败: " + std::string(e.what()));
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("LowState", false, duration.count() / 1000.0);
        return ConversionResult::CONVERSION_ERROR;
    }
}

ConversionResult Go2MessageConverter::convertMotorInfo(
    const std::vector<unitree_go::msg::MotorState>& go2_motors,
    std::vector<robot_base_interfaces::state_interface::MotorInfo>& unified_motors) const {
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    try {
        unified_motors.clear();
        unified_motors.reserve(go2_motors.size());
        
        // 遍历所有Go2电机状态并转换为统一格式
        for (size_t i = 0; i < go2_motors.size(); ++i) {
            const auto& go2_motor = go2_motors[i];
            
            robot_base_interfaces::state_interface::MotorInfo motor_info;
            motor_info.motor_id = static_cast<uint8_t>(i);
            motor_info.temperature = go2_motor.temperature;
            motor_info.position = go2_motor.q;
            motor_info.velocity = go2_motor.dq;
            motor_info.torque_estimated = go2_motor.tau_est;
            // torque_desired字段在MotorInfo中不存在，使用torque_estimated
            // motor_info.torque_desired = go2_motor.tau_est; // 期望力矩（使用估计力矩）
            
            // 根据电机温度和状态判断是否在线
            motor_info.is_online = (go2_motor.temperature > -100.0f && 
                                  go2_motor.temperature < 100.0f);
            
            // 数据范围验证（如果启用）
            if (options_.validate_ranges) {
                motor_info.temperature = std::clamp(static_cast<float>(motor_info.temperature), -50.0f, 100.0f);
                motor_info.position = std::clamp(motor_info.position, -6.28f, 6.28f);
                motor_info.velocity = std::clamp(motor_info.velocity, -50.0f, 50.0f);
            }
            
            unified_motors.push_back(motor_info);
        }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("MotorInfo", true, duration.count() / 1000.0);
        
        return ConversionResult::SUCCESS;
        
    } catch (const std::exception& e) {
        setError("MotorInfo转换失败: " + std::string(e.what()));
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("MotorInfo", false, duration.count() / 1000.0);
        return ConversionResult::CONVERSION_ERROR;
    }
}

ConversionResult Go2MessageConverter::convertFootInfo(
    const std::array<int16_t, 4>& go2_foot_forces,
    std::vector<robot_base_interfaces::state_interface::FootInfo>& unified_feet) const {
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    try {
        unified_feet.clear();
        unified_feet.reserve(4); // Go2有4条腿
        
        // 足端编号：0=前右，1=前左，2=后右，3=后左
        const std::array<std::string, 4> foot_names = {"front_right", "front_left", 
                                                       "rear_right", "rear_left"};
        
        for (size_t i = 0; i < 4; ++i) {
            robot_base_interfaces::state_interface::FootInfo foot_info;
            foot_info.foot_id = static_cast<uint8_t>(i);
            
            // 转换足端力信息（Go2提供int16_t类型的力数据）
            float force_value = static_cast<float>(go2_foot_forces[i]);
            foot_info.force = force_value;  // 足端力
            foot_info.force_estimated = force_value;  // 估计足端力
            
            // 判断是否接触地面（基于力阈值）
            const float contact_threshold = 10.0f; // 接触力阈值（牛顿）
            foot_info.in_contact = (force_value > contact_threshold);
            foot_info.contact_probability = foot_info.in_contact ? 1.0f : 0.0f;
            
            // 数据范围验证
            if (options_.validate_ranges) {
                foot_info.force = std::clamp(foot_info.force, -100.0f, 1000.0f);
                foot_info.force_estimated = std::clamp(foot_info.force_estimated, -100.0f, 1000.0f);
            }
            
            unified_feet.push_back(foot_info);
        }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("FootInfo", true, duration.count() / 1000.0);
        
        return ConversionResult::SUCCESS;
        
    } catch (const std::exception& e) {
        setError("FootInfo转换失败: " + std::string(e.what()));
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("FootInfo", false, duration.count() / 1000.0);
        return ConversionResult::CONVERSION_ERROR;
    }
}

// ============= 电源状态转换函数 =============

ConversionResult Go2MessageConverter::convertBmsState(
    const unitree_go::msg::BmsState& go2_bms,
    robot_base_interfaces::power_interface::BatteryInfo& unified_battery) const {
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    try {
        // 基础电池信息转换 - 根据Go2实际BmsState结构
        // Go2 BmsState不直接提供电压字段，设置为默认值
        unified_battery.voltage = 24.0; // Go2典型电池电压24V
        
        // 电流转换：Go2 current字段是int32_t，需要转换为实际安培值
        unified_battery.current = static_cast<double>(go2_bms.current) / 1000.0; // 毫安转安培
        
        // 电量百分比转换
        unified_battery.soc_percentage = static_cast<double>(go2_bms.soc);
        
        // 温度信息：使用BQ NTC温度传感器数组的第一个值
        if (go2_bms.bq_ntc.size() > 0) {
            unified_battery.temperature = static_cast<double>(go2_bms.bq_ntc[0]);
        } else {
            unified_battery.temperature = 25.0; // 默认温度
        }
        
        // 转换健康状态（使用status字段）
        unified_battery.health = convertBatteryHealth(go2_bms.status);
        unified_battery.status = static_cast<uint8_t>(go2_bms.status & 0xFF);
        
        // 设置电池循环次数信息
        // unified_battery.cycle_count = go2_bms.cycle; // 如果BatteryInfo有此字段
        
        // Go2 BmsState中没有容量字段，设置典型值
        unified_battery.capacity_mah = 15000; // Go2典型电池容量15Ah
        // 根据SOC计算剩余容量
        unified_battery.remaining_mah = 
            static_cast<double>(unified_battery.capacity_mah * go2_bms.soc / 100.0);
        
        // 数据范围验证
        if (options_.validate_ranges) {
            unified_battery.voltage = std::clamp(static_cast<double>(unified_battery.voltage), 0.0, 100.0);
            unified_battery.current = std::clamp(static_cast<double>(unified_battery.current), -50.0, 50.0);
            unified_battery.soc_percentage = std::clamp(static_cast<double>(unified_battery.soc_percentage), 0.0, 100.0);
            unified_battery.temperature = std::clamp(static_cast<double>(unified_battery.temperature), -20.0, 80.0);
        }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("BmsState", true, duration.count() / 1000.0);
        
        return ConversionResult::SUCCESS;
        
    } catch (const std::exception& e) {
        setError("BmsState转换失败: " + std::string(e.what()));
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("BmsState", false, duration.count() / 1000.0);
        return ConversionResult::CONVERSION_ERROR;
    }
}

// ============= 控制命令转换函数 =============

ConversionResult Go2MessageConverter::convertVelocityCommand(
    const robot_base_interfaces::motion_interface::Velocity& velocity,
    unitree_api::msg::Request& go2_request) const {
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    try {
        // 设置Go2 API请求的基本信息
        go2_request.header.identity.id = 1001; // 速度控制API ID
        go2_request.header.identity.api_id = 1001;
        
        // 构建速度命令参数字符串（JSON格式）
        std::ostringstream param_stream;
        param_stream << std::fixed << std::setprecision(3);
        param_stream << "{";
        param_stream << "\"x\":" << velocity.linear_x << ",";
        param_stream << "\"y\":" << velocity.linear_y << ",";
        param_stream << "\"z\":0.0,"; // Go2通常z方向速度为0
        param_stream << "\"yaw\":" << velocity.angular_z;
        param_stream << "}";
        
        // 将参数字符串复制到请求消息中
        std::string param_str = param_stream.str();
        go2_request.parameter.resize(param_str.length());
        std::copy(param_str.begin(), param_str.end(), go2_request.parameter.begin());
        
        // 数据范围验证和限制
        if (options_.validate_ranges) {
            // Go2的速度限制：线速度±1.5m/s，角速度±2.0rad/s
            if (std::abs(velocity.linear_x) > 1.5f || 
                std::abs(velocity.linear_y) > 1.5f || 
                std::abs(velocity.angular_z) > 2.0f) {
                setError("速度命令超出Go2的安全范围");
                return ConversionResult::INVALID_INPUT;
            }
        }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("VelocityCommand", true, duration.count() / 1000.0);
        
        return ConversionResult::SUCCESS;
        
    } catch (const std::exception& e) {
        setError("VelocityCommand转换失败: " + std::string(e.what()));
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("VelocityCommand", false, duration.count() / 1000.0);
        return ConversionResult::CONVERSION_ERROR;
    }
}

ConversionResult Go2MessageConverter::convertPostureCommand(
    const robot_base_interfaces::motion_interface::Posture& posture,
    unitree_api::msg::Request& go2_request) const {
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    try {
        // 设置Go2姿态控制API请求信息
        go2_request.header.identity.id = 1002; // 姿态控制API ID
        go2_request.header.identity.api_id = 1002;
        
        // 构建姿态命令参数（JSON格式）
        std::ostringstream param_stream;
        param_stream << std::fixed << std::setprecision(3);
        param_stream << "{";
        param_stream << "\"body_height\":" << posture.body_height << ",";
        param_stream << "\"roll\":" << posture.roll << ",";
        param_stream << "\"pitch\":" << posture.pitch << ",";
        param_stream << "\"yaw\":" << posture.yaw;
        param_stream << "}";
        
        std::string param_str = param_stream.str();
        go2_request.parameter.resize(param_str.length());
        std::copy(param_str.begin(), param_str.end(), go2_request.parameter.begin());
        
        // 姿态参数范围验证
        if (options_.validate_ranges) {
            // Go2的姿态限制检查
            if (posture.body_height < 0.05f || posture.body_height > 0.4f) {
                setError("身体高度超出Go2范围 (0.05-0.4m)");
                return ConversionResult::INVALID_INPUT;
            }
            if (std::abs(posture.roll) > 0.5f || std::abs(posture.pitch) > 0.5f) {
                setError("Roll/Pitch角度超出Go2安全范围 (±0.5rad)");
                return ConversionResult::INVALID_INPUT;
            }
        }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("PostureCommand", true, duration.count() / 1000.0);
        
        return ConversionResult::SUCCESS;
        
    } catch (const std::exception& e) {
        setError("PostureCommand转换失败: " + std::string(e.what()));
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("PostureCommand", false, duration.count() / 1000.0);
        return ConversionResult::CONVERSION_ERROR;
    }
}

// ============= 辅助函数和工具函数 =============

void Go2MessageConverter::recordConversion(const std::string& type, bool success, double time_ms) const {
    // 更新总体统计信息
    stats_.total_conversions++;
    if (success) {
        stats_.successful_conversions++;
    } else {
        stats_.failed_conversions++;
        stats_.type_errors[type]++;
    }
    
    // 更新按类型统计
    stats_.type_counts[type]++;
    
    // 更新性能统计
    stats_.total_conversion_time_ms += time_ms;
    stats_.max_conversion_time_ms = std::max(stats_.max_conversion_time_ms, time_ms);
    stats_.min_conversion_time_ms = std::min(stats_.min_conversion_time_ms, time_ms);
}

void Go2MessageConverter::setError(const std::string& error_msg) const {
    last_error_ = error_msg;
    // 可以在这里添加日志记录
    // RCLCPP_ERROR(rclcpp::get_logger("Go2MessageConverter"), "%s", error_msg.c_str());
}

std::string Go2MessageConverter::getLastError() const {
    return last_error_;
}

void Go2MessageConverter::resetStatistics() {
    stats_ = ConversionStats{};
    stats_.min_conversion_time_ms = 999999.0;
}

std::string Go2MessageConverter::getConversionStatistics() const {
    std::ostringstream stats_stream;
    stats_stream << "Go2MessageConverter统计信息:\n";
    stats_stream << "总转换次数: " << stats_.total_conversions << "\n";
    stats_stream << "成功转换: " << stats_.successful_conversions << "\n";
    stats_stream << "失败转换: " << stats_.failed_conversions << "\n";
    
    if (stats_.total_conversions > 0) {
        double avg_time = stats_.total_conversion_time_ms / stats_.total_conversions;
        stats_stream << "平均转换时间: " << std::fixed << std::setprecision(3) 
                    << avg_time << "ms\n";
        stats_stream << "最大转换时间: " << stats_.max_conversion_time_ms << "ms\n";
        stats_stream << "最小转换时间: " << stats_.min_conversion_time_ms << "ms\n";
    }
    
    return stats_stream.str();
}

std::vector<std::string> Go2MessageConverter::getSupportedMessageTypes() const {
    return {"SportModeState", "LowState", "BmsState", "MotorInfo", "FootInfo", 
            "VelocityCommand", "PostureCommand", "IMU", "PointCloud2", "Twist"};
}

bool Go2MessageConverter::validateRange(float value, float min_val, float max_val, 
                                       const std::string& field_name) const {
    if (value < min_val || value > max_val) {
        setError("字段 " + field_name + " 值 " + std::to_string(value) + 
                " 超出范围 [" + std::to_string(min_val) + ", " + 
                std::to_string(max_val) + "]");
        return false;
    }
    return true;
}

bool Go2MessageConverter::validateVector(const std::vector<float>& vec, size_t expected_size, 
                                        const std::string& field_name) const {
    if (vec.size() != expected_size) {
        setError("字段 " + field_name + " 的向量大小 " + std::to_string(vec.size()) + 
                " 不匹配期望大小 " + std::to_string(expected_size));
        return false;
    }
    return true;
}

} // namespace go2_adapter
} // namespace robot_adapters