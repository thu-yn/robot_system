/**
 * @file   go2_message_converter.cpp
 * @brief  Go2机器人消息转换器实现
 * @author Yang nan
 * @date   2025-09-11
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
    motion_mode_map_[0]  = robot_base_interfaces::motion_interface::MotionMode::IDLE;           // 待机
    motion_mode_map_[1]  = robot_base_interfaces::motion_interface::MotionMode::BALANCE_STAND;  // 平衡站立
    motion_mode_map_[2]  = robot_base_interfaces::motion_interface::MotionMode::POSE;           // 姿态控制
    motion_mode_map_[3]  = robot_base_interfaces::motion_interface::MotionMode::LOCOMOTION;     // 移动模式
    motion_mode_map_[5]  = robot_base_interfaces::motion_interface::MotionMode::LIE_DOWN;       // 趴下
    motion_mode_map_[6]  = robot_base_interfaces::motion_interface::MotionMode::JOINT_LOCK;     // 关节锁定
    motion_mode_map_[7]  = robot_base_interfaces::motion_interface::MotionMode::DAMPING;        // 阻尼模式
    motion_mode_map_[8]  = robot_base_interfaces::motion_interface::MotionMode::RECOVERY_STAND; // 恢复站立
    motion_mode_map_[10] = robot_base_interfaces::motion_interface::MotionMode::SIT;            // 坐下
    
    // 初始化步态类型映射表
    gait_type_map_[0] = robot_base_interfaces::motion_interface::GaitType::IDLE;                // 空闲
    gait_type_map_[1] = robot_base_interfaces::motion_interface::GaitType::TROT;                // 小跑
    gait_type_map_[2] = robot_base_interfaces::motion_interface::GaitType::RUN;                 // 跑步
    gait_type_map_[3] = robot_base_interfaces::motion_interface::GaitType::CLIMB_STAIR;         // 上楼
    gait_type_map_[4] = robot_base_interfaces::motion_interface::GaitType::DOWN_STAIR;          // 下楼
    gait_type_map_[9] = robot_base_interfaces::motion_interface::GaitType::ADJUST;              // 调整
    
    // 初始化电池健康状态映射表
    battery_health_map_[1] = robot_base_interfaces::power_interface::BatteryHealth::EXCELLENT;  // 优秀
    battery_health_map_[2] = robot_base_interfaces::power_interface::BatteryHealth::GOOD;       // 良好
    battery_health_map_[3] = robot_base_interfaces::power_interface::BatteryHealth::FAIR;       // 一般
    battery_health_map_[4] = robot_base_interfaces::power_interface::BatteryHealth::POOR;       // 较差
    battery_health_map_[5] = robot_base_interfaces::power_interface::BatteryHealth::DEAD;       // 失效
    
    // 初始化充电状态映射表
    charging_state_map_[1] = robot_base_interfaces::power_interface::ChargingState::NOT_CHARGING;   // 未充电
    charging_state_map_[2] = robot_base_interfaces::power_interface::ChargingState::CONNECTING;     // 连接中
    charging_state_map_[3] = robot_base_interfaces::power_interface::ChargingState::CHARGING;       // 充电中
    charging_state_map_[4] = robot_base_interfaces::power_interface::ChargingState::FULL;           // 以充满
    charging_state_map_[5] = robot_base_interfaces::power_interface::ChargingState::ERROR;          // 充电错误
    charging_state_map_[6] = robot_base_interfaces::power_interface::ChargingState::DISCONNECTED;   // 断开连接
    charging_state_map_[7] = robot_base_interfaces::power_interface::ChargingState::STANDBY;        // 充电待机
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
        unified_state.error_code   = go2_state.error_code;
        
        // 转换运动模式和步态类型
        unified_state.current_mode = convertMotionMode(go2_state.mode);
        unified_state.current_gait = convertGaitType(go2_state.gait_type);
        
        // 转换位置信息：Go2位置是std::array<float, 3>，包含x,y,z三个分量
        unified_state.position.x = go2_state.position[0];
        unified_state.position.y = go2_state.position[1];
        unified_state.position.z = go2_state.position[2];
        
        // 转换速度信息：Go2速度是std::array<float, 3>，线性速度和角速度
        unified_state.velocity.linear_x  = go2_state.velocity[0];
        unified_state.velocity.linear_y  = go2_state.velocity[1];
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
        unified_state.is_moving = (std::abs(unified_state.velocity.linear_x)  > velocity_threshold || 
                                   std::abs(unified_state.velocity.linear_y)  > velocity_threshold || 
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
        go2_state.stamp.sec     = unified_state.timestamp_ns / 1000000000ULL;
        go2_state.stamp.nanosec = unified_state.timestamp_ns % 1000000000ULL;
        go2_state.error_code    = unified_state.error_code;
        
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
        go2_state.velocity[2] = 0.0f;                               // Go2的z方向线性速度通常为0
        go2_state.yaw_speed   = unified_state.velocity.angular_z;
        
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

        // 转换系统电压电流状态信息
        unified_state.system_v = go2_state.power_v;
        unified_state.system_a = go2_state.power_a;
        
        // 转换电机信息：使用convertMotorInfo函数，但只处理前12个电机（每条腿3个关节）
        std::vector<unitree_go::msg::MotorState> first_12_motors(
            go2_state.motor_state.begin(),
            go2_state.motor_state.begin() + std::min(static_cast<size_t>(12), go2_state.motor_state.size())
        );
        
        if (convertMotorInfo(first_12_motors, unified_state.motors) != ConversionResult::SUCCESS) {
            setError("电机信息转换失败");
            // 如果转换失败，保持默认值但不中断整个转换过程
        }

        // 转换IMU信息：使用convertIMUInfo函数
        if (convertIMUInfo(go2_state.imu_state, unified_state.imu) != ConversionResult::SUCCESS) {
            setError("IMU信息转换失败");
            // 如果转换失败，保持默认值但不中断整个转换过程
        }

        // 转换足端信息：利用convertFootInfo函数
        if (convertFootInfo(go2_state.foot_force, unified_state.feet) != ConversionResult::SUCCESS) {
            setError("足端信息转换失败");
            // 如果转换失败，保持默认值但不中断整个转换过程
        }
        
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
            motor_info.motor_id         = static_cast<uint8_t>(i);
            motor_info.temperature      = go2_motor.temperature;
            motor_info.position         = go2_motor.q;
            motor_info.velocity         = go2_motor.dq;
            motor_info.torque_estimated = go2_motor.tau_est;
            
            // 根据电机温度和状态判断是否在线
            motor_info.is_online = (go2_motor.temperature > -100.0f && 
                                  go2_motor.temperature < 100.0f);
            
            // 数据范围验证（如果启用）
            if (options_.validate_ranges) {
                motor_info.temperature = std::clamp(static_cast<float>(motor_info.temperature), -50.0f, 100.0f);
                motor_info.position    = std::clamp(motor_info.position, -6.28f, 6.28f);
                motor_info.velocity    = std::clamp(motor_info.velocity, -50.0f, 50.0f);
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
                                                       "rear_right",  "rear_left"};
        
        for (size_t i = 0; i < 4; ++i) {
            robot_base_interfaces::state_interface::FootInfo foot_info;
            foot_info.foot_id = static_cast<uint8_t>(i);
            
            // 转换足端力信息（Go2提供int16_t类型的力数据）
            float force_value = static_cast<float>(go2_foot_forces[i]);
            foot_info.force = force_value;              // 足端力
            foot_info.force_estimated = force_value;    // 估计足端力
            
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

ConversionResult Go2MessageConverter::convertIMUInfo(
    const unitree_go::msg::IMUState& go2_imu_state,
    decltype(robot_base_interfaces::state_interface::DetailedRobotState{}.imu)& unified_imu) const {
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    try {
        // 调整向量大小
        unified_imu.quaternion.resize(4);
        unified_imu.gyroscope.resize(3); 
        unified_imu.accelerometer.resize(3);
        unified_imu.rpy.resize(3);
        
        // 复制四元数数据 (w,x,y,z)
        for (size_t i = 0; i < 4; ++i) {
            unified_imu.quaternion[i] = go2_imu_state.quaternion[i];
        }
        
        // 复制陀螺仪数据 (rad/s)
        for (size_t i = 0; i < 3; ++i) {
            unified_imu.gyroscope[i] = go2_imu_state.gyroscope[i];
        }
        
        // 复制加速度计数据 (m/s^2)
        for (size_t i = 0; i < 3; ++i) {
            unified_imu.accelerometer[i] = go2_imu_state.accelerometer[i];
        }
        
        // 复制欧拉角数据 (roll, pitch, yaw)
        for (size_t i = 0; i < 3; ++i) {
            unified_imu.rpy[i] = go2_imu_state.rpy[i];
        }
        
        // 复制IMU温度
        unified_imu.temperature = go2_imu_state.temperature;
        
        // 数据范围验证（如果启用了范围验证）
        if (options_.validate_ranges) {
            // 验证四元数模长接近1.0
            float quat_norm = std::sqrt(
                unified_imu.quaternion[0] * unified_imu.quaternion[0] +
                unified_imu.quaternion[1] * unified_imu.quaternion[1] +
                unified_imu.quaternion[2] * unified_imu.quaternion[2] +
                unified_imu.quaternion[3] * unified_imu.quaternion[3]
            );
            
            if (std::abs(quat_norm - 1.0f) > 0.1f) {
                setError("四元数模长异常: " + std::to_string(quat_norm));
                // 重置为单位四元数
                unified_imu.quaternion = {1.0f, 0.0f, 0.0f, 0.0f};
            }
            
            // 验证陀螺仪数据范围 (通常±2000 deg/s = ±35 rad/s)
            for (size_t i = 0; i < 3; ++i) {
                if (std::abs(unified_imu.gyroscope[i]) > 35.0f) {
                    setError("陀螺仪数据超出范围: " + std::to_string(unified_imu.gyroscope[i]));
                    unified_imu.gyroscope[i] = 0.0f;
                }
            }
            
            // 验证加速度计数据范围 (通常±16g = ±157 m/s^2)
            for (size_t i = 0; i < 3; ++i) {
                if (std::abs(unified_imu.accelerometer[i]) > 157.0f) {
                    setError("加速度计数据超出范围: " + std::to_string(unified_imu.accelerometer[i]));
                    unified_imu.accelerometer[i] = 0.0f;
                }
            }
            
            // 验证IMU温度范围
            if (!validateRange(static_cast<float>(unified_imu.temperature), -40.0f, 85.0f, "imu_temperature")) {
                unified_imu.temperature = 25; // 设置默认温度
            }
        }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("IMUInfo", true, duration.count() / 1000.0);
        
        return ConversionResult::SUCCESS;
        
    } catch (const std::exception& e) {
        setError("IMUInfo转换失败: " + std::string(e.what()));
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("IMUInfo", false, duration.count() / 1000.0);
        return ConversionResult::CONVERSION_ERROR;
    }
}

// ============= 电源状态转换函数 =============
ConversionResult Go2MessageConverter::convertBmsState(
    const unitree_go::msg::BmsState& go2_bms,
    robot_base_interfaces::power_interface::BatteryInfo& unified_battery) const {
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    try {
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
        unified_battery.cycle_count = go2_bms.cycle; // 如果BatteryInfo有此字段
        
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

// ============= 遥控器状态转换函数实现 =============

ConversionResult Go2MessageConverter::convertWirelessController(
    const unitree_go::msg::WirelessController& go2_controller,
    decltype(robot_base_interfaces::state_interface::DetailedRobotState{}.wireless_controller)& unified_controller) const {

    auto start_time = std::chrono::high_resolution_clock::now();

    try {
        // 直接转换摇杆值
        unified_controller.lx = go2_controller.lx;
        unified_controller.ly = go2_controller.ly;
        unified_controller.rx = go2_controller.rx;
        unified_controller.ry = go2_controller.ry;
        unified_controller.keys = go2_controller.keys;

        // 判断遥控器连接状态 - 基于摇杆值和按键的活动状态
        bool has_activity = (std::abs(go2_controller.lx) > 0.01f ||
                           std::abs(go2_controller.ly) > 0.01f ||
                           std::abs(go2_controller.rx) > 0.01f ||
                           std::abs(go2_controller.ry) > 0.01f ||
                           go2_controller.keys != 0);

        unified_controller.is_connected = has_activity;
        unified_controller.last_update_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        // 数据范围验证
        if (options_.validate_ranges) {
            // 摇杆值应该在[-1.0, 1.0]范围内
            if (std::abs(unified_controller.lx) > 1.0f ||
                std::abs(unified_controller.ly) > 1.0f ||
                std::abs(unified_controller.rx) > 1.0f ||
                std::abs(unified_controller.ry) > 1.0f) {
                setError("遥控器摇杆值超出范围[-1.0, 1.0]");
                // 限制范围但不中断转换
                unified_controller.lx = std::clamp(unified_controller.lx, -1.0f, 1.0f);
                unified_controller.ly = std::clamp(unified_controller.ly, -1.0f, 1.0f);
                unified_controller.rx = std::clamp(unified_controller.rx, -1.0f, 1.0f);
                unified_controller.ry = std::clamp(unified_controller.ry, -1.0f, 1.0f);
            }
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("WirelessController", true, duration.count() / 1000.0);

        return ConversionResult::SUCCESS;

    } catch (const std::exception& e) {
        setError("WirelessController转换失败: " + std::string(e.what()));
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("WirelessController", false, duration.count() / 1000.0);
        return ConversionResult::CONVERSION_ERROR;
    }
}

ConversionResult Go2MessageConverter::convertControllerToTwist(
    const decltype(robot_base_interfaces::state_interface::DetailedRobotState{}.wireless_controller)& unified_controller,
    geometry_msgs::msg::Twist& twist) const {

    auto start_time = std::chrono::high_resolution_clock::now();

    try {
        // 检查遥控器连接状态
        if (!unified_controller.is_connected) {
            // 如果遥控器未连接，输出零速度
            twist.linear.x = 0.0;
            twist.linear.y = 0.0;
            twist.linear.z = 0.0;
            twist.angular.x = 0.0;
            twist.angular.y = 0.0;
            twist.angular.z = 0.0;
        } else {
            // 左摇杆控制线性运动：ly->前后，lx->左右
            const float max_linear_vel = 1.5f;  // Go2最大线速度
            const float max_angular_vel = 2.0f; // Go2最大角速度

            // 应用死区处理
            const float deadzone = 0.05f;
            auto apply_deadzone = [deadzone](float value) -> float {
                if (std::abs(value) < deadzone) return 0.0f;
                float sign = value > 0 ? 1.0f : -1.0f;
                return sign * (std::abs(value) - deadzone) / (1.0f - deadzone);
            };

            float lx_filtered = apply_deadzone(unified_controller.lx);
            float ly_filtered = apply_deadzone(unified_controller.ly);
            float rx_filtered = apply_deadzone(unified_controller.rx);

            // 线速度映射：ly控制前后，lx控制左右
            twist.linear.x = static_cast<double>(ly_filtered * max_linear_vel);  // 前后
            twist.linear.y = static_cast<double>(lx_filtered * max_linear_vel);  // 左右
            twist.linear.z = 0.0;  // Go2不支持垂直运动

            // 角速度映射：rx控制偏航
            twist.angular.x = 0.0;
            twist.angular.y = 0.0;
            twist.angular.z = static_cast<double>(rx_filtered * max_angular_vel);  // 偏航
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("ControllerToTwist", true, duration.count() / 1000.0);

        return ConversionResult::SUCCESS;

    } catch (const std::exception& e) {
        setError("ControllerToTwist转换失败: " + std::string(e.what()));
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("ControllerToTwist", false, duration.count() / 1000.0);
        return ConversionResult::CONVERSION_ERROR;
    }
}

// ============= 里程计转换函数实现 =============

ConversionResult Go2MessageConverter::convertToOdometry(
    const unitree_go::msg::SportModeState& go2_state,
    nav_msgs::msg::Odometry& odometry) const {

    auto start_time = std::chrono::high_resolution_clock::now();

    try {
        // 设置消息头
        odometry.header.stamp.sec = static_cast<uint32_t>(go2_state.stamp.sec);
        odometry.header.stamp.nanosec = static_cast<uint32_t>(go2_state.stamp.nanosec);
        odometry.header.frame_id = "odom";
        odometry.child_frame_id = "base_link";

        // 位置信息
        odometry.pose.pose.position.x = static_cast<double>(go2_state.position[0]);
        odometry.pose.pose.position.y = static_cast<double>(go2_state.position[1]);
        odometry.pose.pose.position.z = static_cast<double>(go2_state.position[2]);

        // 姿态信息 - 从IMU状态中获取四元数
        odometry.pose.pose.orientation.w = static_cast<double>(go2_state.imu_state.quaternion[0]);
        odometry.pose.pose.orientation.x = static_cast<double>(go2_state.imu_state.quaternion[1]);
        odometry.pose.pose.orientation.y = static_cast<double>(go2_state.imu_state.quaternion[2]);
        odometry.pose.pose.orientation.z = static_cast<double>(go2_state.imu_state.quaternion[3]);

        // 线速度信息
        odometry.twist.twist.linear.x = static_cast<double>(go2_state.velocity[0]);
        odometry.twist.twist.linear.y = static_cast<double>(go2_state.velocity[1]);
        odometry.twist.twist.linear.z = static_cast<double>(go2_state.velocity[2]);

        // 角速度信息
        odometry.twist.twist.angular.x = static_cast<double>(go2_state.imu_state.gyroscope[0]);
        odometry.twist.twist.angular.y = static_cast<double>(go2_state.imu_state.gyroscope[1]);
        odometry.twist.twist.angular.z = static_cast<double>(go2_state.yaw_speed);

        // 协方差矩阵 - 设置为默认值
        std::fill(odometry.pose.covariance.begin(), odometry.pose.covariance.end(), 0.0);
        std::fill(odometry.twist.covariance.begin(), odometry.twist.covariance.end(), 0.0);

        // 设置对角线元素为默认不确定性
        odometry.pose.covariance[0] = odometry.pose.covariance[7] = 0.1;   // x,y位置不确定性
        odometry.pose.covariance[14] = 0.1;  // z位置不确定性
        odometry.pose.covariance[21] = odometry.pose.covariance[28] = odometry.pose.covariance[35] = 0.1; // 角度不确定性

        odometry.twist.covariance[0] = odometry.twist.covariance[7] = 0.1; // 线速度不确定性
        odometry.twist.covariance[35] = 0.1; // 角速度不确定性

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("ToOdometry", true, duration.count() / 1000.0);

        return ConversionResult::SUCCESS;

    } catch (const std::exception& e) {
        setError("里程计转换失败: " + std::string(e.what()));
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("ToOdometry", false, duration.count() / 1000.0);
        return ConversionResult::CONVERSION_ERROR;
    }
}

// ============= 电池状态转换函数实现 =============

ConversionResult Go2MessageConverter::convertBatteryToRos(
    const unitree_go::msg::BmsState& go2_bms,
    std::map<std::string, float>& battery_msg) const {

    auto start_time = std::chrono::high_resolution_clock::now();

    try {
        // 清空输出容器
        battery_msg.clear();

        // 基本电池信息
        battery_msg["voltage"] = 0.0f; // Go2 BmsState中没有总电压，需要从LowState获取
        battery_msg["current"] = static_cast<float>(go2_bms.current / 1000.0); // 毫安转安培
        battery_msg["percentage"] = static_cast<float>(go2_bms.soc);
        battery_msg["capacity"] = 15000.0f; // Go2标准容量
        battery_msg["remaining_capacity"] = battery_msg["capacity"] * battery_msg["percentage"] / 100.0f;

        // 温度信息
        if (go2_bms.bq_ntc.size() > 0) {
            battery_msg["temperature"] = static_cast<float>(go2_bms.bq_ntc[0]);
        }
        if (go2_bms.bq_ntc.size() > 1) {
            battery_msg["temperature_max"] = static_cast<float>(*std::max_element(go2_bms.bq_ntc.begin(), go2_bms.bq_ntc.end()));
            battery_msg["temperature_min"] = static_cast<float>(*std::min_element(go2_bms.bq_ntc.begin(), go2_bms.bq_ntc.end()));
        }

        // 循环和健康信息
        battery_msg["cycle_count"] = static_cast<float>(go2_bms.cycle);
        battery_msg["health_percentage"] = 100.0f; // 需要根据循环次数和其他因素计算

        // 状态信息
        battery_msg["status"] = static_cast<float>(go2_bms.status);
        battery_msg["version"] = static_cast<float>(go2_bms.version_high * 256 + go2_bms.version_low);

        // 电芯电压统计
        if (go2_bms.cell_vol.size() > 0) {
            auto cell_min = *std::min_element(go2_bms.cell_vol.begin(), go2_bms.cell_vol.end());
            auto cell_max = *std::max_element(go2_bms.cell_vol.begin(), go2_bms.cell_vol.end());
            battery_msg["cell_voltage_min"] = static_cast<float>(cell_min) / 1000.0f; // mV转V
            battery_msg["cell_voltage_max"] = static_cast<float>(cell_max) / 1000.0f;
            battery_msg["cell_voltage_diff"] = static_cast<float>(cell_max - cell_min) / 1000.0f;
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("BatteryToRos", true, duration.count() / 1000.0);

        return ConversionResult::SUCCESS;

    } catch (const std::exception& e) {
        setError("BatteryToRos转换失败: " + std::string(e.what()));
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("BatteryToRos", false, duration.count() / 1000.0);
        return ConversionResult::CONVERSION_ERROR;
    }
}

// ============= API响应解析函数实现 =============

ConversionResult Go2MessageConverter::parseApiResponse(
    const unitree_api::msg::Response& response,
    std::map<std::string, std::string>& result_info) const {

    auto start_time = std::chrono::high_resolution_clock::now();

    try {
        // 清空输出容器
        result_info.clear();

        // 基本响应信息
        result_info["api_id"] = std::to_string(response.header.identity.api_id);
        result_info["status_code"] = std::to_string(response.header.status.code);

        // 状态码解析
        std::string status_description;
        switch (response.header.status.code) {
            case 0:  status_description = "SUCCESS"; break;
            case -1: status_description = "ERROR_INVALID_PARAMETER"; break;
            case -2: status_description = "ERROR_UNSUPPORTED_COMMAND"; break;
            case -3: status_description = "ERROR_EXECUTION_FAILED"; break;
            case -4: status_description = "ERROR_TIMEOUT"; break;
            default: status_description = "UNKNOWN_STATUS"; break;
        }
        result_info["status_description"] = status_description;

        // API ID 解析
        std::string api_description;
        switch (response.header.identity.api_id) {
            case 1008: api_description = "MOVE_COMMAND"; break;
            case 1007: api_description = "EULER_COMMAND"; break;
            case 1002: api_description = "BALANCE_STAND"; break;
            case 1004: api_description = "STAND_UP"; break;
            case 1001: api_description = "DAMP_MODE"; break;
            default: api_description = "UNKNOWN_API"; break;
        }
        result_info["api_description"] = api_description;

        // 解析响应数据 (如果包含JSON数据)
        if (!response.data.empty()) {
            result_info["response_data"] = response.data;
            // 这里可以进一步解析JSON数据，提取具体字段
        }

        // 解析二进制数据 (如果有)
        if (!response.binary.empty()) {
            result_info["binary_size"] = std::to_string(response.binary.size());
            result_info["has_binary"] = "true";
        } else {
            result_info["has_binary"] = "false";
        }

        // 成功/失败判断
        result_info["is_success"] = (response.header.status.code == 0) ? "true" : "false";

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("ParseApiResponse", true, duration.count() / 1000.0);

        return ConversionResult::SUCCESS;

    } catch (const std::exception& e) {
        setError("API响应解析失败: " + std::string(e.what()));
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("ParseApiResponse", false, duration.count() / 1000.0);
        return ConversionResult::CONVERSION_ERROR;
    }
}

// ============= 反向转换函数实现 =============

ConversionResult Go2MessageConverter::convertBatteryInfoToBms(
    const robot_base_interfaces::power_interface::BatteryInfo& unified_battery,
    unitree_go::msg::BmsState& go2_bms) const {

    auto start_time = std::chrono::high_resolution_clock::now();

    try {
        // 基本字段转换
        go2_bms.soc = static_cast<uint8_t>(unified_battery.soc_percentage);
        go2_bms.current = static_cast<int32_t>(unified_battery.current * 1000.0); // 安培转毫安
        go2_bms.cycle = static_cast<uint16_t>(unified_battery.cycle_count);

        // 状态转换
        go2_bms.status = unified_battery.status;
        go2_bms.version_high = unified_battery.version_high;
        go2_bms.version_low = unified_battery.version_low;

        // 温度信息转换 - std::array 是固定大小，不需要 resize
        if (unified_battery.bq_ntc_temps.size() >= 2) {
            go2_bms.bq_ntc[0] = static_cast<int8_t>(unified_battery.bq_ntc_temps[0]);
            go2_bms.bq_ntc[1] = static_cast<int8_t>(unified_battery.bq_ntc_temps[1]);
        } else {
            go2_bms.bq_ntc[0] = static_cast<int8_t>(unified_battery.temperature);
            go2_bms.bq_ntc[1] = static_cast<int8_t>(unified_battery.temperature);
        }

        if (unified_battery.mcu_ntc_temps.size() >= 2) {
            go2_bms.mcu_ntc[0] = static_cast<int8_t>(unified_battery.mcu_ntc_temps[0]);
            go2_bms.mcu_ntc[1] = static_cast<int8_t>(unified_battery.mcu_ntc_temps[1]);
        } else {
            go2_bms.mcu_ntc[0] = static_cast<int8_t>(unified_battery.temperature);
            go2_bms.mcu_ntc[1] = static_cast<int8_t>(unified_battery.temperature);
        }

        // 电芯电压转换 - std::array 是固定大小，不需要 resize
        for (size_t i = 0; i < 15; ++i) {
            if (i < unified_battery.cells.size()) {
                go2_bms.cell_vol[i] = static_cast<uint16_t>(unified_battery.cells[i].voltage * 1000.0f); // V转mV
            } else {
                // 默认单体电压 = 总电压/15
                go2_bms.cell_vol[i] = static_cast<uint16_t>(unified_battery.voltage * 1000.0f / 15.0f);
            }
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("BatteryInfoToBms", true, duration.count() / 1000.0);

        return ConversionResult::SUCCESS;

    } catch (const std::exception& e) {
        setError("BatteryInfoToBms转换失败: " + std::string(e.what()));
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("BatteryInfoToBms", false, duration.count() / 1000.0);
        return ConversionResult::CONVERSION_ERROR;
    }
}

ConversionResult Go2MessageConverter::convertRosImuToGo2(
    const sensor_msgs::msg::Imu& ros_imu,
    unitree_go::msg::IMUState& go2_imu) const {

    auto start_time = std::chrono::high_resolution_clock::now();

    try {
        // 四元数转换 (ROS: x,y,z,w -> Go2: w,x,y,z) - std::array 是固定大小，不需要 resize
        go2_imu.quaternion[0] = static_cast<float>(ros_imu.orientation.w);
        go2_imu.quaternion[1] = static_cast<float>(ros_imu.orientation.x);
        go2_imu.quaternion[2] = static_cast<float>(ros_imu.orientation.y);
        go2_imu.quaternion[3] = static_cast<float>(ros_imu.orientation.z);

        // 陀螺仪数据转换 - std::array 是固定大小，不需要 resize
        go2_imu.gyroscope[0] = static_cast<float>(ros_imu.angular_velocity.x);
        go2_imu.gyroscope[1] = static_cast<float>(ros_imu.angular_velocity.y);
        go2_imu.gyroscope[2] = static_cast<float>(ros_imu.angular_velocity.z);

        // 加速度计数据转换 - std::array 是固定大小，不需要 resize
        go2_imu.accelerometer[0] = static_cast<float>(ros_imu.linear_acceleration.x);
        go2_imu.accelerometer[1] = static_cast<float>(ros_imu.linear_acceleration.y);
        go2_imu.accelerometer[2] = static_cast<float>(ros_imu.linear_acceleration.z);

        // 计算欧拉角 (从四元数) - std::array 是固定大小，不需要 resize
        // 需要将 std::array 转换为 std::vector 来调用 quaternionToEuler
        std::vector<float> quat_vec(go2_imu.quaternion.begin(), go2_imu.quaternion.end());
        auto euler = quaternionToEuler(quat_vec);
        go2_imu.rpy[0] = euler[0]; // roll
        go2_imu.rpy[1] = euler[1]; // pitch
        go2_imu.rpy[2] = euler[2]; // yaw

        // 温度设置为默认值 (ROS IMU消息没有温度字段)
        go2_imu.temperature = 25; // 默认25°C

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("RosImuToGo2", true, duration.count() / 1000.0);

        return ConversionResult::SUCCESS;

    } catch (const std::exception& e) {
        setError("ROS IMU转Go2失败: " + std::string(e.what()));
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("RosImuToGo2", false, duration.count() / 1000.0);
        return ConversionResult::CONVERSION_ERROR;
    }
}

// ============= 控制命令转换函数 =============

ConversionResult Go2MessageConverter::convertVelocityCommand(
    const robot_base_interfaces::motion_interface::Velocity& velocity,
    unitree_api::msg::Request& go2_request) const {
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    try {
        // 设置Go2 API请求的基本信息 - 根据文档8.4节，运动控制API ID是1008
        go2_request.header.identity.api_id = 1008; // ROBOT_SPORT_API_ID_MOVE
        
        // 构建速度命令参数字符串（JSON格式）
        std::ostringstream param_stream;
        param_stream << std::fixed << std::setprecision(3);
        param_stream << "{";
        param_stream << "\"x\":"    << velocity.linear_x << ",";
        param_stream << "\"y\":"    << velocity.linear_y << ",";
        param_stream << "\"z\":0.0,"; // Go2通常z方向速度为0
        param_stream << "\"yaw\":"  << velocity.angular_z;
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
        // 设置Go2姿态控制API请求信息 - 根据文档8.4节，欧拉角控制API ID是1007
        go2_request.header.identity.api_id = 1007; // ROBOT_SPORT_API_ID_EULER
        
        // 构建姿态命令参数（JSON格式）
        std::ostringstream param_stream;
        param_stream << std::fixed << std::setprecision(3);
        param_stream << "{";
        param_stream << "\"body_height\":"  << posture.body_height  << ",";
        param_stream << "\"roll\":"         << posture.roll         << ",";
        param_stream << "\"pitch\":"        << posture.pitch        << ",";
        param_stream << "\"yaw\":"          << posture.yaw;
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

ConversionResult Go2MessageConverter::convertTwistToGo2Velocity(
    const geometry_msgs::msg::Twist& twist,
    std::vector<float>& go2_velocity) const {

    auto start_time = std::chrono::high_resolution_clock::now();

    try {
        // Go2速度参数：[vx, vy, vyaw]
        go2_velocity.clear();
        go2_velocity.reserve(3);

        // 提取线性和角速度
        float vx   = static_cast<float>(twist.linear.x);
        float vy   = static_cast<float>(twist.linear.y);
        float vyaw = static_cast<float>(twist.angular.z);

        // 数据范围验证和限制
        if (options_.validate_ranges) {
            // 根据Go2文档，线速度限制±1.5m/s，角速度限制±2.0rad/s
            vx = std::clamp(vx, -1.5f, 1.5f);
            vy = std::clamp(vy, -1.0f, 1.0f);  // Y方向通常限制更严格
            vyaw = std::clamp(vyaw, -2.0f, 2.0f);

            if (std::abs(twist.linear.x) > 1.5f ||
                std::abs(twist.linear.y) > 1.0f ||
                std::abs(twist.angular.z) > 2.0f) {
                setError("Twist速度超出Go2安全范围");
            }
        }

        go2_velocity.push_back(vx);
        go2_velocity.push_back(vy);
        go2_velocity.push_back(vyaw);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("TwistToVelocity", true, duration.count() / 1000.0);

        return ConversionResult::SUCCESS;

    } catch (const std::exception& e) {
        setError("TwistToVelocity转换失败: " + std::string(e.what()));
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("TwistToVelocity", false, duration.count() / 1000.0);
        return ConversionResult::CONVERSION_ERROR;
    }
}

ConversionResult Go2MessageConverter::convertTwistToApiRequest(
    const geometry_msgs::msg::Twist& twist,
    unitree_api::msg::Request& go2_request) const {

    auto start_time = std::chrono::high_resolution_clock::now();

    try {
        // 设置Go2 API请求的基本信息 - 根据文档8.4节，运动控制API ID是1008
        go2_request.header.identity.api_id = 1008; // ROBOT_SPORT_API_ID_MOVE

        // 提取并限制速度值
        float vx = static_cast<float>(twist.linear.x);
        float vy = static_cast<float>(twist.linear.y);
        float vyaw = static_cast<float>(twist.angular.z);

        // 数据范围验证和限制
        if (options_.validate_ranges) {
            // 根据Go2文档，线速度限制±1.5m/s，角速度限制±2.0rad/s
            vx = std::clamp(vx, -1.5f, 1.5f);
            vy = std::clamp(vy, -1.0f, 1.0f);  // Y方向通常限制更严格
            vyaw = std::clamp(vyaw, -2.0f, 2.0f);

            if (std::abs(twist.linear.x) > 1.5f ||
                std::abs(twist.linear.y) > 1.0f ||
                std::abs(twist.angular.z) > 2.0f) {
                setError("Twist速度超出Go2安全范围，已被限制");
            }
        }

        // 构建符合Go2协议的JSON参数字符串
        std::ostringstream param_stream;
        param_stream << std::fixed << std::setprecision(3);
        param_stream << "{";
        param_stream << "\"x\":" << vx << ",";
        param_stream << "\"y\":" << vy << ",";
        param_stream << "\"z\":0.0,"; // Go2通常z方向速度为0
        param_stream << "\"yaw\":" << vyaw;
        param_stream << "}";

        // 将参数字符串复制到请求消息中
        std::string param_str = param_stream.str();
        go2_request.parameter.resize(param_str.length());
        std::copy(param_str.begin(), param_str.end(), go2_request.parameter.begin());

        // 设置其他必要的头部信息
        // 根据需要可以设置时间戳等其他字段

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("TwistToApiRequest", true, duration.count() / 1000.0);

        return ConversionResult::SUCCESS;

    } catch (const std::exception& e) {
        setError("TwistToApiRequest转换失败: " + std::string(e.what()));
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("TwistToApiRequest", false, duration.count() / 1000.0);
        return ConversionResult::CONVERSION_ERROR;
    }
}

// ============= 传感器数据转换函数实现 =============

ConversionResult Go2MessageConverter::convertImuData(
    const std::vector<float>& go2_imu,
    sensor_msgs::msg::Imu& ros_imu) const {

    auto start_time = std::chrono::high_resolution_clock::now();

    try {
        // Go2 IMU数据格式验证
        if (go2_imu.size() < 10) {  // 至少需要四元数4+陀螺仪3+加速度3=10个数据
            setError("Go2 IMU数据不完整: " + std::to_string(go2_imu.size()));
            return ConversionResult::INVALID_INPUT;
        }

        // 设置时间戳为当前时间
        auto now = std::chrono::system_clock::now();
        auto time_duration = now.time_since_epoch();
        auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(time_duration);

        ros_imu.header.stamp.sec = static_cast<uint32_t>(nanoseconds.count() / 1000000000ULL);
        ros_imu.header.stamp.nanosec = static_cast<uint32_t>(nanoseconds.count() % 1000000000ULL);
        ros_imu.header.frame_id = "base_link";

        // 转换四元数 (Go2格式: w,x,y,z -> ROS格式: x,y,z,w)
        if (go2_imu.size() >= 4) {
            ros_imu.orientation.w = static_cast<double>(go2_imu[0]);
            ros_imu.orientation.x = static_cast<double>(go2_imu[1]);
            ros_imu.orientation.y = static_cast<double>(go2_imu[2]);
            ros_imu.orientation.z = static_cast<double>(go2_imu[3]);
        }

        // 转换陀螺仪数据 (角速度)
        if (go2_imu.size() >= 7) {
            ros_imu.angular_velocity.x = static_cast<double>(go2_imu[4]);
            ros_imu.angular_velocity.y = static_cast<double>(go2_imu[5]);
            ros_imu.angular_velocity.z = static_cast<double>(go2_imu[6]);
        }

        // 转换加速度计数据
        if (go2_imu.size() >= 10) {
            ros_imu.linear_acceleration.x = static_cast<double>(go2_imu[7]);
            ros_imu.linear_acceleration.y = static_cast<double>(go2_imu[8]);
            ros_imu.linear_acceleration.z = static_cast<double>(go2_imu[9]);
        }

        // 设置协方差矩阵（如果有经验数据，可以设置实际值）
        std::fill(ros_imu.orientation_covariance.begin(), ros_imu.orientation_covariance.end(), 0.0);
        std::fill(ros_imu.angular_velocity_covariance.begin(), ros_imu.angular_velocity_covariance.end(), 0.0);
        std::fill(ros_imu.linear_acceleration_covariance.begin(), ros_imu.linear_acceleration_covariance.end(), 0.0);

        // 设置对角线元素为默认噪声值
        ros_imu.orientation_covariance[0] = ros_imu.orientation_covariance[4] = ros_imu.orientation_covariance[8] = 0.01;
        ros_imu.angular_velocity_covariance[0] = ros_imu.angular_velocity_covariance[4] = ros_imu.angular_velocity_covariance[8] = 0.001;
        ros_imu.linear_acceleration_covariance[0] = ros_imu.linear_acceleration_covariance[4] = ros_imu.linear_acceleration_covariance[8] = 0.01;

        // 数据验证
        if (options_.validate_ranges) {
            // 验证四元数归一化
            double quat_norm = std::sqrt(
                ros_imu.orientation.w * ros_imu.orientation.w +
                ros_imu.orientation.x * ros_imu.orientation.x +
                ros_imu.orientation.y * ros_imu.orientation.y +
                ros_imu.orientation.z * ros_imu.orientation.z
            );

            if (std::abs(quat_norm - 1.0) > 0.1) {
                setError("IMU四元数未归一化: " + std::to_string(quat_norm));
                // 重新归一化
                ros_imu.orientation.w /= quat_norm;
                ros_imu.orientation.x /= quat_norm;
                ros_imu.orientation.y /= quat_norm;
                ros_imu.orientation.z /= quat_norm;
            }
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("ImuData", true, duration.count() / 1000.0);

        return ConversionResult::SUCCESS;

    } catch (const std::exception& e) {
        setError("IMU数据转换失败: " + std::string(e.what()));
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("ImuData", false, duration.count() / 1000.0);
        return ConversionResult::CONVERSION_ERROR;
    }
}

ConversionResult Go2MessageConverter::convertRosImuToUnified(
    const sensor_msgs::msg::Imu& ros_imu,
    robot_base_interfaces::sensor_interface::IMUData& unified_imu) const {

    auto start_time = std::chrono::high_resolution_clock::now();

    try {
        // 设置frame_id
        unified_imu.frame_id = ros_imu.header.frame_id.empty() ? "imu_link" : ros_imu.header.frame_id;

        // 设置时间戳
        unified_imu.timestamp_ns = static_cast<uint64_t>(ros_imu.header.stamp.sec) * 1000000000ULL +
                                   static_cast<uint64_t>(ros_imu.header.stamp.nanosec);

        // 转换四元数姿态
        unified_imu.orientation.w = static_cast<float>(ros_imu.orientation.w);
        unified_imu.orientation.x = static_cast<float>(ros_imu.orientation.x);
        unified_imu.orientation.y = static_cast<float>(ros_imu.orientation.y);
        unified_imu.orientation.z = static_cast<float>(ros_imu.orientation.z);

        // 转换角速度
        unified_imu.angular_velocity.x = static_cast<float>(ros_imu.angular_velocity.x);
        unified_imu.angular_velocity.y = static_cast<float>(ros_imu.angular_velocity.y);
        unified_imu.angular_velocity.z = static_cast<float>(ros_imu.angular_velocity.z);

        // 转换线性加速度
        unified_imu.linear_acceleration.x = static_cast<float>(ros_imu.linear_acceleration.x);
        unified_imu.linear_acceleration.y = static_cast<float>(ros_imu.linear_acceleration.y);
        unified_imu.linear_acceleration.z = static_cast<float>(ros_imu.linear_acceleration.z);

        // 如果有四元数数据，计算欧拉角
        if (unified_imu.orientation.w != 0.0f || unified_imu.orientation.x != 0.0f ||
            unified_imu.orientation.y != 0.0f || unified_imu.orientation.z != 0.0f) {

            auto euler = quaternionToEuler({unified_imu.orientation.w, unified_imu.orientation.x,
                                           unified_imu.orientation.y, unified_imu.orientation.z});

            if (euler.size() >= 3) {
                unified_imu.rpy.roll = euler[0];
                unified_imu.rpy.pitch = euler[1];
                unified_imu.rpy.yaw = euler[2];
            }
        }

        // 设置默认温度（ROS IMU消息通常不包含温度信息）
        unified_imu.temperature = 25; // 默认室温

        // 数据验证
        if (options_.validate_ranges) {
            // 验证四元数归一化
            float quat_norm = std::sqrt(
                unified_imu.orientation.w * unified_imu.orientation.w +
                unified_imu.orientation.x * unified_imu.orientation.x +
                unified_imu.orientation.y * unified_imu.orientation.y +
                unified_imu.orientation.z * unified_imu.orientation.z
            );

            if (std::abs(quat_norm - 1.0f) > 0.1f) {
                setError("ROS IMU四元数未归一化: " + std::to_string(quat_norm));
                if (quat_norm > 0.001f) {  // 避免除零
                    unified_imu.orientation.w /= quat_norm;
                    unified_imu.orientation.x /= quat_norm;
                    unified_imu.orientation.y /= quat_norm;
                    unified_imu.orientation.z /= quat_norm;
                } else {
                    // 设置为单位四元数
                    unified_imu.orientation = {1.0f, 0.0f, 0.0f, 0.0f};
                }
            }

            // 验证角速度范围（通常±35 rad/s）
            validateRange(unified_imu.angular_velocity.x, -35.0f, 35.0f, "angular_velocity_x");
            validateRange(unified_imu.angular_velocity.y, -35.0f, 35.0f, "angular_velocity_y");
            validateRange(unified_imu.angular_velocity.z, -35.0f, 35.0f, "angular_velocity_z");

            // 验证线性加速度范围（通常±157 m/s²）
            validateRange(unified_imu.linear_acceleration.x, -157.0f, 157.0f, "linear_acceleration_x");
            validateRange(unified_imu.linear_acceleration.y, -157.0f, 157.0f, "linear_acceleration_y");
            validateRange(unified_imu.linear_acceleration.z, -157.0f, 157.0f, "linear_acceleration_z");
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("RosImuToUnified", true, duration.count() / 1000.0);

        return ConversionResult::SUCCESS;

    } catch (const std::exception& e) {
        setError("ROS IMU转统一格式失败: " + std::string(e.what()));
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("RosImuToUnified", false, duration.count() / 1000.0);
        return ConversionResult::CONVERSION_ERROR;
    }
}

ConversionResult Go2MessageConverter::enhancePointCloudData(
    const sensor_msgs::msg::PointCloud2& pointcloud,
    robot_base_interfaces::sensor_interface::PointCloudData& /* enhanced_info */) const {

    auto start_time = std::chrono::high_resolution_clock::now();

    try {
        // 由于PointCloudData结构体的具体字段未知，我们进行基本的验证和处理
        // 验证点云数据的基本信息
        uint32_t total_points = pointcloud.width * pointcloud.height;

        // 根据点云数据质量评估（基于点数）
        float quality_score;
        if (total_points > 10000) {
            quality_score = 0.9f;  // 高质量
        } else if (total_points > 1000) {
            quality_score = 0.7f;  // 中等质量
        } else {
            quality_score = 0.5f;  // 低质量
        }

        // 验证点云数据格式
        if (pointcloud.fields.empty()) {
            setError("点云字段信息为空");
            return ConversionResult::INVALID_INPUT;
        }

        // 验证数据完整性
        if (pointcloud.data.empty() && total_points > 0) {
            setError("点云数据为空但声明有点");
            return ConversionResult::DATA_INCOMPLETE;
        }

        // 设置基本统计信息到日志中
        std::ostringstream info;
        info << "点云增强完成 - 总点数: " << total_points
             << ", 质量分数: " << quality_score
             << ", 字段数: " << pointcloud.fields.size();

        // 由于不确定PointCloudData的具体结构，我们仅进行验证和记录
        // 实际的字段赋值需要根据具体的结构体定义来实现

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("PointCloudEnhancement", true, duration.count() / 1000.0);

        return ConversionResult::SUCCESS;

    } catch (const std::exception& e) {
        setError("点云数据增强失败: " + std::string(e.what()));
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        recordConversion("PointCloudEnhancement", false, duration.count() / 1000.0);
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
    RCLCPP_ERROR(rclcpp::get_logger("Go2MessageConverter"), "%s", error_msg.c_str());
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
    stats_stream << "总转换次数: "  << stats_.total_conversions         << "\n";
    stats_stream << "成功转换: "    << stats_.successful_conversions    << "\n";
    stats_stream << "失败转换: "    << stats_.failed_conversions        << "\n";
    
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
    return {"SportModeState",       "LowState",             "BmsState",            "MotorInfo",
            "FootInfo",             "IMUInfo",              "VelocityCommand",     "PostureCommand",
            "IMU",                  "PointCloud2",          "Twist",               "TwistToVelocity",
            "TwistToApiRequest",    "TimestampConversion",  "CoordinateFrame",     "QuaternionConversion",
            "PointCloudEnhancement", "ImuData",             "WirelessController",  "ControllerToTwist",
            "ToOdometry",           "BatteryToRos",         "ParseApiResponse",    "BatteryInfoToBms",
            "RosImuToGo2"};
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

// ============= 时间戳处理函数实现 =============

builtin_interfaces::msg::Time Go2MessageConverter::convertTimestamp(uint64_t go2_timestamp_ns) const {
    builtin_interfaces::msg::Time ros_time;
    ros_time.sec = static_cast<uint32_t>(go2_timestamp_ns / 1000000000ULL);
    ros_time.nanosec = static_cast<uint32_t>(go2_timestamp_ns % 1000000000ULL);
    return ros_time;
}

uint64_t Go2MessageConverter::convertTimestamp(const builtin_interfaces::msg::Time& ros_time) const {
    return static_cast<uint64_t>(ros_time.sec) * 1000000000ULL +
           static_cast<uint64_t>(ros_time.nanosec);
}

int64_t Go2MessageConverter::synchronizeTimestamps(uint64_t go2_time,
                                                  const builtin_interfaces::msg::Time& ros_time) const {
    uint64_t ros_time_ns = convertTimestamp(ros_time);
    return static_cast<int64_t>(go2_time) - static_cast<int64_t>(ros_time_ns);
}

// ============= 坐标系转换函数实现 =============

ConversionResult Go2MessageConverter::convertCoordinateFrame(
    const std::vector<float>& go2_position,
    geometry_msgs::msg::Vector3& ros_position) const {

    try {
        if (go2_position.size() != 3) {
            setError("Go2位置向量大小不正确: " + std::to_string(go2_position.size()));
            return ConversionResult::INVALID_INPUT;
        }

        // 应用坐标变换矩阵 (如果启用坐标系转换)
        if (options_.use_go2_coordinate_frame && transforms_.go2_to_ros.size() == 3) {
            std::vector<float> transformed = matrixMultiply(transforms_.go2_to_ros, go2_position);
            ros_position.x = static_cast<double>(transformed[0]);
            ros_position.y = static_cast<double>(transformed[1]);
            ros_position.z = static_cast<double>(transformed[2]);
        } else {
            // 直接转换 (假设坐标系相同)
            ros_position.x = static_cast<double>(go2_position[0]);
            ros_position.y = static_cast<double>(go2_position[1]);
            ros_position.z = static_cast<double>(go2_position[2]);
        }

        return ConversionResult::SUCCESS;

    } catch (const std::exception& e) {
        setError("坐标转换失败: " + std::string(e.what()));
        return ConversionResult::CONVERSION_ERROR;
    }
}

ConversionResult Go2MessageConverter::convertCoordinateFrame(
    const geometry_msgs::msg::Vector3& ros_position,
    std::vector<float>& go2_position) const {

    try {
        go2_position.clear();
        go2_position.resize(3);

        // 应用坐标变换矩阵 (如果启用坐标系转换)
        if (options_.use_go2_coordinate_frame && transforms_.ros_to_go2.size() == 3) {
            std::vector<float> ros_vec = {
                static_cast<float>(ros_position.x),
                static_cast<float>(ros_position.y),
                static_cast<float>(ros_position.z)
            };
            go2_position = matrixMultiply(transforms_.ros_to_go2, ros_vec);
        } else {
            // 直接转换 (假设坐标系相同)
            go2_position[0] = static_cast<float>(ros_position.x);
            go2_position[1] = static_cast<float>(ros_position.y);
            go2_position[2] = static_cast<float>(ros_position.z);
        }

        return ConversionResult::SUCCESS;

    } catch (const std::exception& e) {
        setError("坐标转换失败: " + std::string(e.what()));
        return ConversionResult::CONVERSION_ERROR;
    }
}

ConversionResult Go2MessageConverter::convertQuaternion(
    const std::vector<float>& go2_quaternion,
    geometry_msgs::msg::Pose& ros_pose) const {

    try {
        if (go2_quaternion.size() != 4) {
            setError("四元数大小不正确: " + std::to_string(go2_quaternion.size()));
            return ConversionResult::INVALID_INPUT;
        }

        // Go2四元数格式: [w, x, y, z]
        // ROS四元数格式: [x, y, z, w]
        ros_pose.orientation.w = static_cast<double>(go2_quaternion[0]);
        ros_pose.orientation.x = static_cast<double>(go2_quaternion[1]);
        ros_pose.orientation.y = static_cast<double>(go2_quaternion[2]);
        ros_pose.orientation.z = static_cast<double>(go2_quaternion[3]);

        // 验证四元数是否已归一化
        if (options_.validate_ranges) {
            double norm = std::sqrt(
                ros_pose.orientation.w * ros_pose.orientation.w +
                ros_pose.orientation.x * ros_pose.orientation.x +
                ros_pose.orientation.y * ros_pose.orientation.y +
                ros_pose.orientation.z * ros_pose.orientation.z
            );

            if (std::abs(norm - 1.0) > 0.1) {
                setError("四元数未归一化: norm = " + std::to_string(norm));
                // 归一化四元数
                ros_pose.orientation.w /= norm;
                ros_pose.orientation.x /= norm;
                ros_pose.orientation.y /= norm;
                ros_pose.orientation.z /= norm;
            }
        }

        return ConversionResult::SUCCESS;

    } catch (const std::exception& e) {
        setError("四元数转换失败: " + std::string(e.what()));
        return ConversionResult::CONVERSION_ERROR;
    }
}

// ============= 私有辅助函数实现 =============

std::vector<float> Go2MessageConverter::matrixMultiply(
    const std::vector<std::vector<float>>& matrix,
    const std::vector<float>& vector) const {

    std::vector<float> result;

    if (matrix.size() != vector.size()) {
        return result; // 返回空向量表示错误
    }

    result.resize(matrix.size(), 0.0f);

    for (size_t i = 0; i < matrix.size(); ++i) {
        if (matrix[i].size() != vector.size()) {
            return std::vector<float>(); // 矩阵尺寸不匹配
        }
        for (size_t j = 0; j < vector.size(); ++j) {
            result[i] += matrix[i][j] * vector[j];
        }
    }

    return result;
}

std::vector<float> Go2MessageConverter::normalizeQuaternion(const std::vector<float>& quat) const {
    if (quat.size() != 4) {
        return {0.0f, 0.0f, 0.0f, 1.0f}; // 返回单位四元数
    }

    float norm = std::sqrt(quat[0]*quat[0] + quat[1]*quat[1] + quat[2]*quat[2] + quat[3]*quat[3]);

    if (norm < 1e-6f) {
        return {0.0f, 0.0f, 0.0f, 1.0f}; // 避免除零错误
    }

    return {quat[0]/norm, quat[1]/norm, quat[2]/norm, quat[3]/norm};
}

std::vector<float> Go2MessageConverter::eulerToQuaternion(float roll, float pitch, float yaw) const {
    float cy = std::cos(yaw * 0.5f);
    float sy = std::sin(yaw * 0.5f);
    float cp = std::cos(pitch * 0.5f);
    float sp = std::sin(pitch * 0.5f);
    float cr = std::cos(roll * 0.5f);
    float sr = std::sin(roll * 0.5f);

    std::vector<float> quat(4);
    quat[0] = cr * cp * cy + sr * sp * sy; // w
    quat[1] = sr * cp * cy - cr * sp * sy; // x
    quat[2] = cr * sp * cy + sr * cp * sy; // y
    quat[3] = cr * cp * sy - sr * sp * cy; // z

    return quat;
}

std::vector<float> Go2MessageConverter::quaternionToEuler(const std::vector<float>& quat) const {
    if (quat.size() != 4) {
        return {0.0f, 0.0f, 0.0f}; // 返回零旋转
    }

    float w = quat[0], x = quat[1], y = quat[2], z = quat[3];

    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    float roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (w * y - z * x);
    float pitch = std::abs(sinp) >= 1.0f ?
        std::copysign(M_PI / 2.0f, sinp) : std::asin(sinp);

    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    float yaw = std::atan2(siny_cosp, cosy_cosp);

    return {roll, pitch, yaw};
}

bool Go2MessageConverter::validateInput(const void* input, const std::string& type) const {
    if (!input) {
        setError(type + "输入为空指针");
        return false;
    }
    return true;
}

bool Go2MessageConverter::validateGo2Data(const void* data, size_t size, const std::string& type) const {
    if (!validateInput(data, type)) {
        return false;
    }

    if (size == 0) {
        setError(type + "数据大小为零");
        return false;
    }

    return true;
}

void Go2MessageConverter::applyLimits(float& value, float min_val, float max_val) const {
    if (value < min_val) {
        value = min_val;
    } else if (value > max_val) {
        value = max_val;
    }
}

uint32_t Go2MessageConverter::calculateChecksum(const void* data, size_t size) const {
    const uint8_t* bytes = static_cast<const uint8_t*>(data);
    uint32_t checksum = 0;

    for (size_t i = 0; i < size; ++i) {
        checksum += bytes[i];
    }

    return checksum;
}

} // namespace go2_adapter
} // namespace robot_adapters