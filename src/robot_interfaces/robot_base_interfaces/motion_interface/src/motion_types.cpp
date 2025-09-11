/**
 * @file   motion_types.cpp
 * @brief  机器人运动控制相关数据类型的实现文件
 * 
 *         该文件实现了motion_types.hpp中定义的运动控制相关数据类型和工具函数。
 *         包括速度控制、姿态控制、步态参数、运动能力等数据结构的操作方法。
 * 
 *         主要功能：
 *         - 运动数据类型的构造和初始化
 *         - 数据验证和范围检查
 *         - 坐标系转换和单位换算
 *         - 运动参数的插值和平滑处理
 *         - 调试信息输出和字符串转换
 * 
 * @author Yang Nan
 * @date   2025-09-11
 */

#include "robot_base_interfaces/motion_interface/motion_types.hpp"
#include <algorithm>
#include <cmath>
#include <string>

namespace robot_base_interfaces {
namespace motion_interface {

// ========== 辅助函数 ==========

/**
 * @brief 将运动模式转换为字符串
 */
std::string motionModeToString(MotionMode mode) {
    switch (mode) {
        case MotionMode::IDLE: return "IDLE";
        case MotionMode::BALANCE_STAND: return "BALANCE_STAND";
        case MotionMode::POSE: return "POSE";
        case MotionMode::LOCOMOTION: return "LOCOMOTION";
        case MotionMode::LIE_DOWN: return "LIE_DOWN";
        case MotionMode::JOINT_LOCK: return "JOINT_LOCK";
        case MotionMode::DAMPING: return "DAMPING";
        case MotionMode::RECOVERY_STAND: return "RECOVERY_STAND";
        case MotionMode::SIT: return "SIT";
        case MotionMode::CUSTOM: return "CUSTOM";
        default: return "UNKNOWN";
    }
}

/**
 * @brief 将步态类型转换为字符串
 */
std::string gaitTypeToString(GaitType gait) {
    switch (gait) {
        case GaitType::IDLE: return "IDLE";
        case GaitType::TROT: return "TROT";
        case GaitType::RUN: return "RUN";
        case GaitType::CLIMB_STAIR: return "CLIMB_STAIR";
        case GaitType::DOWN_STAIR: return "DOWN_STAIR";
        case GaitType::ADJUST: return "ADJUST";
        case GaitType::CUSTOM: return "CUSTOM";
        default: return "UNKNOWN";
    }
}

/**
 * @brief 将机器人类型转换为字符串
 */
std::string robotTypeToString(RobotType type) {
    switch (type) {
        case RobotType::GO2: return "GO2";
        case RobotType::SPOT: return "SPOT";
        case RobotType::ANYMAL: return "ANYMAL";
        case RobotType::GENERIC: return "GENERIC";
        default: return "UNKNOWN";
    }
}

/**
 * @brief 限制值在指定范围内
 */
template<typename T>
T clamp(T value, T min_val, T max_val) {
    return std::max(min_val, std::min(value, max_val));
}

/**
 * @brief 检查浮点数是否接近零
 */
bool isNearZero(float value, float epsilon = 1e-6f) {
    return std::abs(value) < epsilon;
}

} // namespace motion_interface
} // namespace robot_base_interfaces