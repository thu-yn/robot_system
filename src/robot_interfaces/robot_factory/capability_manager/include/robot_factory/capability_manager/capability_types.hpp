/**
 * @file capability_types.hpp
 * @brief 能力管理器通用类型定义
 * @author Claude Code
 * @date 2024
 */

#ifndef ROBOT_FACTORY__CAPABILITY_MANAGER__CAPABILITY_TYPES_HPP_
#define ROBOT_FACTORY__CAPABILITY_MANAGER__CAPABILITY_TYPES_HPP_

#include "robot_base_interfaces/motion_interface/motion_types.hpp"

namespace robot_factory {
namespace capability_manager {

using RobotType = robot_base_interfaces::motion_interface::RobotType;

/**
 * @brief 能力类型枚举
 */
enum class CapabilityType {
    MOTION = 0,       ///< 运动能力
    SENSOR = 1,       ///< 传感器能力
    POWER = 2,        ///< 电源能力
    COMMUNICATION = 3, ///< 通信能力
    NAVIGATION = 4,   ///< 导航能力
    MANIPULATION = 5, ///< 操作能力 (预留)
    QUADRUPED_TRICKS = 6, ///< 四足特技能力
    CUSTOM = 100      ///< 自定义能力
};

/**
 * @brief 能力级别枚举
 */
enum class CapabilityLevel {
    NONE = 0,        ///< 不支持
    BASIC = 1,       ///< 基础支持
    INTERMEDIATE = 2, ///< 中等支持
    ADVANCED = 3,    ///< 高级支持
    EXPERT = 4       ///< 专家级支持
};

} // namespace capability_manager
} // namespace robot_factory

#endif // ROBOT_FACTORY__CAPABILITY_MANAGER__CAPABILITY_TYPES_HPP_