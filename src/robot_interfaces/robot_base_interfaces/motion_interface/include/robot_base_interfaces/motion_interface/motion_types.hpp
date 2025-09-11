/**
 * @file   motion_types.hpp
 * @brief  机器人运动相关数据类型定义 - 完全适配Go2，预留扩展
 *         说明：本文件中的默认取值与注释以 Go2 为示例，不构成对其他机型的约束；
 *         具体能力与限制应由适配器在运行时通过能力描述进行声明。
 * @author Yang Nan
 * @date   2025-09-11
 */

#ifndef ROBOT_BASE_INTERFACES__MOTION_INTERFACE__MOTION_TYPES_HPP_
#define ROBOT_BASE_INTERFACES__MOTION_INTERFACE__MOTION_TYPES_HPP_

#include <vector>
#include <cstdint>

namespace robot_base_interfaces {
namespace motion_interface {

/**
 * @brief 机器人类型枚举
 */
enum class RobotType {
    GO2     = 0,    ///< 宇树Go2四足机器人
    SPOT    = 1,    ///< Boston Dynamics Spot (预留)  
    ANYMAL  = 2,    ///< ANYbotics ANYmal (预留)
    GENERIC = 99,   ///< 通用机器人
    UNKNOWN = 255   ///< 未知机器人类型
};

/**
 * @brief 运动模式枚举 - 基于Go2定义，其他机器人可映射
 */
enum class MotionMode {
    IDLE           = 0,    ///< 待机模式
    BALANCE_STAND  = 1,    ///< 平衡站立
    POSE           = 2,    ///< 姿态控制
    LOCOMOTION     = 3,    ///< 移动模式
    LIE_DOWN       = 5,    ///< 趴下
    JOINT_LOCK     = 6,    ///< 关节锁定
    DAMPING        = 7,    ///< 阻尼模式
    RECOVERY_STAND = 8,    ///< 恢复站立
    SIT            = 10,   ///< 坐下
    CUSTOM         = 100   ///< 自定义模式 (其他机器人扩展用)
};

/**
 * @brief 步态类型枚举 - 基于Go2定义
 */
enum class GaitType {
    IDLE        = 0,    ///< 待机
    TROT        = 1,    ///< 小跑
    RUN         = 2,    ///< 奔跑
    CLIMB_STAIR = 3,    ///< 爬楼梯
    DOWN_STAIR  = 4,    ///< 下楼梯
    ADJUST      = 9,    ///< 调整
    CUSTOM      = 100   ///< 自定义步态 (扩展用)
};

/**
 * @brief 3D速度向量
 */
struct Velocity {
    float linear_x  = 0.0f;     ///< 前进速度 (m/s)
    float linear_y  = 0.0f;     ///< 侧移速度 (m/s)  
    float linear_z  = 0.0f;     ///< 垂直速度 (m/s, 部分机器人支持)
    float angular_x = 0.0f;     ///< 滚转角速度 (rad/s)
    float angular_y = 0.0f;     ///< 俯仰角速度 (rad/s)
    float angular_z = 0.0f;     ///< 偏航角速度 (rad/s)

    Velocity() = default;
    Velocity(float vx, float vy, float wz) 
        : linear_x(vx), linear_y(vy), angular_z(wz) {}
};

/**
 * @brief 机器人姿态
 */
struct Posture {
    float roll        = 0.0f;     ///< 滚转角 (rad)
    float pitch       = 0.0f;     ///< 俯仰角 (rad)
    float yaw         = 0.0f;     ///< 偏航角 (rad)
    float body_height = 0.40f;    ///< 机身高度 (m)

    Posture() = default;
    Posture(float r, float p, float y, float h = 0.40f)
        : roll(r), pitch(p), yaw(y), body_height(h) {}
};

/**
 * @brief 3D位置
 */
struct Position {
    float x = 0.0f;  ///< X坐标 (m)
    float y = 0.0f;  ///< Y坐标 (m) 
    float z = 0.0f;  ///< Z坐标 (m)

    Position() = default;
    Position(float x_val, float y_val, float z_val = 0.0f)
        : x(x_val), y(y_val), z(z_val) {}
};

/**
 * @brief 运动能力定义 - 完全覆盖Go2能力
 */
struct MotionCapabilities {
    // 速度限制
    float max_linear_velocity  = 1.5f;   ///< 最大线速度 (m/s, Go2: 1.5)
    float max_angular_velocity = 2.0f;   ///< 最大角速度 (rad/s, Go2: 2.0)
    float max_lateral_velocity = 0.8f;   ///< 最大侧移速度 (m/s)
    
    // 姿态限制
    float max_roll_angle  = 0.4f;        ///< 最大滚转角 (rad)
    float max_pitch_angle = 0.4f;        ///< 最大俯仰角 (rad)
    float min_body_height = 0.20f;       ///< 最小机身高度 (m, Go2: 0.20)
    float max_body_height = 0.42f;       ///< 最大机身高度 (m, Go2: 0.42)
    
    // 特殊能力
    bool can_climb_stairs = true;        ///< 是否支持爬楼梯
    bool can_balance      = true;        ///< 是否支持平衡控制
    bool can_lateral_move = true;        ///< 是否支持侧移
    bool can_dance        = true;        ///< 是否支持舞蹈动作 (Go2特有)
    bool can_jump         = true;        ///< 是否支持跳跃 (Go2特有)
    bool can_flip         = true;        ///< 是否支持翻滚 (Go2特有)
    
    // 支持的运动模式
    std::vector<MotionMode> supported_modes = {
        MotionMode::IDLE, 
        MotionMode::BALANCE_STAND, 
        MotionMode::POSE,
        MotionMode::LOCOMOTION, 
        MotionMode::LIE_DOWN, 
        MotionMode::SIT
    };
    
    // 支持的步态类型
    std::vector<GaitType> supported_gaits = {
        GaitType::IDLE, 
        GaitType::TROT, 
        GaitType::RUN, 
        GaitType::CLIMB_STAIR
    };
    
    // TODO: 为其他机器人预留扩展字段
    // std::map<std::string, float> extended_limits;        // 扩展限制参数
    // std::map<std::string, bool>  extended_capabilities;  // 扩展能力标志
};

/**
 * @brief 运动状态 - 基于Go2 SportModeState设计
 */
struct MotionState {
    // 基本状态
    MotionMode current_mode    = MotionMode::IDLE; ///< 当前模式
    GaitType   current_gait    = GaitType::IDLE;   ///< 当前步态
    bool       is_moving       = false;            ///< 是否移动
    bool       is_balanced     = false;            ///< 是否平衡
    float      motion_progress = 0.0f;             ///< 动作执行进度 (0.0-1.0)
    
    // 位置和姿态
    Position position;  ///< 当前位置
    Posture  posture;   ///< 当前姿态
    Velocity velocity;  ///< 当前速度
    
    // Go2特定状态
    float                 foot_raise_height = 0.09f;    ///< 足端抬起高度 (m)
    std::vector<float>    foot_forces;                  ///< 足端力 (N) - 4个足端
    std::vector<Position> foot_positions;               ///< 足端位置 - 4个足端
    std::vector<Velocity> foot_velocities;              ///< 足端速度 - 4个足端
    
    // 障碍物感知 (Go2特有)
    std::vector<float> range_obstacles; ///< 4个方向的障碍物距离 (m)
    
    // 时间戳和错误码
    uint64_t timestamp_ns   = 0;    ///< 时间戳 (nanoseconds)
    uint32_t error_code     = 0;    ///< 错误代码
    
    MotionState() {
        // 为Go2初始化4个足端的数据
        foot_forces.resize(4, 0.0f);
        foot_positions.resize(4);
        foot_velocities.resize(4);
        range_obstacles.resize(4, 0.0f);
    }
    
    // TODO: 其他机器人可能有不同数量的足端或轮子
    // 预留扩展字段用于支持不同的locomotion方式
};

/**
 * @brief 运动命令结果
 */
enum class MotionResult {
    SUCCESS             = 0,    ///< 成功
    INVALID_PARAMETER   = 1,    ///< 参数无效
    CAPABILITY_LIMITED  = 2,    ///< 超出能力限制
    EMERGENCY_STOP      = 3,    ///< 紧急停止
    COMMUNICATION_ERROR = 4,    ///< 通信错误
    UNKNOWN_ERROR       = 99    ///< 未知错误
};

/**
 * @brief 紧急停止级别
 */
enum class EmergencyStopLevel {
    SOFT_STOP = 0,      ///< 软停止 - 缓慢减速停止
    HARD_STOP = 1,      ///< 硬停止 - 立即停止运动
    POWER_OFF = 2       ///< 断电停止 - 关闭动力系统
};

} // namespace motion_interface
} // namespace robot_base_interfaces

#endif // ROBOT_BASE_INTERFACES__MOTION_INTERFACE__MOTION_TYPES_HPP_