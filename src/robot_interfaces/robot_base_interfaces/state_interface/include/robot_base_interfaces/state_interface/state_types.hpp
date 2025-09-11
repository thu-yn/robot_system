/**
 * @file   state_types.hpp
 * @brief  机器人状态相关数据类型定义 - 完全适配Go2，预留扩展
 *         说明：固定数量（如20电机/4足）仅为 Go2 示例默认，其他机型应通过能力
 *         描述或运行期配置声明自身拓扑规模，避免将示例默认视为强约束。
 *         此文件只进行机器人状态的获取，因此重新定义了相关结构体。
 * @author Yang Nan
 * @date   2025-09-11
 */

#ifndef ROBOT_BASE_INTERFACES__STATE_INTERFACE__ROBOT_STATE_TYPES_HPP_
#define ROBOT_BASE_INTERFACES__STATE_INTERFACE__ROBOT_STATE_TYPES_HPP_

#include <vector>
#include <string>
#include <map>
#include <cstdint>

namespace robot_base_interfaces {
namespace state_interface {

/**
 * @brief 机器人整体状态枚举
 */
enum class RobotState {
    UNKNOWN        = 0, ///< 未知状态
    INITIALIZING   = 1, ///< 初始化中
    STANDBY        = 2, ///< 待机状态
    ACTIVE         = 3, ///< 活动状态
    MOVING         = 4, ///< 移动中
    ERROR          = 5, ///< 错误状态
    EMERGENCY_STOP = 6, ///< 紧急停止
    CHARGING       = 7, ///< 充电中
    LOW_POWER      = 8, ///< 低电量
    MAINTENANCE    = 9, ///< 维护模式
    SHUTDOWN       = 10 ///< 关机状态
};

/**
 * @brief 健康状态级别
 */
enum class HealthLevel {
    EXCELLENT = 0,  ///< 优秀 (90-100%)
    GOOD      = 1,  ///< 良好 (70-89%)
    FAIR      = 2,  ///< 一般 (50-69%)
    POOR      = 3,  ///< 较差 (30-49%)
    CRITICAL  = 4,  ///< 严重 (0-29%)
    UNKNOWN   = 5   ///< 未知
};

/**
 * @brief 告警类型枚举
 */
enum class AlertType {
    INFO     = 0,   ///< 信息
    WARNING  = 1,   ///< 警告
    ERROR    = 2,   ///< 错误
    CRITICAL = 3    ///< 严重错误
};

/**
 * @brief 系统模块枚举 - 基于Go2系统架构
 */
enum class SystemModule {
    // 核心系统模块
    MOTION_CONTROL   = 0,   ///< 运动控制系统
    SENSOR_SYSTEM    = 1,   ///< 传感器系统
    POWER_MANAGEMENT = 2,   ///< 电源管理
    COMMUNICATION    = 3,   ///< 通信系统
    NAVIGATION       = 4,   ///< 导航系统
    
    // Go2特有模块
    MOTOR_DRIVERS   = 10,   ///< 电机驱动器
    IMU_SYSTEM      = 11,   ///< IMU系统
    LIDAR_SYSTEM    = 12,   ///< 激光雷达系统
    CHARGING_SYSTEM = 13,   ///< 充电系统
    THERMAL_SYSTEM  = 14,   ///< 热管理系统
    
    // 扩展模块 (其他机器人)
    CAMERA_SYSTEM = 20,     ///< 摄像头系统
    GPS_SYSTEM    = 21,     ///< GPS系统
    CUSTOM_MODULE = 100     ///< 自定义模块
};

/**
 * @brief 电机状态信息 - 基于Go2 MotorState
 */
struct MotorInfo {
    uint8_t  motor_id         = 0;     ///< 电机ID (Go2有20个电机)
    uint8_t  mode             = 0;     ///< 电机模式 (0x00=停止, 0x01=FOC模式)
    float    position         = 0.0f;  ///< 关节位置 (rad)
    float    velocity         = 0.0f;  ///< 关节速度 (rad/s)
    float    acceleration     = 0.0f;  ///< 关节加速度 (rad/s^2)
    float    torque_estimated = 0.0f;  ///< 估计扭矩 (N·m)
    float    position_raw     = 0.0f;  ///< 原始位置数据
    float    velocity_raw     = 0.0f;  ///< 原始速度数据
    float    acceleration_raw = 0.0f;  ///< 原始加速度数据
    int8_t   temperature      = 0;     ///< 电机温度 (°C)
    uint32_t lost_count       = 0;     ///< 通信丢失计数
    bool     is_online        = true;  ///< 是否在线
    uint32_t error_code       = 0;     ///< 错误代码
};

/**
 * @brief 足端信息 - Go2特有 (4足机器人)
 */
struct FootInfo {
    uint8_t foot_id = 0;    ///< 足端ID (0-3: FR,FL,RR,RL)
    
    // 足端位置 (相对于机身坐标系)
    struct {
        float x = 0.0f;
        float y = 0.0f; 
        float z = 0.0f;
    } position;
    
    // 足端速度
    struct {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    } velocity;
    
    float force               = 0.0f;   ///< 足端力 (N)
    float force_estimated     = 0.0f;   ///< 估计足端力 (N)
    bool in_contact           = false;  ///< 是否接触地面
    float contact_probability = 0.0f;   ///< 接触概率 (0.0-1.0)
};

/**
 * @brief 机器人详细状态信息 - 基于Go2设计
 */
struct DetailedRobotState {
    // 基本状态
    std::string robot_id     = "";
    RobotState  state        = RobotState::UNKNOWN;
    HealthLevel health_level = HealthLevel::UNKNOWN;
    float       health_score = 0.0f;                ///< 健康评分 (0.0-1.0)
    uint64_t    timestamp_ns = 0;                   ///< 时间戳
    uint32_t    error_code   = 0;                   ///< 全局错误代码
    
    // 运动状态 (来自SportModeState)
    struct {
        uint8_t mode              = 0;      ///< 运动模式
        uint8_t gait_type         = 0;      ///< 步态类型
        float   progress          = 0.0f;   ///< 动作进度
        float   body_height       = 0.42f;  ///< 机身高度
        float   foot_raise_height = 0.09f;  ///< 足端抬起高度
        
        // 位置和速度
        struct {
            float x = 0.0f;
            float y = 0.0f;
            float z = 0.0f;
        } position;
        
        struct {
            float x = 0.0f;
            float y = 0.0f;
            float z = 0.0f;
        } velocity;
        
        float yaw_speed = 0.0f;     ///< 偏航角速度
    } motion;
    
    // 电机状态 (Go2有20个电机)
    std::vector<MotorInfo> motors;
    
    // 足端状态 (Go2有4个足端)
    std::vector<FootInfo>  feet;
    
    // 传感器状态
    struct {
        bool   imu_online      = false;
        bool   lidar_online    = false;
        bool   camera_online   = false; // 预留
        int8_t imu_temperature = 0;
        float  lidar_frequency = 0.0f;
    } sensors;
    
    // 通信状态
    struct {
        bool     ros2_online           = false;
        bool     sdk_online            = false; // 预留
        uint32_t message_lost_count    = 0;
        float    communication_quality = 1.0f;  // 0.0-1.0
    } communication;
    
    // 系统资源使用情况
    struct {
        float  cpu_usage       = 0.0f;  ///< CPU使用率 (0.0-1.0)
        float  memory_usage    = 0.0f;  ///< 内存使用率 (0.0-1.0)
        float  disk_usage      = 0.0f;  ///< 磁盘使用率 (0.0-1.0)
        float  network_usage   = 0.0f;  ///< 网络使用率 (0.0-1.0)
        int8_t cpu_temperature = 0;     ///< CPU温度 (°C)
    } system_resources;
    
    // 障碍物感知 (Go2特有)
    std::vector<float> range_obstacles; ///< 4个方向的障碍物距离
    
    DetailedRobotState() {
        // 为Go2初始化固定数量的电机和足端
        motors.resize(20);
        feet.resize(4);
        range_obstacles.resize(4, 0.0f);
        
        // 初始化电机ID
        for (size_t i = 0; i < motors.size(); ++i) {
            motors[i].motor_id = static_cast<uint8_t>(i);
        }
        
        // 初始化足端ID
        for (size_t i = 0; i < feet.size(); ++i) {
            feet[i].foot_id = static_cast<uint8_t>(i);
        }
    }
};

/**
 * @brief 告警信息
 */
struct AlertInfo {
    AlertType    type   = AlertType::INFO;
    SystemModule module = SystemModule::MOTION_CONTROL;
    uint32_t code = 0;              ///< 告警代码
    std::string message;            ///< 告警消息
    std::string description;        ///< 详细描述
    uint64_t timestamp_ns = 0;      ///< 告警时间戳
    bool is_active = true;          ///< 是否仍然活跃
    uint32_t occurrence_count = 1;  ///< 发生次数
    
    // 告警相关数据
    std::map<std::string, float> numeric_data;
    std::map<std::string, std::string> string_data;
    
    AlertInfo() = default;
    AlertInfo(AlertType t, SystemModule m, uint32_t c, const std::string& msg)
        : type(t), module(m), code(c), message(msg) {}
};

/**
 * @brief 系统诊断信息
 */
struct DiagnosticInfo {
    SystemModule module;
    HealthLevel health_level = HealthLevel::UNKNOWN;
    float health_score = 0.0f;      ///< 健康评分 (0.0-1.0)
    std::string status_message;     ///< 状态消息
    uint64_t last_update_ns = 0;    ///< 最后更新时间
    
    // 模块特定指标
    std::map<std::string, float> metrics;
    std::map<std::string, bool>  status_flags;
    std::vector<AlertInfo>       active_alerts;
    
    DiagnosticInfo() = default;
    DiagnosticInfo(SystemModule m) : module(m) {}
};

/**
 * @brief 性能统计信息
 */
struct PerformanceStats {
    // 运动性能
    struct {
        float    max_speed_achieved = 0.0f; ///< 达到的最大速度 (m/s)
        float    average_speed      = 0.0f; ///< 平均速度
        float    total_distance     = 0.0f; ///< 总行驶距离 (m)
        uint32_t step_count         = 0;    ///< 步数统计
        float    uptime_hours       = 0.0f; ///< 运行时间 (小时)
    } motion;
    
    // 电池性能
    struct {
        uint32_t charge_cycles       = 0;       ///< 充电循环次数
        float    average_consumption = 0.0f;    ///< 平均功耗 (W)
        float    efficiency_score    = 1.0f;    ///< 效率评分 (0.0-1.0)
    } power;
    
    // 传感器性能
    struct {
        uint32_t lidar_scan_count        = 0;       ///< 激光扫描次数
        float    average_lidar_frequency = 0.0f;    ///< 平均激光频率
        uint32_t imu_sample_count        = 0;       ///< IMU采样次数
        float    sensor_error_rate       = 0.0f;    ///< 传感器错误率
    } sensors;
    
    // 通信性能
    struct {
        uint32_t messages_sent      = 0;    ///< 发送消息数
        uint32_t messages_received  = 0;    ///< 接收消息数
        uint32_t messages_lost      = 0;    ///< 丢失消息数
        float    average_latency_ms = 0.0f; ///< 平均延迟 (ms)
    } communication;
};

/**
 * @brief Go2特定状态配置
 */
struct Go2StateConfig {
    // 状态监控频率配置
    float motion_state_frequency = 50.0f;   ///< 运动状态频率 (Hz)
    float low_state_frequency    = 100.0f;  ///< 低级状态频率 (Hz)
    float diagnostic_frequency   = 1.0f;    ///< 诊断频率 (Hz)
    
    // 健康评估阈值
    struct {
        float excellent_threshold = 0.9f;   ///< 优秀阈值
        float good_threshold      = 0.7f;   ///< 良好阈值
        float fair_threshold      = 0.5f;   ///< 一般阈值
        float poor_threshold      = 0.3f;   ///< 较差阈值
    } health_thresholds;
    
    // 温度阈值 (°C)
    struct {
        int8_t motor_warning  = 60;         ///< 电机温度警告
        int8_t motor_critical = 80;         ///< 电机温度严重
        int8_t cpu_warning    = 70;         ///< CPU温度警告
        int8_t cpu_critical   = 90;         ///< CPU温度严重
    } temperature_limits;
    
    // 通信质量阈值
    struct {
        float    good_quality = 0.95f;      ///< 良好通信质量
        float    poor_quality = 0.8f;       ///< 较差通信质量
        uint32_t max_lost_messages = 100;   ///< 最大丢失消息数
    } communication_thresholds;
};

} // namespace state_interface
} // namespace robot_base_interfaces

#endif // ROBOT_BASE_INTERFACES__STATE_INTERFACE__ROBOT_STATE_TYPES_HPP_