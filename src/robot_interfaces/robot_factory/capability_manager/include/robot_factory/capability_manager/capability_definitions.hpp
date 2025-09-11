/**
 * @file capability_definitions.hpp
 * @brief 机器人能力定义的完整数据结构
 * @author Claude Code
 * @date 2024
 * 
 * 该文件定义了机器人能力的完整数据结构，包括：
 * 1. 运动能力 (速度、加速度、四足特有能力等)
 * 2. 传感器能力 (LiDAR、摄像头、IMU等)
 * 3. 电源能力 (电池、充电系统等)
 * 4. 通信能力 (网络、ROS2、DDS配置等)
 * 5. 计算能力 (CPU、内存、AI/ML支持等)
 * 6. 物理属性 (尺寸、重量、环境适应性等)
 */

#ifndef ROBOT_FACTORY__CAPABILITY_MANAGER__CAPABILITY_DEFINITIONS_HPP_
#define ROBOT_FACTORY__CAPABILITY_MANAGER__CAPABILITY_DEFINITIONS_HPP_

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <chrono>

// 导入共同的类型定义
#include "robot_factory/capability_manager/capability_types.hpp"

namespace robot_factory {
namespace capability_manager {

// 前向声明以避免循环依赖
class CapabilityManager;

/**
 * @brief 能力类别枚举
 */
enum class CapabilityCategory {
    MOTION = 0,        ///< 运动能力
    SENSOR = 1,        ///< 传感器能力  
    POWER = 2,         ///< 电源能力
    COMMUNICATION = 3, ///< 通信能力
    PROCESSING = 4,    ///< 计算处理能力
    PHYSICAL = 5,      ///< 物理属性
    NAVIGATION = 6,    ///< 导航能力
    MANIPULATION = 7,  ///< 操控能力 (预留)
    SAFETY = 8,        ///< 安全能力
    CUSTOM = 99        ///< 自定义能力
};

// ============= 运动能力定义 =============

/**
 * @brief 四足机器人专用运动能力
 */
struct QuadrupedMotionCapabilities {
    bool can_walk = true;              ///< 支持步行模式
    bool can_trot = false;             ///< 支持小跑模式
    bool can_bound = false;            ///< 支持跳跃模式
    bool can_crawl = false;            ///< 支持爬行模式
    bool can_pronk = false;            ///< 支持原地跳跃
    
    // 地形适应能力
    bool can_climb_stairs = false;     ///< 可以爬楼梯
    bool can_traverse_rough_terrain = false; ///< 可以穿越崎岖地形
    bool can_balance_on_two_legs = false;    ///< 可以双腿平衡站立
    
    // 步态参数
    float max_step_height = 0.0f;      ///< 最大跨越高度 (m)
    float max_step_length = 0.0f;      ///< 最大步长 (m)
    float min_ground_clearance = 0.0f; ///< 最小离地间隙 (m)
    
    // 稳定性参数
    float static_stability_margin = 0.0f; ///< 静态稳定裕度
    float dynamic_stability_margin = 0.0f; ///< 动态稳定裕度
    float max_slope_angle_deg = 0.0f;      ///< 最大爬坡角度 (度)
    
    QuadrupedMotionCapabilities() = default;
};

/**
 * @brief 运动能力总体定义
 */
struct MotionCapabilities {
    // 基础运动参数
    float max_linear_velocity = 0.0f;     ///< 最大线速度 (m/s)
    float max_angular_velocity = 0.0f;    ///< 最大角速度 (rad/s)
    float max_linear_acceleration = 0.0f; ///< 最大线加速度 (m/s²)
    float max_angular_acceleration = 0.0f; ///< 最大角加速度 (rad/s²)
    
    // 运动控制参数
    float turning_radius = 0.0f;          ///< 最小转弯半径 (m)
    float stopping_distance = 0.0f;       ///< 制动距离 (m)
    float position_accuracy = 0.0f;       ///< 位置精度 (m)
    float orientation_accuracy = 0.0f;    ///< 姿态精度 (rad)
    
    // 支持的运动模式
    std::vector<std::string> locomotion_modes; ///< 运动模式列表
    std::string default_locomotion_mode;   ///< 默认运动模式
    
    // 控制频率
    float control_frequency = 0.0f;       ///< 控制频率 (Hz)
    float max_update_rate = 0.0f;         ///< 最大更新率 (Hz)
    
    // 四足机器人特有能力 (如果适用)
    QuadrupedMotionCapabilities quadruped_specific;
    
    // 能力级别评估
    CapabilityLevel overall_level = CapabilityLevel::NONE;
    CapabilityLevel speed_level = CapabilityLevel::NONE;
    CapabilityLevel agility_level = CapabilityLevel::NONE;
    CapabilityLevel stability_level = CapabilityLevel::NONE;
    
    MotionCapabilities() = default;
    
    // 方法声明
    std::string toString() const;
    bool isValid() const;
    static MotionCapabilities createDefault();
};

// ============= 传感器能力定义 =============

/**
 * @brief LiDAR传感器能力
 */
struct LiDARCapabilities {
    bool has_2d_lidar = false;             ///< 是否有2D激光雷达
    bool has_3d_lidar = false;             ///< 是否有3D激光雷达
    
    // 性能参数
    float max_range = 0.0f;                ///< 最大测距 (m)
    float min_range = 0.0f;                ///< 最小测距 (m)
    float accuracy = 0.0f;                 ///< 测距精度 (m)
    float angular_resolution = 0.0f;       ///< 角分辨率 (度)
    int scan_frequency = 0;                ///< 扫描频率 (Hz)
    
    // 视场参数
    float horizontal_fov_deg = 0.0f;       ///< 水平视场角 (度)
    float vertical_fov_deg = 0.0f;         ///< 垂直视场角 (度)
    
    // 点云参数
    int max_points_per_scan = 0;           ///< 每次扫描最大点数
    bool supports_intensity = false;       ///< 支持强度信息
    bool supports_color = false;           ///< 支持颜色信息
    
    // 环境适应性
    float min_operating_temp = -40.0f;     ///< 最低工作温度 (°C)
    float max_operating_temp = 85.0f;      ///< 最高工作温度 (°C)
    int ip_rating = 0;                     ///< IP防护等级
    
    std::string lidar_model;               ///< LiDAR型号
    std::string manufacturer;              ///< 制造商
    
    LiDARCapabilities() = default;
};

/**
 * @brief 摄像头传感器能力
 */
struct CameraCapabilities {
    int rgb_camera_count = 0;              ///< RGB摄像头数量
    int depth_camera_count = 0;            ///< 深度摄像头数量
    int thermal_camera_count = 0;          ///< 热红外摄像头数量
    
    // 图像参数
    std::string max_resolution;            ///< 最大分辨率 (例如："1920x1080")
    int max_framerate = 0;                 ///< 最大帧率 (fps)
    std::vector<std::string> supported_formats; ///< 支持的图像格式
    
    // 视觉能力
    bool has_stereo_vision = false;        ///< 支持立体视觉
    bool has_auto_focus = false;           ///< 支持自动对焦
    bool has_auto_exposure = false;        ///< 支持自动曝光
    bool has_auto_white_balance = false;   ///< 支持自动白平衡
    
    // 视场参数
    float horizontal_fov_deg = 0.0f;       ///< 水平视场角 (度)
    float vertical_fov_deg = 0.0f;         ///< 垂直视场角 (度)
    
    // 深度感知
    float min_depth_range = 0.0f;          ///< 最小深度测量 (m)
    float max_depth_range = 0.0f;          ///< 最大深度测量 (m)
    float depth_accuracy = 0.0f;           ///< 深度精度 (m)
    
    CameraCapabilities() = default;
};

/**
 * @brief IMU传感器能力
 */
struct IMUCapabilities {
    int degrees_of_freedom = 6;            ///< 自由度 (6DOF或9DOF)
    int sampling_rate = 100;               ///< 采样率 (Hz)
    
    // 陀螺仪参数
    float gyro_range_dps = 0.0f;           ///< 陀螺仪量程 (度/秒)
    float gyro_resolution = 0.0f;          ///< 陀螺仪分辨率
    float gyro_noise_density = 0.0f;       ///< 陀螺仪噪声密度
    
    // 加速度计参数  
    float accel_range_g = 0.0f;            ///< 加速度计量程 (g)
    float accel_resolution = 0.0f;         ///< 加速度计分辨率
    float accel_noise_density = 0.0f;      ///< 加速度计噪声密度
    
    // 磁力计参数 (如果有)
    bool has_magnetometer = false;         ///< 是否有磁力计
    float mag_range_gauss = 0.0f;          ///< 磁力计量程 (高斯)
    float mag_resolution = 0.0f;           ///< 磁力计分辨率
    
    // 温度补偿
    bool has_temperature_compensation = false; ///< 是否有温度补偿
    float operating_temp_range_min = -40.0f;   ///< 最低工作温度 (°C)
    float operating_temp_range_max = 85.0f;    ///< 最高工作温度 (°C)
    
    IMUCapabilities() = default;
};

/**
 * @brief 传感器能力总体定义
 */
struct SensorCapabilities {
    LiDARCapabilities lidar;               ///< LiDAR能力
    CameraCapabilities camera;             ///< 摄像头能力
    IMUCapabilities imu;                   ///< IMU能力
    
    // 其他传感器
    bool has_gps = false;                  ///< 是否有GPS
    bool has_encoders = true;              ///< 是否有编码器
    bool has_force_sensors = false;        ///< 是否有力传感器
    bool has_temperature_sensors = false;  ///< 是否有温度传感器
    bool has_humidity_sensor = false;      ///< 是否有湿度传感器
    bool has_pressure_sensor = false;      ///< 是否有气压传感器
    
    // GPS能力 (如果有)
    struct {
        float position_accuracy = 0.0f;     ///< 位置精度 (m)
        bool supports_rtk = false;          ///< 支持RTK
        bool supports_dgps = false;         ///< 支持DGPS
        int update_rate = 0;                ///< 更新率 (Hz)
    } gps_specific;
    
    // 传感器融合能力
    bool supports_sensor_fusion = false;   ///< 支持传感器融合
    std::vector<std::string> fusion_algorithms; ///< 支持的融合算法
    
    CapabilityLevel overall_level = CapabilityLevel::NONE;
    
    SensorCapabilities() = default;
    
    // 方法声明
    std::string toString() const;
    bool isValid() const;
    static SensorCapabilities createDefault();
};

// ============= 电源能力定义 =============

/**
 * @brief 充电系统能力
 */
enum class ChargingType {
    NONE = 0,              ///< 不支持充电
    CONTACT = 1,           ///< 接触式充电
    WIRELESS = 2,          ///< 无线充电  
    INDUCTIVE = 3,         ///< 感应式充电
    SOLAR = 4,             ///< 太阳能充电
    HYBRID = 5             ///< 混合充电方式
};

struct ChargingCapabilities {
    ChargingType primary_type = ChargingType::NONE;        ///< 主要充电方式
    std::vector<ChargingType> supported_types;             ///< 支持的充电方式
    
    // 充电参数
    float max_charging_power = 0.0f;       ///< 最大充电功率 (W)
    float charging_efficiency = 0.0f;      ///< 充电效率 (0-1)
    int full_charge_time_minutes = 0;      ///< 完整充电时间 (分钟)
    int fast_charge_time_minutes = 0;      ///< 快速充电时间 (分钟，80%容量)
    
    // 充电站兼容性
    std::vector<std::string> compatible_stations; ///< 兼容的充电站类型
    bool supports_auto_docking = false;    ///< 支持自动对接
    float docking_accuracy = 0.0f;         ///< 对接精度 (m)
    
    // 安全特性
    bool has_over_current_protection = false;  ///< 过流保护
    bool has_over_voltage_protection = false;  ///< 过压保护
    bool has_temperature_monitoring = false;   ///< 温度监控
    
    ChargingCapabilities() = default;
};

/**
 * @brief 电源能力总体定义
 */
struct PowerCapabilities {
    // 电池参数
    float battery_capacity_mah = 0.0f;     ///< 电池容量 (mAh)
    float battery_voltage = 0.0f;          ///< 电池电压 (V)
    float operating_voltage_min = 0.0f;    ///< 最低工作电压 (V)
    float operating_voltage_max = 0.0f;    ///< 最高工作电压 (V)
    
    // 运行时间
    float estimated_runtime_hours = 0.0f;  ///< 预计运行时间 (小时)
    float idle_runtime_hours = 0.0f;       ///< 待机运行时间 (小时)
    float active_runtime_hours = 0.0f;     ///< 活跃运行时间 (小时)
    
    // 功耗信息
    float idle_power_consumption = 0.0f;   ///< 待机功耗 (W)
    float normal_power_consumption = 0.0f; ///< 正常功耗 (W)
    float peak_power_consumption = 0.0f;   ///< 峰值功耗 (W)
    
    // 电池管理
    bool supports_hot_swap = false;        ///< 支持热插拔
    bool has_power_monitoring = true;      ///< 有电源监控
    bool has_low_power_mode = false;       ///< 有低功耗模式
    bool has_hibernation_mode = false;     ///< 有休眠模式
    bool can_estimate_remaining_time = false; ///< 可以估算剩余时间
    
    // 充电能力
    ChargingCapabilities charging;
    
    // 电池健康监控
    bool has_battery_health_monitoring = false; ///< 电池健康监控
    bool can_report_charge_cycles = false;      ///< 可以报告充电周期数
    bool has_cell_balancing = false;            ///< 有电池均衡
    
    CapabilityLevel overall_level = CapabilityLevel::NONE;
    
    PowerCapabilities() = default;
    
    // 方法声明
    std::string toString() const;
    bool isValid() const;
    static PowerCapabilities createDefault();
};

// ============= 通信能力定义 =============

/**
 * @brief 网络通信能力
 */
struct NetworkCapabilities {
    // 支持的网络类型
    std::vector<std::string> supported_networks; ///< 支持的网络 (wifi, ethernet, 4g, 5g等)
    std::vector<std::string> supported_protocols; ///< 支持的协议 (tcp, udp, websocket等)
    
    // 网络参数
    bool has_wifi = false;                 ///< 支持WiFi
    bool has_ethernet = true;              ///< 支持以太网
    bool has_4g_5g = false;                ///< 支持4G/5G
    bool has_bluetooth = false;            ///< 支持蓝牙
    bool has_zigbee = false;               ///< 支持ZigBee
    
    // 网络性能
    float max_wifi_range_m = 0.0f;         ///< WiFi最大范围 (m)
    float typical_latency_ms = 0.0f;       ///< 典型延迟 (ms)
    float max_bandwidth_mbps = 0.0f;       ///< 最大带宽 (Mbps)
    
    // 远程控制
    bool has_remote_control = false;       ///< 支持远程控制
    bool supports_teleoperation = false;   ///< 支持远程操作
    bool supports_video_streaming = false; ///< 支持视频流
    
    NetworkCapabilities() = default;
};

/**
 * @brief ROS2通信能力
 */
struct ROS2Capabilities {
    // ROS2配置
    std::string default_rmw_implementation; ///< 默认RMW实现
    std::vector<std::string> supported_rmw; ///< 支持的RMW实现
    bool supports_multi_domain = false;     ///< 支持多域
    
    // DDS配置
    std::string preferred_dds;              ///< 首选DDS实现
    int max_participants = 100;            ///< 最大参与者数
    bool supports_security = false;        ///< 支持DDS安全
    bool supports_discovery = true;        ///< 支持自动发现
    
    // QoS支持
    bool supports_reliable_qos = true;     ///< 支持可靠QoS
    bool supports_best_effort_qos = true;  ///< 支持尽力而为QoS  
    bool supports_deadline_qos = false;    ///< 支持截止时间QoS
    bool supports_lifespan_qos = false;    ///< 支持生命周期QoS
    
    // 性能参数
    float max_publish_rate = 0.0f;         ///< 最大发布频率 (Hz)
    int max_subscribers = 0;               ///< 最大订阅者数
    int max_publishers = 0;                ///< 最大发布者数
    
    ROS2Capabilities() = default;
};

/**
 * @brief 通信能力总体定义
 */
struct CommunicationCapabilities {
    NetworkCapabilities network;           ///< 网络通信能力
    ROS2Capabilities ros2;                 ///< ROS2通信能力
    
    // 通信安全
    bool supports_encryption = false;      ///< 支持加密
    bool supports_authentication = false;  ///< 支持身份验证
    bool supports_access_control = false;  ///< 支持访问控制
    
    // 通信诊断
    bool has_connection_monitoring = false; ///< 连接监控
    bool has_bandwidth_monitoring = false;  ///< 带宽监控
    bool has_latency_monitoring = false;    ///< 延迟监控
    
    CapabilityLevel overall_level = CapabilityLevel::NONE;
    
    CommunicationCapabilities() = default;
    
    // 方法声明
    std::string toString() const;
    bool isValid() const;
    static CommunicationCapabilities createDefault();
};

// ============= 计算处理能力定义 =============

/**
 * @brief AI/ML处理能力
 */
struct AIMLCapabilities {
    // 框架支持
    bool supports_tensorflow = false;      ///< 支持TensorFlow
    bool supports_pytorch = false;         ///< 支持PyTorch  
    bool supports_onnx = false;           ///< 支持ONNX
    bool supports_tensorrt = false;        ///< 支持TensorRT
    bool supports_openvino = false;        ///< 支持OpenVINO
    
    // 硬件加速
    bool has_gpu_acceleration = false;     ///< GPU加速
    bool has_npu_acceleration = false;     ///< NPU加速
    bool has_tpu_acceleration = false;     ///< TPU加速
    
    // 推理性能
    float max_inference_fps = 0.0f;        ///< 最大推理帧率 (fps)
    std::vector<std::string> supported_models; ///< 支持的模型类型
    
    // 内存限制
    float max_model_size_mb = 0.0f;        ///< 最大模型大小 (MB)
    float available_inference_memory_mb = 0.0f; ///< 可用推理内存 (MB)
    
    AIMLCapabilities() = default;
};

/**
 * @brief 计算处理能力总体定义
 */
struct ProcessingCapabilities {
    // 硬件规格
    std::string cpu_architecture;          ///< CPU架构 (ARM64, x86_64等)
    int cpu_cores = 0;                     ///< CPU核心数
    float cpu_frequency_ghz = 0.0f;        ///< CPU频率 (GHz)
    int ram_gb = 0;                        ///< 内存大小 (GB)
    int storage_gb = 0;                    ///< 存储大小 (GB)
    
    // GPU信息
    bool has_gpu = false;                  ///< 是否有GPU
    std::string gpu_model;                 ///< GPU型号
    int gpu_memory_gb = 0;                 ///< GPU内存 (GB)
    
    // AI/ML支持
    AIMLCapabilities aiml;
    
    // 实时性能
    float max_control_frequency = 0.0f;    ///< 最大控制频率 (Hz)
    float typical_latency_ms = 0.0f;       ///< 典型延迟 (ms)
    float max_processing_load = 0.0f;      ///< 最大处理负载 (%)
    
    // 操作系统
    std::string os_type;                   ///< 操作系统类型
    std::string os_version;                ///< 操作系统版本
    
    CapabilityLevel overall_level = CapabilityLevel::NONE;
    
    ProcessingCapabilities() = default;
    
    // 方法声明
    std::string toString() const;
    bool isValid() const;
    static ProcessingCapabilities createDefault();
};

// ============= 物理属性定义 =============

/**
 * @brief 环境适应能力
 */
struct EnvironmentalCapabilities {
    // 温度范围
    float min_operating_temperature = -20.0f; ///< 最低工作温度 (°C)
    float max_operating_temperature = 60.0f;  ///< 最高工作温度 (°C)
    float min_storage_temperature = -30.0f;   ///< 最低储存温度 (°C)
    float max_storage_temperature = 70.0f;    ///< 最高储存温度 (°C)
    
    // 湿度和防护
    float max_humidity = 95.0f;             ///< 最大湿度 (%)
    int ip_rating = 0;                      ///< IP防护等级 (例如54表示IP54)
    bool is_waterproof = false;             ///< 是否防水
    bool is_dustproof = false;              ///< 是否防尘
    
    // 海拔和气压
    float max_altitude = 3000.0f;           ///< 最大海拔 (m)
    float min_pressure_kpa = 80.0f;         ///< 最低气压 (kPa)
    
    // 震动和冲击
    float max_vibration_g = 0.0f;           ///< 最大震动 (g)
    float max_shock_g = 0.0f;               ///< 最大冲击 (g)
    
    EnvironmentalCapabilities() = default;
};

/**
 * @brief 物理属性总体定义
 */
struct PhysicalProperties {
    // 尺寸参数 (米)
    float length = 0.0f;                   ///< 长度 (m)
    float width = 0.0f;                    ///< 宽度 (m)
    float height = 0.0f;                   ///< 高度 (m)
    float weight = 0.0f;                   ///< 重量 (kg)
    
    // 负载能力
    float max_payload = 0.0f;              ///< 最大负载 (kg)
    std::vector<std::string> mounting_points; ///< 安装点位置
    
    // 运动学参数
    float wheelbase = 0.0f;                ///< 轴距 (m) - 适用于轮式机器人
    float track_width = 0.0f;              ///< 轮距 (m) - 适用于轮式机器人
    float ground_clearance = 0.0f;         ///< 离地间隙 (m)
    
    // 重心和稳定性
    float center_of_gravity_height = 0.0f; ///< 重心高度 (m)
    float static_stability_margin = 0.0f;  ///< 静态稳定裕度
    
    // 材料和构造
    std::vector<std::string> construction_materials; ///< 构造材料
    std::string frame_type;                ///< 车架类型
    
    // 环境适应性
    EnvironmentalCapabilities environmental;
    
    // 维护参数
    float expected_lifespan_years = 0.0f;  ///< 预期寿命 (年)
    int maintenance_interval_hours = 0;     ///< 维护间隔 (小时)
    
    PhysicalProperties() = default;
    
    // 方法声明
    std::string toString() const;
    bool isValid() const;
    static PhysicalProperties createDefault();
};

// ============= 综合能力结构 =============

/**
 * @brief 能力验证结果
 */
struct CapabilityValidationResult {
    bool is_valid = true;                  ///< 验证是否通过
    float completeness_score = 0.0f;       ///< 完整性评分 (0-1)
    
    std::vector<std::string> errors;       ///< 错误列表
    std::vector<std::string> warnings;     ///< 警告列表
    std::vector<std::string> missing_fields; ///< 缺失字段列表
    std::vector<std::string> suggestions;  ///< 改进建议
    
    CapabilityValidationResult() = default;
    CapabilityValidationResult(bool valid, float score = 0.0f) 
        : is_valid(valid), completeness_score(score) {}
};

/**
 * @brief 机器人综合能力定义
 * 
 * 这是机器人能力的完整定义，包含所有类别的能力信息。
 */
struct RobotCapabilities {
    // 基础信息
    RobotType robot_type = static_cast<RobotType>(0);  ///< 机器人类型
    std::string robot_model;               ///< 机器人型号
    std::string manufacturer;              ///< 制造商
    std::string firmware_version;          ///< 固件版本
    std::string hardware_version;          ///< 硬件版本
    
    // 各类能力定义
    MotionCapabilities motion;             ///< 运动能力
    SensorCapabilities sensors;            ///< 传感器能力
    PowerCapabilities power;               ///< 电源能力
    CommunicationCapabilities communication; ///< 通信能力
    ProcessingCapabilities processing;     ///< 计算处理能力
    PhysicalProperties physical;           ///< 物理属性
    
    // 高级能力 (预留扩展)
    std::map<std::string, std::string> navigation_capabilities;  ///< 导航能力
    std::map<std::string, std::string> safety_capabilities;     ///< 安全能力
    std::map<std::string, std::string> custom_capabilities;     ///< 自定义能力
    
    // 元数据
    std::string profile_version = "1.0.0"; ///< 配置文件版本
    std::string last_updated;             ///< 最后更新时间
    std::string created_by;               ///< 创建者
    std::vector<std::string> tags;        ///< 标签列表
    
    // 能力评估
    CapabilityLevel overall_capability_level = CapabilityLevel::NONE;
    float capability_score = 0.0f;        ///< 综合能力评分 (0-100)
    
    // 兼容性信息
    std::vector<std::string> compatible_software_versions; ///< 兼容的软件版本
    std::vector<std::string> required_dependencies;        ///< 必需的依赖
    std::vector<std::string> optional_dependencies;        ///< 可选的依赖
    
    RobotCapabilities() = default;
    RobotCapabilities(RobotType type, const std::string& model, const std::string& manufacturer)
        : robot_type(type), robot_model(model), manufacturer(manufacturer) {}
    
    // 方法声明
    std::string toString() const;
    bool isValid() const;
    static RobotCapabilities createDefault();
    static RobotCapabilities createGo2Profile();
};

// ============= 工具函数声明 =============

/**
 * @brief 机器人能力工具类
 */
class RobotCapabilityUtils {
public:
    /**
     * @brief 将机器人类型转换为字符串
     */
    static std::string robotTypeToString(RobotType robot_type);
    
    /**
     * @brief 将字符串转换为机器人类型
     */
    static RobotType stringToRobotType(const std::string& type_str);
    
    /**
     * @brief 将能力级别转换为字符串
     */
    static std::string capabilityLevelToString(CapabilityLevel level);
    
    /**
     * @brief 将字符串转换为能力级别
     */
    static CapabilityLevel stringToCapabilityLevel(const std::string& level_str);
    
    /**
     * @brief 将充电类型转换为字符串
     */
    static std::string chargingTypeToString(ChargingType type);
    
    /**
     * @brief 将字符串转换为充电类型
     */
    static ChargingType stringToChargingType(const std::string& type_str);
    
    /**
     * @brief 验证机器人能力配置
     */
    static CapabilityValidationResult validateCapabilities(const RobotCapabilities& capabilities);
    
    /**
     * @brief 计算能力完整性评分
     */
    static float calculateCompletenessScore(const RobotCapabilities& capabilities);
    
    /**
     * @brief 计算综合能力评分
     */
    static float calculateOverallScore(const RobotCapabilities& capabilities);
    
    /**
     * @brief 比较两个机器人的能力
     */
    static std::map<std::string, float> compareCapabilities(
        const RobotCapabilities& capabilities1, 
        const RobotCapabilities& capabilities2);
};

} // namespace capability_manager
} // namespace robot_factory

#endif // ROBOT_FACTORY__CAPABILITY_MANAGER__CAPABILITY_DEFINITIONS_HPP_