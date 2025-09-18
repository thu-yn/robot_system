/**
 * @file   go2_config_manager.hpp
 * @brief  Go2适配器配置管理器
 * @author Yang Nan
 * @date   2025-09-18
 *
 * 提供统一的配置文件读取和管理功能，支持：
 * - YAML配置文件解析
 * - 配置参数验证
 * - 默认值管理
 * - 配置热重载
 */

#pragma once

#include <string>
#include <memory>
#include <stdexcept>
#include <filesystem>
#include <vector>
#include <map>

namespace robot_adapters {
namespace go2_adapter {

/**
 * @brief 通信模块配置结构体
 * 对应go2_communication.hpp中的NetworkConfig
 */
struct CommunicationConfig {
    struct Network {
        std::string robot_ip = "192.168.123.18";        // Go2机器人的默认IP地址
        std::string local_ip = "192.168.123.99";        // 本地设备的IP地址
        std::string network_interface = "enp129s0";     // 默认使用的网络接口
    } network;

    struct DDS {
        int domain_id = 0;                               // DDS域ID
    } dds;

    struct Connection {
        int timeout_ms = 10000;                          // 连接超时时间（毫秒）
        int reconnect_interval_ms = 5000;                // 自动重连间隔（毫秒）
        int max_reconnect_attempts = 0;                  // 最大重连次数（0为无限）
    } connection;
};

/**
 * @brief 消息转换器配置结构体
 * 对应go2_message_converter.hpp中的转换器配置
 */
struct MessageConverterConfig {
    struct ConversionOptions {
        bool validate_ranges = true;                     // 是否验证数值范围
        bool fill_missing_data = true;                   // 是否填充缺失数据
        bool preserve_timestamps = true;                 // 是否保持时间戳
        float default_timeout_s = 1.0f;                  // 默认超时时间(秒)
        bool use_go2_coordinate_frame = true;            // 使用Go2坐标系
        bool enable_go2_extensions = true;               // 启用Go2扩展功能
        bool strict_validation = false;                  // 严格验证模式
    } conversion_options;

    struct BufferSizes {
        size_t sport_mode_state = 10;                    // 运动模式状态缓冲区大小
        size_t low_state = 10;                           // 底层状态缓冲区大小
        size_t bms_state = 10;                           // 电池管理状态缓冲区大小
        size_t point_cloud = 5;                          // 点云数据缓冲区大小
        size_t imu_data = 20;                            // IMU数据缓冲区大小
        size_t odometry = 10;                            // 里程计数据缓冲区大小
    } buffer_sizes;

    struct ValidationRanges {
        struct Velocity {
            float linear_x_max = 1.5f;                   // 最大前进速度 (m/s)
            float linear_y_max = 0.5f;                   // 最大侧向速度 (m/s)
            float angular_z_max = 2.0f;                  // 最大角速度 (rad/s)
            float movement_threshold = 0.01f;            // 运动状态判断速度阈值 (m/s)
        } velocity;

        struct Posture {
            float roll_max = 0.5f;                       // 最大翻滚角 (rad)
            float pitch_max = 0.5f;                      // 最大俯仰角 (rad)
            float yaw_max = 3.14159f;                    // 最大偏航角 (rad)
            float body_height_min = 0.05f;               // 最小机身高度 (m)
            float body_height_max = 0.4f;                // 最大机身高度 (m)
        } posture;

        struct Battery {
            float voltage_min = 20.0f;                   // 最小电池电压 (V)
            float voltage_max = 100.0f;                  // 最大电池电压 (V)
            float current_max = 50.0f;                   // 最大电流 (A)
            float temperature_min = -20.0f;              // 最小工作温度 (°C)
            float temperature_max = 80.0f;               // 最大工作温度 (°C)
            float capacity_min = 0.0f;                   // 最小容量百分比
            float capacity_max = 100.0f;                 // 最大容量百分比
            float nominal_voltage = 25.2f;               // 标准电压 (V)
            uint16_t capacity_mah = 15000;               // 电池容量 (mAh)
            uint8_t cell_count = 15;                     // 电芯数量
            float default_cell_temperature = 25.0f;      // 默认电芯温度 (°C)
        } battery;

        struct BatteryHealth {
            uint16_t excellent_cycles = 100;             // 优秀状态循环次数
            uint16_t good_cycles = 300;                  // 良好状态循环次数
            uint16_t fair_cycles = 600;                  // 一般状态循环次数
            uint16_t poor_cycles = 1000;                 // 较差状态循环次数
            uint8_t alarm_status = 11;                   // 电池报警状态码
            float default_health_percentage = 100.0f;    // 默认健康度百分比
        } battery_health;

        struct Sensor {
            // IMU传感器范围
            float imu_gyroscope_max = 35.0f;             // 陀螺仪最大值 (rad/s)
            float imu_accelerometer_max = 157.0f;        // 加速度计最大值 (m/s²)
            float imu_temperature_min = -40.0f;          // IMU最低温度 (°C)
            float imu_temperature_max = 85.0f;           // IMU最高温度 (°C)
            float imu_default_temperature = 25;          // IMU默认温度 (°C)
            float quaternion_tolerance = 0.1f;           // 四元数模长容差

            // 电机传感器范围
            float motor_temperature_min = -100.0f;       // 电机最低温度 (°C)
            float motor_temperature_max = 100.0f;        // 电机最高温度 (°C)
            float motor_temp_clamp_min = -50.0f;         // 电机温度限制下限 (°C)
            float motor_temp_clamp_max = 100.0f;         // 电机温度限制上限 (°C)
            float motor_position_max = 6.28f;            // 电机最大位置 (rad)
            float motor_velocity_max = 50.0f;            // 电机最大速度 (rad/s)
            uint8_t motor_count = 12;                    // 电机总数

            // 足端传感器
            float foot_contact_threshold = 10.0f;        // 足端接触力阈值 (N)
            float foot_force_min = -100.0f;              // 最小足端力 (N)
            float foot_force_max = 1000.0f;              // 最大足端力 (N)
            uint8_t foot_count = 4;                      // 足端总数
        } sensor;

        struct Controller {
            float deadzone = 0.05f;                      // 控制器摇杆死区
            float activity_threshold = 0.01f;            // 控制器活动检测阈值
            float stick_range = 1.0f;                    // 摇杆值范围
        } controller;

        struct Timing {
            uint64_t nanoseconds_per_second = 1000000000ULL;  // 每秒纳秒数
        } timing;

        struct Odometry {
            float position_covariance = 0.1f;            // 位置协方差
            float orientation_covariance = 0.1f;         // 角度协方差
            float velocity_covariance = 0.1f;            // 速度协方差
        } odometry;
    } validation_ranges;

    struct CoordinateTransforms {
        // 3x3变换矩阵，使用vector<vector<float>>存储
        std::vector<std::vector<float>> go2_to_ros_matrix;  // Go2到ROS变换矩阵
        std::vector<std::vector<float>> ros_to_go2_matrix;  // ROS到Go2变换矩阵
    } coordinate_transforms;

    struct TypeMappings {
        std::map<uint8_t, std::string> motion_modes;     // 运动模式映射
        std::map<uint8_t, std::string> gait_types;       // 步态类型映射
        std::map<uint8_t, std::string> battery_health;   // 电池健康状态映射
        std::map<uint8_t, std::string> charging_states;  // 充电状态映射
    } type_mappings;
};

/**
 * @brief Go2适配器配置管理器
 *
 * 负责加载和管理Go2适配器的所有配置参数
 * - 支持从YAML文件加载配置
 * - 提供配置参数验证
 * - 支持默认配置值
 */
class Go2ConfigManager {
public:
    /**
     * @brief 构造函数
     * @param config_file_path 配置文件路径
     */
    explicit Go2ConfigManager(const std::string& config_file_path = "");

    /**
     * @brief 析构函数
     */
    ~Go2ConfigManager() = default;

    /**
     * @brief 加载配置文件
     * @param config_file_path 配置文件路径
     * @return 加载成功返回true，否则返回false
     */
    bool loadConfig(const std::string& config_file_path);

    /**
     * @brief 重新加载配置文件
     * @return 加载成功返回true，否则返回false
     */
    bool reloadConfig();

    /**
     * @brief 获取通信模块配置
     * @return 通信模块配置结构体
     */
    const CommunicationConfig& getCommunicationConfig() const;

    /**
     * @brief 获取消息转换器配置
     * @return 消息转换器配置结构体
     */
    const MessageConverterConfig& getMessageConverterConfig() const;

    /**
     * @brief 验证配置参数的有效性
     * @return 配置有效返回true，否则返回false
     */
    bool validateConfig() const;

    /**
     * @brief 获取配置文件路径
     * @return 配置文件路径
     */
    const std::string& getConfigFilePath() const;

    /**
     * @brief 检查配置文件是否存在
     * @return 文件存在返回true，否则返回false
     */
    bool configFileExists() const;

    /**
     * @brief 获取默认配置文件路径
     * @return 默认配置文件路径
     */
    static std::string getDefaultConfigPath();

private:
    /**
     * @brief 加载默认配置
     */
    void loadDefaultConfig();

    /**
     * @brief 从YAML文件加载通信配置
     * @param config_node YAML配置节点
     */
    void loadCommunicationConfig(const void* config_node);

    /**
     * @brief 从YAML文件加载消息转换器配置
     * @param config_node YAML配置节点
     */
    void loadMessageConverterConfig(const void* config_node);

    /**
     * @brief 验证IP地址格式
     * @param ip IP地址字符串
     * @return 有效返回true，否则返回false
     */
    static bool isValidIPAddress(const std::string& ip);

    /**
     * @brief 验证网络接口名称
     * @param interface 网络接口名称
     * @return 有效返回true，否则返回false
     */
    static bool isValidNetworkInterface(const std::string& interface);

private:
    std::string config_file_path_;                    ///< 配置文件路径
    CommunicationConfig communication_config_;        ///< 通信模块配置
    MessageConverterConfig message_converter_config_; ///< 消息转换器配置
    bool config_loaded_;                              ///< 配置是否已加载
};

/**
 * @brief 配置管理器异常类
 */
class ConfigException : public std::runtime_error {
public:
    explicit ConfigException(const std::string& message)
        : std::runtime_error("Go2ConfigManager Error: " + message) {}
};

} // namespace go2_adapter
} // namespace robot_adapters