/**
 * @file   go2_message_converter.hpp
 * @brief  Go2机器人消息转换器
 * @author Yang nan
 * @date   2025-09-11
 * 
 * 负责Go2原生消息格式与统一接口消息格式之间的转换：
 *      - Go2 SportModeState <-> 统一 MotionState
 *      - Go2 LowState <-> 统一 DetailedRobotState
 *      - Go2 BmsState <-> 统一 BatteryInfo
 *      - Go2 WirelessController <-> 统一控制输入
 *      - 标准ROS消息与Go2原生消息的转换
 */

#ifndef ROBOT_ADAPTERS__GO2_ADAPTER__GO2_MESSAGE_CONVERTER_HPP_
#define ROBOT_ADAPTERS__GO2_ADAPTER__GO2_MESSAGE_CONVERTER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <functional>
#include <cstdint>
#include <thread>

// Go2配置管理器
#include "robot_adapters/go2_adapter/go2_config_manager.hpp"

// Go2 原生消息类型 - 从go_ros2包导入
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/bms_state.hpp"
#include "unitree_go/msg/wireless_controller.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"

// 标准ROS2消息类型
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

// 统一接口消息类型
#include "robot_base_interfaces/motion_interface/motion_types.hpp"
#include "robot_base_interfaces/power_interface/power_types.hpp"
#include "robot_base_interfaces/sensor_interface/sensor_types.hpp"
#include "robot_base_interfaces/state_interface/state_types.hpp"

namespace robot_adapters {
namespace go2_adapter {

/**
 * @brief 消息转换结果枚举
 */
enum class ConversionResult {
    SUCCESS          = 0,   ///< 转换成功
    INVALID_INPUT    = 1,   ///< 输入无效
    CONVERSION_ERROR = 2,   ///< 转换错误
    UNSUPPORTED_TYPE = 3,   ///< 不支持的类型
    DATA_INCOMPLETE  = 4    ///< 数据不完整
};

/**
 * @brief 转换选项结构体
 */
struct ConversionOptions {
    bool validate_ranges     = true;    ///< 是否验证数值范围
    bool fill_missing_data   = true;    ///< 是否填充缺失数据
    bool preserve_timestamps = true;    ///< 是否保持时间戳
    float default_timeout_s  = 1.0f;    ///< 默认超时时间(秒)
    
    // Go2特定选项
    bool use_go2_coordinate_frame = true;   ///< 使用Go2坐标系
    bool enable_go2_extensions    = true;   ///< 启用Go2扩展功能
    bool strict_validation        = false;  ///< 严格验证模式
};

/**
 * @brief Go2消息转换器
 * 
 * 提供Go2原生消息与统一接口消息之间的双向转换功能：
 * - 类型安全的消息转换
 * - 数据完整性验证
 * - 坐标系转换
 * - 时间戳同步
 * - 扩展字段映射
 */
class Go2MessageConverter {
public:
    /**
     * @brief 默认构造函数（使用默认配置）
     */
    Go2MessageConverter();

    /**
     * @brief 构造函数（带配置选项）
     * @param options 转换选项
     */
    explicit Go2MessageConverter(const ConversionOptions& options);

    /**
     * @brief 构造函数（带配置文件）
     * @param config_file_path 配置文件路径
     */
    explicit Go2MessageConverter(const std::string& config_file_path);

    /**
     * @brief 构造函数（带配置选项和配置文件）
     * @param options 转换选项
     * @param config_file_path 配置文件路径
     */
    Go2MessageConverter(const ConversionOptions& options, const std::string& config_file_path);

    /**
     * @brief 构造函数（带配置管理器）
     * @param config_manager 配置管理器的智能指针
     */
    explicit Go2MessageConverter(std::shared_ptr<Go2ConfigManager> config_manager);
    
    /**
     * @brief 析构函数
     */
    virtual ~Go2MessageConverter();

    // ============= 配置管理 =============
    
    /**
     * @brief 设置转换选项
     * @param options 转换选项
     */
    void setConversionOptions(const ConversionOptions& options);
    
    /**
     * @brief 获取转换选项
     * @return 当前转换选项
     */
    const ConversionOptions& getConversionOptions() const { return options_; }
    
    /**
     * @brief 重置为默认配置
     */
    void resetToDefaults();

    /**
     * @brief 加载配置文件
     * @param config_file_path 配置文件路径
     * @return 加载成功返回true，否则返回false
     */
    bool loadConfigFile(const std::string& config_file_path);

    /**
     * @brief 重新加载配置文件
     * @return 加载成功返回true，否则返回false
     */
    bool reloadConfig();

    /**
     * @brief 设置配置管理器
     * @param config_manager 配置管理器的智能指针
     */
    void setConfigManager(std::shared_ptr<Go2ConfigManager> config_manager);

    /**
     * @brief 获取配置管理器
     * @return 配置管理器的智能指针
     */
    std::shared_ptr<Go2ConfigManager> getConfigManager() const;

    /**
     * @brief 获取消息转换器配置
     * @return 消息转换器配置的引用
     */
    const MessageConverterConfig& getMessageConverterConfig() const;

    // ============= 运动状态转换 =============
    
    /**
     * @brief Go2 SportModeState -> 统一 MotionState
     * @param go2_state Go2运动状态
     * @param unified_state 统一运动状态（输出）
     * @return 转换结果
     */
    ConversionResult convertSportModeState(
        const unitree_go::msg::SportModeState& go2_state,
        robot_base_interfaces::motion_interface::MotionState& unified_state) const;
    
    /**
     * @brief 统一 MotionState -> Go2 SportModeState
     * @param unified_state 统一运动状态
     * @param go2_state Go2运动状态（输出）
     * @return 转换结果
     */
    ConversionResult convertMotionState(
        const robot_base_interfaces::motion_interface::MotionState& unified_state,
        unitree_go::msg::SportModeState& go2_state) const;
    
    /**
     * @brief 转换运动模式
     * @param go2_mode Go2运动模式
     * @return 统一运动模式
     */
    robot_base_interfaces::motion_interface::MotionMode 
    convertMotionMode(uint8_t go2_mode) const;
    
    /**
     * @brief 转换步态类型
     * @param go2_gait Go2步态
     * @return 统一步态类型
     */
    robot_base_interfaces::motion_interface::GaitType 
    convertGaitType(uint8_t go2_gait) const;

    // ============= 机器人状态转换 =============
    
    /**
     * @brief Go2 LowState -> 统一 DetailedRobotState
     * @param go2_state Go2低级状态
     * @param unified_state 统一详细状态（输出）
     * @return 转换结果
     */
    ConversionResult convertLowState(
        const unitree_go::msg::LowState& go2_state,
        robot_base_interfaces::state_interface::DetailedRobotState& unified_state) const;
    
    /**
     * @brief 转换电机信息
     * @param go2_motors Go2电机状态数组
     * @param unified_motors 统一电机信息（输出）
     * @return 转换结果
     */
    ConversionResult convertMotorInfo(
        const std::vector<unitree_go::msg::MotorState>& go2_motors,
        std::vector<robot_base_interfaces::state_interface::MotorInfo>& unified_motors) const;
    
    /**
     * @brief 转换足端信息
     * @param go2_feet Go2足端状态
     * @param unified_feet 统一足端信息（输出）
     * @return 转换结果
     */
    ConversionResult convertFootInfo(
        const std::array<int16_t, 4>& go2_foot_forces,
        std::vector<robot_base_interfaces::state_interface::FootInfo>& unified_feet) const;
    
    /**
     * @brief 转换IMU信息
     * @param go2_imu_state Go2 IMU状态
     * @param unified_imu 统一IMU信息（输出）
     * @return 转换结果
     */
    ConversionResult convertIMUInfo(
        const unitree_go::msg::IMUState& go2_imu_state,
        decltype(robot_base_interfaces::state_interface::DetailedRobotState{}.imu)& unified_imu) const;

    // ============= 电源状态转换 =============
    
    /**
     * @brief Go2 BmsState -> 统一 BatteryInfo
     * @param go2_bms Go2电池管理状态
     * @param unified_battery 统一电池信息（输出）
     * @return 转换结果
     */
    ConversionResult convertBmsState(
        const unitree_go::msg::BmsState& go2_bms,
        robot_base_interfaces::power_interface::BatteryInfo& unified_battery) const;

    /**
     * @brief 根据BMS数据推断电池健康状态
     * @param go2_bms Go2 BMS状态数据
     * @return 推断的电池健康状态
     */
    robot_base_interfaces::power_interface::BatteryHealth
    inferBatteryHealthFromBms(const unitree_go::msg::BmsState& go2_bms) const;

    /**
     * @brief 转换充电状态
     * @param go2_charging_status Go2充电状态
     * @return 统一充电状态
     */
    robot_base_interfaces::power_interface::ChargingState 
    convertChargingState(uint8_t go2_charging_status) const;

    // ============= 遥控器状态转换 =============

    /**
     * @brief Go2 WirelessController -> 统一遥控器状态
     * @param go2_controller Go2遥控器状态
     * @param unified_controller 统一遥控器状态（输出）
     * @return 转换结果
     */
    ConversionResult convertWirelessController(
        const unitree_go::msg::WirelessController& go2_controller,
        decltype(robot_base_interfaces::state_interface::DetailedRobotState{}.wireless_controller)& unified_controller) const;

    /**
     * @brief 统一遥控器状态 -> ROS Twist
     * @param unified_controller 统一遥控器状态
     * @param twist ROS Twist消息（输出）
     * @return 转换结果
     */
    ConversionResult convertControllerToTwist(
        const decltype(robot_base_interfaces::state_interface::DetailedRobotState{}.wireless_controller)& unified_controller,
        geometry_msgs::msg::Twist& twist) const;

    // ============= 传感器数据转换 =============

    /**
     * @brief Go2 IMU数据转换
     * @param go2_imu Go2 IMU数据
     * @param ros_imu ROS IMU消息（输出）
     * @return 转换结果
     */
    ConversionResult convertImuData(
        const std::vector<float>& go2_imu,
        sensor_msgs::msg::Imu& ros_imu) const;

    /**
     * @brief ROS IMU消息转换为统一格式
     * @param ros_imu ROS IMU消息
     * @param unified_imu 统一格式IMU数据（输出）
     * @return 转换结果
     */
    ConversionResult convertRosImuToUnified(
        const sensor_msgs::msg::Imu& ros_imu,
        robot_base_interfaces::sensor_interface::IMUData& unified_imu) const;

    /**
     * @brief 点云数据增强处理
     * @param pointcloud 原始点云
     * @param enhanced_info 增强信息（输出）
     * @return 转换结果
     */
    ConversionResult enhancePointCloudData(
        const sensor_msgs::msg::PointCloud2& pointcloud,
        robot_base_interfaces::sensor_interface::PointCloudData& enhanced_info) const;

    // ============= 里程计转换 =============

    /**
     * @brief Go2状态 -> ROS里程计消息
     * @param go2_state Go2运动状态
     * @param odometry ROS里程计消息（输出）
     * @return 转换结果
     */
    ConversionResult convertToOdometry(
        const unitree_go::msg::SportModeState& go2_state,
        nav_msgs::msg::Odometry& odometry) const;

    // ============= 电池状态转换 =============

    /**
     * @brief Go2电池状态 -> ROS电池消息 (预留接口)
     * @param go2_bms Go2电池管理状态
     * @param battery_msg ROS电池消息（输出）
     * @return 转换结果
     */
    ConversionResult convertBatteryToRos(
        const unitree_go::msg::BmsState& go2_bms,
        std::map<std::string, float>& battery_msg) const;

    // ============= API响应解析 =============

    /**
     * @brief 解析Go2 API响应消息
     * @param response Go2 API响应
     * @param result_info 解析结果信息（输出）
     * @return 转换结果
     */
    ConversionResult parseApiResponse(
        const unitree_api::msg::Response& response,
        std::map<std::string, std::string>& result_info) const;

    // ============= 反向转换 (统一格式 -> Go2) =============

    /**
     * @brief 统一BatteryInfo -> Go2 BmsState (部分字段)
     * @param unified_battery 统一电池信息
     * @param go2_bms Go2电池状态（输出）
     * @return 转换结果
     */
    ConversionResult convertBatteryInfoToBms(
        const robot_base_interfaces::power_interface::BatteryInfo& unified_battery,
        unitree_go::msg::BmsState& go2_bms) const;

    /**
     * @brief 统一DetailedRobotState -> Go2 LowState (部分字段)
     * @param unified_state 统一详细状态
     * @param go2_low_state Go2低级状态（输出）
     * @return 转换结果
     */
    ConversionResult convertDetailedStateToLowState(
        const robot_base_interfaces::state_interface::DetailedRobotState& unified_state,
        unitree_go::msg::LowState& go2_low_state) const;

    /**
     * @brief ROS Odometry -> Go2 SportModeState (部分字段)
     * @param odometry ROS里程计消息
     * @param go2_state Go2运动状态（输出）
     * @return 转换结果
     */
    ConversionResult convertOdometryToSportMode(
        const nav_msgs::msg::Odometry& odometry,
        unitree_go::msg::SportModeState& go2_state) const;

    /**
     * @brief ROS Imu -> Go2 IMUState
     * @param ros_imu ROS IMU消息
     * @param go2_imu Go2 IMU状态（输出）
     * @return 转换结果
     */
    ConversionResult convertRosImuToGo2(
        const sensor_msgs::msg::Imu& ros_imu,
        unitree_go::msg::IMUState& go2_imu) const;

    // ============= 控制命令转换 =============
    
    /**
     * @brief 统一速度命令 -> Go2 API Request
     * @param velocity 统一速度命令
     * @param go2_request Go2 API请求（输出）
     * @return 转换结果
     */
    ConversionResult convertVelocityCommand(
        const robot_base_interfaces::motion_interface::Velocity& velocity,
        unitree_api::msg::Request& go2_request) const;
    
    /**
     * @brief 统一姿态命令 -> Go2 API Request
     * @param posture 统一姿态命令
     * @param go2_request Go2 API请求（输出）
     * @return 转换结果
     */
    ConversionResult convertPostureCommand(
        const robot_base_interfaces::motion_interface::Posture& posture,
        unitree_api::msg::Request& go2_request) const;
    
    /**
     * @brief ROS Twist -> Go2速度命令
     * @param twist ROS Twist消息
     * @param go2_velocity Go2速度参数（输出）
     * @return 转换结果
     */
    ConversionResult convertTwistToGo2Velocity(
        const geometry_msgs::msg::Twist& twist,
        std::vector<float>& go2_velocity) const;

    /**
     * @brief ROS Twist -> Go2 API Request (直接转换)
     * @param twist ROS Twist消息
     * @param go2_request Go2 API请求（输出）
     * @return 转换结果
     * @details 这是一个便利函数，直接将Twist转换为可发送的Go2 API请求
     */
    ConversionResult convertTwistToApiRequest(
        const geometry_msgs::msg::Twist& twist,
        unitree_api::msg::Request& go2_request) const;

    // ============= 坐标系转换 =============
    
    /**
     * @brief Go2坐标系 -> ROS坐标系
     * @param go2_position Go2位置 [x, y, z]
     * @param ros_position ROS位置（输出）
     * @return 转换结果
     */
    ConversionResult convertCoordinateFrame(
        const std::vector<float>& go2_position,
        geometry_msgs::msg::Vector3& ros_position) const;
    
    /**
     * @brief ROS坐标系 -> Go2坐标系
     * @param ros_position ROS位置
     * @param go2_position Go2位置（输出）
     * @return 转换结果
     */
    ConversionResult convertCoordinateFrame(
        const geometry_msgs::msg::Vector3& ros_position,
        std::vector<float>& go2_position) const;
    
    /**
     * @brief 姿态四元数转换
     * @param go2_quaternion Go2四元数 [w, x, y, z]
     * @param ros_pose ROS姿态（输出）
     * @return 转换结果
     */
    ConversionResult convertQuaternion(
        const std::vector<float>& go2_quaternion,
        geometry_msgs::msg::Pose& ros_pose) const;

    // ============= 时间戳处理 =============
    
    /**
     * @brief Go2时间戳 -> ROS时间戳
     * @param go2_timestamp_ns Go2纳秒时间戳
     * @return ROS时间戳
     */
    builtin_interfaces::msg::Time convertTimestamp(uint64_t go2_timestamp_ns) const;
    
    /**
     * @brief ROS时间戳 -> Go2时间戳
     * @param ros_time ROS时间戳
     * @return Go2纳秒时间戳
     */
    uint64_t convertTimestamp(const builtin_interfaces::msg::Time& ros_time) const;
    
    /**
     * @brief 同步时间戳
     * @param go2_time Go2时间
     * @param ros_time ROS时间
     * @return 时间偏移(纳秒)
     */
    int64_t synchronizeTimestamps(uint64_t go2_time, const builtin_interfaces::msg::Time& ros_time) const;

    // ============= 数据验证 =============
    
    /**
     * @brief 验证Go2状态数据
     * @param data 数据指针
     * @param size 数据大小
     * @param type 数据类型标识
     * @return true if valid
     */
    bool validateGo2Data(const void* data, size_t size, const std::string& type) const;
    
    /**
     * @brief 验证数值范围
     * @param value 数值
     * @param min_val 最小值
     * @param max_val 最大值
     * @param field_name 字段名称（用于错误报告）
     * @return true if in range
     */
    bool validateRange(float value, float min_val, float max_val, const std::string& field_name) const;
    
    /**
     * @brief 验证向量数据
     * @param vec 向量数据
     * @param expected_size 期望大小
     * @param field_name 字段名称
     * @return true if valid
     */
    bool validateVector(const std::vector<float>& vec, size_t expected_size, const std::string& field_name) const;

    // ============= 辅助功能 =============
    
    /**
     * @brief 获取转换统计信息
     * @return 统计信息(JSON格式)
     */
    std::string getConversionStatistics() const;
    
    /**
     * @brief 重置转换统计
     */
    void resetStatistics();
    
    /**
     * @brief 获取支持的消息类型列表
     * @return 支持的类型列表
     */
    std::vector<std::string> getSupportedMessageTypes() const;
    
    /**
     * @brief 获取转换器版本
     * @return 版本字符串
     */
    std::string getVersion() const { return "1.0.0-go2"; }
    
    /**
     * @brief 获取最后的转换错误
     * @return 错误信息
     */
    std::string getLastError() const;

    // ============= 批量转换 =============
    
    /**
     * @brief 批量转换Go2状态消息
     * @param go2_messages Go2消息列表
     * @param unified_states 统一状态列表（输出）
     * @return 成功转换的数量
     */
    template<typename Go2MsgType, typename UnifiedType>
    size_t batchConvert(
        const std::vector<Go2MsgType>& go2_messages,
        std::vector<UnifiedType>& unified_states) const;
    
    /**
     * @brief 异步批量转换
     * @param go2_messages Go2消息列表
     * @param callback 完成回调
     */
    template<typename Go2MsgType, typename UnifiedType>
    void asyncBatchConvert(
        const std::vector<Go2MsgType>& go2_messages,
        std::function<void(const std::vector<UnifiedType>&)> callback) const;

private:
    // ============= 内部数据 =============
    ConversionOptions options_;                       ///< 转换选项
    std::shared_ptr<Go2ConfigManager> config_manager_; ///< 配置管理器
    mutable std::string last_error_;                  ///< 最后错误信息
    
    // 统计信息
    struct ConversionStats {
        uint64_t total_conversions      = 0;    ///< 总转换次数
        uint64_t successful_conversions = 0;    ///< 成功转换次数
        uint64_t failed_conversions     = 0;    ///< 失败转换次数
        
        // 按消息类型统计
        std::map<std::string, uint64_t> type_counts;
        std::map<std::string, uint64_t> type_errors;
        
        // 性能统计
        double total_conversion_time_ms = 0.0;      ///< 总转换时间
        double max_conversion_time_ms   = 0.0;      ///< 最长转换时间
        double min_conversion_time_ms   = 999999.0; ///< 最短转换时间
    } mutable stats_;
    
    // 坐标系变换矩阵
    struct TransformMatrices {
        std::vector<std::vector<float>> go2_to_ros;     ///< Go2到ROS变换矩阵
        std::vector<std::vector<float>> ros_to_go2;     ///< ROS到Go2变换矩阵
    } transforms_;
    
    // 常量映射表
    std::map<uint8_t,  robot_base_interfaces::motion_interface::MotionMode>   motion_mode_map_;
    std::map<uint8_t,  robot_base_interfaces::motion_interface::GaitType>     gait_type_map_;
    std::map<uint32_t, robot_base_interfaces::power_interface::BatteryHealth> battery_health_map_;
    std::map<uint8_t,  robot_base_interfaces::power_interface::ChargingState> charging_state_map_;

    // ============= 私有方法 =============
    
    /**
     * @brief 初始化转换映射表
     */
    void initializeMappingTables();

    /**
     * @brief 从配置管理器初始化转换映射表
     */
    void initializeMappingTablesFromConfig();
    
    /**
     * @brief 初始化坐标变换矩阵
     */
    void initializeTransformMatrices();

    /**
     * @brief 从配置管理器初始化坐标变换矩阵
     */
    void initializeTransformMatricesFromConfig();
    
    /**
     * @brief 记录转换统计
     */
    void recordConversion(const std::string& type, bool success, double time_ms) const;
    
    /**
     * @brief 设置错误信息
     */
    void setError(const std::string& error_msg) const;
    
    /**
     * @brief 验证输入参数
     */
    bool validateInput(const void* input, const std::string& type) const;
    
    /**
     * @brief 填充默认数据
     */
    template<typename T>
    void fillDefaults(T& data) const;
    
    /**
     * @brief 应用数值限制
     */
    void applyLimits(float& value, float min_val, float max_val) const;
    
    /**
     * @brief 计算校验和
     */
    uint32_t calculateChecksum(const void* data, size_t size) const;
    
    /**
     * @brief 矩阵乘法
     */
    std::vector<float> matrixMultiply(
        const std::vector<std::vector<float>>& matrix,
        const std::vector<float>& vector) const;
    
    /**
     * @brief 四元数归一化
     */
    std::vector<float> normalizeQuaternion(const std::vector<float>& quat) const;
    
    /**
     * @brief 欧拉角转四元数
     */
    std::vector<float> eulerToQuaternion(float roll, float pitch, float yaw) const;
    
    /**
     * @brief 四元数转欧拉角
     */
    std::vector<float> quaternionToEuler(const std::vector<float>& quat) const;
    
    /**
     * @brief 安全类型转换
     */
    template<typename To, typename From>
    To safeCast(const From& value, To default_value = To()) const;
};

/**
 * @brief Go2MessageConverter智能指针类型定义
 */
using Go2MessageConverterPtr = std::shared_ptr<Go2MessageConverter>;
using Go2MessageConverterUniquePtr = std::unique_ptr<Go2MessageConverter>;

// ============= 模板实现 =============

template<typename Go2MsgType, typename UnifiedType>
size_t Go2MessageConverter::batchConvert(
    const std::vector<Go2MsgType>& go2_messages,
    std::vector<UnifiedType>& unified_states) const {
    
    unified_states.clear();
    unified_states.reserve(go2_messages.size());
    size_t success_count = 0;
    
    for (const auto& go2_msg : go2_messages) {
        UnifiedType unified_state;
        // 这里需要根据具体类型调用相应的转换函数
        // ConversionResult result = convertSpecificType(go2_msg, unified_state);
        // if (result == ConversionResult::SUCCESS) {
        //     unified_states.push_back(unified_state);
        //     ++success_count;
        // }
    }
    
    return success_count;
}

template<typename Go2MsgType, typename UnifiedType>
void Go2MessageConverter::asyncBatchConvert(
    const std::vector<Go2MsgType>& go2_messages,
    std::function<void(const std::vector<UnifiedType>&)> callback) const {
    
    // 异步转换实现
    std::thread([this, go2_messages, callback]() {
        std::vector<UnifiedType> unified_states;
        batchConvert(go2_messages, unified_states);
        callback(unified_states);
    }).detach();
}

template<typename T>
void Go2MessageConverter::fillDefaults(T& /* data */) const {
    // 根据类型填充默认值的实现
    // 这需要为每个支持的类型特化
}

template<typename To, typename From>
To Go2MessageConverter::safeCast(const From& value, To default_value) const {
    try {
        return static_cast<To>(value);
    } catch (...) {
        setError("Safe cast failed from " + std::string(typeid(From).name()) + 
                " to " + std::string(typeid(To).name()));
        return default_value;
    }
}

} // namespace go2_adapter
} // namespace robot_adapters
#endif //ROBOT_ADAPTERS__GO2_ADAPTER__GO2_MESSAGE_CONVERTER_HPP_