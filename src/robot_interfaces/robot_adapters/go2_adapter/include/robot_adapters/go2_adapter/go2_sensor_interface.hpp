/**
 * @file   go2_sensor_interface.hpp
 * @brief  Go2机器人传感器接口头文件
 * @author Yang Nan
 * @date   2025-09-14
 *
 * 基于go2_communication和go2_message_converter实现的Go2传感器接口
 * 提供标准化的传感器数据访问和消息转换功能
 */

#ifndef ROBOT_ADAPTERS__GO2_ADAPTER__GO2_SENSOR_INTERFACE_HPP_
#define ROBOT_ADAPTERS__GO2_ADAPTER__GO2_SENSOR_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <map>
#include <vector>
#include <string>
#include <mutex>
#include <functional>
#include <atomic>

// 统一传感器接口
#include "robot_base_interfaces/sensor_interface/i_sensor_interface.hpp"
#include "robot_base_interfaces/sensor_interface/sensor_types.hpp"
#include "robot_base_interfaces/state_interface/state_types.hpp"

// Go2核心组件
#include "robot_adapters/go2_adapter/go2_communication.hpp"
#include "robot_adapters/go2_adapter/go2_message_converter.hpp"

// Go2原生消息类型
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/bms_state.hpp"
#include "unitree_go/msg/wireless_controller.hpp"

// 标准ROS2传感器消息类型
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace robot_adapters {
namespace go2_adapter {

/**
 * @class Go2SensorInterface
 * @brief Go2机器人传感器数据接口实现
 *
 * 该类基于go2_communication和go2_message_converter实现统一传感器接口，
 * 专门用于处理Go2机器人的传感器数据，支持：
 *
 * **硬件传感器：**
 * - Livox Mid360 3D激光雷达（40米探测距离）
 * - 内置6轴IMU（加速度计+陀螺仪）
 * - 前置立体摄像头（RGB+深度）
 * - 足部接触力传感器（4个足端）
 * - 关节位置和扭矩传感器（20个关节）
 *
 * **数据源集成：**
 * - SportModeState：运动状态、IMU、位置、速度
 * - LowState：低层状态、电机、足端力、电池
 * - PointCloud2：激光雷达点云数据
 * - 标准ROS2传感器消息
 *
 * **核心特性：**
 * - 统一的数据访问接口
 * - 自动消息格式转换
 * - 多源数据融合
 * - 实时状态监控
 */
class Go2SensorInterface : public robot_base_interfaces::sensor_interface::ISensorInterface,
                           public rclcpp::Node {
public:
    /**
     * @brief 构造函数
     * @param node_name ROS2节点名称
     * @param communication_ptr 可选的通信管理器（用于依赖注入）
     */
    explicit Go2SensorInterface(const std::string& node_name = "go2_sensor_interface",
                                std::shared_ptr<Go2Communication> communication_ptr = nullptr);
    
    /**
     * @brief 析构函数
     */
    ~Go2SensorInterface() override;

    // ========== ISensorInterface 接口实现 ==========
    
    /**
     * @brief 初始化传感器接口
     * @return bool 初始化成功返回true
     * 
     * 初始化所有传感器的ROS2订阅器和数据缓存
     */
    bool initialize() override;
    
    /**
     * @brief 关闭传感器接口
     */
    bool shutdown() override;
    
    /**
     * @brief 获取所有可用的传感器信息
     */
    std::vector<robot_base_interfaces::sensor_interface::SensorInfo> getAvailableSensors() const override;
    
    /** 检查特定传感器是否可用 */
    bool isSensorAvailable(robot_base_interfaces::sensor_interface::SensorType sensor_type) const override;
    
    /** 启动/停止传感器 */
    bool startSensor(robot_base_interfaces::sensor_interface::SensorType sensor_type) override;
    bool stopSensor (robot_base_interfaces::sensor_interface::SensorType sensor_type) override;
    
    /** 传感器状态查询 */
    robot_base_interfaces::sensor_interface::SensorStatus getSensorStatus(robot_base_interfaces::sensor_interface::SensorType sensor_type) const override;
    
    /** 最新数据获取（通用/点云/IMU） */
    std::shared_ptr<robot_base_interfaces::sensor_interface::SensorData>     getLatestData(robot_base_interfaces::sensor_interface::SensorType sensor_type) const override;
    std::shared_ptr<robot_base_interfaces::sensor_interface::PointCloudData> getLatestPointCloud() const override;
    std::shared_ptr<robot_base_interfaces::sensor_interface::IMUData>        getLatestIMU()        const override;

    /**
     * @brief 获取融合的IMU数据
     * @return 融合后的IMU数据
     */
    std::shared_ptr<robot_base_interfaces::sensor_interface::IMUData> getFusedIMUData() const;
    
    /** 数据回调设置 */
    void setPointCloudCallback(std::function<void(const std::shared_ptr<robot_base_interfaces::sensor_interface::PointCloudData>&)> callback) override;
    void setIMUCallback(std::function<void(const std::shared_ptr<robot_base_interfaces::sensor_interface::IMUData>&)> callback) override;
    void setSensorCallback(
        robot_base_interfaces::sensor_interface::SensorType sensor_type,
        std::function<void(const std::shared_ptr<robot_base_interfaces::sensor_interface::SensorData>&)> callback) override;
    
    /** 传感器参数配置 */
    bool setSensorParameter(robot_base_interfaces::sensor_interface::SensorType sensor_type,
                            const std::string& parameter_name,
                            float value) override;
    bool getSensorParameter(robot_base_interfaces::sensor_interface::SensorType sensor_type,
                            const std::string& parameter_name,
                            float& value) const override;
    bool setSensorFrequency(robot_base_interfaces::sensor_interface::SensorType sensor_type,
                            float frequency) override;
    
    /** 传感器校准接口 */
    bool calibrateSensor(robot_base_interfaces::sensor_interface::SensorType sensor_type) override;
    robot_base_interfaces::sensor_interface::CalibrationData getCalibrationData(robot_base_interfaces::sensor_interface::SensorType sensor_type) const override;
    bool setCalibrationData(const robot_base_interfaces::sensor_interface::CalibrationData& calibration_data) override;
    
    /** 坐标变换 */
    std::vector<float> getSensorTransform(robot_base_interfaces::sensor_interface::SensorType sensor_type) const override;
    bool setSensorTransform(robot_base_interfaces::sensor_interface::SensorType sensor_type,
                            const std::vector<float>& transform) override;
    
    /** 诊断和监控 */
    std::map<robot_base_interfaces::sensor_interface::SensorType, robot_base_interfaces::sensor_interface::SensorStatus> getSensorHealth() const override;
    std::string getSensorError(robot_base_interfaces::sensor_interface::SensorType sensor_type) const override;
    std::map<std::string, float> getSensorStatistics(robot_base_interfaces::sensor_interface::SensorType sensor_type) const override;

    // ========== Go2特有方法 ==========

    /**
     * @brief 获取Go2运动状态数据（包含IMU信息）
     * @return 最新的Go2运动状态
     */
    std::shared_ptr<unitree_go::msg::SportModeState> getLatestSportModeState() const;

    /**
     * @brief 获取Go2低层状态数据（包含关节、足端信息）
     * @return 最新的Go2低层状态
     */
    std::shared_ptr<unitree_go::msg::LowState> getLatestLowState() const;

    /**
     * @brief 获取Go2电池管理状态
     * @return 最新的电池状态
     */
    std::shared_ptr<unitree_go::msg::BmsState> getLatestBmsState() const;

    /**
     * @brief 获取关节传感器数据
     * @param joint_id 关节ID（0-19）
     * @return 关节信息，包含位置、速度、扭矩
     */
    robot_base_interfaces::state_interface::MotorInfo getMotorData(int joint_id) const;

    /**
     * @brief 获取足端传感器数据
     * @param foot_id 足端ID（0-3：前右、前左、后右、后左）
     * @return 足端力传感器数据
     */
    robot_base_interfaces::state_interface::FootInfo getFootData(int foot_id) const;

    /**
     * @brief 设置Go2特有传感器回调
     * @param sport_callback 运动状态回调
     * @param low_state_callback 低层状态回调
     * @param bms_callback 电池状态回调
     */
    void setGo2Callbacks(
        std::function<void(const std::shared_ptr<unitree_go::msg::SportModeState>&)> sport_callback,
        std::function<void(const std::shared_ptr<unitree_go::msg::LowState>&)> low_state_callback,
        std::function<void(const std::shared_ptr<unitree_go::msg::BmsState>&)> bms_callback);

    /**
     * @brief 获取通信质量统计
     * @return 通信统计信息
     */
    std::map<std::string, float> getCommunicationStatistics() const;

    /**
     * @brief 强制更新所有传感器数据
     * @details 主动从通信管理器获取最新数据
     */
    void refreshAllSensorData();

private:
    // ========== 核心组件 ==========

    /// Go2通信管理器
    std::shared_ptr<Go2Communication> communication_;

    /// Go2消息转换器
    std::unique_ptr<Go2MessageConverter> message_converter_;

    /// 是否拥有通信管理器（用于析构时决定是否释放）
    bool owns_communication_;

    // ========== 初始化方法 ==========
    
    /**
     * @brief 初始化Go2通信管理器
     * @return 初始化是否成功
     */
    bool initializeCommunication();

    /**
     * @brief 初始化消息转换器
     */
    void initializeMessageConverter();

    /**
     * @brief 设置Go2原生消息回调
     */
    void setupGo2MessageCallbacks();

    /**
     * @brief 初始化传感器状态映射
     */
    void initializeSensorMappings();

    /**
     * @brief 验证Go2通信连接
     * @return 连接是否正常
     */
    bool validateCommunication() const;

    // ========== Go2原生数据回调函数 ==========

    /**
     * @brief Go2运动状态回调（来自通信管理器）
     * @param msg Go2运动状态消息
     * @details 包含IMU、位置、速度、步态等信息
     */
    void onSportModeStateReceived(const unitree_go::msg::SportModeState::SharedPtr msg);

    /**
     * @brief Go2低层状态回调（来自通信管理器）
     * @param msg Go2低层状态消息
     * @details 包含电机、足端力、温度等详细信息
     */
    void onLowStateReceived(const unitree_go::msg::LowState::SharedPtr msg);

    /**
     * @brief Go2电池状态回调（来自通信管理器）
     * @param msg Go2电池管理状态消息
     */
    void onBmsStateReceived(const unitree_go::msg::BmsState::SharedPtr msg);

    /**
     * @brief 标准点云数据回调（来自通信管理器）
     * @param msg 点云消息
     */
    void onPointCloudReceived(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    /**
     * @brief 标准IMU数据回调（来自通信管理器）
     * @param msg IMU消息
     */
    void onStandardImuReceived(const sensor_msgs::msg::Imu::SharedPtr msg);

    /**
     * @brief 标准里程计数据回调（来自通信管理器）
     * @param msg 里程计消息
     */
    void onOdometryReceived(const nav_msgs::msg::Odometry::SharedPtr msg);

    // ========== 数据处理与转换方法 ==========

    /**
     * @brief 从Go2运动状态中提取IMU数据
     * @param sport_state Go2运动状态
     * @return 统一格式的IMU数据
     */
    robot_base_interfaces::sensor_interface::IMUData extractIMUFromSportState(
        const unitree_go::msg::SportModeState& sport_state) const;

    /**
     * @brief 从Go2低层状态中提取关节数据
     * @param low_state Go2低层状态
     * @param motor_id 电机ID
     * @return 关节传感器数据
     */
    robot_base_interfaces::state_interface::MotorInfo extractMotorData(
        const unitree_go::msg::LowState& low_state, int motor_id) const;

    /**
     * @brief 从Go2低层状态中提取足端数据
     * @param low_state Go2低层状态
     * @param foot_id 足端ID
     * @return 足端传感器数据
     */
    robot_base_interfaces::state_interface::FootInfo extractFootData(
        const unitree_go::msg::LowState& low_state, int foot_id) const;

    /**
     * @brief 融合多源IMU数据
     * @param go2_imu Go2内置IMU数据（来自SportModeState）
     * @param standard_imu 标准IMU数据（来自/imu/data话题）
     * @return 融合后的IMU数据
     */
    robot_base_interfaces::sensor_interface::IMUData fuseIMUData(
        const robot_base_interfaces::sensor_interface::IMUData& go2_imu,
        const robot_base_interfaces::sensor_interface::IMUData& standard_imu) const;

    /**
     * @brief 更新传感器时间戳
     * @param sensor_type 传感器类型
     * @param timestamp 时间戳
     */
    void updateSensorTimestamp(robot_base_interfaces::sensor_interface::SensorType sensor_type,
                              const rclcpp::Time& timestamp);

    /**
     * @brief 检查传感器数据新鲜度
     * @param sensor_type 传感器类型
     * @param max_age_ms 最大数据年龄（毫秒）
     * @return 数据是否新鲜
     */
    bool isSensorDataFresh(robot_base_interfaces::sensor_interface::SensorType sensor_type,
                          int max_age_ms = 1000) const;

private:
    // ========== 数据存储 ==========

    /// 传感器数据缓存互斥锁（线程安全）
    mutable std::mutex sensor_data_mutex_;

    /// Go2原生消息数据缓存
    unitree_go::msg::SportModeState::SharedPtr latest_sport_mode_state_;
    unitree_go::msg::LowState::SharedPtr latest_low_state_;
    unitree_go::msg::BmsState::SharedPtr latest_bms_state_;

    /// 标准ROS2传感器消息缓存
    sensor_msgs::msg::PointCloud2::SharedPtr latest_point_cloud_;
    sensor_msgs::msg::Imu::SharedPtr latest_standard_imu_;
    nav_msgs::msg::Odometry::SharedPtr latest_odometry_;

    /// 转换后的统一格式数据缓存
    mutable std::mutex converted_data_mutex_;
    std::shared_ptr<robot_base_interfaces::sensor_interface::PointCloudData> cached_point_cloud_;
    std::shared_ptr<robot_base_interfaces::sensor_interface::IMUData>        cached_fused_imu_;
    std::map<int, robot_base_interfaces::state_interface::MotorInfo>         cached_motor_data_;
    std::map<int, robot_base_interfaces::state_interface::FootInfo>          cached_foot_data_;

    // ========== 传感器状态管理 ==========

    /// 传感器初始化状态
    std::map<robot_base_interfaces::sensor_interface::SensorType, bool> sensor_initialized_;

    /// 传感器数据时间戳（用于检测数据新鲜度）
    std::map<robot_base_interfaces::sensor_interface::SensorType, rclcpp::Time> sensor_timestamps_;

    /// 传感器错误计数和统计
    std::map<robot_base_interfaces::sensor_interface::SensorType, int> sensor_error_count_;
    std::map<robot_base_interfaces::sensor_interface::SensorType, uint64_t> sensor_data_count_;

    /// Go2传感器到统一接口的类型映射
    std::map<std::string, robot_base_interfaces::sensor_interface::SensorType> go2_to_unified_sensor_map_;

    /// 传感器可用性状态
    mutable std::atomic<bool> communication_active_{false};
    mutable std::atomic<bool> sport_state_available_{false};
    mutable std::atomic<bool> low_state_available_{false};
    mutable std::atomic<bool> point_cloud_available_{false};

    /// 用户设置的回调函数
    std::function<void(const std::shared_ptr<unitree_go::msg::SportModeState>&)> user_sport_callback_;
    std::function<void(const std::shared_ptr<unitree_go::msg::LowState>&)>       user_low_state_callback_;
    std::function<void(const std::shared_ptr<unitree_go::msg::BmsState>&)>       user_bms_callback_;

    /// 数据更新定时器
    rclcpp::TimerBase::SharedPtr data_validation_timer_;

    // ========== Go2传感器规格参数 ==========
    
    // Livox Mid360激光雷达规格
    static constexpr double LIDAR_MAX_RANGE = 40.0;        // 最大探测距离（米）
    static constexpr double LIDAR_MIN_RANGE = 0.5;         // 最小探测距离（米）
    static constexpr double LIDAR_RESOLUTION = 0.01;       // 距离分辨率（米）
    static constexpr double LIDAR_FOV_HORIZONTAL = 360.0;   // 水平视场角（度）
    static constexpr double LIDAR_FOV_VERTICAL = 38.4;     // 垂直视场角（度）
    static constexpr int LIDAR_FREQUENCY = 10;             // 数据频率（Hz）
    
    // IMU规格参数
    static constexpr double IMU_FREQUENCY = 1000.0;        // IMU数据频率（Hz）
    static constexpr double IMU_TEMP_RANGE_MIN = -40.0;    // 工作温度范围最小值（℃）
    static constexpr double IMU_TEMP_RANGE_MAX = 85.0;     // 工作温度范围最大值（℃）
    
    // 摄像头规格参数
    static constexpr int CAMERA_WIDTH = 1280;              // 图像宽度（像素）
    static constexpr int CAMERA_HEIGHT = 720;              // 图像高度（像素）
    static constexpr int CAMERA_FPS = 30;                  // 帧率（fps）
    
    // Go2关节和足部传感器参数
    static constexpr int NUM_JOINTS = 20;                  // 关节数量
    static constexpr int NUM_FEET = 4;                     // 足部数量

protected:
    // ========== ISensorInterface 保护接口实现 ==========

    /**
     * @brief 验证传感器类型是否支持
     */
    bool validateSensorType(robot_base_interfaces::sensor_interface::SensorType sensor_type) const override;

    /**
     * @brief 更新传感器状态
     */
    void updateSensorStatus(robot_base_interfaces::sensor_interface::SensorType sensor_type,
                            robot_base_interfaces::sensor_interface::SensorStatus status) override;

    // ========== 内部辅助方法 ==========

    /**
     * @brief 数据验证定时器回调
     * @details 定期检查数据新鲜度和通信状态
     */
    void dataValidationTimerCallback();

    /**
     * @brief 记录传感器错误
     */
    void recordSensorError(robot_base_interfaces::sensor_interface::SensorType sensor_type,
                          const std::string& error_message);

    /**
     * @brief 获取Go2电机名称
     * @param motor_id 电机ID
     * @return 电机名称字符串
     */
    std::string getMotorName(int motor_id) const;

    /**
     * @brief 获取Go2足端名称
     * @param foot_id 足端ID
     * @return 足端名称字符串
     */
    std::string getFootName(int foot_id) const;
};

/// Go2SensorInterface智能指针类型定义
using Go2SensorInterfacePtr = std::shared_ptr<Go2SensorInterface>;
using Go2SensorInterfaceUniquePtr = std::unique_ptr<Go2SensorInterface>;

} // namespace go2_adapter
} // namespace robot_adapters

#endif // ROBOT_ADAPTERS__GO2_ADAPTER__GO2_SENSOR_INTERFACE_HPP_