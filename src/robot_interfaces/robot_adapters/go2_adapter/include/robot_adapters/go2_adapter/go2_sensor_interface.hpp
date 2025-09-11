#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <map>
#include <vector>
#include <string>
#include <mutex>
#include <functional>

// 统一传感器接口
#include "robot_base_interfaces/sensor_interface/i_sensor_interface.hpp"
#include "robot_base_interfaces/sensor_interface/sensor_types.hpp"

// Go2传感器消息类型
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "unitree_go/msg/low_state.hpp"

namespace robot_adapters {
namespace go2_adapter {

/**
 * @class Go2SensorInterface
 * @brief Go2机器人传感器数据接口实现
 * 
 * 该类实现了统一传感器接口，专门用于处理Go2机器人的传感器数据
 * 支持的传感器包括：
 * - Livox Mid360 3D激光雷达
 * - 内置IMU（6轴惯性测量单元）
 * - 前置立体摄像头
 * - 足部接触传感器
 * - 关节位置和扭矩传感器
 */
class Go2SensorInterface : public robot_base_interfaces::sensor_interface::ISensorInterface,
                           public rclcpp::Node {
public:
    /**
     * @brief 构造函数
     * @param node_name ROS2节点名称
     */
    explicit Go2SensorInterface(const std::string& node_name = "go2_sensor_interface");
    
    /**
     * @brief 析构函数
     */
    ~Go2SensorInterface() override = default;

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
    bool stopSensor(robot_base_interfaces::sensor_interface::SensorType sensor_type) override;
    
    /** 传感器状态查询 */
    robot_base_interfaces::sensor_interface::SensorStatus getSensorStatus(robot_base_interfaces::sensor_interface::SensorType sensor_type) const override;
    
    /** 最新数据获取（通用/点云/IMU） */
    std::shared_ptr<robot_base_interfaces::sensor_interface::SensorData> getLatestData(robot_base_interfaces::sensor_interface::SensorType sensor_type) const override;
    std::shared_ptr<robot_base_interfaces::sensor_interface::PointCloudData> getLatestPointCloud() const override;
    std::shared_ptr<robot_base_interfaces::sensor_interface::IMUData> getLatestIMU() const override;
    
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
     * @brief 获取Livox雷达/IMU等Go2相关配置与状态
     */
    robot_base_interfaces::sensor_interface::Go2SensorConfig getLidarStatus() const;

private:
    // ========== 初始化方法 ==========
    
    /**
     * @brief 初始化激光雷达订阅器
     */
    void initializeLidar();
    
    /**
     * @brief 初始化IMU订阅器
     */
    void initializeIMU();
    
    /**
     * @brief 初始化摄像头订阅器
     */
    void initializeCameras();
    
    /**
     * @brief 初始化机器人状态订阅器（包含关节和足部传感器）
     */
    void initializeRobotState();

    // ========== 数据回调函数 ==========
    
    /**
     * @brief Livox激光雷达点云数据回调
     * @param msg 点云消息
     * 
     * 处理来自Livox Mid360的3D点云数据
     */
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    
    /**
     * @brief IMU数据回调
     * @param msg IMU消息
     * 
     * 处理6轴IMU的加速度计和陀螺仪数据
     */
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    
    /**
     * @brief RGB摄像头数据回调
     * @param msg 图像消息
     */
    void rgbCameraCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    
    /**
     * @brief 深度摄像头数据回调
     * @param msg 图像消息
     */
    void depthCameraCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    
    /**
     * @brief 机器人低层状态回调
     * @param msg 低层状态消息
     * 
     * 包含关节角度、扭矩、足部接触等详细传感器数据
     */
    void robotStateCallback(const unitree_go::msg::LowState::SharedPtr msg);

    // ========== 数据处理方法 ==========
    
    /**
     * @brief 转换点云数据格式
     * @param ros_cloud ROS点云消息
     * @return PointCloudData 统一格式的点云数据
     */
    robot_base_interfaces::sensor_interface::PointCloudData convertPointCloud(
        const sensor_msgs::msg::PointCloud2::SharedPtr& ros_cloud) const;
    
    /**
     * @brief 转换IMU数据格式
     * @param ros_imu ROS IMU消息
     * @return IMUData 统一格式的IMU数据
     */
    robot_base_interfaces::sensor_interface::IMUData convertIMU(
        const sensor_msgs::msg::Imu::SharedPtr& ros_imu) const;
    

    // 注：关节/足端等专用数据类型在当前接口未定义，相关处理留在实现中

private:
    // ========== ROS2通信组件 ==========
    
    // 激光雷达订阅器
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    
    // IMU订阅器
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    
    // 摄像头订阅器
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_camera_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_camera_sub_;
    
    // 机器人状态订阅器
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr robot_state_sub_;

    // ========== 数据缓存 ==========
    
    // 最新传感器数据缓存（线程安全）
    mutable std::mutex sensor_data_mutex_;
    
    // 各传感器的最新数据
    sensor_msgs::msg::PointCloud2::SharedPtr latest_lidar_data_;
    sensor_msgs::msg::Imu::SharedPtr latest_imu_data_;
    sensor_msgs::msg::Image::SharedPtr latest_rgb_image_;
    sensor_msgs::msg::Image::SharedPtr latest_depth_image_;
    unitree_go::msg::LowState::SharedPtr latest_robot_state_;

    // ========== 传感器状态管理 ==========
    
    // 传感器初始化状态
    std::map<robot_base_interfaces::sensor_interface::SensorType, bool> sensor_initialized_;
    
    // 传感器数据时间戳（用于检测数据新鲜度）
    std::map<robot_base_interfaces::sensor_interface::SensorType, rclcpp::Time> sensor_timestamps_;
    
    // 传感器错误计数
    std::map<robot_base_interfaces::sensor_interface::SensorType, int> sensor_error_count_;

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
    bool validateSensorType(robot_base_interfaces::sensor_interface::SensorType sensor_type) const override;
    void updateSensorStatus(robot_base_interfaces::sensor_interface::SensorType sensor_type,
                            robot_base_interfaces::sensor_interface::SensorStatus status) override;
};

} // namespace go2_adapter
} // namespace robot_adapters