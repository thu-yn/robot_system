/**
 * @file   sensor_types.hpp
 * @brief  机器人传感器相关数据类型定义 - 完全适配Go2，预留扩展
 * @author Yang Nan
 * @date   2025-09-11
 */

#ifndef ROBOT_BASE_INTERFACES__SENSOR_INTERFACE__SENSOR_TYPES_HPP_
#define ROBOT_BASE_INTERFACES__SENSOR_INTERFACE__SENSOR_TYPES_HPP_

#include <vector>
#include <string>
#include <memory>
#include <cstdint>
#include <map>

namespace robot_base_interfaces {
namespace sensor_interface {

/**
 * @brief 传感器类型枚举
 */
enum class SensorType {
    // Go2主要传感器
    LIDAR_3D     = 0,   ///< 3D激光雷达 (Livox Mid360)
    LIDAR_2D     = 1,   ///< 2D激光雷达 (预留)
    IMU          = 2,   ///< 惯性测量单元
    CAMERA_RGB   = 3,   ///< RGB摄像头 (预留)
    CAMERA_DEPTH = 4,   ///< 深度摄像头 (预留)
    ULTRASONIC   = 5,   ///< 超声波传感器 (预留)
    
    // 其他机器人可能的传感器类型
    GPS           = 10,     ///< GPS (户外机器人)
    WHEEL_ENCODER = 11,     ///< 轮式编码器
    FORCE_SENSOR  = 12,     ///< 力传感器
    CUSTOM        = 100     ///< 自定义传感器类型
};

/**
 * @brief 传感器状态枚举
 */
enum class SensorStatus {
    UNKNOWN      = 0,   ///< 未知状态
    INITIALIZING = 1,   ///< 初始化中
    ACTIVE       = 2,   ///< 正常工作
    ERROR        = 3,   ///< 错误状态
    CALIBRATING  = 4,   ///< 校准中
    DISCONNECTED = 5    ///< 断开连接
};

/**
 * @brief 3D点结构
 */
struct Point3D {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    float intensity = 0.0f;  ///< 强度信息 (激光雷达用)

    Point3D() = default;
    Point3D(float x_val, float y_val, float z_val, float intensity_val = 0.0f)
        : x(x_val), y(y_val), z(z_val), intensity(intensity_val) {}
};

/**
 * @brief 点云数据 - 对应ROS2 PointCloud2
 */
struct PointCloudData {
    std::vector<Point3D> points;            ///< 点云数据
    std::string frame_id  = "base_link";    ///< 坐标系ID
    uint64_t timestamp_ns = 0;              ///< 时间戳 (nanoseconds)
    uint32_t width        = 0;              ///< 点云宽度 (有序点云用)
    uint32_t height       = 1;              ///< 点云高度 (有序点云用)
    bool is_dense = true;                   ///< 是否稠密点云
    
    /**
     * @brief 获取点云大小
     */
    size_t size() const { return points.size(); }
    
    /**
     * @brief 清空点云数据
     */
    void clear() { points.clear(); width = 0; }
};

/**
 * @brief 2D激光扫描数据 - 对应ROS2 LaserScan
 */
struct LaserScanData {
    std::vector<float> ranges;       ///< 距离数据 (m)
    std::vector<float> intensities;  ///< 强度数据
    float angle_min = 0.0f;          ///< 最小角度 (rad)
    float angle_max = 0.0f;          ///< 最大角度 (rad)
    float angle_increment = 0.0f;    ///< 角度增量 (rad)
    float time_increment  = 0.0f;    ///< 时间增量 (s)
    float scan_time = 0.0f;          ///< 扫描时间 (s)
    float range_min = 0.0f;          ///< 最小有效距离 (m)
    float range_max = 0.0f;          ///< 最大有效距离 (m)
    std::string frame_id = "laser";  ///< 坐标系ID
    uint64_t timestamp_ns = 0;       ///< 时间戳
};

/**
 * @brief IMU数据 - 基于Go2 IMUState设计
 */
struct IMUData {
    // 方向 (四元数)
    struct {
        float w = 1.0f;
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    } orientation;
    
    // 角速度 (rad/s)
    struct {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    } angular_velocity;
    
    // 线性加速度 (m/s^2)
    struct {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    } linear_acceleration;
    
    // RPY角度 (rad) - Go2特有
    struct {
        float roll = 0.0f;
        float pitch = 0.0f;
        float yaw = 0.0f;
    } rpy;
    
    // 协方差矩阵 (9x9, 对应ROS2 Imu消息)
    std::vector<float> orientation_covariance;          // 9个元素
    std::vector<float> angular_velocity_covariance;     // 9个元素
    std::vector<float> linear_acceleration_covariance;  // 9个元素
    
    int8_t temperature = 0;          ///< IMU温度 (°C, Go2特有)
    std::string frame_id = "imu";    ///< 坐标系ID
    uint64_t timestamp_ns = 0;       ///< 时间戳
    
    IMUData() {
        // 初始化协方差矩阵为9个元素
        orientation_covariance.resize(9, -1.0f);        // -1表示未知
        angular_velocity_covariance.resize(9, -1.0f);
        linear_acceleration_covariance.resize(9, -1.0f);
    }
};

/**
 * @brief 图像数据 (预留扩展)
 */
struct ImageData {
    std::vector<uint8_t> data;          ///< 图像原始数据
    uint32_t width  = 0;                ///< 图像宽度
    uint32_t height = 0;                ///< 图像高度
    uint32_t step   = 0;                ///< 行字节数
    std::string encoding = "rgb8";      ///< 编码格式
    std::string frame_id = "camera";    ///< 坐标系ID
    uint64_t timestamp_ns = 0;          ///< 时间戳
    
    /**
     * @brief 获取图像数据大小
     */
    size_t size() const { return data.size(); }
};

/**
 * @brief 传感器信息结构
 */
struct SensorInfo {
    SensorType type;                                ///< 传感器类型
    std::string name;                               ///< 传感器名称
    std::string topic_name;                         ///< 对应的ROS2话题名
    std::string frame_id;                           ///< 坐标系ID
    float frequency = 0.0f;                         ///< 数据频率 (Hz)
    SensorStatus status = SensorStatus::UNKNOWN;    ///< 传感器状态
    
    // 传感器特定参数
    std::map<std::string, float> parameters;
    
    SensorInfo() = default;
    SensorInfo(SensorType t, const std::string& n, const std::string& topic)
        : type(t), name(n), topic_name(topic) {}
};

/**
 * @brief Go2传感器配置 - 基于实际硬件
 */
struct Go2SensorConfig {
    // Livox Mid360激光雷达配置
    struct {
        std::string topic    = "/utlidar/cloud";
        std::string frame_id = "utlidar_lidar";
        float max_range  = 40.0f;   ///< 最大测距 (m)
        float min_range  = 0.1f;    ///< 最小测距 (m)
        float resolution = 0.01f;   ///< 分辨率 (m)
        float frequency  = 10.0f;   ///< 频率 (Hz)
    } lidar;
    
    // IMU配置
    struct {
        std::string frame_id = "imu_link";
        float frequency  = 100.0f;  ///< 频率 (Hz)
        bool provide_rpy = true;    ///< 是否提供RPY角度
    } imu;
    
    // 预留其他传感器配置
    // struct {
    //     std::string topic = "/camera/image_raw";
    //     std::string frame_id = "camera_link";
    //     float frequency = 30.0f;
    // } camera;  // 未来扩展用
};

/**
 * @brief 传感器校准数据
 */
struct CalibrationData {
    SensorType sensor_type;
    std::map<std::string, float> calibration_parameters;
    std::vector<float> transformation_matrix;  ///< 4x4变换矩阵 (16个元素)
    bool is_valid = false;
    uint64_t calibration_time = 0;
    
    CalibrationData() {
        transformation_matrix.resize(16, 0.0f);
        // 初始化为单位矩阵
        transformation_matrix[0] = transformation_matrix[5] = 
        transformation_matrix[10] = transformation_matrix[15] = 1.0f;
    }
};

/**
 * @brief 传感器数据联合体 - 用于统一数据接口
 */
struct SensorData {
    SensorType type;
    uint64_t timestamp_ns = 0;
    
    // 数据指针 (使用智能指针管理内存)
    std::shared_ptr<PointCloudData> point_cloud;
    std::shared_ptr<LaserScanData>  laser_scan;
    std::shared_ptr<IMUData>        imu;
    std::shared_ptr<ImageData>      image;
    
    // 通用数据存储 (用于自定义传感器)
    std::map<std::string, float>        numeric_data;
    std::map<std::string, std::string>  string_data;
    std::vector<uint8_t> raw_data;
    
    SensorData() = default;
    SensorData(SensorType t) : type(t) {}
    
    /**
     * @brief 检查是否包含有效数据
     */
    bool hasData() const {
        return point_cloud || laser_scan || imu || image || !raw_data.empty();
    }
    
    /**
     * @brief 清空所有数据
     */
    void clear() {
        point_cloud.reset();
        laser_scan.reset();
        imu.reset();
        image.reset();
        numeric_data.clear();
        string_data.clear();
        raw_data.clear();
    }
};

} // namespace sensor_interface
} // namespace robot_base_interfaces

#endif // ROBOT_BASE_INTERFACES__SENSOR_INTERFACE__SENSOR_TYPES_HPP_