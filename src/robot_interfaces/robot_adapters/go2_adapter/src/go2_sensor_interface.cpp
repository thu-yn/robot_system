/**
 * @file go2_sensor_interface.cpp
 * @brief Go2 sensor interface complete implementation
 */

#include "robot_adapters/go2_adapter/go2_sensor_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>

namespace robot_adapters {
namespace go2_adapter {

Go2SensorInterface::Go2SensorInterface(const std::string& node_name)
    : rclcpp::Node(node_name) {
    RCLCPP_INFO(this->get_logger(), "Go2 Sensor Interface starting up...");
    
    // 初始化传感器状态
    sensor_initialized_[robot_base_interfaces::sensor_interface::SensorType::LIDAR_3D] = false;
    sensor_initialized_[robot_base_interfaces::sensor_interface::SensorType::IMU] = false;
    sensor_initialized_[robot_base_interfaces::sensor_interface::SensorType::CAMERA_RGB] = false;
    sensor_initialized_[robot_base_interfaces::sensor_interface::SensorType::CAMERA_DEPTH] = false;
}

bool Go2SensorInterface::initialize() {
    RCLCPP_INFO(this->get_logger(), "Initializing Go2 sensor interface...");
    
    try {
        initializeLidar();
        initializeIMU();
        initializeCameras();
        initializeRobotState();
        
        RCLCPP_INFO(this->get_logger(), "Go2 sensor interface initialized successfully");
        return true;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize Go2 sensor interface: %s", e.what());
        return false;
    }
}

bool Go2SensorInterface::shutdown() {
    RCLCPP_INFO(this->get_logger(), "Shutting down Go2 sensor interface...");
    
    // 重置所有订阅器
    lidar_sub_.reset();
    imu_sub_.reset();
    rgb_camera_sub_.reset();
    depth_camera_sub_.reset();
    robot_state_sub_.reset();
    
    // 清空数据缓存
    {
        std::lock_guard<std::mutex> lock(sensor_data_mutex_);
        latest_lidar_data_.reset();
        latest_imu_data_.reset();
        latest_rgb_image_.reset();
        latest_depth_image_.reset();
        latest_robot_state_.reset();
    }
    
    RCLCPP_INFO(this->get_logger(), "Go2 sensor interface shut down successfully");
    return true;
}

std::vector<robot_base_interfaces::sensor_interface::SensorInfo> 
Go2SensorInterface::getAvailableSensors() const {
    using SensorType = robot_base_interfaces::sensor_interface::SensorType;
    using SensorInfo = robot_base_interfaces::sensor_interface::SensorInfo;
    using SensorStatus = robot_base_interfaces::sensor_interface::SensorStatus;
    
    std::vector<SensorInfo> sensors;
    
    // Livox Mid360 激光雷达
    SensorInfo lidar_info(SensorType::LIDAR_3D, "Livox Mid360", "/utlidar/cloud");
    lidar_info.frame_id = "utlidar_lidar";
    lidar_info.frequency = LIDAR_FREQUENCY;
    lidar_info.status = sensor_initialized_.at(SensorType::LIDAR_3D) ? 
                       SensorStatus::ACTIVE : SensorStatus::UNKNOWN;
    lidar_info.parameters["max_range"] = LIDAR_MAX_RANGE;
    lidar_info.parameters["min_range"] = LIDAR_MIN_RANGE;
    lidar_info.parameters["resolution"] = LIDAR_RESOLUTION;
    sensors.push_back(lidar_info);
    
    // IMU传感器
    SensorInfo imu_info(SensorType::IMU, "Go2 IMU", "/imu/data");
    imu_info.frame_id = "imu_link";
    imu_info.frequency = IMU_FREQUENCY;
    imu_info.status = sensor_initialized_.at(SensorType::IMU) ? 
                     SensorStatus::ACTIVE : SensorStatus::UNKNOWN;
    sensors.push_back(imu_info);
    
    // RGB摄像头 (预留)
    SensorInfo rgb_info(SensorType::CAMERA_RGB, "Front RGB Camera", "/camera/image_raw");
    rgb_info.frame_id = "camera_link";
    rgb_info.frequency = CAMERA_FPS;
    rgb_info.status = SensorStatus::DISCONNECTED;  // 当前未连接
    sensors.push_back(rgb_info);
    
    return sensors;
}

bool Go2SensorInterface::isSensorAvailable(robot_base_interfaces::sensor_interface::SensorType sensor_type) const {
    switch (sensor_type) {
        case robot_base_interfaces::sensor_interface::SensorType::LIDAR_3D:
        case robot_base_interfaces::sensor_interface::SensorType::IMU:
            return true;
        case robot_base_interfaces::sensor_interface::SensorType::CAMERA_RGB:
        case robot_base_interfaces::sensor_interface::SensorType::CAMERA_DEPTH:
            return false;  // 当前版本不支持
        default:
            return false;
    }
}

bool Go2SensorInterface::startSensor(robot_base_interfaces::sensor_interface::SensorType sensor_type) {
    RCLCPP_INFO(this->get_logger(), "Starting sensor: %d", static_cast<int>(sensor_type));
    // Go2的传感器通常是自动启动的，这里主要是状态管理
    return isSensorAvailable(sensor_type);
}

bool Go2SensorInterface::stopSensor(robot_base_interfaces::sensor_interface::SensorType sensor_type) {
    RCLCPP_INFO(this->get_logger(), "Stopping sensor: %d", static_cast<int>(sensor_type));
    // Go2的传感器通常不能手动停止，这里主要是状态管理
    return true;
}

robot_base_interfaces::sensor_interface::SensorStatus 
Go2SensorInterface::getSensorStatus(robot_base_interfaces::sensor_interface::SensorType sensor_type) const {
    using SensorStatus = robot_base_interfaces::sensor_interface::SensorStatus;
    
    if (!isSensorAvailable(sensor_type)) {
        return SensorStatus::DISCONNECTED;
    }
    
    auto it = sensor_initialized_.find(sensor_type);
    if (it != sensor_initialized_.end() && it->second) {
        // 检查数据新鲜度
        auto timestamp_it = sensor_timestamps_.find(sensor_type);
        if (timestamp_it != sensor_timestamps_.end()) {
            auto now = rclcpp::Clock().now();
            auto data_age = now - timestamp_it->second;
            if (data_age.seconds() > 1.0) {  // 超过1秒认为数据不新鲜
                return SensorStatus::ERROR;
            }
        }
        return SensorStatus::ACTIVE;
    }
    
    return SensorStatus::UNKNOWN;
}

std::shared_ptr<robot_base_interfaces::sensor_interface::SensorData> 
Go2SensorInterface::getLatestData(robot_base_interfaces::sensor_interface::SensorType sensor_type) const {
    using SensorType = robot_base_interfaces::sensor_interface::SensorType;
    using SensorData = robot_base_interfaces::sensor_interface::SensorData;
    
    auto data = std::make_shared<SensorData>(sensor_type);
    
    std::lock_guard<std::mutex> lock(sensor_data_mutex_);
    
    switch (sensor_type) {
        case SensorType::LIDAR_3D:
            if (latest_lidar_data_) {
                data->point_cloud = std::make_shared<robot_base_interfaces::sensor_interface::PointCloudData>(
                    convertPointCloud(latest_lidar_data_));
                data->timestamp_ns = data->point_cloud->timestamp_ns;
            }
            break;
            
        case SensorType::IMU:
            if (latest_imu_data_) {
                data->imu = std::make_shared<robot_base_interfaces::sensor_interface::IMUData>(
                    convertIMU(latest_imu_data_));
                data->timestamp_ns = data->imu->timestamp_ns;
            }
            break;
            
        default:
            return nullptr;
    }
    
    return data->hasData() ? data : nullptr;
}

std::shared_ptr<robot_base_interfaces::sensor_interface::PointCloudData> 
Go2SensorInterface::getLatestPointCloud() const {
    std::lock_guard<std::mutex> lock(sensor_data_mutex_);
    if (latest_lidar_data_) {
        return std::make_shared<robot_base_interfaces::sensor_interface::PointCloudData>(
            convertPointCloud(latest_lidar_data_));
    }
    return nullptr;
}

std::shared_ptr<robot_base_interfaces::sensor_interface::IMUData> 
Go2SensorInterface::getLatestIMU() const {
    std::lock_guard<std::mutex> lock(sensor_data_mutex_);
    if (latest_imu_data_) {
        return std::make_shared<robot_base_interfaces::sensor_interface::IMUData>(
            convertIMU(latest_imu_data_));
    }
    return nullptr;
}

void Go2SensorInterface::setPointCloudCallback(
    std::function<void(const std::shared_ptr<robot_base_interfaces::sensor_interface::PointCloudData>&)> callback) {
    (void)callback;
    // 这里可以存储回调函数，在数据到达时调用
    // 暂时简化实现
}

void Go2SensorInterface::setIMUCallback(
    std::function<void(const std::shared_ptr<robot_base_interfaces::sensor_interface::IMUData>&)> callback) {
    (void)callback;
    // 这里可以存储回调函数，在数据到达时调用
    // 暂时简化实现
}

void Go2SensorInterface::setSensorCallback(
    robot_base_interfaces::sensor_interface::SensorType sensor_type,
    std::function<void(const std::shared_ptr<robot_base_interfaces::sensor_interface::SensorData>&)> callback) {
    (void)sensor_type;
    (void)callback;
    // 通用传感器回调设置
    // 暂时简化实现
}

// 传感器参数配置方法的基础实现
bool Go2SensorInterface::setSensorParameter(robot_base_interfaces::sensor_interface::SensorType sensor_type,
                                           const std::string& parameter_name,
                                           float value) {
    (void)sensor_type;
    (void)parameter_name;
    (void)value;
    // Go2的传感器参数通常是固定的，这里返回false表示不支持修改
    return false;
}

bool Go2SensorInterface::getSensorParameter(robot_base_interfaces::sensor_interface::SensorType sensor_type,
                                           const std::string& parameter_name,
                                           float& value) const {
    using SensorType = robot_base_interfaces::sensor_interface::SensorType;
    
    if (sensor_type == SensorType::LIDAR_3D) {
        if (parameter_name == "max_range") {
            value = LIDAR_MAX_RANGE;
            return true;
        } else if (parameter_name == "min_range") {
            value = LIDAR_MIN_RANGE;
            return true;
        } else if (parameter_name == "frequency") {
            value = LIDAR_FREQUENCY;
            return true;
        }
    }
    
    return false;
}

bool Go2SensorInterface::setSensorFrequency(robot_base_interfaces::sensor_interface::SensorType sensor_type,
                                           float frequency) {
    (void)sensor_type;
    (void)frequency;
    // Go2的传感器频率通常是固定的
    return false;
}

bool Go2SensorInterface::calibrateSensor(robot_base_interfaces::sensor_interface::SensorType sensor_type) {
    (void)sensor_type;
    // 基础实现：Go2的传感器校准通常在出厂时完成
    return false;
}

robot_base_interfaces::sensor_interface::CalibrationData 
Go2SensorInterface::getCalibrationData(robot_base_interfaces::sensor_interface::SensorType sensor_type) const {
    robot_base_interfaces::sensor_interface::CalibrationData calib_data;
    calib_data.sensor_type = sensor_type;
    calib_data.is_valid = false;  // 当前版本不提供校准数据
    return calib_data;
}

bool Go2SensorInterface::setCalibrationData(const robot_base_interfaces::sensor_interface::CalibrationData& calibration_data) {
    (void)calibration_data;
    // 当前版本不支持设置校准数据
    return false;
}

std::vector<float> Go2SensorInterface::getSensorTransform(robot_base_interfaces::sensor_interface::SensorType sensor_type) const {
    (void)sensor_type;
    std::vector<float> transform(16, 0.0f);
    // 初始化为单位矩阵
    transform[0] = transform[5] = transform[10] = transform[15] = 1.0f;
    
    // 这里可以根据实际的传感器安装位置设置变换矩阵
    // 当前返回单位矩阵
    return transform;
}

bool Go2SensorInterface::setSensorTransform(robot_base_interfaces::sensor_interface::SensorType sensor_type,
                                           const std::vector<float>& transform) {
    (void)sensor_type;
    (void)transform;
    // 当前版本不支持动态设置变换矩阵
    return false;
}

std::map<robot_base_interfaces::sensor_interface::SensorType, robot_base_interfaces::sensor_interface::SensorStatus> 
Go2SensorInterface::getSensorHealth() const {
    using SensorType = robot_base_interfaces::sensor_interface::SensorType;
    using SensorStatus = robot_base_interfaces::sensor_interface::SensorStatus;
    
    std::map<SensorType, SensorStatus> health;
    health[SensorType::LIDAR_3D] = getSensorStatus(SensorType::LIDAR_3D);
    health[SensorType::IMU] = getSensorStatus(SensorType::IMU);
    
    return health;
}

std::string Go2SensorInterface::getSensorError(robot_base_interfaces::sensor_interface::SensorType sensor_type) const {
    // 基础实现：返回空字符串表示无错误
    auto it = sensor_error_count_.find(sensor_type);
    if (it != sensor_error_count_.end() && it->second > 0) {
        return "Sensor data timeout or communication error";
    }
    return "";
}

std::map<std::string, float> Go2SensorInterface::getSensorStatistics(robot_base_interfaces::sensor_interface::SensorType sensor_type) const {
    std::map<std::string, float> stats;
    
    // 基础统计信息
    auto it = sensor_error_count_.find(sensor_type);
    if (it != sensor_error_count_.end()) {
        stats["error_count"] = static_cast<float>(it->second);
    }
    
    return stats;
}

robot_base_interfaces::sensor_interface::Go2SensorConfig Go2SensorInterface::getLidarStatus() const {
    robot_base_interfaces::sensor_interface::Go2SensorConfig config;
    // 返回默认配置
    return config;
}

// 私有方法实现

void Go2SensorInterface::initializeLidar() {
    RCLCPP_INFO(this->get_logger(), "Initializing Livox Mid360 LiDAR...");
    
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/utlidar/cloud",  // Go2 Livox雷达话题
        10,
        std::bind(&Go2SensorInterface::lidarCallback, this, std::placeholders::_1)
    );
    
    sensor_initialized_[robot_base_interfaces::sensor_interface::SensorType::LIDAR_3D] = true;
    RCLCPP_INFO(this->get_logger(), "LiDAR subscription created");
}

void Go2SensorInterface::initializeIMU() {
    RCLCPP_INFO(this->get_logger(), "Initializing IMU...");
    
    // Go2的IMU数据通常包含在SportModeState中，这里创建标准IMU订阅
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data",  // 标准IMU话题
        50,  // IMU频率较高，需要较大的队列
        std::bind(&Go2SensorInterface::imuCallback, this, std::placeholders::_1)
    );
    
    sensor_initialized_[robot_base_interfaces::sensor_interface::SensorType::IMU] = true;
    RCLCPP_INFO(this->get_logger(), "IMU subscription created");
}

void Go2SensorInterface::initializeCameras() {
    RCLCPP_INFO(this->get_logger(), "Initializing Go2 cameras...");
    
    try {
        // 初始化RGB摄像头订阅器 - Go2前置摄像头
        rgb_camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw",  // Go2 RGB摄像头话题
            10,
            std::bind(&Go2SensorInterface::rgbCameraCallback, this, std::placeholders::_1)
        );
        
        // 初始化深度摄像头订阅器 - Go2深度摄像头
        depth_camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_raw",  // Go2深度摄像头话题
            10,
            std::bind(&Go2SensorInterface::depthCameraCallback, this, std::placeholders::_1)
        );
        
        // 标记摄像头已初始化
        sensor_initialized_[robot_base_interfaces::sensor_interface::SensorType::CAMERA_RGB] = true;
        sensor_initialized_[robot_base_interfaces::sensor_interface::SensorType::CAMERA_DEPTH] = true;
        
        RCLCPP_INFO(this->get_logger(), "Go2 cameras initialized successfully");
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize cameras: %s", e.what());
        sensor_initialized_[robot_base_interfaces::sensor_interface::SensorType::CAMERA_RGB] = false;
        sensor_initialized_[robot_base_interfaces::sensor_interface::SensorType::CAMERA_DEPTH] = false;
    }
}

void Go2SensorInterface::initializeRobotState() {
    RCLCPP_INFO(this->get_logger(), "Initializing robot state subscription...");
    
    robot_state_sub_ = this->create_subscription<unitree_go::msg::LowState>(
        "/lowstate",  // Go2低层状态话题
        10,
        std::bind(&Go2SensorInterface::robotStateCallback, this, std::placeholders::_1)
    );
    
    RCLCPP_INFO(this->get_logger(), "Robot state subscription created");
}

void Go2SensorInterface::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lock(sensor_data_mutex_);
        latest_lidar_data_ = msg;
        sensor_timestamps_[robot_base_interfaces::sensor_interface::SensorType::LIDAR_3D] = 
            this->get_clock()->now();
    }
    
    // 可以在这里触发回调函数
    RCLCPP_DEBUG(this->get_logger(), "Received LiDAR data with %d points", 
                 msg->width * msg->height);
}

void Go2SensorInterface::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lock(sensor_data_mutex_);
        latest_imu_data_ = msg;
        sensor_timestamps_[robot_base_interfaces::sensor_interface::SensorType::IMU] = 
            this->get_clock()->now();
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Received IMU data");
}

void Go2SensorInterface::rgbCameraCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lock(sensor_data_mutex_);
        latest_rgb_image_ = msg;
        sensor_timestamps_[robot_base_interfaces::sensor_interface::SensorType::CAMERA_RGB] = 
            this->get_clock()->now();
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Received RGB camera image: %dx%d, encoding: %s", 
                 msg->width, msg->height, msg->encoding.c_str());
    
    // 这里可以触发图像处理回调或发布到其他话题
}

void Go2SensorInterface::depthCameraCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lock(sensor_data_mutex_);
        latest_depth_image_ = msg;
        sensor_timestamps_[robot_base_interfaces::sensor_interface::SensorType::CAMERA_DEPTH] = 
            this->get_clock()->now();
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Received depth camera image: %dx%d, encoding: %s", 
                 msg->width, msg->height, msg->encoding.c_str());
    
    // 这里可以触发深度图像处理回调或发布到其他话题
}

void Go2SensorInterface::robotStateCallback(const unitree_go::msg::LowState::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lock(sensor_data_mutex_);
        latest_robot_state_ = msg;
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Received robot low state data");
}

robot_base_interfaces::sensor_interface::PointCloudData 
Go2SensorInterface::convertPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& ros_cloud) const {
    robot_base_interfaces::sensor_interface::PointCloudData pc_data;
    
    if (!ros_cloud) {
        return pc_data;
    }
    
    pc_data.frame_id = ros_cloud->header.frame_id;
    pc_data.timestamp_ns = ros_cloud->header.stamp.sec * 1e9 + ros_cloud->header.stamp.nanosec;
    pc_data.width = ros_cloud->width;
    pc_data.height = ros_cloud->height;
    pc_data.is_dense = ros_cloud->is_dense;
    
    // 解析点云数据
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*ros_cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*ros_cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*ros_cloud, "z");
    sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*ros_cloud, "intensity");
    
    pc_data.points.reserve(ros_cloud->width * ros_cloud->height);
    
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
        robot_base_interfaces::sensor_interface::Point3D point;
        point.x = *iter_x;
        point.y = *iter_y;
        point.z = *iter_z;
        point.intensity = *iter_intensity;
        pc_data.points.push_back(point);
    }
    
    return pc_data;
}

robot_base_interfaces::sensor_interface::IMUData 
Go2SensorInterface::convertIMU(const sensor_msgs::msg::Imu::SharedPtr& ros_imu) const {
    robot_base_interfaces::sensor_interface::IMUData imu_data;
    
    if (!ros_imu) {
        return imu_data;
    }
    
    imu_data.frame_id = ros_imu->header.frame_id;
    imu_data.timestamp_ns = ros_imu->header.stamp.sec * 1e9 + ros_imu->header.stamp.nanosec;
    
    // 四元数
    imu_data.orientation.w = ros_imu->orientation.w;
    imu_data.orientation.x = ros_imu->orientation.x;
    imu_data.orientation.y = ros_imu->orientation.y;
    imu_data.orientation.z = ros_imu->orientation.z;
    
    // 角速度
    imu_data.angular_velocity.x = ros_imu->angular_velocity.x;
    imu_data.angular_velocity.y = ros_imu->angular_velocity.y;
    imu_data.angular_velocity.z = ros_imu->angular_velocity.z;
    
    // 线性加速度
    imu_data.linear_acceleration.x = ros_imu->linear_acceleration.x;
    imu_data.linear_acceleration.y = ros_imu->linear_acceleration.y;
    imu_data.linear_acceleration.z = ros_imu->linear_acceleration.z;
    
    // 转换为RPY角度 
    double w = ros_imu->orientation.w;
    double x = ros_imu->orientation.x;
    double y = ros_imu->orientation.y;
    double z = ros_imu->orientation.z;
    
    // 四元数到RPY的转换公式
    imu_data.rpy.roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    imu_data.rpy.pitch = std::asin(2.0 * (w * y - z * x));
    imu_data.rpy.yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    
    // 协方差矩阵
    if (ros_imu->orientation_covariance.size() >= 9) {
        std::copy(ros_imu->orientation_covariance.begin(),
                  ros_imu->orientation_covariance.begin() + 9,
                  imu_data.orientation_covariance.begin());
    }
    
    if (ros_imu->angular_velocity_covariance.size() >= 9) {
        std::copy(ros_imu->angular_velocity_covariance.begin(),
                  ros_imu->angular_velocity_covariance.begin() + 9,
                  imu_data.angular_velocity_covariance.begin());
    }
    
    if (ros_imu->linear_acceleration_covariance.size() >= 9) {
        std::copy(ros_imu->linear_acceleration_covariance.begin(),
                  ros_imu->linear_acceleration_covariance.begin() + 9,
                  imu_data.linear_acceleration_covariance.begin());
    }
    
    return imu_data;
}

bool Go2SensorInterface::validateSensorType(robot_base_interfaces::sensor_interface::SensorType sensor_type) const {
    return isSensorAvailable(sensor_type);
}

void Go2SensorInterface::updateSensorStatus(robot_base_interfaces::sensor_interface::SensorType sensor_type,
                                           robot_base_interfaces::sensor_interface::SensorStatus status) {
    // 更新传感器状态的内部实现
    RCLCPP_DEBUG(this->get_logger(), "Sensor %d status updated to %d", 
                 static_cast<int>(sensor_type), static_cast<int>(status));
}

} // namespace go2_adapter
} // namespace robot_adapters