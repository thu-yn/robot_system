/**
 * @file   i_sensor_interface.hpp  
 * @brief  机器人传感器抽象接口 - 完全适配Go2，预留扩展
 * @author Yang Nan
 * @date   2025-09-11
 */

#ifndef ROBOT_BASE_INTERFACES__SENSOR_INTERFACE__I_SENSOR_INTERFACE_HPP_
#define ROBOT_BASE_INTERFACES__SENSOR_INTERFACE__I_SENSOR_INTERFACE_HPP_

#include "robot_base_interfaces/sensor_interface/sensor_types.hpp"
#include "robot_base_interfaces/common/result.hpp"
#include <functional>
#include <memory>
#include <vector>
#include <map>

namespace robot_base_interfaces {
namespace sensor_interface {

/**
 * @brief 传感器接口抽象类
 * 
 * 该接口设计覆盖Go2的所有传感器能力：
 * - Livox Mid360 3D激光雷达
 * - 内置IMU传感器
 * - 预留摄像头和其他传感器扩展接口
 * 
 * 其他机器人可通过实现此接口来适配统一的感知系统
 */
class ISensorInterface {
public:
    virtual ~ISensorInterface() = default;

    // ============= 初始化和配置 =============
    
    /**
     * @brief 初始化传感器接口
     * @return true if successful, false otherwise
     */
    virtual bool initialize() = 0;
    
    /**
     * @brief 关闭传感器接口
     * @return true if successful, false otherwise
     */
    virtual bool shutdown() = 0;
    
    /**
     * @brief 获取可用的传感器列表
     * @return 传感器信息列表
     */
    virtual std::vector<SensorInfo> getAvailableSensors() const = 0;
    
    /**
     * @brief 检查特定传感器是否可用
     * @param sensor_type 传感器类型
     * @return true if available, false otherwise
     */
    virtual bool isSensorAvailable(SensorType sensor_type) const = 0;
    
    // ============= 传感器控制 =============
    
    /**
     * @brief 启动特定传感器
     * @param sensor_type 传感器类型
     * @return true if successful, false otherwise
     */
    virtual bool startSensor(SensorType sensor_type) = 0;
    
    /**
     * @brief 停止特定传感器
     * @param sensor_type 传感器类型
     * @return true if successful, false otherwise
     */
    virtual bool stopSensor(SensorType sensor_type) = 0;
    
    /**
     * @brief 获取传感器状态
     * @param sensor_type 传感器类型
     * @return 传感器状态
     */
    virtual SensorStatus getSensorStatus(SensorType sensor_type) const = 0;
    
    // ============= 数据获取 =============
    
    /**
     * @brief 获取最新的传感器数据
     * @param sensor_type 传感器类型
     * @return 传感器数据，如果无数据返回nullptr
     */
    virtual std::shared_ptr<SensorData> getLatestData(SensorType sensor_type) const = 0;

    /**
     * @brief 获取最新的通用传感器数据（强类型结果）
     */
    virtual robot_base_interfaces::common::Result<std::shared_ptr<SensorData>>
    getLatestDataResult(SensorType sensor_type) const {
        auto data = getLatestData(sensor_type);
        if (data && data->hasData()) {
            return robot_base_interfaces::common::Result<std::shared_ptr<SensorData>>::success(data);
        }
        return robot_base_interfaces::common::Result<std::shared_ptr<SensorData>>::failure(
            robot_base_interfaces::common::ErrorCode::UNAVAILABLE, "no sensor data available");
    }
    
    /**
     * @brief 获取最新的点云数据 - Go2主要传感器
     * @return 点云数据，如果无数据返回nullptr
     */
    virtual std::shared_ptr<PointCloudData> getLatestPointCloud() const = 0;

    /**
     * @brief 获取最新的点云数据（强类型结果）
     */
    virtual robot_base_interfaces::common::Result<std::shared_ptr<PointCloudData>>
    getLatestPointCloudResult() const {
        auto data = getLatestPointCloud();
        if (data) {
            return robot_base_interfaces::common::Result<std::shared_ptr<PointCloudData>>::success(data);
        }
        return robot_base_interfaces::common::Result<std::shared_ptr<PointCloudData>>::failure(
            robot_base_interfaces::common::ErrorCode::UNAVAILABLE, "no point cloud available");
    }
    
    /**
     * @brief 获取最新的IMU数据 - Go2内置传感器
     * @return IMU数据，如果无数据返回nullptr
     */
    virtual std::shared_ptr<IMUData> getLatestIMU() const = 0;

    /**
     * @brief 获取最新的IMU数据（强类型结果）
     */
    virtual robot_base_interfaces::common::Result<std::shared_ptr<IMUData>>
    getLatestIMUResult() const {
        auto data = getLatestIMU();
        if (data) {
            return robot_base_interfaces::common::Result<std::shared_ptr<IMUData>>::success(data);
        }
        return robot_base_interfaces::common::Result<std::shared_ptr<IMUData>>::failure(
            robot_base_interfaces::common::ErrorCode::UNAVAILABLE, "no imu available");
    }
    
    /**
     * @brief 获取最新的2D激光扫描数据 - 扩展接口
     * @return 激光扫描数据，如果无数据返回nullptr
     */
    virtual std::shared_ptr<LaserScanData> getLatestLaserScan() const {
        // 默认实现：不支持2D激光雷达
        return nullptr;
    }

    /**
     * @brief 获取最新的2D激光扫描数据（强类型结果）
     */
    virtual robot_base_interfaces::common::Result<std::shared_ptr<LaserScanData>>
    getLatestLaserScanResult() const {
        auto data = getLatestLaserScan();
        if (data) {
            return robot_base_interfaces::common::Result<std::shared_ptr<LaserScanData>>::success(data);
        }
        return robot_base_interfaces::common::Result<std::shared_ptr<LaserScanData>>::failure(
            robot_base_interfaces::common::ErrorCode::UNAVAILABLE, "no laser scan available");
    }
    
    /**
     * @brief 获取最新的图像数据 - 扩展接口
     * @param camera_id 摄像头ID (0=前置, 1=后置等)
     * @return 图像数据，如果无数据返回nullptr
     */
    virtual std::shared_ptr<ImageData> getLatestImage(int camera_id = 0) const {
        // 默认实现：不支持摄像头
        (void)camera_id; // 避免未使用参数警告
        return nullptr;
    }

    /**
     * @brief 获取最新的图像数据（强类型结果）
     */
    virtual robot_base_interfaces::common::Result<std::shared_ptr<ImageData>>
    getLatestImageResult(int camera_id = 0) const {
        auto data = getLatestImage(camera_id);
        if (data) {
            return robot_base_interfaces::common::Result<std::shared_ptr<ImageData>>::success(data);
        }
        return robot_base_interfaces::common::Result<std::shared_ptr<ImageData>>::failure(
            robot_base_interfaces::common::ErrorCode::UNAVAILABLE, "no image available");
    }
    
    // ============= 数据回调接口 =============
    
    /**
     * @brief 设置点云数据回调函数
     * @param callback 回调函数
     * 线程语义：回调可能在采集或处理线程异步触发；应避免在回调中长时间阻塞，
     * 回调内如需与接口交互请注意潜在重入并采取排队/拷贝转移策略。
     */
    virtual void setPointCloudCallback(
        std::function<void(const std::shared_ptr<PointCloudData>&)> callback) = 0;
    
    /**
     * @brief 设置IMU数据回调函数
     * @param callback 回调函数
     * 线程语义：可能在高频数据线程触发（如100Hz+）；回调应尽量无锁、快速返回，
     * 建议仅做数据搬运或轻量处理。
     */
    virtual void setIMUCallback(
        std::function<void(const std::shared_ptr<IMUData>&)> callback) = 0;
    
    /**
     * @brief 设置2D激光扫描数据回调函数
     * @param callback 回调函数
     * 线程语义：与其他回调一致，可能并行触发；调用方需保证线程安全。
     */
    virtual void setLaserScanCallback(
        std::function<void(const std::shared_ptr<LaserScanData>&)> callback) {
        // 默认实现：不支持2D激光雷达
        (void)callback;
    }
    
    /**
     * @brief 设置图像数据回调函数
     * @param callback  回调函数
     * @param camera_id 摄像头ID
     * 线程语义：相机数据吞吐较大，建议回调内避免拷贝大块数据，采用共享指针或零拷贝。
     */
    virtual void setImageCallback(
        std::function<void(const std::shared_ptr<ImageData>&)> callback,
        int camera_id = 0) {
        // 默认实现：不支持摄像头
        (void)callback;
        (void)camera_id;
    }
    
    /**
     * @brief 设置通用传感器数据回调函数
     * @param sensor_type 传感器类型
     * @param callback    回调函数
     * 线程语义：不同传感器可能在不同线程回调；实现应避免回调之间相互阻塞。
     */
    virtual void setSensorCallback(
        SensorType sensor_type,
        std::function<void(const std::shared_ptr<SensorData>&)> callback) = 0;
    
    // ============= 传感器配置 =============
    
    /**
     * @brief 设置传感器参数
     * @param sensor_type    传感器类型
     * @param parameter_name 参数名
     * @param value 参数值
     * @return true if successful, false otherwise
     */
    virtual bool setSensorParameter(SensorType sensor_type, 
                                  const std::string& parameter_name, 
                                  float value) = 0;
    
    /**
     * @brief 获取传感器参数
     * @param sensor_type    传感器类型
     * @param parameter_name 参数名
     * @param value 输出参数值
     * @return true if successful, false otherwise
     */
    virtual bool getSensorParameter(SensorType sensor_type,
                                  const std::string& parameter_name,
                                  float& value) const = 0;
    
    /**
     * @brief 设置传感器数据频率
     * @param sensor_type 传感器类型
     * @param frequency   目标频率 (Hz)
     * @return true if successful, false otherwise
     */
    virtual bool setSensorFrequency(SensorType sensor_type, float frequency) = 0;
    
    // ============= 校准接口 =============
    
    /**
     * @brief 校准传感器
     * @param sensor_type 传感器类型
     * @return true if successful, false otherwise
     */
    virtual bool calibrateSensor(SensorType sensor_type) = 0;
    
    /**
     * @brief 获取传感器校准数据
     * @param sensor_type 传感器类型
     * @return 校准数据
     */
    virtual CalibrationData getCalibrationData(SensorType sensor_type) const = 0;

    /**
     * @brief 获取传感器校准数据（强类型结果）
     */
    virtual robot_base_interfaces::common::Result<CalibrationData>
    getCalibrationDataResult(SensorType sensor_type) const {
        try {
            auto data = getCalibrationData(sensor_type);
            return robot_base_interfaces::common::Result<CalibrationData>::success(data);
        } catch (const std::exception&) {
            return robot_base_interfaces::common::Result<CalibrationData>::failure(
                robot_base_interfaces::common::ErrorCode::UNAVAILABLE, "calibration data not available");
        }
    }
    
    /**
     * @brief 设置传感器校准数据
     * @param calibration_data 校准数据
     * @return true if successful, false otherwise
     */
    virtual bool setCalibrationData(const CalibrationData& calibration_data) = 0;
    
    // ============= 坐标系变换 =============
    
    /**
     * @brief 获取传感器坐标系到机器人坐标系的变换
     * @param sensor_type 传感器类型
     * @return 4x4变换矩阵 (16个元素)
     */
    virtual std::vector<float> getSensorTransform(SensorType sensor_type) const = 0;
    
    /**
     * @brief 设置传感器坐标系变换
     * @param sensor_type 传感器类型
     * @param transform 4x4变换矩阵 (16个元素)
     * @return true if successful, false otherwise
     */
    virtual bool setSensorTransform(SensorType sensor_type,
                                   const std::vector<float>& transform) = 0;
    
    // ============= 诊断和监控 =============
    
    /**
     * @brief 获取传感器健康状态
     * @return 所有传感器的健康状态
     */
    virtual std::map<SensorType, SensorStatus> getSensorHealth() const = 0;
    
    /**
     * @brief 获取传感器错误信息
     * @param sensor_type 传感器类型
     * @return 错误信息字符串，空字符串表示无错误
     */
    virtual std::string getSensorError(SensorType sensor_type) const = 0;
    
    /**
     * @brief 获取传感器统计信息 (数据频率、丢帧数等)
     * @param sensor_type 传感器类型
     * @return 统计信息
     */
    virtual std::map<std::string, float> getSensorStatistics(SensorType sensor_type) const = 0;
    
    // ============= 扩展接口 =============
    
    /**
     * @brief 执行自定义传感器命令 - 扩展接口
     * @param sensor_type 传感器类型
     * @param command     命令字符串
     * @param parameters  参数 (JSON格式)
     * @return 执行结果 (JSON格式)
     */
    virtual std::string executeCustomCommand(SensorType sensor_type,
                                           const std::string& command,
                                           const std::string& parameters = "") {
        // 默认实现：不支持自定义命令
        (void)sensor_type;
        (void)command;
        (void)parameters;
        return "{\"error\": \"not_supported\"}";
    }
    
    /**
     * @brief 获取传感器接口版本
     * @return 版本字符串
     */
    virtual std::string getVersion() const {
        return "1.0.0";
    }
    
protected:
    /**
     * @brief 验证传感器类型是否支持
     * @param sensor_type 传感器类型
     * @return true if supported, false otherwise
     */
    virtual bool validateSensorType(SensorType sensor_type) const = 0;
    
    /**
     * @brief 更新传感器状态
     * @param sensor_type 传感器类型
     * @param status 新状态
     */
    virtual void updateSensorStatus(SensorType sensor_type, SensorStatus status) = 0;
};

/**
 * @brief 传感器接口智能指针类型定义
 */
using ISensorInterfacePtr = std::shared_ptr<ISensorInterface>;
using ISensorInterfaceUniquePtr = std::unique_ptr<ISensorInterface>;

} // namespace sensor_interface
} // namespace robot_base_interfaces

#endif // ROBOT_BASE_INTERFACES__SENSOR_INTERFACE__I_SENSOR_INTERFACE_HPP_