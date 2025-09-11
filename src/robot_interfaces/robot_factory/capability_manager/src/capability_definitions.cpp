/**
 * @file capability_definitions.cpp
 * @brief 机器人能力定义结构的实现
 * @author Claude Code
 * @date 2024
 */

#include "robot_factory/capability_manager/capability_definitions.hpp"
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <cmath>

namespace robot_factory {
namespace capability_manager {

// ============= 运动能力结构实现 =============

std::string MotionCapabilities::toString() const {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "运动能力:\n";
    ss << "  最大线性速度: " << max_linear_velocity << " m/s\n";
    ss << "  最大角速度: " << max_angular_velocity << " rad/s\n";
    ss << "  最大线加速度: " << max_linear_acceleration << " m/s²\n";    ss << "  最大角加速度: " << max_angular_acceleration << " rad/s²\n";
    ss << "  转弯半径: " << turning_radius << " m\n";
    ss << "  爬楼梯能力: " << (quadruped_specific.can_climb_stairs ? "是" : "否") << "\n";
    ss << "  双腿平衡能力: " << (quadruped_specific.can_balance_on_two_legs ? "是" : "否") << "\n";
    ss << "  运动模式: ";
    for (size_t i = 0; i < locomotion_modes.size(); ++i) {
        ss << locomotion_modes[i];
        if (i < locomotion_modes.size() - 1) ss << ", ";
    }
    ss << "\n";
    
    // 四足机器人特有能力
    ss << "  四足特有能力:\n";
    ss << "    小跑模式: " << (quadruped_specific.can_trot ? "是" : "否") << "\n";
    ss << "    跳跃模式: " << (quadruped_specific.can_bound ? "是" : "否") << "\n";
    ss << "    爬行模式: " << (quadruped_specific.can_crawl ? "是" : "否") << "\n";
    ss << "    最大跨越高度: " << quadruped_specific.max_step_height << " m\n";
    
    return ss.str();
}

bool MotionCapabilities::isValid() const {
    return max_linear_velocity > 0 && 
           max_angular_velocity > 0 && 
           max_linear_acceleration > 0 && 
           turning_radius >= 0 &&
           !locomotion_modes.empty();
}

MotionCapabilities MotionCapabilities::createDefault() {
    MotionCapabilities capabilities;
    capabilities.max_linear_velocity = 1.0;
    capabilities.max_angular_velocity = 1.0;
    capabilities.max_linear_acceleration = 1.0;
    capabilities.max_angular_acceleration = 1.0;
    capabilities.turning_radius = 0.5;
    capabilities.locomotion_modes = {"walk"};
    
    // 四足机器人默认能力
    capabilities.quadruped_specific.can_trot = false;
    capabilities.quadruped_specific.can_bound = false;
    capabilities.quadruped_specific.can_crawl = true;
    capabilities.quadruped_specific.max_step_height = 0.1;
    
    return capabilities;
}

// ============= 传感器能力结构实现 =============

std::string SensorCapabilities::toString() const {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "传感器能力:\n";
    
    // LiDAR能力
    ss << "  LiDAR:\n";
    ss << "    2D LiDAR: " << (lidar.has_2d_lidar ? "是" : "否") << "\n";
    ss << "    3D LiDAR: " << (lidar.has_3d_lidar ? "是" : "否") << "\n";
    ss << "    最大测距: " << lidar.max_range << " m\n";
    ss << "    角分辨率: " << lidar.angular_resolution << " 度\n";
    ss << "    测距精度: " << lidar.accuracy << " m\n";
    ss << "    扫描频率: " << lidar.scan_frequency << " Hz\n";
    
    // 摄像头能力
    ss << "  摄像头:\n";
    ss << "    RGB摄像头数量: " << camera.rgb_camera_count << "\n";
    ss << "    深度摄像头数量: " << camera.depth_camera_count << "\n";
    ss << "    最大分辨率: " << camera.max_resolution << "\n";
    ss << "    最大帧率: " << camera.max_framerate << " fps\n";
    ss << "    立体视觉: " << (camera.has_stereo_vision ? "是" : "否") << "\n";
    
    // IMU能力
    ss << "  IMU:\n";
    ss << "    自由度: " << imu.degrees_of_freedom << "DOF\n";
    ss << "    采样率: " << imu.sampling_rate << " Hz\n";
    ss << "    陀螺仪量程: ±" << imu.gyro_range_dps << " 度/s\n";
    ss << "    加速度计量程: ±" << imu.accel_range_g << " g\n";
    ss << "    磁力计: " << (imu.has_magnetometer ? "是" : "否") << "\n";
    
    // 其他传感器
    ss << "  其他传感器:\n";
    ss << "    GPS: " << (has_gps ? "是" : "否") << "\n";
    ss << "    编码器: " << (has_encoders ? "是" : "否") << "\n";
    ss << "    力传感器: " << (has_force_sensors ? "是" : "否") << "\n";
    ss << "    温度传感器: " << (has_temperature_sensors ? "是" : "否") << "\n";
    
    return ss.str();
}

bool SensorCapabilities::isValid() const {
    // 至少需要一种主要传感器
    bool has_main_sensor = lidar.has_2d_lidar || 
                          lidar.has_3d_lidar || 
                          camera.rgb_camera_count > 0 || 
                          camera.depth_camera_count > 0;
    
    return has_main_sensor && 
           imu.degrees_of_freedom >= 6 && 
           imu.sampling_rate > 0;
}

SensorCapabilities SensorCapabilities::createDefault() {
    SensorCapabilities capabilities;
    
    // 默认LiDAR配置
    capabilities.lidar.has_2d_lidar = false;
    capabilities.lidar.has_3d_lidar = false;
    capabilities.lidar.max_range = 10.0;
    capabilities.lidar.angular_resolution = 1.0;
    capabilities.lidar.accuracy = 0.1;
    capabilities.lidar.scan_frequency = 10;
    
    // 默认摄像头配置
    capabilities.camera.rgb_camera_count = 1;
    capabilities.camera.depth_camera_count = 0;
    capabilities.camera.max_resolution = "640x480";
    capabilities.camera.max_framerate = 30;
    capabilities.camera.has_stereo_vision = false;
    
    // 默认IMU配置
    capabilities.imu.degrees_of_freedom = 6;
    capabilities.imu.sampling_rate = 100;
    capabilities.imu.gyro_range_dps = 250.0;
    capabilities.imu.accel_range_g = 2.0;
    capabilities.imu.has_magnetometer = false;
    
    // 其他传感器默认配置
    capabilities.has_gps = false;
    capabilities.has_encoders = true;
    capabilities.has_force_sensors = false;
    capabilities.has_temperature_sensors = false;
    
    return capabilities;
}

// ============= 电源能力结构实现 =============

std::string PowerCapabilities::toString() const {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(1);
    ss << "电源能力:\n";
    ss << "  电池容量: " << battery_capacity_mah << " mAh\n";
    ss << "  工作电压: " << battery_voltage << " V\n";
    ss << "  充电类型: " << "contact" << "\n"; // simplified
    ss << "  充电时间: " << "120" << " 分钟\n"; // simplified
    ss << "  预计运行时间: " << estimated_runtime_hours << " 小时\n";
    ss << "  热插拔支持: " << (supports_hot_swap ? "是" : "否") << "\n";
    ss << "  电源管理特性:\n";
    ss << "    电源监控: " << (has_power_monitoring ? "是" : "否") << "\n";
    ss << "    低功耗模式: " << (has_low_power_mode ? "是" : "否") << "\n";
    ss << "    休眠模式: " << (has_hibernation_mode ? "是" : "否") << "\n";
    ss << "    剩余时间估算: " << (can_estimate_remaining_time ? "是" : "否") << "\n";
    
    return ss.str();
}

bool PowerCapabilities::isValid() const {
    return battery_capacity_mah > 0 && 
           battery_voltage > 0 && 
           estimated_runtime_hours > 0;
}

PowerCapabilities PowerCapabilities::createDefault() {
    PowerCapabilities capabilities;
    capabilities.battery_capacity_mah = 5000.0;    // 5Ah默认容量
    capabilities.battery_voltage = 12.0;           // 12V默认电压
    capabilities.estimated_runtime_hours = 2.0;    // 2小时运行时间
    capabilities.supports_hot_swap = false;
    
    // 默认电源管理特性
    capabilities.has_power_monitoring = true;
    capabilities.has_low_power_mode = false;
    capabilities.has_hibernation_mode = false;
    capabilities.can_estimate_remaining_time = true;
    
    return capabilities;
}

// NOTE: Removed calculateEnergyDensity method as it's not declared in header

// ============= 通信能力结构实现 =============

std::string CommunicationCapabilities::toString() const {
    std::stringstream ss;
    ss << "通信能力:\n";
    ss << "  支持的网络: ";
    for (size_t i = 0; i < network.supported_networks.size(); ++i) {
        ss << network.supported_networks[i];
        if (i < network.supported_networks.size() - 1) ss << ", ";
    }
    ss << "\n";
    
    ss << "  支持的协议: ";
    for (size_t i = 0; i < network.supported_protocols.size(); ++i) {
        ss << network.supported_protocols[i];
        if (i < network.supported_protocols.size() - 1) ss << ", ";
    }
    ss << "\n";
    
    ss << "  蓝牙: " << (network.has_bluetooth ? "是" : "否") << "\n";
    ss << "  遥控支持: " << (network.has_remote_control ? "是" : "否") << "\n";
    
    ss << "  ROS2通信:\n";
    ss << "    默认RMW实现: " << ros2.default_rmw_implementation << "\n";
    ss << "    支持的RMW: ";
    for (size_t i = 0; i < ros2.supported_rmw.size(); ++i) {
        ss << ros2.supported_rmw[i];
        if (i < ros2.supported_rmw.size() - 1) ss << ", ";
    }
    ss << "\n";
    ss << "    多域支持: " << (ros2.supports_multi_domain ? "是" : "否") << "\n";
    
    ss << "  DDS配置:\n";
    ss << "    首选DDS: " << ros2.preferred_dds << "\n";
    ss << "    最大参与者: " << ros2.max_participants << "\n";
    ss << "    安全支持: " << (ros2.supports_security ? "是" : "否") << "\n";
    
    return ss.str();
}

bool CommunicationCapabilities::isValid() const {
    return !network.supported_networks.empty() && 
           !network.supported_protocols.empty() &&
           !ros2.default_rmw_implementation.empty() &&
           !ros2.supported_rmw.empty() &&
           ros2.max_participants > 0;
}

CommunicationCapabilities CommunicationCapabilities::createDefault() {
    CommunicationCapabilities capabilities;
    
    // 网络通信默认配置
    capabilities.network.supported_networks = {"ethernet", "wifi"};
    capabilities.network.supported_protocols = {"tcp", "udp"};
    capabilities.network.has_bluetooth = false;
    capabilities.network.has_remote_control = false;
    
    // ROS2通信默认配置
    capabilities.ros2.default_rmw_implementation = "rmw_fastrtps_cpp";
    capabilities.ros2.supported_rmw = {"rmw_fastrtps_cpp", "rmw_cyclonedds_cpp"};
    capabilities.ros2.supports_multi_domain = true;
    
    // DDS配置默认值
    capabilities.ros2.preferred_dds = "fastrtps";
    capabilities.ros2.max_participants = 50;
    capabilities.ros2.supports_security = false;
    
    return capabilities;
}

// ============= 计算能力结构实现 =============

std::string ProcessingCapabilities::toString() const {
    std::stringstream ss;
    ss << "计算能力:\n";
    ss << "  硬件规格:\n";
    ss << "    CPU架构: " << cpu_architecture << "\n";
    ss << "    CPU核心数: " << cpu_cores << "\n";
    ss << "    内存: " << ram_gb << " GB\n";
    ss << "    存储: " << storage_gb << " GB\n";
    ss << "    GPU加速: " << (has_gpu ? "是" : "否") << "\n";
    
    ss << "  AI/ML支持:\n";
    ss << "    TensorFlow: " << (aiml.supports_tensorflow ? "是" : "否") << "\n";
    ss << "    PyTorch: " << (aiml.supports_pytorch ? "是" : "否") << "\n";
    ss << "    TensorRT: " << (aiml.supports_tensorrt ? "是" : "否") << "\n";
    ss << "    OpenVINO: " << (aiml.supports_openvino ? "是" : "否") << "\n";
    
    ss << "  实时性能:\n";
    ss << "    最大控制频率: " << max_control_frequency << " Hz\n";
    ss << "    典型延迟: " << typical_latency_ms << " ms\n";
    
    return ss.str();
}

bool ProcessingCapabilities::isValid() const {
    return cpu_cores > 0 && 
           ram_gb > 0 && 
           storage_gb > 0 &&
           !cpu_architecture.empty() &&
           max_control_frequency > 0 &&
           typical_latency_ms >= 0;
}

ProcessingCapabilities ProcessingCapabilities::createDefault() {
    ProcessingCapabilities capabilities;
    
    // 硬件规格默认值
    capabilities.cpu_architecture = "x86_64";
    capabilities.cpu_cores = 2;
    capabilities.ram_gb = 4;
    capabilities.storage_gb = 32;
    capabilities.has_gpu = false;
    
    // AI/ML支持默认配置
    capabilities.aiml.supports_tensorflow = false;
    capabilities.aiml.supports_pytorch = false;
    capabilities.aiml.supports_tensorrt = false;
    capabilities.aiml.supports_openvino = false;
    
    // 实时性能默认值
    capabilities.max_control_frequency = 100.0;    // 100Hz控制频率
    capabilities.typical_latency_ms = 10.0;        // 10ms典型延迟
    
    return capabilities;
}

// NOTE: Removed calculatePerformanceScore method as it's not declared in header

// ============= 物理属性结构实现 =============

std::string PhysicalProperties::toString() const {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3);
    ss << "物理属性:\n";
    ss << "  尺寸 (L×W×H): " << length << "×" << width << "×" << height << " m\n";
    ss << "  重量: " << weight << " kg\n";
    ss << "  最大负载: " << max_payload << " kg\n";
    
    ss << "  环境适应性:\n";
    ss << "    工作温度范围: " << environmental.min_operating_temperature << "°C ~ " 
       << environmental.max_operating_temperature << "°C\n";
    ss << "    IP防护等级: IP" << environmental.ip_rating << "\n";
    ss << "    防水: " << (environmental.is_waterproof ? "是" : "否") << "\n";
    ss << "    防尘: " << (environmental.is_dustproof ? "是" : "否") << "\n";
    
    ss << "  安装点: ";
    for (size_t i = 0; i < mounting_points.size(); ++i) {
        ss << mounting_points[i];
        if (i < mounting_points.size() - 1) ss << ", ";
    }
    ss << "\n";
    
    return ss.str();
}

bool PhysicalProperties::isValid() const {
    return length > 0 && 
           width > 0 && 
           height > 0 && 
           weight > 0 &&
           max_payload >= 0 &&
           environmental.min_operating_temperature < environmental.max_operating_temperature;
}

PhysicalProperties PhysicalProperties::createDefault() {
    PhysicalProperties properties;
    
    // 默认尺寸 (小型机器人)
    properties.length = 0.5;
    properties.width = 0.3;
    properties.height = 0.2;
    properties.weight = 5.0;
    properties.max_payload = 1.0;
    
    // 默认环境适应性
    properties.environmental.min_operating_temperature = -10.0;
    properties.environmental.max_operating_temperature = 45.0;
    properties.environmental.ip_rating = 44;        // IP44防护等级
    properties.environmental.is_waterproof = false;
    properties.environmental.is_dustproof = true;
    
    // 默认安装点
    properties.mounting_points = {"top_mount"};
    
    return properties;
}

// NOTE: Removed undeclared methods

// ============= 综合能力结构实现 =============

std::string RobotCapabilities::toString() const {
    std::stringstream ss;
    ss << "=== " << robot_model << " 机器人能力报告 ===\n";
    ss << "制造商: " << manufacturer << "\n";
    ss << "固件版本: " << firmware_version << "\n";
    ss << "配置版本: " << profile_version << "\n";
    ss << "最后更新: " << last_updated << "\n";
    ss << "\n";
    
    // 各项能力详细信息
    ss << motion.toString() << "\n";
    ss << sensors.toString() << "\n";
    ss << power.toString() << "\n";
    ss << communication.toString() << "\n";
    ss << processing.toString() << "\n";
    ss << physical.toString() << "\n";
    
    // 自定义属性
    if (!custom_capabilities.empty()) {
        ss << "自定义属性:\n";
        for (const auto& [key, value] : custom_capabilities) {
            ss << "  " << key << ": " << value << "\n";
        }
    }
    
    return ss.str();
}

bool RobotCapabilities::isValid() const {
    return !robot_model.empty() &&
           !manufacturer.empty() &&
           motion.isValid() &&
           sensors.isValid() &&
           power.isValid() &&
           communication.isValid() &&
           processing.isValid() &&
           physical.isValid();
}

RobotCapabilities RobotCapabilities::createDefault() {
    RobotCapabilities capabilities;
    
    capabilities.robot_model = "Generic Robot";
    capabilities.manufacturer = "Generic";
    capabilities.robot_type = static_cast<RobotType>(0);
    capabilities.firmware_version = "1.0.0";
    capabilities.profile_version = "1.0.0";
    capabilities.last_updated = "2024-01-01";
    
    // 使用默认能力配置
    capabilities.motion = MotionCapabilities::createDefault();
    capabilities.sensors = SensorCapabilities::createDefault();
    capabilities.power = PowerCapabilities::createDefault();
    capabilities.communication = CommunicationCapabilities::createDefault();
    capabilities.processing = ProcessingCapabilities::createDefault();
    capabilities.physical = PhysicalProperties::createDefault();
    
    return capabilities;
}

RobotCapabilities RobotCapabilities::createGo2Profile() {
    RobotCapabilities capabilities = createDefault();
    capabilities.robot_model = "Go2";
    capabilities.manufacturer = "Unitree";
    // TODO: Add Go2-specific configurations
    return capabilities;
}

// ============= 工具函数实现 =============

std::string RobotCapabilityUtils::robotTypeToString(RobotType robot_type) {
    // TODO: Implement based on actual RobotType values
    return "UNKNOWN";
}

RobotType RobotCapabilityUtils::stringToRobotType(const std::string& type_str) {
    // TODO: Implement based on actual RobotType values  
    return static_cast<RobotType>(0);
}

std::string RobotCapabilityUtils::capabilityLevelToString(CapabilityLevel level) {
    switch (level) {
        case CapabilityLevel::NONE: return "NONE";
        case CapabilityLevel::BASIC: return "BASIC";
        case CapabilityLevel::INTERMEDIATE: return "INTERMEDIATE";
        case CapabilityLevel::ADVANCED: return "ADVANCED";
        case CapabilityLevel::EXPERT: return "EXPERT";
        default: return "UNKNOWN";
    }
}

CapabilityLevel RobotCapabilityUtils::stringToCapabilityLevel(const std::string& level_str) {
    if (level_str == "NONE") return CapabilityLevel::NONE;
    if (level_str == "BASIC") return CapabilityLevel::BASIC;
    if (level_str == "INTERMEDIATE") return CapabilityLevel::INTERMEDIATE;
    if (level_str == "ADVANCED") return CapabilityLevel::ADVANCED;
    if (level_str == "EXPERT") return CapabilityLevel::EXPERT;
    return CapabilityLevel::NONE;
}

std::string RobotCapabilityUtils::chargingTypeToString(ChargingType type) {
    switch (type) {
        case ChargingType::NONE: return "NONE";
        case ChargingType::CONTACT: return "CONTACT";
        case ChargingType::WIRELESS: return "WIRELESS";
        case ChargingType::INDUCTIVE: return "INDUCTIVE";
        case ChargingType::SOLAR: return "SOLAR";
        case ChargingType::HYBRID: return "HYBRID";
        default: return "UNKNOWN";
    }
}

ChargingType RobotCapabilityUtils::stringToChargingType(const std::string& type_str) {
    if (type_str == "NONE") return ChargingType::NONE;
    if (type_str == "CONTACT") return ChargingType::CONTACT;
    if (type_str == "WIRELESS") return ChargingType::WIRELESS;
    if (type_str == "INDUCTIVE") return ChargingType::INDUCTIVE;
    if (type_str == "SOLAR") return ChargingType::SOLAR;
    if (type_str == "HYBRID") return ChargingType::HYBRID;
    return ChargingType::NONE;
}

CapabilityValidationResult RobotCapabilityUtils::validateCapabilities(const RobotCapabilities& capabilities) {
    CapabilityValidationResult result;
    result.is_valid = capabilities.isValid();
    result.completeness_score = calculateCompletenessScore(capabilities);
    
    if (!result.is_valid) {
        result.errors.push_back("Capabilities validation failed");
    }
    
    return result;
}

float RobotCapabilityUtils::calculateCompletenessScore(const RobotCapabilities& capabilities) {
    // Simple completeness calculation
    float score = 0.0f;
    
    if (!capabilities.robot_model.empty()) score += 10.0f;
    if (!capabilities.manufacturer.empty()) score += 10.0f;
    if (capabilities.motion.isValid()) score += 20.0f;
    if (capabilities.sensors.isValid()) score += 20.0f;
    if (capabilities.power.isValid()) score += 20.0f;
    if (capabilities.communication.isValid()) score += 10.0f;
    if (capabilities.processing.isValid()) score += 5.0f;
    if (capabilities.physical.isValid()) score += 5.0f;
    
    return score / 100.0f;
}

float RobotCapabilityUtils::calculateOverallScore(const RobotCapabilities& capabilities) {
    return calculateCompletenessScore(capabilities) * 100.0f;
}

std::map<std::string, float> RobotCapabilityUtils::compareCapabilities(
    const RobotCapabilities& capabilities1, 
    const RobotCapabilities& capabilities2) {
    
    std::map<std::string, float> comparison;
    
    // Simple comparison - can be expanded
    comparison["motion"] = capabilities1.motion.max_linear_velocity - capabilities2.motion.max_linear_velocity;
    comparison["power"] = capabilities1.power.battery_capacity_mah - capabilities2.power.battery_capacity_mah;
    
    return comparison;
}

} // namespace capability_manager
} // namespace robot_factory