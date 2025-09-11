/**
 * @file capability_loader.cpp
 * @brief 机器人能力加载器的实现
 * @author Claude Code
 * @date 2024
 */

#include "robot_factory/capability_manager/capability_loader.hpp"
#include "robot_factory/capability_manager/capability_definitions.hpp"
#include <fstream>
#include <filesystem>
#include <rclcpp/logging.hpp>

namespace robot_factory {
namespace capability_manager {

CapabilityLoader::CapabilityLoader(const std::string& logger_name) 
    : logger_(rclcpp::get_logger(logger_name)) {
    initializeSearchPaths();
}


std::map<robot_detector::RobotType, RobotCapabilities> CapabilityLoader::loadCapabilities(
    const std::string& file_path,
    ConfigFormat format,
    const LoadOptions& options) {
    
    std::map<robot_detector::RobotType, RobotCapabilities> capabilities;
    total_loads_++;
    
    try {
        // 检查文件是否存在
        if (!isFileAccessible(file_path)) {
            throw std::runtime_error("Configuration file not accessible: " + file_path);
        }
        
        // 检查缓存
        if (options.cache_results && cache_enabled_ && isCacheValid(file_path)) {
            RCLCPP_DEBUG(logger_, "Loading capabilities from cache: %s", file_path.c_str());
            cache_hits_++;
            return config_cache_[file_path].capabilities;
        }
        
        // 自动检测格式
        if (format == ConfigFormat::AUTO_DETECT) {
            format = detectConfigFormat(file_path);
        }
        
        // 读取文件内容
        std::ifstream file(file_path);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open file: " + file_path);
        }
        
        std::string content((std::istreambuf_iterator<char>(file)),
                           std::istreambuf_iterator<char>());
        file.close();
        
        // 根据格式解析内容
        if (format == ConfigFormat::YAML) {
            capabilities = loadCapabilitiesFromYAML(content, options);
        } else if (format == ConfigFormat::JSON) {
            throw std::runtime_error("JSON format not yet implemented");
        } else {
            throw std::runtime_error("Unsupported configuration format");
        }
        
        // 验证配置
        if (options.validate_on_load) {
            for (const auto& cap_pair : capabilities) {
                auto validation_result = validateCapabilities(cap_pair.second, cap_pair.first);
                if (!validation_result.is_valid) {
                    RCLCPP_WARN(logger_, "Validation failed for robot type %d: %s", 
                               static_cast<int>(cap_pair.first), validation_result.summary.c_str());
                    
                    if (validation_failed_callback_) {
                        validation_failed_callback_(file_path, validation_result);
                    }
                }
            }
        }
        
        // 缓存结果
        if (options.cache_results && cache_enabled_) {
            CacheEntry entry;
            entry.capabilities = capabilities;
            entry.load_time = std::chrono::system_clock::now();
            entry.expiry_time = entry.load_time + options.cache_expiry;
            entry.file_hash = getFileHash(file_path);
            config_cache_[file_path] = entry;
            
            // 清理过期缓存
            cleanupExpiredCache();
        }
        
        // 执行加载完成回调
        if (load_completed_callback_) {
            ValidationResult overall_validation;
            overall_validation.is_valid = true;
            load_completed_callback_(file_path, capabilities, overall_validation);
        }
        
        RCLCPP_INFO(logger_, "Successfully loaded capabilities from %s (%zu robot types)", 
                   file_path.c_str(), capabilities.size());
        cache_misses_++;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Failed to load capabilities from %s: %s", file_path.c_str(), e.what());
        
        // 如果启用了与默认配置合并，返回默认配置
        if (options.merge_with_defaults) {
            RCLCPP_WARN(logger_, "Falling back to default capabilities");
            auto default_capabilities = generateDefaultCapabilities(robot_detector::RobotType::GENERIC);
            capabilities[robot_detector::RobotType::GENERIC] = default_capabilities;
        }
        
        throw; // 重新抛出异常
    }
    
    return capabilities;
}

bool CapabilityLoader::loadCapabilitiesFromString(
    const std::string& content,
    ConfigFormat format,
    std::map<RobotType, RobotCapabilities>& capabilities,
    const LoadOptions& options) {
    
    // TODO: Implement string parsing based on format
    // For now, create default capabilities
    auto default_capabilities = RobotCapabilities::createDefault();
    capabilities[static_cast<RobotType>(0)] = default_capabilities;
    
    return true;
}

std::string CapabilityLoader::capabilitiesToString(
    const std::map<RobotType, RobotCapabilities>& capabilities,
    ConfigFormat format,
    bool pretty_print) {
    
    // TODO: Implement serialization based on format
    return "{}"; // Empty JSON object for now
}

ValidationResult CapabilityLoader::validateCapabilities(
    const RobotCapabilities& capabilities,
    RobotType robot_type) {
    
    ValidationResult result;
    result.is_valid = capabilities.isValid();
    result.confidence_score = 1.0f;
    
    if (!result.is_valid) {
        result.errors.push_back("Capability validation failed");
    }
    
    return result;
}

RobotCapabilities CapabilityLoader::generateDefaultCapabilities(RobotType robot_type) {
    auto capabilities = RobotCapabilities::createDefault();
    capabilities.robot_type = robot_type;
    
    // TODO: Add robot-specific default configurations based on type
    
    return capabilities;
}

bool CapabilityLoader::saveCapabilities(
    const std::map<RobotType, RobotCapabilities>& capabilities,
    const std::string& file_path,
    const SaveOptions& options) {
    
    // TODO: Implement file saving
    return false;
}



ConfigFormat CapabilityLoader::detectConfigFormat(const std::string& file_path) {
    std::filesystem::path path(file_path);
    std::string extension = path.extension().string();
    
    if (extension == ".json") {
        return ConfigFormat::JSON;
    } else if (extension == ".yaml" || extension == ".yml") {
        return ConfigFormat::YAML;
    }
    
    return ConfigFormat::JSON; // Default
}

// ============= 私有辅助方法实现 =============

void CapabilityLoader::initializeSearchPaths() {
    // Initialize default search paths
    search_paths_.clear();
    search_paths_.push_back("./config");
    search_paths_.push_back("../config");
    
    try {
        auto pkg_share = ament_index_cpp::get_package_share_directory("capability_manager");
        search_paths_.push_back(pkg_share + "/config");
    } catch (const std::exception& e) {
        RCLCPP_WARN(logger_, "Could not find capability_manager package share directory: %s", e.what());
    }
}

// ============= YAML解析实现 =============

std::map<robot_detector::RobotType, RobotCapabilities> CapabilityLoader::loadCapabilitiesFromYAML(
    const std::string& yaml_content,
    const LoadOptions& options) {
    
    std::map<robot_detector::RobotType, RobotCapabilities> capabilities;
    
    try {
        YAML::Node root = YAML::Load(yaml_content);
        
        if (!root["robots"]) {
            throw std::runtime_error("Missing 'robots' section in YAML file");
        }
        
        YAML::Node robots_node = root["robots"];
        
        for (const auto& robot_pair : robots_node) {
            std::string robot_name = robot_pair.first.as<std::string>();
            YAML::Node robot_node = robot_pair.second;
            
            // 解析机器人类型
            robot_detector::RobotType robot_type = robot_detector::RobotType::GENERIC;
            if (robot_name == "go2") {
                robot_type = robot_detector::RobotType::GO2;
            } else if (robot_name == "spot") {
                robot_type = robot_detector::RobotType::SPOT;
            } else if (robot_name == "anymal") {
                robot_type = robot_detector::RobotType::ANYMAL;
            }
            
            // 解析机器人能力
            RobotCapabilities robot_capabilities = parseCapabilitiesFromYAML(robot_node, robot_type);
            capabilities[robot_type] = robot_capabilities;
            
            RCLCPP_DEBUG(logger_, "Parsed capabilities for robot: %s (type: %d)", 
                        robot_name.c_str(), static_cast<int>(robot_type));
        }
        
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("YAML parsing error: " + std::string(e.what()));
    }
    
    return capabilities;
}

RobotCapabilities CapabilityLoader::parseCapabilitiesFromYAML(
    const YAML::Node& robot_node,
    robot_detector::RobotType robot_type) {
    
    RobotCapabilities capabilities;
    
    // 基础信息
    capabilities.robot_type = robot_type;
    if (robot_node["robot_model"]) {
        capabilities.robot_model = robot_node["robot_model"].as<std::string>();
    }
    if (robot_node["manufacturer"]) {
        capabilities.manufacturer = robot_node["manufacturer"].as<std::string>();
    }
    if (robot_node["firmware_version"]) {
        capabilities.firmware_version = robot_node["firmware_version"].as<std::string>();
    }
    
    // 解析运动能力
    if (robot_node["motion_capabilities"]) {
        capabilities.motion = parseMotionCapabilities(robot_node["motion_capabilities"]);
    }
    
    // 解析传感器能力
    if (robot_node["sensor_capabilities"]) {
        capabilities.sensors = parseSensorCapabilities(robot_node["sensor_capabilities"]);
    }
    
    // 解析电源能力
    if (robot_node["power_capabilities"]) {
        capabilities.power = parsePowerCapabilities(robot_node["power_capabilities"]);
    }
    
    // 解析通信能力
    if (robot_node["communication_capabilities"]) {
        capabilities.communication = parseCommunicationCapabilities(robot_node["communication_capabilities"]);
    }
    
    // 解析计算能力
    if (robot_node["processing_capabilities"]) {
        capabilities.processing = parseProcessingCapabilities(robot_node["processing_capabilities"]);
    }
    
    // 解析物理属性
    if (robot_node["physical_properties"]) {
        capabilities.physical = parsePhysicalProperties(robot_node["physical_properties"]);
    }
    
    return capabilities;
}

MotionCapabilities CapabilityLoader::parseMotionCapabilities(const YAML::Node& node) {
    MotionCapabilities motion;
    
    if (node["max_linear_velocity"]) {
        motion.max_linear_velocity = node["max_linear_velocity"].as<float>();
    }
    if (node["max_angular_velocity"]) {
        motion.max_angular_velocity = node["max_angular_velocity"].as<float>();
    }
    if (node["max_acceleration"]) {
        motion.max_linear_acceleration = node["max_acceleration"].as<float>();
    }
    if (node["turning_radius"]) {
        motion.turning_radius = node["turning_radius"].as<float>();
    }
    
    // 解析运动模式
    if (node["locomotion_modes"] && node["locomotion_modes"].IsSequence()) {
        for (const auto& mode_node : node["locomotion_modes"]) {
            motion.locomotion_modes.push_back(mode_node.as<std::string>());
        }
    }
    
    // 解析四足机器人特有能力
    if (node["quadruped_specific"]) {
        const YAML::Node& quad_node = node["quadruped_specific"];
        if (quad_node["can_trot"]) {
            motion.quadruped_specific.can_trot = quad_node["can_trot"].as<bool>();
        }
        if (quad_node["can_bound"]) {
            motion.quadruped_specific.can_bound = quad_node["can_bound"].as<bool>();
        }
        if (quad_node["can_crawl"]) {
            motion.quadruped_specific.can_crawl = quad_node["can_crawl"].as<bool>();
        }
        if (quad_node["max_step_height"]) {
            motion.quadruped_specific.max_step_height = quad_node["max_step_height"].as<float>();
        }
    }
    
    return motion;
}

SensorCapabilities CapabilityLoader::parseSensorCapabilities(const YAML::Node& node) {
    SensorCapabilities sensors;
    
    // 解析LiDAR能力
    if (node["lidar"]) {
        const YAML::Node& lidar_node = node["lidar"];
        if (lidar_node["has_2d_lidar"]) {
            sensors.lidar.has_2d_lidar = lidar_node["has_2d_lidar"].as<bool>();
        }
        if (lidar_node["has_3d_lidar"]) {
            sensors.lidar.has_3d_lidar = lidar_node["has_3d_lidar"].as<bool>();
        }
        if (lidar_node["max_range"]) {
            sensors.lidar.max_range = lidar_node["max_range"].as<float>();
        }
        if (lidar_node["resolution"]) {
            sensors.lidar.angular_resolution = lidar_node["resolution"].as<float>();
        }
        if (lidar_node["accuracy"]) {
            sensors.lidar.accuracy = lidar_node["accuracy"].as<float>();
        }
        if (lidar_node["scan_frequency"]) {
            sensors.lidar.scan_frequency = lidar_node["scan_frequency"].as<int>();
        }
    }
    
    // 解析摄像头能力
    if (node["camera"]) {
        const YAML::Node& camera_node = node["camera"];
        if (camera_node["rgb_camera_count"]) {
            sensors.camera.rgb_camera_count = camera_node["rgb_camera_count"].as<int>();
        }
        if (camera_node["depth_camera_count"]) {
            sensors.camera.depth_camera_count = camera_node["depth_camera_count"].as<int>();
        }
        if (camera_node["max_resolution"]) {
            sensors.camera.max_resolution = camera_node["max_resolution"].as<std::string>();
        }
        if (camera_node["max_framerate"]) {
            sensors.camera.max_framerate = camera_node["max_framerate"].as<int>();
        }
        if (camera_node["has_stereo_vision"]) {
            sensors.camera.has_stereo_vision = camera_node["has_stereo_vision"].as<bool>();
        }
    }
    
    // 解析IMU能力
    if (node["imu"]) {
        const YAML::Node& imu_node = node["imu"];
        if (imu_node["dof"]) {
            sensors.imu.degrees_of_freedom = imu_node["dof"].as<int>();
        }
        if (imu_node["sampling_rate"]) {
            sensors.imu.sampling_rate = imu_node["sampling_rate"].as<int>();
        }
        if (imu_node["gyro_range"]) {
            sensors.imu.gyro_range_dps = imu_node["gyro_range"].as<float>();
        }
    }
    
    // 解析其他传感器
    if (node["has_gps"]) {
        sensors.has_gps = node["has_gps"].as<bool>();
    }
    if (node["has_encoders"]) {
        sensors.has_encoders = node["has_encoders"].as<bool>();
    }
    if (node["has_force_sensors"]) {
        sensors.has_force_sensors = node["has_force_sensors"].as<bool>();
    }
    
    return sensors;
}

PowerCapabilities CapabilityLoader::parsePowerCapabilities(const YAML::Node& node) {
    PowerCapabilities power;
    
    if (node["battery_capacity"]) {
        power.battery_capacity_mah = node["battery_capacity"].as<float>();
    }
    if (node["operating_voltage"]) {
        power.battery_voltage = node["operating_voltage"].as<float>();
    }
    if (node["charging_time_minutes"]) {
        // 将充电时间（分钟）转换为充电功率等效参数，这里暂时不处理
        // 因为结构体中没有对应的充电时间字段
    }
    if (node["estimated_runtime_hours"]) {
        power.estimated_runtime_hours = node["estimated_runtime_hours"].as<float>();
    }
    if (node["supports_hot_swap"]) {
        power.supports_hot_swap = node["supports_hot_swap"].as<bool>();
    }
    
    // 电源管理特性
    if (node["has_power_monitoring"]) {
        power.has_power_monitoring = node["has_power_monitoring"].as<bool>();
    }
    if (node["has_low_power_mode"]) {
        power.has_low_power_mode = node["has_low_power_mode"].as<bool>();
    }
    
    return power;
}

CommunicationCapabilities CapabilityLoader::parseCommunicationCapabilities(const YAML::Node& node) {
    CommunicationCapabilities comm;
    
    // 解析网络支持
    if (node["supported_networks"] && node["supported_networks"].IsSequence()) {
        for (const auto& network : node["supported_networks"]) {
            comm.network.supported_networks.push_back(network.as<std::string>());
        }
    }
    
    if (node["has_bluetooth"]) {
        comm.network.has_bluetooth = node["has_bluetooth"].as<bool>();
    }
    
    // 解析ROS2配置
    if (node["default_rmw_implementation"]) {
        comm.ros2.default_rmw_implementation = node["default_rmw_implementation"].as<std::string>();
    }
    if (node["supported_rmw"] && node["supported_rmw"].IsSequence()) {
        for (const auto& rmw : node["supported_rmw"]) {
            comm.ros2.supported_rmw.push_back(rmw.as<std::string>());
        }
    }
    
    // 解析DDS配置
    if (node["preferred_dds"]) {
        comm.ros2.preferred_dds = node["preferred_dds"].as<std::string>();
    }
    if (node["max_participants"]) {
        comm.ros2.max_participants = node["max_participants"].as<int>();
    }
    
    return comm;
}

ProcessingCapabilities CapabilityLoader::parseProcessingCapabilities(const YAML::Node& node) {
    ProcessingCapabilities processing;
    
    if (node["cpu_architecture"]) {
        processing.cpu_architecture = node["cpu_architecture"].as<std::string>();
    }
    if (node["cpu_cores"]) {
        processing.cpu_cores = node["cpu_cores"].as<int>();
    }
    if (node["ram_gb"]) {
        processing.ram_gb = node["ram_gb"].as<int>();
    }
    if (node["storage_gb"]) {
        processing.storage_gb = node["storage_gb"].as<int>();
    }
    if (node["has_gpu"]) {
        processing.has_gpu = node["has_gpu"].as<bool>();
    }
    
    // AI/ML支持
    if (node["supports_tensorflow"]) {
        processing.aiml.supports_tensorflow = node["supports_tensorflow"].as<bool>();
    }
    if (node["supports_pytorch"]) {
        processing.aiml.supports_pytorch = node["supports_pytorch"].as<bool>();
    }
    
    // 实时性能
    if (node["max_control_frequency"]) {
        processing.max_control_frequency = node["max_control_frequency"].as<float>();
    }
    if (node["typical_latency_ms"]) {
        processing.typical_latency_ms = node["typical_latency_ms"].as<float>();
    }
    
    return processing;
}

PhysicalProperties CapabilityLoader::parsePhysicalProperties(const YAML::Node& node) {
    PhysicalProperties physical;
    
    // 直接访问尺寸参数
    if (node["length"]) {
        physical.length = node["length"].as<float>();
    }
    if (node["width"]) {
        physical.width = node["width"].as<float>();
    }
    if (node["height"]) {
        physical.height = node["height"].as<float>();
    }
    if (node["weight"]) {
        physical.weight = node["weight"].as<float>();
    }
    
    // 负载能力
    if (node["max_payload"]) {
        physical.max_payload = node["max_payload"].as<float>();
    }
    
    // 环境限制
    if (node["environmental"]) {
        const YAML::Node& env_node = node["environmental"];
        if (env_node["min_temperature"]) {
            physical.environmental.min_operating_temperature = env_node["min_temperature"].as<float>();
        }
        if (env_node["max_temperature"]) {
            physical.environmental.max_operating_temperature = env_node["max_temperature"].as<float>();
        }
        if (env_node["ip_rating"]) {
            physical.environmental.ip_rating = env_node["ip_rating"].as<int>();
        }
        if (env_node["waterproof"]) {
            physical.environmental.is_waterproof = env_node["waterproof"].as<bool>();
        }
        if (env_node["dustproof"]) {
            physical.environmental.is_dustproof = env_node["dustproof"].as<bool>();
        }
    }
    
    return physical;
}

// ============= 辅助方法实现 =============

bool CapabilityLoader::isFileAccessible(const std::string& file_path) {
    std::ifstream file(file_path);
    return file.good();
}

bool CapabilityLoader::isCacheValid(const std::string& file_path) const {
    auto it = config_cache_.find(file_path);
    if (it == config_cache_.end()) {
        return false;
    }
    
    const CacheEntry& entry = it->second;
    auto now = std::chrono::system_clock::now();
    
    // 检查是否过期
    if (now > entry.expiry_time) {
        return false;
    }
    
    // 检查文件是否有变化
    std::string current_hash = getFileHash(file_path);
    if (current_hash != entry.file_hash) {
        return false;
    }
    
    return true;
}

std::string CapabilityLoader::getFileHash(const std::string& file_path) const {
    // 简单实现：使用文件修改时间作为"哈希"
    // 实际应用中可以使用真正的哈希算法
    try {
        auto ftime = std::filesystem::last_write_time(file_path);
        auto sctp = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
            ftime - std::filesystem::file_time_type::clock::now() + std::chrono::system_clock::now());
        auto time_t = std::chrono::system_clock::to_time_t(sctp);
        return std::to_string(time_t);
    } catch (...) {
        return "0";
    }
}

void CapabilityLoader::cleanupExpiredCache() const {
    auto now = std::chrono::system_clock::now();
    for (auto it = config_cache_.begin(); it != config_cache_.end();) {
        if (now > it->second.expiry_time) {
            it = config_cache_.erase(it);
        } else {
            ++it;
        }
    }
}

} // namespace capability_manager
} // namespace robot_factory