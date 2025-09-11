/**
 * @file robot_adapter_factory.cpp
 * @brief 机器人适配器工厂的具体实现
 * @author Claude Code  
 * @date 2024
 */

#include "robot_factory/adapter_factory/robot_adapter_factory.hpp"
#include "robot_factory/adapter_factory/adapter_registry.hpp"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cstdlib>

namespace robot_factory {
namespace adapter_factory {

// ============= 构造函数和初始化 =============

RobotAdapterFactory::RobotAdapterFactory() {
    // 初始化默认配置
    default_config_ = AdapterCreationConfig();
    
    // 设置默认的机器人检测器
#if defined(ROBOT_HAS_DETECTOR)
    robot_detector_ = std::make_shared<robot_factory::robot_detector::RobotDetector>();
#endif
    
    // 初始化默认适配器创建器
    initializeDefaultCreators();
}

// ============= 主要创建接口实现 =============

AdapterCreationResult RobotAdapterFactory::createAdapter(const AdapterCreationConfig& config) {
    AdapterCreationResult result;
    result.creation_time = getCurrentTimeString();
    
    // 验证配置
    if (!validateConfig(config)) {
        result.success = false;
        result.error_message = "Invalid adapter creation configuration";
        result.error_code = -1;
        return result;
    }
    
    // 自动检测机器人类型
    RobotType detected_type = RobotType::GENERIC;
    
    if (robot_detector_) {
#if defined(ROBOT_HAS_DETECTOR)
        auto detection_result = robot_detector_->detectRobot();
        if (detection_result.is_detected) {
            detected_type = detection_result.robot_type;
            result.detected_robot_type = detected_type;
        }
#endif
    }
    
    // 使用检测到的类型创建适配器
    return createAdapter(detected_type, config);
}

AdapterCreationResult RobotAdapterFactory::createAdapter(RobotType robot_type, 
                                                         const AdapterCreationConfig& config) {
    AdapterCreationResult result;
    result.detected_robot_type = robot_type;
    result.creation_time = getCurrentTimeString();
    
    // 更新统计信息
    creation_count_[robot_type]++;
    
    try {
        // 设置环境变量 (特别是DDS配置)
        setupEnvironmentVariables(config);
        
        // 从注册表获取创建器
        auto& registry = AdapterRegistry::getInstance();
        auto creator = registry.getCreator(robot_type);
        
        if (!creator) {
            result.success = false;
            result.error_message = "No creator found for robot type: " + 
                                 AdapterRegistry::robotTypeToString(robot_type);
            result.error_code = -2;
            setLastError(result.error_message);
            return result;
        }
        
        // 创建适配器
        auto adapter = creator(config);
        
        if (adapter) {
            result.success = true;
            result.adapter = adapter;
            result.adapter_version = adapter->getAdapterVersion();
            
            // 更新成功统计
            success_count_[robot_type]++;
            updateStatistics(robot_type, true);
            
        } else {
            result.success = false;
            result.error_message = "Adapter creation returned null";
            result.error_code = -3;
            setLastError(result.error_message);
            updateStatistics(robot_type, false);
        }
        
    } catch (const std::exception& e) {
        result.success = false;
        result.error_message = "Exception during adapter creation: " + std::string(e.what());
        result.error_code = -4;
        setLastError(result.error_message);
        updateStatistics(robot_type, false);
    }
    
    return result;
}

AdapterCreationResult RobotAdapterFactory::createAdapterFromDetection(
    const RobotDetectionResult& detection_result,
    const AdapterCreationConfig& config) {
    
    AdapterCreationResult result;
    
    if (!detection_result.is_detected) {
        result.success = false;
        result.error_message = "Robot detection failed, cannot create adapter";
        result.error_code = -5;
        return result;
    }
    
    // 使用检测结果中的配置信息更新创建配置
    AdapterCreationConfig updated_config = config;
    
    if (!detection_result.network_address.empty()) {
        updated_config.robot_ip = detection_result.network_address;
    }
    
    if (detection_result.communication_port > 0) {
        updated_config.robot_port = detection_result.communication_port;
    }
    
    // 创建适配器
    result = createAdapter(detection_result.robot_type, updated_config);
    
    // 添加检测相关的信息
    if (result.success) {
        result.detected_robot_type = detection_result.robot_type;
    }
    
    return result;
}

std::vector<AdapterCreationResult> RobotAdapterFactory::createMultipleAdapters(
    const std::vector<std::pair<RobotType, AdapterCreationConfig>>& configs) {
    
    std::vector<AdapterCreationResult> results;
    results.reserve(configs.size());
    
    for (const auto& config_pair : configs) {
        auto result = createAdapter(config_pair.first, config_pair.second);
        results.push_back(std::move(result));
    }
    
    return results;
}

// ============= 适配器注册和扩展实现 =============

bool RobotAdapterFactory::registerAdapterCreator(RobotType robot_type, AdapterCreator creator) {
    auto& registry = AdapterRegistry::getInstance();
    std::string creator_name = "Custom" + AdapterRegistry::robotTypeToString(robot_type) + "Creator";
    return registry.registerCreator(robot_type, creator_name, creator);
}

bool RobotAdapterFactory::unregisterAdapterCreator(RobotType robot_type) {
    auto& registry = AdapterRegistry::getInstance();
    return registry.unregisterCreator(robot_type);
}

bool RobotAdapterFactory::isRobotTypeSupported(RobotType robot_type) const {
    auto& registry = AdapterRegistry::getInstance();
    return registry.hasCreator(robot_type);
}

std::vector<RobotType> RobotAdapterFactory::getSupportedRobotTypes() const {
    auto& registry = AdapterRegistry::getInstance();
    return registry.getRegisteredTypes();
}

// ============= Go2专用创建方法实现 =============

AdapterCreationResult RobotAdapterFactory::createGo2Adapter(const AdapterCreationConfig& config) {
    return createAdapter(RobotType::GO2, config);
}

AdapterCreationResult RobotAdapterFactory::createGo2AdapterWithDefaults(
    const std::string& robot_ip,
    const std::string& network_interface) {
    
    AdapterCreationConfig config;
    config.robot_ip = robot_ip;
    config.network_interface = network_interface;
    config.robot_port = 8080;
    config.rmw_implementation = "rmw_cyclonedds_cpp";
    
    // 生成Go2专用的CycloneDDS配置
    config.cyclonedds_uri = "<CycloneDDS><Domain><General><Interfaces>"
                           "<NetworkInterface name=\"" + network_interface + 
                           "\" priority=\"default\" multicast=\"default\" />"
                           "</Interfaces></General></Domain></CycloneDDS>";
    
    return createGo2Adapter(config);
}

// ============= 配置和管理实现 =============

void RobotAdapterFactory::setDefaultConfig(const AdapterCreationConfig& config) {
    default_config_ = config;
}

AdapterCreationConfig RobotAdapterFactory::getDefaultConfig() const {
    return default_config_;
}

void RobotAdapterFactory::setRobotDetector(std::shared_ptr<robot_detector::RobotDetector> detector) {
    robot_detector_ = detector;
}

std::shared_ptr<robot_detector::RobotDetector> RobotAdapterFactory::getRobotDetector() const {
    return robot_detector_;
}

// ============= 诊断和调试实现 =============

bool RobotAdapterFactory::testAdapterCreation(RobotType robot_type, const AdapterCreationConfig& config) {
    auto& registry = AdapterRegistry::getInstance();
    return registry.testCreator(robot_type, &config);
}

std::string RobotAdapterFactory::getCreationStatistics() const {
    std::ostringstream stats;
    stats << "{\n";
    stats << "  \"creation_counts\": {\n";
    
    bool first = true;
    for (const auto& pair : creation_count_) {
        if (!first) stats << ",\n";
        stats << "    \"" << AdapterRegistry::robotTypeToString(pair.first) << "\": " << pair.second;
        first = false;
    }
    
    stats << "\n  },\n";
    stats << "  \"success_counts\": {\n";
    
    first = true;
    for (const auto& pair : success_count_) {
        if (!first) stats << ",\n";
        stats << "    \"" << AdapterRegistry::robotTypeToString(pair.first) << "\": " << pair.second;
        first = false;
    }
    
    stats << "\n  }\n}";
    return stats.str();
}

std::string RobotAdapterFactory::getLastError() const {
    return last_error_;
}

void RobotAdapterFactory::clearErrors() {
    last_error_.clear();
}

// ============= 静态工厂方法实现 =============

IRobotAdapterPtr RobotAdapterFactory::quickCreateGo2Adapter(const std::string& robot_ip,
                                                            const std::string& network_interface) {
    auto& factory = getInstance();
    auto result = factory.createGo2AdapterWithDefaults(robot_ip, network_interface);
    return result.success ? result.adapter : nullptr;
}

IRobotAdapterPtr RobotAdapterFactory::autoCreateAdapter() {
    auto& factory = getInstance();
    auto result = factory.createAdapter();
    return result.success ? result.adapter : nullptr;
}

RobotAdapterFactory& RobotAdapterFactory::getInstance() {
    static RobotAdapterFactory instance;
    return instance;
}

// ============= 私有辅助方法实现 =============

void RobotAdapterFactory::initializeDefaultCreators() {
    // 初始化默认的适配器创建器
    auto& registry = AdapterRegistry::getInstance();
    
    // 如果注册表为空，注册默认创建器
    if (registry.getRegisteredTypes().empty()) {
        registry.registerDefaultCreators();
    }
}

bool RobotAdapterFactory::validateConfig(const AdapterCreationConfig& config) const {
    // 验证IP地址格式
    if (config.robot_ip.empty()) {
        setLastError("Robot IP address cannot be empty");
        return false;
    }
    
    // 验证端口号
    if (config.robot_port <= 0 || config.robot_port > 65535) {
        setLastError("Invalid robot port: " + std::to_string(config.robot_port));
        return false;
    }
    
    // 验证超时时间
    if (config.connection_timeout_ms < 0) {
        setLastError("Connection timeout cannot be negative");
        return false;
    }
    
    // 验证网络接口名称
    if (config.network_interface.empty()) {
        setLastError("Network interface name cannot be empty");
        return false;
    }
    
    return true;
}

void RobotAdapterFactory::setupEnvironmentVariables(const AdapterCreationConfig& config) const {
    // 设置RMW实现
    if (!config.rmw_implementation.empty()) {
        setenv("RMW_IMPLEMENTATION", config.rmw_implementation.c_str(), 1);
    }
    
    // 设置CycloneDDS URI
    if (!config.cyclonedds_uri.empty()) {
        setenv("CYCLONEDDS_URI", config.cyclonedds_uri.c_str(), 1);
    }
    
    // 设置调试日志级别
    if (config.enable_debug_logging) {
        setenv("RCUTILS_LOGGING_SEVERITY_THRESHOLD", "DEBUG", 1);
    }
}

void RobotAdapterFactory::updateStatistics(RobotType robot_type, bool success) const {
    // 统计信息在类成员变量中更新，这里可以添加额外的统计逻辑
    // 例如记录到日志文件或发送统计事件等
}

void RobotAdapterFactory::setLastError(const std::string& error_message) const {
    last_error_ = error_message;
    std::cerr << "[RobotAdapterFactory ERROR] " << error_message << std::endl;
}

std::string RobotAdapterFactory::getCurrentTimeString() const {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    
    std::ostringstream oss;
    oss << std::put_time(std::gmtime(&time_t), "%Y-%m-%dT%H:%M:%SZ");
    return oss.str();
}

} // namespace adapter_factory
} // namespace robot_factory