/**
 * @file robot_detector.cpp
 * @brief 机器人自动检测器的具体实现
 * @author Claude Code
 * @date 2024
 */

#include "robot_factory/robot_detector/robot_detector.hpp"
#include "robot_factory/robot_detector/robot_types.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cstring>
#include <chrono>
#include <cstdlib>

// ROS2 includes for topic discovery
#include <rclcpp/rclcpp.hpp>

// 系统包含 (用于网络和系统调用)
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/time.h>

namespace robot_factory {
namespace robot_detector {

// ============= 构造函数和析构函数 =============

RobotDetector::RobotDetector() {
    // 初始化检测方法启用状态
    enabled_methods_["environment"] = true;
    enabled_methods_["network"] = true;
    enabled_methods_["config"] = true;
    enabled_methods_["topics"] = true;
    
    // 设置默认网络配置
    network_config_ = NetworkDetectionConfig();
    
    last_error_.clear();
}

// ============= 主要检测接口实现 =============

RobotDetectionResult RobotDetector::detectRobot() {
    // 检查缓存是否有效
    uint64_t current_time = getCurrentTimeNanoseconds();
    if (current_time - last_detection_time_ns_ < CACHE_VALIDITY_NS && 
        !detection_cache_.empty()) {
        // 返回最可信的缓存结果
        for (const auto& pair : detection_cache_) {
            if (pair.second.is_detected && pair.second.detection_confidence > 0.8f) {
                return pair.second;
            }
        }
    }
    
    RobotDetectionResult result;
    result.detection_time_ns = current_time;
    
    // 1. 优先检查环境变量检测 (最高置信度)
    if (enabled_methods_["environment"]) {
        auto env_result = detectByEnvironment();
        if (env_result.is_detected && env_result.detection_confidence > 0.9f) {
            updateDetectionCache(env_result);
            return env_result;
        }
        // 即使不成功也保留结果作为备选
        if (env_result.detection_confidence > result.detection_confidence) {
            result = env_result;
        }
    }
    
    // 2. 网络检测 (次高置信度)
    if (enabled_methods_["network"]) {
        auto network_result = detectByNetwork();
        if (network_result.is_detected && network_result.detection_confidence > result.detection_confidence) {
            result = network_result;
        }
    }
    
    // 3. ROS2话题检测
    if (enabled_methods_["topics"]) {
        auto topic_result = detectByROS2Topics();
        if (topic_result.is_detected && topic_result.detection_confidence > result.detection_confidence) {
            result = topic_result;
        }
    }
    
    // 4. 配置文件检测 (最低优先级)
    if (enabled_methods_["config"]) {
        auto config_result = detectByConfigFile();
        if (config_result.is_detected && config_result.detection_confidence > result.detection_confidence) {
            result = config_result;
        }
    }
    
    // 更新缓存
    if (result.is_detected) {
        updateDetectionCache(result);
    }
    
    return result;
}

RobotDetectionResult RobotDetector::detectSpecificRobot(RobotType target_type) {
    // 检查缓存
    if (isCacheValid(target_type)) {
        auto it = detection_cache_.find(target_type);
        if (it != detection_cache_.end()) {
            return it->second;
        }
    }
    
    RobotDetectionResult result;
    result.robot_type = target_type;
    result.detection_time_ns = getCurrentTimeNanoseconds();
    
    // 根据目标类型执行特定检测
    switch (target_type) {
        case RobotType::GO2:
            result = detectGo2Robot();
            break;
        case RobotType::SPOT:
            result = detectSpotRobot();
            break;
        case RobotType::ANYMAL:
            result = detectAnymalRobot();
            break;
        default:
            result.is_detected = false;
            result.detection_method = "unsupported";
            result.evidence.push_back("Robot type not implemented: " + 
                                    RobotTypeUtils::robotTypeToString(target_type));
            setLastError("Unsupported robot type for specific detection");
            break;
    }
    
    if (result.is_detected) {
        updateDetectionCache(result);
    }
    
    return result;
}

std::vector<RobotDetectionResult> RobotDetector::detectAllRobots() {
    std::vector<RobotDetectionResult> results;
    
    // 获取所有已实现的机器人类型
    auto implemented_types = RobotTypeUtils::getImplementedTypes();
    
    for (RobotType robot_type : implemented_types) {
        auto result = detectSpecificRobot(robot_type);
        if (result.is_detected) {
            results.push_back(result);
        }
    }
    
    return results;
}

bool RobotDetector::isRobotOnline(RobotType robot_type) {
    auto result = detectSpecificRobot(robot_type);
    return result.is_detected && result.detection_confidence > 0.5f;
}

// ============= 配置管理实现 =============

void RobotDetector::setNetworkDetectionConfig(const NetworkDetectionConfig& config) {
    network_config_ = config;
}

NetworkDetectionConfig RobotDetector::getNetworkDetectionConfig() const {
    return network_config_;
}

void RobotDetector::enableDetectionMethod(const std::string& method_name, bool enabled) {
    enabled_methods_[method_name] = enabled;
}

void RobotDetector::setDetectionTimeout(int timeout_ms) {
    detection_timeout_ms_ = timeout_ms;
}

// ============= 具体检测方法实现 =============

RobotDetectionResult RobotDetector::detectByEnvironment() {
    RobotDetectionResult result;
    result.detection_method = "environment";
    result.detection_time_ns = getCurrentTimeNanoseconds();
    
    // 检查常见的环境变量
    std::vector<std::pair<std::string, RobotType>> env_mappings = {
        {"ROBOT_TYPE", RobotType::GENERIC},  // 通用环境变量
        {"UNITREE_ROBOT_TYPE", RobotType::GO2},  // 宇树专用
        {"GO2_ROBOT", RobotType::GO2},
        {"SPOT_ROBOT", RobotType::SPOT},
        {"ANYMAL_ROBOT", RobotType::ANYMAL}
    };
    
    for (const auto& mapping : env_mappings) {
        std::string env_value = getEnvironmentVariable(mapping.first);
        if (!env_value.empty()) {
            result.evidence.push_back("Found environment variable: " + mapping.first + "=" + env_value);
            
            // 尝试解析环境变量值
            RobotType detected_type = RobotTypeUtils::stringToRobotType(env_value);
            if (detected_type != RobotType::GENERIC || env_value != "generic") {
                result.is_detected = true;
                result.robot_type = detected_type;
                result.robot_name = RobotTypeUtils::robotTypeToString(detected_type);
                result.detection_confidence = RobotTypeUtils::getDetectionConfidence(detected_type, "environment");
                
                // 获取额外信息
                std::string robot_ip = getEnvironmentVariable("ROBOT_IP");
                if (!robot_ip.empty()) {
                    result.network_address = robot_ip;
                    result.evidence.push_back("Found ROBOT_IP: " + robot_ip);
                }
                
                std::string robot_name = getEnvironmentVariable("ROBOT_NAME");
                if (!robot_name.empty()) {
                    result.robot_name = robot_name;
                    result.evidence.push_back("Found ROBOT_NAME: " + robot_name);
                }
                
                break;
            } else if (mapping.second != RobotType::GENERIC) {
                // 环境变量存在但值为空或true，使用映射的类型
                result.is_detected = true;
                result.robot_type = mapping.second;
                result.robot_name = RobotTypeUtils::robotTypeToString(mapping.second);
                result.detection_confidence = RobotTypeUtils::getDetectionConfidence(mapping.second, "environment");
                break;
            }
        }
    }
    
    if (!result.is_detected) {
        result.evidence.push_back("No robot type environment variables found");
        setLastError("Environment variable detection failed: no relevant env vars");
    }
    
    return result;
}

RobotDetectionResult RobotDetector::detectByNetwork() {
    RobotDetectionResult result;
    result.detection_method = "network";
    result.detection_time_ns = getCurrentTimeNanoseconds();
    
    // 尝试检测Go2机器人
    if (checkGo2NetworkConnection()) {
        result.is_detected = true;
        result.robot_type = RobotType::GO2;
        result.robot_name = "Unitree Go2";
        result.network_address = network_config_.go2.ip_addresses[0];
        result.communication_port = network_config_.go2.ports[0];
        result.detection_confidence = RobotTypeUtils::getDetectionConfidence(RobotType::GO2, "network");
        result.evidence.push_back("Go2 network connection successful");
        
        // 获取机器人详细信息
        auto robot_info = getGo2RobotInfo();
        for (const auto& info : robot_info) {
            result.evidence.push_back(info.first + ": " + info.second);
            
            // 提取关键信息
            if (info.first == "serial_number") {
                result.serial_number = info.second;
            } else if (info.first == "firmware_version") {
                result.firmware_version = info.second;
            }
        }
    }
    
    // 这里可以添加其他机器人的网络检测
    // TODO: 实现Spot和ANYmal的网络检测
    
    if (!result.is_detected) {
        result.evidence.push_back("No robot network connections detected");
        setLastError("Network detection failed: no robot connections found");
    }
    
    return result;
}

RobotDetectionResult RobotDetector::detectByConfigFile(const std::string& config_file_path) {
    RobotDetectionResult result;
    result.detection_method = "config";
    result.detection_time_ns = getCurrentTimeNanoseconds();
    
    std::vector<std::string> config_paths;
    
    // 如果提供了特定路径，使用该路径
    if (!config_file_path.empty()) {
        config_paths.push_back(config_file_path);
    } else {
        // 搜索常见的配置文件位置
        config_paths = {
            "/etc/robot_config.yaml",
            "/etc/robot_config.json",
            "~/.robot_config.yaml",
            "~/.robot_config.json",
            "./robot_config.yaml",
            "./robot_config.json",
            "./config/robot_config.yaml",
            "./config/robot_config.json"
        };
    }
    
    for (const std::string& path : config_paths) {
        std::ifstream config_file(path);
        if (config_file.is_open()) {
            result.evidence.push_back("Found config file: " + path);
            
            std::string line;
            while (std::getline(config_file, line)) {
                // 简单的关键字搜索 (实际实现中应该使用proper的YAML/JSON解析)
                if (line.find("robot_type") != std::string::npos || 
                    line.find("type") != std::string::npos) {
                    
                    // 提取类型信息
                    if (line.find("go2") != std::string::npos || 
                        line.find("Go2") != std::string::npos) {
                        result.is_detected = true;
                        result.robot_type = RobotType::GO2;
                        result.robot_name = "Unitree Go2";
                        result.detection_confidence = RobotTypeUtils::getDetectionConfidence(RobotType::GO2, "config");
                    } else if (line.find("spot") != std::string::npos || 
                              line.find("Spot") != std::string::npos) {
                        result.is_detected = true;
                        result.robot_type = RobotType::SPOT;
                        result.robot_name = "Boston Dynamics Spot";
                        result.detection_confidence = RobotTypeUtils::getDetectionConfidence(RobotType::SPOT, "config");
                    }
                    // 可以添加更多机器人类型的检测
                }
                
                // 提取网络信息
                if (line.find("ip") != std::string::npos || line.find("address") != std::string::npos) {
                    // 简单的IP地址提取 (需要改进)
                    size_t pos = line.find("192.168.");
                    if (pos != std::string::npos) {
                        size_t end_pos = line.find_first_of(" \t\n\"',", pos);
                        if (end_pos != std::string::npos) {
                            result.network_address = line.substr(pos, end_pos - pos);
                        }
                    }
                }
            }
            
            config_file.close();
            break;  // 找到有效配置文件后停止搜索
        }
    }
    
    if (!result.is_detected) {
        result.evidence.push_back("No valid robot configuration files found");
        setLastError("Config file detection failed: no valid config files");
    }
    
    return result;
}

RobotDetectionResult RobotDetector::detectByROS2Topics() {
    RobotDetectionResult result;
    result.detection_method = "topics";
    result.detection_time_ns = getCurrentTimeNanoseconds();
    
    // 检查Go2特征话题
    auto go2_topics = checkGo2ROS2Topics();
    if (go2_topics.size() >= 3) {  // 至少检测到3个特征话题才认为是Go2
        result.is_detected = true;
        result.robot_type = RobotType::GO2;
        result.robot_name = "Unitree Go2";
        result.detection_confidence = RobotTypeUtils::getDetectionConfidence(RobotType::GO2, "topics");
        
        result.evidence.push_back("Detected Go2 signature topics:");
        for (const std::string& topic : go2_topics) {
            result.evidence.push_back("  - " + topic);
        }
    }
    
    // 这里可以添加其他机器人的话题检测
    // TODO: 实现Spot和ANYmal的话题检测
    
    if (!result.is_detected) {
        result.evidence.push_back("No robot signature topics detected");
        setLastError("ROS2 topics detection failed: no signature topics found");
    }
    
    return result;
}

// ============= Go2特定检测方法实现 =============

RobotDetectionResult RobotDetector::detectGo2Robot() {
    RobotDetectionResult result;
    result.robot_type = RobotType::GO2;
    result.robot_name = "Unitree Go2";
    result.detection_time_ns = getCurrentTimeNanoseconds();
    result.detection_method = "go2_specific";
    
    float max_confidence = 0.0f;
    
    // 1. 环境变量检测
    if (enabled_methods_["environment"]) {
        std::string robot_env = getEnvironmentVariable("ROBOT_TYPE");
        std::string go2_env = getEnvironmentVariable("GO2_ROBOT");
        
        if (robot_env == "go2" || robot_env == "Go2" || !go2_env.empty()) {
            result.is_detected = true;
            max_confidence = std::max(max_confidence, go2_constants::ENV_DETECTION_CONFIDENCE);
            result.evidence.push_back("Go2 environment variable detected");
        }
    }
    
    // 2. 网络连接检测
    if (enabled_methods_["network"] && checkGo2NetworkConnection()) {
        result.is_detected = true;
        max_confidence = std::max(max_confidence, go2_constants::NETWORK_DETECTION_CONFIDENCE);
        result.evidence.push_back("Go2 network connection established");
        result.network_address = go2_constants::DEFAULT_IP;
        result.communication_port = go2_constants::DEFAULT_PORT;
    }
    
    // 3. ROS2话题检测
    if (enabled_methods_["topics"]) {
        auto topics = checkGo2ROS2Topics();
        if (topics.size() >= 2) {  // 至少2个特征话题
            result.is_detected = true;
            max_confidence = std::max(max_confidence, go2_constants::TOPIC_DETECTION_CONFIDENCE);
            result.evidence.push_back("Go2 ROS2 topics detected: " + std::to_string(topics.size()) + " topics");
        }
    }
    
    result.detection_confidence = max_confidence;
    
    if (result.is_detected) {
        // 获取额外的机器人信息
        auto robot_info = getGo2RobotInfo();
        for (const auto& info : robot_info) {
            if (info.first == "serial_number") {
                result.serial_number = info.second;
            } else if (info.first == "firmware_version") {
                result.firmware_version = info.second;
            } else if (info.first == "model") {
                result.robot_model = info.second;
            }
        }
    }
    
    return result;
}

bool RobotDetector::checkGo2NetworkConnection(const std::string& ip_address) {
    std::string target_ip = ip_address.empty() ? go2_constants::DEFAULT_IP : ip_address;
    
    // 1. Ping测试
    if (!pingAddress(target_ip, network_config_.network_timeout_ms)) {
        return false;
    }
    
    // 2. 端口连通性测试
    bool port_open = false;
    for (int port : network_config_.go2.ports) {
        if (checkPortOpen(target_ip, port, network_config_.network_timeout_ms)) {
            port_open = true;
            break;
        }
    }
    
    return port_open;
}

std::vector<std::string> RobotDetector::checkGo2ROS2Topics() {
    std::vector<std::string> detected_topics;
    
    try {
        // 创建临时ROS2节点用于话题发现
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        
        auto temp_node = rclcpp::Node::make_shared("robot_detector_temp_" + 
                                                  std::to_string(getCurrentTimeNanoseconds()));
        
        // 获取所有可用话题和类型
        auto topic_names_and_types = temp_node->get_topic_names_and_types();
        
        // 检查Go2特征话题是否存在
        for (const std::string& signature_topic : go2_constants::SIGNATURE_TOPICS) {
            for (const auto& topic_pair : topic_names_and_types) {
                const std::string& topic_name = topic_pair.first;
                
                // 检查话题名称是否匹配
                if (topic_name == signature_topic || 
                    topic_name.find(signature_topic) != std::string::npos) {
                    detected_topics.push_back(topic_name);
                    break;  // 找到匹配的话题后跳出内层循环
                }
            }
        }
        
        // 清理临时节点
        temp_node.reset();
        
    } catch (const std::exception& e) {
        setLastError("ROS2 topic detection failed: " + std::string(e.what()));
        
        // 如果ROS2检测失败，尝试使用系统命令作为备选方案
        try {
            return checkGo2ROS2TopicsViaCLI();
        } catch (...) {
            // 如果所有方法都失败，返回空列表
            return {};
        }
    }
    
    return detected_topics;
}

std::map<std::string, std::string> RobotDetector::getGo2RobotInfo(const std::string& ip_address) {
    std::map<std::string, std::string> info;
    
    std::string target_ip = ip_address.empty() ? go2_constants::DEFAULT_IP : ip_address;
    
    // 基础信息
    info["manufacturer"] = go2_constants::MANUFACTURER;
    info["model"] = go2_constants::MODEL_NAME;
    info["device_type"] = go2_constants::DEVICE_TYPE;
    info["ip_address"] = target_ip;
    info["default_port"] = std::to_string(go2_constants::DEFAULT_PORT);
    
    // 物理规格
    info["length_m"] = std::to_string(go2_constants::LENGTH_M);
    info["width_m"] = std::to_string(go2_constants::WIDTH_M);
    info["height_m"] = std::to_string(go2_constants::HEIGHT_M);
    info["weight_kg"] = std::to_string(go2_constants::WEIGHT_KG);
    info["max_payload_kg"] = std::to_string(go2_constants::MAX_PAYLOAD_KG);
    
    // 网络连接信息
    if (checkGo2NetworkConnection(target_ip)) {
        info["network_status"] = "connected";
        info["connection_method"] = "ethernet";
        
        // 这里可以通过网络API获取更多信息
        // 例如固件版本、序列号等
        // 实际实现中需要调用Go2的网络API
        
        info["firmware_version"] = "1.0.0";  // 示例值
        info["serial_number"] = "GO2001234";  // 示例值
    } else {
        info["network_status"] = "disconnected";
    }
    
    return info;
}

// ============= 工具方法实现 =============

std::string RobotDetector::robotTypeToString(RobotType robot_type) {
    return RobotTypeUtils::robotTypeToString(robot_type);
}

RobotType RobotDetector::stringToRobotType(const std::string& type_string) {
    return RobotTypeUtils::stringToRobotType(type_string);
}

std::vector<RobotType> RobotDetector::getSupportedRobotTypes() {
    return RobotTypeUtils::getAllSupportedTypes();
}

// ============= 私有辅助方法实现 =============

bool RobotDetector::pingAddress(const std::string& ip_address, int timeout_ms) {
    // 简单的ping实现，通过系统调用
    std::string ping_command = "ping -c 1 -W " + std::to_string(timeout_ms / 1000) + " " + ip_address + " > /dev/null 2>&1";
    int result = std::system(ping_command.c_str());
    return result == 0;
}

bool RobotDetector::checkPortOpen(const std::string& ip_address, int port, int timeout_ms) {
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        return false;
    }
    
    // 设置非阻塞模式
    int flags = fcntl(sockfd, F_GETFL, 0);
    fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);
    
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    inet_pton(AF_INET, ip_address.c_str(), &server_addr.sin_addr);
    
    int result = connect(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr));
    
    if (result == 0) {
        // 立即连接成功
        close(sockfd);
        return true;
    }
    
    if (errno == EINPROGRESS) {
        // 连接正在进行中，等待完成
        fd_set write_fds;
        FD_ZERO(&write_fds);
        FD_SET(sockfd, &write_fds);
        
        struct timeval tv;
        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;
        
        int select_result = select(sockfd + 1, NULL, &write_fds, NULL, &tv);
        if (select_result > 0 && FD_ISSET(sockfd, &write_fds)) {
            // 检查连接是否成功
            int error = 0;
            socklen_t len = sizeof(error);
            getsockopt(sockfd, SOL_SOCKET, SO_ERROR, &error, &len);
            close(sockfd);
            return error == 0;
        }
    }
    
    close(sockfd);
    return false;
}

std::string RobotDetector::getEnvironmentVariable(const std::string& var_name, 
                                                 const std::string& default_value) const {
    const char* env_value = std::getenv(var_name.c_str());
    return env_value ? std::string(env_value) : default_value;
}

uint64_t RobotDetector::getCurrentTimeNanoseconds() const {
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
}

bool RobotDetector::isCacheValid(RobotType robot_type) const {
    auto it = detection_cache_.find(robot_type);
    if (it == detection_cache_.end()) {
        return false;
    }
    
    uint64_t current_time = getCurrentTimeNanoseconds();
    return (current_time - it->second.detection_time_ns) < CACHE_VALIDITY_NS;
}

void RobotDetector::updateDetectionCache(const RobotDetectionResult& result) {
    detection_cache_[result.robot_type] = result;
    last_detection_time_ns_ = result.detection_time_ns;
}

void RobotDetector::setLastError(const std::string& error_message) const {
    last_error_ = error_message;
    std::cerr << "[RobotDetector ERROR] " << error_message << std::endl;
}

std::vector<std::string> RobotDetector::checkGo2ROS2TopicsViaCLI() {
    std::vector<std::string> detected_topics;
    
    try {
        // 使用ros2命令行工具获取话题列表
        std::string cmd = "timeout 5s ros2 topic list 2>/dev/null";
        FILE* pipe = popen(cmd.c_str(), "r");
        if (!pipe) {
            throw std::runtime_error("Failed to execute ros2 topic list command");
        }
        
        char buffer[256];
        std::string result;
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            result += buffer;
        }
        
        int exit_code = pclose(pipe);
        if (exit_code != 0) {
            throw std::runtime_error("ros2 topic list command failed with code " + std::to_string(exit_code));
        }
        
        // 解析输出，查找Go2特征话题
        std::istringstream topic_stream(result);
        std::string topic_line;
        
        while (std::getline(topic_stream, topic_line)) {
            // 移除行末换行符
            if (!topic_line.empty() && topic_line.back() == '\n') {
                topic_line.pop_back();
            }
            
            // 检查是否匹配Go2特征话题
            for (const std::string& signature_topic : go2_constants::SIGNATURE_TOPICS) {
                if (topic_line == signature_topic || 
                    topic_line.find(signature_topic) != std::string::npos) {
                    detected_topics.push_back(topic_line);
                    break;
                }
            }
        }
        
    } catch (const std::exception& e) {
        setLastError("CLI-based topic detection failed: " + std::string(e.what()));
    }
    
    return detected_topics;
}

} // namespace robot_detector
} // namespace robot_factory