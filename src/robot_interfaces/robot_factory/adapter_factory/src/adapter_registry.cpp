/**
 * @file adapter_registry.cpp
 * @brief 适配器注册管理器的具体实现
 * @author Claude Code
 * @date 2024
 */

#include "robot_factory/adapter_factory/adapter_registry.hpp"
#include "robot_factory/adapter_factory/robot_adapter_factory.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <regex>
#include <iomanip>

// JSON支持 (如果可用)
#ifdef __has_include
#if __has_include(<nlohmann/json.hpp>)
#include <nlohmann/json.hpp>
#define HAS_JSON_SUPPORT 1
#endif
#endif

namespace robot_factory {
namespace adapter_factory {

// ============= 单例模式实现 =============

AdapterRegistry& AdapterRegistry::getInstance() {
    static AdapterRegistry instance;
    return instance;
}

AdapterRegistry::AdapterRegistry() {
    global_stats_.start_time = std::chrono::system_clock::now();
    last_error_.clear();
}

// ============= 创建器注册管理实现 =============

bool AdapterRegistry::registerCreator(RobotType robot_type, 
                                     const std::string& creator_name,
                                     AdapterCreator creator,
                                     const std::string& version,
                                     const std::string& description) {
    std::lock_guard<std::recursive_mutex> lock(registry_mutex_);
    
    // 验证输入参数
    if (!creator) {
        setLastError("Cannot register null creator function");
        return false;
    }
    
    if (creator_name.empty()) {
        setLastError("Creator name cannot be empty");
        return false;
    }
    
    if (!isValidVersion(version)) {
        setLastError("Invalid version format: " + version);
        return false;
    }
    
    // 检查是否已存在
    if (hasCreator(robot_type)) {
        setLastError("Creator for robot type " + robotTypeToString(robot_type) + " already exists");
        return false;
    }
    
    // 创建创建器信息
    auto creator_info = std::make_shared<AdapterCreatorInfo>();
    creator_info->creator = std::move(creator);
    creator_info->creator_name = creator_name;
    creator_info->version = version;
    creator_info->description = description;
    creator_info->registration_time = std::chrono::system_clock::now();
    
    // 注册创建器
    creators_[robot_type] = creator_info;
    
    // 更新统计信息
    global_stats_.total_registrations++;
    
    // 触发事件
    triggerEvent(RegistryEvent::CREATOR_REGISTERED, robot_type, creator_name + " v" + version);
    
    return true;
}

bool AdapterRegistry::registerCreator(RobotType robot_type, const AdapterCreatorInfo& creator_info) {
    std::lock_guard<std::recursive_mutex> lock(registry_mutex_);
    
    // 验证创建器信息
    if (!creator_info.creator) {
        setLastError("Creator function cannot be null");
        return false;
    }
    
    if (creator_info.creator_name.empty()) {
        setLastError("Creator name cannot be empty");
        return false;
    }
    
    if (!isValidVersion(creator_info.version)) {
        setLastError("Invalid version format: " + creator_info.version);
        return false;
    }
    
    // 检查是否已存在
    if (hasCreator(robot_type)) {
        setLastError("Creator for robot type " + robotTypeToString(robot_type) + " already exists");
        return false;
    }
    
    // 创建创建器信息副本
    auto info_copy = std::make_shared<AdapterCreatorInfo>(creator_info);
    info_copy->registration_time = std::chrono::system_clock::now();
    
    // 注册创建器
    creators_[robot_type] = info_copy;
    
    // 更新统计信息
    global_stats_.total_registrations++;
    
    // 触发事件
    triggerEvent(RegistryEvent::CREATOR_REGISTERED, robot_type, 
                creator_info.creator_name + " v" + creator_info.version);
    
    return true;
}

bool AdapterRegistry::unregisterCreator(RobotType robot_type) {
    std::lock_guard<std::recursive_mutex> lock(registry_mutex_);
    
    auto it = creators_.find(robot_type);
    if (it == creators_.end()) {
        setLastError("No creator found for robot type " + robotTypeToString(robot_type));
        return false;
    }
    
    std::string creator_name = it->second->creator_name;
    creators_.erase(it);
    
    // 更新统计信息
    global_stats_.total_unregistrations++;
    
    // 触发事件
    triggerEvent(RegistryEvent::CREATOR_UNREGISTERED, robot_type, creator_name);
    
    return true;
}

bool AdapterRegistry::updateCreatorInfo(RobotType robot_type, const AdapterCreatorInfo& creator_info) {
    std::lock_guard<std::recursive_mutex> lock(registry_mutex_);
    
    auto it = creators_.find(robot_type);
    if (it == creators_.end()) {
        setLastError("No creator found for robot type " + robotTypeToString(robot_type));
        return false;
    }
    
    // 保留原始注册时间和统计信息
    auto old_info = it->second;
    auto new_info = std::make_shared<AdapterCreatorInfo>(creator_info);
    new_info->registration_time = old_info->registration_time;
    new_info->creation_count = old_info->creation_count;
    new_info->success_count = old_info->success_count;
    new_info->last_used_time = old_info->last_used_time;
    
    creators_[robot_type] = new_info;
    
    // 触发事件
    triggerEvent(RegistryEvent::CREATOR_UPDATED, robot_type, creator_info.creator_name);
    
    return true;
}

// ============= 创建器查询功能实现 =============

bool AdapterRegistry::hasCreator(RobotType robot_type) const {
    std::lock_guard<std::recursive_mutex> lock(registry_mutex_);
    return creators_.find(robot_type) != creators_.end();
}

AdapterCreator AdapterRegistry::getCreator(RobotType robot_type) const {
    std::lock_guard<std::recursive_mutex> lock(registry_mutex_);
    
    auto it = creators_.find(robot_type);
    if (it != creators_.end()) {
        // 更新使用时间和统计信息
        it->second->last_used_time = std::chrono::system_clock::now();
        return it->second->creator;
    }
    
    return nullptr;
}

std::shared_ptr<AdapterCreatorInfo> AdapterRegistry::getCreatorInfo(RobotType robot_type) const {
    std::lock_guard<std::recursive_mutex> lock(registry_mutex_);
    
    auto it = creators_.find(robot_type);
    if (it != creators_.end()) {
        return it->second;
    }
    
    return nullptr;
}

std::vector<RobotType> AdapterRegistry::getRegisteredTypes() const {
    std::lock_guard<std::recursive_mutex> lock(registry_mutex_);
    
    std::vector<RobotType> types;
    types.reserve(creators_.size());
    
    for (const auto& pair : creators_) {
        types.push_back(pair.first);
    }
    
    return types;
}

std::map<RobotType, std::string> AdapterRegistry::getCreatorVersions() const {
    std::lock_guard<std::recursive_mutex> lock(registry_mutex_);
    
    std::map<RobotType, std::string> versions;
    for (const auto& pair : creators_) {
        versions[pair.first] = pair.second->version;
    }
    
    return versions;
}

std::map<RobotType, std::pair<int, int>> AdapterRegistry::getCreationStatistics() const {
    std::lock_guard<std::recursive_mutex> lock(registry_mutex_);
    
    std::map<RobotType, std::pair<int, int>> stats;
    for (const auto& pair : creators_) {
        const auto& info = pair.second;
        stats[pair.first] = std::make_pair(info->creation_count, info->success_count);
    }
    
    return stats;
}

// ============= 批量操作实现 =============

void AdapterRegistry::registerDefaultCreators() {
    // 这里注册默认的创建器
    // 实际实现中应该包含Go2、通用机器人等的默认创建器
    
    // Go2机器人创建器 (示例)
    auto go2_creator = [](const AdapterCreationConfig& /* config */) -> IRobotAdapterPtr {
        // 这里应该创建实际的Go2适配器
        // return std::make_shared<Go2RobotAdapter>(config);
        return nullptr; // 暂时返回nullptr作为占位符
    };
    
    AdapterCreatorInfo go2_info;
    go2_info.creator = go2_creator;
    go2_info.creator_name = "DefaultGo2Creator";
    go2_info.version = "1.0.0";
    go2_info.description = "Default Unitree Go2 robot adapter creator";
    go2_info.supports_auto_config = true;
    go2_info.supports_reconnection = true;
    go2_info.required_parameters = {"robot_ip", "network_interface"};
    go2_info.optional_parameters = {"connection_timeout_ms", "enable_debug_logging"};
    
    registerCreator(RobotType::GO2, go2_info);
    
    // 通用机器人创建器
    auto generic_creator = [](const AdapterCreationConfig& /* config */) -> IRobotAdapterPtr {
        // return std::make_shared<GenericRobotAdapter>(config);
        return nullptr; // 暂时返回nullptr作为占位符
    };
    
    AdapterCreatorInfo generic_info;
    generic_info.creator = generic_creator;
    generic_info.creator_name = "DefaultGenericCreator";
    generic_info.version = "1.0.0";
    generic_info.description = "Default generic robot adapter creator";
    generic_info.supports_auto_config = true;
    generic_info.supports_reconnection = false;
    
    registerCreator(RobotType::GENERIC, generic_info);
}

void AdapterRegistry::clearAllCreators() {
    std::lock_guard<std::recursive_mutex> lock(registry_mutex_);
    
    int cleared_count = creators_.size();
    creators_.clear();
    
    // 触发事件
    for (int i = 0; i < cleared_count; ++i) {
        triggerEvent(RegistryEvent::CREATOR_UNREGISTERED, RobotType::GENERIC, 
                    "Registry cleared (" + std::to_string(cleared_count) + " creators)");
    }
}

int AdapterRegistry::registerMultipleCreators(const std::map<RobotType, AdapterCreatorInfo>& creators) {
    int success_count = 0;
    
    for (const auto& pair : creators) {
        if (registerCreator(pair.first, pair.second)) {
            success_count++;
        }
    }
    
    return success_count;
}

bool AdapterRegistry::exportRegistry(const std::string& file_path, const std::string& format) const {
    std::lock_guard<std::recursive_mutex> lock(registry_mutex_);
    
#ifdef HAS_JSON_SUPPORT
    if (format == "json") {
        nlohmann::json registry_json;
        
        // 导出创建器信息
        for (const auto& pair : creators_) {
            nlohmann::json creator_json;
            const auto& info = pair.second;
            
            creator_json["creator_name"] = info->creator_name;
            creator_json["version"] = info->version;
            creator_json["description"] = info->description;
            creator_json["supports_auto_config"] = info->supports_auto_config;
            creator_json["supports_reconnection"] = info->supports_reconnection;
            creator_json["required_parameters"] = info->required_parameters;
            creator_json["optional_parameters"] = info->optional_parameters;
            creator_json["creation_count"] = info->creation_count;
            creator_json["success_count"] = info->success_count;
            
            registry_json[robotTypeToString(pair.first)] = creator_json;
        }
        
        // 保存到文件
        std::ofstream file(file_path);
        if (file.is_open()) {
            file << registry_json.dump(2);
            file.close();
            return true;
        }
    }
#endif
    
    // 简单的文本格式导出 (备选方案)
    std::ofstream file(file_path);
    if (!file.is_open()) {
        setLastError("Cannot open file for writing: " + file_path);
        return false;
    }
    
    file << "# Robot Adapter Registry Export\n";
    file << "# Generated at: " << getCurrentTimestamp() << "\n\n";
    
    for (const auto& pair : creators_) {
        const auto& info = pair.second;
        file << "[" << robotTypeToString(pair.first) << "]\n";
        file << "creator_name = " << info->creator_name << "\n";
        file << "version = " << info->version << "\n";
        file << "description = " << info->description << "\n";
        file << "creation_count = " << info->creation_count << "\n";
        file << "success_count = " << info->success_count << "\n";
        file << "\n";
    }
    
    file.close();
    return true;
}

bool AdapterRegistry::importRegistry(const std::string& file_path, const std::string& /* format */) {
    // 实际实现中需要解析配置文件并重新注册创建器
    // 这里提供一个基础的框架
    
    std::ifstream file(file_path);
    if (!file.is_open()) {
        setLastError("Cannot open file for reading: " + file_path);
        return false;
    }
    
    // 简单的文本格式解析 (示例)
    std::string line;
    while (std::getline(file, line)) {
        // 跳过注释和空行
        if (line.empty() || line[0] == '#') {
            continue;
        }
        
        // 解析配置项
        // 实际实现中需要更复杂的解析逻辑
    }
    
    file.close();
    return true;
}

// ============= 诊断和验证功能实现 =============

RegistryValidationResult AdapterRegistry::validateAllCreators() const {
    std::lock_guard<std::recursive_mutex> lock(registry_mutex_);
    
    RegistryValidationResult result(true);
    result.total_creators = creators_.size();
    result.active_creators = 0;
    result.implemented_types = 0;
    
    for (const auto& pair : creators_) {
        const auto& info = pair.second;
        
        // 检查创建器函数有效性
        if (!info->creator) {
            result.is_valid = false;
            result.errors.push_back("Creator function is null for " + robotTypeToString(pair.first));
            continue;
        }
        
        // 检查名称有效性
        if (info->creator_name.empty()) {
            result.is_valid = false;
            result.errors.push_back("Creator name is empty for " + robotTypeToString(pair.first));
        }
        
        // 检查版本有效性
        if (!isValidVersion(info->version)) {
            result.is_valid = false;
            result.errors.push_back("Invalid version format for " + robotTypeToString(pair.first) + ": " + info->version);
        }
        
        // 统计活跃创建器 (最近30天内使用过)
        auto now = std::chrono::system_clock::now();
        auto thirty_days_ago = now - std::chrono::hours(24 * 30);
        if (info->last_used_time > thirty_days_ago) {
            result.active_creators++;
        }
        
        result.implemented_types++;
        
        // 生成建议
        if (info->creation_count > 0 && info->success_count == 0) {
            result.warnings.push_back("Creator for " + robotTypeToString(pair.first) + " has never succeeded");
            result.suggestions.push_back("Check creator implementation for " + robotTypeToString(pair.first));
        }
        
        if (info->description.empty()) {
            result.warnings.push_back("No description provided for " + robotTypeToString(pair.first) + " creator");
        }
    }
    
    // 全局检查
    if (creators_.empty()) {
        result.warnings.push_back("No creators registered");
        result.suggestions.push_back("Register default creators using registerDefaultCreators()");
    }
    
    return result;
}

bool AdapterRegistry::testCreator(RobotType robot_type, const AdapterCreationConfig* test_config) const {
    auto creator = getCreator(robot_type);
    if (!creator) {
        return false;
    }
    
    try {
        // 使用提供的配置或默认配置进行测试
        AdapterCreationConfig config;
        if (test_config) {
            config = *test_config;
        }
        
        // 尝试创建适配器 (不一定要成功连接)
        auto adapter = creator(config);
        return adapter != nullptr;
    } catch (const std::exception& e) {
        setLastError("Creator test failed: " + std::string(e.what()));
        return false;
    }
}

std::string AdapterRegistry::getRegistryStatus() const {
    std::lock_guard<std::recursive_mutex> lock(registry_mutex_);
    
    std::ostringstream status;
    status << "{\n";
    status << "  \"total_creators\": " << creators_.size() << ",\n";
    status << "  \"total_registrations\": " << global_stats_.total_registrations << ",\n";
    status << "  \"total_unregistrations\": " << global_stats_.total_unregistrations << ",\n";
    status << "  \"total_creations\": " << global_stats_.total_creations << ",\n";
    status << "  \"total_successes\": " << global_stats_.total_successes << ",\n";
    status << "  \"start_time\": \"" << getCurrentTimestamp() << "\",\n";
    status << "  \"registered_types\": [";
    
    bool first = true;
    for (const auto& pair : creators_) {
        if (!first) status << ", ";
        status << "\"" << robotTypeToString(pair.first) << "\"";
        first = false;
    }
    
    status << "]\n}";
    return status.str();
}

std::string AdapterRegistry::getDiagnosticInfo() const {
    auto validation_result = validateAllCreators();
    
    std::ostringstream diag;
    diag << "{\n";
    diag << "  \"validation\": {\n";
    diag << "    \"is_valid\": " << (validation_result.is_valid ? "true" : "false") << ",\n";
    diag << "    \"total_creators\": " << validation_result.total_creators << ",\n";
    diag << "    \"active_creators\": " << validation_result.active_creators << ",\n";
    diag << "    \"implemented_types\": " << validation_result.implemented_types << ",\n";
    diag << "    \"error_count\": " << validation_result.errors.size() << ",\n";
    diag << "    \"warning_count\": " << validation_result.warnings.size() << "\n";
    diag << "  },\n";
    diag << "  \"status\": " << getRegistryStatus() << "\n";
    diag << "}";
    
    return diag.str();
}

std::map<RobotType, std::map<std::string, double>> AdapterRegistry::getPerformanceStatistics() const {
    std::lock_guard<std::recursive_mutex> lock(registry_mutex_);
    
    std::map<RobotType, std::map<std::string, double>> perf_stats;
    
    for (const auto& pair : creators_) {
        const auto& info = pair.second;
        auto& stats = perf_stats[pair.first];
        
        stats["creation_count"] = static_cast<double>(info->creation_count);
        stats["success_count"] = static_cast<double>(info->success_count);
        stats["success_rate"] = info->creation_count > 0 ? 
            static_cast<double>(info->success_count) / info->creation_count : 0.0;
        
        // 计算注册时长 (小时)
        auto now = std::chrono::system_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::hours>(now - info->registration_time);
        stats["registration_age_hours"] = static_cast<double>(duration.count());
        
        // 计算最后使用时间 (小时前)
        if (info->last_used_time.time_since_epoch().count() > 0) {
            auto last_used_duration = std::chrono::duration_cast<std::chrono::hours>(now - info->last_used_time);
            stats["hours_since_last_used"] = static_cast<double>(last_used_duration.count());
        } else {
            stats["hours_since_last_used"] = -1.0; // 从未使用
        }
    }
    
    return perf_stats;
}

// ============= 事件和回调实现 =============

void AdapterRegistry::setEventCallback(EventCallback callback) {
    std::lock_guard<std::recursive_mutex> lock(registry_mutex_);
    event_callback_ = std::move(callback);
}

void AdapterRegistry::removeEventCallback() {
    std::lock_guard<std::recursive_mutex> lock(registry_mutex_);
    event_callback_ = nullptr;
}

// ============= 工具函数实现 =============

std::string AdapterRegistry::robotTypeToString(RobotType robot_type) {
    switch (robot_type) {
        case RobotType::GENERIC: return "Generic";
        case RobotType::GO2: return "Go2";
        case RobotType::SPOT: return "Spot";
        case RobotType::ANYMAL: return "ANYmal";
        default: return "Unknown";
    }
}

RobotType AdapterRegistry::stringToRobotType(const std::string& type_string) {
    std::string lower_str = type_string;
    std::transform(lower_str.begin(), lower_str.end(), lower_str.begin(), ::tolower);
    
    if (lower_str == "generic") return RobotType::GENERIC;
    if (lower_str == "go2" || lower_str == "unitree_go2") return RobotType::GO2;
    if (lower_str == "spot" || lower_str == "boston_dynamics_spot") return RobotType::SPOT;
    if (lower_str == "anymal" || lower_str == "anybotics_anymal") return RobotType::ANYMAL;
    
    return RobotType::GENERIC; // 默认返回通用类型
}

std::string AdapterRegistry::getLastError() const {
    std::lock_guard<std::recursive_mutex> lock(registry_mutex_);
    return last_error_;
}

void AdapterRegistry::clearErrors() {
    std::lock_guard<std::recursive_mutex> lock(registry_mutex_);
    last_error_.clear();
}

// ============= 私有辅助方法实现 =============

bool AdapterRegistry::validateCreator(const AdapterCreator& creator) const {
    return creator != nullptr;
}

void AdapterRegistry::triggerEvent(RegistryEvent event, RobotType robot_type, const std::string& message) const {
    if (event_callback_) {
        try {
            event_callback_(event, robot_type, message);
        } catch (const std::exception& e) {
            std::cerr << "[AdapterRegistry] Event callback exception: " << e.what() << std::endl;
        }
    }
}

void AdapterRegistry::updateStatistics(RobotType robot_type, bool success) const {
    auto it = creators_.find(robot_type);
    if (it != creators_.end()) {
        it->second->creation_count++;
        if (success) {
            it->second->success_count++;
            global_stats_.total_successes++;
        }
        global_stats_.total_creations++;
    }
}

void AdapterRegistry::setLastError(const std::string& error_message) const {
    last_error_ = error_message;
    std::cerr << "[AdapterRegistry ERROR] " << error_message << std::endl;
}

std::string AdapterRegistry::generateCreatorName(RobotType robot_type) const {
    return robotTypeToString(robot_type) + "Creator";
}

bool AdapterRegistry::isValidVersion(const std::string& version) const {
    // 简单的版本号验证 (x.y.z格式)
    std::regex version_regex(R"(^\d+\.\d+\.\d+$)");
    return std::regex_match(version, version_regex);
}

std::string AdapterRegistry::getCurrentTimestamp() const {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    
    std::ostringstream oss;
    oss << std::put_time(std::gmtime(&time_t), "%Y-%m-%dT%H:%M:%SZ");
    return oss.str();
}

} // namespace adapter_factory
} // namespace robot_factory