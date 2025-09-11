/**
 * @file capability_loader.hpp
 * @brief 机器人能力配置加载器
 * @author Claude Code
 * @date 2024
 * 
 * 该文件实现了机器人能力配置的加载、解析、验证和保存功能。
 * 支持多种配置格式(YAML/JSON)，提供配置合并和默认配置生成功能。
 */

#ifndef ROBOT_FACTORY__CAPABILITY_MANAGER__CAPABILITY_LOADER_HPP_
#define ROBOT_FACTORY__CAPABILITY_MANAGER__CAPABILITY_LOADER_HPP_

#include <string>
#include <map>
#include <vector>
#include <memory>
#include <functional>
#include <chrono>

#include "capability_definitions.hpp"
#include "robot_factory/robot_detector/robot_types.hpp"

// ROS2依赖
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// YAML解析库
#ifndef NO_YAML_SUPPORT
#include <yaml-cpp/yaml.h>
#endif

namespace robot_factory {
namespace capability_manager {

/**
 * @brief 配置文件格式枚举
 */
enum class ConfigFormat {
    YAML,           ///< YAML格式 (.yaml, .yml)
    JSON,           ///< JSON格式 (.json) 
    XML,            ///< XML格式 (.xml) - 预留
    AUTO_DETECT     ///< 自动检测格式
};

/**
 * @brief 配置验证结果结构
 */
struct ValidationResult {
    bool is_valid = false;                          ///< 是否验证通过
    int error_count = 0;                           ///< 错误数量
    int warning_count = 0;                         ///< 警告数量
    float confidence_score = 0.0f;                 ///< 置信度评分
    std::vector<std::string> errors;               ///< 错误信息列表
    std::vector<std::string> warnings;             ///< 警告信息列表
    std::string summary;                           ///< 验证摘要
    
    /**
     * @brief 添加错误信息
     */
    void addError(const std::string& error) {
        errors.push_back(error);
        error_count++;
        is_valid = false;
    }
    
    /**
     * @brief 添加警告信息
     */
    void addWarning(const std::string& warning) {
        warnings.push_back(warning);
        warning_count++;
    }
    
    /**
     * @brief 生成验证摘要
     */
    void generateSummary() {
        if (is_valid) {
            summary = "配置验证通过";
            if (warning_count > 0) {
                summary += " (包含 " + std::to_string(warning_count) + " 个警告)";
            }
        } else {
            summary = "配置验证失败: " + std::to_string(error_count) + " 个错误";
            if (warning_count > 0) {
                summary += ", " + std::to_string(warning_count) + " 个警告";
            }
        }
    }
};

/**
 * @brief 配置加载选项结构
 */
struct LoadOptions {
    bool validate_on_load = true;                  ///< 加载时是否验证
    bool merge_with_defaults = true;               ///< 是否与默认配置合并
    bool ignore_unknown_fields = false;           ///< 是否忽略未知字段
    bool strict_mode = false;                      ///< 严格模式(所有字段都必须存在)
    bool cache_results = true;                     ///< 是否缓存加载结果
    std::chrono::seconds cache_expiry{300};       ///< 缓存过期时间(秒)
    std::string encoding = "UTF-8";               ///< 文件编码
    std::vector<std::string> include_paths;       ///< 包含路径列表
};

/**
 * @brief 保存选项结构
 */
struct SaveOptions {
    ConfigFormat format = ConfigFormat::YAML;      ///< 保存格式
    bool pretty_print = true;                      ///< 是否格式化输出
    bool backup_original = true;                   ///< 是否备份原文件
    bool validate_before_save = true;              ///< 保存前是否验证
    int yaml_indent = 2;                          ///< YAML缩进空格数
    std::string encoding = "UTF-8";               ///< 文件编码
    std::string backup_suffix = ".bak";           ///< 备份文件后缀
};

/**
 * @brief 机器人能力配置加载器类
 * 
 * 提供完整的配置文件加载、解析、验证和保存功能。
 * 支持多种配置格式，具备配置合并、缓存和错误处理能力。
 */
class CapabilityLoader {
public:
    /**
     * @brief 构造函数
     * @param logger_name 日志器名称(用于日志输出)
     */
    explicit CapabilityLoader(const std::string& logger_name = "capability_loader");
    
    /**
     * @brief 析构函数
     */
    ~CapabilityLoader() = default;

    // ============= 主要加载接口 =============
    
    /**
     * @brief 从文件加载机器人能力配置
     * @param file_path 配置文件路径
     * @param format 配置格式(AUTO_DETECT自动检测)
     * @param options 加载选项
     * @return 机器人能力配置映射表
     * @throws std::runtime_error 文件不存在或解析失败时抛出异常
     */
    std::map<robot_detector::RobotType, RobotCapabilities> loadCapabilities(
        const std::string& file_path, 
        ConfigFormat format = ConfigFormat::AUTO_DETECT,
        const LoadOptions& options = LoadOptions()
    );
    
    /**
     * @brief 从字符串内容加载机器人能力配置
     * @param config_content 配置内容字符串
     * @param format 配置格式
     * @param capabilities 输出的能力配置映射表
     * @param options 加载选项
     * @return 是否加载成功
     */
    bool loadCapabilitiesFromString(
        const std::string& config_content,
        ConfigFormat format,
        std::map<robot_detector::RobotType, RobotCapabilities>& capabilities,
        const LoadOptions& options = LoadOptions()
    );
    
    /**
     * @brief 从多个配置文件加载并合并
     * @param config_files 配置文件路径列表
     * @param merge_strategy 合并策略("override", "merge", "append")
     * @param options 加载选项
     * @return 合并后的配置映射表
     */
    std::map<robot_detector::RobotType, RobotCapabilities> loadMultipleConfigurations(
        const std::vector<std::string>& config_files,
        const std::string& merge_strategy = "merge",
        const LoadOptions& options = LoadOptions()
    );

    // ============= 配置保存接口 =============
    
    /**
     * @brief 将机器人能力配置保存到文件
     * @param capabilities 能力配置映射表
     * @param file_path 输出文件路径
     * @param options 保存选项
     * @return 是否保存成功
     */
    bool saveCapabilities(
        const std::map<robot_detector::RobotType, RobotCapabilities>& capabilities,
        const std::string& file_path,
        const SaveOptions& options = SaveOptions()
    );
    
    /**
     * @brief 将机器人能力配置转换为字符串
     * @param capabilities 能力配置映射表
     * @param format 输出格式
     * @param pretty_print 是否格式化输出
     * @return 配置内容字符串
     */
    std::string capabilitiesToString(
        const std::map<robot_detector::RobotType, RobotCapabilities>& capabilities,
        ConfigFormat format = ConfigFormat::YAML,
        bool pretty_print = true
    );

    // ============= 配置验证接口 =============
    
    /**
     * @brief 验证配置文件
     * @param file_path 配置文件路径
     * @param format 文件格式(AUTO_DETECT自动检测)
     * @return 验证结果
     */
    ValidationResult validateConfigFile(
        const std::string& file_path,
        ConfigFormat format = ConfigFormat::AUTO_DETECT
    );
    
    /**
     * @brief 验证单个机器人能力配置
     * @param capabilities 机器人能力配置
     * @param robot_type 机器人类型
     * @return 验证结果
     */
    ValidationResult validateCapabilities(
        const RobotCapabilities& capabilities,
        robot_detector::RobotType robot_type = robot_detector::RobotType::UNKNOWN
    );
    
    /**
     * @brief 验证完整的能力配置映射表
     * @param capabilities_map 能力配置映射表
     * @return 验证结果
     */
    ValidationResult validateCapabilitiesMap(
        const std::map<robot_detector::RobotType, RobotCapabilities>& capabilities_map
    );

    // ============= 默认配置和模板生成 =============
    
    /**
     * @brief 生成指定机器人类型的默认能力配置
     * @param robot_type 机器人类型
     * @return 默认能力配置
     */
    RobotCapabilities generateDefaultCapabilities(robot_detector::RobotType robot_type);
    
    /**
     * @brief 生成包含多种机器人类型的默认配置文件
     * @param output_path 输出文件路径
     * @param robot_types 要包含的机器人类型列表
     * @param options 保存选项
     * @return 是否生成成功
     */
    bool generateDefaultConfigFile(
        const std::string& output_path,
        const std::vector<robot_detector::RobotType>& robot_types,
        const SaveOptions& options = SaveOptions()
    );
    
    /**
     * @brief 生成配置文件模板
     * @param template_type 模板类型("minimal", "complete", "example")
     * @param robot_types 包含的机器人类型
     * @return 模板内容字符串
     */
    std::string generateConfigTemplate(
        const std::string& template_type = "complete",
        const std::vector<robot_detector::RobotType>& robot_types = {}
    );

    // ============= 配置合并和转换 =============
    
    /**
     * @brief 合并两个能力配置映射表
     * @param base_config 基础配置
     * @param override_config 覆盖配置
     * @param merge_strategy 合并策略
     * @return 合并后的配置
     */
    std::map<robot_detector::RobotType, RobotCapabilities> mergeConfigurations(
        const std::map<robot_detector::RobotType, RobotCapabilities>& base_config,
        const std::map<robot_detector::RobotType, RobotCapabilities>& override_config,
        const std::string& merge_strategy = "merge"
    );
    
    /**
     * @brief 转换配置文件格式
     * @param input_file 输入文件路径
     * @param output_file 输出文件路径
     * @param output_format 输出格式
     * @return 是否转换成功
     */
    bool convertConfigFormat(
        const std::string& input_file,
        const std::string& output_file,
        ConfigFormat output_format
    );

    // ============= 缓存管理 =============
    
    /**
     * @brief 清理配置缓存
     * @param file_path 文件路径(空表示清理所有缓存)
     */
    void clearCache(const std::string& file_path = "");
    
    /**
     * @brief 获取缓存统计信息
     * @return 缓存统计信息字符串
     */
    std::string getCacheStatistics() const;
    
    /**
     * @brief 设置缓存配置
     * @param enable_cache 是否启用缓存
     * @param cache_size_limit 缓存大小限制
     * @param expiry_seconds 过期时间(秒)
     */
    void configurateCache(bool enable_cache = true, size_t cache_size_limit = 100, 
                         int expiry_seconds = 300);

    // ============= 配置路径管理 =============
    
    /**
     * @brief 添加配置搜索路径
     * @param search_path 搜索路径
     */
    void addSearchPath(const std::string& search_path);
    
    /**
     * @brief 获取所有搜索路径
     * @return 搜索路径列表
     */
    std::vector<std::string> getSearchPaths() const;
    
    /**
     * @brief 查找配置文件
     * @param filename 文件名
     * @return 找到的完整路径(找不到返回空字符串)
     */
    std::string findConfigFile(const std::string& filename) const;
    
    /**
     * @brief 获取默认配置文件路径
     * @param config_name 配置名称("capability_matrix", "default_capabilities")
     * @return 配置文件的完整路径
     */
    std::string getDefaultConfigPath(const std::string& config_name) const;

    // ============= 工具函数 =============
    
    /**
     * @brief 自动检测配置文件格式
     * @param file_path 文件路径
     * @return 检测到的格式
     */
    static ConfigFormat detectConfigFormat(const std::string& file_path);
    
    /**
     * @brief 获取配置格式的文件扩展名
     * @param format 配置格式
     * @return 文件扩展名列表
     */
    static std::vector<std::string> getFormatExtensions(ConfigFormat format);
    
    /**
     * @brief 检查文件是否存在且可读
     * @param file_path 文件路径
     * @return 是否可访问
     */
    static bool isFileAccessible(const std::string& file_path);

    // ============= 回调和事件处理 =============
    
    /**
     * @brief 配置加载完成回调函数类型
     */
    using LoadCompletedCallback = std::function<void(
        const std::string& file_path, 
        const std::map<robot_detector::RobotType, RobotCapabilities>& capabilities,
        const ValidationResult& validation
    )>;
    
    /**
     * @brief 设置配置加载完成回调
     * @param callback 回调函数
     */
    void setLoadCompletedCallback(LoadCompletedCallback callback);
    
    /**
     * @brief 配置验证失败回调函数类型
     */
    using ValidationFailedCallback = std::function<void(
        const std::string& file_path,
        const ValidationResult& validation
    )>;
    
    /**
     * @brief 设置配置验证失败回调
     * @param callback 回调函数
     */
    void setValidationFailedCallback(ValidationFailedCallback callback);

private:
    // ============= 私有成员变量 =============
    
    rclcpp::Logger logger_;                        ///< ROS2日志器
    std::vector<std::string> search_paths_;       ///< 配置搜索路径列表
    
    // 缓存相关
    struct CacheEntry {
        std::map<robot_detector::RobotType, RobotCapabilities> capabilities;
        std::chrono::system_clock::time_point load_time;
        std::chrono::system_clock::time_point expiry_time;
        std::string file_hash;                     ///< 文件内容哈希值
    };
    
    mutable std::map<std::string, CacheEntry> config_cache_;  ///< 配置缓存
    bool cache_enabled_ = true;                    ///< 缓存启用标志
    size_t cache_size_limit_ = 100;               ///< 缓存大小限制
    std::chrono::seconds default_cache_expiry_{300}; ///< 默认缓存过期时间
    
    // 回调函数
    LoadCompletedCallback load_completed_callback_;
    ValidationFailedCallback validation_failed_callback_;
    
    // 统计信息
    mutable size_t cache_hits_ = 0;               ///< 缓存命中次数
    mutable size_t cache_misses_ = 0;             ///< 缓存未命中次数
    mutable size_t total_loads_ = 0;              ///< 总加载次数

    // ============= 私有辅助方法 =============
    
    /**
     * @brief 初始化搜索路径
     */
    void initializeSearchPaths();
    
    /**
     * @brief 从YAML节点解析机器人能力配置
     * @param robot_node YAML节点
     * @param robot_type 机器人类型
     * @return 解析的能力配置
     */
    RobotCapabilities parseCapabilitiesFromYAML(
        const YAML::Node& robot_node, 
        robot_detector::RobotType robot_type
    );
    
    /**
     * @brief 将机器人能力配置转换为YAML节点
     * @param capabilities 能力配置
     * @return YAML节点
     */
    YAML::Node capabilitiesToYAML(const RobotCapabilities& capabilities);
    
    /**
     * @brief 解析运动能力配置
     */
    MotionCapabilities parseMotionCapabilities(const YAML::Node& node);
    
    /**
     * @brief 解析传感器能力配置
     */
    SensorCapabilities parseSensorCapabilities(const YAML::Node& node);
    
    /**
     * @brief 解析电源能力配置
     */
    PowerCapabilities parsePowerCapabilities(const YAML::Node& node);
    
    /**
     * @brief 解析通信能力配置
     */
    CommunicationCapabilities parseCommunicationCapabilities(const YAML::Node& node);
    
    /**
     * @brief 解析处理能力配置
     */
    ProcessingCapabilities parseProcessingCapabilities(const YAML::Node& node);
    
    /**
     * @brief 解析物理属性配置
     */
    PhysicalProperties parsePhysicalProperties(const YAML::Node& node);
    
    /**
     * @brief 将运动能力转换为YAML
     */
    YAML::Node motionCapabilitiesToYAML(const MotionCapabilities& motion);
    
    /**
     * @brief 将传感器能力转换为YAML
     */
    YAML::Node sensorCapabilitiesToYAML(const SensorCapabilities& sensors);
    
    /**
     * @brief 将电源能力转换为YAML
     */
    YAML::Node powerCapabilitiesToYAML(const PowerCapabilities& power);
    
    /**
     * @brief 将通信能力转换为YAML
     */
    YAML::Node communicationCapabilitiesToYAML(const CommunicationCapabilities& comm);
    
    /**
     * @brief 将处理能力转换为YAML
     */
    YAML::Node processingCapabilitiesToYAML(const ProcessingCapabilities& processing);
    
    /**
     * @brief 将物理属性转换为YAML
     */
    YAML::Node physicalPropertiesToYAML(const PhysicalProperties& physical);
    
    /**
     * @brief 检查缓存是否有效
     * @param file_path 文件路径
     * @return 是否有有效缓存
     */
    bool isCacheValid(const std::string& file_path) const;
    
    /**
     * @brief 获取文件内容哈希值
     * @param file_path 文件路径
     * @return 哈希值字符串
     */
    std::string getFileHash(const std::string& file_path) const;
    
    /**
     * @brief 清理过期的缓存项
     */
    void cleanupExpiredCache() const;
    
    /**
     * @brief 验证必需字段
     * @param capabilities 能力配置
     * @param result 验证结果
     */
    void validateRequiredFields(const RobotCapabilities& capabilities, ValidationResult& result);
    
    /**
     * @brief 验证数值范围
     * @param capabilities 能力配置
     * @param result 验证结果
     */
    void validateValueRanges(const RobotCapabilities& capabilities, ValidationResult& result);
    
    /**
     * @brief 验证逻辑一致性
     * @param capabilities 能力配置
     * @param result 验证结果
     */
    void validateLogicalConsistency(const RobotCapabilities& capabilities, ValidationResult& result);
    
    /**
     * @brief 合并单个机器人的能力配置
     * @param base 基础配置
     * @param override 覆盖配置
     * @param strategy 合并策略
     * @return 合并后的配置
     */
    RobotCapabilities mergeSingleCapability(
        const RobotCapabilities& base,
        const RobotCapabilities& override,
        const std::string& strategy
    );
    
    /**
     * @brief 创建默认搜索路径列表
     * @return 搜索路径列表
     */
    std::vector<std::string> createDefaultSearchPaths();
    
    /**
     * @brief 从YAML内容加载机器人能力配置
     * @param yaml_content YAML内容字符串
     * @param options 加载选项
     * @return 机器人能力配置映射表
     */
    std::map<robot_detector::RobotType, RobotCapabilities> loadCapabilitiesFromYAML(
        const std::string& yaml_content,
        const LoadOptions& options
    );
};

} // namespace capability_manager
} // namespace robot_factory

#endif // ROBOT_FACTORY__CAPABILITY_MANAGER__CAPABILITY_LOADER_HPP_