/**
 * @file adapter_registry.hpp
 * @brief 适配器注册管理器 - 管理和维护适配器创建器的注册
 * @author Claude Code
 * @date 2024
 * 
 * 该文件实现了适配器注册管理器，负责：
 * 1. 管理适配器创建器的注册和注销
 * 2. 提供适配器创建器的查询和验证功能
 * 3. 维护创建器的版本信息和元数据
 * 4. 提供线程安全的注册表操作
 */

#ifndef ROBOT_FACTORY__ADAPTER_FACTORY__ADAPTER_REGISTRY_HPP_
#define ROBOT_FACTORY__ADAPTER_FACTORY__ADAPTER_REGISTRY_HPP_

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <functional>
#include <mutex>
#include <chrono>

// 条件包含相关接口头文件
#include "robot_factory/adapter_factory/i_robot_adapter.hpp"
#include "robot_base_interfaces/motion_interface/motion_types.hpp"

namespace robot_factory {
namespace adapter_factory {

using RobotType = robot_base_interfaces::motion_interface::RobotType;

// 前向声明
struct AdapterCreationConfig;

/**
 * @brief 适配器创建器函数类型定义
 * 
 * 创建器函数接收创建配置并返回适配器实例的智能指针。
 * 如果创建失败，应该返回nullptr。
 */
using AdapterCreator = std::function<IRobotAdapterPtr(const AdapterCreationConfig&)>;

/**
 * @brief 创建器信息结构体
 * 
 * 包含适配器创建器的详细信息，包括版本、注册时间等元数据。
 */
struct AdapterCreatorInfo {
    AdapterCreator creator;                           ///< 创建器函数
    std::string creator_name;                         ///< 创建器名称
    std::string version;                              ///< 版本号
    std::string description;                          ///< 描述信息
    std::chrono::system_clock::time_point registration_time; ///< 注册时间戳
    
    // 创建器的能力和特性
    bool supports_auto_config = false;               ///< 是否支持自动配置
    bool supports_reconnection = false;              ///< 是否支持自动重连
    std::vector<std::string> required_parameters;    ///< 必需的配置参数
    std::vector<std::string> optional_parameters;    ///< 可选的配置参数
    
    // 统计信息
    mutable int creation_count = 0;                   ///< 创建次数统计
    mutable int success_count = 0;                    ///< 成功创建次数
    mutable std::chrono::system_clock::time_point last_used_time; ///< 最后使用时间
    
    AdapterCreatorInfo() = default;
    AdapterCreatorInfo(AdapterCreator func, const std::string& name, const std::string& ver)
        : creator(std::move(func)), creator_name(name), version(ver) {
        registration_time = std::chrono::system_clock::now();
    }
};

/**
 * @brief 注册验证结果结构体
 */
struct RegistryValidationResult {
    bool is_valid = true;                             ///< 验证是否通过
    std::vector<std::string> errors;                  ///< 错误信息列表
    std::vector<std::string> warnings;               ///< 警告信息列表
    std::vector<std::string> suggestions;            ///< 改进建议列表
    
    // 统计信息
    int total_creators = 0;                           ///< 总创建器数量
    int active_creators = 0;                          ///< 活跃创建器数量
    int implemented_types = 0;                        ///< 已实现的机器人类型数量
    
    RegistryValidationResult() = default;
    RegistryValidationResult(bool valid) : is_valid(valid) {}
};

/**
 * @brief 适配器注册管理器类
 * 
 * 该类实现单例模式，负责管理所有适配器创建器的注册、查询和验证。
 * 提供线程安全的操作，支持动态注册和注销适配器创建器。
 * 
 * 主要功能：
 * 1. 适配器创建器的注册和注销
 * 2. 创建器信息的查询和统计
 * 3. 注册表的验证和诊断
 * 4. 默认创建器的自动注册
 */
class AdapterRegistry {
public:
    /**
     * @brief 获取单例实例
     * @return 注册管理器实例引用
     */
    static AdapterRegistry& getInstance();
    
    // 删除拷贝构造函数和赋值操作符，确保单例
    AdapterRegistry(const AdapterRegistry&) = delete;
    AdapterRegistry& operator=(const AdapterRegistry&) = delete;
    
    // ============= 创建器注册管理 =============
    
    /**
     * @brief 注册适配器创建器
     * @param robot_type 机器人类型
     * @param creator_name 创建器名称
     * @param creator 创建器函数
     * @param version 版本号 (默认为"1.0.0")
     * @param description 描述信息 (可选)
     * @return true if successful, false otherwise
     */
    bool registerCreator(RobotType robot_type, 
                        const std::string& creator_name,
                        AdapterCreator creator,
                        const std::string& version = "1.0.0",
                        const std::string& description = "");
    
    /**
     * @brief 注册适配器创建器 (详细版本)
     * @param robot_type 机器人类型
     * @param creator_info 创建器详细信息
     * @return true if successful, false otherwise
     */
    bool registerCreator(RobotType robot_type, const AdapterCreatorInfo& creator_info);
    
    /**
     * @brief 注销适配器创建器
     * @param robot_type 机器人类型
     * @return true if successful, false otherwise
     */
    bool unregisterCreator(RobotType robot_type);
    
    /**
     * @brief 更新创建器信息
     * @param robot_type 机器人类型
     * @param creator_info 新的创建器信息
     * @return true if successful, false otherwise
     */
    bool updateCreatorInfo(RobotType robot_type, const AdapterCreatorInfo& creator_info);
    
    // ============= 创建器查询功能 =============
    
    /**
     * @brief 检查是否有指定类型的创建器
     * @param robot_type 机器人类型
     * @return true if exists, false otherwise
     */
    bool hasCreator(RobotType robot_type) const;
    
    /**
     * @brief 获取适配器创建器
     * @param robot_type 机器人类型
     * @return 创建器函数，如果不存在返回nullptr
     */
    AdapterCreator getCreator(RobotType robot_type) const;
    
    /**
     * @brief 获取创建器详细信息
     * @param robot_type 机器人类型
     * @return 创建器信息指针，如果不存在返回nullptr
     */
    std::shared_ptr<AdapterCreatorInfo> getCreatorInfo(RobotType robot_type) const;
    
    /**
     * @brief 获取所有已注册的机器人类型
     * @return 机器人类型列表
     */
    std::vector<RobotType> getRegisteredTypes() const;
    
    /**
     * @brief 获取创建器版本信息
     * @return 类型到版本的映射表
     */
    std::map<RobotType, std::string> getCreatorVersions() const;
    
    /**
     * @brief 获取创建器统计信息
     * @return 类型到统计信息的映射表
     */
    std::map<RobotType, std::pair<int, int>> getCreationStatistics() const; // <total_count, success_count>
    
    // ============= 批量操作 =============
    
    /**
     * @brief 注册所有默认创建器
     * 
     * 该函数会注册所有已实现机器人类型的默认创建器，
     * 包括Go2、通用机器人等。
     */
    void registerDefaultCreators();
    
    /**
     * @brief 清除所有创建器
     * @warning 谨慎使用，会清除所有注册的创建器
     */
    void clearAllCreators();
    
    /**
     * @brief 批量注册创建器
     * @param creators 创建器映射表
     * @return 成功注册的数量
     */
    int registerMultipleCreators(const std::map<RobotType, AdapterCreatorInfo>& creators);
    
    /**
     * @brief 导出注册表配置
     * @param file_path 导出文件路径
     * @param format 导出格式 ("json", "yaml")
     * @return true if successful, false otherwise
     */
    bool exportRegistry(const std::string& file_path, const std::string& format = "json") const;
    
    /**
     * @brief 导入注册表配置
     * @param file_path 配置文件路径
     * @param format 文件格式 ("json", "yaml", "auto")
     * @return true if successful, false otherwise
     */
    bool importRegistry(const std::string& file_path, const std::string& format = "auto");
    
    // ============= 诊断和验证功能 =============
    
    /**
     * @brief 验证所有创建器
     * @return 验证结果
     */
    RegistryValidationResult validateAllCreators() const;
    
    /**
     * @brief 测试特定创建器
     * @param robot_type 机器人类型
     * @param test_config 测试配置 (可选)
     * @return 测试是否成功
     */
    bool testCreator(RobotType robot_type, const AdapterCreationConfig* test_config = nullptr) const;
    
    /**
     * @brief 获取注册表状态摘要
     * @return 状态信息 (JSON格式字符串)
     */
    std::string getRegistryStatus() const;
    
    /**
     * @brief 获取详细的诊断信息
     * @return 诊断信息 (JSON格式字符串)
     */
    std::string getDiagnosticInfo() const;
    
    /**
     * @brief 获取创建器性能统计
     * @return 性能统计信息
     */
    std::map<RobotType, std::map<std::string, double>> getPerformanceStatistics() const;
    
    // ============= 事件和回调 =============
    
    /**
     * @brief 注册器事件回调类型
     */
    enum class RegistryEvent {
        CREATOR_REGISTERED,     ///< 创建器已注册
        CREATOR_UNREGISTERED,   ///< 创建器已注销
        CREATOR_UPDATED,        ///< 创建器已更新
        CREATOR_USED,          ///< 创建器被使用
        CREATOR_FAILED         ///< 创建器执行失败
    };
    
    /**
     * @brief 事件回调函数类型
     */
    using EventCallback = std::function<void(RegistryEvent event, RobotType robot_type, const std::string& message)>;
    
    /**
     * @brief 设置事件回调
     * @param callback 回调函数
     */
    void setEventCallback(EventCallback callback);
    
    /**
     * @brief 移除事件回调
     */
    void removeEventCallback();
    
    // ============= 工具函数 =============
    
    /**
     * @brief 机器人类型转字符串
     * @param robot_type 机器人类型
     * @return 类型字符串
     */
    static std::string robotTypeToString(RobotType robot_type);
    
    /**
     * @brief 字符串转机器人类型
     * @param type_string 类型字符串
     * @return 机器人类型
     */
    static RobotType stringToRobotType(const std::string& type_string);
    
    /**
     * @brief 获取注册管理器版本
     * @return 版本字符串
     */
    std::string getVersion() const { return "1.0.0"; }
    
    /**
     * @brief 获取最后的错误信息
     * @return 错误信息
     */
    std::string getLastError() const;
    
    /**
     * @brief 清除错误状态
     */
    void clearErrors();

protected:
    /**
     * @brief 构造函数 (受保护，单例模式)
     */
    AdapterRegistry();
    
    /**
     * @brief 析构函数
     */
    virtual ~AdapterRegistry() = default;

private:
    // ============= 私有成员变量 =============
    
    /// 创建器存储 (机器人类型 -> 创建器信息)
    std::map<RobotType, std::shared_ptr<AdapterCreatorInfo>> creators_;
    
    /// 线程安全互斥锁
    mutable std::recursive_mutex registry_mutex_;
    
    /// 事件回调函数
    EventCallback event_callback_;
    
    /// 错误信息
    mutable std::string last_error_;
    
    /// 全局统计信息
    mutable struct {
        int total_registrations = 0;          ///< 总注册次数
        int total_unregistrations = 0;        ///< 总注销次数
        int total_creations = 0;              ///< 总创建次数
        int total_successes = 0;              ///< 总成功次数
        std::chrono::system_clock::time_point start_time; ///< 启动时间
    } global_stats_;
    
    // ============= 私有辅助方法 =============
    
    /**
     * @brief 验证创建器函数
     * @param creator 创建器函数
     * @return true if valid, false otherwise
     */
    bool validateCreator(const AdapterCreator& creator) const;
    
    /**
     * @brief 触发事件
     * @param event 事件类型
     * @param robot_type 机器人类型
     * @param message 消息内容
     */
    void triggerEvent(RegistryEvent event, RobotType robot_type, const std::string& message = "") const;
    
    /**
     * @brief 更新统计信息
     * @param robot_type 机器人类型
     * @param success 操作是否成功
     */
    void updateStatistics(RobotType robot_type, bool success) const;
    
    /**
     * @brief 设置最后的错误信息
     * @param error_message 错误信息
     */
    void setLastError(const std::string& error_message) const;
    
    /**
     * @brief 生成创建器默认名称
     * @param robot_type 机器人类型
     * @return 默认名称
     */
    std::string generateCreatorName(RobotType robot_type) const;
    
    /**
     * @brief 检查版本号格式
     * @param version 版本号字符串
     * @return true if valid, false otherwise
     */
    bool isValidVersion(const std::string& version) const;
    
    /**
     * @brief 生成时间戳字符串
     * @return ISO 8601格式时间戳
     */
    std::string getCurrentTimestamp() const;
};

/**
 * @brief 适配器注册管理器智能指针类型定义
 */
using AdapterRegistryPtr = std::shared_ptr<AdapterRegistry>;

// ============= 便利宏定义 =============

/**
 * @brief 注册适配器创建器的便利宏
 * 
 * 使用示例:
 * REGISTER_ROBOT_ADAPTER(RobotType::GO2, "Go2Adapter", createGo2Adapter, "1.0.0");
 */
#define REGISTER_ROBOT_ADAPTER(robot_type, name, creator_func, version) \
    do { \
        robot_factory::adapter_factory::AdapterRegistry::getInstance() \
            .registerCreator(robot_type, name, creator_func, version); \
    } while(0)

/**
 * @brief 自动注册适配器创建器的便利宏 (使用默认参数)
 * 
 * 使用示例:
 * AUTO_REGISTER_ROBOT_ADAPTER(RobotType::GO2, createGo2Adapter);
 */
#define AUTO_REGISTER_ROBOT_ADAPTER(robot_type, creator_func) \
    REGISTER_ROBOT_ADAPTER(robot_type, #creator_func, creator_func, "1.0.0")

} // namespace adapter_factory
} // namespace robot_factory

#endif // ROBOT_FACTORY__ADAPTER_FACTORY__ADAPTER_REGISTRY_HPP_