/**
 * @file robot_adapter_factory.hpp
 * @brief 机器人适配器工厂 - 根据机器人类型创建对应的适配器
 * @author Claude Code
 * @date 2024
 */

#ifndef ROBOT_FACTORY__ADAPTER_FACTORY__ROBOT_ADAPTER_FACTORY_HPP_
#define ROBOT_FACTORY__ADAPTER_FACTORY__ROBOT_ADAPTER_FACTORY_HPP_

#include <memory>
#include <string>
#include <map>
#include <functional>
#include <vector>

#include "robot_factory/adapter_factory/i_robot_adapter.hpp"
#include "robot_base_interfaces/motion_interface/motion_types.hpp"
#include "robot_factory/robot_detector/robot_detector.hpp"

namespace robot_factory {
namespace adapter_factory {

using RobotType = robot_base_interfaces::motion_interface::RobotType;
using RobotDetectionResult = robot_factory::robot_detector::RobotDetectionResult;

/**
 * @brief 适配器创建配置
 */
struct AdapterCreationConfig {
    // 网络配置
    std::string robot_ip = "192.168.123.18";       ///< 机器人IP地址
    int robot_port = 8080;                          ///< 机器人端口
    std::string network_interface = "eth0";         ///< 网络接口名称
    int connection_timeout_ms = 5000;               ///< 连接超时时间
    
    // DDS配置 (Go2特有)
    std::string rmw_implementation = "rmw_cyclonedds_cpp";
    std::string cyclonedds_uri;                     ///< CycloneDDS URI配置
    
    // 配置文件路径
    std::string config_file_path;                   ///< 适配器配置文件路径
    
    // 调试选项
    bool enable_debug_logging = false;              ///< 启用调试日志
    bool auto_reconnect = true;                     ///< 自动重连
    int max_reconnect_attempts = 3;                 ///< 最大重连尝试次数
    
    // 扩展配置 (用于特定机器人)
    std::map<std::string, std::string> custom_parameters;
    
    AdapterCreationConfig() {
        // Go2默认DDS配置
        cyclonedds_uri = "<CycloneDDS><Domain><General><Interfaces>"
                        "<NetworkInterface name=\"" + network_interface + 
                        "\" priority=\"default\" multicast=\"default\" />"
                        "</Interfaces></General></Domain></CycloneDDS>";
    }
};

/**
 * @brief 适配器工厂创建结果
 */
struct AdapterCreationResult {
    bool success = false;                           ///< 创建是否成功
    IRobotAdapterPtr adapter;                       ///< 创建的适配器 (成功时有效)
    std::string error_message;                      ///< 错误信息 (失败时有效)
    int error_code = 0;                            ///< 错误代码
    RobotType detected_robot_type = RobotType::GENERIC; ///< 检测到的机器人类型
    std::string adapter_version;                    ///< 适配器版本
    std::string creation_time;                      ///< 创建时间
    
    AdapterCreationResult() = default;
    AdapterCreationResult(bool success, const std::string& error_msg = "")
        : success(success), error_message(error_msg) {}
};

/**
 * @brief 机器人适配器工厂类
 * 
 * 该工厂类负责：
 * 1. 自动检测连接的机器人类型
 * 2. 根据机器人类型创建对应的适配器实例
 * 3. 管理适配器的生命周期
 * 4. 提供适配器的注册和扩展机制
 */
class RobotAdapterFactory {
public:
    /**
     * @brief 适配器创建函数类型定义
     */
    using AdapterCreator = std::function<IRobotAdapterPtr(const AdapterCreationConfig&)>;
    
    RobotAdapterFactory();
    virtual ~RobotAdapterFactory() = default;
    
    // ============= 主要创建接口 =============
    
    /**
     * @brief 自动创建适配器 (自动检测机器人类型)
     * @param config 创建配置
     * @return 创建结果
     */
    AdapterCreationResult createAdapter(const AdapterCreationConfig& config = AdapterCreationConfig());
    
    /**
     * @brief 创建指定类型的适配器
     * @param robot_type 机器人类型
     * @param config 创建配置
     * @return 创建结果
     */
    AdapterCreationResult createAdapter(RobotType robot_type, 
                                       const AdapterCreationConfig& config = AdapterCreationConfig());
    
    /**
     * @brief 从检测结果创建适配器
     * @param detection_result 机器人检测结果
     * @param config 创建配置
     * @return 创建结果
     */
    AdapterCreationResult createAdapterFromDetection(const RobotDetectionResult& detection_result,
                                                    const AdapterCreationConfig& config = AdapterCreationConfig());
    
    /**
     * @brief 创建多个适配器 (多机器人系统)
     * @param configs 每个机器人的配置列表
     * @return 创建结果列表
     */
    std::vector<AdapterCreationResult> createMultipleAdapters(
        const std::vector<std::pair<RobotType, AdapterCreationConfig>>& configs);
    
    // ============= 适配器注册和扩展 =============
    
    /**
     * @brief 注册新的适配器创建器
     * @param robot_type 机器人类型
     * @param creator 创建器函数
     * @return true if successful, false otherwise
     */
    bool registerAdapterCreator(RobotType robot_type, AdapterCreator creator);
    
    /**
     * @brief 注销适配器创建器
     * @param robot_type 机器人类型
     * @return true if successful, false otherwise
     */
    bool unregisterAdapterCreator(RobotType robot_type);
    
    /**
     * @brief 检查是否支持指定机器人类型
     * @param robot_type 机器人类型
     * @return true if supported, false otherwise
     */
    bool isRobotTypeSupported(RobotType robot_type) const;
    
    /**
     * @brief 获取所有支持的机器人类型
     * @return 支持的机器人类型列表
     */
    std::vector<RobotType> getSupportedRobotTypes() const;
    
    // ============= Go2专用创建方法 =============
    
    /**
     * @brief 创建Go2适配器
     * @param config 创建配置
     * @return 创建结果
     */
    AdapterCreationResult createGo2Adapter(const AdapterCreationConfig& config = AdapterCreationConfig());
    
    /**
     * @brief 使用默认Go2配置创建适配器
     * @param robot_ip 机器人IP地址
     * @param network_interface 网络接口名
     * @return 创建结果
     */
    AdapterCreationResult createGo2AdapterWithDefaults(const std::string& robot_ip = "192.168.123.18",
                                                       const std::string& network_interface = "eth0");
    
    // ============= 扩展创建方法 (预留) =============
    
    /**
     * @brief 创建Spot适配器 (预留)
     * @param config 创建配置
     * @return 创建结果
     */
    AdapterCreationResult createSpotAdapter(const AdapterCreationConfig& config = AdapterCreationConfig()) {
        AdapterCreationResult result(false, "Spot adapter not yet implemented");
        result.error_code = -1;
        result.detected_robot_type = RobotType::SPOT;
        return result;
    }
    
    /**
     * @brief 创建ANYmal适配器 (预留)
     * @param config 创建配置  
     * @return 创建结果
     */
    AdapterCreationResult createAnymalAdapter(const AdapterCreationConfig& config = AdapterCreationConfig()) {
        AdapterCreationResult result(false, "ANYmal adapter not yet implemented");
        result.error_code = -1;
        result.detected_robot_type = RobotType::ANYMAL;
        return result;
    }
    
    // ============= 配置和管理 =============
    
    /**
     * @brief 设置默认创建配置
     * @param config 默认配置
     */
    void setDefaultConfig(const AdapterCreationConfig& config);
    
    /**
     * @brief 获取默认创建配置
     * @return 默认配置
     */
    AdapterCreationConfig getDefaultConfig() const;
    
    /**
     * @brief 设置机器人检测器
     * @param detector 机器人检测器
     */
    void setRobotDetector(std::shared_ptr<robot_detector::RobotDetector> detector);
    
    /**
     * @brief 获取机器人检测器
     * @return 机器人检测器
     */
    std::shared_ptr<robot_detector::RobotDetector> getRobotDetector() const;
    
    // ============= 诊断和调试 =============
    
    /**
     * @brief 测试适配器创建 (不实际创建，只验证参数)
     * @param robot_type 机器人类型
     * @param config 创建配置
     * @return 测试结果
     */
    bool testAdapterCreation(RobotType robot_type, const AdapterCreationConfig& config);
    
    /**
     * @brief 获取创建统计信息
     * @return 统计信息 (JSON格式字符串)
     */
    std::string getCreationStatistics() const;
    
    /**
     * @brief 获取最后的错误信息
     * @return 错误信息
     */
    std::string getLastError() const;
    
    /**
     * @brief 清除错误状态
     */
    void clearErrors();
    
    /**
     * @brief 获取工厂版本
     * @return 版本字符串
     */
    std::string getVersion() const { return "1.0.0"; }
    
    // ============= 静态工厂方法 =============
    
    /**
     * @brief 快速创建Go2适配器 (静态方法)
     * @param robot_ip 机器人IP
     * @param network_interface 网络接口
     * @return 适配器智能指针，失败时返回nullptr
     */
    static IRobotAdapterPtr quickCreateGo2Adapter(const std::string& robot_ip = "192.168.123.18",
                                                  const std::string& network_interface = "eth0");
    
    /**
     * @brief 自动检测并创建适配器 (静态方法)
     * @return 适配器智能指针，失败时返回nullptr
     */
    static IRobotAdapterPtr autoCreateAdapter();
    
    /**
     * @brief 获取单例工厂实例
     * @return 工厂实例引用
     */
    static RobotAdapterFactory& getInstance();

private:
    // ============= 私有成员变量 =============
    
    std::map<RobotType, AdapterCreator> adapter_creators_;  ///< 适配器创建器映射
    AdapterCreationConfig default_config_;                  ///< 默认创建配置
    std::shared_ptr<robot_detector::RobotDetector> robot_detector_; ///< 机器人检测器
    
    // 统计和调试信息
    mutable std::string last_error_;                        ///< 最后的错误信息
    mutable std::map<RobotType, int> creation_count_;       ///< 创建计数统计
    mutable std::map<RobotType, int> success_count_;        ///< 成功创建计数
    
    // ============= 私有辅助方法 =============
    
    /**
     * @brief 初始化默认适配器创建器
     */
    void initializeDefaultCreators();
    
    /**
     * @brief 验证创建配置
     * @param config 配置
     * @return true if valid, false otherwise
     */
    bool validateConfig(const AdapterCreationConfig& config) const;
    
    /**
     * @brief 设置环境变量 (DDS配置用)
     * @param config 配置
     */
    void setupEnvironmentVariables(const AdapterCreationConfig& config) const;
    
    /**
     * @brief 更新创建统计
     * @param robot_type 机器人类型
     * @param success 是否成功
     */
    void updateStatistics(RobotType robot_type, bool success) const;
    
    /**
     * @brief 设置最后的错误信息
     * @param error_message 错误信息
     */
    void setLastError(const std::string& error_message) const;
    
    /**
     * @brief 获取当前时间字符串
     * @return 时间字符串
     */
    std::string getCurrentTimeString() const;
};

/**
 * @brief 机器人适配器工厂智能指针类型定义
 */
using RobotAdapterFactoryPtr = std::shared_ptr<RobotAdapterFactory>;

} // namespace adapter_factory
} // namespace robot_factory

#endif // ROBOT_FACTORY__ADAPTER_FACTORY__ROBOT_ADAPTER_FACTORY_HPP_