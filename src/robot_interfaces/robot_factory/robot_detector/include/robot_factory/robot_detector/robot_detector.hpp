/**
 * @file robot_detector.hpp
 * @brief 机器人自动检测器 - 自动识别连接的机器人类型
 * @author Claude Code
 * @date 2024
 */

#ifndef ROBOT_FACTORY__ROBOT_DETECTOR__ROBOT_DETECTOR_HPP_
#define ROBOT_FACTORY__ROBOT_DETECTOR__ROBOT_DETECTOR_HPP_

#include <string>
#include <vector>
#include <map>
#include <memory>
#include "robot_base_interfaces/motion_interface/motion_types.hpp"

namespace robot_factory {
namespace robot_detector {

using RobotType = robot_base_interfaces::motion_interface::RobotType;

/**
 * @brief 机器人检测结果结构
 */
struct RobotDetectionResult {
    bool is_detected = false;           ///< 是否检测到机器人
    RobotType robot_type = RobotType::GENERIC; ///< 检测到的机器人类型
    std::string robot_name;             ///< 机器人名称
    std::string robot_model;            ///< 机器人型号
    std::string serial_number;          ///< 序列号
    std::string firmware_version;       ///< 固件版本
    std::string network_address;        ///< 网络地址
    int communication_port = 0;         ///< 通信端口
    float detection_confidence = 0.0f;  ///< 检测置信度 (0.0-1.0)
    
    // 检测方法信息
    std::string detection_method;       ///< 检测方法 (network, environment, config)
    std::vector<std::string> evidence;  ///< 检测证据列表
    uint64_t detection_time_ns = 0;     ///< 检测时间戳
    
    RobotDetectionResult() = default;
    RobotDetectionResult(RobotType type, const std::string& name)
        : is_detected(true), robot_type(type), robot_name(name), detection_confidence(1.0f) {}
};

/**
 * @brief 网络检测配置
 */
struct NetworkDetectionConfig {
    // Go2网络配置
    struct {
        std::vector<std::string> ip_addresses = {"192.168.123.18"};  ///< 可能的IP地址
        std::vector<int> ports = {8080, 8081};                       ///< 可能的端口
        std::vector<std::string> ros2_topics = {                     ///< 特征性ROS2话题
            "/sportmodestate", "/lowstate", "/utlidar/cloud",
            "/api/sport/request", "/wirelesscontroller"
        };
        std::string dds_domain = "cyclonedds";                       ///< DDS域名
        std::string network_interface = "eth0";                      ///< 网络接口
    } go2;
    
    // 其他机器人网络配置 (预留)
    // struct {
    //     std::vector<std::string> ip_addresses = {"192.168.80.3"};
    //     std::vector<int> ports = {443};
    //     std::vector<std::string> ros2_topics = {"/status", "/cmd_vel"};
    // } spot;
    
    // 通用检测配置
    int network_timeout_ms = 3000;      ///< 网络超时时间 (毫秒)
    int max_ping_attempts = 3;          ///< 最大ping尝试次数
    int topic_discovery_timeout_ms = 5000; ///< 话题发现超时时间
};

/**
 * @brief 机器人检测器类
 * 
 * 该类负责自动检测系统中连接的机器人类型。
 * 检测方法包括：
 * 1. 环境变量检测
 * 2. 网络发现 (ping + ROS2话题检测)
 * 3. 配置文件检测
 * 4. 硬件特征检测
 */
class RobotDetector {
public:
    RobotDetector();
    virtual ~RobotDetector() = default;
    
    // ============= 主要检测接口 =============
    
    /**
     * @brief 自动检测机器人类型
     * @return 检测结果
     */
    RobotDetectionResult detectRobot();
    
    /**
     * @brief 检测特定类型的机器人
     * @param target_type 目标机器人类型
     * @return 检测结果
     */
    RobotDetectionResult detectSpecificRobot(RobotType target_type);
    
    /**
     * @brief 检测多个机器人 (支持多机器人系统)
     * @return 检测到的所有机器人列表
     */
    std::vector<RobotDetectionResult> detectAllRobots();
    
    /**
     * @brief 检查特定机器人是否在线
     * @param robot_type 机器人类型
     * @return true if online, false otherwise
     */
    bool isRobotOnline(RobotType robot_type);
    
    // ============= 配置管理 =============
    
    /**
     * @brief 设置网络检测配置
     * @param config 网络检测配置
     */
    void setNetworkDetectionConfig(const NetworkDetectionConfig& config);
    
    /**
     * @brief 获取网络检测配置
     * @return 当前网络检测配置
     */
    NetworkDetectionConfig getNetworkDetectionConfig() const;
    
    /**
     * @brief 启用或禁用特定检测方法
     * @param method_name 检测方法名称
     * @param enabled 是否启用
     */
    void enableDetectionMethod(const std::string& method_name, bool enabled);
    
    /**
     * @brief 设置检测超时时间
     * @param timeout_ms 超时时间 (毫秒)
     */
    void setDetectionTimeout(int timeout_ms);
    
    // ============= 具体检测方法 =============
    
    /**
     * @brief 通过环境变量检测机器人类型
     * @return 检测结果
     */
    RobotDetectionResult detectByEnvironment();
    
    /**
     * @brief 通过网络发现检测机器人
     * @return 检测结果
     */
    RobotDetectionResult detectByNetwork();
    
    /**
     * @brief 通过配置文件检测机器人
     * @param config_file_path 配置文件路径
     * @return 检测结果
     */
    RobotDetectionResult detectByConfigFile(const std::string& config_file_path = "");
    
    /**
     * @brief 通过ROS2话题检测机器人
     * @return 检测结果
     */
    RobotDetectionResult detectByROS2Topics();
    
    // ============= Go2特定检测方法 =============
    
    /**
     * @brief 检测Go2机器人
     * @return 检测结果
     */
    RobotDetectionResult detectGo2Robot();
    
    /**
     * @brief 检查Go2网络连接
     * @param ip_address IP地址
     * @return 连接检测结果
     */
    bool checkGo2NetworkConnection(const std::string& ip_address = "192.168.123.18");
    
    /**
     * @brief 检查Go2特征性ROS2话题
     * @return 话题检测结果
     */
    std::vector<std::string> checkGo2ROS2Topics();
    
    /**
     * @brief 获取Go2机器人信息
     * @param ip_address IP地址
     * @return 机器人详细信息
     */
    std::map<std::string, std::string> getGo2RobotInfo(const std::string& ip_address = "192.168.123.18");
    
    // ============= 扩展检测方法 (预留) =============
    
    /**
     * @brief 检测Spot机器人 (预留)
     * @return 检测结果
     */
    RobotDetectionResult detectSpotRobot() {
        RobotDetectionResult result;
        result.is_detected = false;
        result.detection_method = "not_implemented";
        result.evidence.push_back("Spot detection not yet implemented");
        return result;
    }
    
    /**
     * @brief 检测ANYmal机器人 (预留)
     * @return 检测结果
     */
    RobotDetectionResult detectAnymalRobot() {
        RobotDetectionResult result;
        result.is_detected = false;
        result.detection_method = "not_implemented";
        result.evidence.push_back("ANYmal detection not yet implemented");
        return result;
    }
    
    // ============= 工具方法 =============
    
    /**
     * @brief 将机器人类型转换为字符串
     * @param robot_type 机器人类型
     * @return 类型字符串
     */
    static std::string robotTypeToString(RobotType robot_type);
    
    /**
     * @brief 将字符串转换为机器人类型
     * @param type_string 类型字符串
     * @return 机器人类型
     */
    static RobotType stringToRobotType(const std::string& type_string);
    
    /**
     * @brief 获取所有支持的机器人类型
     * @return 支持的机器人类型列表
     */
    static std::vector<RobotType> getSupportedRobotTypes();
    
    /**
     * @brief 获取检测器版本
     * @return 版本字符串
     */
    std::string getVersion() const { return "1.0.0"; }
    
private:
    // ============= 私有成员变量 =============
    
    NetworkDetectionConfig network_config_;          ///< 网络检测配置
    int detection_timeout_ms_ = 10000;              ///< 检测超时时间
    std::map<std::string, bool> enabled_methods_;   ///< 启用的检测方法
    mutable std::string last_error_;                ///< 最后的错误信息
    
    // 缓存的检测结果
    mutable std::map<RobotType, RobotDetectionResult> detection_cache_;
    mutable uint64_t last_detection_time_ns_ = 0;
    static constexpr uint64_t CACHE_VALIDITY_NS = 30000000000ULL; // 30秒缓存有效期
    
    // ============= 私有辅助方法 =============
    
    /**
     * @brief ping指定的IP地址
     * @param ip_address IP地址
     * @param timeout_ms 超时时间
     * @return true if ping successful
     */
    bool pingAddress(const std::string& ip_address, int timeout_ms = 3000);
    
    /**
     * @brief 检查端口是否开放
     * @param ip_address IP地址
     * @param port 端口号
     * @param timeout_ms 超时时间
     * @return true if port is open
     */
    bool checkPortOpen(const std::string& ip_address, int port, int timeout_ms = 3000);
    
    /**
     * @brief 获取系统环境变量
     * @param var_name 环境变量名
     * @param default_value 默认值
     * @return 环境变量值
     */
    std::string getEnvironmentVariable(const std::string& var_name, 
                                     const std::string& default_value = "") const;
    
    /**
     * @brief 获取当前时间戳 (纳秒)
     * @return 时间戳
     */
    uint64_t getCurrentTimeNanoseconds() const;
    
    /**
     * @brief 检查缓存是否有效
     * @param robot_type 机器人类型
     * @return true if cache is valid
     */
    bool isCacheValid(RobotType robot_type) const;
    
    /**
     * @brief 更新检测缓存
     * @param result 检测结果
     */
    void updateDetectionCache(const RobotDetectionResult& result);
    
    /**
     * @brief 设置最后的错误信息
     * @param error_message 错误信息
     */
    void setLastError(const std::string& error_message) const;
    
    /**
     * @brief 通过CLI命令检测Go2 ROS2话题 (备选方案)
     * @return 检测到的话题列表
     */
    std::vector<std::string> checkGo2ROS2TopicsViaCLI();
};

/**
 * @brief 机器人检测器智能指针类型定义
 */
using RobotDetectorPtr = std::shared_ptr<RobotDetector>;
using RobotDetectorUniquePtr = std::unique_ptr<RobotDetector>;

} // namespace robot_detector
} // namespace robot_factory

#endif // ROBOT_FACTORY__ROBOT_DETECTOR__ROBOT_DETECTOR_HPP_