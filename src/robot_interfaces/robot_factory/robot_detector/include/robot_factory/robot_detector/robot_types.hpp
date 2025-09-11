/**
 * @file robot_types.hpp
 * @brief 机器人类型定义和相关常量
 * @author Claude Code
 * @date 2024
 * 
 * 该文件定义了支持的机器人类型枚举、相关常量和工具函数。
 * 为机器人检测和适配器创建提供标准化的类型定义。
 */

#ifndef ROBOT_FACTORY__ROBOT_DETECTOR__ROBOT_TYPES_HPP_
#define ROBOT_FACTORY__ROBOT_DETECTOR__ROBOT_TYPES_HPP_

#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <cctype>

namespace robot_factory {
namespace robot_detector {

// 使用标准的机器人类型定义
#include "robot_base_interfaces/motion_interface/motion_types.hpp"
using RobotType = robot_base_interfaces::motion_interface::RobotType;

/**
 * @brief 机器人类型名称映射表
 * 
 * 提供机器人类型枚举到可读名称的映射关系。
 * 用于日志输出、配置文件解析和用户界面显示。
 * 注意：映射基于标准定义的枚举值
 */
const std::map<RobotType, std::string> ROBOT_TYPE_NAMES = {
    {RobotType::GO2, "Unitree Go2"},
    {RobotType::SPOT, "Boston Dynamics Spot"},
    {RobotType::ANYMAL, "ANYbotics ANYmal"},
    {RobotType::GENERIC, "Generic"}
};

/**
 * @brief 机器人类型简称映射表
 * 
 * 提供简短的机器人类型标识符，用于配置文件和命令行参数。
 */
const std::map<RobotType, std::string> ROBOT_TYPE_SHORT_NAMES = {
    {RobotType::GENERIC, "generic"},
    {RobotType::GO2, "go2"},
    {RobotType::SPOT, "spot"},
    {RobotType::ANYMAL, "anymal"}
};

/**
 * @brief 字符串到机器人类型的映射表
 * 
 * 用于从配置文件或环境变量解析机器人类型。
 * 支持多种命名方式的解析。
 */
const std::map<std::string, RobotType> STRING_TO_ROBOT_TYPE = {
    // 标准名称
    {"generic", RobotType::GENERIC},
    {"go2", RobotType::GO2},
    {"spot", RobotType::SPOT},
    {"anymal", RobotType::ANYMAL},
    
    // 全称别名
    {"unitree_go2", RobotType::GO2},
    {"unitree-go2", RobotType::GO2},
    {"boston_dynamics_spot", RobotType::SPOT},
    {"boston-dynamics-spot", RobotType::SPOT},
    {"anybotics_anymal", RobotType::ANYMAL},
    {"anybotics-anymal", RobotType::ANYMAL},
    
    // 数字形式
    {"0", RobotType::GO2},
    {"1", RobotType::SPOT},
    {"2", RobotType::ANYMAL},
    {"99", RobotType::GENERIC}
};

// ============= Go2机器人特定常量 =============
namespace go2_constants {
    /**
     * @brief Go2机器人网络配置常量
     */
    const std::string DEFAULT_IP = "192.168.123.18";           ///< 默认IP地址
    const int DEFAULT_PORT = 8080;                             ///< 默认通信端口
    const int ALTERNATIVE_PORT = 8081;                         ///< 备用端口
    const std::string DEFAULT_NETWORK_INTERFACE = "eth0";       ///< 默认网络接口
    
    /**
     * @brief Go2机器人特征性ROS2话题列表
     * 
     * 这些话题是Go2机器人的特征性话题，用于机器人类型检测。
     * 检测时会查找这些话题的存在来判断是否为Go2机器人。
     */
    const std::vector<std::string> SIGNATURE_TOPICS = {
        "/sportmodestate",          ///< 运动模式状态话题
        "/lowstate",               ///< 低级状态话题
        "/api/sport/request",      ///< 运动API请求话题
        "/utlidar/cloud",          ///< LiDAR点云话题
        "/wirelesscontroller",     ///< 无线控制器话题
        "/api/sport/response"      ///< 运动API响应话题 (可选)
    };
    
    /**
     * @brief Go2机器人DDS配置常量
     */
    const std::string DEFAULT_DDS_DOMAIN = "cyclonedds";       ///< 默认DDS域名
    const std::string RMW_IMPLEMENTATION = "rmw_cyclonedds_cpp"; ///< RMW实现
    
    /**
     * @brief Go2机器人设备信息常量
     */
    const std::string DEVICE_TYPE = "quadruped";               ///< 设备类型
    const std::string MANUFACTURER = "Unitree";                ///< 制造商
    const std::string MODEL_NAME = "Go2";                      ///< 型号名称
    const std::string DEVICE_FAMILY = "Go";                    ///< 设备系列
    
    /**
     * @brief Go2机器人检测优先级分数
     * 
     * 用于在多种检测方法中评估检测结果的可信度。
     * 分数越高，检测结果越可靠。
     */
    const float NETWORK_DETECTION_CONFIDENCE = 0.9f;          ///< 网络检测置信度
    const float TOPIC_DETECTION_CONFIDENCE = 0.8f;           ///< 话题检测置信度
    const float ENV_DETECTION_CONFIDENCE = 0.95f;            ///< 环境变量检测置信度
    const float CONFIG_DETECTION_CONFIDENCE = 0.7f;          ///< 配置文件检测置信度
    
    /**
     * @brief Go2机器人物理规格常量
     */
    const float LENGTH_M = 0.845f;                            ///< 长度 (米)
    const float WIDTH_M = 0.405f;                             ///< 宽度 (米)  
    const float HEIGHT_M = 0.32f;                             ///< 高度 (米)
    const float WEIGHT_KG = 15.0f;                            ///< 重量 (千克)
    const float MAX_PAYLOAD_KG = 3.0f;                        ///< 最大负载 (千克)
}

// ============= Spot机器人特定常量 (预留) =============
namespace spot_constants {
    const std::string DEFAULT_IP = "192.168.80.3";            ///< 默认IP地址
    const int DEFAULT_PORT = 443;                             ///< 默认HTTPS端口
    const std::string MANUFACTURER = "Boston Dynamics";        ///< 制造商
    const std::string MODEL_NAME = "Spot";                    ///< 型号名称
    
    /**
     * @brief Spot机器人特征性话题 (预留)
     */
    const std::vector<std::string> SIGNATURE_TOPICS = {
        "/status",                 ///< 状态话题
        "/cmd_vel",               ///< 速度命令话题
        "/joint_states",          ///< 关节状态话题
        "/robot_state"            ///< 机器人状态话题
    };
}

// ============= ANYmal机器人特定常量 (预留) =============
namespace anymal_constants {
    const std::string DEFAULT_IP = "192.168.1.11";            ///< 默认IP地址
    const int DEFAULT_PORT = 22000;                           ///< 默认端口
    const std::string MANUFACTURER = "ANYbotics";             ///< 制造商
    const std::string MODEL_NAME = "ANYmal C";                ///< 型号名称
    
    /**
     * @brief ANYmal机器人特征性话题 (预留)
     */
    const std::vector<std::string> SIGNATURE_TOPICS = {
        "/anymal_lowlevel_controller/actuator_commands",
        "/anymal_lowlevel_controller/imu",
        "/state_estimator/pose_in_odom"
    };
}

/**
 * @brief 机器人类型工具函数类
 * 
 * 提供机器人类型相关的工具函数，包括类型转换、验证等功能。
 */
class RobotTypeUtils {
public:
    /**
     * @brief 将机器人类型转换为字符串
     * @param robot_type 机器人类型枚举值
     * @return 对应的字符串名称，未知类型返回"Unknown"
     */
    static std::string robotTypeToString(RobotType robot_type) {
        auto it = ROBOT_TYPE_NAMES.find(robot_type);
        if (it != ROBOT_TYPE_NAMES.end()) {
            return it->second;
        }
        return "Unknown";
    }
    
    /**
     * @brief 将机器人类型转换为简称
     * @param robot_type 机器人类型枚举值
     * @return 对应的简称，未知类型返回"unknown"
     */
    static std::string robotTypeToShortString(RobotType robot_type) {
        auto it = ROBOT_TYPE_SHORT_NAMES.find(robot_type);
        if (it != ROBOT_TYPE_SHORT_NAMES.end()) {
            return it->second;
        }
        return "unknown";
    }
    
    /**
     * @brief 将字符串转换为机器人类型
     * @param type_string 类型字符串 (支持多种格式)
     * @return 对应的机器人类型，无法识别时返回RobotType::UNKNOWN
     */
    static RobotType stringToRobotType(const std::string& type_string) {
        // 转换为小写进行匹配
        std::string lower_str = type_string;
        std::transform(lower_str.begin(), lower_str.end(), lower_str.begin(), ::tolower);
        
        auto it = STRING_TO_ROBOT_TYPE.find(lower_str);
        if (it != STRING_TO_ROBOT_TYPE.end()) {
            return it->second;
        }
        return RobotType::GENERIC;  // 默认返回通用类型
    }
    
    /**
     * @brief 获取所有支持的机器人类型
     * @return 机器人类型列表
     */
    static std::vector<RobotType> getAllSupportedTypes() {
        std::vector<RobotType> types;
        for (const auto& pair : ROBOT_TYPE_NAMES) {
            types.push_back(pair.first);
        }
        return types;
    }
    
    /**
     * @brief 获取所有已实现的机器人类型
     * @return 已实现的机器人类型列表
     */
    static std::vector<RobotType> getImplementedTypes() {
        // 目前只有GO2和GENERIC是已实现的
        return {RobotType::GENERIC, RobotType::GO2};
    }
    
    /**
     * @brief 检查机器人类型是否已实现
     * @param robot_type 机器人类型
     * @return 如果已实现返回true，否则返回false
     */
    static bool isTypeImplemented(RobotType robot_type) {
        auto implemented = getImplementedTypes();
        return std::find(implemented.begin(), implemented.end(), robot_type) != implemented.end();
    }
    
    /**
     * @brief 检查机器人类型是否有效
     * @param robot_type 机器人类型
     * @return 如果是有效类型返回true，否则返回false
     */
    static bool isValidType(RobotType robot_type) {
        return ROBOT_TYPE_NAMES.find(robot_type) != ROBOT_TYPE_NAMES.end();
    }
    
    /**
     * @brief 获取机器人类型的制造商信息
     * @param robot_type 机器人类型
     * @return 制造商名称字符串
     */
    static std::string getRobotManufacturer(RobotType robot_type) {
        switch (robot_type) {
            case RobotType::GO2:
                return "Unitree";
            case RobotType::SPOT:
                return "Boston Dynamics";
            case RobotType::ANYMAL:
                return "ANYbotics";
            case RobotType::GENERIC:
                return "Generic";
            default:
                return "Unknown";
        }
    }
    
    /**
     * @brief 获取机器人类型的默认IP地址
     * @param robot_type 机器人类型
     * @return 默认IP地址字符串
     */
    static std::string getDefaultIPAddress(RobotType robot_type) {
        switch (robot_type) {
            case RobotType::GO2:
                return go2_constants::DEFAULT_IP;
            case RobotType::SPOT:
                return spot_constants::DEFAULT_IP;
            case RobotType::ANYMAL:
                return anymal_constants::DEFAULT_IP;
            default:
                return "127.0.0.1";  // 本地回环地址作为默认值
        }
    }
    
    /**
     * @brief 获取机器人类型的默认通信端口
     * @param robot_type 机器人类型
     * @return 默认端口号
     */
    static int getDefaultPort(RobotType robot_type) {
        switch (robot_type) {
            case RobotType::GO2:
                return go2_constants::DEFAULT_PORT;
            case RobotType::SPOT:
                return spot_constants::DEFAULT_PORT;
            case RobotType::ANYMAL:
                return anymal_constants::DEFAULT_PORT;
            default:
                return 8080;  // 通用默认端口
        }
    }
    
    /**
     * @brief 获取机器人类型的特征性ROS2话题
     * @param robot_type 机器人类型
     * @return 特征性话题列表
     */
    static std::vector<std::string> getSignatureTopics(RobotType robot_type) {
        switch (robot_type) {
            case RobotType::GO2:
                return go2_constants::SIGNATURE_TOPICS;
            case RobotType::SPOT:
                return spot_constants::SIGNATURE_TOPICS;
            case RobotType::ANYMAL:
                return anymal_constants::SIGNATURE_TOPICS;
            default:
                return {};  // 通用类型没有特殊话题
        }
    }
    
    /**
     * @brief 获取机器人类型的检测置信度基准值
     * @param robot_type 机器人类型
     * @param detection_method 检测方法名称
     * @return 置信度分数 (0.0-1.0)
     */
    static float getDetectionConfidence(RobotType robot_type, const std::string& detection_method) {
        if (robot_type != RobotType::GO2) {
            return 0.5f;  // 其他类型的默认置信度
        }
        
        // Go2机器人的置信度评分
        if (detection_method == "environment") {
            return go2_constants::ENV_DETECTION_CONFIDENCE;
        } else if (detection_method == "network") {
            return go2_constants::NETWORK_DETECTION_CONFIDENCE;
        } else if (detection_method == "topics") {
            return go2_constants::TOPIC_DETECTION_CONFIDENCE;
        } else if (detection_method == "config") {
            return go2_constants::CONFIG_DETECTION_CONFIDENCE;
        }
        
        return 0.5f;  // 默认置信度
    }
};

} // namespace robot_detector
} // namespace robot_factory

#endif // ROBOT_FACTORY__ROBOT_DETECTOR__ROBOT_TYPES_HPP_