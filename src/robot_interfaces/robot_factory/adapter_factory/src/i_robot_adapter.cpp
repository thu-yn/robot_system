/**
 * @file i_robot_adapter.cpp
 * @brief 机器人适配器基础接口的实现文件
 * @author Claude Code
 * @date 2024
 * 
 * 该文件提供IRobotAdapter接口的基础实现和辅助函数。
 * 由于大部分接口都是纯虚函数，这里主要包含一些共用的工具函数。
 */

#include "robot_factory/adapter_factory/i_robot_adapter.hpp"
#include <iostream>
#include <sstream>

namespace robot_factory {
namespace adapter_factory {

// IRobotAdapter类的基础实现
// 由于大部分函数都是纯虚函数，这里主要提供一些辅助功能

// 这里可以添加一些通用的辅助函数或默认实现
// 例如配置验证函数等

/**
 * @brief 验证网络地址格式的辅助函数
 * @param address 网络地址
 * @return true if valid, false otherwise
 */
bool validateNetworkAddress(const std::string& address) {
    if (address.empty()) {
        return false;
    }
    
    // 简单的IP地址格式验证
    if (address.find("192.168.") == 0 || 
        address.find("127.0.") == 0 || 
        address == "localhost") {
        return true;
    }
    
    return false;
}

/**
 * @brief 验证端口号的辅助函数
 * @param port 端口号
 * @return true if valid, false otherwise
 */
bool validatePort(int port) {
    return port > 0 && port <= 65535;
}

/**
 * @brief 格式化诊断信息的辅助函数
 * @param module_name 模块名称
 * @param status 状态信息
 * @param details 详细信息
 * @return 格式化的诊断信息字符串
 */
std::string formatDiagnosticInfo(const std::string& module_name, 
                                const std::string& status,
                                const std::vector<std::string>& details) {
    std::ostringstream oss;
    oss << "{\n";
    oss << "  \"module\": \"" << module_name << "\",\n";
    oss << "  \"status\": \"" << status << "\",\n";
    oss << "  \"details\": [";
    
    for (size_t i = 0; i < details.size(); ++i) {
        if (i > 0) oss << ", ";
        oss << "\"" << details[i] << "\"";
    }
    
    oss << "]\n}";
    return oss.str();
}

} // namespace adapter_factory
} // namespace robot_factory