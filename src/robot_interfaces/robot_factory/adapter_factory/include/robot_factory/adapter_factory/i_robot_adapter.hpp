/**
 * @file i_robot_adapter.hpp
 * @brief 机器人适配器基础接口 - 统一适配器规范
 * @author Claude Code
 * @date 2024
 */

#ifndef ROBOT_FACTORY__ADAPTER_FACTORY__I_ROBOT_ADAPTER_HPP_
#define ROBOT_FACTORY__ADAPTER_FACTORY__I_ROBOT_ADAPTER_HPP_

#include "robot_base_interfaces/motion_interface/i_motion_controller.hpp"
#include "robot_base_interfaces/sensor_interface/i_sensor_interface.hpp"
#include "robot_base_interfaces/state_interface/i_state_monitor.hpp"
#include "robot_base_interfaces/power_interface/i_power_manager.hpp"

#include <memory>
#include <string>
#include <map>
#include <vector>
#include <functional>

namespace robot_factory {
namespace adapter_factory {

using namespace robot_base_interfaces;

/**
 * @brief 机器人适配器基础接口
 * 
 * 这是所有具体机器人适配器必须实现的基础接口。
 * 它提供了统一的方式来访问不同机器人的各种功能模块。
 * 
 * 每个具体的机器人（如Go2、Spot等）都需要实现一个继承自此接口的适配器类。
 */
class IRobotAdapter {
public:
    virtual ~IRobotAdapter() = default;

    // ============= 初始化和生命周期管理 =============
    
    /**
     * @brief 初始化机器人适配器
     * @return true if successful, false otherwise
     */
    virtual bool initialize() = 0;
    
    /**
     * @brief 关闭机器人适配器
     * @return true if successful, false otherwise
     */
    virtual bool shutdown() = 0;
    
    /**
     * @brief 检查适配器是否已初始化
     * @return true if initialized, false otherwise
     */
    virtual bool isInitialized() const = 0;
    
    /**
     * @brief 检查适配器是否正常工作
     * @return true if operational, false otherwise
     */
    virtual bool isOperational() const = 0;
    
    // ============= 接口模块访问 =============
    
    /**
     * @brief 获取运动控制器接口
     * @return 运动控制器智能指针
     */
    virtual std::shared_ptr<robot_base_interfaces::motion_interface::IMotionController> getMotionController() = 0;
    
    /**
     * @brief 获取传感器接口
     * @return 传感器接口智能指针
     */
    virtual std::shared_ptr<robot_base_interfaces::sensor_interface::ISensorInterface> getSensorInterface() = 0;
    
    /**
     * @brief 获取状态监控器接口
     * @return 状态监控器智能指针
     */
    virtual std::shared_ptr<robot_base_interfaces::state_interface::IStateMonitor> getStateMonitor() = 0;
    
    /**
     * @brief 获取电源管理器接口
     * @return 电源管理器智能指针
     */
    virtual std::shared_ptr<robot_base_interfaces::power_interface::IPowerManager> getPowerManager() = 0;
    
    // ============= 机器人信息 =============
    
    /**
     * @brief 获取机器人类型
     * @return 机器人类型
     */
    virtual robot_base_interfaces::motion_interface::RobotType getRobotType() const = 0;
    
    /**
     * @brief 获取机器人名称
     * @return 机器人名称字符串
     */
    virtual std::string getRobotName() const = 0;
    
    /**
     * @brief 获取机器人型号
     * @return 机器人型号字符串
     */
    virtual std::string getRobotModel() const = 0;
    
    /**
     * @brief 获取适配器版本
     * @return 版本字符串
     */
    virtual std::string getAdapterVersion() const = 0;
    
    /**
     * @brief 获取机器人固件版本
     * @return 固件版本字符串
     */
    virtual std::string getFirmwareVersion() const = 0;
    
    /**
     * @brief 获取机器人序列号
     * @return 序列号字符串
     */
    virtual std::string getSerialNumber() const = 0;
    
    // ============= 能力查询 =============
    
    /**
     * @brief 获取机器人运动能力
     * @return 运动能力结构体
     */
    virtual robot_base_interfaces::motion_interface::MotionCapabilities getMotionCapabilities() const = 0;
    
    /**
     * @brief 获取可用传感器列表
     * @return 传感器信息列表
     */
    virtual std::vector<robot_base_interfaces::sensor_interface::SensorInfo> getAvailableSensors() const = 0;
    
    /**
     * @brief 获取支持的充电类型
     * @return 充电类型列表
     */
    virtual std::vector<robot_base_interfaces::power_interface::ChargingType> getSupportedChargingTypes() const = 0;
    
    /**
     * @brief 检查是否支持特定功能
     * @param capability_name 功能名称
     * @return true if supported, false otherwise
     */
    virtual bool hasCapability(const std::string& capability_name) const = 0;
    
    // ============= 配置管理 =============
    
    /**
     * @brief 加载配置文件
     * @param config_file_path 配置文件路径
     * @return true if successful, false otherwise
     */
    virtual bool loadConfiguration(const std::string& config_file_path) = 0;
    
    /**
     * @brief 保存配置到文件
     * @param config_file_path 配置文件路径
     * @return true if successful, false otherwise
     */
    virtual bool saveConfiguration(const std::string& config_file_path) const = 0;
    
    /**
     * @brief 获取配置参数
     * @param parameter_name 参数名
     * @param value 输出参数值
     * @return true if found, false otherwise
     */
    virtual bool getConfigParameter(const std::string& parameter_name, std::string& value) const = 0;
    
    /**
     * @brief 设置配置参数
     * @param parameter_name 参数名
     * @param value 参数值
     * @return true if successful, false otherwise
     */
    virtual bool setConfigParameter(const std::string& parameter_name, const std::string& value) = 0;
    
    // ============= 网络和通信 =============
    
    /**
     * @brief 获取机器人网络地址
     * @return IP地址字符串
     */
    virtual std::string getRobotNetworkAddress() const = 0;
    
    /**
     * @brief 获取通信端口
     * @return 端口号
     */
    virtual int getCommunicationPort() const = 0;
    
    /**
     * @brief 检查与机器人的连接状态
     * @return true if connected, false otherwise
     */
    virtual bool isConnected() const = 0;
    
    /**
     * @brief 建立与机器人的连接
     * @return true if successful, false otherwise
     */
    virtual bool connect() = 0;
    
    /**
     * @brief 断开与机器人的连接
     * @return true if successful, false otherwise
     */
    virtual bool disconnect() = 0;
    
    // ============= 诊断和调试 =============
    
    /**
     * @brief 执行系统自检
     * @return 自检结果，键为模块名，值为检查结果
     */
    virtual std::map<std::string, bool> performSystemCheck() = 0;
    
    /**
     * @brief 获取诊断信息
     * @return 诊断信息 (JSON格式字符串)
     */
    virtual std::string getDiagnosticInfo() const = 0;
    
    /**
     * @brief 获取最后的错误信息
     * @return 错误信息字符串
     */
    virtual std::string getLastError() const = 0;
    
    /**
     * @brief 清除错误状态
     * @return true if successful, false otherwise
     */
    virtual bool clearErrors() = 0;
    
    // ============= 扩展接口 =============
    
    /**
     * @brief 执行自定义命令 - 机器人特定功能扩展
     * @param command_name 命令名称
     * @param parameters 参数 (JSON格式)
     * @return 执行结果 (JSON格式)
     */
    virtual std::string executeCustomCommand(const std::string& command_name,
                                           const std::string& parameters = "") {
        // 默认实现：不支持自定义命令
        (void)command_name;
        (void)parameters;
        return "{\"error\": \"not_supported\", \"message\": \"Custom commands not implemented in base adapter\"}";
    }
    
    /**
     * @brief 获取支持的自定义命令列表
     * @return 命令列表
     */
    virtual std::vector<std::string> getSupportedCustomCommands() const {
        // 默认实现：返回空列表
        return {};
    }
    
    // ============= 事件和回调 =============
    
    /**
     * @brief 设置适配器状态变化回调
     * @param callback 回调函数
     */
    virtual void setAdapterStatusCallback(
        std::function<void(bool is_operational, const std::string& status_msg)> callback) = 0;
    
    /**
     * @brief 设置连接状态变化回调
     * @param callback 回调函数
     */
    virtual void setConnectionStatusCallback(
        std::function<void(bool is_connected, const std::string& connection_info)> callback) = 0;
    
    /**
     * @brief 设置错误事件回调
     * @param callback 回调函数
     */
    virtual void setErrorCallback(
        std::function<void(const std::string& error_msg, int error_code)> callback) = 0;

protected:
    /**
     * @brief 设置最后的错误信息 (供子类使用)
     * @param error_message 错误信息
     * @param error_code 错误代码
     */
    virtual void setLastError(const std::string& error_message, int error_code = -1) = 0;
    
    /**
     * @brief 验证配置参数 (供子类使用)
     * @param config 配置映射
     * @return true if valid, false otherwise
     */
    virtual bool validateConfiguration(const std::map<std::string, std::string>& config) const {
        // 默认实现：总是返回true
        (void)config;
        return true;
    }
};

/**
 * @brief 机器人适配器智能指针类型定义
 */
using IRobotAdapterPtr = std::shared_ptr<IRobotAdapter>;
using IRobotAdapterUniquePtr = std::unique_ptr<IRobotAdapter>;

} // namespace adapter_factory
} // namespace robot_factory

#endif // ROBOT_FACTORY__ADAPTER_FACTORY__I_ROBOT_ADAPTER_HPP_