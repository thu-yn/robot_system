/**
 * @file   i_state_monitor.hpp
 * @brief  机器人状态监控抽象接口 - 完全适配Go2，预留扩展
 * @author Yang Nan
 * @date   2025-09-11
 */

#ifndef ROBOT_BASE_INTERFACES__STATE_INTERFACE__I_STATE_MONITOR_HPP_
#define ROBOT_BASE_INTERFACES__STATE_INTERFACE__I_STATE_MONITOR_HPP_

#include "robot_base_interfaces/state_interface/state_types.hpp"
#include <functional>
#include <memory>
#include <vector>

namespace robot_base_interfaces {
namespace state_interface {

/**
 * @brief 状态监控器抽象接口
 * 
 * 该接口设计覆盖Go2的完整状态监控能力：
 * - 实时运动状态监控 (SportModeState)
 * - 低级硬件状态监控 (LowState)
 * - 系统健康诊断
 * - 告警管理
 * - 性能统计
 * 
 * 其他机器人可通过实现此接口来适配统一的状态监控系统
 */
class IStateMonitor {
public:
    virtual ~IStateMonitor() = default;

    // ============= 初始化和配置 =============
    
    /**
     * @brief 初始化状态监控器
     * @return true if successful, false otherwise
     */
    virtual bool initialize() = 0;
    
    /**
     * @brief 关闭状态监控器
     * @return true if successful, false otherwise
     */
    virtual bool shutdown() = 0;
    
    /**
     * @brief 开始监控
     * @return true if successful, false otherwise
     */
    virtual bool startMonitoring() = 0;
    
    /**
     * @brief 停止监控
     * @return true if successful, false otherwise
     */
    virtual bool stopMonitoring() = 0;
    
    /**
     * @brief 检查监控器是否在运行
     * @return true if monitoring, false otherwise
     */
    virtual bool isMonitoring() const = 0;
    
    // ============= 基础状态查询 =============
    
    /**
     * @brief 获取机器人当前状态
     * @return 机器人状态
     */
    virtual RobotState getRobotState() const = 0;
    
    /**
     * @brief 获取机器人健康状态
     * @return 健康级别
     */
    virtual HealthLevel getHealthStatus() const = 0;
    
    /**
     * @brief 获取健康评分
     * @return 健康评分 (0.0-1.0)
     */
    virtual float getHealthScore() const = 0;
    
    /**
     * @brief 检查机器人是否可操作
     * @return true if operational, false otherwise
     */
    virtual bool isOperational() const = 0;
    
    /**
     * @brief 获取当前错误代码
     * @return 错误代码，0表示无错误
     */
    virtual uint32_t getErrorCode() const = 0;
    
    // ============= 详细状态查询 =============
    
    /**
     * @brief 获取详细机器人状态
     * @return 详细状态信息
     */
    virtual DetailedRobotState getDetailedState() const = 0;
    
    /**
     * @brief 获取特定电机状态 - Go2特有
     * @param motor_id 电机ID (0-19)
     * @return 电机信息
     */
    virtual MotorInfo getMotorInfo(uint8_t motor_id) const = 0;
    
    /**
     * @brief 获取所有电机状态 - Go2特有
     * @return 所有电机信息
     */
    virtual std::vector<MotorInfo> getAllMotorInfo() const = 0;
    
    /**
     * @brief 获取特定足端状态 - Go2特有
     * @param foot_id 足端ID (0-3)
     * @return 足端信息
     */
    virtual FootInfo getFootInfo(uint8_t foot_id) const = 0;
    
    /**
     * @brief 获取所有足端状态 - Go2特有
     * @return 所有足端信息
     */
    virtual std::vector<FootInfo> getAllFootInfo() const = 0;
    
    // ============= 系统诊断 =============
    
    /**
     * @brief 获取系统诊断信息
     * @return 所有模块的诊断信息
     */
    virtual std::vector<DiagnosticInfo> getSystemDiagnostics() const = 0;
    
    /**
     * @brief 获取特定模块的诊断信息
     * @param module 系统模块
     * @return 模块诊断信息
     */
    virtual DiagnosticInfo getModuleDiagnostic(SystemModule module) const = 0;
    
    /**
     * @brief 执行系统自检
     * @return true if all systems pass, false otherwise
     */
    virtual bool performSystemCheck() = 0;
    
    /**
     * @brief 获取系统自检结果
     * @return 自检结果详情
     */
    virtual std::map<SystemModule, bool> getSystemCheckResults() const = 0;
    
    // ============= 告警管理 =============
    
    /**
     * @brief 获取当前活跃告警
     * @return 活跃告警列表
     */
    virtual std::vector<AlertInfo> getActiveAlerts() const = 0;
    
    /**
     * @brief 获取特定级别的告警
     * @param type 告警类型
     * @return 指定类型的告警列表
     */
    virtual std::vector<AlertInfo> getAlertsByType(AlertType type) const = 0;
    
    /**
     * @brief 获取特定模块的告警
     * @param module 系统模块
     * @return 指定模块的告警列表
     */
    virtual std::vector<AlertInfo> getAlertsByModule(SystemModule module) const = 0;
    
    /**
     * @brief 清除已解决的告警
     * @param alert_code 告警代码，0表示清除所有已解决告警
     * @return 清除的告警数量
     */
    virtual int clearResolvedAlerts(uint32_t alert_code = 0) = 0;
    
    /**
     * @brief 确认告警 (标记为已知但未解决)
     * @param alert_code 告警代码
     * @return true if successful, false otherwise
     */
    virtual bool acknowledgeAlert(uint32_t alert_code) = 0;
    
    // ============= 性能统计 =============
    
    /**
     * @brief 获取性能统计信息
     * @return 性能统计数据
     */
    virtual PerformanceStats getPerformanceStats() const = 0;
    
    /**
     * @brief 重置性能统计计数器
     * @return true if successful, false otherwise
     */
    virtual bool resetPerformanceStats() = 0;
    
    /**
     * @brief 获取运行时间统计
     * @return 运行时间 (秒)
     */
    virtual uint64_t getUptimeSeconds() const = 0;
    
    // ============= 回调和事件 =============
    
    /**
     * @brief 设置状态变化回调函数
     * @param callback 回调函数
     */
    virtual void setStateChangeCallback(
        std::function<void(RobotState old_state, RobotState new_state)> callback) = 0;
    
    /**
     * @brief 设置健康状态变化回调函数
     * @param callback 回调函数
     */
    virtual void setHealthChangeCallback(
        std::function<void(HealthLevel old_level, HealthLevel new_level, float score)> callback) = 0;
    
    /**
     * @brief 设置新告警回调函数
     * @param callback 回调函数
     */
    virtual void setAlertCallback(
        std::function<void(const AlertInfo& alert)> callback) = 0;
    
    /**
     * @brief 设置错误事件回调函数
     * @param callback 回调函数
     */
    virtual void setErrorCallback(
        std::function<void(uint32_t error_code, const std::string& error_msg)> callback) = 0;
    
    /**
     * @brief 设置详细状态更新回调函数
     * @param callback 回调函数
     */
    virtual void setDetailedStateCallback(
        std::function<void(const DetailedRobotState& state)> callback) = 0;
    
    // ============= 配置管理 =============
    
    /**
     * @brief 设置监控频率
     * @param frequency 监控频率 (Hz)
     * @return true if successful, false otherwise
     */
    virtual bool setMonitoringFrequency(float frequency) = 0;
    
    /**
     * @brief 设置健康评估阈值
     * @param excellent_threshold 优秀阈值 (0.0-1.0)
     * @param good_threshold 良好阈值 (0.0-1.0)
     * @param fair_threshold 一般阈值 (0.0-1.0)
     * @param poor_threshold 较差阈值 (0.0-1.0)
     * @return true if successful, false otherwise
     */
    virtual bool setHealthThresholds(float excellent_threshold,
                                   float good_threshold,
                                   float fair_threshold,
                                   float poor_threshold) = 0;
    
    /**
     * @brief 启用或禁用特定模块的监控
     * @param module 系统模块
     * @param enabled 是否启用
     * @return true if successful, false otherwise
     */
    virtual bool setModuleMonitoring(SystemModule module, bool enabled) = 0;
    
    // ============= 数据记录和导出 =============
    
    /**
     * @brief 开始记录状态数据
     * @param duration_seconds 记录持续时间 (秒)，0表示持续记录
     * @return true if successful, false otherwise
     */
    virtual bool startDataRecording(uint32_t duration_seconds = 0) = 0;
    
    /**
     * @brief 停止记录状态数据
     * @return true if successful, false otherwise
     */
    virtual bool stopDataRecording() = 0;
    
    /**
     * @brief 导出状态数据
     * @param file_path 导出文件路径
     * @param format 导出格式 ("json", "csv", "binary")
     * @return true if successful, false otherwise
     */
    virtual bool exportStateData(const std::string& file_path, 
                                const std::string& format = "json") = 0;
    
    // ============= 扩展接口 =============
    
    /**
     * @brief 执行自定义诊断命令 - 扩展接口
     * @param command 命令字符串
     * @param parameters 参数 (JSON格式)
     * @return 执行结果 (JSON格式)
     */
    virtual std::string executeCustomDiagnostic(const std::string& command,
                                              const std::string& parameters = "") {
        // 默认实现：不支持自定义诊断
        (void)command;
        (void)parameters;
        return "{\"error\": \"not_supported\"}";
    }
    
    /**
     * @brief 获取监控器版本信息
     * @return 版本字符串
     */
    virtual std::string getVersion() const {
        return "1.0.0";
    }
    
    /**
     * @brief 获取支持的系统模块列表
     * @return 系统模块列表
     */
    virtual std::vector<SystemModule> getSupportedModules() const = 0;
    
protected:
    /**
     * @brief 更新机器人状态
     * @param new_state 新状态
     */
    virtual void updateRobotState(RobotState new_state) = 0;
    
    /**
     * @brief 更新健康状态
     * @param new_level 新健康级别
     * @param score 健康评分
     */
    virtual void updateHealthStatus(HealthLevel new_level, float score) = 0;
    
    /**
     * @brief 添加新告警
     * @param alert 告警信息
     */
    virtual void addAlert(const AlertInfo& alert) = 0;
    
    /**
     * @brief 计算健康评分
     * @return 健康评分 (0.0-1.0)
     */
    virtual float calculateHealthScore() const = 0;
};

/**
 * @brief 状态监控器智能指针类型定义
 */
using IStateMonitorPtr = std::shared_ptr<IStateMonitor>;
using IStateMonitorUniquePtr = std::unique_ptr<IStateMonitor>;

} // namespace state_interface
} // namespace robot_base_interfaces

#endif // ROBOT_BASE_INTERFACES__STATE_INTERFACE__I_STATE_MONITOR_HPP_