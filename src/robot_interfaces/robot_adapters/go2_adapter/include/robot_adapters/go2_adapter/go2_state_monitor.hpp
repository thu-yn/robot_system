#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <chrono>
#include <functional>
#include <mutex>
#include <map>
#include <vector>

// 引入统一状态监控接口
#include "robot_base_interfaces/state_interface/i_state_monitor.hpp"
#include "robot_base_interfaces/state_interface/state_types.hpp"

// Go2特定消息类型
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/bms_state.hpp"

#include "robot_adapters/go2_adapter/go2_communication.hpp" // 引入Go2通信管理模块
#include "robot_adapters/go2_adapter/go2_message_converter.hpp" // 引入Go2消息转换器

namespace robot_adapters {
namespace go2_adapter {

/**
 * @class Go2StateMonitor
 * @brief Go2机器人状态监控器实现类
 * 
 * 该类实现了统一状态监控接口，专门用于监控宇树Go2四足机器人
 * 功能包括：
 * - 实时状态监控（SportModeState）
 * - 低级硬件状态监控（LowState）  
 * - 系统健康评估和诊断
 * - 电机和足端状态监控
 * - 告警管理和事件通知
 * - 性能统计和数据记录
 */
class Go2StateMonitor : public robot_base_interfaces::state_interface::IStateMonitor,
                        public rclcpp::Node {
public:
    /**
     * @brief 构造函数
     * @param node_name ROS2节点名称，默认为"go2_state_monitor"
     */
    explicit Go2StateMonitor(const std::string& node_name = "go2_state_monitor");
    
    /**
     * @brief 析构函数，自动清理资源
     */
    ~Go2StateMonitor() override = default;

    // ============= IStateMonitor 接口实现 =============
    
    /**
     * @brief 初始化状态监控器
     * @return true 成功，false 失败
     * 
     * 初始化包括：
     * - 创建Go2状态订阅器
     * - 设置监控频率和阈值  
     * - 启动状态评估线程
     */
    bool initialize() override;
    
    /**
     * @brief 关闭状态监控器
     * @return true 成功，false 失败
     */
    bool shutdown() override;
    
    /**
     * @brief 开始监控
     * @return true 成功，false 失败
     */
    bool startMonitoring() override;
    
    /**
     * @brief 停止监控  
     * @return true 成功，false 失败
     */
    bool stopMonitoring() override;
    
    /**
     * @brief 检查监控器是否在运行
     * @return true 正在监控，false 已停止
     */
    bool isMonitoring() const override;
    
    // ============= 基础状态查询 =============
    
    /**
     * @brief 获取机器人当前状态
     * @return 机器人状态枚举
     */
    robot_base_interfaces::state_interface::RobotState getRobotState() const override;
    
    /**
     * @brief 获取机器人健康状态
     * @return 健康级别枚举
     */
    robot_base_interfaces::state_interface::HealthLevel getHealthStatus() const override;
    
    /**
     * @brief 获取健康评分
     * @return 健康评分 (0.0-1.0)
     */
    float getHealthScore() const override;
    
    /**
     * @brief 检查机器人是否可操作
     * @return true 可操作，false 存在问题
     */
    bool isOperational() const override;
    
    /**
     * @brief 获取当前错误代码
     * @return 错误代码，0表示无错误
     */
    uint32_t getErrorCode() const override;
    
    // ============= 详细状态查询 =============
    
    /**
     * @brief 获取详细机器人状态
     * @return 详细状态信息结构体
     */
    robot_base_interfaces::state_interface::DetailedRobotState getDetailedState() const override;
    
    /**
     * @brief 获取特定电机状态 - Go2有20个电机
     * @param motor_id 电机ID (0-19)
     * @return 电机信息结构体
     */
    robot_base_interfaces::state_interface::MotorInfo getMotorInfo(uint8_t motor_id) const override;
    
    /**
     * @brief 获取所有电机状态
     * @return 所有电机信息向量
     */
    std::vector<robot_base_interfaces::state_interface::MotorInfo> getAllMotorInfo() const override;
    
    /**
     * @brief 获取特定足端状态 - Go2有4个足端
     * @param foot_id 足端ID (0-3)
     * @return 足端信息结构体
     */
    robot_base_interfaces::state_interface::FootInfo getFootInfo(uint8_t foot_id) const override;
    
    /**
     * @brief 获取所有足端状态
     * @return 所有足端信息向量
     */
    std::vector<robot_base_interfaces::state_interface::FootInfo> getAllFootInfo() const override;
    
    // ============= 系统诊断 =============
    
    /**
     * @brief 获取系统诊断信息
     * @return 所有模块的诊断信息向量
     */
    std::vector<robot_base_interfaces::state_interface::DiagnosticInfo> getSystemDiagnostics() const override;
    
    /**
     * @brief 获取特定模块的诊断信息
     * @param module 系统模块枚举
     * @return 模块诊断信息
     */
    robot_base_interfaces::state_interface::DiagnosticInfo getModuleDiagnostic(
        robot_base_interfaces::state_interface::SystemModule module) const override;
    
    /**
     * @brief 执行系统自检
     * @return true 所有系统正常，false 存在问题
     */
    bool performSystemCheck() override;
    
    /**
     * @brief 获取系统自检结果
     * @return 模块自检结果映射
     */
    std::map<robot_base_interfaces::state_interface::SystemModule, bool> getSystemCheckResults() const override;
    
    // ============= 告警管理 =============
    
    /**
     * @brief 获取当前活跃告警
     * @return 活跃告警列表
     */
    std::vector<robot_base_interfaces::state_interface::AlertInfo> getActiveAlerts() const override;
    
    /**
     * @brief 获取特定类型的告警
     * @param type 告警类型
     * @return 指定类型的告警列表  
     */
    std::vector<robot_base_interfaces::state_interface::AlertInfo> getAlertsByType(
        robot_base_interfaces::state_interface::AlertType type) const override;
    
    /**
     * @brief 获取特定模块的告警
     * @param module 系统模块
     * @return 指定模块的告警列表
     */
    std::vector<robot_base_interfaces::state_interface::AlertInfo> getAlertsByModule(
        robot_base_interfaces::state_interface::SystemModule module) const override;
    
    /**
     * @brief 清除已解决的告警
     * @param alert_code 告警代码，0表示清除所有已解决告警
     * @return 清除的告警数量
     */
    int clearResolvedAlerts(uint32_t alert_code = 0) override;
    
    /**
     * @brief 确认告警 (标记为已知但未解决)
     * @param alert_code 告警代码
     * @return true 成功，false 失败
     */
    bool acknowledgeAlert(uint32_t alert_code) override;
    
    // ============= 性能统计 =============
    
    /**
     * @brief 获取性能统计信息
     * @return 性能统计数据结构体
     */
    robot_base_interfaces::state_interface::PerformanceStats getPerformanceStats() const override;
    
    /**
     * @brief 重置性能统计计数器
     * @return true 成功，false 失败
     */
    bool resetPerformanceStats() override;
    
    /**
     * @brief 获取运行时间统计
     * @return 运行时间 (秒)
     */
    uint64_t getUptimeSeconds() const override;
    
    // ============= 回调和事件 =============
    
    /**
     * @brief 设置状态变化回调函数
     * @param callback 回调函数
     */
    void setStateChangeCallback(
        std::function<void(robot_base_interfaces::state_interface::RobotState old_state, 
                          robot_base_interfaces::state_interface::RobotState new_state)> callback) override;
    
    /**
     * @brief 设置健康状态变化回调函数
     * @param callback 回调函数
     */
    void setHealthChangeCallback(
        std::function<void(robot_base_interfaces::state_interface::HealthLevel old_level, 
                          robot_base_interfaces::state_interface::HealthLevel new_level, 
                          float score)> callback) override;
    
    /**
     * @brief 设置新告警回调函数
     * @param callback 回调函数
     */
    void setAlertCallback(
        std::function<void(const robot_base_interfaces::state_interface::AlertInfo& alert)> callback) override;
    
    /**
     * @brief 设置错误事件回调函数
     * @param callback 回调函数
     */
    void setErrorCallback(
        std::function<void(uint32_t error_code, const std::string& error_msg)> callback) override;
    
    /**
     * @brief 设置详细状态更新回调函数
     * @param callback 回调函数
     */
    void setDetailedStateCallback(
        std::function<void(const robot_base_interfaces::state_interface::DetailedRobotState& state)> callback) override;
    
    // ============= 配置管理 =============
    
    /**
     * @brief 设置监控频率
     * @param frequency 监控频率 (Hz)
     * @return true 成功，false 失败
     */
    bool setMonitoringFrequency(float frequency) override;
    
    /**
     * @brief 设置健康评估阈值
     * @param excellent_threshold 优秀阈值 (0.0-1.0)
     * @param good_threshold 良好阈值 (0.0-1.0)
     * @param fair_threshold 一般阈值 (0.0-1.0)
     * @param poor_threshold 较差阈值 (0.0-1.0)
     * @return true 成功，false 失败
     */
    bool setHealthThresholds(float excellent_threshold,
                           float good_threshold,
                           float fair_threshold,
                           float poor_threshold) override;
    
    /**
     * @brief 启用或禁用特定模块的监控
     * @param module 系统模块
     * @param enabled 是否启用
     * @return true 成功，false 失败
     */
    bool setModuleMonitoring(robot_base_interfaces::state_interface::SystemModule module, bool enabled) override;
    
    // ============= 数据记录和导出 =============
    
    /**
     * @brief 开始记录状态数据
     * @param duration_seconds 记录持续时间 (秒)，0表示持续记录
     * @return true 成功，false 失败
     */
    bool startDataRecording(uint32_t duration_seconds = 0) override;
    
    /**
     * @brief 停止记录状态数据
     * @return true 成功，false 失败
     */
    bool stopDataRecording() override;
    
    /**
     * @brief 导出状态数据
     * @param file_path 导出文件路径
     * @param format 导出格式 ("json", "csv", "binary")
     * @return true 成功，false 失败
     */
    bool exportStateData(const std::string& file_path, 
                        const std::string& format = "json") override;
    
    // ============= 扩展接口 =============
    
    /**
     * @brief 获取支持的系统模块列表
     * @return 系统模块列表
     */
    std::vector<robot_base_interfaces::state_interface::SystemModule> getSupportedModules() const override;

protected:
    // ============= IStateMonitor 受保护接口实现 =============
    
    /**
     * @brief 更新机器人状态
     * @param new_state 新状态
     */
    void updateRobotState(robot_base_interfaces::state_interface::RobotState new_state) override;
    
    /**
     * @brief 更新健康状态
     * @param new_level 新健康级别
     * @param score 健康评分
     */
    void updateHealthStatus(robot_base_interfaces::state_interface::HealthLevel new_level, float score) override;
    
    /**
     * @brief 添加新告警
     * @param alert 告警信息
     */
    void addAlert(const robot_base_interfaces::state_interface::AlertInfo& alert) override;
    
    /**
     * @brief 计算健康评分
     * @return 健康评分 (0.0-1.0)
     */
    float calculateHealthScore() const override;

private:
    // ============= 私有成员变量 =============
    
    /** @brief 初始化状态标志 */
    bool is_initialized_;
    
    /** @brief 监控运行标志 */
    bool is_monitoring_;
    
    /** @brief 当前机器人状态，线程安全访问需要加锁 */
    mutable std::mutex state_mutex_;
    robot_base_interfaces::state_interface::RobotState current_state_;
    robot_base_interfaces::state_interface::HealthLevel current_health_level_;
    float current_health_score_;
    uint32_t current_error_code_;
    
    /** @brief 详细状态信息 */
    mutable std::mutex detailed_state_mutex_;
    robot_base_interfaces::state_interface::DetailedRobotState detailed_state_;
    
    /** @brief 监控配置参数 */
    float monitoring_frequency_;
    float excellent_threshold_;
    float good_threshold_;
    float fair_threshold_;  
    float poor_threshold_;
    
    /** @brief 系统启动时间，用于计算运行时间 */
    std::chrono::steady_clock::time_point start_time_;
    
    /** @brief 性能统计数据 */
    mutable std::mutex stats_mutex_;
    robot_base_interfaces::state_interface::PerformanceStats performance_stats_;
    
    /** @brief 告警管理 */
    mutable std::mutex alerts_mutex_;
    std::vector<robot_base_interfaces::state_interface::AlertInfo> active_alerts_;
    uint32_t next_alert_id_;
    
    /** @brief 系统诊断结果 */
    mutable std::mutex diagnostic_mutex_;
    std::map<robot_base_interfaces::state_interface::SystemModule, bool> system_check_results_;
    std::map<robot_base_interfaces::state_interface::SystemModule, bool> module_enabled_;
    
    // ============= ROS2通信组件 =============

    /** @brief Go2运动状态订阅器 */
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sport_state_sub_;

    /** @brief Go2低级状态订阅器 */
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_sub_;

    // 注意：Go2机器人没有独立的BMS状态话题，电池状态信息包含在LowState中

    /** @brief 状态监控定时器 */
    rclcpp::TimerBase::SharedPtr monitoring_timer_;

    // ============= Go2专用组件 =============

    /** @brief Go2通信管理器 */
    std::shared_ptr<Go2Communication> go2_communication_;

    /** @brief Go2消息转换器 */
    std::shared_ptr<Go2MessageConverter> message_converter_;
    
    // ============= 回调函数存储 =============
    
    /** @brief 状态变化回调函数 */
    std::function<void(robot_base_interfaces::state_interface::RobotState, 
                      robot_base_interfaces::state_interface::RobotState)> state_change_callback_;
    
    /** @brief 健康状态变化回调函数 */
    std::function<void(robot_base_interfaces::state_interface::HealthLevel, 
                      robot_base_interfaces::state_interface::HealthLevel, 
                      float)> health_change_callback_;
    
    /** @brief 告警回调函数 */
    std::function<void(const robot_base_interfaces::state_interface::AlertInfo&)> alert_callback_;
    
    /** @brief 错误回调函数 */
    std::function<void(uint32_t, const std::string&)> error_callback_;
    
    /** @brief 详细状态回调函数 */
    std::function<void(const robot_base_interfaces::state_interface::DetailedRobotState&)> detailed_state_callback_;
    
    // ============= 私有方法声明 =============
    
    /**
     * @brief 初始化ROS2发布器和订阅器
     */
    void initializeROS2Communications();
    
    /**
     * @brief Go2运动状态回调函数
     * @param msg 接收到的SportModeState消息
     */
    void sportModeStateCallback(const unitree_go::msg::SportModeState::SharedPtr msg);
    
    /**
     * @brief Go2低级状态回调函数
     * @param msg 接收到的LowState消息
     */
    void lowStateCallback(const unitree_go::msg::LowState::SharedPtr msg);
    
    // 注意：电池状态处理由go2_power_manager.cpp负责，这里不需要相关函数
    
    /**
     * @brief 监控定时器回调函数
     */
    void monitoringTimerCallback();
    
    /**
     * @brief 检查特定模块健康状态
     * @param module 系统模块
     * @return 模块健康状态
     */
    bool checkModuleHealth(robot_base_interfaces::state_interface::SystemModule module) const;
    
    /**
     * @brief 触发状态变化回调
     * @param old_state 旧状态
     * @param new_state 新状态
     */
    void triggerStateChangeCallback(robot_base_interfaces::state_interface::RobotState old_state,
                                   robot_base_interfaces::state_interface::RobotState new_state);
    
    /**
     * @brief 触发健康状态变化回调
     * @param old_level 旧健康级别
     * @param new_level 新健康级别
     * @param score 健康评分
     */
    void triggerHealthChangeCallback(robot_base_interfaces::state_interface::HealthLevel old_level,
                                    robot_base_interfaces::state_interface::HealthLevel new_level,
                                    float score);
    
    /**
     * @brief 触发告警回调
     * @param alert 告警信息
     */
    void triggerAlertCallback(const robot_base_interfaces::state_interface::AlertInfo& alert);
    
    /**
     * @brief 触发错误回调
     * @param error_code 错误代码
     * @param error_msg 错误消息
     */
    void triggerErrorCallback(uint32_t error_code, const std::string& error_msg);
    
    /**
     * @brief 将Go2状态转换为统一格式
     * @param sport_state Go2运动状态
     * @return 统一格式的机器人状态
     */
    robot_base_interfaces::state_interface::RobotState convertGo2StateToRobotState(
        const unitree_go::msg::SportModeState::SharedPtr& sport_state) const;
    
    /**
     * @brief 将Go2电机状态转换为统一格式
     * @param low_state Go2低级状态
     * @return 电机信息向量
     */
    std::vector<robot_base_interfaces::state_interface::MotorInfo> convertGo2MotorInfo(
        const unitree_go::msg::LowState::SharedPtr& low_state) const;
    
    /**
     * @brief 更新足端力信息（从LowState获取）
     * @param low_state Go2低级状态
     */
    void updateFootForceInfo(const unitree_go::msg::LowState::SharedPtr& low_state);
    
    /**
     * @brief 更新足端位置和速度信息（从SportModeState获取）
     * @param sport_state Go2运动状态
     */
    void updateFootPositionAndVelocity(const unitree_go::msg::SportModeState::SharedPtr& sport_state);

    /**
     * @brief 检查运动模式变化并生成相应告警
     * @param motion_state 统一运动状态
     */
    void checkMotionModeAlerts(const robot_base_interfaces::motion_interface::MotionState& motion_state);

    /**
     * @brief 检查电机温度告警
     * @param motors 电机信息列表
     */
    void checkMotorTemperatureAlerts(const std::vector<robot_base_interfaces::state_interface::MotorInfo>& motors);

    /**
     * @brief 检查电池状态告警
     * @param bms_state Go2电池管理状态
     */
    void checkBatteryAlerts(const unitree_go::msg::BmsState& bms_state);
    
    // ============= Go2专用常量定义 =============
    
    /** @brief Go2电机数量 */
    static constexpr uint8_t GO2_MOTOR_COUNT = 20;
    
    /** @brief Go2足端数量 */
    static constexpr uint8_t GO2_FOOT_COUNT = 4;
    
    /** @brief 默认监控频率 (Hz) */
    static constexpr float DEFAULT_MONITORING_FREQUENCY = 10.0f;
    
    /** @brief 默认健康阈值 */
    static constexpr float DEFAULT_EXCELLENT_THRESHOLD = 0.9f;
    static constexpr float DEFAULT_GOOD_THRESHOLD = 0.7f;
    static constexpr float DEFAULT_FAIR_THRESHOLD = 0.5f;
    static constexpr float DEFAULT_POOR_THRESHOLD = 0.3f;
    
    /** @brief 电机温度告警阈值 (摄氏度) */
    static constexpr float MOTOR_TEMP_WARNING_THRESHOLD = 60.0f;
    static constexpr float MOTOR_TEMP_CRITICAL_THRESHOLD = 70.0f;
    
    /** @brief 电池电量告警阈值 (%) */
    static constexpr float BATTERY_LOW_WARNING_THRESHOLD = 20.0f;
    static constexpr float BATTERY_CRITICAL_THRESHOLD = 10.0f;
};

} // namespace go2_adapter
} // namespace robot_adapters