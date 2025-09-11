/**
 * @file   i_power_manager.hpp  
 * @brief  机器人电源管理抽象接口 - 完全适配Go2，预留扩展
 * @author Yang Nan
 * @date   2025-09-11
 */

#ifndef ROBOT_BASE_INTERFACES__POWER_INTERFACE__I_POWER_MANAGER_HPP_
#define ROBOT_BASE_INTERFACES__POWER_INTERFACE__I_POWER_MANAGER_HPP_

#include "robot_base_interfaces/power_interface/power_types.hpp"
#include "robot_base_interfaces/common/result.hpp"
#include <functional>
#include <memory>
#include <vector>

namespace robot_base_interfaces {
namespace power_interface {

/**
 * @brief 电源管理器抽象接口
 * 
 * 该接口设计覆盖Go2的完整电源管理能力：
 * - 电池状态监控 (基于BmsState)
 * - 充电管理和控制
 * - 功耗优化和管理
 * - 自主回充决策支持
 * - 电源安全保护
 * 
 * 其他机器人可通过实现此接口来适配统一的电源管理系统
 */
class IPowerManager {
public:
    virtual ~IPowerManager() = default;

    // ============= 初始化和配置 =============
    
    /**
     * @brief 初始化电源管理器
     * @return true if successful, false otherwise
     */
    virtual bool initialize() = 0;
    
    /**
     * @brief 关闭电源管理器
     * @return true if successful, false otherwise
     */
    virtual bool shutdown() = 0;
    
    /**
     * @brief 获取支持的充电类型
     * @return 充电类型列表
     */
    virtual std::vector<ChargingType> getSupportedChargingTypes() const = 0;
    
    /**
     * @brief 检查电源管理器是否正常工作
     * @return true if operational, false otherwise
     */
    virtual bool isOperational() const = 0;
    
    // ============= 电池状态查询 =============
    
    /**
     * @brief 获取电池信息
     * @return 详细电池信息
     */
    virtual BatteryInfo getBatteryInfo() const = 0;
    
    /**
     * @brief 获取电池剩余电量百分比
     * @return 电量百分比 (0.0-100.0)
     */
    virtual float getBatteryPercentage() const = 0;
    
    /**
     * @brief 获取电池电压
     * @return 电压 (V)
     */
    virtual float getBatteryVoltage() const = 0;
    
    /**
     * @brief 获取电池电流
     * @return 电流 (A, 正值为充电，负值为放电)
     */
    virtual float getBatteryCurrent() const = 0;
    
    /**
     * @brief 获取电池功率
     * @return 功率 (W, 正值为充电，负值为放电)
     */
    virtual float getBatteryPower() const = 0;
    
    /**
     * @brief 获取电池温度
     * @return 温度 (°C)
     */
    virtual float getBatteryTemperature() const = 0;
    
    /**
     * @brief 获取电池健康状态
     * @return 健康状态
     */
    virtual BatteryHealth getBatteryHealth() const = 0;
    
    /**
     * @brief 获取电池循环次数
     * @return 循环次数
     */
    virtual uint16_t getBatteryCycles() const = 0;
    
    /**
     * @brief 预计剩余运行时间
     * @param current_load 当前负载功耗 (W)
     * @return 剩余时间 (分钟)
     */
    virtual float getEstimatedRuntime(float current_load = 50.0f) const = 0;
    
    // ============= 充电管理 =============
    
    /**
     * @brief 获取充电状态
     * @return 充电状态信息
     */
    virtual ChargingStatus getChargingStatus() const = 0;
    
    /**
     * @brief 获取当前充电状态
     * @return 充电状态枚举
     */
    virtual ChargingState getChargingState() const = 0;
    
    /**
     * @brief 请求开始充电
     * @param charging_type 充电类型 (可选，默认使用主要充电类型)
     * @return true if successful, false otherwise
     */
    virtual bool requestCharging(ChargingType charging_type = ChargingType::WIRELESS) = 0;

    /**
     * @brief 请求开始充电（强类型结果）
     */
    virtual robot_base_interfaces::common::Result<bool>
    requestChargingResult(ChargingType charging_type = ChargingType::WIRELESS) {
        bool ok = requestCharging(charging_type);
        if (ok) {
            return robot_base_interfaces::common::Result<bool>::success(true);
        }
        return robot_base_interfaces::common::Result<bool>::failure(
            robot_base_interfaces::common::ErrorCode::UNAVAILABLE, "charging request failed");
    }
    
    /**
     * @brief 请求停止充电
     * @return true if successful, false otherwise
     */
    virtual bool requestStopCharging() = 0;
    
    /**
     * @brief 检查是否正在充电
     * @return true if charging, false otherwise
     */
    virtual bool isCharging() const = 0;
    
    /**
     * @brief 检查是否需要充电
     * @param threshold 电量阈值 (%, 默认使用配置阈值)
     * @return true if needs charging
     */
    virtual bool needsCharging(float threshold = -1.0f) const = 0;
    
    /**
     * @brief 检查电量是否严重不足
     * @return true if critically low
     */
    virtual bool isBatteryCritical() const = 0;
    
    /**
     * @brief 获取预计充满时间
     * @return 充满时间 (分钟)
     */
    virtual uint32_t getEstimatedChargeTime() const = 0;
    
    // ============= 充电站管理 =============
    
    /**
     * @brief 注册充电站
     * @param station_info 充电站信息
     * @return true if successful, false otherwise
     */
    virtual bool registerChargingStation(const ChargingStationInfo& station_info) = 0;
    
    /**
     * @brief 获取已知充电站列表
     * @return 充电站信息列表
     */
    virtual std::vector<ChargingStationInfo> getKnownChargingStations() const = 0;
    
    /**
     * @brief 查找最近的可用充电站
     * @param current_position 当前位置 [x, y, z]
     * @return 最近充电站信息，如果没有返回nullptr
     */
    virtual std::shared_ptr<ChargingStationInfo> findNearestChargingStation(
        const std::vector<float>& current_position) const = 0;
    
    /**
     * @brief 更新充电站状态
     * @param station_id   充电站ID
     * @param is_available 是否可用
     * @param is_occupied  是否被占用
     * @return true if successful, false otherwise
     */
    virtual bool updateChargingStationStatus(const std::string& station_id,
                                            bool is_available,
                                            bool is_occupied) = 0;
    
    // ============= 功耗管理 =============
    
    /**
     * @brief 获取当前功耗
     * @return 当前功耗 (W)
     */
    virtual float getCurrentPowerConsumption() const = 0;
    
    /**
     * @brief 获取平均功耗
     * @param duration_seconds 统计时长 (秒)
     * @return 平均功耗 (W)
     */
    virtual float getAveragePowerConsumption(uint32_t duration_seconds = 60) const = 0;
    
    /**
     * @brief 设置功耗配置文件
     * @param profile 功耗配置文件
     * @return true if successful, false otherwise
     */
    virtual bool setPowerProfile(const PowerConsumptionProfile& profile) = 0;
    
    /**
     * @brief 获取当前功耗配置文件
     * @return 功耗配置文件
     */
    virtual PowerConsumptionProfile getPowerProfile() const = 0;
    
    /**
     * @brief 启用节能模式
     * @param enable 是否启用
     * @return true if successful, false otherwise
     */
    virtual bool enablePowerSaving(bool enable) = 0;
    
    /**
     * @brief 检查是否在节能模式
     * @return true if in power saving mode
     */
    virtual bool isPowerSavingEnabled() const = 0;
    
    // ============= 电源控制 =============
    
    /**
     * @brief 请求系统关机
     * @param delay_seconds 延迟关机时间 (秒)
     * @return true if successful, false otherwise
     */
    virtual bool requestSystemShutdown(uint32_t delay_seconds = 5) = 0;
    
    /**
     * @brief 请求系统重启
     * @param delay_seconds 延迟重启时间 (秒)
     * @return true if successful, false otherwise
     */
    virtual bool requestSystemReboot(uint32_t delay_seconds = 5) = 0;
    
    /**
     * @brief 进入休眠模式
     * @param duration_seconds 休眠时长 (秒)，0表示无限期休眠
     * @return true if successful, false otherwise
     */
    virtual bool enterSleepMode(uint32_t duration_seconds = 0) = 0;
    
    /**
     * @brief 从休眠模式唤醒
     * @return true if successful, false otherwise
     */
    virtual bool wakeFromSleep() = 0;
    
    // ============= 安全和保护 =============
    
    /**
     * @brief 检查电池是否有故障
     * @return true if battery has fault
     */
    virtual bool hasBatteryFault() const = 0;
    
    /**
     * @brief 获取电池故障代码
     * @return 故障代码，0表示无故障
     */
    virtual uint32_t getBatteryFaultCode() const = 0;
    
    /**
     * @brief 检查充电是否安全
     * @return true if safe to charge
     */
    virtual bool isChargingSafe() const = 0;
    
    /**
     * @brief 检查温度是否在安全范围
     * @return true if temperature is safe
     */
    virtual bool isTemperatureSafe() const = 0;
    
    /**
     * @brief 执行电池校准
     * @return true if successful, false otherwise
     */
    virtual bool calibrateBattery() = 0;
    
    /**
     * @brief 重置电池统计信息
     * @return true if successful, false otherwise
     */
    virtual bool resetBatteryStats() = 0;
    
    // ============= 事件和回调 =============
    
    /**
     * @brief 设置电池状态变化回调
     * @param callback 回调函数
     */
    virtual void setBatteryCallback(
        std::function<void(const BatteryInfo&)> callback) = 0;
    
    /**
     * @brief 设置充电状态变化回调
     * @param callback 回调函数
     */
    virtual void setChargingCallback(
        std::function<void(const ChargingStatus&)> callback) = 0;
    
    /**
     * @brief 设置电源事件回调
     * @param callback 回调函数
     */
    virtual void setPowerEventCallback(
        std::function<void(const PowerEventInfo&)> callback) = 0;
    
    /**
     * @brief 设置低电量告警回调
     * @param callback 回调函数
     */
    virtual void setLowBatteryCallback(
        std::function<void(float percentage)> callback) = 0;
    
    /**
     * @brief 设置充电完成回调
     * @param callback 回调函数
     */
    virtual void setChargeCompleteCallback(
        std::function<void()> callback) = 0;
    
    // ============= 配置管理 =============
    
    /**
     * @brief 设置低电量阈值
     * @param threshold 阈值百分比 (0.0-100.0)
     * @return true if successful, false otherwise
     */
    virtual bool setLowBatteryThreshold(float threshold) = 0;
    
    /**
     * @brief 设置严重低电量阈值
     * @param threshold 阈值百分比 (0.0-100.0)
     * @return true if successful, false otherwise
     */
    virtual bool setCriticalBatteryThreshold(float threshold) = 0;
    
    /**
     * @brief 设置自动充电阈值
     * @param threshold 阈值百分比 (0.0-100.0)
     * @return true if successful, false otherwise
     */
    virtual bool setAutoChargeThreshold(float threshold) = 0;
    
    /**
     * @brief 启用或禁用自动充电
     * @param enable 是否启用
     * @return true if successful, false otherwise
     */
    virtual bool enableAutoCharging(bool enable) = 0;
    
    /**
     * @brief 检查是否启用了自动充电
     * @return true if auto charging enabled
     */
    virtual bool isAutoChargingEnabled() const = 0;
    
    // ============= 诊断和统计 =============
    
    /**
     * @brief 获取电源系统诊断信息
     * @return 诊断信息 (JSON格式字符串)
     */
    virtual std::string getPowerDiagnostics() const = 0;
    
    /**
     * @brief 获取充电历史统计
     * @return 充电统计信息 (JSON格式字符串)
     */
    virtual std::string getChargingStatistics() const = 0;
    
    /**
     * @brief 导出电池数据
     * @param file_path 导出文件路径
     * @param format 导出格式 ("json", "csv")
     * @return true if successful, false otherwise
     */
    virtual bool exportBatteryData(const std::string& file_path,
                                  const std::string& format = "json") = 0;
    
    // ============= 扩展接口 =============
    
    /**
     * @brief 执行自定义电源命令 - 扩展接口
     * @param command 命令字符串
     * @param parameters 参数 (JSON格式)
     * @return 执行结果 (JSON格式)
     */
    virtual std::string executeCustomCommand(const std::string& command,
                                           const std::string& parameters = "") {
        // 默认实现：不支持自定义命令
        (void)command;
        (void)parameters;
        return "{\"error\": \"not_supported\"}";
    }
    
    /**
     * @brief 获取电源管理器版本
     * @return 版本字符串
     */
    virtual std::string getVersion() const {
        return "1.0.0";
    }
    
    /**
     * @brief 获取电源管理器配置
     * @return 配置信息 (JSON格式字符串)
     */
    virtual std::string getConfiguration() const = 0;
    
protected:
    /**
     * @brief 触发电源事件
     * @param event 电源事件
     */
    virtual void triggerPowerEvent(const PowerEventInfo& event) = 0;
    
    /**
     * @brief 更新电池信息
     * @param battery_info 新的电池信息
     */
    virtual void updateBatteryInfo(const BatteryInfo& battery_info) = 0;
    
    /**
     * @brief 更新充电状态
     * @param charging_status 新的充电状态
     */
    virtual void updateChargingStatus(const ChargingStatus& charging_status) = 0;
    
    /**
     * @brief 检查并处理安全限制
     * @return true if safe to continue operation
     */
    virtual bool checkSafetyLimits() = 0;
};

/**
 * @brief 电源管理器智能指针类型定义
 */
using IPowerManagerPtr = std::shared_ptr<IPowerManager>;
using IPowerManagerUniquePtr = std::unique_ptr<IPowerManager>;

} // namespace power_interface
} // namespace robot_base_interfaces

#endif // ROBOT_BASE_INTERFACES__POWER_INTERFACE__I_POWER_MANAGER_HPP_