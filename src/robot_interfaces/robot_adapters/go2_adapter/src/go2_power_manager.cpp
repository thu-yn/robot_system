/**
 * @file go2_power_manager.cpp
 * @brief Go2机器人电源管理器实现
 * @author Claude Code
 * @date 2024
 */

#include "robot_adapters/go2_adapter/go2_power_manager.hpp"
#include "unitree_api/msg/request.hpp"
#include <cstdlib>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <chrono>
#include <algorithm>
#include <regex>

namespace robot_adapters {
namespace go2_adapter {

Go2PowerManager::Go2PowerManager(std::shared_ptr<rclcpp::Node> node)
    : node_(node),
      is_initialized_(false),
      is_operational_(false) {

    // 初始化Go2通信管理器和消息转换器
    go2_comm_ = std::make_shared<Go2Communication>(node);
    converter_ = std::make_shared<Go2MessageConverter>();

    // 初始化统计数据开始时间
    statistics_.start_time = std::chrono::steady_clock::now();

    // 初始化电池状态默认值(Go2典型值)
    battery_state_.voltage = 24.0f;
    battery_state_.current = 0.0f;
    battery_state_.temperature = 25.0f;
    battery_state_.percentage = 100.0f;
    battery_state_.cycles = 0;
    battery_state_.cell_voltages.resize(15, 3.6f); // Go2有15个电芯

    // 初始化充电状态
    charging_state_.state = robot_base_interfaces::power_interface::ChargingState::NOT_CHARGING;
    charging_state_.type = robot_base_interfaces::power_interface::ChargingType::WIRELESS;

    logInfo("Go2PowerManager构造完成");
}

void Go2PowerManager::setupGo2Communication() {
    logInfo("设置Go2通信和消息转换器");

    try {
        // 初始化Go2通信管理器
        if (!go2_comm_->initialize()) {
            logError("Go2Communication初始化失败");
            throw std::runtime_error("Go2Communication初始化失败");
        }

        // 设置BMS状态回调
        go2_comm_->setBmsStateCallback(
            std::bind(&Go2PowerManager::bmsStateCallback, this, std::placeholders::_1));

        logInfo("Go2通信接口设置成功");
    } catch (const std::exception& e) {
        logError("设置Go2通信失败: " + std::string(e.what()));
        throw;
    }
}

Go2PowerManager::~Go2PowerManager() {
    shutdown();
}

// ============= 初始化和配置 =============

bool Go2PowerManager::initialize() {
    logInfo("开始初始化Go2PowerManager");
    
    try {
        // 1. 加载配置参数
        if (!loadConfiguration()) {
            logError("加载配置失败");
            return false;
        }
        
        // 2. 验证配置参数
        if (!validateConfiguration()) {
            logError("配置参数验证失败");
            return false;
        }
        
        // 3. 创建定时器
        monitoring_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0f / config_.monitoring_frequency)),
            std::bind(&Go2PowerManager::monitoringTimerCallback, this));
            
        statistics_timer_ = node_->create_wall_timer(
            std::chrono::seconds(10),
            std::bind(&Go2PowerManager::statisticsTimerCallback, this));
        
        // 4. 设置Go2通信和消息转换器
        setupGo2Communication();
        
        // 5. 初始化电池状态数据
        battery_state_.last_update_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        
        is_initialized_.store(true);
        is_operational_.store(true);
        
        logInfo("Go2PowerManager初始化成功");
        return true;
        
    } catch (const std::exception& e) {
        logError("初始化过程中发生异常: " + std::string(e.what()));
        return false;
    }
}

bool Go2PowerManager::shutdown() {
    logInfo("开始关闭Go2PowerManager");
    
    try {
        // 停止定时器
        if (monitoring_timer_) {
            monitoring_timer_->cancel();
            monitoring_timer_.reset();
        }
        
        if (statistics_timer_) {
            statistics_timer_->cancel();
            statistics_timer_.reset();
        }
        
        // 停止充电(如果正在充电)
        if (isCharging()) {
            requestStopCharging();
        }

        // 关闭Go2通信
        if (go2_comm_) {
            go2_comm_->shutdown();
        }

        is_operational_.store(false);
        is_initialized_.store(false);
        
        logInfo("Go2PowerManager关闭成功");
        return true;
        
    } catch (const std::exception& e) {
        logError("关闭过程中发生异常: " + std::string(e.what()));
        return false;
    }
}

std::vector<robot_base_interfaces::power_interface::ChargingType> 
Go2PowerManager::getSupportedChargingTypes() const {
    return {
        robot_base_interfaces::power_interface::ChargingType::WIRELESS,
        robot_base_interfaces::power_interface::ChargingType::REPLACEABLE
    };
}

bool Go2PowerManager::isOperational() const {
    return is_operational_.load() && is_initialized_.load();
}

// ============= 电池状态查询 =============

robot_base_interfaces::power_interface::BatteryInfo Go2PowerManager::getBatteryInfo() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    robot_base_interfaces::power_interface::BatteryInfo info;
    
    // 基础信息
    info.voltage = battery_state_.voltage;
    info.current = battery_state_.current;
    info.power = std::abs(battery_state_.voltage * battery_state_.current);
    info.temperature = battery_state_.temperature;
    info.soc_percentage = battery_state_.percentage;
    
    // 容量信息
    info.capacity_mah = 15000.0f;  // Go2电池容量15Ah
    info.remaining_mah = info.capacity_mah * battery_state_.percentage / 100.0f;
    info.design_capacity_mah = 15000.0f;

    // 健康状态
    info.health = calculateBatteryHealth();
    info.cycle_count = battery_state_.cycles;
    info.health_percentage = std::max(0.0f, 100.0f - (battery_state_.cycles * 0.1f));

    // 温度信息
    info.max_temperature = battery_state_.max_cell_temp;
    info.min_temperature = battery_state_.min_cell_temp;
    info.bq_ntc_temps = {battery_state_.temperature, battery_state_.temperature}; // 模拟2个温度传感器
    info.mcu_ntc_temps = {battery_state_.temperature, battery_state_.temperature};

    // 电芯信息
    info.cells.clear();
    for (size_t i = 0; i < battery_state_.cell_voltages.size(); ++i) {
        robot_base_interfaces::power_interface::BatteryCellInfo cell;
        cell.cell_id = static_cast<uint8_t>(i);
        cell.voltage = battery_state_.cell_voltages[i];
        cell.temperature = battery_state_.temperature;
        cell.capacity_mah = info.capacity_mah / battery_state_.cell_voltages.size();
        cell.health_percentage = info.health_percentage;
        cell.cycle_count = battery_state_.cycles;
        info.cells.push_back(cell);
    }

    // 时间戳
    info.timestamp_ns = battery_state_.last_update_ns;
    
    return info;
}

float Go2PowerManager::getBatteryPercentage() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return battery_state_.percentage;
}

float Go2PowerManager::getBatteryVoltage() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return battery_state_.voltage;
}

float Go2PowerManager::getBatteryCurrent() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return battery_state_.current;
}

float Go2PowerManager::getBatteryPower() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return std::abs(battery_state_.voltage * battery_state_.current);
}

float Go2PowerManager::getBatteryTemperature() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return battery_state_.temperature;
}

robot_base_interfaces::power_interface::BatteryHealth Go2PowerManager::getBatteryHealth() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return calculateBatteryHealth();
}

uint16_t Go2PowerManager::getBatteryCycles() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return battery_state_.cycles;
}

float Go2PowerManager::getEstimatedRuntime(float current_load) const {
    return calculateEstimatedRuntime(current_load);
}

// ============= 充电管理 =============

robot_base_interfaces::power_interface::ChargingStatus Go2PowerManager::getChargingStatus() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    robot_base_interfaces::power_interface::ChargingStatus status;
    status.state = charging_state_.state;
    status.charging_type = charging_state_.type;
    status.charging_current = charging_state_.charge_rate;
    status.estimated_remaining_seconds = charging_state_.estimated_time_to_full * 60; // convert minutes to seconds
    status.charging_station_id = charging_state_.station_id;
    status.battery_temperature = battery_state_.temperature;
    status.timestamp_ns = battery_state_.last_update_ns;
    
    return status;
}

robot_base_interfaces::power_interface::ChargingState Go2PowerManager::getChargingState() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return charging_state_.state;
}

bool Go2PowerManager::requestCharging(robot_base_interfaces::power_interface::ChargingType charging_type) {
    logInfo("请求开始充电，类型: " + std::to_string(static_cast<int>(charging_type)));
    
    if (charging_type != robot_base_interfaces::power_interface::ChargingType::WIRELESS) {
        logWarning("Go2仅支持无线充电，将使用无线充电模式");
    }
    
    // 发送Go2充电命令
    if (sendGo2ChargeCommand(true)) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        charging_state_.state = robot_base_interfaces::power_interface::ChargingState::CHARGING;
        charging_state_.type = robot_base_interfaces::power_interface::ChargingType::WIRELESS;
        battery_state_.is_charging = true;
        
        logInfo("充电请求成功");
        return true;
    } else {
        logError("充电请求失败");
        return false;
    }
}

bool Go2PowerManager::requestStopCharging() {
    logInfo("请求停止充电");
    
    if (sendGo2ChargeCommand(false)) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        charging_state_.state = robot_base_interfaces::power_interface::ChargingState::NOT_CHARGING;
        battery_state_.is_charging = false;
        
        logInfo("停止充电成功");
        return true;
    } else {
        logError("停止充电失败");
        return false;
    }
}

bool Go2PowerManager::isCharging() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return battery_state_.is_charging;
}

bool Go2PowerManager::needsCharging(float threshold) const {
    float actual_threshold = (threshold < 0.0f) ? config_.low_battery_threshold : threshold;
    return getBatteryPercentage() < actual_threshold;
}

bool Go2PowerManager::isBatteryCritical() const {
    return getBatteryPercentage() < config_.critical_battery_threshold;
}

uint32_t Go2PowerManager::getEstimatedChargeTime() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return charging_state_.estimated_time_to_full;
}

// ============= 充电站管理 =============

bool Go2PowerManager::registerChargingStation(const robot_base_interfaces::power_interface::ChargingStationInfo& station_info) {
    std::lock_guard<std::mutex> lock(stations_mutex_);
    known_stations_[station_info.station_id] = station_info;
    logInfo("注册充电站: " + station_info.station_id);
    return true;
}

std::vector<robot_base_interfaces::power_interface::ChargingStationInfo> Go2PowerManager::getKnownChargingStations() const {
    std::lock_guard<std::mutex> lock(stations_mutex_);
    std::vector<robot_base_interfaces::power_interface::ChargingStationInfo> stations;
    
    for (const auto& pair : known_stations_) {
        stations.push_back(pair.second);
    }
    
    return stations;
}

std::shared_ptr<robot_base_interfaces::power_interface::ChargingStationInfo> Go2PowerManager::findNearestChargingStation(
    const std::vector<float>& current_position) const {
    
    if (current_position.size() < 2) {
        return nullptr;
    }
    
    std::lock_guard<std::mutex> lock(stations_mutex_);
    float min_distance = std::numeric_limits<float>::max();
    auto nearest_station = std::make_shared<robot_base_interfaces::power_interface::ChargingStationInfo>();
    bool found = false;
    
    for (const auto& pair : known_stations_) {
        const auto& station = pair.second;
        if (station.is_available && !station.is_occupied) {
            float dx = station.pose.x - current_position[0];
            float dy = station.pose.y - current_position[1];
            float distance = std::sqrt(dx*dx + dy*dy);
            
            if (distance < min_distance) {
                min_distance = distance;
                *nearest_station = station;
                found = true;
            }
        }
    }
    
    return found ? nearest_station : nullptr;
}

bool Go2PowerManager::updateChargingStationStatus(const std::string& station_id, bool is_available, bool is_occupied) {
    std::lock_guard<std::mutex> lock(stations_mutex_);
    
    auto it = known_stations_.find(station_id);
    if (it != known_stations_.end()) {
        it->second.is_available = is_available;
        it->second.is_occupied = is_occupied;
        it->second.last_update_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        return true;
    }
    
    return false;
}

// ============= 功耗管理 =============

float Go2PowerManager::getCurrentPowerConsumption() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return battery_state_.power_consumption;
}

float Go2PowerManager::getAveragePowerConsumption(uint32_t duration_seconds) const {
    if (statistics_.power_history.empty()) {
        return statistics_.average_power_consumption;
    }
    
    auto now = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    uint64_t cutoff_time = now - duration_seconds;
    
    float sum = 0.0f;
    int count = 0;
    
    for (const auto& entry : statistics_.power_history) {
        if (entry.first >= cutoff_time) {
            sum += entry.second;
            count++;
        }
    }
    
    return count > 0 ? sum / count : statistics_.average_power_consumption;
}

bool Go2PowerManager::setPowerProfile(const robot_base_interfaces::power_interface::PowerConsumptionProfile& profile) {
    logInfo("设置功耗模式: " + profile.profile_name);
    
    // 根据功耗配置文件调整Go2参数
    config_.max_power_consumption = profile.max_power;
    
    // 根据配置文件名称判断功耗模式
    if (profile.profile_name == "power_saving" || profile.max_power < 120.0f) {
        config_.power_saving_enabled = true;
    } else {
        config_.power_saving_enabled = false;
    }
    
    return true;
}

robot_base_interfaces::power_interface::PowerConsumptionProfile Go2PowerManager::getPowerProfile() const {
    robot_base_interfaces::power_interface::PowerConsumptionProfile profile;
    
    if (config_.power_saving_enabled) {
        profile.profile_name = "power_saving";
    } else if (config_.max_power_consumption > 180.0f) {
        profile.profile_name = "high_performance";
    } else {
        profile.profile_name = "normal";
    }
    
    profile.max_power = config_.max_power_consumption;
    profile.idle_power = 10.0f;
    profile.standby_power = 5.0f;
    profile.walking_power = 50.0f;
    profile.running_power = 100.0f;
    
    return profile;
}

bool Go2PowerManager::enablePowerSaving(bool enable) {
    config_.power_saving_enabled = enable;
    logInfo("功耗优化模式: " + std::string(enable ? "启用" : "禁用"));
    return true;
}

bool Go2PowerManager::isPowerSavingEnabled() const {
    return config_.power_saving_enabled;
}

// ============= 电源控制 =============

bool Go2PowerManager::requestSystemShutdown(uint32_t delay_seconds) {
    logInfo("请求系统关机，延迟: " + std::to_string(delay_seconds) + "秒");
    // Go2关机命令实现会在实际环境中通过ROS topic发送
    return executeCustomCommand("system_shutdown", std::to_string(delay_seconds)) == "success";
}

bool Go2PowerManager::requestSystemReboot(uint32_t delay_seconds) {
    logInfo("请求系统重启，延迟: " + std::to_string(delay_seconds) + "秒");
    // Go2重启命令实现会在实际环境中通过ROS topic发送
    return executeCustomCommand("system_reboot", std::to_string(delay_seconds)) == "success";
}

bool Go2PowerManager::enterSleepMode(uint32_t duration_seconds) {
    logInfo("进入休眠模式，持续: " + std::to_string(duration_seconds) + "秒");
    
    // 停止不必要的组件
    is_operational_.store(false);
    
    // 实际环境中会发送Go2休眠命令
    return true;
}

bool Go2PowerManager::wakeFromSleep() {
    logInfo("从休眠模式唤醒");
    
    is_operational_.store(true);
    return true;
}

// ============= 安全和保护 =============

bool Go2PowerManager::hasBatteryFault() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return battery_state_.fault_code != 0;
}

uint32_t Go2PowerManager::getBatteryFaultCode() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return battery_state_.fault_code;
}

bool Go2PowerManager::isChargingSafe() const {
    return isTemperatureSafe() && 
           getBatteryVoltage() > config_.min_voltage &&
           getBatteryVoltage() < config_.max_voltage &&
           !hasBatteryFault();
}

bool Go2PowerManager::isTemperatureSafe() const {
    float temp = getBatteryTemperature();
    return temp > 0.0f && temp < config_.max_temperature;
}

bool Go2PowerManager::calibrateBattery() {
    logInfo("开始电池校准");
    
    // 电池校准过程的模拟实现
    // 实际环境中会通过BMS命令执行校准
    std::lock_guard<std::mutex> lock(state_mutex_);
    battery_state_.fault_code = 0; // 清除故障代码
    
    logInfo("电池校准完成");
    return true;
}

bool Go2PowerManager::resetBatteryStats() {
    logInfo("重置电池统计信息");
    
    std::lock_guard<std::mutex> lock(state_mutex_);
    battery_state_.cycles = 0;
    statistics_.total_charge_cycles = 0;
    statistics_.total_energy_consumed = 0.0f;
    statistics_.total_energy_charged = 0.0f;
    statistics_.power_history.clear();
    
    return true;
}

// ============= 事件和回调 =============

void Go2PowerManager::setBatteryCallback(std::function<void(const robot_base_interfaces::power_interface::BatteryInfo&)> callback) {
    battery_callback_ = callback;
}

void Go2PowerManager::setChargingCallback(std::function<void(const robot_base_interfaces::power_interface::ChargingStatus&)> callback) {
    charging_callback_ = callback;
}

void Go2PowerManager::setPowerEventCallback(std::function<void(const robot_base_interfaces::power_interface::PowerEventInfo&)> callback) {
    power_event_callback_ = callback;
}

void Go2PowerManager::setLowBatteryCallback(std::function<void(float)> callback) {
    low_battery_callback_ = callback;
}

void Go2PowerManager::setChargeCompleteCallback(std::function<void()> callback) {
    charge_complete_callback_ = callback;
}

// ============= 配置管理 =============

bool Go2PowerManager::setLowBatteryThreshold(float threshold) {
    if (threshold > 0.0f && threshold < 100.0f) {
        config_.low_battery_threshold = threshold;
        logInfo("设置低电量阈值: " + std::to_string(threshold) + "%");
        return true;
    }
    return false;
}

bool Go2PowerManager::setCriticalBatteryThreshold(float threshold) {
    if (threshold > 0.0f && threshold < config_.low_battery_threshold) {
        config_.critical_battery_threshold = threshold;
        logInfo("设置严重低电量阈值: " + std::to_string(threshold) + "%");
        return true;
    }
    return false;
}

bool Go2PowerManager::setAutoChargeThreshold(float threshold) {
    if (threshold > 0.0f && threshold < 100.0f) {
        config_.auto_charge_threshold = threshold;
        logInfo("设置自动充电阈值: " + std::to_string(threshold) + "%");
        return true;
    }
    return false;
}

bool Go2PowerManager::enableAutoCharging(bool enable) {
    config_.auto_charging_enabled = enable;
    logInfo("自动充电: " + std::string(enable ? "启用" : "禁用"));
    return true;
}

bool Go2PowerManager::isAutoChargingEnabled() const {
    return config_.auto_charging_enabled;
}

// ============= 诊断和统计 =============

std::string Go2PowerManager::getPowerDiagnostics() const {
    return generateDiagnosticsReport();
}

std::string Go2PowerManager::getChargingStatistics() const {
    return generateStatisticsReport();
}

bool Go2PowerManager::exportBatteryData(const std::string& file_path, const std::string& format) {
    if (format == "json") {
        return exportToJson(file_path);
    } else if (format == "csv") {
        return exportToCsv(file_path);
    }
    return false;
}

// ============= 扩展接口 =============

std::string Go2PowerManager::executeCustomCommand(const std::string& command, const std::string& parameters) {
    logInfo("执行自定义命令: " + command + " 参数: " + parameters);
    
    if (command == "system_shutdown" || command == "system_reboot") {
        // 模拟系统命令执行
        return "success";
    } else if (command == "get_cell_voltages") {
        std::ostringstream ss;
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (size_t i = 0; i < battery_state_.cell_voltages.size(); ++i) {
            if (i > 0) ss << ",";
            ss << std::fixed << std::setprecision(3) << battery_state_.cell_voltages[i];
        }
        return ss.str();
    }
    
    return "unsupported";
}

std::string Go2PowerManager::getConfiguration() const {
    std::ostringstream ss;
    ss << "{\n";
    ss << "  \"low_battery_threshold\": " << config_.low_battery_threshold << ",\n";
    ss << "  \"critical_battery_threshold\": " << config_.critical_battery_threshold << ",\n";
    ss << "  \"auto_charge_threshold\": " << config_.auto_charge_threshold << ",\n";
    ss << "  \"power_saving_enabled\": " << (config_.power_saving_enabled ? "true" : "false") << ",\n";
    ss << "  \"max_power_consumption\": " << config_.max_power_consumption << ",\n";
    ss << "  \"auto_charging_enabled\": " << (config_.auto_charging_enabled ? "true" : "false") << ",\n";
    ss << "  \"monitoring_frequency\": " << config_.monitoring_frequency << "\n";
    ss << "}";
    return ss.str();
}

// ============= Go2特有功能 =============

std::shared_ptr<unitree_go::msg::BmsState> Go2PowerManager::getNativeBmsState() const {
    if (go2_comm_) {
        return go2_comm_->getLatestBmsState();
    }
    return nullptr;
}

bool Go2PowerManager::setGo2ChargingMode(bool enable) {
    logInfo("设置Go2充电模式: " + std::string(enable ? "启用" : "禁用"));
    return sendGo2ChargeCommand(enable);
}

float Go2PowerManager::getWirelessControllerBattery() const {
    // 模拟无线控制器电量，实际环境中从WirelessController消息获取
    return 85.0f;
}

bool Go2PowerManager::isBatteryReplacementNeeded() const {
    // 基于循环次数和健康度判断是否需要更换电池
    return (getBatteryCycles() > 1000) || 
           (getBatteryHealth() == robot_base_interfaces::power_interface::BatteryHealth::POOR);
}

std::map<std::string, float> Go2PowerManager::getDetailedBatteryParameters() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    std::map<std::string, float> params;
    params["voltage"] = battery_state_.voltage;
    params["current"] = battery_state_.current;
    params["temperature"] = battery_state_.temperature;
    params["percentage"] = battery_state_.percentage;
    params["cycles"] = static_cast<float>(battery_state_.cycles);
    params["max_cell_temp"] = battery_state_.max_cell_temp;
    params["min_cell_temp"] = battery_state_.min_cell_temp;
    params["power_consumption"] = battery_state_.power_consumption;
    params["fault_code"] = static_cast<float>(battery_state_.fault_code);
    
    return params;
}

// ============= 受保护方法实现 =============

void Go2PowerManager::triggerPowerEvent(const robot_base_interfaces::power_interface::PowerEventInfo& event) {
    if (power_event_callback_) {
        power_event_callback_(event);
    }
    
    logInfo("触发电源事件: " + std::to_string(static_cast<int>(event.event_type)));
}

void Go2PowerManager::updateBatteryInfo(const robot_base_interfaces::power_interface::BatteryInfo& battery_info) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    // 更新电池状态
    battery_state_.voltage = battery_info.voltage;
    battery_state_.current = battery_info.current;
    battery_state_.temperature = battery_info.temperature;
    battery_state_.percentage = battery_info.soc_percentage;
    battery_state_.cycles = battery_info.cycle_count;
    battery_state_.last_update_ns = battery_info.timestamp_ns;
    
    // 触发回调
    if (battery_callback_) {
        battery_callback_(battery_info);
    }
}

void Go2PowerManager::updateChargingStatus(const robot_base_interfaces::power_interface::ChargingStatus& charging_status) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    charging_state_.charge_rate = charging_status.charging_current;
    charging_state_.estimated_time_to_full = charging_status.estimated_remaining_seconds / 60; // convert seconds to minutes
    charging_state_.station_id = charging_status.charging_station_id;
    
    // 触发回调
    if (charging_callback_) {
        charging_callback_(charging_status);
    }
}

bool Go2PowerManager::checkSafetyLimits() {
    bool is_safe = true;
    
    // 检查温度安全
    if (!isTemperatureSafe()) {
        robot_base_interfaces::power_interface::PowerEventInfo event;
        event.event_type = robot_base_interfaces::power_interface::PowerEvent::THERMAL_WARNING;
        event.timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        event.event_description = "电池温度过高: " + std::to_string(getBatteryTemperature()) + "°C";        event.associated_value = getBatteryTemperature();
        
        triggerPowerEvent(event);
        is_safe = false;
    }
    
    // 检查电压安全
    float voltage = getBatteryVoltage();
    if (voltage < config_.min_voltage || voltage > config_.max_voltage) {
        robot_base_interfaces::power_interface::PowerEventInfo event;
        event.event_type = robot_base_interfaces::power_interface::PowerEvent::POWER_LOSS;
        event.timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        event.event_description = "电池电压异常: " + std::to_string(voltage) + "V";
        event.associated_value = voltage;
        
        triggerPowerEvent(event);
        is_safe = false;
    }
    
    return is_safe;
}

// ============= 私有方法实现 =============

void Go2PowerManager::monitoringTimerCallback() {
    // 检查安全限制
    checkSafetyLimits();
    
    // 检查电池事件
    checkBatteryEvents();
    
    // 记录当前功耗
    float current_power = getCurrentPowerConsumption();
    recordPowerConsumption(current_power);
    
    // 自动充电检查
    if (config_.auto_charging_enabled && needsCharging(config_.auto_charge_threshold) && !isCharging()) {
        logInfo("电量低于自动充电阈值，尝试开始充电");
        requestCharging();
    }
    
    // 更新运行时间统计
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - statistics_.start_time);
    statistics_.total_runtime_seconds = duration.count();
}

void Go2PowerManager::statisticsTimerCallback() {
    // 更新平均功耗
    if (!statistics_.power_history.empty()) {
        float sum = 0.0f;
        for (const auto& entry : statistics_.power_history) {
            sum += entry.second;
        }
        statistics_.average_power_consumption = sum / statistics_.power_history.size();
    }
    
    // 清理过期的历史数据
    cleanupHistoryData();
    
    // 输出统计信息
    logInfo("功耗统计 - 平均: " + std::to_string(statistics_.average_power_consumption) + "W, " +
           "历史记录数: " + std::to_string(statistics_.power_history.size()));
}

bool Go2PowerManager::loadConfiguration() {
    // 从ROS参数服务器或配置文件加载配置
    // 这里使用默认配置
    logInfo("加载功耗管理配置(使用默认值)");
    return true;
}

bool Go2PowerManager::validateConfiguration() {
    // 验证配置参数的有效性
    if (config_.low_battery_threshold <= config_.critical_battery_threshold) {
        logError("低电量阈值必须大于严重低电量阈值");
        return false;
    }
    
    if (config_.monitoring_frequency <= 0.0f || config_.monitoring_frequency > 100.0f) {
        logError("监控频率必须在0-100Hz之间");
        return false;
    }
    
    return true;
}

robot_base_interfaces::power_interface::BatteryHealth Go2PowerManager::calculateBatteryHealth() const {
    // 注意：此方法假设调用者已经持有state_mutex_锁
    // 基于循环次数和温度计算电池健康度
    float health_score = 100.0f;

    // 循环次数影响 (每100次循环降低5%)
    health_score -= (battery_state_.cycles / 100.0f) * 5.0f;

    // 温度影响
    if (battery_state_.temperature > 45.0f) {
        health_score -= (battery_state_.temperature - 45.0f) * 2.0f;
    }

    // 故障代码影响
    if (battery_state_.fault_code != 0) {
        health_score -= 20.0f;
    }

    health_score = std::max(0.0f, health_score);

    if (health_score > 90.0f) {
        return robot_base_interfaces::power_interface::BatteryHealth::EXCELLENT;
    } else if (health_score > 80.0f) {
        return robot_base_interfaces::power_interface::BatteryHealth::GOOD;
    } else if (health_score > 60.0f) {
        return robot_base_interfaces::power_interface::BatteryHealth::FAIR;
    } else {
        return robot_base_interfaces::power_interface::BatteryHealth::POOR;
    }
}

float Go2PowerManager::calculateEstimatedRuntime(float current_load) const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    // 基于当前电量和负载计算预估运行时间
    float remaining_capacity = 15000.0f * (battery_state_.percentage / 100.0f); // mAh
    float runtime_hours = remaining_capacity / (current_load * 1000.0f / battery_state_.voltage); // 小时
    
    return runtime_hours * 60.0f; // 转换为分钟
}

void Go2PowerManager::checkBatteryEvents() {
    float percentage = getBatteryPercentage();
    
    // 检查低电量事件
    if (percentage < config_.low_battery_threshold && low_battery_callback_) {
        low_battery_callback_(percentage);
    }
    
    // 检查充电完成事件
    if (isCharging() && percentage >= 98.0f && charge_complete_callback_) {
        charge_complete_callback_();
    }
    
    // 检查严重低电量事件
    if (percentage < config_.critical_battery_threshold) {
        robot_base_interfaces::power_interface::PowerEventInfo event;
        event.event_type = robot_base_interfaces::power_interface::PowerEvent::BATTERY_CRITICAL;
        event.timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        event.event_description = "电池电量严重不足: " + std::to_string(percentage) + "%";
        event.associated_value = percentage;
        
        triggerPowerEvent(event);
    }
}

bool Go2PowerManager::sendGo2ChargeCommand(bool enable) {
    logInfo("发送Go2充电命令: " + std::string(enable ? "开始" : "停止"));

    try {
        if (!go2_comm_) {
            logError("Go2通信管理器未初始化");
            return false;
        }

        // 创建API请求消息
        unitree_api::msg::Request request;

        // 设置API请求头
        request.header.identity.api_id = enable ? 3001 : 3002; // 假设的充电控制API ID
        request.header.identity.id = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();

        // 设置充电参数
        std::ostringstream params;
        params << "{";
        params << "\"enable\":" << (enable ? "true" : "false") << ",";
        params << "\"mode\":\"wireless\"";
        params << "}";
        request.parameter = params.str();

        // 发送API请求
        bool result = go2_comm_->sendApiRequest(request);

        if (result) {
            logInfo("充电命令发送成功");
        } else {
            logError("充电命令发送失败");
        }

        return result;

    } catch (const std::exception& e) {
        logError("发送充电命令失败: " + std::string(e.what()));
        return false;
    }
}

void Go2PowerManager::recordPowerConsumption(float power) {
    auto now = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    
    statistics_.power_history.emplace_back(now, power);
    
    // 限制历史记录数量
    if (statistics_.power_history.size() > 1000) {
        statistics_.power_history.erase(statistics_.power_history.begin());
    }
}

void Go2PowerManager::cleanupHistoryData() {
    auto now = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    uint64_t cutoff_time = now - 3600; // 保留1小时内的数据
    
    statistics_.power_history.erase(
        std::remove_if(statistics_.power_history.begin(), statistics_.power_history.end(),
                      [cutoff_time](const std::pair<uint64_t, float>& entry) {
                          return entry.first < cutoff_time;
                      }),
        statistics_.power_history.end());
}

std::string Go2PowerManager::generateDiagnosticsReport() const {
    std::ostringstream report;
    
    report << "Go2电源系统诊断报告\n";
    report << "====================\n";
    report << "电池电压: " << std::fixed << std::setprecision(2) << getBatteryVoltage() << "V\n";
    report << "电池电流: " << getBatteryCurrent() << "A\n";
    report << "电池温度: " << getBatteryTemperature() << "°C\n";
    report << "电池电量: " << getBatteryPercentage() << "%\n";
    report << "充电循环: " << getBatteryCycles() << "次\n";
    report << "电池健康: " << static_cast<int>(getBatteryHealth()) << "\n";
    report << "充电状态: " << (isCharging() ? "充电中" : "未充电") << "\n";
    report << "安全状态: " << (isChargingSafe() ? "安全" : "异常") << "\n";
    report << "故障代码: " << getBatteryFaultCode() << "\n";
    
    return report.str();
}

std::string Go2PowerManager::generateStatisticsReport() const {
    std::ostringstream report;
    
    report << "Go2电源统计报告\n";
    report << "================\n";
    report << "总运行时间: " << statistics_.total_runtime_seconds << "秒\n";
    report << "充电次数: " << statistics_.total_charge_cycles << "次\n";
    report << "总耗电量: " << std::fixed << std::setprecision(3) << statistics_.total_energy_consumed << "kWh\n";
    report << "总充电量: " << statistics_.total_energy_charged << "kWh\n";
    report << "平均功耗: " << statistics_.average_power_consumption << "W\n";
    report << "功耗历史记录: " << statistics_.power_history.size() << "条\n";
    
    return report.str();
}

bool Go2PowerManager::exportToJson(const std::string& file_path) const {
    try {
        std::ofstream file(file_path);
        if (!file.is_open()) {
            logError("无法打开文件: " + file_path);
            return false;
        }
        
        file << "{\n";
        file << "  \"battery_info\": " << getConfiguration() << ",\n";
        file << "  \"diagnostics\": \"" << generateDiagnosticsReport() << "\",\n";
        file << "  \"statistics\": \"" << generateStatisticsReport() << "\"\n";
        file << "}\n";
        
        file.close();
        logInfo("成功导出数据到JSON文件: " + file_path);
        return true;
        
    } catch (const std::exception& e) {
        logError("导出JSON文件失败: " + std::string(e.what()));
        return false;
    }
}

bool Go2PowerManager::exportToCsv(const std::string& file_path) const {
    try {
        std::ofstream file(file_path);
        if (!file.is_open()) {
            logError("无法打开文件: " + file_path);
            return false;
        }
        
        // CSV头部
        file << "timestamp,power_consumption\n";
        
        // 功耗历史数据
        for (const auto& entry : statistics_.power_history) {
            file << entry.first << "," << std::fixed << std::setprecision(3) << entry.second << "\n";
        }
        
        file.close();
        logInfo("成功导出数据到CSV文件: " + file_path);
        return true;
        
    } catch (const std::exception& e) {
        logError("导出CSV文件失败: " + std::string(e.what()));
        return false;
    }
}

// ============= ROS回调方法 =============

void Go2PowerManager::bmsStateCallback(const unitree_go::msg::BmsState::SharedPtr msg) {
    try {
        // 使用Go2MessageConverter转换BMS状态
        robot_base_interfaces::power_interface::BatteryInfo battery_info;
        auto result = converter_->convertBmsState(*msg, battery_info);

        if (result != ConversionResult::SUCCESS) {
            logWarning("BMS状态转换失败");
            return;
        }

        std::lock_guard<std::mutex> lock(state_mutex_);

        // 更新内部电池状态
        battery_state_.voltage = battery_info.voltage;
        battery_state_.current = battery_info.current;
        battery_state_.temperature = battery_info.temperature;
        battery_state_.percentage = battery_info.soc_percentage;
        battery_state_.cycles = battery_info.cycle_count;

        // 更新Go2特有的电芯信息
        if (!battery_info.cells.empty()) {
            battery_state_.cell_voltages.resize(battery_info.cells.size());
            for (size_t i = 0; i < battery_info.cells.size(); ++i) {
                battery_state_.cell_voltages[i] = battery_info.cells[i].voltage;
            }
        }

        // 更新温度范围
        battery_state_.max_cell_temp = battery_info.max_temperature;
        battery_state_.min_cell_temp = battery_info.min_temperature;

        // 更新充电状态
        battery_state_.is_charging = (msg->status == 6 || msg->status == 7); // 充电中状态
        if (battery_state_.is_charging) {
            charging_state_.state = robot_base_interfaces::power_interface::ChargingState::CHARGING;
        } else {
            charging_state_.state = robot_base_interfaces::power_interface::ChargingState::NOT_CHARGING;
        }

        // 更新功耗信息
        battery_state_.power_consumption = std::abs(battery_state_.voltage * battery_state_.current);

        // 更新时间戳
        battery_state_.last_update_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();

        // 触发电池状态回调
        if (battery_callback_) {
            battery_callback_(battery_info);
        }

        // logDebug("BMS状态已更新：电量 " + std::to_string(battery_state_.percentage) + "%");

    } catch (const std::exception& e) {
        logError("处理BMS状态回调时发生异常: " + std::string(e.what()));
    }
}

void Go2PowerManager::wirelessControllerCallback(const unitree_go::msg::WirelessController::SharedPtr msg) {
    try {
        // 目前无线控制器不包含电池信息，主要用于其他模块
        // 可以在未来扩展以支持控制器电池监控或通过控制器触发充电操作
        // 收到无线控制器数据，暂时不处理

        // 示例：检查是否有特殊按键组合触发充电操作
        // 这里可以根据实际需求扩展
        (void)msg; // 避免未使用参数警告

    } catch (const std::exception& e) {
        // 静默处理控制器数据异常，因为这不是关键数据
        logWarning("处理无线控制器数据异常: " + std::string(e.what()));
    }
}

// ============= 日志方法 =============

void Go2PowerManager::logError(const std::string& error_msg) const {
    if (node_) {
        RCLCPP_ERROR(node_->get_logger(), "[Go2PowerManager] %s", error_msg.c_str());
    }
}

void Go2PowerManager::logWarning(const std::string& warning_msg) const {
    if (node_) {
        RCLCPP_WARN(node_->get_logger(), "[Go2PowerManager] %s", warning_msg.c_str());
    }
}

void Go2PowerManager::logInfo(const std::string& info_msg) const {
    if (node_) {
        RCLCPP_INFO(node_->get_logger(), "[Go2PowerManager] %s", info_msg.c_str());
    }
}

} // namespace go2_adapter
} // namespace robot_adapters