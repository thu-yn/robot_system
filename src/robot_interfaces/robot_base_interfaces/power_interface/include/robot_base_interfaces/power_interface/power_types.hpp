/**
 * @file   power_types.hpp
 * @brief  机器人电源管理相关数据类型定义 - 完全适配Go2，预留扩展
 * @author Yang Nan
 * @date   2025-09-11
 */

#ifndef ROBOT_BASE_INTERFACES__POWER_INTERFACE__POWER_TYPES_HPP_
#define ROBOT_BASE_INTERFACES__POWER_INTERFACE__POWER_TYPES_HPP_

#include <vector>
#include <string>
#include <map>
#include <cstdint>

namespace robot_base_interfaces {
namespace power_interface {

/**
 * @brief 充电类型枚举
 */
enum class ChargingType {
    NONE        = 0,    ///< 不支持充电
    WIRELESS    = 1,    ///< 无线充电 (Go2使用)
    CONTACT     = 2,    ///< 接触式充电
    INDUCTIVE   = 3,    ///< 电感充电
    SOLAR       = 4,    ///< 太阳能充电 (户外机器人)
    REPLACEABLE = 5,    ///< 可更换电池
    CUSTOM      = 100   ///< 自定义充电方式
};

/**
 * @brief 充电状态枚举
 */
enum class ChargingState {
    UNKNOWN      = 0,   ///< 未知状态
    NOT_CHARGING = 1,   ///< 未充电
    CONNECTING   = 2,   ///< 连接中
    CHARGING     = 3,   ///< 充电中
    FULL         = 4,   ///< 已充满
    ERROR        = 5,   ///< 充电错误
    DISCONNECTED = 6,   ///< 断开连接
    STANDBY      = 7    ///< 充电待机
};

/**
 * @brief 电池健康状态枚举
 */
enum class BatteryHealth {
    UNKNOWN   = 0,  ///< 未知
    EXCELLENT = 1,  ///< 优秀 (90-100% capacity)
    GOOD      = 2,  ///< 良好 (80-89% capacity)
    FAIR      = 3,  ///< 一般 (70-79% capacity)
    POOR      = 4,  ///< 较差 (50-69% capacity)
    DEAD      = 5   ///< 失效 (<50% capacity)
};

/**
 * @brief 电池类型枚举
 */
enum class BatteryType {
    UNKNOWN         = 0,    ///< 未知类型
    LITHIUM_ION     = 1,    ///< 锂离子电池 (Go2使用)
    LITHIUM_POLYMER = 2,    ///< 锂聚合物电池
    NICKEL_METAL    = 3,    ///< 镍氢电池
    LEAD_ACID       = 4,    ///< 铅酸电池
    FUEL_CELL       = 5,    ///< 燃料电池
    CUSTOM          = 100   ///< 自定义电池
};

/**
 * @brief 电池单体信息 - 基于Go2 BmsState
 */
struct BatteryCellInfo {
    uint8_t  cell_id           = 0;         ///< 电池单体ID
    float    voltage           = 0.0f;      ///< 电压 (V)
    float    temperature       = 0.0f;      ///< 温度 (°C)
    float    capacity_mah      = 0.0f;      ///< 容量 (mAh)
    float    health_percentage = 100.0f;    ///< 健康度百分比
    uint32_t cycle_count       = 0;         ///< 循环次数
    bool     is_balancing      = false;     ///< 是否在均衡
    bool     has_fault         = false;     ///< 是否有故障
    uint32_t fault_code        = 0;         ///< 故障代码
};

/**
 * @brief 电池详细信息 - 基于Go2 BmsState设计
 */
struct BatteryInfo {
    // 基本电池信息
    BatteryType   type         = BatteryType::LITHIUM_ION;
    BatteryHealth health       = BatteryHealth::UNKNOWN;
    uint8_t       version_high = 0;                         ///< BMS版本高位
    uint8_t       version_low  = 0;                         ///< BMS版本低位
    uint8_t       status       = 0;                         ///< 电池状态寄存器
    
    // 电量和容量
    float soc_percentage      = 0.0f;       ///< 剩余电量百分比 (0.0-100.0)
    float voltage             = 0.0f;       ///< 总电压 (V)
    float current             = 0.0f;       ///< 电流 (A, 正值为充电，负值为放电)
    float power               = 0.0f;       ///< 功率 (W)
    float capacity_mah        = 15000.0f;   ///< 总容量 (mAh, Go2默认15000)
    float remaining_mah       = 0.0f;       ///< 剩余容量 (mAh)
    float design_capacity_mah = 15000.0f;   ///< 设计容量 (mAh)
    
    // 温度信息
    float temperature     = 0.0f;       ///< 平均温度 (°C)
    float max_temperature = 0.0f;       ///< 最高温度 (°C)
    float min_temperature = 0.0f;       ///< 最低温度 (°C)
    std::vector<float> bq_ntc_temps;    ///< BQ芯片NTC温度 (2个)
    std::vector<float> mcu_ntc_temps;   ///< MCU NTC温度 (2个)
    
    // 循环和寿命信息
    uint16_t cycle_count        = 0;        ///< 充电循环次数
    float    health_percentage  = 100.0f;   ///< 电池健康度百分比
    uint64_t manufacturing_date = 0;        ///< 制造日期 (时间戳)
    uint64_t first_use_date     = 0;        ///< 首次使用日期 (时间戳)
    
    // 电池单体信息 (Go2有15个单体)
    std::vector<BatteryCellInfo> cells;
    
    // 故障和告警信息
    bool     has_fault                = false;  ///< 是否有故障
    uint32_t fault_code               = 0;      ///< 故障代码
    bool     low_voltage_warning      = false;  ///< 低电压告警
    bool     high_temperature_warning = false;  ///< 高温告警
    bool     overcurrent_warning      = false;  ///< 过流告警
    
    // 时间戳
    uint64_t timestamp_ns = 0;      ///< 时间戳
    
    BatteryInfo() {
        // 为Go2初始化15个电池单体
        cells.resize(15);
        for (size_t i = 0; i < cells.size(); ++i) {
            cells[i].cell_id = static_cast<uint8_t>(i);
        }
        
        // 初始化NTC温度数组
        bq_ntc_temps.resize(2, 0.0f);
        mcu_ntc_temps.resize(2, 0.0f);
    }
    
    /**
     * @brief 计算预计剩余运行时间
     * @param current_power_consumption 当前功耗 (W)
     * @return 剩余运行时间 (分钟)
     */
    float estimatedRunTimeMinutes(float current_power_consumption = 50.0f) const {
        if (current_power_consumption <= 0.0f) return 0.0f;
        return (remaining_mah * voltage / 1000.0f) / current_power_consumption * 60.0f;
    }
    
    /**
     * @brief 检查是否需要充电
     * @param low_threshold 低电量阈值 (%)
     * @return true if needs charging
     */
    bool needsCharging(float low_threshold = 20.0f) const {
        return soc_percentage < low_threshold;
    }
};

/**
 * @brief 充电站信息
 */
struct ChargingStationInfo {
    std::string  station_id;        ///< 充电站ID
    ChargingType charging_type = ChargingType::WIRELESS;
    
    // 位置信息
    struct {
        float x   = 0.0f;           ///< X坐标 (m)
        float y   = 0.0f;           ///< Y坐标 (m)
        float z   = 0.0f;           ///< Z坐标 (m)
        float yaw = 0.0f;           ///< 偏航角 (rad)
    } pose;
    
    // 充电站状态
    bool is_available = false;              ///< 是否可用
    bool is_occupied  = false;              ///< 是否被占用
    float max_power_output     = 100.0f;    ///< 最大输出功率 (W)
    float current_power_output = 0.0f;      ///< 当前输出功率 (W)
    
    // 对接精度要求
    float position_tolerance = 0.05f;   ///< 位置容差 (m)
    float angle_tolerance    = 0.1f;    ///< 角度容差 (rad)
    
    // 充电站信息
    std::string station_name;       ///< 充电站名称
    std::string station_type;       ///< 充电站类型
    uint64_t last_update_ns = 0;    ///< 最后更新时间
    
    ChargingStationInfo() = default;
    ChargingStationInfo(const std::string& id, ChargingType type)
        : station_id(id), charging_type(type) {}
};

/**
 * @brief 充电状态详细信息
 */
struct ChargingStatus {
    ChargingState state = ChargingState::NOT_CHARGING;
    ChargingType charging_type = ChargingType::NONE;
    
    // 充电参数
    float charging_voltage    = 0.0f;   ///< 充电电压 (V)
    float charging_current    = 0.0f;   ///< 充电电流 (A)
    float charging_power      = 0.0f;   ///< 充电功率 (W)
    float charging_efficiency = 0.85f;  ///< 充电效率 (0.0-1.0)
    
    // 充电时间信息
    uint32_t charging_time_seconds       = 0; ///< 已充电时间 (秒)
    uint32_t estimated_full_time_seconds = 0; ///< 预计充满时间 (秒)
    uint32_t estimated_remaining_seconds = 0; ///< 预计剩余充电时间 (秒)
    
    // 温度信息
    float battery_temperature = 0.0f;   ///< 电池温度 (°C)
    float charger_temperature = 0.0f;   ///< 充电器温度 (°C)
    
    // 充电站信息
    std::string charging_station_id;    ///< 当前充电站ID
    float station_power_output = 0.0f;  ///< 充电站输出功率 (W)
    
    // 错误和状态
    bool has_error = false;         ///< 是否有错误
    uint32_t error_code = 0;        ///< 错误代码
    std::string error_message;      ///< 错误消息
    
    // 充电历史
    uint32_t charge_cycle_count = 0;        ///< 充电循环计数
    float total_charged_energy_kwh = 0.0f;  ///< 总充电能量 (kWh)
    
    uint64_t timestamp_ns = 0;      ///< 时间戳
};

/**
 * @brief 功耗配置文件
 */
struct PowerConsumptionProfile {
    std::string profile_name;       ///< 配置文件名称
    
    // 不同状态下的功耗 (W)
    float idle_power    = 10.0f;    ///< 待机功耗
    float standby_power = 5.0f;     ///< 休眠功耗
    float walking_power = 50.0f;    ///< 步行功耗
    float running_power = 100.0f;   ///< 奔跑功耗
    float max_power     = 150.0f;   ///< 最大功耗
    
    // 传感器功耗
    float lidar_power         = 15.0f;  ///< 激光雷达功耗
    float camera_power        = 5.0f;   ///< 摄像头功耗 (预留)
    float imu_power           = 1.0f;   ///< IMU功耗
    float communication_power = 8.0f;   ///< 通信功耗
    
    // 环境因子
    float temperature_factor = 1.0f;    ///< 温度系数
    float terrain_factor     = 1.0f;    ///< 地形系数
    float payload_factor     = 1.0f;    ///< 负载系数
    
    /**
     * @brief 估算特定状态下的功耗
     * @param state 机器人状态
     * @param additional_load 附加负载功耗 (W)
     * @return 估算功耗 (W)
     */
    float estimatePowerConsumption(const std::string& state, 
                                 float additional_load = 0.0f) const {
        float base_power = idle_power;
        
        if (state == "walking")      base_power = walking_power;
        else if (state == "running") base_power = running_power;
        else if (state == "standby") base_power = standby_power;
        
        return (base_power + additional_load) * temperature_factor * 
               terrain_factor * payload_factor;
    }
};

/**
 * @brief 电源管理配置 - Go2特定配置
 */
struct Go2PowerConfig {
    // 电池参数
    struct {
        float nominal_voltage     = 25.2f;      ///< 标称电压 (V)
        float max_voltage         = 29.4f;      ///< 最大电压 (V)
        float min_voltage         = 20.0f;      ///< 最小电压 (V)
        float design_capacity_mah = 15000.0f;   ///< 设计容量 (mAh)
        int cell_count = 15;                    ///< 电池单体数量
    } battery;
    
    // 充电参数
    struct {
        ChargingType type = ChargingType::WIRELESS;
        float max_charging_current = 5.0f;          ///< 最大充电电流 (A)
        float max_charging_power   = 100.0f;        ///< 最大充电功率 (W)
        float charging_efficiency  = 0.85f;         ///< 充电效率
        uint32_t full_charge_time_minutes = 120;    ///< 满电充电时间 (分钟)
    } charging;
    
    // 功耗管理阈值
    struct {
        float low_battery_threshold        = 20.0f; ///< 低电量阈值 (%)
        float critical_battery_threshold   = 10.0f; ///< 严重低电量阈值 (%)
        float auto_charge_threshold        = 25.0f; ///< 自动充电阈值 (%)
        float emergency_shutdown_threshold = 5.0f;  ///< 紧急关机阈值 (%)
    } power_management;
    
    // 温度保护
    struct {
        float max_charging_temperature     = 45.0f; ///< 最大充电温度 (°C)
        float min_charging_temperature     = 0.0f;  ///< 最小充电温度 (°C)
        float max_operating_temperature    = 60.0f; ///< 最大工作温度 (°C)
        float thermal_throttle_temperature = 50.0f; ///< 温度限制阈值 (°C)
    } thermal;
};

/**
 * @brief 电源事件类型
 */
enum class PowerEvent {
    BATTERY_LOW        = 0, ///< 电量低
    BATTERY_CRITICAL   = 1, ///< 电量严重不足
    CHARGING_STARTED   = 2, ///< 开始充电
    CHARGING_COMPLETED = 3, ///< 充电完成
    CHARGING_ERROR     = 4, ///< 充电错误
    POWER_LOSS         = 5, ///< 断电
    THERMAL_WARNING    = 6, ///< 温度告警
    BATTERY_FAULT      = 7  ///< 电池故障
};

/**
 * @brief 电源事件信息
 */
struct PowerEventInfo {
    PowerEvent event_type;
    uint32_t event_code = 0;        ///< 事件代码
    std::string event_message;      ///< 事件消息
    std::string event_description;  ///< 事件详细描述
    uint64_t timestamp_ns = 0;      ///< 事件时间戳
    float associated_value = 0.0f;  ///< 关联数值 (如电量百分比、温度等)
    
    // 事件相关数据
    std::map<std::string, float> numeric_data;
    std::map<std::string, std::string> string_data;
    
    PowerEventInfo() = default;
    PowerEventInfo(PowerEvent type, const std::string& msg)
        : event_type(type), event_message(msg) {}
};

} // namespace power_interface
} // namespace robot_base_interfaces

#endif // ROBOT_BASE_INTERFACES__POWER_INTERFACE__POWER_TYPES_HPP_