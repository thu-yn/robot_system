/**
 * @file test_power_types.cpp
 * @brief 电源类型单元测试
 * @author Claude Code
 * @date 2024
 */

#include <gtest/gtest.h>
#include "robot_base_interfaces/power_interface/power_types.hpp"

using namespace robot_base_interfaces::power_interface;

class PowerTypesTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 测试前的设置
    }

    void TearDown() override {
        // 测试后的清理
    }
};

// ========== 枚举测试 ==========

TEST_F(PowerTypesTest, ChargingTypeEnums) {
    EXPECT_EQ(static_cast<int>(ChargingType::NONE), 0);
    EXPECT_EQ(static_cast<int>(ChargingType::WIRELESS), 1);
    EXPECT_EQ(static_cast<int>(ChargingType::CONTACT), 2);
    EXPECT_EQ(static_cast<int>(ChargingType::INDUCTIVE), 3);
    EXPECT_EQ(static_cast<int>(ChargingType::SOLAR), 4);
    EXPECT_EQ(static_cast<int>(ChargingType::REPLACEABLE), 5);
    EXPECT_EQ(static_cast<int>(ChargingType::CUSTOM), 100);
}

TEST_F(PowerTypesTest, ChargingStateEnums) {
    EXPECT_EQ(static_cast<int>(ChargingState::UNKNOWN), 0);
    EXPECT_EQ(static_cast<int>(ChargingState::NOT_CHARGING), 1);
    EXPECT_EQ(static_cast<int>(ChargingState::CONNECTING), 2);
    EXPECT_EQ(static_cast<int>(ChargingState::CHARGING), 3);
    EXPECT_EQ(static_cast<int>(ChargingState::FULL), 4);
    EXPECT_EQ(static_cast<int>(ChargingState::ERROR), 5);
    EXPECT_EQ(static_cast<int>(ChargingState::DISCONNECTED), 6);
    EXPECT_EQ(static_cast<int>(ChargingState::STANDBY), 7);
}

TEST_F(PowerTypesTest, BatteryHealthEnums) {
    EXPECT_EQ(static_cast<int>(BatteryHealth::UNKNOWN), 0);
    EXPECT_EQ(static_cast<int>(BatteryHealth::EXCELLENT), 1);
    EXPECT_EQ(static_cast<int>(BatteryHealth::GOOD), 2);
    EXPECT_EQ(static_cast<int>(BatteryHealth::FAIR), 3);
    EXPECT_EQ(static_cast<int>(BatteryHealth::POOR), 4);
    EXPECT_EQ(static_cast<int>(BatteryHealth::DEAD), 5);
}

TEST_F(PowerTypesTest, BatteryTypeEnums) {
    EXPECT_EQ(static_cast<int>(BatteryType::UNKNOWN), 0);
    EXPECT_EQ(static_cast<int>(BatteryType::LITHIUM_ION), 1);
    EXPECT_EQ(static_cast<int>(BatteryType::LITHIUM_POLYMER), 2);
    EXPECT_EQ(static_cast<int>(BatteryType::NICKEL_METAL), 3);
    EXPECT_EQ(static_cast<int>(BatteryType::LEAD_ACID), 4);
    EXPECT_EQ(static_cast<int>(BatteryType::FUEL_CELL), 5);
    EXPECT_EQ(static_cast<int>(BatteryType::CUSTOM), 100);
}

TEST_F(PowerTypesTest, PowerEventEnums) {
    EXPECT_EQ(static_cast<int>(PowerEvent::BATTERY_LOW), 0);
    EXPECT_EQ(static_cast<int>(PowerEvent::BATTERY_CRITICAL), 1);
    EXPECT_EQ(static_cast<int>(PowerEvent::CHARGING_STARTED), 2);
    EXPECT_EQ(static_cast<int>(PowerEvent::CHARGING_COMPLETED), 3);
    EXPECT_EQ(static_cast<int>(PowerEvent::CHARGING_ERROR), 4);
    EXPECT_EQ(static_cast<int>(PowerEvent::POWER_LOSS), 5);
    EXPECT_EQ(static_cast<int>(PowerEvent::THERMAL_WARNING), 6);
    EXPECT_EQ(static_cast<int>(PowerEvent::BATTERY_FAULT), 7);
}

// ========== BatteryCellInfo 测试 ==========

TEST_F(PowerTypesTest, BatteryCellInfoDefaultConstructor) {
    BatteryCellInfo cell;
    
    EXPECT_EQ(cell.cell_id, 0);
    EXPECT_FLOAT_EQ(cell.voltage, 0.0f);
    EXPECT_FLOAT_EQ(cell.temperature, 0.0f);
    EXPECT_FLOAT_EQ(cell.capacity_mah, 0.0f);
    EXPECT_FLOAT_EQ(cell.health_percentage, 100.0f);
    EXPECT_EQ(cell.cycle_count, 0);
    EXPECT_FALSE(cell.is_balancing);
    EXPECT_FALSE(cell.has_fault);
    EXPECT_EQ(cell.fault_code, 0);
}

TEST_F(PowerTypesTest, BatteryCellInfoParameterAssignment) {
    BatteryCellInfo cell;
    
    cell.cell_id = 5;
    cell.voltage = 3.7f;
    cell.temperature = 25.5f;
    cell.capacity_mah = 1000.0f;
    cell.health_percentage = 95.5f;
    cell.cycle_count = 150;
    cell.is_balancing = true;
    cell.has_fault = false;
    
    EXPECT_EQ(cell.cell_id, 5);
    EXPECT_FLOAT_EQ(cell.voltage, 3.7f);
    EXPECT_FLOAT_EQ(cell.temperature, 25.5f);
    EXPECT_FLOAT_EQ(cell.capacity_mah, 1000.0f);
    EXPECT_FLOAT_EQ(cell.health_percentage, 95.5f);
    EXPECT_EQ(cell.cycle_count, 150);
    EXPECT_TRUE(cell.is_balancing);
    EXPECT_FALSE(cell.has_fault);
}

// ========== BatteryInfo 测试 ==========

TEST_F(PowerTypesTest, BatteryInfoDefaultConstructor) {
    BatteryInfo battery;
    
    // 基本信息
    EXPECT_EQ(battery.type, BatteryType::LITHIUM_ION);
    EXPECT_EQ(battery.health, BatteryHealth::UNKNOWN);
    EXPECT_EQ(battery.version_high, 0);
    EXPECT_EQ(battery.version_low, 0);
    EXPECT_EQ(battery.status, 0);
    
    // 电量信息
    EXPECT_FLOAT_EQ(battery.soc_percentage, 0.0f);
    EXPECT_FLOAT_EQ(battery.voltage, 0.0f);
    EXPECT_FLOAT_EQ(battery.current, 0.0f);
    EXPECT_FLOAT_EQ(battery.power, 0.0f);
    EXPECT_FLOAT_EQ(battery.capacity_mah, 15000.0f);  // Go2默认容量
    EXPECT_FLOAT_EQ(battery.remaining_mah, 0.0f);
    EXPECT_FLOAT_EQ(battery.design_capacity_mah, 15000.0f);
    
    // 温度信息
    EXPECT_FLOAT_EQ(battery.temperature, 0.0f);
    EXPECT_FLOAT_EQ(battery.max_temperature, 0.0f);
    EXPECT_FLOAT_EQ(battery.min_temperature, 0.0f);
    EXPECT_EQ(battery.bq_ntc_temps.size(), 2);
    EXPECT_EQ(battery.mcu_ntc_temps.size(), 2);
    
    // 循环和健康信息
    EXPECT_EQ(battery.cycle_count, 0);
    EXPECT_FLOAT_EQ(battery.health_percentage, 100.0f);
    EXPECT_EQ(battery.manufacturing_date, 0);
    EXPECT_EQ(battery.first_use_date, 0);
    
    // Go2特定：15个电池单体
    EXPECT_EQ(battery.cells.size(), 15);
    for (size_t i = 0; i < battery.cells.size(); ++i) {
        EXPECT_EQ(battery.cells[i].cell_id, static_cast<uint8_t>(i));
    }
    
    // 故障信息
    EXPECT_FALSE(battery.has_fault);
    EXPECT_EQ(battery.fault_code, 0);
    EXPECT_FALSE(battery.low_voltage_warning);
    EXPECT_FALSE(battery.high_temperature_warning);
    EXPECT_FALSE(battery.overcurrent_warning);
    
    EXPECT_EQ(battery.timestamp_ns, 0);
}

TEST_F(PowerTypesTest, BatteryInfoEstimatedRunTime) {
    BatteryInfo battery;
    
    battery.remaining_mah = 7500.0f;  // 50%电量
    battery.voltage = 25.2f;
    
    // 测试不同功耗下的运行时间
    float runtime_50w = battery.estimatedRunTimeMinutes(50.0f);
    float runtime_100w = battery.estimatedRunTimeMinutes(100.0f);
    
    // 50W功耗下应该运行更长时间
    EXPECT_GT(runtime_50w, runtime_100w);
    EXPECT_GT(runtime_50w, 0.0f);
    
    // 测试0功耗的边界情况
    float runtime_0w = battery.estimatedRunTimeMinutes(0.0f);
    EXPECT_FLOAT_EQ(runtime_0w, 0.0f);
}

TEST_F(PowerTypesTest, BatteryInfoNeedsCharging) {
    BatteryInfo battery;
    
    // 测试高电量不需要充电
    battery.soc_percentage = 80.0f;
    EXPECT_FALSE(battery.needsCharging(20.0f));
    
    // 测试低电量需要充电
    battery.soc_percentage = 15.0f;
    EXPECT_TRUE(battery.needsCharging(20.0f));
    
    // 测试临界值
    battery.soc_percentage = 20.0f;
    EXPECT_FALSE(battery.needsCharging(20.0f));
    
    battery.soc_percentage = 19.9f;
    EXPECT_TRUE(battery.needsCharging(20.0f));
}

// ========== ChargingStationInfo 测试 ==========

TEST_F(PowerTypesTest, ChargingStationInfoDefaultConstructor) {
    ChargingStationInfo station;
    
    EXPECT_TRUE(station.station_id.empty());
    EXPECT_EQ(station.charging_type, ChargingType::WIRELESS);
    
    // 位置信息
    EXPECT_FLOAT_EQ(station.pose.x, 0.0f);
    EXPECT_FLOAT_EQ(station.pose.y, 0.0f);
    EXPECT_FLOAT_EQ(station.pose.z, 0.0f);
    EXPECT_FLOAT_EQ(station.pose.yaw, 0.0f);
    
    // 状态信息
    EXPECT_FALSE(station.is_available);
    EXPECT_FALSE(station.is_occupied);
    EXPECT_FLOAT_EQ(station.max_power_output, 100.0f);
    EXPECT_FLOAT_EQ(station.current_power_output, 0.0f);
    
    // 精度要求
    EXPECT_FLOAT_EQ(station.position_tolerance, 0.05f);
    EXPECT_FLOAT_EQ(station.angle_tolerance, 0.1f);
    
    EXPECT_TRUE(station.station_name.empty());
    EXPECT_TRUE(station.station_type.empty());
    EXPECT_EQ(station.last_update_ns, 0);
}

TEST_F(PowerTypesTest, ChargingStationInfoParameterizedConstructor) {
    ChargingStationInfo station("station_001", ChargingType::CONTACT);
    
    EXPECT_EQ(station.station_id, "station_001");
    EXPECT_EQ(station.charging_type, ChargingType::CONTACT);
}

TEST_F(PowerTypesTest, ChargingStationInfoParameterAssignment) {
    ChargingStationInfo station;
    
    station.station_id = "home_base";
    station.charging_type = ChargingType::WIRELESS;
    station.pose.x = 1.5f;
    station.pose.y = 2.0f;
    station.pose.yaw = 1.57f;  // 90度
    station.is_available = true;
    station.max_power_output = 150.0f;
    station.station_name = "Home Base Station";
    
    EXPECT_EQ(station.station_id, "home_base");
    EXPECT_EQ(station.charging_type, ChargingType::WIRELESS);
    EXPECT_FLOAT_EQ(station.pose.x, 1.5f);
    EXPECT_FLOAT_EQ(station.pose.y, 2.0f);
    EXPECT_FLOAT_EQ(station.pose.yaw, 1.57f);
    EXPECT_TRUE(station.is_available);
    EXPECT_FLOAT_EQ(station.max_power_output, 150.0f);
    EXPECT_EQ(station.station_name, "Home Base Station");
}

// ========== ChargingStatus 测试 ==========

TEST_F(PowerTypesTest, ChargingStatusDefaultConstructor) {
    ChargingStatus status;
    
    EXPECT_EQ(status.state, ChargingState::NOT_CHARGING);
    EXPECT_EQ(status.charging_type, ChargingType::NONE);
    
    // 充电参数
    EXPECT_FLOAT_EQ(status.charging_voltage, 0.0f);
    EXPECT_FLOAT_EQ(status.charging_current, 0.0f);
    EXPECT_FLOAT_EQ(status.charging_power, 0.0f);
    EXPECT_FLOAT_EQ(status.charging_efficiency, 0.85f);  // 默认效率
    
    // 时间信息
    EXPECT_EQ(status.charging_time_seconds, 0);
    EXPECT_EQ(status.estimated_full_time_seconds, 0);
    EXPECT_EQ(status.estimated_remaining_seconds, 0);
    
    // 温度
    EXPECT_FLOAT_EQ(status.battery_temperature, 0.0f);
    EXPECT_FLOAT_EQ(status.charger_temperature, 0.0f);
    
    // 充电站信息
    EXPECT_TRUE(status.charging_station_id.empty());
    EXPECT_FLOAT_EQ(status.station_power_output, 0.0f);
    
    // 错误信息
    EXPECT_FALSE(status.has_error);
    EXPECT_EQ(status.error_code, 0);
    EXPECT_TRUE(status.error_message.empty());
    
    // 历史信息
    EXPECT_EQ(status.charge_cycle_count, 0);
    EXPECT_FLOAT_EQ(status.total_charged_energy_kwh, 0.0f);
    
    EXPECT_EQ(status.timestamp_ns, 0);
}

TEST_F(PowerTypesTest, ChargingStatusActiveCharging) {
    ChargingStatus status;
    
    // 设置充电中状态
    status.state = ChargingState::CHARGING;
    status.charging_type = ChargingType::WIRELESS;
    status.charging_voltage = 25.2f;
    status.charging_current = 3.0f;
    status.charging_power = 75.6f;
    status.charging_time_seconds = 1800;  // 30分钟
    status.estimated_remaining_seconds = 3600;  // 1小时
    status.battery_temperature = 30.0f;
    status.charging_station_id = "home_base";
    status.charge_cycle_count = 25;
    
    EXPECT_EQ(status.state, ChargingState::CHARGING);
    EXPECT_EQ(status.charging_type, ChargingType::WIRELESS);
    EXPECT_FLOAT_EQ(status.charging_voltage, 25.2f);
    EXPECT_FLOAT_EQ(status.charging_current, 3.0f);
    EXPECT_FLOAT_EQ(status.charging_power, 75.6f);
    EXPECT_EQ(status.charging_time_seconds, 1800);
    EXPECT_EQ(status.estimated_remaining_seconds, 3600);
    EXPECT_FLOAT_EQ(status.battery_temperature, 30.0f);
    EXPECT_EQ(status.charging_station_id, "home_base");
    EXPECT_EQ(status.charge_cycle_count, 25);
}

// ========== PowerConsumptionProfile 测试 ==========

TEST_F(PowerTypesTest, PowerConsumptionProfileDefaultConstructor) {
    PowerConsumptionProfile profile;
    
    EXPECT_TRUE(profile.profile_name.empty());
    
    // 运动状态功耗
    EXPECT_FLOAT_EQ(profile.idle_power, 10.0f);
    EXPECT_FLOAT_EQ(profile.standby_power, 5.0f);
    EXPECT_FLOAT_EQ(profile.walking_power, 50.0f);
    EXPECT_FLOAT_EQ(profile.running_power, 100.0f);
    EXPECT_FLOAT_EQ(profile.max_power, 150.0f);
    
    // 传感器功耗
    EXPECT_FLOAT_EQ(profile.lidar_power, 15.0f);
    EXPECT_FLOAT_EQ(profile.camera_power, 5.0f);
    EXPECT_FLOAT_EQ(profile.imu_power, 1.0f);
    EXPECT_FLOAT_EQ(profile.communication_power, 8.0f);
    
    // 环境因子
    EXPECT_FLOAT_EQ(profile.temperature_factor, 1.0f);
    EXPECT_FLOAT_EQ(profile.terrain_factor, 1.0f);
    EXPECT_FLOAT_EQ(profile.payload_factor, 1.0f);
}

TEST_F(PowerTypesTest, PowerConsumptionProfileEstimation) {
    PowerConsumptionProfile profile;
    profile.profile_name = "go2_default";
    profile.temperature_factor = 1.1f;  // 10%额外功耗
    profile.terrain_factor = 1.2f;     // 20%额外功耗
    
    // 测试不同状态的功耗估算
    float idle_consumption = profile.estimatePowerConsumption("idle", 0.0f);
    float walking_consumption = profile.estimatePowerConsumption("walking", 5.0f);
    float running_consumption = profile.estimatePowerConsumption("running", 10.0f);
    float standby_consumption = profile.estimatePowerConsumption("standby", 0.0f);
    
    // 验证基本功耗
    EXPECT_GT(walking_consumption, idle_consumption);
    EXPECT_GT(running_consumption, walking_consumption);
    EXPECT_LT(standby_consumption, idle_consumption);
    
    // 验证因子影响
    float expected_walking = (50.0f + 5.0f) * 1.1f * 1.2f * 1.0f;
    EXPECT_FLOAT_EQ(walking_consumption, expected_walking);
}

// ========== Go2PowerConfig 测试 ==========

TEST_F(PowerTypesTest, Go2PowerConfigDefaults) {
    Go2PowerConfig config;
    
    // 电池参数
    EXPECT_FLOAT_EQ(config.battery.nominal_voltage, 25.2f);
    EXPECT_FLOAT_EQ(config.battery.max_voltage, 29.4f);
    EXPECT_FLOAT_EQ(config.battery.min_voltage, 20.0f);
    EXPECT_FLOAT_EQ(config.battery.design_capacity_mah, 15000.0f);
    EXPECT_EQ(config.battery.cell_count, 15);
    
    // 充电参数
    EXPECT_EQ(config.charging.type, ChargingType::WIRELESS);
    EXPECT_FLOAT_EQ(config.charging.max_charging_current, 5.0f);
    EXPECT_FLOAT_EQ(config.charging.max_charging_power, 100.0f);
    EXPECT_FLOAT_EQ(config.charging.charging_efficiency, 0.85f);
    EXPECT_EQ(config.charging.full_charge_time_minutes, 120);
    
    // 功耗管理阈值
    EXPECT_FLOAT_EQ(config.power_management.low_battery_threshold, 20.0f);
    EXPECT_FLOAT_EQ(config.power_management.critical_battery_threshold, 10.0f);
    EXPECT_FLOAT_EQ(config.power_management.auto_charge_threshold, 25.0f);
    EXPECT_FLOAT_EQ(config.power_management.emergency_shutdown_threshold, 5.0f);
    
    // 温度保护
    EXPECT_FLOAT_EQ(config.thermal.max_charging_temperature, 45.0f);
    EXPECT_FLOAT_EQ(config.thermal.min_charging_temperature, 0.0f);
    EXPECT_FLOAT_EQ(config.thermal.max_operating_temperature, 60.0f);
    EXPECT_FLOAT_EQ(config.thermal.thermal_throttle_temperature, 50.0f);
}

TEST_F(PowerTypesTest, Go2PowerConfigParameterValidation) {
    Go2PowerConfig config;
    
    // 验证参数范围合理性
    EXPECT_GT(config.battery.max_voltage, config.battery.nominal_voltage);
    EXPECT_GT(config.battery.nominal_voltage, config.battery.min_voltage);
    EXPECT_GT(config.battery.min_voltage, 0.0f);
    
    EXPECT_GT(config.charging.max_charging_power, 0.0f);
    EXPECT_LT(config.charging.max_charging_power, 1000.0f);  // 合理上限
    
    EXPECT_GT(config.power_management.low_battery_threshold, 
              config.power_management.critical_battery_threshold);
    EXPECT_GT(config.power_management.critical_battery_threshold, 
              config.power_management.emergency_shutdown_threshold);
    
    EXPECT_GT(config.thermal.max_charging_temperature, 
              config.thermal.min_charging_temperature);
    EXPECT_LT(config.thermal.max_operating_temperature, 100.0f);  // 不应超过沸点
}

// ========== PowerEventInfo 测试 ==========

TEST_F(PowerTypesTest, PowerEventInfoDefaultConstructor) {
    PowerEventInfo event;
    
    EXPECT_EQ(event.event_code, 0);
    EXPECT_TRUE(event.event_message.empty());
    EXPECT_TRUE(event.event_description.empty());
    EXPECT_EQ(event.timestamp_ns, 0);
    EXPECT_FLOAT_EQ(event.associated_value, 0.0f);
    EXPECT_TRUE(event.numeric_data.empty());
    EXPECT_TRUE(event.string_data.empty());
}

TEST_F(PowerTypesTest, PowerEventInfoParameterizedConstructor) {
    PowerEventInfo event(PowerEvent::BATTERY_LOW, "Battery level is low");
    
    EXPECT_EQ(event.event_type, PowerEvent::BATTERY_LOW);
    EXPECT_EQ(event.event_message, "Battery level is low");
}

TEST_F(PowerTypesTest, PowerEventInfoDataMaps) {
    PowerEventInfo event;
    event.event_type = PowerEvent::THERMAL_WARNING;
    
    // 添加数值数据
    event.numeric_data["temperature"] = 65.5f;
    event.numeric_data["threshold"] = 60.0f;
    
    // 添加字符串数据
    event.string_data["component"] = "battery_pack";
    event.string_data["location"] = "rear_compartment";
    
    // 验证数据
    EXPECT_FLOAT_EQ(event.numeric_data["temperature"], 65.5f);
    EXPECT_FLOAT_EQ(event.numeric_data["threshold"], 60.0f);
    EXPECT_EQ(event.string_data["component"], "battery_pack");
    EXPECT_EQ(event.string_data["location"], "rear_compartment");
}

// ========== 集成和边界测试 ==========

TEST_F(PowerTypesTest, Go2SpecificConstraints) {
    BatteryInfo battery;
    Go2PowerConfig config;
    
    // 验证Go2特定配置
    EXPECT_EQ(battery.cells.size(), 15);  // Go2有15个电池单体
    EXPECT_EQ(config.battery.cell_count, 15);
    EXPECT_FLOAT_EQ(battery.capacity_mah, config.battery.design_capacity_mah);
    
    // 验证默认容量与配置一致
    EXPECT_FLOAT_EQ(battery.design_capacity_mah, 15000.0f);
    EXPECT_FLOAT_EQ(config.battery.design_capacity_mah, 15000.0f);
    
    // 验证充电类型一致
    EXPECT_EQ(config.charging.type, ChargingType::WIRELESS);
}

TEST_F(PowerTypesTest, BatteryHealthLogic) {
    BatteryInfo battery;
    
    // 测试不同健康度对应的健康状态
    battery.health_percentage = 95.0f;
    // 应该被归类为EXCELLENT (90-100%)
    
    battery.health_percentage = 85.0f;
    // 应该被归类为GOOD (80-89%)
    
    battery.health_percentage = 75.0f;
    // 应该被归类为FAIR (70-79%)
    
    battery.health_percentage = 60.0f;
    // 应该被归类为POOR (50-69%)
    
    battery.health_percentage = 30.0f;
    // 应该被归类为DEAD (<50%)
    
    // 验证健康度在合理范围内
    EXPECT_GE(battery.health_percentage, 0.0f);
    EXPECT_LE(battery.health_percentage, 100.0f);
}

TEST_F(PowerTypesTest, ChargingEfficiencyCalculations) {
    ChargingStatus status;
    status.charging_efficiency = 0.85f;
    status.charging_power = 100.0f;  // 输入功率
    
    // 实际有效功率应该是输入功率 * 效率
    float effective_power = status.charging_power * status.charging_efficiency;
    EXPECT_FLOAT_EQ(effective_power, 85.0f);
    
    // 验证效率在合理范围内
    EXPECT_GE(status.charging_efficiency, 0.0f);
    EXPECT_LE(status.charging_efficiency, 1.0f);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}