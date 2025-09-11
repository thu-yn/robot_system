/**
 * @file   test_robot_state_types.cpp
 * @brief  机器人状态类型单元测试
 * @author Yang Nan
 * @date   2025-09-11
 */

#include <gtest/gtest.h>
#include "robot_base_interfaces/state_interface/state_types.hpp"

using namespace robot_base_interfaces::state_interface;

class RobotStateTypesTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 测试前的设置
    }

    void TearDown() override {
        // 测试后的清理
    }
};

// ========== 枚举测试 ==========

TEST_F(RobotStateTypesTest, RobotStateEnums) {
    EXPECT_EQ(static_cast<int>(RobotState::UNKNOWN), 0);
    EXPECT_EQ(static_cast<int>(RobotState::INITIALIZING), 1);
    EXPECT_EQ(static_cast<int>(RobotState::STANDBY), 2);
    EXPECT_EQ(static_cast<int>(RobotState::ACTIVE), 3);
    EXPECT_EQ(static_cast<int>(RobotState::MOVING), 4);
    EXPECT_EQ(static_cast<int>(RobotState::ERROR), 5);
    EXPECT_EQ(static_cast<int>(RobotState::EMERGENCY_STOP), 6);
    EXPECT_EQ(static_cast<int>(RobotState::CHARGING), 7);
    EXPECT_EQ(static_cast<int>(RobotState::LOW_POWER), 8);
    EXPECT_EQ(static_cast<int>(RobotState::MAINTENANCE), 9);
    EXPECT_EQ(static_cast<int>(RobotState::SHUTDOWN), 10);
}

TEST_F(RobotStateTypesTest, HealthLevelEnums) {
    EXPECT_EQ(static_cast<int>(HealthLevel::EXCELLENT), 0);
    EXPECT_EQ(static_cast<int>(HealthLevel::GOOD), 1);
    EXPECT_EQ(static_cast<int>(HealthLevel::FAIR), 2);
    EXPECT_EQ(static_cast<int>(HealthLevel::POOR), 3);
    EXPECT_EQ(static_cast<int>(HealthLevel::CRITICAL), 4);
    EXPECT_EQ(static_cast<int>(HealthLevel::UNKNOWN), 5);
}

TEST_F(RobotStateTypesTest, AlertTypeEnums) {
    EXPECT_EQ(static_cast<int>(AlertType::INFO), 0);
    EXPECT_EQ(static_cast<int>(AlertType::WARNING), 1);
    EXPECT_EQ(static_cast<int>(AlertType::ERROR), 2);
    EXPECT_EQ(static_cast<int>(AlertType::CRITICAL), 3);
}

TEST_F(RobotStateTypesTest, SystemModuleEnums) {
    // 核心系统模块
    EXPECT_EQ(static_cast<int>(SystemModule::MOTION_CONTROL), 0);
    EXPECT_EQ(static_cast<int>(SystemModule::SENSOR_SYSTEM), 1);
    EXPECT_EQ(static_cast<int>(SystemModule::POWER_MANAGEMENT), 2);
    EXPECT_EQ(static_cast<int>(SystemModule::COMMUNICATION), 3);
    EXPECT_EQ(static_cast<int>(SystemModule::NAVIGATION), 4);
    
    // Go2特有模块
    EXPECT_EQ(static_cast<int>(SystemModule::MOTOR_DRIVERS), 10);
    EXPECT_EQ(static_cast<int>(SystemModule::IMU_SYSTEM), 11);
    EXPECT_EQ(static_cast<int>(SystemModule::LIDAR_SYSTEM), 12);
    EXPECT_EQ(static_cast<int>(SystemModule::CHARGING_SYSTEM), 13);
    EXPECT_EQ(static_cast<int>(SystemModule::THERMAL_SYSTEM), 14);
}

// ========== MotorInfo 测试 ==========

TEST_F(RobotStateTypesTest, MotorInfoDefaultConstructor) {
    MotorInfo motor;
    
    EXPECT_EQ(motor.motor_id, 0);
    EXPECT_EQ(motor.mode, 0);
    EXPECT_FLOAT_EQ(motor.position, 0.0f);
    EXPECT_FLOAT_EQ(motor.velocity, 0.0f);
    EXPECT_FLOAT_EQ(motor.acceleration, 0.0f);
    EXPECT_FLOAT_EQ(motor.torque_estimated, 0.0f);
    EXPECT_FLOAT_EQ(motor.position_raw, 0.0f);
    EXPECT_FLOAT_EQ(motor.velocity_raw, 0.0f);
    EXPECT_FLOAT_EQ(motor.acceleration_raw, 0.0f);
    EXPECT_EQ(motor.temperature, 0);
    EXPECT_EQ(motor.lost_count, 0);
    EXPECT_TRUE(motor.is_online);
    EXPECT_EQ(motor.error_code, 0);
}

TEST_F(RobotStateTypesTest, MotorInfoParameterAssignment) {
    MotorInfo motor;
    motor.motor_id = 5;
    motor.mode = 1;  // FOC模式
    motor.position = 1.57f;  // 90度
    motor.velocity = 2.0f;
    motor.torque_estimated = 10.5f;
    motor.temperature = 45;
    motor.is_online = true;
    
    EXPECT_EQ(motor.motor_id, 5);
    EXPECT_EQ(motor.mode, 1);
    EXPECT_FLOAT_EQ(motor.position, 1.57f);
    EXPECT_FLOAT_EQ(motor.velocity, 2.0f);
    EXPECT_FLOAT_EQ(motor.torque_estimated, 10.5f);
    EXPECT_EQ(motor.temperature, 45);
    EXPECT_TRUE(motor.is_online);
}

// ========== FootInfo 测试 ==========

TEST_F(RobotStateTypesTest, FootInfoDefaultConstructor) {
    FootInfo foot;
    
    EXPECT_EQ(foot.foot_id, 0);
    EXPECT_FLOAT_EQ(foot.position.x, 0.0f);
    EXPECT_FLOAT_EQ(foot.position.y, 0.0f);
    EXPECT_FLOAT_EQ(foot.position.z, 0.0f);
    EXPECT_FLOAT_EQ(foot.velocity.x, 0.0f);
    EXPECT_FLOAT_EQ(foot.velocity.y, 0.0f);
    EXPECT_FLOAT_EQ(foot.velocity.z, 0.0f);
    EXPECT_FLOAT_EQ(foot.force, 0.0f);
    EXPECT_FLOAT_EQ(foot.force_estimated, 0.0f);
    EXPECT_FALSE(foot.in_contact);
    EXPECT_FLOAT_EQ(foot.contact_probability, 0.0f);
}

TEST_F(RobotStateTypesTest, FootInfoParameterAssignment) {
    FootInfo foot;
    foot.foot_id = 2;  // 右后足
    foot.position.x = 0.3f;
    foot.position.y = -0.2f;
    foot.position.z = -0.4f;
    foot.velocity.x = 0.1f;
    foot.force = 50.0f;
    foot.in_contact = true;
    foot.contact_probability = 0.95f;
    
    EXPECT_EQ(foot.foot_id, 2);
    EXPECT_FLOAT_EQ(foot.position.x, 0.3f);
    EXPECT_FLOAT_EQ(foot.position.y, -0.2f);
    EXPECT_FLOAT_EQ(foot.position.z, -0.4f);
    EXPECT_FLOAT_EQ(foot.velocity.x, 0.1f);
    EXPECT_FLOAT_EQ(foot.force, 50.0f);
    EXPECT_TRUE(foot.in_contact);
    EXPECT_FLOAT_EQ(foot.contact_probability, 0.95f);
}

// ========== DetailedRobotState 测试 ==========

TEST_F(RobotStateTypesTest, DetailedRobotStateDefaultConstructor) {
    DetailedRobotState state;
    
    // 基本状态
    EXPECT_EQ(state.state, RobotState::UNKNOWN);
    EXPECT_EQ(state.health_level, HealthLevel::UNKNOWN);
    EXPECT_FLOAT_EQ(state.health_score, 0.0f);
    EXPECT_EQ(state.timestamp_ns, 0);
    EXPECT_EQ(state.error_code, 0);
    
    // 运动状态
    EXPECT_EQ(state.motion.mode, 0);
    EXPECT_EQ(state.motion.gait_type, 0);
    EXPECT_FLOAT_EQ(state.motion.progress, 0.0f);
    EXPECT_FLOAT_EQ(state.motion.body_height, 0.32f);  // Go2默认高度
    EXPECT_FLOAT_EQ(state.motion.foot_raise_height, 0.09f);
    EXPECT_FLOAT_EQ(state.motion.position.x, 0.0f);
    EXPECT_FLOAT_EQ(state.motion.velocity.x, 0.0f);
    EXPECT_FLOAT_EQ(state.motion.yaw_speed, 0.0f);
    
    // Go2特定配置：20个电机，4个足端，4个障碍物检测方向
    EXPECT_EQ(state.motors.size(), 20);
    EXPECT_EQ(state.feet.size(), 4);
    EXPECT_EQ(state.range_obstacles.size(), 4);
    
    // 验证电机ID初始化
    for (size_t i = 0; i < state.motors.size(); ++i) {
        EXPECT_EQ(state.motors[i].motor_id, static_cast<uint8_t>(i));
    }
    
    // 验证足端ID初始化
    for (size_t i = 0; i < state.feet.size(); ++i) {
        EXPECT_EQ(state.feet[i].foot_id, static_cast<uint8_t>(i));
    }
    
    // 验证障碍物距离初始化
    for (float distance : state.range_obstacles) {
        EXPECT_FLOAT_EQ(distance, 0.0f);
    }
    
    // 传感器状态
    EXPECT_FALSE(state.sensors.imu_online);
    EXPECT_FALSE(state.sensors.lidar_online);
    EXPECT_FALSE(state.sensors.camera_online);
    EXPECT_EQ(state.sensors.imu_temperature, 0);
    EXPECT_FLOAT_EQ(state.sensors.lidar_frequency, 0.0f);
    
    // 通信状态
    EXPECT_FALSE(state.communication.ros2_online);
    EXPECT_FALSE(state.communication.sdk_online);
    EXPECT_EQ(state.communication.message_lost_count, 0);
    EXPECT_FLOAT_EQ(state.communication.communication_quality, 1.0f);
    
    // 系统资源
    EXPECT_FLOAT_EQ(state.system_resources.cpu_usage, 0.0f);
    EXPECT_FLOAT_EQ(state.system_resources.memory_usage, 0.0f);
    EXPECT_FLOAT_EQ(state.system_resources.disk_usage, 0.0f);
    EXPECT_FLOAT_EQ(state.system_resources.network_usage, 0.0f);
    EXPECT_EQ(state.system_resources.cpu_temperature, 0);
}

TEST_F(RobotStateTypesTest, DetailedRobotStateParameterAssignment) {
    DetailedRobotState state;
    
    // 设置基本状态
    state.state = RobotState::ACTIVE;
    state.health_level = HealthLevel::GOOD;
    state.health_score = 0.85f;
    state.error_code = 0;
    
    // 设置运动状态
    state.motion.mode = 3;  // LOCOMOTION
    state.motion.gait_type = 1;  // TROT
    state.motion.progress = 0.5f;
    state.motion.body_height = 0.30f;
    state.motion.position.x = 1.5f;
    state.motion.velocity.x = 0.8f;
    state.motion.yaw_speed = 0.3f;
    
    // 设置传感器状态
    state.sensors.imu_online = true;
    state.sensors.lidar_online = true;
    state.sensors.imu_temperature = 35;
    state.sensors.lidar_frequency = 20.0f;
    
    // 设置通信状态
    state.communication.ros2_online = true;
    state.communication.message_lost_count = 5;
    state.communication.communication_quality = 0.98f;
    
    // 验证设置
    EXPECT_EQ(state.state, RobotState::ACTIVE);
    EXPECT_EQ(state.health_level, HealthLevel::GOOD);
    EXPECT_FLOAT_EQ(state.health_score, 0.85f);
    EXPECT_EQ(state.motion.mode, 3);
    EXPECT_EQ(state.motion.gait_type, 1);
    EXPECT_FLOAT_EQ(state.motion.progress, 0.5f);
    EXPECT_FLOAT_EQ(state.motion.body_height, 0.30f);
    EXPECT_TRUE(state.sensors.imu_online);
    EXPECT_TRUE(state.sensors.lidar_online);
    EXPECT_TRUE(state.communication.ros2_online);
    EXPECT_FLOAT_EQ(state.communication.communication_quality, 0.98f);
}

// ========== AlertInfo 测试 ==========

TEST_F(RobotStateTypesTest, AlertInfoDefaultConstructor) {
    AlertInfo alert;
    
    EXPECT_EQ(alert.type, AlertType::INFO);
    EXPECT_EQ(alert.module, SystemModule::MOTION_CONTROL);
    EXPECT_EQ(alert.code, 0);
    EXPECT_TRUE(alert.message.empty());
    EXPECT_TRUE(alert.description.empty());
    EXPECT_EQ(alert.timestamp_ns, 0);
    EXPECT_TRUE(alert.is_active);
    EXPECT_EQ(alert.occurrence_count, 1);
    EXPECT_TRUE(alert.numeric_data.empty());
    EXPECT_TRUE(alert.string_data.empty());
}

TEST_F(RobotStateTypesTest, AlertInfoParameterizedConstructor) {
    AlertInfo alert(AlertType::WARNING, SystemModule::POWER_MANAGEMENT, 2001, "Low battery warning");
    
    EXPECT_EQ(alert.type, AlertType::WARNING);
    EXPECT_EQ(alert.module, SystemModule::POWER_MANAGEMENT);
    EXPECT_EQ(alert.code, 2001);
    EXPECT_EQ(alert.message, "Low battery warning");
    EXPECT_TRUE(alert.is_active);
    EXPECT_EQ(alert.occurrence_count, 1);
}

TEST_F(RobotStateTypesTest, AlertInfoDataMaps) {
    AlertInfo alert;
    
    // 添加数值数据
    alert.numeric_data["temperature"] = 85.5f;
    alert.numeric_data["voltage"] = 12.3f;
    
    // 添加字符串数据
    alert.string_data["device"] = "motor_0";
    alert.string_data["location"] = "front_right";
    
    EXPECT_FLOAT_EQ(alert.numeric_data["temperature"], 85.5f);
    EXPECT_FLOAT_EQ(alert.numeric_data["voltage"], 12.3f);
    EXPECT_EQ(alert.string_data["device"], "motor_0");
    EXPECT_EQ(alert.string_data["location"], "front_right");
}

// ========== DiagnosticInfo 测试 ==========

TEST_F(RobotStateTypesTest, DiagnosticInfoDefaultConstructor) {
    DiagnosticInfo diagnostic;
    
    EXPECT_EQ(diagnostic.health_level, HealthLevel::UNKNOWN);
    EXPECT_FLOAT_EQ(diagnostic.health_score, 0.0f);
    EXPECT_TRUE(diagnostic.status_message.empty());
    EXPECT_EQ(diagnostic.last_update_ns, 0);
    EXPECT_TRUE(diagnostic.metrics.empty());
    EXPECT_TRUE(diagnostic.status_flags.empty());
    EXPECT_TRUE(diagnostic.active_alerts.empty());
}

TEST_F(RobotStateTypesTest, DiagnosticInfoParameterizedConstructor) {
    DiagnosticInfo diagnostic(SystemModule::SENSOR_SYSTEM);
    
    EXPECT_EQ(diagnostic.module, SystemModule::SENSOR_SYSTEM);
    EXPECT_EQ(diagnostic.health_level, HealthLevel::UNKNOWN);
}

TEST_F(RobotStateTypesTest, DiagnosticInfoMetricsAndFlags) {
    DiagnosticInfo diagnostic(SystemModule::MOTOR_DRIVERS);
    
    // 添加指标
    diagnostic.metrics["average_temperature"] = 45.2f;
    diagnostic.metrics["max_torque"] = 25.8f;
    diagnostic.metrics["efficiency"] = 0.92f;
    
    // 添加状态标志
    diagnostic.status_flags["all_motors_online"] = true;
    diagnostic.status_flags["thermal_protection"] = false;
    diagnostic.status_flags["calibration_required"] = false;
    
    // 添加告警
    AlertInfo motor_alert(AlertType::WARNING, SystemModule::MOTOR_DRIVERS, 1001, "Motor temperature high");
    diagnostic.active_alerts.push_back(motor_alert);
    
    EXPECT_FLOAT_EQ(diagnostic.metrics["average_temperature"], 45.2f);
    EXPECT_TRUE(diagnostic.status_flags["all_motors_online"]);
    EXPECT_FALSE(diagnostic.status_flags["thermal_protection"]);
    EXPECT_EQ(diagnostic.active_alerts.size(), 1);
    EXPECT_EQ(diagnostic.active_alerts[0].code, 1001);
}

// ========== PerformanceStats 测试 ==========

TEST_F(RobotStateTypesTest, PerformanceStatsDefaultConstructor) {
    PerformanceStats stats;
    
    // 运动性能
    EXPECT_FLOAT_EQ(stats.motion.max_speed_achieved, 0.0f);
    EXPECT_FLOAT_EQ(stats.motion.average_speed, 0.0f);
    EXPECT_FLOAT_EQ(stats.motion.total_distance, 0.0f);
    EXPECT_EQ(stats.motion.step_count, 0);
    EXPECT_FLOAT_EQ(stats.motion.uptime_hours, 0.0f);
    
    // 电池性能
    EXPECT_EQ(stats.power.charge_cycles, 0);
    EXPECT_FLOAT_EQ(stats.power.average_consumption, 0.0f);
    EXPECT_FLOAT_EQ(stats.power.efficiency_score, 1.0f);
    
    // 传感器性能
    EXPECT_EQ(stats.sensors.lidar_scan_count, 0);
    EXPECT_FLOAT_EQ(stats.sensors.average_lidar_frequency, 0.0f);
    EXPECT_EQ(stats.sensors.imu_sample_count, 0);
    EXPECT_FLOAT_EQ(stats.sensors.sensor_error_rate, 0.0f);
    
    // 通信性能
    EXPECT_EQ(stats.communication.messages_sent, 0);
    EXPECT_EQ(stats.communication.messages_received, 0);
    EXPECT_EQ(stats.communication.messages_lost, 0);
    EXPECT_FLOAT_EQ(stats.communication.average_latency_ms, 0.0f);
}

TEST_F(RobotStateTypesTest, PerformanceStatsAssignment) {
    PerformanceStats stats;
    
    // 设置运动性能数据
    stats.motion.max_speed_achieved = 1.2f;
    stats.motion.average_speed = 0.6f;
    stats.motion.total_distance = 1500.5f;
    stats.motion.step_count = 25000;
    stats.motion.uptime_hours = 10.5f;
    
    // 设置电池性能数据
    stats.power.charge_cycles = 15;
    stats.power.average_consumption = 45.2f;
    stats.power.efficiency_score = 0.88f;
    
    // 设置传感器性能数据
    stats.sensors.lidar_scan_count = 50000;
    stats.sensors.average_lidar_frequency = 20.0f;
    stats.sensors.imu_sample_count = 1000000;
    stats.sensors.sensor_error_rate = 0.001f;
    
    // 设置通信性能数据
    stats.communication.messages_sent = 150000;
    stats.communication.messages_received = 149800;
    stats.communication.messages_lost = 200;
    stats.communication.average_latency_ms = 2.5f;
    
    // 验证设置
    EXPECT_FLOAT_EQ(stats.motion.max_speed_achieved, 1.2f);
    EXPECT_FLOAT_EQ(stats.motion.total_distance, 1500.5f);
    EXPECT_EQ(stats.motion.step_count, 25000);
    EXPECT_EQ(stats.power.charge_cycles, 15);
    EXPECT_FLOAT_EQ(stats.power.efficiency_score, 0.88f);
    EXPECT_EQ(stats.sensors.lidar_scan_count, 50000);
    EXPECT_FLOAT_EQ(stats.sensors.sensor_error_rate, 0.001f);
    EXPECT_EQ(stats.communication.messages_lost, 200);
    EXPECT_FLOAT_EQ(stats.communication.average_latency_ms, 2.5f);
}

// ========== Go2StateConfig 测试 ==========

TEST_F(RobotStateTypesTest, Go2StateConfigDefaultValues) {
    Go2StateConfig config;
    
    // 状态监控频率
    EXPECT_FLOAT_EQ(config.motion_state_frequency, 50.0f);
    EXPECT_FLOAT_EQ(config.low_state_frequency, 100.0f);
    EXPECT_FLOAT_EQ(config.diagnostic_frequency, 1.0f);
    
    // 健康评估阈值
    EXPECT_FLOAT_EQ(config.health_thresholds.excellent_threshold, 0.9f);
    EXPECT_FLOAT_EQ(config.health_thresholds.good_threshold, 0.7f);
    EXPECT_FLOAT_EQ(config.health_thresholds.fair_threshold, 0.5f);
    EXPECT_FLOAT_EQ(config.health_thresholds.poor_threshold, 0.3f);
    
    // 温度阈值
    EXPECT_EQ(config.temperature_limits.motor_warning, 60);
    EXPECT_EQ(config.temperature_limits.motor_critical, 80);
    EXPECT_EQ(config.temperature_limits.cpu_warning, 70);
    EXPECT_EQ(config.temperature_limits.cpu_critical, 90);
    
    // 通信质量阈值
    EXPECT_FLOAT_EQ(config.communication_thresholds.good_quality, 0.95f);
    EXPECT_FLOAT_EQ(config.communication_thresholds.poor_quality, 0.8f);
    EXPECT_EQ(config.communication_thresholds.max_lost_messages, 100);
}

TEST_F(RobotStateTypesTest, Go2StateConfigCustomValues) {
    Go2StateConfig config;
    
    // 修改配置
    config.motion_state_frequency = 60.0f;
    config.diagnostic_frequency = 2.0f;
    config.health_thresholds.excellent_threshold = 0.95f;
    config.temperature_limits.motor_warning = 55;
    config.communication_thresholds.good_quality = 0.98f;
    
    // 验证修改
    EXPECT_FLOAT_EQ(config.motion_state_frequency, 60.0f);
    EXPECT_FLOAT_EQ(config.diagnostic_frequency, 2.0f);
    EXPECT_FLOAT_EQ(config.health_thresholds.excellent_threshold, 0.95f);
    EXPECT_EQ(config.temperature_limits.motor_warning, 55);
    EXPECT_FLOAT_EQ(config.communication_thresholds.good_quality, 0.98f);
}

// ========== 边界值和特殊情况测试 ==========

TEST_F(RobotStateTypesTest, Go2SpecificConstraints) {
    DetailedRobotState state;
    
    // 验证Go2特定配置
    EXPECT_EQ(state.motors.size(), 20);  // Go2有20个电机
    EXPECT_EQ(state.feet.size(), 4);     // 四足机器人
    EXPECT_EQ(state.range_obstacles.size(), 4);  // 4个方向的障碍物检测
    
    // 验证默认机身高度符合Go2规格
    EXPECT_FLOAT_EQ(state.motion.body_height, 0.32f);  // Go2默认站立高度
    EXPECT_FLOAT_EQ(state.motion.foot_raise_height, 0.09f);  // 默认足端抬起高度
    
    // 测试足端ID范围
    for (size_t i = 0; i < state.feet.size(); ++i) {
        EXPECT_GE(state.feet[i].foot_id, 0);
        EXPECT_LT(state.feet[i].foot_id, 4);
    }
    
    // 测试电机ID范围
    for (size_t i = 0; i < state.motors.size(); ++i) {
        EXPECT_GE(state.motors[i].motor_id, 0);
        EXPECT_LT(state.motors[i].motor_id, 20);
    }
}

TEST_F(RobotStateTypesTest, HealthScoreValidRange) {
    DetailedRobotState state;
    DiagnosticInfo diagnostic;
    
    // 健康评分应该在0.0-1.0范围内
    state.health_score = 0.0f;
    EXPECT_GE(state.health_score, 0.0f);
    EXPECT_LE(state.health_score, 1.0f);
    
    state.health_score = 1.0f;
    EXPECT_GE(state.health_score, 0.0f);
    EXPECT_LE(state.health_score, 1.0f);
    
    // 诊断信息也应该遵循同样的范围
    diagnostic.health_score = 0.75f;
    EXPECT_GE(diagnostic.health_score, 0.0f);
    EXPECT_LE(diagnostic.health_score, 1.0f);
}

TEST_F(RobotStateTypesTest, TemperatureLimitsRealistic) {
    Go2StateConfig config;
    
    // 验证温度阈值在合理范围内
    EXPECT_GT(config.temperature_limits.motor_warning, 30);    // 应该高于室温
    EXPECT_LT(config.temperature_limits.motor_warning, 100);   // 应该低于沸点
    EXPECT_GT(config.temperature_limits.motor_critical, config.temperature_limits.motor_warning);
    
    EXPECT_GT(config.temperature_limits.cpu_warning, 50);     // CPU通常运行较热
    EXPECT_LT(config.temperature_limits.cpu_warning, 100);
    EXPECT_GT(config.temperature_limits.cpu_critical, config.temperature_limits.cpu_warning);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}