/**
 * @file   go2_config_manager.cpp
 * @brief  Go2适配器配置管理器实现
 * @author Yang Nan
 * @date   2025-09-18
 */

#include "robot_adapters/go2_adapter/go2_config_manager.hpp"

#include <iostream>
#include <fstream>
#include <regex>
#include <yaml-cpp/yaml.h>

namespace robot_adapters {
namespace go2_adapter {

Go2ConfigManager::Go2ConfigManager(const std::string& config_file_path)
    : config_file_path_(config_file_path)
    , config_loaded_(false)
{
    if (config_file_path_.empty()) {
        config_file_path_ = getDefaultConfigPath();
    }

    loadDefaultConfig();

    if (configFileExists()) {
        if (!loadConfig(config_file_path_)) {
            std::cerr << "Warning: Failed to load config file: " << config_file_path_
                      << ". Using default configuration." << std::endl;
        }
    } else {
        std::cout << "Config file not found: " << config_file_path_
                  << ". Using default configuration." << std::endl;
    }
}

bool Go2ConfigManager::loadConfig(const std::string& config_file_path) {
    try {
        if (!config_file_path.empty()) {
            config_file_path_ = config_file_path;
        }

        if (!configFileExists()) {
            throw ConfigException("Config file does not exist: " + config_file_path_);
        }

        YAML::Node config = YAML::LoadFile(config_file_path_);

        // 加载通信配置
        if (config["communication"]) {
            const YAML::Node& comm_config = config["communication"];
            loadCommunicationConfig(&comm_config);
        }

        // 加载消息转换器配置
        if (config["message_converter"]) {
            const YAML::Node& converter_config = config["message_converter"];
            loadMessageConverterConfig(&converter_config);
        }

        // 验证配置
        if (!validateConfig()) {
            throw ConfigException("Configuration validation failed");
        }

        config_loaded_ = true;
        std::cout << "Successfully loaded config from: " << config_file_path_ << std::endl;
        return true;

    } catch (const YAML::Exception& e) {
        std::cerr << "YAML parsing error: " << e.what() << std::endl;
        return false;
    } catch (const ConfigException& e) {
        std::cerr << e.what() << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "Unexpected error loading config: " << e.what() << std::endl;
        return false;
    }
}

bool Go2ConfigManager::reloadConfig() {
    return loadConfig(config_file_path_);
}

const CommunicationConfig& Go2ConfigManager::getCommunicationConfig() const {
    return communication_config_;
}

const MessageConverterConfig& Go2ConfigManager::getMessageConverterConfig() const {
    return message_converter_config_;
}

bool Go2ConfigManager::validateConfig() const {
    // 验证IP地址
    if (!isValidIPAddress(communication_config_.network.robot_ip)) {
        std::cerr << "Invalid robot IP address: " << communication_config_.network.robot_ip << std::endl;
        return false;
    }

    if (!isValidIPAddress(communication_config_.network.local_ip)) {
        std::cerr << "Invalid local IP address: " << communication_config_.network.local_ip << std::endl;
        return false;
    }

    // 验证网络接口
    if (communication_config_.network.network_interface.empty()) {
        std::cerr << "Network interface cannot be empty" << std::endl;
        return false;
    }

    // 验证DDS域ID
    if (communication_config_.dds.domain_id < 0 || communication_config_.dds.domain_id > 232) {
        std::cerr << "Invalid DDS domain ID: " << communication_config_.dds.domain_id
                  << " (must be 0-232)" << std::endl;
        return false;
    }

    // 验证超时设置
    if (communication_config_.connection.timeout_ms <= 0) {
        std::cerr << "Connection timeout must be positive: "
                  << communication_config_.connection.timeout_ms << std::endl;
        return false;
    }

    if (communication_config_.connection.reconnect_interval_ms <= 0) {
        std::cerr << "Reconnect interval must be positive: "
                  << communication_config_.connection.reconnect_interval_ms << std::endl;
        return false;
    }

    if (communication_config_.connection.max_reconnect_attempts < 0) {
        std::cerr << "Max reconnect attempts cannot be negative: "
                  << communication_config_.connection.max_reconnect_attempts << std::endl;
        return false;
    }

    return true;
}

const std::string& Go2ConfigManager::getConfigFilePath() const {
    return config_file_path_;
}

bool Go2ConfigManager::configFileExists() const {
    std::ifstream file(config_file_path_);
    return file.good();
}

std::string Go2ConfigManager::getDefaultConfigPath() {
    // 返回相对于当前包的默认配置路径
    return "config/go2_adapter_config.yaml";
}

void Go2ConfigManager::loadDefaultConfig() {
    // 加载默认的通信配置
    communication_config_.network.robot_ip = "192.168.123.18";
    communication_config_.network.local_ip = "192.168.123.99";
    communication_config_.network.network_interface = "enp129s0";

    communication_config_.dds.domain_id = 0;

    communication_config_.connection.timeout_ms = 10000;
    communication_config_.connection.reconnect_interval_ms = 5000;
    communication_config_.connection.max_reconnect_attempts = 0;

    // 加载默认的消息转换器配置
    message_converter_config_.conversion_options.validate_ranges = true;
    message_converter_config_.conversion_options.fill_missing_data = true;
    message_converter_config_.conversion_options.preserve_timestamps = true;
    message_converter_config_.conversion_options.default_timeout_s = 1.0f;
    message_converter_config_.conversion_options.use_go2_coordinate_frame = true;
    message_converter_config_.conversion_options.enable_go2_extensions = true;
    message_converter_config_.conversion_options.strict_validation = false;

    message_converter_config_.buffer_sizes.sport_mode_state = 10;
    message_converter_config_.buffer_sizes.low_state = 10;
    message_converter_config_.buffer_sizes.bms_state = 10;
    message_converter_config_.buffer_sizes.point_cloud = 5;
    message_converter_config_.buffer_sizes.imu_data = 20;
    message_converter_config_.buffer_sizes.odometry = 10;

    // 速度验证范围
    message_converter_config_.validation_ranges.velocity.linear_x_max = 1.5f;
    message_converter_config_.validation_ranges.velocity.linear_y_max = 1.0f;
    message_converter_config_.validation_ranges.velocity.angular_z_max = 2.0f;
    message_converter_config_.validation_ranges.velocity.movement_threshold = 0.01f;

    // 姿态验证范围
    message_converter_config_.validation_ranges.posture.roll_max = 0.5f;
    message_converter_config_.validation_ranges.posture.pitch_max = 0.5f;
    message_converter_config_.validation_ranges.posture.yaw_max = 3.14159f;
    message_converter_config_.validation_ranges.posture.body_height_min = 0.05f;
    message_converter_config_.validation_ranges.posture.body_height_max = 0.4f;

    // 电池验证范围
    message_converter_config_.validation_ranges.battery.voltage_min = 20.0f;
    message_converter_config_.validation_ranges.battery.voltage_max = 30.0f;
    message_converter_config_.validation_ranges.battery.current_max = 30.0f;
    message_converter_config_.validation_ranges.battery.temperature_min = -10.0f;
    message_converter_config_.validation_ranges.battery.temperature_max = 60.0f;
    message_converter_config_.validation_ranges.battery.capacity_min = 0.0f;
    message_converter_config_.validation_ranges.battery.capacity_max = 100.0f;
    message_converter_config_.validation_ranges.battery.nominal_voltage = 25.2f;
    message_converter_config_.validation_ranges.battery.capacity_mah = 15000;
    message_converter_config_.validation_ranges.battery.cell_count = 15;
    message_converter_config_.validation_ranges.battery.default_cell_temperature = 25.0f;

    // 电池健康度验证范围
    message_converter_config_.validation_ranges.battery_health.excellent_cycles = 100;
    message_converter_config_.validation_ranges.battery_health.good_cycles = 300;
    message_converter_config_.validation_ranges.battery_health.fair_cycles = 600;
    message_converter_config_.validation_ranges.battery_health.poor_cycles = 1000;
    message_converter_config_.validation_ranges.battery_health.alarm_status = 11;
    message_converter_config_.validation_ranges.battery_health.default_health_percentage = 100.0f;

    // 传感器验证范围
    message_converter_config_.validation_ranges.sensor.imu_gyroscope_max = 35.0f;
    message_converter_config_.validation_ranges.sensor.imu_accelerometer_max = 157.0f;
    message_converter_config_.validation_ranges.sensor.imu_temperature_min = -40.0f;
    message_converter_config_.validation_ranges.sensor.imu_temperature_max = 85.0f;
    message_converter_config_.validation_ranges.sensor.imu_default_temperature = 25.0f;
    message_converter_config_.validation_ranges.sensor.quaternion_tolerance = 0.1f;
    message_converter_config_.validation_ranges.sensor.motor_temperature_min = -100.0f;
    message_converter_config_.validation_ranges.sensor.motor_temperature_max = 100.0f;
    message_converter_config_.validation_ranges.sensor.motor_temp_clamp_min = -50.0f;
    message_converter_config_.validation_ranges.sensor.motor_temp_clamp_max = 100.0f;
    message_converter_config_.validation_ranges.sensor.motor_position_max = 6.28f;
    message_converter_config_.validation_ranges.sensor.motor_velocity_max = 50.0f;
    message_converter_config_.validation_ranges.sensor.motor_count = 12;
    message_converter_config_.validation_ranges.sensor.foot_contact_threshold = 10.0f;
    message_converter_config_.validation_ranges.sensor.foot_force_min = -100.0f;
    message_converter_config_.validation_ranges.sensor.foot_force_max = 1000.0f;
    message_converter_config_.validation_ranges.sensor.foot_count = 4;

    // 控制器验证范围
    message_converter_config_.validation_ranges.controller.deadzone = 0.05f;
    message_converter_config_.validation_ranges.controller.activity_threshold = 0.01f;
    message_converter_config_.validation_ranges.controller.stick_range = 1.0f;

    // 时间验证范围
    message_converter_config_.validation_ranges.timing.nanoseconds_per_second = 1000000000ULL;

    // 里程计验证范围
    message_converter_config_.validation_ranges.odometry.position_covariance = 0.1f;
    message_converter_config_.validation_ranges.odometry.orientation_covariance = 0.1f;
    message_converter_config_.validation_ranges.odometry.velocity_covariance = 0.1f;

    // 初始化默认的3x3单位矩阵
    message_converter_config_.coordinate_transforms.go2_to_ros_matrix = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };
    message_converter_config_.coordinate_transforms.ros_to_go2_matrix = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };

    // 初始化默认的类型映射
    message_converter_config_.type_mappings.motion_modes = {
        {0, "IDLE"}, {1, "WALK"}, {2, "TROT"}, {3, "RUN"}, {4, "CLIMB_STAIR"}, {5, "DOWN_STAIR"}
    };
    message_converter_config_.type_mappings.gait_types = {
        {0, "STAND"}, {1, "TROT"}, {2, "WALK"}, {3, "BOUND"}
    };
    message_converter_config_.type_mappings.battery_health = {
        {0, "UNKNOWN"}, {1, "GOOD"}, {2, "OVERHEAT"}, {3, "DEAD"}, {4, "OVER_VOLTAGE"}, {5, "UNSPECIFIED_FAILURE"}, {6, "COLD"}
    };
    message_converter_config_.type_mappings.charging_states = {
        {0, "UNKNOWN"}, {1, "CHARGING"}, {2, "DISCHARGING"}, {3, "NOT_CHARGING"}, {4, "CHARGE_FAULT"}
    };

    config_loaded_ = true;
}

void Go2ConfigManager::loadCommunicationConfig(const void* config_node) {
    const YAML::Node* node = static_cast<const YAML::Node*>(config_node);

    if (!node || !node->IsMap()) {
        return;
    }

    // 加载网络配置
    if ((*node)["network"]) {
        const YAML::Node& network = (*node)["network"];
        if (network["robot_ip"]) {
            communication_config_.network.robot_ip = network["robot_ip"].as<std::string>();
        }
        if (network["local_ip"]) {
            communication_config_.network.local_ip = network["local_ip"].as<std::string>();
        }
        if (network["network_interface"]) {
            communication_config_.network.network_interface = network["network_interface"].as<std::string>();
        }
    }

    // 加载DDS配置
    if ((*node)["dds"]) {
        const YAML::Node& dds = (*node)["dds"];
        if (dds["domain_id"]) {
            communication_config_.dds.domain_id = dds["domain_id"].as<int>();
        }
    }

    // 加载连接配置
    if ((*node)["connection"]) {
        const YAML::Node& connection = (*node)["connection"];
        if (connection["timeout_ms"]) {
            communication_config_.connection.timeout_ms = connection["timeout_ms"].as<int>();
        }
        if (connection["reconnect_interval_ms"]) {
            communication_config_.connection.reconnect_interval_ms = connection["reconnect_interval_ms"].as<int>();
        }
        if (connection["max_reconnect_attempts"]) {
            communication_config_.connection.max_reconnect_attempts = connection["max_reconnect_attempts"].as<int>();
        }
    }
}

void Go2ConfigManager::loadMessageConverterConfig(const void* config_node) {
    const YAML::Node* node = static_cast<const YAML::Node*>(config_node);

    if (!node || !node->IsMap()) {
        return;
    }

    // 加载转换选项配置
    if ((*node)["conversion_options"]) {
        const YAML::Node& options = (*node)["conversion_options"];
        if (options["validate_ranges"]) {
            message_converter_config_.conversion_options.validate_ranges = options["validate_ranges"].as<bool>();
        }
        if (options["fill_missing_data"]) {
            message_converter_config_.conversion_options.fill_missing_data = options["fill_missing_data"].as<bool>();
        }
        if (options["preserve_timestamps"]) {
            message_converter_config_.conversion_options.preserve_timestamps = options["preserve_timestamps"].as<bool>();
        }
        if (options["default_timeout_s"]) {
            message_converter_config_.conversion_options.default_timeout_s = options["default_timeout_s"].as<float>();
        }
        if (options["use_go2_coordinate_frame"]) {
            message_converter_config_.conversion_options.use_go2_coordinate_frame = options["use_go2_coordinate_frame"].as<bool>();
        }
        if (options["enable_go2_extensions"]) {
            message_converter_config_.conversion_options.enable_go2_extensions = options["enable_go2_extensions"].as<bool>();
        }
        if (options["strict_validation"]) {
            message_converter_config_.conversion_options.strict_validation = options["strict_validation"].as<bool>();
        }
    }

    // 加载缓冲区大小配置
    if ((*node)["buffer_sizes"]) {
        const YAML::Node& buffers = (*node)["buffer_sizes"];
        if (buffers["sport_mode_state"]) {
            message_converter_config_.buffer_sizes.sport_mode_state = buffers["sport_mode_state"].as<size_t>();
        }
        if (buffers["low_state"]) {
            message_converter_config_.buffer_sizes.low_state = buffers["low_state"].as<size_t>();
        }
        if (buffers["bms_state"]) {
            message_converter_config_.buffer_sizes.bms_state = buffers["bms_state"].as<size_t>();
        }
        if (buffers["point_cloud"]) {
            message_converter_config_.buffer_sizes.point_cloud = buffers["point_cloud"].as<size_t>();
        }
        if (buffers["imu_data"]) {
            message_converter_config_.buffer_sizes.imu_data = buffers["imu_data"].as<size_t>();
        }
        if (buffers["odometry"]) {
            message_converter_config_.buffer_sizes.odometry = buffers["odometry"].as<size_t>();
        }
    }

    // 加载验证范围配置
    if ((*node)["validation_ranges"]) {
        const YAML::Node& ranges = (*node)["validation_ranges"];

        // 速度范围
        if (ranges["velocity"]) {
            const YAML::Node& velocity = ranges["velocity"];
            if (velocity["linear_x_max"]) {
                message_converter_config_.validation_ranges.velocity.linear_x_max = velocity["linear_x_max"].as<float>();
            }
            if (velocity["linear_y_max"]) {
                message_converter_config_.validation_ranges.velocity.linear_y_max = velocity["linear_y_max"].as<float>();
            }
            if (velocity["angular_z_max"]) {
                message_converter_config_.validation_ranges.velocity.angular_z_max = velocity["angular_z_max"].as<float>();
            }
            if (velocity["movement_threshold"]) {
                message_converter_config_.validation_ranges.velocity.movement_threshold = velocity["movement_threshold"].as<float>();
            }
        }

        // 姿态范围
        if (ranges["posture"]) {
            const YAML::Node& posture = ranges["posture"];
            if (posture["roll_max"]) {
                message_converter_config_.validation_ranges.posture.roll_max = posture["roll_max"].as<float>();
            }
            if (posture["pitch_max"]) {
                message_converter_config_.validation_ranges.posture.pitch_max = posture["pitch_max"].as<float>();
            }
            if (posture["yaw_max"]) {
                message_converter_config_.validation_ranges.posture.yaw_max = posture["yaw_max"].as<float>();
            }
            if (posture["body_height_min"]) {
                message_converter_config_.validation_ranges.posture.body_height_min = posture["body_height_min"].as<float>();
            }
            if (posture["body_height_max"]) {
                message_converter_config_.validation_ranges.posture.body_height_max = posture["body_height_max"].as<float>();
            }
        }

        // 电池范围
        if (ranges["battery"]) {
            const YAML::Node& battery = ranges["battery"];
            if (battery["voltage_min"]) {
                message_converter_config_.validation_ranges.battery.voltage_min = battery["voltage_min"].as<float>();
            }
            if (battery["voltage_max"]) {
                message_converter_config_.validation_ranges.battery.voltage_max = battery["voltage_max"].as<float>();
            }
            if (battery["current_max"]) {
                message_converter_config_.validation_ranges.battery.current_max = battery["current_max"].as<float>();
            }
            if (battery["temperature_min"]) {
                message_converter_config_.validation_ranges.battery.temperature_min = battery["temperature_min"].as<float>();
            }
            if (battery["temperature_max"]) {
                message_converter_config_.validation_ranges.battery.temperature_max = battery["temperature_max"].as<float>();
            }
            if (battery["capacity_min"]) {
                message_converter_config_.validation_ranges.battery.capacity_min = battery["capacity_min"].as<float>();
            }
            if (battery["capacity_max"]) {
                message_converter_config_.validation_ranges.battery.capacity_max = battery["capacity_max"].as<float>();
            }
            if (battery["nominal_voltage"]) {
                message_converter_config_.validation_ranges.battery.nominal_voltage = battery["nominal_voltage"].as<float>();
            }
            if (battery["capacity_mah"]) {
                message_converter_config_.validation_ranges.battery.capacity_mah = battery["capacity_mah"].as<uint16_t>();
            }
            if (battery["cell_count"]) {
                message_converter_config_.validation_ranges.battery.cell_count = battery["cell_count"].as<uint8_t>();
            }
            if (battery["default_cell_temperature"]) {
                message_converter_config_.validation_ranges.battery.default_cell_temperature = battery["default_cell_temperature"].as<float>();
            }
        }

        // 电池健康度范围
        if (ranges["battery_health"]) {
            const YAML::Node& battery_health = ranges["battery_health"];
            if (battery_health["excellent_cycles"]) {
                message_converter_config_.validation_ranges.battery_health.excellent_cycles = battery_health["excellent_cycles"].as<uint16_t>();
            }
            if (battery_health["good_cycles"]) {
                message_converter_config_.validation_ranges.battery_health.good_cycles = battery_health["good_cycles"].as<uint16_t>();
            }
            if (battery_health["fair_cycles"]) {
                message_converter_config_.validation_ranges.battery_health.fair_cycles = battery_health["fair_cycles"].as<uint16_t>();
            }
            if (battery_health["poor_cycles"]) {
                message_converter_config_.validation_ranges.battery_health.poor_cycles = battery_health["poor_cycles"].as<uint16_t>();
            }
            if (battery_health["alarm_status"]) {
                message_converter_config_.validation_ranges.battery_health.alarm_status = battery_health["alarm_status"].as<uint8_t>();
            }
            if (battery_health["default_health_percentage"]) {
                message_converter_config_.validation_ranges.battery_health.default_health_percentage = battery_health["default_health_percentage"].as<float>();
            }
        }

        // 传感器范围
        if (ranges["sensor"]) {
            const YAML::Node& sensor = ranges["sensor"];
            if (sensor["imu_gyroscope_max"]) {
                message_converter_config_.validation_ranges.sensor.imu_gyroscope_max = sensor["imu_gyroscope_max"].as<float>();
            }
            if (sensor["imu_accelerometer_max"]) {
                message_converter_config_.validation_ranges.sensor.imu_accelerometer_max = sensor["imu_accelerometer_max"].as<float>();
            }
            if (sensor["imu_temperature_min"]) {
                message_converter_config_.validation_ranges.sensor.imu_temperature_min = sensor["imu_temperature_min"].as<float>();
            }
            if (sensor["imu_temperature_max"]) {
                message_converter_config_.validation_ranges.sensor.imu_temperature_max = sensor["imu_temperature_max"].as<float>();
            }
            if (sensor["imu_default_temperature"]) {
                message_converter_config_.validation_ranges.sensor.imu_default_temperature = sensor["imu_default_temperature"].as<float>();
            }
            if (sensor["quaternion_tolerance"]) {
                message_converter_config_.validation_ranges.sensor.quaternion_tolerance = sensor["quaternion_tolerance"].as<float>();
            }
            if (sensor["motor_temperature_min"]) {
                message_converter_config_.validation_ranges.sensor.motor_temperature_min = sensor["motor_temperature_min"].as<float>();
            }
            if (sensor["motor_temperature_max"]) {
                message_converter_config_.validation_ranges.sensor.motor_temperature_max = sensor["motor_temperature_max"].as<float>();
            }
            if (sensor["motor_temp_clamp_min"]) {
                message_converter_config_.validation_ranges.sensor.motor_temp_clamp_min = sensor["motor_temp_clamp_min"].as<float>();
            }
            if (sensor["motor_temp_clamp_max"]) {
                message_converter_config_.validation_ranges.sensor.motor_temp_clamp_max = sensor["motor_temp_clamp_max"].as<float>();
            }
            if (sensor["motor_position_max"]) {
                message_converter_config_.validation_ranges.sensor.motor_position_max = sensor["motor_position_max"].as<float>();
            }
            if (sensor["motor_velocity_max"]) {
                message_converter_config_.validation_ranges.sensor.motor_velocity_max = sensor["motor_velocity_max"].as<float>();
            }
            if (sensor["motor_count"]) {
                message_converter_config_.validation_ranges.sensor.motor_count = sensor["motor_count"].as<uint8_t>();
            }
            if (sensor["foot_contact_threshold"]) {
                message_converter_config_.validation_ranges.sensor.foot_contact_threshold = sensor["foot_contact_threshold"].as<float>();
            }
            if (sensor["foot_force_min"]) {
                message_converter_config_.validation_ranges.sensor.foot_force_min = sensor["foot_force_min"].as<float>();
            }
            if (sensor["foot_force_max"]) {
                message_converter_config_.validation_ranges.sensor.foot_force_max = sensor["foot_force_max"].as<float>();
            }
            if (sensor["foot_count"]) {
                message_converter_config_.validation_ranges.sensor.foot_count = sensor["foot_count"].as<uint8_t>();
            }
        }

        // 控制器范围
        if (ranges["controller"]) {
            const YAML::Node& controller = ranges["controller"];
            if (controller["deadzone"]) {
                message_converter_config_.validation_ranges.controller.deadzone = controller["deadzone"].as<float>();
            }
            if (controller["activity_threshold"]) {
                message_converter_config_.validation_ranges.controller.activity_threshold = controller["activity_threshold"].as<float>();
            }
            if (controller["stick_range"]) {
                message_converter_config_.validation_ranges.controller.stick_range = controller["stick_range"].as<float>();
            }
        }

        // 时间范围
        if (ranges["timing"]) {
            const YAML::Node& timing = ranges["timing"];
            if (timing["nanoseconds_per_second"]) {
                message_converter_config_.validation_ranges.timing.nanoseconds_per_second = timing["nanoseconds_per_second"].as<uint64_t>();
            }
        }

        // 里程计范围
        if (ranges["odometry"]) {
            const YAML::Node& odometry = ranges["odometry"];
            if (odometry["position_covariance"]) {
                message_converter_config_.validation_ranges.odometry.position_covariance = odometry["position_covariance"].as<float>();
            }
            if (odometry["orientation_covariance"]) {
                message_converter_config_.validation_ranges.odometry.orientation_covariance = odometry["orientation_covariance"].as<float>();
            }
            if (odometry["velocity_covariance"]) {
                message_converter_config_.validation_ranges.odometry.velocity_covariance = odometry["velocity_covariance"].as<float>();
            }
        }
    }

    // 加载坐标变换配置
    if ((*node)["coordinate_transforms"]) {
        const YAML::Node& transforms = (*node)["coordinate_transforms"];

        // Go2到ROS变换矩阵
        if (transforms["go2_to_ros_matrix"]) {
            const YAML::Node& matrix = transforms["go2_to_ros_matrix"];
            if (matrix.IsSequence() && matrix.size() == 3) {
                message_converter_config_.coordinate_transforms.go2_to_ros_matrix.clear();
                for (size_t i = 0; i < 3; ++i) {
                    if (matrix[i].IsSequence() && matrix[i].size() == 3) {
                        std::vector<float> row = {
                            matrix[i][0].as<float>(),
                            matrix[i][1].as<float>(),
                            matrix[i][2].as<float>()
                        };
                        message_converter_config_.coordinate_transforms.go2_to_ros_matrix.push_back(row);
                    }
                }
            }
        }

        // ROS到Go2变换矩阵
        if (transforms["ros_to_go2_matrix"]) {
            const YAML::Node& matrix = transforms["ros_to_go2_matrix"];
            if (matrix.IsSequence() && matrix.size() == 3) {
                message_converter_config_.coordinate_transforms.ros_to_go2_matrix.clear();
                for (size_t i = 0; i < 3; ++i) {
                    if (matrix[i].IsSequence() && matrix[i].size() == 3) {
                        std::vector<float> row = {
                            matrix[i][0].as<float>(),
                            matrix[i][1].as<float>(),
                            matrix[i][2].as<float>()
                        };
                        message_converter_config_.coordinate_transforms.ros_to_go2_matrix.push_back(row);
                    }
                }
            }
        }
    }

    // 加载类型映射配置
    if ((*node)["type_mappings"]) {
        const YAML::Node& mappings = (*node)["type_mappings"];

        // 运动模式映射
        if (mappings["motion_modes"]) {
            const YAML::Node& modes = mappings["motion_modes"];
            for (const auto& pair : modes) {
                uint8_t key = pair.first.as<uint8_t>();
                std::string value = pair.second.as<std::string>();
                message_converter_config_.type_mappings.motion_modes[key] = value;
            }
        }

        // 步态类型映射
        if (mappings["gait_types"]) {
            const YAML::Node& gaits = mappings["gait_types"];
            for (const auto& pair : gaits) {
                uint8_t key = pair.first.as<uint8_t>();
                std::string value = pair.second.as<std::string>();
                message_converter_config_.type_mappings.gait_types[key] = value;
            }
        }

        // 电池健康状态映射
        if (mappings["battery_health"]) {
            const YAML::Node& health = mappings["battery_health"];
            for (const auto& pair : health) {
                uint8_t key = pair.first.as<uint8_t>();
                std::string value = pair.second.as<std::string>();
                message_converter_config_.type_mappings.battery_health[key] = value;
            }
        }

        // 充电状态映射
        if (mappings["charging_states"]) {
            const YAML::Node& states = mappings["charging_states"];
            for (const auto& pair : states) {
                uint8_t key = pair.first.as<uint8_t>();
                std::string value = pair.second.as<std::string>();
                message_converter_config_.type_mappings.charging_states[key] = value;
            }
        }
    }
}

bool Go2ConfigManager::isValidIPAddress(const std::string& ip) {
    // IPv4地址正则表达式
    std::regex ipv4_pattern(
        R"(^(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$)"
    );

    return std::regex_match(ip, ipv4_pattern);
}

bool Go2ConfigManager::isValidNetworkInterface(const std::string& interface) {
    // 网络接口名称一般格式: eth0, enp0s3, wlan0等
    std::regex interface_pattern(R"(^[a-zA-Z][a-zA-Z0-9]*[0-9]+(?:\.[0-9]+)*$)");

    return !interface.empty() && std::regex_match(interface, interface_pattern);
}

} // namespace go2_adapter
} // namespace robot_adapters