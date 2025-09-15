/**
 * @file   go2_adapter.cpp
 * @brief  Go2机器人适配器主类的实现文件
 * @author Yang nan
 * @date   2025-09-12
 *
 * @details
 * 本文件包含了`Go2Adapter`类所有方法的具体实现。
 * `Go2Adapter`是与Unitree Go2机器人硬件交互的核心桥梁，它封装了所有与机器人通信、
 * 控制、状态监控和传感器数据采集相关的复杂逻辑。
 *
 * 主要职责包括：
 * - **生命周期管理**: 控制适配器的初始化、运行和安全关闭。
 * - **配置加载与验证**: 从ROS参数服务器加载配置，并校验其有效性。
 * - **子系统初始化**: 根据配置动态创建和管理运动控制、传感器、状态监控等子模块。
 * - **ROS接口设置**: 创建服务和话题，将机器人的功能暴露给ROS2生态系统。
 * - **健康监控与诊断**: 提供心跳机制、系统自检和错误报告功能。
 */

#include "robot_adapters/go2_adapter/go2_adapter.hpp" // 引入Go2Adapter类的头文件定义

#include <chrono>      // 用于处理时间相关的操作，例如定时器和时间戳
#include <iostream>    // C++标准输入输出流（当前文件实现中未直接使用，主要通过RCLCPP日志系统输出）
#include <sstream>     // 用于高效构建字符串，例如生成JSON格式的状态和配置信息
#include <fstream>     // 用于文件读写操作，此处特用于从/proc文件系统读取内存使用情况
#include <cstdlib>     // 提供`system`函数，用于执行外部shell命令，例如ping命令来检查网络连通性
#include <set>         // 用于存储支持的能力集合

// 使用C++14标准的时间字面量，使得可以直接使用 `100ms` 这样的语法来表示时间段
using namespace std::chrono_literals;

namespace robot_adapters {
namespace go2_adapter {

// ============= 构造函数与析构函数 (Constructor & Destructor) =============

/**
 * @brief Go2Adapter类的构造函数
 * @param node_name ROS节点的名称
 * @param node_options ROS节点的配置选项
 *
 * @details
 * 初始化ROS节点，设置成员变量的初始状态，并调用`setupRosParameters`来声明和加载配置。
 * 构造函数执行时，适配器尚未完全初始化，仅创建了基础框架。
 */
Go2Adapter::Go2Adapter(const std::string& node_name, const rclcpp::NodeOptions& node_options)
    : rclcpp::Node(node_name, node_options) // 调用父类rclcpp::Node的构造函数
    , is_initialized_(false)                        // 初始化标志位，初始为false
    , is_operational_(false)                        // 可操作标志位，初始为false
    , current_status_("NOT_INITIALIZED")          // 当前状态字符串，初始为"未初始化"
    , last_error_code_(0)                          // 初始化错误代码
    , start_time_(std::chrono::steady_clock::now()) // 记录节点启动时间
{
    // 记录日志，表明Go2Adapter节点已创建
    RCLCPP_INFO(this->get_logger(), "Go2Adapter created with node name: %s", node_name.c_str());

    // 调用内部函数来设置和加载ROS参数
    setupRosParameters();

    // 记录日志，表明构造函数执行完毕
    RCLCPP_INFO(this->get_logger(), "Go2Adapter constructor completed");
}

/**
 * @brief Go2Adapter类的析构函数
 *
 * @details
 * 在节点销毁时被调用，负责调用`shutdown`方法来安全地释放所有资源，
 * 例如关闭子系统、取消定时器等。
 */
Go2Adapter::~Go2Adapter() {
    RCLCPP_INFO(this->get_logger(), "Go2Adapter destructor called");
    // 调用shutdown函数，确保资源被正确释放
    shutdown();
    RCLCPP_INFO(this->get_logger(), "Go2Adapter destructor completed");
}

// ============= 生命周期管理 (Lifecycle Management) =============

/**
 * @brief 初始化Go2适配器
 * @return 如果初始化成功，返回true；否则返回false。
 *
 * @details
 * 这是适配器的核心初始化函数，按顺序执行以下步骤：
 *      1. 检查是否已初始化，防止重复执行。
 *      2. 更新状态为"INITIALIZING"。
 *      3. 加载并验证配置。
 *      4. 初始化所有必要的子系统（如运动控制、传感器接口等）。
 *      5. 设置ROS相关的服务和话题。
 *      6. 启动一个心跳定时器，用于周期性健康检查。
 *      7. 如果所有步骤成功，更新状态为"OPERATIONAL"。
 * 任何步骤失败都会导致初始化失败，并记录详细错误信息。
 */
bool Go2Adapter::initialize() {
    // 使用互斥锁保护状态变量，确保线程安全
    std::lock_guard<std::mutex> lock(status_mutex_);

    // 如果已经初始化，则直接返回true，并打印警告信息
    if (is_initialized_) {
        RCLCPP_WARN(this->get_logger(), "Go2Adapter already initialized");
        return true;
    }

    RCLCPP_INFO(this->get_logger(), "Initializing Go2Adapter...");
    updateStatus("INITIALIZING"); // 更新当前状态

    try {
        // 步骤1: 加载配置参数
        if (!loadConfiguration()) {
            logError("Failed to load configuration");
            updateStatus("INITIALIZATION_FAILED");
            return false;
        }

        // 步骤2: 验证配置参数的有效性
        if (!validateConfiguration()) {
            logError("Invalid configuration");
            updateStatus("INITIALIZATION_FAILED");
            return false;
        }

        // 步骤3: 初始化所有子系统
        if (!initializeSubsystems()) {
            logError("Failed to initialize subsystems");
            updateStatus("INITIALIZATION_FAILED");
            return false;
        }

        // 步骤4: 设置ROS接口（服务、话题等）
        setupRosInterfaces();

        // 步骤5: 创建并启动心跳定时器，用于周期性检查
        heartbeat_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / config_.heartbeat_frequency)),
            [this]() { this->heartbeatCallback(); }
        );

        // 所有步骤成功后，更新状态
        is_initialized_ = true;
        is_operational_ = true;
        updateStatus("OPERATIONAL");

        RCLCPP_INFO(this->get_logger(), "Go2Adapter initialized successfully");
        return true;

    } catch (const std::exception& e) {
        // 捕获初始化过程中可能出现的任何标准异常
        logError(std::string("Exception during initialization: ") + e.what());
        updateStatus("INITIALIZATION_FAILED");
        return false;
    }
}

/**
 * @brief 关闭Go2适配器并释放资源
 *
 * @details
 * 执行与`initialize`相反的操作，按顺序安全地关闭所有组件：
 * 1. 停止心跳定时器。
 * 2. 关闭所有子系统。
 * 3. 更新状态标志位为"SHUTDOWN"。
 */
bool Go2Adapter::shutdown() {
    // 使用互斥锁保护状态变量
    std::lock_guard<std::mutex> lock(status_mutex_);

    // 如果未初始化，则无需关闭，直接返回true（表示成功）
    if (!is_initialized_) {
        RCLCPP_WARN(this->get_logger(), "Go2Adapter not initialized, nothing to shutdown");
        return true;
    }

    RCLCPP_INFO(this->get_logger(), "Shutting down Go2Adapter...");
    updateStatus("SHUTTING_DOWN");

    try {
        // 停止并重置心跳定时器
        if (heartbeat_timer_) {
            heartbeat_timer_->cancel();
            heartbeat_timer_.reset();
        }

        // 关闭所有子系统
        shutdownSubsystems();

        // 更新状态标志
        is_operational_ = false;
        is_initialized_ = false;
        updateStatus("SHUTDOWN");

        RCLCPP_INFO(this->get_logger(), "Go2Adapter shutdown completed");
        return true;

    } catch (const std::exception& e) {
        // 捕获关闭过程中可能出现的异常
        logError(std::string("Exception during shutdown: ") + e.what());
        updateStatus("SHUTDOWN_FAILED");
        return false;
    }
}

/**
 * @brief 检查适配器是否处于可操作状态
 * @return 如果适配器已初始化且正在运行，返回true；否则返回false。
 */
bool Go2Adapter::isOperational() const {
    // 使用互斥锁保护is_operational_的读取
    std::lock_guard<std::mutex> lock(status_mutex_);
    return is_operational_;
}

// ============= 配置和参数 (Configuration & Parameters) =============

/**
 * @brief 从ROS参数服务器加载配置
 * @return 如果加载成功，返回true；否则返回false。
 *
 * @details
 * 声明并获取所有在YAML配置文件中定义的参数。
 * 如果`verbose_logging`为true，则会打印加载到的详细配置信息。
 */
bool Go2Adapter::loadConfigurationImpl() {
    RCLCPP_INFO(this->get_logger(), "Loading configuration parameters...");

    try {
        // 声明并设置各模块启用状态的默认值
        this->declare_parameter<bool>("enable_motion_control",   config_.enable_motion_control);
        this->declare_parameter<bool>("enable_sensor_interface", config_.enable_sensor_interface);
        this->declare_parameter<bool>("enable_state_monitor",    config_.enable_state_monitor);
        this->declare_parameter<bool>("enable_power_manager",    config_.enable_power_manager);

        // 声明并设置核心参数的默认值
        this->declare_parameter<double>("heartbeat_frequency",   config_.heartbeat_frequency);
        this->declare_parameter<double>("timeout_duration",      config_.timeout_duration);

        // 声明并设置Go2机器人连接信息的默认值
        this->declare_parameter<std::string>("go2_ip_address",   config_.go2_ip_address);
        this->declare_parameter<int>("go2_port",                 config_.go2_port);

        // 声明并设置调试相关的参数默认值
        this->declare_parameter<bool>("debug_mode",              config_.debug_mode);
        this->declare_parameter<bool>("verbose_logging",         config_.verbose_logging);

        // 从参数服务器获取实际的参数值，覆盖默认值
        config_.enable_motion_control   = this->get_parameter("enable_motion_control").as_bool();
        config_.enable_sensor_interface = this->get_parameter("enable_sensor_interface").as_bool();
        config_.enable_state_monitor    = this->get_parameter("enable_state_monitor").as_bool();
        config_.enable_power_manager    = this->get_parameter("enable_power_manager").as_bool();

        config_.heartbeat_frequency     = this->get_parameter("heartbeat_frequency").as_double();
        config_.timeout_duration        = this->get_parameter("timeout_duration").as_double();

        config_.go2_ip_address          = this->get_parameter("go2_ip_address").as_string();
        config_.go2_port                = this->get_parameter("go2_port").as_int();

        config_.debug_mode              = this->get_parameter("debug_mode").as_bool();
        config_.verbose_logging         = this->get_parameter("verbose_logging").as_bool();

        RCLCPP_INFO(this->get_logger(), "Configuration loaded successfully");

        // 如果启用了详细日志，则打印所有加载的配置项
        if (config_.verbose_logging) {
            RCLCPP_INFO(this->get_logger(), "Configuration details:");
            RCLCPP_INFO(this->get_logger(), "  Motion Control: %s",   config_.enable_motion_control   ? "enabled" : "disabled");
            RCLCPP_INFO(this->get_logger(), "  Sensor Interface: %s", config_.enable_sensor_interface ? "enabled" : "disabled");
            RCLCPP_INFO(this->get_logger(), "  State Monitor: %s",    config_.enable_state_monitor    ? "enabled" : "disabled");
            RCLCPP_INFO(this->get_logger(), "  Power Manager: %s",    config_.enable_power_manager    ? "enabled" : "disabled");
            RCLCPP_INFO(this->get_logger(), "  Go2 Address: %s:%d",   config_.go2_ip_address.c_str(), config_.go2_port);
        }

        return true;

    } catch (const std::exception& e) {
        // 捕获参数加载过程中可能出现的异常
        RCLCPP_ERROR(this->get_logger(), "Failed to load configuration: %s", e.what());
        return false;
    }
}

/**
 * @brief 保存当前配置（尚未实现）
 * @return 总是返回true。
 * TODO: 类似于loadConfiguration函数，将其保存到`configs`里某个合适的文件夹下，格式就是yaml格式。
 */
bool Go2Adapter::saveConfigurationImpl() {
    RCLCPP_INFO(this->get_logger(), "Saving configuration not implemented yet");
    return true;
}

/**
 * @brief 重新加载配置
 * @return 如果重新加载成功，返回true；否则返回false。
 * @details
 * 该函数简单地再次调用`loadConfiguration`来覆盖当前配置。
 */
bool Go2Adapter::reloadConfiguration() {
    RCLCPP_INFO(this->get_logger(), "Reloading configuration...");
    return loadConfiguration();
}

/**
 * @brief 获取JSON格式的当前配置字符串
 * @return 包含所有配置项的JSON字符串。
 */
std::string Go2Adapter::getConfiguration() const {
    // 使用互斥锁保护配置对象的读取
    std::lock_guard<std::mutex> lock(config_mutex_);

    std::ostringstream oss;
    oss << "{"
        << "\"enable_motion_control\": "    << (config_.enable_motion_control   ? "true" : "false") << ","
        << "\"enable_sensor_interface\": "  << (config_.enable_sensor_interface ? "true" : "false") << ","
        << "\"enable_state_monitor\": "     << (config_.enable_state_monitor    ? "true" : "false") << ","
        << "\"enable_power_manager\": "     << (config_.enable_power_manager    ? "true" : "false") << ","
        << "\"heartbeat_frequency\": "      << config_.heartbeat_frequency                          << ","
        << "\"timeout_duration\": "         << config_.timeout_duration                             << ","
        << "\"go2_ip_address\": \""         << config_.go2_ip_address                               << "\","
        << "\"go2_port\": "                 << config_.go2_port                                     << ","
        << "\"debug_mode\": "               << (config_.debug_mode              ? "true" : "false") << ","
        << "\"verbose_logging\": "          << (config_.verbose_logging         ? "true" : "false")
        << "}";

    return oss.str();
}

// ============= 诊断和监控 (Diagnostics & Monitoring) =============

/**
 * @brief 获取JSON格式的适配器状态字符串
 * @return 包含核心状态信息的JSON字符串。
 */
std::string Go2Adapter::getAdapterStatus() const {
    // 使用互斥锁保护状态变量的读取
    std::lock_guard<std::mutex> lock(status_mutex_);

    std::ostringstream oss;
    oss << "{"
        << "\"status\": \""     << current_status_  << "\","
        << "\"initialized\": "  << (is_initialized_ ? "true" : "false") << ","
        << "\"operational\": "  << (is_operational_ ? "true" : "false") << ","
        << "\"version\": \""    << getVersion()     << "\","
        << "\"robot_type\": \"GO2\","
        << "\"node_name\": \""  << this->get_name() << "\","
        << "\"error_count\": "  << error_messages_.size()
        << "}";

    return oss.str();
}

/**
 * @brief 获取JSON格式的性能统计字符串
 * @return 包含运行时长、心跳计数、内存使用等信息的JSON字符串。
 */
std::string Go2Adapter::getPerformanceStats() const {
    auto now = std::chrono::steady_clock::now();
    auto uptime_duration = std::chrono::duration_cast<std::chrono::seconds>(now - start_time_);

    std::ostringstream oss;
    oss << "{"
        << "\"uptime_seconds\": "   << uptime_duration.count() << ","
        << "\"heartbeat_count\": "  << heartbeat_count_.load() << ","
        << "\"error_count\": "      << error_messages_.size() << ","
        << "\"memory_usage_mb\": "  << getMemoryUsageMB()
        << "}";

    return oss.str();
}

/**
 * @brief 执行系统自检
 * @return 一个map，键是模块名，值是该模块是否健康 (true/false)。
 *
 * @details
 * 检查适配器自身状态、配置有效性、与Go2的连接以及所有已启用子系统的状态。
 */
std::map<std::string, bool> Go2Adapter::performSystemCheck() {
    std::map<std::string, bool> results;

    // 检查适配器核心状态
    results["adapter_initialized"] = is_initialized_;
    results["adapter_operational"] = is_operational_;
    results["configuration_valid"] = validateConfiguration();
    results["go2_connection"]      = isGo2Available();

    // 检查各子系统的运行状态
    if (motion_controller_) {
        results["motion_controller"] = motion_controller_->isOperational();
    } else {
        results["motion_controller"] = false;
    }

    if (sensor_interface_) {
        // 检查传感器接口是否至少有一个可用的传感器（以3D激光雷达为例）
        results["sensor_interface"] = sensor_interface_->isSensorAvailable(robot_base_interfaces::sensor_interface::SensorType::LIDAR_3D);
    } else {
        results["sensor_interface"] = false;
    }

    if (state_monitor_) {
        results["state_monitor"] = state_monitor_->isMonitoring();
    } else {
        results["state_monitor"] = false;
    }

    if (power_manager_) {
        results["power_manager"] = power_manager_->isOperational();
    } else {
        results["power_manager"] = false; // 当前电源管理器未启用
    }

    RCLCPP_INFO(this->get_logger(), "System check completed");
    return results;
}

/**
 * @brief 获取当前记录的所有错误信息
 * @return 包含错误消息字符串的vector。
 */
std::vector<std::string> Go2Adapter::getErrorMessages() const {
    std::lock_guard<std::mutex> lock(error_mutex_);
    return error_messages_;
}

/**
 * @brief 清除所有已记录的错误信息
 */
bool Go2Adapter::clearErrors() {
    std::lock_guard<std::mutex> lock(error_mutex_);
    error_messages_.clear();
    last_error_.clear();
    last_error_code_ = 0;
    RCLCPP_INFO(this->get_logger(), "Error messages cleared");
    return true;
}

// ============= 事件和回调 (Events & Callbacks) =============

/**
 * @brief 设置状态变化时的回调函数
 * @param callback 当状态发生变化时要调用的函数。
 */
void Go2Adapter::setStatusChangeCallback(std::function<void(const std::string&)> callback) {
    status_callback_ = callback;
}


// ============= 静态方法 (Static Methods) =============

/**
 * @brief 检查Go2机器人是否在线
 * @return 如果机器人可访问，返回true；否则返回false。
 * @details
 * 通过执行`ping`命令来检查与Go2机器的网络连通性。这是一个简单的检查方法。
 * TODO: 增加传入参数，用go2_adapter.hpp中AdapterConfig配置里的go2_ip_address来定义go2_ip。
 */
bool Go2Adapter::isGo2Available() {
    // 使用ping命令检查与Go2默认IP地址的网络连接
    std::string go2_ip = "192.168.123.18"; // Go2的默认IP地址
    // -c 1: 发送1个包; -W 2: 等待2秒超时
    std::string ping_cmd = "ping -c 1 -W 2 " + go2_ip + " > /dev/null 2>&1";

    // 执行shell命令
    int result = system(ping_cmd.c_str());

    // system返回0表示命令成功执行
    if (result == 0) {
        return true; // ping成功，认为机器人可用
    }

    // 在实际应用中，即使ping失败，如果网络配置正确（例如在同一DDS域内），
    // ROS2通信也可能成功。此处的检查是一个简化实现。
    return false;
}

// ============= Protected方法实现 (Protected Methods Implementation) =============

/**
 * @brief 初始化所有子系统
 * @return 如果所有启用的子系统都初始化成功，返回true；否则返回false。
 * @details
 * 根据配置文件中的`enable_*`标志，创建并初始化通信管理器、消息转换器、
 * 运动控制器、传感器接口、状态监视器和电源管理器。
 */
bool Go2Adapter::initializeSubsystems() {
    RCLCPP_INFO(this->get_logger(), "Initializing subsystems...");

    using robot_adapters::go2_adapter::ConversionOptions;

    try {
        // 1. 初始化通信管理器
        communication_manager_ = std::make_shared<Go2Communication>(shared_from_this());
        if (!communication_manager_->initialize()) {
            logError("Failed to initialize communication manager");
            return false;
        }

        // 2. 初始化消息转换器
        // TODO: 未来可以从ROS参数加载转换器选项
        ConversionOptions options;
        message_converter_ = std::make_shared<Go2MessageConverter>(options);

        // 根据配置选择性地初始化各个功能模块
        // 3. 初始化运动控制器
        if (config_.enable_motion_control) {
            motion_controller_ = std::make_shared<Go2MotionController>("go2_motion_controller");
            if (motion_controller_->initialize() != robot_base_interfaces::motion_interface::MotionResult::SUCCESS) {
                logError("Failed to initialize motion controller");
                return false;
            }
        }

        // 4. 初始化传感器接口
        if (config_.enable_sensor_interface) {
            sensor_interface_ = std::make_shared<Go2SensorInterface>("go2_sensor_interface");
            if (!sensor_interface_->initialize()) {
                logError("Failed to initialize sensor interface");
                return false;
            }
        }

        // 5. 初始化状态监控器
        if (config_.enable_state_monitor) {
            state_monitor_ = std::make_shared<Go2StateMonitor>("go2_state_monitor");
            if (!state_monitor_->initialize()) {
                logError("Failed to initialize state monitor");
                return false;
            }
        }

        // 6. 初始化电源管理器
        if (config_.enable_power_manager) {
            // TODO: 电源管理器功能当前被禁用，因为它的一些虚函数尚未完全实现
            RCLCPP_WARN(this->get_logger(), "Power manager initialization disabled (not implemented)");
            // power_manager_ = std::make_shared<Go2PowerManager>(shared_from_this());
            // if (!power_manager_->initialize()) {
            //     logError("Failed to initialize power manager");
            //     return false;
            // }
        }

        RCLCPP_INFO(this->get_logger(), "All subsystems initialized successfully");
        return true;

    } catch (const std::exception& e) {
        logError(std::string("Exception while initializing subsystems: ") + e.what());
        return false;
    }
}

/**
 * @brief 关闭所有子系统
 * @details
 * 按照与初始化相反的顺序，安全地调用每个子系统的`shutdown`方法，并释放其智能指针。
 */
void Go2Adapter::shutdownSubsystems() {
    RCLCPP_INFO(this->get_logger(), "Shutting down subsystems...");

    // 按初始化的逆序关闭和销毁子系统，以减少依赖问题
    if (power_manager_) {
        power_manager_->shutdown();
        power_manager_.reset();
    }

    if (state_monitor_) {
        state_monitor_->shutdown();
        state_monitor_.reset();
    }

    if (sensor_interface_) {
        sensor_interface_->shutdown();
        sensor_interface_.reset();
    }

    if (motion_controller_) {
        motion_controller_->shutdown();
        motion_controller_.reset();
    }

    if (message_converter_) {
        message_converter_.reset();
    }

    if (communication_manager_) {
        communication_manager_->shutdown();
        communication_manager_.reset();
    }

    RCLCPP_INFO(this->get_logger(), "All subsystems shut down");
}

/**
 * @brief 设置ROS参数
 * @details
 * 这是一个占位函数。实际的参数声明和加载逻辑在`loadConfiguration`中完成。
 */
void Go2Adapter::setupRosParameters() {
    RCLCPP_DEBUG(this->get_logger(), "Setting up ROS parameters...");
    // 具体的参数操作已移至 loadConfiguration()
}

/**
 * @brief 设置ROS接口（服务、话题等）
 * @details
 * 负责创建适配器向外提供的ROS服务。当前代码中，服务创建部分被注释掉了，
 * 以避免因依赖问题导致的编译错误。
 */
void Go2Adapter::setupRosInterfaces() {
    RCLCPP_INFO(this->get_logger(), "Setting up ROS interfaces...");
    
    // 创建初始化服务
    initialize_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/initialize",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            this->initializeServiceCallback(request, response);
        }
    );

    // 创建关闭服务
    shutdown_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/shutdown",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            this->shutdownServiceCallback(request, response);
        }
    );

    // 创建系统自检服务
    system_check_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/system_check",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            this->systemCheckServiceCallback(request, response);
        }
    );

    RCLCPP_INFO(this->get_logger(), "ROS interfaces setup completed");
}

/**
 * @brief 验证加载的配置是否有效
 * @return 如果所有关键配置项都在合理范围内，返回true；否则返回false。
 */
bool Go2Adapter::validateConfigurationImpl() const {
    // 验证心跳频率是否在有效范围内
    if (config_.heartbeat_frequency <= 0.0 || config_.heartbeat_frequency > 100.0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid heartbeat frequency: %f", config_.heartbeat_frequency);
        return false;
    }

    // 验证超时时长是否在有效范围内
    if (config_.timeout_duration <= 0.0 || config_.timeout_duration > 60.0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid timeout duration: %f", config_.timeout_duration);
        return false;
    }

    // 验证IP地址是否为空
    if (config_.go2_ip_address.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Go2 IP address is empty");
        return false;
    }

    // 验证端口号是否在有效范围内
    if (config_.go2_port <= 0 || config_.go2_port > 65535) {
        RCLCPP_ERROR(this->get_logger(), "Invalid Go2 port: %d", config_.go2_port);
        return false;
    }

    return true;
}

/**
 * @brief 记录一条错误信息
 * @param error_msg 要记录的错误消息。
 * @details
 * 将错误消息存入一个有大小限制的vector中，并通过RCLCPP_ERROR打印。
 * 如果设置了错误回调函数，则会触发它。
 */
void Go2Adapter::logError(const std::string& error_msg) {
    {
        std::lock_guard<std::mutex> lock(error_mutex_);
        error_messages_.push_back(error_msg);

        // 限制错误消息队列的大小，防止无限增长消耗内存
        if (error_messages_.size() > 100) {
            error_messages_.erase(error_messages_.begin());
        }
    }

    RCLCPP_ERROR(this->get_logger(), "%s", error_msg.c_str());

    // 如果外部注册了错误回调函数，则调用它
    if (error_callback_) {
        error_callback_(error_msg, -1);  // 默认错误代码为-1
    }
}

/**
 * @brief 更新适配器的当前状态
 * @param status 新的状态字符串。
 * @details
 * 更新内部状态变量，并打印状态变化日志。
 * 如果设置了状态变化回调函数，则会触发它。
 */
void Go2Adapter::updateStatus(const std::string& status) {
    std::string old_status = current_status_;
    current_status_ = status;

    // 仅当状态发生变化且启用了详细日志时才打印
    if (config_.verbose_logging && old_status != status) {
        RCLCPP_INFO(this->get_logger(), "Status changed: %s -> %s", old_status.c_str(), status.c_str());
    }

    // 如果外部注册了状态变化回调函数，则调用它
    if (status_callback_) {
        status_callback_(status);
    }
}

/**
 * @brief 获取当前进程的内存使用量（MB）
 * @return 内存使用量（MB），如果读取失败则返回一个估算值。
 * @details
 * 此函数通过读取Linux的`/proc/self/status`文件中的`VmRSS`字段来获取
 * 进程的物理内存使用量。这是一种特定于Linux的实现。
 */
double Go2Adapter::getMemoryUsageMB() const {
    // 尝试从/proc文件系统读取内存信息
    try {
        std::ifstream status_file("/proc/self/status");
        std::string line;

        while (std::getline(status_file, line)) {
            // 查找以"VmRSS:"开头的行
            if (line.substr(0, 6) == "VmRSS:") {
                std::istringstream iss(line);
                std::string name, kb_str;
                int kb_value;
                iss >> name >> kb_value >> kb_str;
                return kb_value / 1024.0; // 将KB转换为MB
            }
        }
    } catch (const std::exception&) {
        // 如果文件读取或解析失败，则忽略异常，并返回一个默认值
    }

    // 如果无法读取，返回一个固定的估算值
    return 50.0; // 默认50MB
}

// ============= 私有方法实现 (Private Methods Implementation) =============

/**
 * @brief 心跳定时器的回调函数
 * @details
 * 该函数被`heartbeat_timer_`周期性调用。主要功能包括：
 * - 增加心跳计数器。
 * - 在调试模式下打印心跳日志。
 * - 定期（每50次心跳）执行一次完整的系统健康检查。
 */
void Go2Adapter::heartbeatCallback() {
    // 如果适配器未处于可操作状态，则不执行任何操作
    if (!is_operational_) {
        return;
    }

    // 心跳计数器自增
    heartbeat_count_++;

    // 在调试模式下，打印每次心跳的日志
    if (config_.debug_mode) {
        RCLCPP_DEBUG(this->get_logger(), "Heartbeat tick #%lu", heartbeat_count_.load());
    }

    // 每隔50次心跳（根据心跳频率而定），执行一次系统自检
    if (heartbeat_count_ % 50 == 0) {
        auto system_status = performSystemCheck();
        bool all_healthy = true;
        // 遍历自检结果
        for (const auto& [module, status] : system_status) {
            if (!status) {
                all_healthy = false;
                // 如果某个模块检查失败，记录错误日志
                logError("System check failed for module: " + module);
            }
        }

        // 如果所有模块都健康且启用了详细日志，则打印通过信息
        if (all_healthy && config_.verbose_logging) {
            RCLCPP_DEBUG(this->get_logger(), "System health check passed");
        }
    }
}

void Go2Adapter::initializeServiceCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    (void)request;  // 避免未使用参数的警告

    bool success = initialize();
    response->success = success;
    response->message = success ? "Initialization successful" : "Initialization failed";
}

void Go2Adapter::shutdownServiceCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    (void)request;  // 避免未使用参数的警告

    shutdown();
    response->success = true;
    response->message = "Shutdown completed";
}

void Go2Adapter::systemCheckServiceCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    (void)request;  // 避免未使用参数的警告

    auto results = performSystemCheck();
    bool all_passed = true;

    std::ostringstream oss;
    oss << "System check results: ";
    for (const auto& [module, status] : results) {
        oss << module << "=" << (status ? "OK" : "FAIL") << " ";
        if (!status) {
            all_passed = false;
        }
    }

    response->success = all_passed;
    response->message = oss.str();
}

// ============= IRobotAdapter 接口实现 (IRobotAdapter Interface Implementation) =============

std::string Go2Adapter::getFirmwareVersion() const {
    // TODO: 从机器人获取真实的固件版本信息
    return "v1.0.0";
}

std::string Go2Adapter::getSerialNumber() const {
    // TODO: 从机器人获取真实的序列号
    return "GO2-SN-000000";
}

robot_base_interfaces::motion_interface::MotionCapabilities Go2Adapter::getMotionCapabilities() const {
    robot_base_interfaces::motion_interface::MotionCapabilities capabilities;
    capabilities.max_linear_velocity = 1.5f;  // m/s
    capabilities.max_angular_velocity = 2.0f; // rad/s
    capabilities.max_lateral_velocity = 0.8f; // m/s
    capabilities.max_roll_angle = 0.4f;       // rad
    capabilities.max_pitch_angle = 0.4f;      // rad
    capabilities.min_body_height = 0.20f;     // m
    capabilities.max_body_height = 0.42f;     // m
    capabilities.can_climb_stairs = true;
    capabilities.can_balance = true;
    capabilities.can_lateral_move = true;
    capabilities.can_dance = true;
    capabilities.can_jump = true;
    capabilities.can_flip = true;
    return capabilities;
}

std::vector<robot_base_interfaces::sensor_interface::SensorInfo> Go2Adapter::getAvailableSensors() const {
    std::vector<robot_base_interfaces::sensor_interface::SensorInfo> sensors;
    
    robot_base_interfaces::sensor_interface::SensorInfo lidar_info;
    lidar_info.name = "livox_mid360";
    lidar_info.type = robot_base_interfaces::sensor_interface::SensorType::LIDAR_3D;
    lidar_info.frame_id = "utlidar_lidar";
    lidar_info.topic_name = "/utlidar/cloud";
    lidar_info.frequency = 10.0f;  // Hz
    lidar_info.status = robot_base_interfaces::sensor_interface::SensorStatus::ACTIVE;
    sensors.push_back(lidar_info);
    
    robot_base_interfaces::sensor_interface::SensorInfo imu_info;
    imu_info.name = "go2_imu";
    imu_info.type = robot_base_interfaces::sensor_interface::SensorType::IMU;
    imu_info.frame_id = "imu_link";
    imu_info.topic_name = "/imu";
    imu_info.frequency = 100.0f;  // Hz
    imu_info.status = robot_base_interfaces::sensor_interface::SensorStatus::ACTIVE;
    sensors.push_back(imu_info);
    
    return sensors;
}

std::vector<robot_base_interfaces::power_interface::ChargingType> Go2Adapter::getSupportedChargingTypes() const {
    return {robot_base_interfaces::power_interface::ChargingType::WIRELESS};
}

bool Go2Adapter::hasCapability(const std::string& capability_name) const {
    static const std::set<std::string> supported_capabilities = {
        "motion_control",
        "lidar_sensing",
        "imu_sensing", 
        "wireless_charging",
        "autonomous_navigation",
        "slam"
    };
    return supported_capabilities.find(capability_name) != supported_capabilities.end();
}

bool Go2Adapter::loadConfiguration(const std::string& config_file_path) {
    if (config_file_path.empty()) {
        // 使用默认的ROS参数加载
        return loadConfiguration();
    } else {
        // TODO: 实现从文件加载配置
        RCLCPP_WARN(this->get_logger(), "Loading configuration from file not implemented yet: %s", 
                    config_file_path.c_str());
        return false;
    }
}

bool Go2Adapter::saveConfiguration(const std::string& config_file_path) const {
    // TODO: 实现保存配置到文件
    RCLCPP_WARN(this->get_logger(), "Saving configuration to file not implemented yet: %s", 
                config_file_path.c_str());
    return false;
}

bool Go2Adapter::getConfigParameter(const std::string& parameter_name, std::string& value) const {
    try {
        if (parameter_name == "go2_ip_address") {
            value = config_.go2_ip_address;
            return true;
        } else if (parameter_name == "go2_port") {
            value = std::to_string(config_.go2_port);
            return true;
        } else if (parameter_name == "heartbeat_frequency") {
            value = std::to_string(config_.heartbeat_frequency);
            return true;
        }
        // TODO: 添加更多参数支持
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error getting config parameter %s: %s", 
                     parameter_name.c_str(), e.what());
        return false;
    }
}

bool Go2Adapter::setConfigParameter(const std::string& parameter_name, const std::string& value) {
    try {
        if (parameter_name == "go2_ip_address") {
            config_.go2_ip_address = value;
            return true;
        } else if (parameter_name == "go2_port") {
            config_.go2_port = std::stoi(value);
            return true;
        } else if (parameter_name == "heartbeat_frequency") {
            config_.heartbeat_frequency = std::stod(value);
            return true;
        }
        // TODO: 添加更多参数支持
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error setting config parameter %s: %s", 
                     parameter_name.c_str(), e.what());
        return false;
    }
}

std::string Go2Adapter::getRobotNetworkAddress() const {
    return config_.go2_ip_address;
}

int Go2Adapter::getCommunicationPort() const {
    return config_.go2_port;
}

bool Go2Adapter::isConnected() const {
    // TODO: 实现真正的连接状态检查
    return is_operational_;
}

bool Go2Adapter::connect() {
    // TODO: 实现连接逻辑
    RCLCPP_INFO(this->get_logger(), "Connecting to Go2 robot...");
    return true;
}

bool Go2Adapter::disconnect() {
    // TODO: 实现断开连接逻辑
    RCLCPP_INFO(this->get_logger(), "Disconnecting from Go2 robot...");
    return true;
}

std::string Go2Adapter::getDiagnosticInfo() const {
    std::ostringstream oss;
    oss << "{";
    oss << "\"initialized\": "     << (is_initialized_ ? "true" : "false") << ",";
    oss << "\"operational\": "     << (is_operational_ ? "true" : "false") << ",";
    oss << "\"status\": \""        << current_status_                      << "\",";
    oss << "\"error_count\": "     << error_messages_.size()               << ",";
    oss << "\"last_error\": \""    << last_error_                          << "\",";
    oss << "\"heartbeat_count\": " << heartbeat_count_.load();
    oss << "}";
    return oss.str();
}

std::string Go2Adapter::getLastError() const {
    std::lock_guard<std::mutex> lock(error_mutex_);
    return last_error_;
}

void Go2Adapter::setAdapterStatusCallback(
    std::function<void(bool is_operational, const std::string& status_msg)> callback) {
    // 将IRobotAdapter的回调转换为内部回调
    setStatusChangeCallback([callback, this](const std::string& status) {
        callback(this->is_operational_, status);
    });
}

void Go2Adapter::setConnectionStatusCallback(
    std::function<void(bool is_connected, const std::string& connection_info)> callback) {
    (void)callback;  // 避免未使用参数的警告
    // TODO: 实现连接状态回调
    RCLCPP_WARN(this->get_logger(), "Connection status callback not implemented yet");
}

void Go2Adapter::setErrorCallback(
    std::function<void(const std::string& error_msg, int error_code)> callback) {
    error_callback_ = [callback](const std::string& error_msg, int error_code) {
        callback(error_msg, error_code);
    };
}

bool Go2Adapter::validateConfiguration(const std::map<std::string, std::string>& config) const {
    (void)config;  // 避免未使用参数的警告
    // TODO: 实现配置验证逻辑
    return true;
}

void Go2Adapter::setLastError(const std::string& error_message, int error_code) {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_ = error_message;
    last_error_code_ = error_code;
    
    // 同时调用错误回调
    if (error_callback_) {
        error_callback_(error_message, error_code);
    }
}


} // namespace go2_adapter
} // namespace robot_adapters
