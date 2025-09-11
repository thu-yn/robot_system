/**
 * @file go2_communication.cpp
 * @brief Go2机器人通信管理器实现文件
 * @author Yang Nan
 * @date 2025-09-10
 *
 * @details
 * 本文件包含了 `Go2Communication` 类的所有方法的具体实现。
 * 它负责执行与Go2机器人通信相关的所有底层操作，包括初始化ROS2的
 * 发布者和订阅者、管理消息缓冲区、监控连接状态、处理重连逻辑以及
 * 收集和报告通信质量统计数据。
 */

#include "robot_adapters/go2_adapter/go2_communication.hpp" // 引入对应的头文件
#include <std_msgs/msg/string.hpp> // 引入标准字符串消息，用于原始数据发送的示例
#include <chrono> // C++时间库
#include <sstream> // C++字符串流库，用于构建字符串
#include <iomanip> // C++输入/输出操纵器，用于格式化输出（如十六进制）

namespace robot_adapters {
namespace go2_adapter {

/**
 * @brief Go2Communication类的构造函数
 * @param node 一个指向外部ROS节点的共享指针。
 *
 * @details
 * 初始化所有成员变量的默认值，包括状态标志、配置项、消息缓冲区大小等。
 * 构造函数本身不执行任何需要ROS2上下文的操作（如创建发布者），这些操作
 * 被推迟到 `initialize` 方法中。
 */
Go2Communication::Go2Communication(std::shared_ptr<rclcpp::Node> node)
    : node_(node),
      is_initialized_(false),
      status_(CommunicationStatus::DISCONNECTED),
      auto_reconnect_enabled_(true),
      verbose_logging_(false),
      should_reconnect_(false),
      reconnect_attempts_(0) {

    // 为不同类型的消息设置默认的缓冲区大小
    buffer_sizes_[MessageType::SPORT_MODE_STATE] = 10;
    buffer_sizes_[MessageType::LOW_STATE] = 10;
    buffer_sizes_[MessageType::BMS_STATE] = 10;
    buffer_sizes_[MessageType::POINT_CLOUD] = 5; // 点云数据量大，缓冲区不宜过大
    buffer_sizes_[MessageType::IMU_DATA] = 20;
    buffer_sizes_[MessageType::ODOMETRY] = 10;

    // 初始化统计数据的时间戳
    stats_.connection_time = std::chrono::steady_clock::now();
    stats_.last_message_time = std::chrono::steady_clock::now();

    logInfo("Go2Communication object created.");
}

/**
 * @brief Go2Communication类的析构函数
 * @details
 * 确保在对象销毁时调用 `shutdown` 方法，以安全地释放所有资源，
 * 如停止线程、取消定时器、销毁ROS实体等。
 */
Go2Communication::~Go2Communication() {
    shutdown();
}

/**
 * @brief 初始化通信管理器
 * @return 如果所有步骤都成功，则返回true；否则返回false。
 *
 * @details
 * 这是设置通信模块的核心函数，它执行以下操作：
 * 1. 配置网络参数（当前为占位符）。
 * 2. 创建所有必需的ROS2发布者和订阅者。
 * 3. 创建用于监控连接和更新统计数据的定时器。
 * 4. 如果启用了自动重连，则启动一个后台线程来处理重连逻辑。
 * 5. 将状态更新为已初始化和已连接。
 */
bool Go2Communication::initialize() {
    logInfo("Initializing Go2Communication module...");

    try {
        // 步骤1: 配置网络参数
        if (!configureNetwork()) {
            recordError("Network configuration failed.");
            return false;
        }

        // 步骤2: 创建ROS2发布者和订阅者
        if (!createPublishersAndSubscribers()) {
            recordError("Failed to create publishers and subscribers.");
            return false;
        }

        // 步骤3: 创建监控定时器，每秒检查一次连接状态
        monitor_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&Go2Communication::monitorTimerCallback, this));

        // 步骤4: 创建统计信息定时器，每5秒更新一次频率等数据
        statistics_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(5000),
            std::bind(&Go2Communication::statisticsTimerCallback, this));

        // 步骤5: 如果启用了自动重连，则启动重连线程
        if (auto_reconnect_enabled_.load()) {
            should_reconnect_ = true; // 允许线程运行
            reconnect_thread_ = std::thread(&Go2Communication::reconnectThreadFunction, this);
        }

        is_initialized_.store(true);
        updateConnectionStatus(CommunicationStatus::CONNECTED); // 假设初始化后即连接

        logInfo("Go2Communication module initialized successfully.");
        return true;

    } catch (const std::exception& e) {
        recordError("Exception during initialization: " + std::string(e.what()));
        return false;
    }
}

/**
 * @brief 关闭通信管理器
 * @return 如果所有步骤都成功，则返回true；否则返回false。
 *
 * @details
 * 安全地释放所有资源，执行与 `initialize` 相反的操作：
 * 1. 停止重连线程。
 * 2. 取消并重置所有定时器。
 * 3. 销毁所有ROS2发布者和订阅者。
 * 4. 清空所有消息缓冲区。
 * 5. 更新状态为未初始化和已断开。
 */
bool Go2Communication::shutdown() {
    logInfo("Shutting down Go2Communication module...");

    try {
        // 步骤1: 停止自动重连线程
        should_reconnect_.store(false);
        if (reconnect_thread_.joinable()) {
            reconnect_cv_.notify_all(); // 唤醒线程以便其能检查到退出条件
            reconnect_thread_.join();   // 等待线程结束
        }

        // 步骤2: 停止所有定时器
        if (monitor_timer_) {
            monitor_timer_->cancel();
            monitor_timer_.reset();
        }
        if (statistics_timer_) {
            statistics_timer_->cancel();
            statistics_timer_.reset();
        }
        if (reconnect_timer_) { // 这个定时器可能在某些逻辑中使用
            reconnect_timer_->cancel();
            reconnect_timer_.reset();
        }

        // 步骤3: 销毁ROS2实体
        destroyPublishersAndSubscribers();

        // 步骤4: 清空所有消息缓冲区
        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            while (!pointcloud_buffer_.empty()) pointcloud_buffer_.pop();
            while (!imu_buffer_.empty()) imu_buffer_.pop();
            while (!odom_buffer_.empty()) odom_buffer_.pop();
        }

        is_initialized_.store(false);
        updateConnectionStatus(CommunicationStatus::DISCONNECTED);

        logInfo("Go2Communication module shut down successfully.");
        return true;

    } catch (const std::exception& e) {
        recordError("Exception during shutdown: " + std::string(e.what()));
        return false;
    }
}

/**
 * @brief 启动通信（占位符）
 * @return 如果模块已初始化，则返回true。
 */
bool Go2Communication::startCommunication() {
    if (!is_initialized_.load()) {
        recordError("Communication module is not initialized. Cannot start communication.");
        return false;
    }
    logInfo("Starting Go2 robot communication.");
    updateConnectionStatus(CommunicationStatus::CONNECTING);

    if (!verifyNetworkConnection()) {
        recordError("Network connection verification failed.");
        updateConnectionStatus(CommunicationStatus::ERROR);
        return false;
    }

    updateConnectionStatus(CommunicationStatus::CONNECTED);
    logInfo("Go2 robot communication started.");
    return true;
}

/**
 * @brief 停止通信（占位符）
 * @return 总是返回true。
 */
bool Go2Communication::stopCommunication() {
    logInfo("Stopping Go2 robot communication.");
    updateConnectionStatus(CommunicationStatus::DISCONNECTED);
    should_reconnect_.store(false); // 停止自动重连
    logInfo("Go2 robot communication stopped.");
    return true;
}

// ============= 连接管理实现 =============

/**
 * @brief 连接到机器人（占位符）
 * @details 实际连接由DDS层自动处理，此函数仅用于模拟连接过程和更新状态。
 */
bool Go2Communication::connectToRobot(const std::string& robot_ip, int timeout_seconds) {
    logInfo("Attempting to connect to Go2 robot at " + robot_ip);
    network_config_.robot_ip = robot_ip;
    network_config_.connection_timeout_ms = timeout_seconds * 1000;
    updateConnectionStatus(CommunicationStatus::CONNECTING);
    stats_.connection_attempts++;

    if (verifyNetworkConnection()) {
        updateConnectionStatus(CommunicationStatus::CONNECTED);
        logInfo("Successfully connected to Go2 robot.");
        return true;
    } else {
        updateConnectionStatus(CommunicationStatus::ERROR);
        recordError("Failed to connect to Go2 robot.");
        return false;
    }
}

/**
 * @brief 断开与机器人的连接（占位符）
 */
bool Go2Communication::disconnectFromRobot() {
    logInfo("Disconnecting from Go2 robot.");
    updateConnectionStatus(CommunicationStatus::DISCONNECTED);
    should_reconnect_.store(false);
    return true;
}

/**
 * @brief 获取当前连接状态
 */
CommunicationStatus Go2Communication::getConnectionStatus() const {
    return status_.load();
}

/**
 * @brief 检查是否已连接
 */
bool Go2Communication::isConnected() const {
    return status_.load() == CommunicationStatus::CONNECTED;
}

/**
 * @brief 检查是否正在通信（已初始化且已连接）
 */
bool Go2Communication::isCommunicating() const {
    return isConnected() && is_initialized_.load();
}

/**
 * @brief 设置自动重连参数
 */
void Go2Communication::setAutoReconnect(bool enable, int retry_interval_ms, int max_retries) {
    auto_reconnect_enabled_.store(enable);
    network_config_.reconnect_interval_ms = retry_interval_ms;
    network_config_.max_reconnect_attempts = max_retries;
    logInfo("Auto-reconnect set to: " + std::string(enable ? "Enabled" : "Disabled") +
            ", Interval: " + std::to_string(retry_interval_ms) + "ms" +
            ", Max Retries: " + std::to_string(max_retries));
}

// ============= 消息发布接口实现 =============

/**
 * @brief 发送速度指令
 */
bool Go2Communication::sendVelocityCommand(const geometry_msgs::msg::Twist& twist) {
    if (!isConnected() || !cmd_vel_pub_) {
        recordError("Failed to send velocity command: Not connected or publisher not created.");
        return false;
    }
    try {
        cmd_vel_pub_->publish(twist);
        updateStatistics(MessageType::LOW_COMMAND, sizeof(twist)); // 更新统计
        logDebug("Velocity command sent successfully.");
        return true;
    } catch (const std::exception& e) {
        recordError("Exception while sending velocity command: " + std::string(e.what()));
        return false;
    }
}

/**
 * @brief 发送原始数据（示例实现）
 * @details 将任意字节数据打包成十六进制字符串，并通过一个临时的`std_msgs/String`发布者发送。
 *          这主要用于调试或特殊用途，不适用于高性能场景。
 */
bool Go2Communication::sendRawData(const std::string& topic_name, const void* data, size_t size) {
    if (!is_initialized_) {
        recordError("Communication manager not initialized.");
        return false;
    }
    if (!data || size == 0) {
        recordError("Data is null or size is zero.");
        return false;
    }
    if (topic_name.empty()) {
        recordError("Topic name is empty.");
        return false;
    }

    try {
        // 将原始字节数据转换为十六进制字符串
        std::ostringstream hex_stream;
        hex_stream << std::hex << std::uppercase << std::setfill('0');
        const auto* byte_data = static_cast<const uint8_t*>(data);
        for (size_t i = 0; i < size; ++i) {
            hex_stream << std::setw(2) << static_cast<int>(byte_data[i]);
        }

        auto raw_msg = std::make_unique<std_msgs::msg::String>();
        raw_msg->data = hex_stream.str();

        // 创建一个临时的发布者来发送一次性消息
        auto raw_pub = node_->create_publisher<std_msgs::msg::String>(topic_name, 10);
        raw_pub->publish(*raw_msg);

        // 更新发送统计
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.messages_sent++;
            stats_.total_bytes_sent += size;
            stats_.last_message_time = std::chrono::steady_clock::now();
        }

        RCLCPP_DEBUG(node_->get_logger(), "Raw data sent to topic %s, size: %zu bytes",
                     topic_name.c_str(), size);
        return true;

    } catch (const std::exception& e) {
        recordError("Exception while sending raw data: " + std::string(e.what()));
        return false;
    }
}


// ============= 消息订阅与缓冲管理实现 =============

/**
 * @brief 设置点云回调函数
 */
void Go2Communication::setPointCloudCallback(std::function<void(const sensor_msgs::msg::PointCloud2::SharedPtr)> callback) {
    pointcloud_callback_ = callback;
}

/**
 * @brief 设置IMU回调函数
 */
void Go2Communication::setImuCallback(std::function<void(const sensor_msgs::msg::Imu::SharedPtr)> callback) {
    imu_callback_ = callback;
}

/**
 * @brief 设置里程计回调函数
 */
void Go2Communication::setOdometryCallback(std::function<void(const nav_msgs::msg::Odometry::SharedPtr)> callback) {
    odom_callback_ = callback;
}

/**
 * @brief 获取最新的点云消息
 */
std::shared_ptr<sensor_msgs::msg::PointCloud2> Go2Communication::getLatestPointCloud() {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    if (!pointcloud_buffer_.empty()) {
        return pointcloud_buffer_.back(); // 返回最新的消息
    }
    return nullptr;
}

/**
 * @brief 获取最新的IMU消息
 */
std::shared_ptr<sensor_msgs::msg::Imu> Go2Communication::getLatestImu() {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    if (!imu_buffer_.empty()) {
        return imu_buffer_.back();
    }
    return nullptr;
}

/**
 * @brief 设置消息缓冲区大小
 */
void Go2Communication::setMessageBufferSize(MessageType message_type, size_t buffer_size) {
    buffer_sizes_[message_type] = buffer_size;
    logDebug("Message buffer size set for type " + std::to_string(static_cast<int>(message_type)) +
             " to " + std::to_string(buffer_size));
}

/**
 * @brief 清空指定类型的消息缓冲区
 */
void Go2Communication::clearMessageBuffer(MessageType message_type) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    switch (message_type) {
        case MessageType::POINT_CLOUD:
            std::queue<std::shared_ptr<sensor_msgs::msg::PointCloud2>>().swap(pointcloud_buffer_);
            break;
        case MessageType::IMU_DATA:
            std::queue<std::shared_ptr<sensor_msgs::msg::Imu>>().swap(imu_buffer_);
            break;
        case MessageType::ODOMETRY:
            std::queue<std::shared_ptr<nav_msgs::msg::Odometry>>().swap(odom_buffer_);
            break;
        default: // 如果类型不匹配，则清空所有缓冲区作为默认行为
            std::queue<std::shared_ptr<sensor_msgs::msg::PointCloud2>>().swap(pointcloud_buffer_);
            std::queue<std::shared_ptr<sensor_msgs::msg::Imu>>().swap(imu_buffer_);
            std::queue<std::shared_ptr<nav_msgs::msg::Odometry>>().swap(odom_buffer_);
            break;
    }
}


// ============= 通信质量监控实现 =============

/**
 * @brief 获取通信统计数据
 */
CommunicationStats Go2Communication::getStatistics() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return stats_;
}

/**
 * @brief 获取消息延迟（示例实现）
 * @details 对于带时间戳的消息（如点云和IMU），计算当前时间与消息时间戳的差值。
 *          对于其他消息，返回一个基于最后接收时间的估算值。
 */
float Go2Communication::getMessageLatency(MessageType message_type) const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    auto now = std::chrono::steady_clock::now();

    try {
        if (message_type == MessageType::POINT_CLOUD) {
            if (!pointcloud_buffer_.empty()) {
                auto latest_msg = pointcloud_buffer_.back();
                auto msg_time = rclcpp::Time(latest_msg->header.stamp);
                return (node_->get_clock()->now() - msg_time).seconds() * 1000.0f;
            }
        } else if (message_type == MessageType::IMU_DATA) {
            if (!imu_buffer_.empty()) {
                auto latest_msg = imu_buffer_.back();
                auto msg_time = rclcpp::Time(latest_msg->header.stamp);
                return (node_->get_clock()->now() - msg_time).seconds() * 1000.0f;
            }
        } else {
            // 对于没有时间戳的消息，估算延迟
            auto time_since_last = std::chrono::duration_cast<std::chrono::milliseconds>(now - stats_.last_message_time);
            return static_cast<float>(time_since_last.count());
        }
    } catch (const std::exception& e) {
        logError("Failed to calculate message latency: " + std::string(e.what()));
    }
    // 如果无法计算，返回一个默认估算值
    return stats_.average_latency_ms > 0.0f ? stats_.average_latency_ms : 20.0f;
}

/**
 * @brief 获取消息频率
 */
float Go2Communication::getMessageFrequency(MessageType message_type) const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    if (message_frequencies_.count(message_type)) {
        return message_frequencies_.at(message_type);
    }
    return 0.0f;
}

/**
 * @brief 判断通信质量是否良好
 */
bool Go2Communication::isCommuncationQualityGood() const {
    return calculateQualityScore() > 0.75f; // 定义质量分数阈值为0.75
}

/**
 * @brief 重置统计数据
 */
void Go2Communication::resetStatistics() {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_ = CommunicationStats{}; // 重置为默认值
    stats_.connection_time = std::chrono::steady_clock::now();
    message_frequencies_.clear();
    logInfo("Communication statistics have been reset.");
}


// ============= 网络配置实现 =============

bool Go2Communication::setNetworkInterface(const std::string& interface_name) {
    network_config_.network_interface = interface_name;
    logInfo("Network interface set to: " + interface_name);
    // 实际应用中，这里可能需要重新配置DDS传输
    return true;
}

bool Go2Communication::setDdsDomainId(int domain_id) {
    network_config_.dds_domain_id = domain_id;
    logInfo("DDS Domain ID set to: " + std::to_string(domain_id));
    // 实际应用中，这里需要重启ROS节点或重新创建上下文才能生效
    return true;
}

void Go2Communication::setQosSettings(rclcpp::ReliabilityPolicy reliability,
                                     rclcpp::DurabilityPolicy durability,
                                     size_t history_depth) {
    // 此函数仅记录QoS设置，实际应用在创建发布者/订阅者时
    logInfo("QoS settings updated. They will be applied to new publishers/subscribers.");
}

bool Go2Communication::applyNetworkOptimization() {
    logInfo("Applying network optimizations (placeholder).");
    return true;
}


// ============= 错误处理与诊断实现 =============

std::string Go2Communication::getLastError() const {
    std::lock_guard<std::mutex> lock(error_mutex_);
    return last_error_;
}

std::vector<std::string> Go2Communication::getErrorHistory() const {
    std::lock_guard<std::mutex> lock(error_mutex_);
    return error_history_;
}

void Go2Communication::clearErrorHistory() {
    std::lock_guard<std::mutex> lock(error_mutex_);
    error_history_.clear();
    last_error_.clear();
    logInfo("Error history cleared.");
}

/**
 * @brief 执行连接诊断
 */
std::map<std::string, bool> Go2Communication::performConnectionDiagnostics() {
    std::map<std::string, bool> diagnostics;
    diagnostics["IsInitialized"] = is_initialized_.load();
    diagnostics["IsConnected"] = isConnected();
    diagnostics["HasValidNode"] = (node_ != nullptr);
    diagnostics["CmdVelPublisherCreated"] = (cmd_vel_pub_ != nullptr);
    diagnostics["PointCloudSubscriberCreated"] = (pointcloud_sub_ != nullptr);
    diagnostics["ImuSubscriberCreated"] = (imu_sub_ != nullptr);
    return diagnostics;
}

/**
 * @brief 获取JSON格式的网络配置信息
 */
std::string Go2Communication::getNetworkDiagnostics() const {
    std::ostringstream ss;
    ss << "{"
       << "  \"robot_ip\": \"" << network_config_.robot_ip << "\",\n"
       << "  \"local_ip\": \"" << network_config_.local_ip << "\",\n"
       << "  \"network_interface\": \"" << network_config_.network_interface << "\",\n"
       << "  \"dds_domain_id\": " << network_config_.dds_domain_id << ",\n"
       << "  \"connection_status\": " << static_cast<int>(status_.load()) << "\n"
       << "}";
    return ss.str();
}


// ============= 事件回调注册实现 =============

void Go2Communication::setConnectionStatusCallback(std::function<void(CommunicationStatus)> callback) {
    status_callback_ = callback;
}

void Go2Communication::setErrorCallback(std::function<void(const std::string&)> callback) {
    error_callback_ = callback;
}

void Go2Communication::setQualityCallback(std::function<void(float)> callback) {
    quality_callback_ = callback;
}


// ============= 配置参数实现 =============

bool Go2Communication::loadConfiguration() {
    logInfo("Loading configuration (using default values).");
    return true;
}

bool Go2Communication::saveConfiguration() {
    logInfo("Saving configuration (not implemented).");
    return true;
}

std::string Go2Communication::getConfiguration() const {
    return getNetworkDiagnostics();
}

void Go2Communication::setVerboseLogging(bool enable) {
    verbose_logging_.store(enable);
    logInfo("Verbose logging " + std::string(enable ? "enabled." : "disabled."));
}


// ============= 私有方法实现 =============

/**
 * @brief 创建ROS2发布者和订阅者
 */
bool Go2Communication::createPublishersAndSubscribers() {
    try {
        // 为传感器数据使用BestEffort QoS，因为它们是高频数据，丢失一两帧影响不大
        auto sensor_qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::BestEffort);
        // 为命令数据使用Reliable QoS，确保命令能被接收
        auto cmd_qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);

        // 创建发布者
        cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", cmd_qos);

        // 创建订阅者
        pointcloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/utlidar/cloud", sensor_qos,
            std::bind(&Go2Communication::pointcloudCallback, this, std::placeholders::_1));

        imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", sensor_qos,
            std::bind(&Go2Communication::imuCallback, this, std::placeholders::_1));

        odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", sensor_qos,
            std::bind(&Go2Communication::odomCallback, this, std::placeholders::_1));

        logInfo("Publishers and subscribers created successfully.");
        return true;

    } catch (const std::exception& e) {
        recordError("Exception in createPublishersAndSubscribers: " + std::string(e.what()));
        return false;
    }
}

/**
 * @brief 销毁所有ROS2发布者和订阅者
 */
void Go2Communication::destroyPublishersAndSubscribers() {
    cmd_vel_pub_.reset();
    pointcloud_sub_.reset();
    imu_sub_.reset();
    odom_sub_.reset();
    logInfo("Publishers and subscribers destroyed.");
}


// ============= 内部回调实现 =============

/**
 * @brief 点云消息回调
 */
void Go2Communication::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    updateStatistics(MessageType::POINT_CLOUD, msg->data.size());
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        manageBuffer(pointcloud_buffer_, msg, buffer_sizes_[MessageType::POINT_CLOUD]);
    }
    if (pointcloud_callback_) {
        pointcloud_callback_(msg);
    }
    logDebug("PointCloud message received.");
}

/**
 * @brief IMU消息回调
 */
void Go2Communication::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    updateStatistics(MessageType::IMU_DATA, sizeof(*msg));
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        manageBuffer(imu_buffer_, msg, buffer_sizes_[MessageType::IMU_DATA]);
    }
    if (imu_callback_) {
        imu_callback_(msg);
    }
    logDebug("IMU message received.");
}

/**
 * @brief 里程计消息回调
 */
void Go2Communication::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    updateStatistics(MessageType::ODOMETRY, sizeof(*msg));
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        manageBuffer(odom_buffer_, msg, buffer_sizes_[MessageType::ODOMETRY]);
    }
    if (odom_callback_) {
        odom_callback_(msg);
    }
    logDebug("Odometry message received.");
}


// ============= 定时器回调实现 =============

/**
 * @brief 监控定时器回调
 * @details 定期检查距离上次收到消息的时间，如果超过阈值，则认为连接可能已断开，
 *          并触发重连逻辑。
 */
void Go2Communication::monitorTimerCallback() {
    if (isConnected()) {
        auto now = std::chrono::steady_clock::now();
        auto time_since_last_msg = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - stats_.last_message_time).count();

        // 如果超过5秒没有收到任何消息，则认为连接丢失
        if (time_since_last_msg > 5000) {
            logWarning("No message received for 5 seconds. Connection might be lost.");
            updateConnectionStatus(CommunicationStatus::DISCONNECTED);
            should_reconnect_.store(true); // 触发重连
        }
    }

    // 如果需要重连，则唤醒重连线程
    if (should_reconnect_.load() && !isConnected() && auto_reconnect_enabled_.load()) {
        reconnect_cv_.notify_one();
    }
}

/**
 * @brief 统计定时器回调
 * @details 定期计算并更新消息频率等统计数据，并触发通信质量回调。
 */
void Go2Communication::statisticsTimerCallback() {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    auto now = std::chrono::steady_clock::now();
    auto connection_duration = std::chrono::duration_cast<std::chrono::seconds>(now - stats_.connection_time).count();

    if (connection_duration > 0) {
        stats_.message_rate_hz = static_cast<float>(stats_.messages_received) / connection_duration;
    }

    // 计算每种消息的频率
    for (auto const& [type, last_time] : last_message_times_) {
        // 这是一个简化的频率计算，更精确的实现需要记录一段时间内的消息数量
    }

    float quality_score = calculateQualityScore();
    if (quality_callback_) {
        quality_callback_(quality_score);
    }
    logDebug("Statistics updated. Quality score: " + std::to_string(quality_score));
}

/**
 * @brief 重连定时器回调（由重连线程调用）
 */
void Go2Communication::reconnectTimerCallback() {
    if (should_reconnect_.load() && !isConnected()) {
        logInfo("Attempting to reconnect to Go2 robot...");
        if (connectToRobot(network_config_.robot_ip)) {
            should_reconnect_.store(false);
            reconnect_attempts_ = 0;
            logInfo("Reconnection successful.");
        } else {
            reconnect_attempts_++;
            logWarning("Reconnection failed. Attempt: " + std::to_string(reconnect_attempts_));
            if (network_config_.max_reconnect_attempts > 0 &&
                reconnect_attempts_ >= network_config_.max_reconnect_attempts) {
                should_reconnect_.store(false);
                recordError("Max reconnection attempts reached. Stopping reconnection.");
            }
        }
    }
}


// ============= 重连线程实现 =============

/**
 * @brief 重连线程的主函数
 * @details 该线程在后台运行，当`should_reconnect_`为true时，它会定期
 *          尝试重新连接，直到连接成功或达到最大尝试次数。
 */
void Go2Communication::reconnectThreadFunction() {
    while (auto_reconnect_enabled_.load()) {
        std::unique_lock<std::mutex> lock(reconnect_mutex_);
        // 等待被唤醒，或超时
        reconnect_cv_.wait_for(lock,
            std::chrono::milliseconds(network_config_.reconnect_interval_ms),
            [this] { return should_reconnect_.load() || !auto_reconnect_enabled_.load(); });

        if (!auto_reconnect_enabled_.load()) break; // 如果禁用了自动重连，则退出线程

        if (should_reconnect_.load() && !isConnected()) {
            reconnectTimerCallback();
        }
    }
}


// ============= 辅助方法实现 =============

/**
 * @brief 更新统计数据
 */
void Go2Communication::updateStatistics(MessageType type, size_t bytes) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.messages_received++;
    stats_.total_bytes_received += bytes;
    stats_.last_message_time = std::chrono::steady_clock::now();
    last_message_times_[type] = stats_.last_message_time;
}

/**
 * @brief 管理消息缓冲区，防止溢出
 */
template<typename T>
void Go2Communication::manageBuffer(std::queue<T>& buffer, const T& msg, size_t max_size) {
    buffer.push(msg);
    while (buffer.size() > max_size) {
        buffer.pop();
    }
}

/**
 * @brief 计算通信质量分数
 * @details 一个简单的质量评估模型，综合考虑连接状态、消息率和错误数量。
 */
float Go2Communication::calculateQualityScore() const {
    if (!isConnected()) return 0.0f;

    float score = 1.0f;
    // 根据消息率调整分数
    if (stats_.message_rate_hz < 5.0f) { // 假设正常通信频率应高于5Hz
        score *= 0.8f;
    }
    // 根据错误数量调整分数
    std::lock_guard<std::mutex> lock(error_mutex_);
    if (!error_history_.empty()) {
        score *= (1.0f - std::min(1.0f, error_history_.size() / 10.0f)); // 错误越多，分数越低
    }
    return score;
}

/**
 * @brief 记录错误信息
 */
void Go2Communication::recordError(const std::string& error_msg) {
    logError(error_msg); // 使用日志记录器打印错误
    {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_error_ = error_msg;
        error_history_.push_back(error_msg);
        if (error_history_.size() > 100) { // 保持历史记录的大小
            error_history_.erase(error_history_.begin());
        }
    }
    if (error_callback_) {
        error_callback_(error_msg);
    }
}

/**
 * @brief 更新连接状态并触发回调
 */
void Go2Communication::updateConnectionStatus(CommunicationStatus new_status) {
    CommunicationStatus old_status = status_.load();
    if (old_status != new_status) {
        status_.store(new_status);
        if (new_status == CommunicationStatus::CONNECTED) {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.connection_time = std::chrono::steady_clock::now();
        }
        logInfo("Connection status changed from " + std::to_string(static_cast<int>(old_status)) +
                " to " + std::to_string(static_cast<int>(new_status)));
        if (status_callback_) {
            status_callback_(new_status);
        }
    }
}

/**
 * @brief 配置网络参数（占位符）
 */
bool Go2Communication::configureNetwork() {
    logInfo("Configuring network parameters...");
    logInfo("Robot IP: " + network_config_.robot_ip);
    logInfo("Local IP: " + network_config_.local_ip);
    logInfo("Network Interface: " + network_config_.network_interface);
    logInfo("DDS Domain ID: " + std::to_string(network_config_.dds_domain_id));
    // 在实际应用中，这里会设置环境变量，如 CYCLONEDDS_URI
    return true;
}

/**
 * @brief 验证网络连接（占位符）
 */
bool Go2Communication::verifyNetworkConnection() {
    logDebug("Verifying network connection...");
    if (!node_) {
        recordError("ROS node is invalid.");
        return false;
    }
    // 实际应用中可以加入ping测试或检查ROS2守护进程状态
    return true;
}


// ============= 日志记录辅助方法 =============

void Go2Communication::logDebug(const std::string& msg) const {
    if (verbose_logging_.load() && node_) {
        RCLCPP_DEBUG(node_->get_logger(), "[Go2Comm] %s", msg.c_str());
    }
}

void Go2Communication::logInfo(const std::string& msg) const {
    if (node_) {
        RCLCPP_INFO(node_->get_logger(), "[Go2Comm] %s", msg.c_str());
    }
}

void Go2Communication::logWarning(const std::string& msg) const {
    if (node_) {
        RCLCPP_WARN(node_->get_logger(), "[Go2Comm] %s", msg.c_str());
    }
}

void Go2Communication::logError(const std::string& msg) const {
    if (node_) {
        RCLCPP_ERROR(node_->get_logger(), "[Go2Comm] %s", msg.c_str());
    }
}

} // namespace go2_adapter
} // namespace robot_adapters
