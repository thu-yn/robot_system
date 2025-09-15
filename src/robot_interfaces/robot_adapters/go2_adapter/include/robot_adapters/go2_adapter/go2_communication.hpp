/**
 * @file   go2_communication.hpp
 * @brief  Go2机器人通信管理器头文件
 * @author Yang Nan
 * @date   2025-09-10
 *
 * @details
 * 本文件定义了 `Go2Communication` 类，这是一个专门负责管理与Go2机器人
 * 所有ROS2通信的核心组件。它封装了底层的ROS2话题订阅与发布、连接状态监控、
 * 消息缓冲、错误处理及自动重连等复杂机制。
 *
 * 设计目标是提供一个稳定、可靠且易于使用的接口，供上层应用（如适配器）
 * 与机器人进行数据交换，同时将通信细节隔离开来。
 *
 * 主要功能：
 *      - 统一的ROS2话题创建和管理。
 *      - 订阅Go2机器人的状态和传感器数据（如IMU, Odometry, Lidar等）。
 *      - 向Go2机器人发布控制命令（如速度指令）。
 *      - 监控通信链路的健康状况，并在连接断开时自动尝试重连。
 *      - 提供通信质量统计数据，如延迟、频率、丢包率等。
 *      - 通过回调函数向上层传递接收到的数据和重要事件。
 */

#ifndef ROBOT_ADAPTERS__GO2_ADAPTER__GO2_COMMUNICATION_HPP_
#define ROBOT_ADAPTERS__GO2_ADAPTER__GO2_COMMUNICATION_HPP_

#include <rclcpp/rclcpp.hpp>    // ROS2 C++核心库
#include <memory>               // C++智能指针库
#include <string>               // C++字符串库
#include <vector>               // C++动态数组
#include <map>                  // C++映射容器
#include <functional>           // C++函数对象库
#include <mutex>                // C++互斥锁，用于多线程同步
#include <atomic>               // C++原子操作库，用于线程安全的变量访问
#include <thread>               // C++线程库
#include <queue>                // C++队列容器，用于消息缓冲
#include <chrono>               // C++时间库
#include <condition_variable>   // C++条件变量，用于线程同步

// 导入Unitree Go2官方定义的ROS2消息类型
#include "unitree_go/msg/sport_mode_state.hpp"      // 运动模式状态
#include "unitree_go/msg/low_state.hpp"             // 机器人底层状态
#include "unitree_go/msg/low_cmd.hpp"               // 机器人底层命令
#include "unitree_go/msg/bms_state.hpp"             // 电池管理系统状态
#include "unitree_go/msg/wireless_controller.hpp"   // 无线手柄数据
#include "unitree_api/msg/request.hpp"              // 通用API请求

// 导入标准的ROS2消息类型
#include <sensor_msgs/msg/point_cloud2.hpp>         // 3D点云数据
#include <sensor_msgs/msg/imu.hpp>                  // 惯性测量单元数据
#include <geometry_msgs/msg/twist.hpp>              // 速度控制指令
#include <nav_msgs/msg/odometry.hpp>                // 里程计数据

namespace robot_adapters {
namespace go2_adapter {

/**
 * @enum CommunicationStatus
 * @brief 定义通信连接的几种状态。
 */
enum class CommunicationStatus {
    DISCONNECTED = 0,   ///< 已断开：初始状态或已手动断开。
    CONNECTING   = 1,   ///< 连接中：正在尝试建立连接。
    CONNECTED    = 2,   ///< 已连接：通信链路正常。
    RECONNECTING = 3,   ///< 重连中：连接丢失后正在尝试自动重连。
    ERROR        = 4    ///< 错误状态：发生不可恢复的通信错误。
};

/**
 * @enum MessageType
 * @brief 定义了本模块处理的几种主要消息类型，用于统计和缓冲管理。
 */
enum class MessageType {
    SPORT_MODE_STATE    = 0,    ///< 运动模式状态消息
    LOW_STATE           = 1,    ///< 底层状态消息
    LOW_COMMAND         = 2,    ///< 底层命令消息
    BMS_STATE           = 3,    ///< 电池管理状态消息
    WIRELESS_CONTROLLER = 4,    ///< 无线手柄消息
    API_REQUEST         = 5,    ///< API请求消息
    POINT_CLOUD         = 6,    ///< 点云数据消息
    IMU_DATA            = 7,    ///< IMU数据消息
    ODOMETRY            = 8     ///< 里程计数据消息
};

/**
 * @struct CommunicationStats
 * @brief 用于存储和报告通信质量的统计数据。
 */
struct CommunicationStats {
    uint64_t messages_sent       = 0;   ///< 已发送的消息总数
    uint64_t messages_received   = 0;   ///< 已接收的消息总数
    uint64_t messages_lost       = 0;   ///< 估算的丢失消息数
    uint64_t connection_attempts = 0;   ///< 尝试建立连接的总次数
    uint64_t reconnections       = 0;   ///< 成功重连的总次数

    float average_latency_ms      = 0.0f;   ///< 平均消息延迟（毫秒）
    float message_rate_hz         = 0.0f;   ///< 平均消息接收频率（赫兹）
    uint64_t total_bytes_sent     = 0;      ///< 已发送的总字节数
    uint64_t total_bytes_received = 0;      ///< 已接收的总字节数

    std::chrono::steady_clock::time_point last_message_time;    ///< 收到最后一条消息的时间点
    std::chrono::steady_clock::time_point connection_time;      ///< 本次连接成功建立的时间点
};

/**
 * @class Go2Communication
 * @brief Go2机器人通信管理器类。
 *
 * @details
 * 封装了与Go2机器人进行ROS2通信的所有逻辑。它通过组合ROS2的发布者、
 * 订阅者和定时器来构建一个健壮的通信层。
 */
class Go2Communication {
public:
    /**
     * @brief 构造函数
     * @param node 一个指向外部ROS节点的共享指针，本类将使用该节点来创建ROS实体。
     */
    explicit Go2Communication(std::shared_ptr<rclcpp::Node> node);

    /**
     * @brief 析构函数
     * @details 确保在对象销毁时调用`shutdown`来释放所有资源。
     */
    virtual ~Go2Communication();

    // ============= 初始化与生命周期管理 =============

    /**
     * @brief 初始化通信管理器。
     * @return 如果初始化成功（例如，创建了发布者和订阅者），返回true。
     */
    bool initialize();

    /**
     * @brief 关闭通信管理器。
     * @return 如果关闭成功（例如，销毁了所有ROS实体），返回true。
     */
    bool shutdown();

    /**
     * @brief 启动通信过程。
     * @details 目前此函数作为占位符，实际的通信在`initialize`后自动开始。
     * @return 总是返回true。
     */
    bool startCommunication();

    /**
     * @brief 停止通信过程。
     * @details 目前此函数作为占位符。
     * @return 总是返回true。
     */
    bool stopCommunication();

    /**
     * @brief 检查通信管理器是否已初始化。
     * @return 如果`initialize`已成功调用，返回true。
     */
    bool isInitialized() const { return is_initialized_.load(); }

    /**
     * @brief 检查通信是否正在进行（已连接）。
     * @return 如果当前状态是`CONNECTED`，返回true。
     */
    bool isCommunicating() const;

    // ============= 连接管理 =============

    /**
     * @brief 尝试连接到Go2机器人。
     * @param robot_ip Go2机器人的IP地址。
     * @param timeout_seconds 连接超时时间（秒）。
     * @return 如果连接成功，返回true。
     * @details 此函数目前是一个占位符，实际的连接状态依赖于ROS2 DDS的发现机制。
     */
    bool connectToRobot(const std::string& robot_ip, int timeout_seconds = 10);

    /**
     * @brief 断开与Go2机器人的连接。
     * @return 如果断开成功，返回true。
     * @details 此函数目前是一个占位符。
     */
    bool disconnectFromRobot();

    /**
     * @brief 获取当前的连接状态。
     * @return `CommunicationStatus`枚举值。
     */
    CommunicationStatus getConnectionStatus() const;

    /**
     * @brief 检查是否已连接。
     * @return 如果状态为`CONNECTED`，返回true。
     */
    bool isConnected() const;

    /**
     * @brief 配置自动重连机制。
     * @param enable 是否启用自动重连。
     * @param retry_interval_ms 每次重连尝试的间隔时间（毫秒）。
     * @param max_retries 最大重试次数（0表示无限次）。
     */
    void setAutoReconnect(bool enable, int retry_interval_ms = 5000, int max_retries = 0);

    // ============= 消息发布接口 =============

    /**
     * @brief 发送Go2 API请求。
     * @param request Go2机器人的API请求消息。
     * @return 如果发布成功，返回true。
     * @details 这是Go2机器人唯一支持的控制命令发送方式，所有运动控制都必须通过此接口
     */
    bool sendApiRequest(const unitree_api::msg::Request& request);

    // ============= 消息订阅接口与回调设置 =============

    /**
     * @brief 设置点云数据的回调函数。
     * @param callback 当收到新的点云数据时将被调用的函数。
     */
    void setPointCloudCallback(std::function<void(const sensor_msgs::msg::PointCloud2::SharedPtr)> callback);

    /**
     * @brief 设置IMU数据的回调函数。
     * @param callback 当收到新的IMU数据时将被调用的函数。
     */
    void setImuCallback(std::function<void(const sensor_msgs::msg::Imu::SharedPtr)> callback);

    /**
     * @brief 设置里程计数据的回调函数。
     * @param callback 当收到新的里程计数据时将被调用的函数。
     */
    void setOdometryCallback(std::function<void(const nav_msgs::msg::Odometry::SharedPtr)> callback);

    /**
     * @brief 设置运动模式状态的回调函数。
     * @param callback 当收到新的运动模式状态时将被调用的函数。
     */
    void setSportModeStateCallback(std::function<void(const unitree_go::msg::SportModeState::SharedPtr)> callback);

    /**
     * @brief 设置底层状态数据的回调函数。
     * @param callback 当收到新的底层状态数据时将被调用的函数。
     */
    void setLowStateCallback(std::function<void(const unitree_go::msg::LowState::SharedPtr)> callback);

    /**
     * @brief 设置电池管理系统状态的回调函数。
     * @param callback 当收到新的BMS状态时将被调用的函数。
     */
    void setBmsStateCallback(std::function<void(const unitree_go::msg::BmsState::SharedPtr)> callback);

    // ============= 消息缓冲与队列管理 =============

    /**
     * @brief 获取最新收到的点云数据。
     * @return 指向最新点云消息的共享指针，如果缓冲区为空则返回`nullptr`。
     */
    std::shared_ptr<sensor_msgs::msg::PointCloud2> getLatestPointCloud();

    /**
     * @brief 获取最新收到的IMU数据。
     * @return 指向最新IMU消息的共享指针，如果缓冲区为空则返回`nullptr`。
     */
    std::shared_ptr<sensor_msgs::msg::Imu> getLatestImu();

    /**
     * @brief 获取最新收到的运动模式状态。
     * @return 指向最新运动模式状态消息的共享指针，如果缓冲区为空则返回`nullptr`。
     */
    std::shared_ptr<unitree_go::msg::SportModeState> getLatestSportModeState();

    /**
     * @brief 获取最新收到的底层状态数据。
     * @return 指向最新底层状态消息的共享指针，如果缓冲区为空则返回`nullptr`。
     */
    std::shared_ptr<unitree_go::msg::LowState> getLatestLowState();

    /**
     * @brief 获取最新收到的BMS状态数据。
     * @return 指向最新BMS状态消息的共享指针，如果缓冲区为空则返回`nullptr`。
     */
    std::shared_ptr<unitree_go::msg::BmsState> getLatestBmsState();

    /**
     * @brief 设置特定消息类型的缓冲区大小。
     * @param message_type 要设置的消息类型。
     * @param buffer_size 缓冲区的最大容量。
     */
    void setMessageBufferSize(MessageType message_type, size_t buffer_size);

    /**
     * @brief 清空指定消息类型的缓冲区。
     * @param message_type 要清空的消息类型。
     */
    void clearMessageBuffer(MessageType message_type);

    // ============= 通信质量监控 =============

    /**
     * @brief 获取当前的通信统计数据。
     * @return `CommunicationStats`结构体。
     */
    CommunicationStats getStatistics() const;

    /**
     * @brief 获取特定消息类型的端到端延迟。
     * @param message_type 消息类型。
     * @return 延迟（毫秒），当前总是返回-1。
     * TODO:需要实现。
     */
    float getMessageLatency(MessageType message_type) const;

    /**
     * @brief 获取特定消息类型的接收频率。
     * @param message_type 消息类型。
     * @return 频率（Hz），如果无数据则返回-1。
     */
    float getMessageFrequency(MessageType message_type) const;

    /**
     * @brief 检查整体通信质量是否良好。
     * @return 如果通信质量良好，返回true。
     */
    bool isCommuncationQualityGood() const;

    /**
     * @brief 重置所有通信统计数据。
     */
    void resetStatistics();

    // ============= 网络配置 =============

    /**
     * @brief 设置用于DDS通信的网络接口。
     * @param interface_name 网络接口的名称 (例如 "enp3s0")。
     * @return 如果设置成功，返回true。
     */
    bool setNetworkInterface(const std::string& interface_name);

    /**
     * @brief 设置DDS域ID。
     * @param domain_id 要使用的DDS域ID。
     * @return 如果设置成功，返回true。
     */
    bool setDdsDomainId(int domain_id);

    /**
     * @brief 设置ROS2 QoS（服务质量）参数。
     * @param reliability 可靠性策略。
     * @param durability 持久性策略。
     * @param history_depth 历史记录深度。
     */
    void setQosSettings(rclcpp::ReliabilityPolicy reliability,
                       rclcpp::DurabilityPolicy durability,
                       size_t history_depth);

    /**
     * @brief 应用网络优化配置。
     * @return 总是返回true。
     * TODO:需要实现。
     */
    bool applyNetworkOptimization();

    // ============= 错误处理与诊断 =============

    /**
     * @brief 获取最后一次发生的错误信息。
     * @return 错误描述字符串。
     */
    std::string getLastError() const;

    /**
     * @brief 获取所有记录在案的错误历史。
     * @return 包含所有错误信息的字符串向量。
     */
    std::vector<std::string> getErrorHistory() const;

    /**
     * @brief 清除所有错误历史记录。
     */
    void clearErrorHistory();

    /**
     * @brief 执行连接诊断。
     * @return 返回一个空的诊断结果map。
     * TODO:需要实现。
     */
    std::map<std::string, bool> performConnectionDiagnostics();

    /**
     * @brief 获取网络诊断信息。
     * @return 返回一个空的JSON字符串。
     * TODO:需要实现。
     */
    std::string getNetworkDiagnostics() const;

    // ============= 事件回调注册 =============

    /**
     * @brief 注册连接状态变化的回调函数。
     * @param callback 当连接状态改变时被调用的函数。
     */
    void setConnectionStatusCallback(std::function<void(CommunicationStatus)> callback);

    /**
     * @brief 注册通信错误的回调函数。
     * @param callback 当发生错误时被调用的函数。
     */
    void setErrorCallback(std::function<void(const std::string&)> callback);

    /**
     * @brief 注册通信质量变化的回调函数。
     * @param callback 当通信质量分数变化时被调用的函数。
     */
    void setQualityCallback(std::function<void(float)> callback);

    // ============= 配置参数 =============

    /**
     * @brief 加载配置。
     * @return 总是返回true。
     * TODO:需要实现。
     */
    bool loadConfiguration();

    /**
     * @brief 保存配置。
     * @return 总是返回true。
     * TODO:需要实现。
     */
    bool saveConfiguration();

    /**
     * @brief 获取配置信息。
     * @return 返回一个空的JSON字符串。
     * TODO:需要实现。
     */
    std::string getConfiguration() const;

    /**
     * @brief 启用或禁用详细日志记录。
     * @param enable true表示启用，false表示禁用。
     */
    void setVerboseLogging(bool enable);

private:
    // ============= 内部成员变量 =============
    std::shared_ptr<rclcpp::Node> node_;        ///< 指向外部ROS节点的共享指针

    // --- 状态标志 ---
    std::atomic<bool> is_initialized_;          ///< 原子变量，标记是否已初始化
    std::atomic<CommunicationStatus> status_;   ///< 原子变量，存储当前通信状态
    std::atomic<bool> auto_reconnect_enabled_;  ///< 原子变量，标记是否启用自动重连
    std::atomic<bool> verbose_logging_;         ///< 原子变量，标记是否启用详细日志

    // --- 网络配置 ---
    struct NetworkConfig {
        std::string robot_ip = "192.168.123.18";    ///< Go2机器人的默认IP地址
        std::string local_ip = "192.168.123.99";    ///< 本地设备的IP地址
        std::string network_interface = "enp129s0"; ///< 默认使用的网络接口
        int dds_domain_id = 0;                      ///< DDS域ID
        int connection_timeout_ms = 10000;          ///< 连接超时时间（毫秒）
        int reconnect_interval_ms = 5000;           ///< 自动重连间隔（毫秒）
        int max_reconnect_attempts = 0;             ///< 最大重连次数（0为无限）
    } network_config_;

    // --- ROS2 发布者与订阅者 ---
    // Go2机器人唯一支持的控制命令发布者
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr api_request_pub_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr   pointcloud_sub_;   // 点云订阅者
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr           imu_sub_;          // IMU订阅者
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr         odom_sub_;         // 里程计订阅者
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sport_state_sub_;
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr       low_state_sub_;
    rclcpp::Subscription<unitree_go::msg::BmsState>::SharedPtr       bms_state_sub_;

    // --- 定时器 ---
    rclcpp::TimerBase::SharedPtr monitor_timer_;          ///< 用于监控连接状态的定时器
    rclcpp::TimerBase::SharedPtr reconnect_timer_;        ///< 用于触发重连的定时器
    rclcpp::TimerBase::SharedPtr statistics_timer_;       ///< 用于更新统计数据的定时器

    // --- 消息缓冲区 ---
    mutable std::mutex buffer_mutex_;               ///< 用于保护消息缓冲区的互斥锁
    std::map<MessageType, size_t> buffer_sizes_;    ///< 存储不同消息类型的缓冲区大小
    std::queue<std::shared_ptr<sensor_msgs::msg::PointCloud2>>   pointcloud_buffer_;    ///< 点云消息队列
    std::queue<std::shared_ptr<sensor_msgs::msg::Imu>>           imu_buffer_;           ///< IMU消息队列
    std::queue<std::shared_ptr<nav_msgs::msg::Odometry>>         odom_buffer_;          ///< 里程计消息队列
    std::queue<std::shared_ptr<unitree_go::msg::SportModeState>> sport_state_buffer_;
    std::queue<std::shared_ptr<unitree_go::msg::LowState>>       low_state_buffer_;
    std::queue<std::shared_ptr<unitree_go::msg::BmsState>>       bms_state_buffer_;

    // --- 统计信息 ---
    mutable std::mutex stats_mutex_; ///< 用于保护统计数据的互斥锁
    CommunicationStats stats_;       ///< 统计数据结构体
    std::map<MessageType, std::chrono::steady_clock::time_point> last_message_times_; ///< 记录各类消息的最后接收时间
    std::map<MessageType, float> message_frequencies_; ///< 记录各类消息的接收频率

    // --- 错误管理 ---
    mutable std::mutex error_mutex_;            ///< 用于保护错误记录的互斥锁
    std::string last_error_;                    ///< 最后一条错误信息
    std::vector<std::string> error_history_;    ///< 错误历史记录

    // --- 外部回调函数 ---
    std::function<void(const sensor_msgs::msg::PointCloud2::SharedPtr)>   pointcloud_callback_;
    std::function<void(const sensor_msgs::msg::Imu::SharedPtr)>           imu_callback_;
    std::function<void(const nav_msgs::msg::Odometry::SharedPtr)>         odom_callback_;
    std::function<void(const unitree_go::msg::SportModeState::SharedPtr)> sport_state_callback_;
    std::function<void(const unitree_go::msg::LowState::SharedPtr)>       low_state_callback_;
    std::function<void(const unitree_go::msg::BmsState::SharedPtr)>       bms_state_callback_;

    std::function<void(CommunicationStatus)> status_callback_; ///< 连接状态变化回调
    std::function<void(const std::string&)> error_callback_;   ///< 错误事件回调
    std::function<void(float)> quality_callback_;              ///< 通信质量变化回调

    // --- 重连管理 ---
    std::thread reconnect_thread_;              ///< 执行重连逻辑的独立线程
    std::atomic<bool> should_reconnect_;        ///< 控制重连线程是否继续运行的标志
    std::condition_variable reconnect_cv_;      ///< 用于唤醒重连线程的条件变量
    std::mutex reconnect_mutex_;                ///< 与条件变量配合使用的互斥锁
    int reconnect_attempts_;                    ///< 当前的重连尝试次数

    // ============= 内部私有方法 =============

    /**
     * @brief 创建所有ROS2发布者和订阅者。
     * @return 如果全部创建成功，返回true。
     */
    bool createPublishersAndSubscribers();

    /**
     * @brief 销毁所有ROS2发布者和订阅者。
     */
    void destroyPublishersAndSubscribers();

    // --- 内部消息回调 ---
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void sportStateCallback(const unitree_go::msg::SportModeState::SharedPtr msg);
    void lowStateCallback(const unitree_go::msg::LowState::SharedPtr msg);
    void bmsStateCallback(const unitree_go::msg::BmsState::SharedPtr msg);

    // --- 内部定时器回调 ---
    void monitorTimerCallback();
    void statisticsTimerCallback();
    void reconnectTimerCallback();

    /**
     * @brief 自动重连线程的执行函数。
     */
    void reconnectThreadFunction();

    /**
     * @brief 更新接收到消息后的统计数据。
     * @param type 消息类型。
     * @param bytes 消息大小（字节）。
     */
    void updateStatistics(MessageType type, size_t bytes);

    /**
     * @brief 管理消息缓冲区，确保其大小不超过限制。
     * @tparam T 消息类型。
     * @param buffer 消息队列的引用。
     * @param msg 新接收到的消息。
     * @param max_size 缓冲区的最大尺寸。
     */
    template<typename T>
    void manageBuffer(std::queue<T>& buffer, const T& msg, size_t max_size);

    /**
     * @brief 计算当前的通信质量分数（0.0到1.0）。
     * @return 通信质量分数。
     */
    float calculateQualityScore() const;

    /**
     * @brief 记录一条错误信息。
     * @param error_msg 错误描述。
     */
    void recordError(const std::string& error_msg);

    /**
     * @brief 更新连接状态并触发回调。
     * @param new_status 新的连接状态。
     */
    void updateConnectionStatus(CommunicationStatus new_status);

    /**
     * @brief 配置底层网络参数（未实现）。
     * @return 总是返回true。
     */
    bool configureNetwork();

    /**
     * @brief 验证网络连接（未实现）。
     * @return 总是返回true。
     */
    bool verifyNetworkConnection();

    // --- 日志记录辅助函数 ---
    void logDebug(const std::string& msg) const;
    void logInfo(const std::string& msg) const;
    void logWarning(const std::string& msg) const;
    void logError(const std::string& msg) const;
};

// --- 类型别名定义 ---
using Go2CommunicationPtr = std::shared_ptr<Go2Communication>; ///< Go2Communication的共享指针
using Go2CommunicationUniquePtr = std::unique_ptr<Go2Communication>; ///< Go2Communication的唯一指针

} // namespace go2_adapter
} // namespace robot_adapters
#endif // ROBOT_ADAPTERS__GO2_ADAPTER__GO2_COMMUNICATION_HPP_