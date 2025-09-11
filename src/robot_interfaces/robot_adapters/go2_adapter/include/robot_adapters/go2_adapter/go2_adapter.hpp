/**
 * @file   go2_adapter.hpp
 * @brief  Go2机器人适配器主类头文件 - 定义了Go2Adapter类的结构和接口
 * @author Yang nan
 * @date   2025-09-11
 * 
 * @details
 * 该文件定义了`Go2Adapter`类，它是连接上层通用机器人系统与Unitree Go2机器人底层接口的核心桥梁。
 * 它继承自`rclcpp::Node`，因此本身就是一个ROS2节点，能够创建发布者、订阅者、服务等。
 * 该类通过组合持有运动控制、传感器、状态监控和电源管理等具体实现模块的实例，
 * 并将它们封装在统一的`IRobotAdapter`接口之后，实现了对上层应用的透明化。
 * 作为Go2机器人与统一接口层之间的桥梁，封装了：                                                                │
 *      - 运动控制接口 (IMotionController)                                                                           │
 *      - 传感器接口 (ISensorInterface)                                                                              │
 *      - 状态监控接口 (IStateMonitor)                                                                               │
 *      - 电源管理接口 (IPowerManager)
 * 使用方式：                                                                                                   │
 *      1. 创建适配器实例                                                                                            │
 *      2. 调用initialize()初始化                                                                                    │
 *      3. 通过get*Interface()获取具体接口进行操作                                                                   │
 *      4. 使用完毕后调用shutdown()清理资源  
 */

 #ifndef ROBOT_ADAPTERS__GO2_ADAPTER__GO2_ADAPTER_HPP_
 #define ROBOT_ADAPTERS__GO2_ADAPTER__GO2_ADAPTER_HPP_

#include <rclcpp/rclcpp.hpp>        // ROS2 C++客户端核心库
#include <std_srvs/srv/trigger.hpp> // ROS2标准服务类型，常用于触发简单操作
#include <memory>                   // 用于智能指针 (std::shared_ptr, std::unique_ptr)
#include <string>                   // 用于字符串处理
#include <map>                      // 用于键值对存储 (std::map)
#include <vector>                   // 用于动态数组 (std::vector)
#include <functional>               // 用于函数对象 (std::function)
#include <mutex>                    // 用于线程同步的互斥锁
#include <atomic>                   // 用于原子操作
#include <chrono>                   // 用于时间和日期

// 基础抽象接口
#include "robot_base_interfaces/motion_interface/i_motion_controller.hpp" // 运动控制模块的抽象接口
#include "robot_base_interfaces/sensor_interface/i_sensor_interface.hpp"  // 传感器模块的抽象接口
#include "robot_base_interfaces/state_interface/i_state_monitor.hpp"      // 状态监控模块的抽象接口
#include "robot_base_interfaces/power_interface/i_power_manager.hpp"      // 电源管理模块的抽象接口

// Go2具体实现
#include "go2_motion_controller.hpp"   // Go2运动控制的具体实现
#include "go2_sensor_interface.hpp"    // Go2传感器接口的具体实现
#include "go2_state_monitor.hpp"       // Go2状态监控的具体实现
#include "go2_power_manager.hpp"       // Go2电源管理的的具体实现
#include "go2_communication.hpp"       // Go2通信管理的具体实现
#include "go2_message_converter.hpp"   // Go2消息格式转换器的具体实现

namespace robot_adapters {
namespace go2_adapter {

/**
 * @class Go2Adapter
 * @brief Go2机器人适配器主类
 * 
 * @details
 * 该类是`go2_adapter`包的核心。它作为一个ROS2节点运行，并执行以下关键任务：
 * 1. **实现统一接口**：为上层系统提供一个与具体机器人无关的标准化交互接口。
 * 2. **聚合功能模块**：创建并管理所有针对Go2的具体功能模块（运动、传感、状态、电源）。
 * 3. **生命周期管理**：负责整个适配器从初始化到安全关闭的完整生命周期。
 * 4. **提供ROS接口**：通过ROS2服务和话题，向外部暴露控制和监控的能力。
 * 
 * @note
 * 该类继承自 `rclcpp::Node`，使其具备完整的ROS2节点功能。
 */
class Go2Adapter : public rclcpp::Node {
public:
    /**
     * @brief 构造函数
     * @details 初始化Go2Adapter节点，但此时尚未与机器人建立连接或初始化子模块。
     * @param node_name ROS节点的名称。
     * @param node_options ROS节点的配置选项。
     */
    explicit Go2Adapter(
        const std::string& node_name = "go2_adapter",
        const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions()
    );
    
    /**
     * @brief 析构函数
     * @details 确保在对象销毁时调用shutdown()方法，安全地释放所有资源。
     */
    virtual ~Go2Adapter();

    // ============= 生命周期管理 (Lifecycle Management) =============
    
    /**
     * @brief 初始化适配器及所有子模块。
     * @details 这是适配器生命周期中最重要的方法之一。它会执行以下操作：
     *      - 加载ROS参数
     *      - 初始化通信管理器
     *      - 创建并初始化所有功能模块（运动、传感、状态、电源）
     *      - 设置心跳定时器和ROS服务
     * @return 如果所有步骤都成功，则返回true；否则返回false。
     */
    bool initialize();
    
    /**
     * @brief 关闭适配器，安全地释放所有资源。
     * @details
     *      - 停止心跳定时器
     *      - 关闭所有ROS接口
     *      - 依次关闭所有子模块
     */
    void shutdown();
    
    /**
     * @brief 检查适配器是否已成功初始化。
     * @return 如果initialize()已成功调用，返回true。
     */
    bool isInitialized() const { return is_initialized_; }
    
    /**
     * @brief 检查适配器当前是否处于可操作状态。
     * @details 可操作意味着已初始化且与机器人的通信正常。
     * @return 如果适配器可正常工作，返回true。
     */
    bool isOperational() const;

    // ============= 接口访问器 (Interface Accessors) =============
    
    /**
     * @brief 获取运动控制模块的接口。
     * @return 指向`IMotionController`接口的共享指针。如果模块未初始化，可能返回nullptr。
     */
    std::shared_ptr<robot_base_interfaces::motion_interface::IMotionController>
    getMotionController() const { return motion_controller_; }
    
    /**
     * @brief 获取传感器模块的接口。
     * @return 指向`ISensorInterface`接口的共享指针。
     */
    std::shared_ptr<robot_base_interfaces::sensor_interface::ISensorInterface>
    getSensorInterface() const { return sensor_interface_; }
    
    /**
     * @brief 获取状态监控模块的接口。
     * @return 指向`IStateMonitor`接口的共享指针。
     */
    std::shared_ptr<robot_base_interfaces::state_interface::IStateMonitor>
    getStateMonitor() const { return state_monitor_; }
    
    /**
     * @brief 获取电源管理模块的接口。
     * @return 指向`IPowerManager`接口的共享指针。
     */
    std::shared_ptr<robot_base_interfaces::power_interface::IPowerManager>
    getPowerManager() const { return power_manager_; }

    // ============= Go2专有功能访问 (Go2-Specific Accessors) =============
    
    /**
     * @brief 获取Go2通信管理器实例。
     * @details 用于访问底层的发布和订阅逻辑。
     * @return 指向`Go2Communication`的共享指针。
     */
    std::shared_ptr<Go2Communication> getCommunicationManager() const { 
        return communication_manager_; 
    }
    
    /**
     * @brief 获取Go2消息转换器实例。
     * @details 用于在标准消息和Go2专用消息之间进行转换。
     * @return 指向`Go2MessageConverter`的共享指针。
     */
    std::shared_ptr<Go2MessageConverter> getMessageConverter() const { 
        return message_converter_; 
    }

    // ============= 配置和参数 (Configuration & Parameters) =============
    
    /**
     * @brief 从ROS参数服务器加载配置。
     * @return 加载成功返回true。
     */
    bool loadConfiguration();
    
    /**
     * @brief (未实现) 保存当前配置到ROS参数服务器。
     * @return 默认为true。
     */
    bool saveConfiguration();
    
    /**
     * @brief 重新加载所有ROS参数。
     * @return 加载成功返回true。
     */
    bool reloadConfiguration();
    
    /**
     * @brief 以JSON字符串格式获取当前配置。
     * @return 包含所有配置项的JSON字符串。
     */
    std::string getConfiguration() const;

    // ============= 诊断和监控 (Diagnostics & Monitoring) =============
    
    /**
     * @brief 以JSON字符串格式获取适配器当前状态。
     * @return 包含运行状态、版本、错误计数等的JSON字符串。
     */
    std::string getAdapterStatus() const;
    
    /**
     * @brief 以JSON字符串格式获取性能统计数据。
     * @return 包含运行时间、心跳计数、内存使用等的JSON字符串。
     */
    std::string getPerformanceStats() const;
    
    /**
     * @brief 执行一次完整的系统自检。
     * @return 一个map，键为检查项，值为检查结果(true/false)。
     */
    std::map<std::string, bool> performSystemCheck();
    
    /**
     * @brief 获取当前记录的所有错误消息。
     * @return 包含错误消息字符串的vector。
     */
    std::vector<std::string> getErrorMessages() const;
    
    /**
     * @brief 清空已记录的错误消息。
     */
    void clearErrors();

    // ============= 事件和回调 (Events & Callbacks) =============
    
    /**
     * @brief 注册一个当适配器状态发生变化时触发的回调函数。
     * @param callback 回调函数，参数为新的状态字符串。
     */
    void setStatusChangeCallback(
        std::function<void(const std::string& status)> callback
    );
    
    /**
     * @brief 注册一个当有新错误发生时触发的回调函数。
     * @param callback 回调函数，参数为错误消息字符串。
     */
    void setErrorCallback(
        std::function<void(const std::string& error)> callback
    );

    // ============= 静态方法 (Static Methods) =============
    
    /**
     * @brief 获取适配器的硬编码版本号。
     * @return 版本号字符串。
     */
    static std::string getVersion() { return "1.0.0"; }
    
    /**
     * @brief 获取此适配器支持的机器人类型。
     * @return `RobotType::GO2`
     */
    static robot_base_interfaces::motion_interface::RobotType 
    getSupportedRobotType() { 
        return robot_base_interfaces::motion_interface::RobotType::GO2; 
    }
    
    /**
     * @brief 检查Go2机器人是否在网络上可达。
     * @details 通常通过ping等方式实现。
     * @return 如果机器人可达，返回true。
     */
    static bool isGo2Available();

protected:
    // ============= 内部辅助方法 (Internal Helper Methods) =============
    
    /**
     * @brief 初始化所有子系统模块。
     * @details 按照正确的依赖顺序创建和初始化所有功能模块的实例。
     * @return 全部成功返回true。
     */
    bool initializeSubsystems();
    
    /**
     * @brief 关闭所有子系统模块。
     * @details 按照与初始化相反的顺序安全地关闭和释放所有功能模块。
     */
    void shutdownSubsystems();
    
    /**
     * @brief 设置（声明和加载）ROS参数。
     */
    void setupRosParameters();
    
    /**
     * @brief 创建并设置所有ROS接口，如服务、定时器等。
     */
    void setupRosInterfaces();
    
    /**
     * @brief 验证当前加载的配置参数是否有效。
     * @return 全部有效返回true。
     */
    bool validateConfiguration() const;
    
    /**
     * @brief 记录一条错误信息到日志和内部错误列表。
     * @param error_msg 要记录的错误消息。
     */
    void logError(const std::string& error_msg);
    
    /**
     * @brief 更新适配器的内部状态并触发回调。
     * @param status 新的状态字符串。
     */
    void updateStatus(const std::string& status);
    
    /**
     * @brief 获取当前节点的内存使用情况（仅在Linux下有效）。
     * @return 估算的内存使用量（MB）。
     */
    double getMemoryUsageMB() const;

private:
    // ============= 核心状态 (Core State) =============
    bool is_initialized_;                       ///< 标志位：适配器是否已成功初始化。
    bool is_operational_;                       ///< 标志位：适配器当前是否可正常操作。
    std::string current_status_;                ///< 描述当前状态的字符串 (e.g., "INITIALIZING", "OPERATIONAL", "ERROR").
    std::vector<std::string> error_messages_;   ///< 存储历史错误消息的列表。

    // ============= 子功能模块实例 (Subsystem Instances) =============
    // 使用智能指针管理模块的生命周期
    std::shared_ptr<Go2MotionController> motion_controller_;        ///< 运动控制模块实例
    std::shared_ptr<Go2SensorInterface>  sensor_interface_;         ///< 传感器接口模块实例
    std::shared_ptr<Go2StateMonitor>     state_monitor_;            ///< 状态监控模块实例
    std::shared_ptr<Go2PowerManager>     power_manager_;            ///< 电源管理模块实例
    
    // ============= Go2专有辅助模块 (Go2-Specific Helpers) =============
    std::shared_ptr<Go2Communication>    communication_manager_;    ///< 通信管理模块，负责所有ROS话题的收发
    std::shared_ptr<Go2MessageConverter> message_converter_;        ///< 消息转换模块，负责在标准格式和Go2格式间转换

    // ============= 配置参数结构体 (Configuration Parameters) =============
    struct AdapterConfig {
        bool enable_motion_control   = true;    ///< 是否启用运动控制功能
        bool enable_sensor_interface = true;    ///< 是否启用传感器接口功能
        bool enable_state_monitor    = true;    ///< 是否启用状态监控功能
        bool enable_power_manager    = true;    ///< 是否启用电源管理功能
        
        double heartbeat_frequency = 10.0;      ///< 心跳检查的频率 (Hz)
        double timeout_duration    = 5.0;       ///< 通信超时时间 (秒)
        
        std::string go2_ip_address = "192.168.123.18";  ///< Go2机器人的IP地址
        int go2_port = 8080;                            ///< Go2机器人的通信端口
        
        bool debug_mode      = false;           ///< 是否开启调试模式（输出更多日志）
        bool verbose_logging = false;           ///< 是否开启详细日志记录
    } config_;

    // ============= 性能统计 (Performance Statistics) =============
    std::chrono::steady_clock::time_point start_time_;  ///< 适配器启动的时间点
    std::atomic<uint64_t> heartbeat_count_{0};       ///< 心跳计数器，原子类型保证线程安全

    // ============= 事件回调函数 (Event Callbacks) =============
    std::function<void(const std::string&)> status_callback_;   ///< 外部注册的状态变化回调函数
    std::function<void(const std::string&)> error_callback_;    ///< 外部注册的错误事件回调函数

    // ============= ROS接口对象 (ROS Interface Objects) =============
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;              ///< 心跳定时器对象
    
    // 服务服务器 (Service Servers)
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr 
        initialize_service_;                                    ///< “初始化”服务
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr 
        shutdown_service_;                                      ///< “关闭”服务
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr 
        system_check_service_;                                  ///< “系统自检”服务

    // ============= 线程同步 (Thread Synchronization) =============
    mutable std::mutex status_mutex_;          ///< 用于保护状态相关变量的互斥锁
    mutable std::mutex error_mutex_;           ///< 用于保护错误消息列表的互斥锁
    mutable std::mutex config_mutex_;          ///< 用于保护配置信息的互斥锁

    // ============= 私有回调方法 (Private Callback Methods) =============
    
    /**
     * @brief 心跳定时器的回调函数。
     */
    void heartbeatCallback();
    
    /**
     * @brief "初始化"ROS服务的处理回调。
     */
    void initializeServiceCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response
    );
    
    /**
     * @brief "关闭"ROS服务的处理回调。
     */
    void shutdownServiceCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response
    );
    
    /**
     * @brief "系统自检"ROS服务的处理回调。
     */
    void systemCheckServiceCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response
    );
};

/**
 * @brief Go2适配器智能指针类型定义
 */
using Go2AdapterPtr = std::shared_ptr<Go2Adapter>;
using Go2AdapterUniquePtr = std::unique_ptr<Go2Adapter>;

} // namespace go2_adapter
} // namespace robot_adapters

#endif // ROBOT_ADAPTERS__GO2_ADAPTER__GO2_ADAPTER_HPP_