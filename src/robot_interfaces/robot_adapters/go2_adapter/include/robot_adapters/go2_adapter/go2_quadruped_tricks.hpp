#pragma once // 防止头文件被重复包含

#include <rclcpp/rclcpp.hpp> // 引入ROS2 C++客户端库
#include <string> // 引入C++标准字符串库
#include <memory> // 引入智能指针库

// Go2机器人特定消息类型
#include "unitree_api/msg/request.hpp" // 引入Go2 API请求消息
#include "unitree_api/msg/response.hpp" // 引入Go2 API响应消息

#include "robot_base_interfaces/motion_interface/i_quadruped_tricks.hpp" // 引入四足机器人特技接口的基类定义
#include "robot_adapters/go2_adapter/go2_communication.hpp" // 引入Go2通信管理器

namespace robot_adapters {
namespace go2_adapter {

/**
 * @class Go2QuadrupedTricks
 * @brief 实现了IQuadrupedTricks接口，用于控制Go2机器人执行各种特殊动作（“特技”）。
 *
 * @details
 * 此类是Go2机器人特技功能的具体实现。它通过向特定的ROS话题发布命令来调用
 * Unitree Go2的底层API，从而执行如站立、坐下、跳舞、作揖等动作。
 *
 * 当前实现是一个初始版本，主要作为占位符，未来可以更深入地与Unitree SDK或更复杂的
 * 运动控制服务集成。
 *
 * 使用方法：
 * 1. 创建一个Go2QuadrupedTricks实例。
 * 2. 调用相应的成员函数（如 `balanceStand()`, `performDance()` 等）。
 * 3. 函数会通过ROS话题发送指令给机器人。
 */
class Go2QuadrupedTricks : public robot_base_interfaces::motion_interface::IQuadrupedTricks {
public:
    /**
     * @brief 构造函数
     * @param communication Go2通信管理器的共享指针，用于与机器人通信。
     * @param logger 一个ROS日志记录器实例，用于输出日志信息。
     */
    Go2QuadrupedTricks(std::shared_ptr<Go2Communication> communication, const rclcpp::Logger& logger);

    /**
     * @brief 析构函数 (默认)
     */
    ~Go2QuadrupedTricks() override = default;

    // --- 核心姿态控制 ---

    /**
     * @brief 控制机器人进入平衡站立姿态。
     * @return MotionResult 表示操作结果的枚举值。
     */
    robot_base_interfaces::motion_interface::MotionResult balanceStand() override;

    /**
     * @brief 控制机器人从卧姿站立起来。
     * @return MotionResult 表示操作结果的枚举值。
     */
    robot_base_interfaces::motion_interface::MotionResult standUp() override;

    /**
     * @brief 控制机器人从站姿趴下。
     * @return MotionResult 表示操作结果的枚举值。
     */
    robot_base_interfaces::motion_interface::MotionResult standDown() override;

    /**
     * @brief 控制机器人坐下。
     * @return MotionResult 表示操作结果的枚举值。
     */
    robot_base_interfaces::motion_interface::MotionResult sit() override;

    /**
     * @brief 控制机器人从异常姿态恢复到站立姿态。
     * @return MotionResult 表示操作结果的枚举值。
     */
    robot_base_interfaces::motion_interface::MotionResult recoveryStand() override;

    // --- 特殊表演动作 ---

    /**
     * @brief 控制机器人表演跳舞。
     * @param dance_type 跳舞的类型（整数，具体含义由机器人固件定义）。默认为1。
     * @return MotionResult 表示操作结果的枚举值。
     */
    robot_base_interfaces::motion_interface::MotionResult performDance(int dance_type = 1) override;

    /**
     * @brief 控制机器人执行前空翻动作。
     * @return MotionResult 表示操作结果的枚举值。
     */
    robot_base_interfaces::motion_interface::MotionResult frontFlip() override;

    /**
     * @brief 控制机器人向前跳跃。
     * @return MotionResult 表示操作结果的枚举值。
     */
    robot_base_interfaces::motion_interface::MotionResult frontJump() override;

    /**
     * @brief 控制机器人作揖（打招呼）。
     * @return MotionResult 表示操作结果的枚举值。
     */
    robot_base_interfaces::motion_interface::MotionResult hello() override;

    /**
     * @brief 控制机器人伸懒腰。
     * @return MotionResult 表示操作结果的枚举值。
     */
    robot_base_interfaces::motion_interface::MotionResult stretch() override;

    // --- 运动参数设置 ---

    /**
     * @brief 设置机器人的速度等级。
     * @param level 速度等级（整数，通常为低、中、高档）。
     * @return MotionResult 表示操作结果的枚举值。
     */
    robot_base_interfaces::motion_interface::MotionResult setSpeedLevel(int level) override;

private:
    rclcpp::Logger logger_; ///< 用于记录日志的ROS日志记录器
    std::shared_ptr<Go2Communication> communication_; ///< Go2通信管理器，用于发送API请求

    /**
     * @brief 发送Go2 API请求到机器人。
     * @param api_id 要调用的Go2 API的ID。
     * @param parameters 命令所需的参数，JSON格式字符串。默认为空字符串。
     * @return 如果命令成功发布，返回true；否则返回false。
     * @details 使用Go2Communication发送标准的unitree_api::msg::Request消息。
     */
    bool sendGo2ApiRequest(uint32_t api_id, const std::string& parameters = "");

    /**
     * @brief 等待一个动作完成的辅助函数。
     * @param duration_ms 等待的毫秒数。默认为3000ms。
     * @details
     * 这是一个简化的实现，通过固定时长的延时来“等待”动作完成。
     * 在实际应用中，应替换为基于机器人状态反馈的更可靠的完成检测机制。
     */
    void waitForCompletion(uint32_t duration_ms = 3000);

    /**
     * @brief 验证通信连接状态。
     * @return 如果通信正常，返回true；否则返回false。
     */
    bool validateConnection() const;
};

} // namespace go2_adapter
} // namespace robot_adapters