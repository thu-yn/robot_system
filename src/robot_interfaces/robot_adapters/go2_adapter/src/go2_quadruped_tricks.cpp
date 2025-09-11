/**
 * @file go2_quadruped_tricks.cpp
 * @brief Go2机器人特技动作控制类的实现文件
 * @author Yang Nan
 * @date 2025-09-10
 *
 * @details
 * 本文件包含了 `Go2QuadrupedTricks` 类所有方法的具体实现。
 * 此类负责将高级的“特技”动作指令（如站立、跳舞、作揖等）转换为
 * 特定格式的字符串命令，并通过ROS2话题发布出去，以控制Go2机器人
 * 执行这些特殊动作。
 */

#include "robot_adapters/go2_adapter/go2_quadruped_tricks.hpp" // 引入对应的头文件
#include <std_msgs/msg/string.hpp> // 引入标准字符串消息类型，用于发布命令
#include <chrono> // C++时间库，用于获取时间戳
#include <thread> // C++线程库，用于实现延时等待

// 使用基类中定义的运动结果枚举类型
using robot_base_interfaces::motion_interface::MotionResult;

namespace robot_adapters {
namespace go2_adapter {

/**
 * @brief Go2QuadrupedTricks类的构造函数
 * @param logger 一个从外部传入的ROS日志记录器实例。
 *
 * @details
 * 在构造时，会初始化一个内部的ROS2节点 `go2_tricks_node`，并创建一个
 * 发布者，该发布者用于向 `/api/sport/request` 话题发送控制指令。
 */
Go2QuadrupedTricks::Go2QuadrupedTricks(const rclcpp::Logger& logger)
    : logger_(logger) {
    // 初始化一个专用于本功能的ROS节点
    node_ = rclcpp::Node::make_shared("go2_tricks_node");
    // 创建一个发布者，用于向Go2的API话题发送字符串命令
    command_publisher_ = node_->create_publisher<std_msgs::msg::String>(
        "/api/sport/request", 10);
}

/**
 * @brief 辅助函数，将布尔成功标志转换为MotionResult枚举
 * @param success 操作是否成功的布尔值。
 * @return 如果success为true，返回MotionResult::SUCCESS；否则返回MotionResult::UNKNOWN_ERROR。
 */
static MotionResult checkResult(bool success) {
    return success ? MotionResult::SUCCESS : MotionResult::UNKNOWN_ERROR;
}

// ============= 基础动作 (Basic Actions) =============

/**
 * @brief 执行平衡站立
 * @details 发送API ID 1001的命令，并等待2秒让机器人完成动作。
 */
MotionResult Go2QuadrupedTricks::balanceStand() {
    RCLCPP_INFO(logger_, "Executing balance stand action.");
    // 发送指令，API ID为1001
    bool success = sendGo2Command(1001, "mode=balance_stand");
    if (success) {
        waitForCompletion(2000); // 等待2000毫秒
    }
    return checkResult(success);
}

/**
 * @brief 执行站立
 * @details 发送API ID 1002的命令，并等待3秒让机器人完成动作。
 */
MotionResult Go2QuadrupedTricks::standUp() {
    RCLCPP_INFO(logger_, "Executing stand up action.");
    bool success = sendGo2Command(1002, "mode=stand_up");
    if (success) {
        waitForCompletion(3000);
    }
    return checkResult(success);
}

/**
 * @brief 执行趴下
 * @details 发送API ID 1003的命令，并等待2秒让机器人完成动作。
 */
MotionResult Go2QuadrupedTricks::standDown() {
    RCLCPP_INFO(logger_, "Executing stand down action.");
    bool success = sendGo2Command(1003, "mode=stand_down");
    if (success) {
        waitForCompletion(2000);
    }
    return checkResult(success);
}

/**
 * @brief 执行坐下
 * @details 发送API ID 1004的命令，并等待3秒让机器人完成动作。
 */
MotionResult Go2QuadrupedTricks::sit() {
    RCLCPP_INFO(logger_, "Executing sit down action.");
    bool success = sendGo2Command(1004, "mode=sit");
    if (success) {
        waitForCompletion(3000);
    }
    return checkResult(success);
}

/**
 * @brief 执行恢复站立
 * @details 从异常姿态恢复。发送API ID 1005的命令，并等待4秒。
 */
MotionResult Go2QuadrupedTricks::recoveryStand() {
    RCLCPP_INFO(logger_, "Executing recovery stand action.");
    bool success = sendGo2Command(1005, "mode=recovery_stand");
    if (success) {
        waitForCompletion(4000);
    }
    return checkResult(success);
}

// ============= 特技动作 (Special Tricks) =============

/**
 * @brief 执行跳舞
 * @param dance_type 舞蹈类型ID (1-5)。
 * @details 发送API ID 2001的命令，并等待8秒。
 */
MotionResult Go2QuadrupedTricks::performDance(int dance_type) {
    RCLCPP_INFO(logger_, "Executing dance action, type: %d", dance_type);

    // 检查舞蹈类型参数是否在有效范围内
    if (dance_type < 1 || dance_type > 5) {
        RCLCPP_WARN(logger_, "Unsupported dance type: %d. Using default type 1.", dance_type);
        dance_type = 1;
    }

    std::string params = "mode=dance;dance_type=" + std::to_string(dance_type);
    bool success = sendGo2Command(2001, params);
    if (success) {
        waitForCompletion(8000); // 舞蹈动作时间较长
    }
    return checkResult(success);
}

/**
 * @brief 执行前空翻
 * @details 发送API ID 2002的命令，并等待5秒。这是一个危险动作。
 */
MotionResult Go2QuadrupedTricks::frontFlip() {
    RCLCPP_INFO(logger_, "Executing front flip action.");
    RCLCPP_WARN(logger_, "Front flip is a dangerous action. Ensure the surrounding area is safe.");

    bool success = sendGo2Command(2002, "mode=front_flip");
    if (success) {
        waitForCompletion(5000);
    }
    return checkResult(success);
}

/**
 * @brief 执行前跳
 * @details 发送API ID 2003的命令，并等待3秒。
 */
MotionResult Go2QuadrupedTricks::frontJump() {
    RCLCPP_INFO(logger_, "Executing front jump action.");
    bool success = sendGo2Command(2003, "mode=front_jump");
    if (success) {
        waitForCompletion(3000);
    }
    return checkResult(success);
}

/**
 * @brief 执行作揖（打招呼）
 * @details 发送API ID 2004的命令，并等待4秒。
 */
MotionResult Go2QuadrupedTricks::hello() {
    RCLCPP_INFO(logger_, "Executing 'hello' (wave) action.");
    bool success = sendGo2Command(2004, "mode=hello");
    if (success) {
        waitForCompletion(4000);
    }
    return checkResult(success);
}

/**
 * @brief 执行伸懒腰
 * @details 发送API ID 2005的命令，并等待5秒。
 */
MotionResult Go2QuadrupedTricks::stretch() {
    RCLCPP_INFO(logger_, "Executing stretch action.");
    bool success = sendGo2Command(2005, "mode=stretch");
    if (success) {
        waitForCompletion(5000);
    }
    return checkResult(success);
}

// ============= 配置方法 (Configuration Methods) =============

/**
 * @brief 设置速度等级
 * @param level 速度等级 (1-9)。
 * @details 发送API ID 3001的命令。这是一个即时生效的命令，无需等待。
 */
MotionResult Go2QuadrupedTricks::setSpeedLevel(int level) {
    RCLCPP_INFO(logger_, "Setting speed level to: %d", level);

    // 检查速度等级参数是否在有效范围内
    if (level < 1 || level > 9) {
        RCLCPP_WARN(logger_, "Invalid speed level: %d. The valid range is 1-9.", level);
        return MotionResult::INVALID_PARAMETER;
    }

    std::string params = "speed_level=" + std::to_string(level);
    bool success = sendGo2Command(3001, params);
    return checkResult(success);
}

// ============= 辅助方法实现 (Helper Methods Implementation) =============

/**
 * @brief 发送格式化的命令到Go2机器人
 * @param api_id 要调用的API的ID。
 * @param parameters 附带的参数字符串。
 * @return 如果命令成功发布，返回true。
 *
 * @details
 * 此函数负责构建特定格式的命令字符串，格式为：
 * "TRICK_CMD:api_id=...;[parameters;]timestamp=..."
 * 然后通过ROS发布者将其发送出去。
 */
bool Go2QuadrupedTricks::sendGo2Command(uint32_t api_id, const std::string& parameters) {
    try {
        if (!command_publisher_) {
            RCLCPP_ERROR(logger_, "Command publisher is not initialized.");
            return false;
        }

        // 使用字符串流构建命令
        std::ostringstream command_stream;
        command_stream << "TRICK_CMD:"
                       << "api_id=" << api_id << ";";
        if (!parameters.empty()) {
            command_stream << parameters << ";";
        }
        // 添加纳秒级时间戳，用于唯一标识和调试
        command_stream << "timestamp=" << std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();

        std::string command_string = command_stream.str();

        // 创建并发布ROS消息
        auto msg = std_msgs::msg::String();
        msg.data = command_string;
        command_publisher_->publish(msg);

        RCLCPP_DEBUG(logger_, "Sent Go2 trick command: %s", command_string.c_str());
        return true;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Failed to send Go2 command: %s", e.what());
        return false;
    }
}

/**
 * @brief 等待指定时间以让动作完成
 * @param duration_ms 等待的毫秒数。
 *
 * @details
 * 这是一个简单的阻塞式延时。它以100毫秒为间隔进行分段等待。
 * 在每次等待的间隙，它会调用 `rclcpp::spin_some` 来处理ROS节点的
 * 回调，防止节点在等待期间无响应。
 *
 * 注意：这是一个简化的实现。在更复杂的系统中，应该通过订阅机器人的
 * 状态话题来判断动作是否真正完成，而不是依赖固定的延时。
 */
void Go2QuadrupedTricks::waitForCompletion(uint32_t duration_ms) {
    RCLCPP_DEBUG(logger_, "Waiting for action to complete, estimated duration: %u ms", duration_ms);

    const uint32_t check_interval = 100; // 每次检查的间隔时间（毫秒）
    uint32_t elapsed = 0;

    while (elapsed < duration_ms) {
        std::this_thread::sleep_for(std::chrono::milliseconds(check_interval));
        elapsed += check_interval;

        // 在此处可以插入检查机器人状态的逻辑，如果检测到动作完成，可以提前退出循环。
        // e.g., if (isActionDone()) { break; }

        // 保持ROS节点活跃，处理可能到来的消息
        if (node_) {
            rclcpp::spin_some(node_);
        }
    }

    RCLCPP_DEBUG(logger_, "Finished waiting for action completion.");
}

} // namespace go2_adapter
} // namespace robot_adapters