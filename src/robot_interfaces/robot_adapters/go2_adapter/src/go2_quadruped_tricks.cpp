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
#include <chrono> // C++时间库，用于获取时间戳
#include <thread> // C++线程库，用于实现延时等待
#include <sstream> // C++字符串流库，用于构建JSON参数

// 使用基类中定义的运动结果枚举类型
using robot_base_interfaces::motion_interface::MotionResult;

namespace robot_adapters {
namespace go2_adapter {

/**
 * @brief Go2QuadrupedTricks类的构造函数
 * @param communication Go2通信管理器的共享指针，用于与机器人通信。
 * @param logger 一个从外部传入的ROS日志记录器实例。
 *
 * @details
 * 使用传入的Go2Communication实例来发送API请求到机器人。
 */
Go2QuadrupedTricks::Go2QuadrupedTricks(std::shared_ptr<Go2Communication> communication, const rclcpp::Logger& logger)
    : logger_(logger), communication_(communication) {
    if (!communication_) {
        RCLCPP_ERROR(logger_, "Go2Communication is null! Tricks functionality will be disabled.");
    } else {
        RCLCPP_INFO(logger_, "Go2QuadrupedTricks initialized with Go2Communication.");
    }
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
 * @details 发送API ID 1002的命令，并等待2秒让机器人完成动作。
 */
MotionResult Go2QuadrupedTricks::balanceStand() {
    RCLCPP_INFO(logger_, "Executing balance stand action.");
    // 发送指令，API ID为1002（根据文档）
    bool success = sendGo2ApiRequest(1002);
    if (success) {
        waitForCompletion(2000); // 等待2000毫秒
    }
    return checkResult(success);
}

/**
 * @brief 执行站立
 * @details 发送API ID 1004的命令，并等待3秒让机器人完成动作。
 */
MotionResult Go2QuadrupedTricks::standUp() {
    RCLCPP_INFO(logger_, "Executing stand up action.");
    bool success = sendGo2ApiRequest(1004); // API ID 1004 for StandUp
    if (success) {
        waitForCompletion(3000);
    }
    return checkResult(success);
}

/**
 * @brief 执行趴下
 * @details 发送API ID 1005的命令，并等待2秒让机器人完成动作。
 */
MotionResult Go2QuadrupedTricks::standDown() {
    RCLCPP_INFO(logger_, "Executing stand down action.");
    bool success = sendGo2ApiRequest(1005); // API ID 1005 for StandDown
    if (success) {
        waitForCompletion(2000);
    }
    return checkResult(success);
}

/**
 * @brief 执行坐下
 * @details 发送API ID 1009的命令，并等待3秒让机器人完成动作。
 */
MotionResult Go2QuadrupedTricks::sit() {
    RCLCPP_INFO(logger_, "Executing sit down action.");
    bool success = sendGo2ApiRequest(1009); // API ID 1009 for Sit
    if (success) {
        waitForCompletion(3000);
    }
    return checkResult(success);
}

/**
 * @brief 执行恢复站立
 * @details 从异常姿态恢复。发送API ID 1006的命令，并等待4秒。
 */
MotionResult Go2QuadrupedTricks::recoveryStand() {
    RCLCPP_INFO(logger_, "Executing recovery stand action.");
    bool success = sendGo2ApiRequest(1006); // API ID 1006 for RecoveryStand
    if (success) {
        waitForCompletion(4000);
    }
    return checkResult(success);
}

// ============= 特技动作 (Special Tricks) =============

/**
 * @brief 执行跳舞
 * @param dance_type 舞蹈类型ID (1，2)。
 * @details 发送API ID 2001的命令，并等待8秒。
 */
MotionResult Go2QuadrupedTricks::performDance(int dance_type) {
    RCLCPP_INFO(logger_, "Executing dance action, type: %d", dance_type);

    // 检查舞蹈类型参数是否在有效范围内
    if (dance_type < 1 || dance_type > 2) {
        RCLCPP_WARN(logger_, "Unsupported dance type: %d. Using default type 1.", dance_type);
        dance_type = 1;
    }

    // 根据舞蹈类型选择对应的API ID
    uint32_t api_id;
    if (dance_type == 1) {
        api_id = 1022; // Dance1
    } else if (dance_type == 2) {
        api_id = 1023; // Dance2
    } else {
        RCLCPP_WARN(logger_, "Unsupported dance type: %d. Using dance type 1.", dance_type);
        api_id = 1022; // 默认使用Dance1
    }

    bool success = sendGo2ApiRequest(api_id);
    if (success) {
        waitForCompletion(8000); // 舞蹈动作时间较长
    }
    return checkResult(success);
}

/**
 * @brief 执行前空翻
 * @details 发送API ID 1030的命令，并等待5秒。这是一个危险动作。
 */
MotionResult Go2QuadrupedTricks::frontFlip() {
    RCLCPP_INFO(logger_, "Executing front flip action.");
    RCLCPP_WARN(logger_, "Front flip is a dangerous action. Ensure the surrounding area is safe.");

    bool success = sendGo2ApiRequest(1030); // API ID 1030 for FrontFlip
    if (success) {
        waitForCompletion(5000);
    }
    return checkResult(success);
}

/**
 * @brief 执行前跳
 * @details 发送API ID 1031的命令，并等待3秒。
 */
MotionResult Go2QuadrupedTricks::frontJump() {
    RCLCPP_INFO(logger_, "Executing front jump action.");
    bool success = sendGo2ApiRequest(1031); // API ID 1031 for FrontJump
    if (success) {
        waitForCompletion(3000);
    }
    return checkResult(success);
}

/**
 * @brief 执行作揖（打招呼）
 * @details 发送API ID 1016的命令，并等待4秒。
 */
MotionResult Go2QuadrupedTricks::hello() {
    RCLCPP_INFO(logger_, "Executing 'hello' (wave) action.");
    bool success = sendGo2ApiRequest(1016); // API ID 1016 for Hello
    if (success) {
        waitForCompletion(4000);
    }
    return checkResult(success);
}

/**
 * @brief 执行伸懒腰
 * @details 发送API ID 1017的命令，并等待5秒。
 */
MotionResult Go2QuadrupedTricks::stretch() {
    RCLCPP_INFO(logger_, "Executing stretch action.");
    bool success = sendGo2ApiRequest(1017); // API ID 1017 for Stretch
    if (success) {
        waitForCompletion(5000);
    }
    return checkResult(success);
}

// ============= 配置方法 (Configuration Methods) =============

/**
 * @brief 设置速度等级
 * @param level 速度等级 (1-5)。
 * @details 发送API ID 1010的命令。这是一个即时生效的命令，无需等待。
 */
MotionResult Go2QuadrupedTricks::setSpeedLevel(int level) {
    RCLCPP_INFO(logger_, "Setting speed level to: %d", level);

    // 检查速度等级参数是否在有效范围内
    if (level < 1 || level > 5) {
        RCLCPP_WARN(logger_, "Invalid speed level: %d. The valid range is 1-9.", level);
        return MotionResult::INVALID_PARAMETER;
    }

    // 构建JSON参数
    std::ostringstream params;
    params << "{\"data\":" << level << "}";
    bool success = sendGo2ApiRequest(1010, params.str()); // API ID 1010 for SpeedLevel
    return checkResult(success);
}

// ============= 辅助方法实现 (Helper Methods Implementation) =============

/**
 * @brief 发送Go2 API请求到机器人
 * @param api_id 要调用的Go2 API的ID。
 * @param parameters 命令所需的参数，JSON格式字符串。默认为空字符串。
 * @return 如果命令成功发布，返回true；否则返回false。
 * @details 使用Go2Communication发送标准的unitree_api::msg::Request消息。
 */
bool Go2QuadrupedTricks::sendGo2ApiRequest(uint32_t api_id, const std::string& parameters) {
    try {
        if (!validateConnection()) {
            RCLCPP_ERROR(logger_, "Go2 communication is not available.");
            return false;
        }

        // 创建 Go2 API 请求
        unitree_api::msg::Request request;
        request.header.identity.api_id = api_id;

        if (!parameters.empty()) {
            request.parameter = parameters;
        }

        // 通过 Go2Communication 发送请求
        bool success = communication_->sendApiRequest(request);

        if (success) {
            RCLCPP_DEBUG(logger_, "Sent Go2 API request: api_id=%u, params=%s", api_id, parameters.c_str());
        } else {
            RCLCPP_ERROR(logger_, "Failed to send Go2 API request: api_id=%u", api_id);
        }

        return success;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Exception while sending Go2 API request: %s", e.what());
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
    }

    RCLCPP_DEBUG(logger_, "Finished waiting for action completion.");
}

/**
 * @brief 验证通信连接状态
 * @return 如果通信正常，返回true；否则返回false
 */
bool Go2QuadrupedTricks::validateConnection() const {
    if (!communication_) {
        RCLCPP_ERROR(logger_, "Go2Communication is null.");
        return false;
    }

    if (!communication_->isInitialized()) {
        RCLCPP_ERROR(logger_, "Go2Communication is not initialized.");
        return false;
    }

    if (!communication_->isConnected()) {
        RCLCPP_WARN(logger_, "Go2 robot is not connected.");
        return false;
    }

    return true;
}

} // namespace go2_adapter
} // namespace robot_adapters