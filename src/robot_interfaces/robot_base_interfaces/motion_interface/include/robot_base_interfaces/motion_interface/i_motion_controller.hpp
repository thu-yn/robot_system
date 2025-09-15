/**
 * @file   i_motion_controller.hpp
 * @brief  机器人运动控制抽象接口 - 完全适配Go2，预留扩展
 * @author Yang Nan
 * @date   2025-09-11
 */

#ifndef ROBOT_BASE_INTERFACES__MOTION_INTERFACE__I_MOTION_CONTROLLER_HPP_
#define ROBOT_BASE_INTERFACES__MOTION_INTERFACE__I_MOTION_CONTROLLER_HPP_

#include "robot_base_interfaces/motion_interface/motion_types.hpp"
#include <memory>
#include <functional>
#include <string>
#include <cmath>

namespace robot_base_interfaces {
namespace motion_interface {

/**
 * @brief 运动控制器抽象接口
 * 
 * 该接口设计完全覆盖Go2的运动能力，包括：
 * - 基础运动控制（速度、姿态）
 * - 高级运动模式（步态、特技动作）
 * - 紧急停止和安全机制
 * - 状态查询和监控
 * 
 * 其他机器人可通过实现此接口来适配统一的导航系统
 */
class IMotionController {
public:
    virtual ~IMotionController() = default;

    // ============= 初始化和配置 =============
    
    /**
     * @brief 初始化运动控制器
     * @return 初始化结果
     */
    virtual MotionResult initialize() = 0;
    
    /**
     * @brief 关闭运动控制器
     * @return 关闭结果  
     */
    virtual MotionResult shutdown() = 0;
    
    /**
     * @brief 获取机器人运动能力
     * @return 运动能力结构体
     */
    virtual MotionCapabilities getCapabilities() const = 0;
    
    // ============= 基础运动控制 =============
    
    /**
     * @brief 设置机器人速度
     * @param velocity 目标速度
     * @return 控制结果
     */
    virtual MotionResult setVelocity(const Velocity& velocity) = 0;
    
    /**
     * @brief 设置机器人姿态
     * @param posture 目标姿态
     * @return 控制结果
     */
    virtual MotionResult setPosture(const Posture& posture) = 0;
    
    /**
     * @brief 设置机身高度
     * @param height 目标高度 (m)
     * @return 控制结果
     */
    virtual MotionResult setBodyHeight(float height) = 0;
    
    /**
     * @brief 紧急停止
     * @param level 停止级别
     * @return 停止结果
     */
    virtual MotionResult emergencyStop(EmergencyStopLevel level = EmergencyStopLevel::SOFT_STOP) = 0;
    
    // ============= 运动模式控制 =============
    
    /**
     * @brief 切换运动模式
     * @param mode 目标运动模式
     * @return 切换结果
     */
    virtual MotionResult switchMode(MotionMode mode) = 0;
    
    /**
     * @brief 设置步态类型
     * @param gait 目标步态
     * @return 设置结果
     */
    virtual MotionResult setGaitType(GaitType gait) = 0;
    
    /**
     * @brief 执行平衡站立
     * @return 执行结果
     */
    virtual MotionResult balanceStand() = 0;
    
    /**
     * @brief 执行站起动作
     * @return 执行结果
     */
    virtual MotionResult standUp() = 0;
    
    /**
     * @brief 执行趴下动作
     * @return 执行结果
     */
    virtual MotionResult standDown() = 0;
    
    /**
     * @brief 执行坐下动作
     * @return 执行结果
     */
    virtual MotionResult sit() = 0;
    
    /**
     * @brief 恢复站立
     * @return 执行结果
     */
    virtual MotionResult recoveryStand() = 0;
    
    // ============= 高级动作控制 =============
    
    /**
     * @brief 执行舞蹈动作
     * @param dance_type 舞蹈类型
     * @return 执行结果
     * @note 迁移说明：建议通过 `IQuadrupedTricks` 扩展接口实现；基础接口保留以保持兼容。
     */
    virtual MotionResult performDance(int dance_type = 1) = 0;
    
    /**
     * @brief 执行前翻
     * @return 执行结果
     * @note 迁移说明：建议通过 `IQuadrupedTricks` 扩展接口实现；基础接口保留以保持兼容。
     */
    virtual MotionResult frontFlip() = 0;
    
    /**
     * @brief 执行前跳
     * @return 执行结果
     * @note 迁移说明：建议通过 `IQuadrupedTricks` 扩展接口实现；基础接口保留以保持兼容。
     */
    virtual MotionResult frontJump() = 0;
    
    /**
     * @brief 执行打招呼动作
     * @return 执行结果
     * @note 迁移说明：建议通过 `IQuadrupedTricks` 扩展接口实现；基础接口保留以保持兼容。
     */
    virtual MotionResult hello() = 0;
    
    /**
     * @brief 伸展动作
     * @return 执行结果
     * @note 迁移说明：建议通过 `IQuadrupedTricks` 扩展接口实现；基础接口保留以保持兼容。
     */
    virtual MotionResult stretch() = 0;
    
    /**
     * @brief 设置速度等级
     * @param level 速度等级 (具体范围由实现类决定)
     * @return 设置结果
     * @note 迁移说明：建议通过 `IQuadrupedTricks` 扩展接口实现；基础接口保留以保持兼容。
     */
    virtual MotionResult setSpeedLevel(int level) = 0;
    
    // ============= 状态查询 =============
    
    /**
     * @brief 获取当前运动状态
     * @return 运动状态
     */
    virtual MotionState getMotionState() const = 0;
    
    /**
     * @brief 检查是否可以执行运动
     * @return true if operational, false otherwise
     */
    virtual bool isOperational() const = 0;
    
    /**
     * @brief 获取当前错误代码
     * @return 错误代码，0表示无错误
     */
    virtual uint32_t getErrorCode() const = 0;
    
    /**
     * @brief 检查运动命令是否完成
     * @return true if completed, false if still executing
     */
    virtual bool isMotionCompleted() const = 0;
    
    // ============= 回调和事件 =============
    
    /**
     * @brief 设置状态变化回调函数
     * @param callback 回调函数
     * 线程语义：回调可能在内部工作线程触发，调用方需确保线程安全；
     * 回调应短小无阻塞，避免在回调内进行耗时操作或再次调用控制接口造成潜在重入。
     */
    virtual void setStateCallback(std::function<void(const MotionState&)> callback) = 0;
    
    /**
     * @brief 设置错误事件回调函数  
     * @param callback 错误回调函数
     * 线程语义：可能在异步线程触发；实现应保证不会与状态回调产生死锁；
     * 建议调用方在回调中仅记录与通知，复杂处理移交到独立执行单元。
     */
    virtual void setErrorCallback(std::function<void(uint32_t error_code, const std::string& error_msg)> callback) = 0;
    
    // ============= 扩展接口 (为其他机器人预留) =============
    
    /**
     * @brief 执行自定义命令 - 扩展接口
     * @param command_name 命令名称
     * @param parameters 命令参数 (JSON格式字符串)
     * @return 执行结果
     */
    virtual MotionResult executeCustomCommand(const std::string& command_name, 
                                            const std::string& parameters = "") {
        // 默认实现：不支持自定义命令
        (void)command_name;
        (void)parameters;
        return MotionResult::CAPABILITY_LIMITED;
    }
    
    /**
     * @brief 获取机器人类型
     * @return 机器人类型
     */
    virtual RobotType getRobotType() const = 0;
    
    /**
     * @brief 获取控制器版本信息
     * @return 版本字符串
     */
    virtual std::string getVersion() const {
        return "1.0.0";
    }
    
protected:
    /**
     * @brief 验证速度参数是否在能力范围内
     * @param velocity 要验证的速度
     * @return true if valid, false otherwise
     */
    virtual bool validateVelocity(const Velocity& velocity) const {
        const auto caps = getCapabilities();
        return (std::abs(velocity.linear_x) <= caps.max_linear_velocity &&
                std::abs(velocity.linear_y) <= caps.max_lateral_velocity &&
                std::abs(velocity.angular_z) <= caps.max_angular_velocity);
    }
    
    /**
     * @brief 验证姿态参数是否在能力范围内
     * @param posture 要验证的姿态
     * @return true if valid, false otherwise
     */
    virtual bool validatePosture(const Posture& posture) const {
        const auto caps = getCapabilities();
        return (std::abs(posture.roll) <= caps.max_roll_angle &&
                std::abs(posture.pitch) <= caps.max_pitch_angle &&
                posture.body_height >= caps.min_body_height &&
                posture.body_height <= caps.max_body_height);
    }
};

/**
 * @brief 运动控制器智能指针类型定义
 */
using IMotionControllerPtr = std::shared_ptr<IMotionController>;
using IMotionControllerUniquePtr = std::unique_ptr<IMotionController>;

} // namespace motion_interface
} // namespace robot_base_interfaces

#endif // ROBOT_BASE_INTERFACES__MOTION_INTERFACE__I_MOTION_CONTROLLER_HPP_