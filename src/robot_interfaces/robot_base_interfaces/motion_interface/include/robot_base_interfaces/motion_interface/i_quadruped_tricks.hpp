/**
 * @file   i_quadruped_tricks.hpp
 * @brief  四足机器人特技/行为扩展接口（从基础运动接口拆分）
 * @author Yang Nan
 * @date   2025-09-11
 */

#ifndef ROBOT_BASE_INTERFACES__MOTION_INTERFACE__I_QUADRUPED_TRICKS_HPP_
#define ROBOT_BASE_INTERFACES__MOTION_INTERFACE__I_QUADRUPED_TRICKS_HPP_

#include "robot_base_interfaces/motion_interface/motion_types.hpp"

namespace robot_base_interfaces {
namespace motion_interface {

/**
 * @brief 四足行为扩展接口
 * 说明：面向 Go2/四足类平台的特技动作集合，基础应用可选依赖；
 * 不同机型可实现子集并通过能力描述声明支持情况。
 */
class IQuadrupedTricks {
public:
    virtual ~IQuadrupedTricks() = default;

    virtual MotionResult balanceStand() = 0;
    virtual MotionResult standUp() = 0;
    virtual MotionResult standDown() = 0;
    virtual MotionResult sit() = 0;
    virtual MotionResult recoveryStand() = 0;

    virtual MotionResult performDance(int dance_type = 1) = 0;
    virtual MotionResult frontFlip() = 0;
    virtual MotionResult frontJump() = 0;
    virtual MotionResult hello() = 0;
    virtual MotionResult stretch() = 0;
    virtual MotionResult setSpeedLevel(int level) = 0;
};

} // namespace motion_interface
} // namespace robot_base_interfaces

#endif // ROBOT_BASE_INTERFACES__MOTION_INTERFACE__I_QUADRUPED_TRICKS_HPP_


