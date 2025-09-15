#ifndef ROBOT_ADAPTERS__GO2_ADAPTER__GO2_MOTION_CONTROLLER_HPP_
#define ROBOT_ADAPTERS__GO2_ADAPTER__GO2_MOTION_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <chrono>
#include <functional>
#include <mutex>

// 引入统一运动控制接口
#include "robot_base_interfaces/motion_interface/i_motion_controller.hpp"
#include "robot_base_interfaces/motion_interface/motion_types.hpp"

// ROS2标准消息类型
#include "geometry_msgs/msg/twist.hpp"

// Go2特定消息类型
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

// Go2适配器相关组件
#include "robot_adapters/go2_adapter/go2_communication.hpp"
#include "robot_adapters/go2_adapter/go2_message_converter.hpp"

// JSON处理库
#include <nlohmann/json.hpp>

namespace robot_adapters {
namespace go2_adapter {

/**
 * @class Go2MotionController
 * @brief Go2机器人运动控制器实现类
 * 
 * 该类实现了统一运动控制接口，专门用于控制宇树Go2四足机器人
 * 功能包括：
 *      - 基础运动控制（速度、姿态、高度调节）
 *      - 运动模式切换（站立、移动、趴下等）
 *      - 步态控制（小跑、奔跑、爬楼梯等）
 *      - Go2特有动作（舞蹈、翻滚、跳跃等）
 *      - 紧急停止和安全保护
 *      - 实时状态监控和反馈
 */
class Go2MotionController : public robot_base_interfaces::motion_interface::IMotionController,
                            public rclcpp::Node {
public:
    /**
     * @brief 构造函数
     * @param node_name ROS2节点名称，默认为"go2_motion_controller"
     */
    explicit Go2MotionController(const std::string& node_name = "go2_motion_controller");
    
    /**
     * @brief 析构函数，自动清理资源
     */
    ~Go2MotionController() override = default;

    // ============= IMotionController 接口实现 =============
    
    /**
     * @brief 初始化运动控制器
     * @return robot_base_interfaces::motion_interface::MotionResult 初始化结果
     * 
     * 初始化包括：
     * - 创建Go2 API请求发布器
     * - 创建SportModeState状态订阅器
     * - 设置默认运动参数
     * - 启动状态监控线程
     */
    robot_base_interfaces::motion_interface::MotionResult initialize() override;
    
    /**
     * @brief 关闭运动控制器，释放所有资源
     * @return robot_base_interfaces::motion_interface::MotionResult 关闭结果
     */
    robot_base_interfaces::motion_interface::MotionResult shutdown() override;
    
    /**
     * @brief 获取Go2机器人的运动能力参数
     * @return robot_base_interfaces::motion_interface::MotionCapabilities 运动能力结构体
     * 
     * 包含Go2的具体参数：
     * - 最大线速度: 1.5 m/s
     * - 最大角速度: 2.0 rad/s  
     * - 机身高度范围: 0.08-0.42 m
     * - 支持的运动模式和步态类型
     */
    robot_base_interfaces::motion_interface::MotionCapabilities getCapabilities() const override;
    
    /**
     * @brief 设置机器人目标速度
     * @param velocity 目标速度向量，包含线速度和角速度
     * @return robot_base_interfaces::motion_interface::MotionResult 控制结果
     * 
     * 速度控制说明：
     * - linear_x: 前进速度 (正值前进，负值后退)
     * - linear_y: 侧移速度 (正值左移，负值右移)
     * - angular_z: 偏航角速度 (正值左转，负值右转)
     * - 自动进行速度范围检查和限制
     */
    robot_base_interfaces::motion_interface::MotionResult setVelocity(
        const robot_base_interfaces::motion_interface::Velocity& velocity) override;
    
    /**
     * @brief 设置机器人姿态
     * @param posture 目标姿态，包含RPY角度和机身高度
     * @return robot_base_interfaces::motion_interface::MotionResult 控制结果
     * 
     * 姿态控制说明：
     * - roll: 滚转角 (左右倾斜)
     * - pitch: 俯仰角 (前后倾斜)  
     * - yaw: 偏航角 (水平旋转)
     * - body_height: 机身高度 (距离地面)
     */
    robot_base_interfaces::motion_interface::MotionResult setPosture(
        const robot_base_interfaces::motion_interface::Posture& posture) override;
    
    /**
     * @brief 设置机身高度
     * @param height 目标高度 (米)，范围：0.08-0.42m
     * @return robot_base_interfaces::motion_interface::MotionResult 控制结果
     */
    robot_base_interfaces::motion_interface::MotionResult setBodyHeight(float height) override;
    
    /**
     * @brief 紧急停止
     * @param level 停止级别（软停止/硬停止/断电）
     * @return robot_base_interfaces::motion_interface::MotionResult 停止结果
     */
    robot_base_interfaces::motion_interface::MotionResult emergencyStop(
        robot_base_interfaces::motion_interface::EmergencyStopLevel level) override;
    
    /**
     * @brief 切换运动模式
     * @param mode 目标运动模式
     * @return robot_base_interfaces::motion_interface::MotionResult 切换结果
     * 
     * Go2支持的模式：
     * - IDLE: 待机模式
     * - BALANCE_STAND: 平衡站立
     * - LOCOMOTION: 移动模式
     * - LIE_DOWN: 趴下
     * - SIT: 坐下
     */
    robot_base_interfaces::motion_interface::MotionResult switchMode(
        robot_base_interfaces::motion_interface::MotionMode mode) override;
    
    /**
     * @brief 设置步态类型
     * @param gait 目标步态
     * @return robot_base_interfaces::motion_interface::MotionResult 设置结果
     * 
     * Go2支持的步态：
     * - TROT: 小跑步态
     * - RUN: 快跑步态
     * - CLIMB_STAIR: 爬楼梯步态
     */
    robot_base_interfaces::motion_interface::MotionResult setGaitType(
        robot_base_interfaces::motion_interface::GaitType gait) override;
    
    // ============= Go2基本动作API =============
    
    /**
     * @brief 执行平衡站立动作 (Go2 API: 1002)
     * @return robot_base_interfaces::motion_interface::MotionResult 执行结果
     */
    robot_base_interfaces::motion_interface::MotionResult balanceStand() override;
    
    /**
     * @brief 执行站起动作 (Go2 API: 1004)
     * @return robot_base_interfaces::motion_interface::MotionResult 执行结果
     */
    robot_base_interfaces::motion_interface::MotionResult standUp() override;
    
    /**
     * @brief 执行趴下动作 (Go2 API: 1005)
     * @return robot_base_interfaces::motion_interface::MotionResult 执行结果
     */
    robot_base_interfaces::motion_interface::MotionResult standDown() override;
    
    /**
     * @brief 执行坐下动作 (Go2 API: 1009)
     * @return robot_base_interfaces::motion_interface::MotionResult 执行结果
     */
    robot_base_interfaces::motion_interface::MotionResult sit() override;
    
    /**
     * @brief 恢复站立 (Go2 API: 1006)
     * @return robot_base_interfaces::motion_interface::MotionResult 执行结果
     */
    robot_base_interfaces::motion_interface::MotionResult recoveryStand() override;
    
    // ============= Go2特有高级功能 =============
    // TODO:由于i_quadruped_tricks.hpp中专门实现了特有高级功能，建议迁移至该类方法下进行实现。
    /**
     * @brief 执行舞蹈动作 (Go2 API: 1018-1020)
     * @param dance_type 舞蹈类型 (1=Dance1, 2=Dance2)
     * @return robot_base_interfaces::motion_interface::MotionResult 执行结果
     */
    robot_base_interfaces::motion_interface::MotionResult performDance(int dance_type = 1) override;
    
    /**
     * @brief 执行前翻动作 (Go2 API: 1030)
     * @return robot_base_interfaces::motion_interface::MotionResult 执行结果
     */
    robot_base_interfaces::motion_interface::MotionResult frontFlip() override;
    
    /**
     * @brief 执行前跳动作 (Go2 API: 1031)
     * @return robot_base_interfaces::motion_interface::MotionResult 执行结果
     */
    robot_base_interfaces::motion_interface::MotionResult frontJump() override;
    
    /**
     * @brief 执行打招呼动作 (Go2 API: 1016)
     * @return robot_base_interfaces::motion_interface::MotionResult 执行结果
     */
    robot_base_interfaces::motion_interface::MotionResult hello() override;
    
    /**
     * @brief 执行伸展动作 (Go2 API: 1017)
     * @return robot_base_interfaces::motion_interface::MotionResult 执行结果
     */
    robot_base_interfaces::motion_interface::MotionResult stretch() override;
    
    /**
     * @brief 设置速度等级 (Go2 API: 1015)
     * @param level 速度等级 (1-9)
     * @return robot_base_interfaces::motion_interface::MotionResult 设置结果
     */
    robot_base_interfaces::motion_interface::MotionResult setSpeedLevel(int level) override;
    
    // ============= 状态查询接口 =============
    
    /**
     * @brief 获取当前运动状态
     * @return robot_base_interfaces::motion_interface::MotionState 运动状态
     */
    robot_base_interfaces::motion_interface::MotionState getMotionState() const override;
    
    /**
     * @brief 检查控制器是否可以正常工作
     * @return bool true表示可操作，false表示存在问题
     */
    bool isOperational() const override;
    
    /**
     * @brief 获取当前错误代码
     * @return uint32_t 错误代码，0表示无错误
     */
    uint32_t getErrorCode() const override;
    
    /**
     * @brief 检查运动命令是否执行完成
     * @return bool true表示完成，false表示正在执行
     */
    bool isMotionCompleted() const override;
    
    // ============= 回调函数设置 =============
    
    /**
     * @brief 设置状态变化回调函数
     * @param callback 状态变化时调用的回调函数
     */
    void setStateCallback(
        std::function<void(const robot_base_interfaces::motion_interface::MotionState&)> callback) override;
    
    /**
     * @brief 设置错误事件回调函数
     * @param callback 发生错误时调用的回调函数
     */
    void setErrorCallback(
        std::function<void(uint32_t error_code, const std::string& error_msg)> callback) override;
    
    /**
     * @brief 获取机器人类型
     * @return robot_base_interfaces::motion_interface::RobotType 机器人类型(GO2)
     */
    robot_base_interfaces::motion_interface::RobotType getRobotType() const override;

private:
    // ============= 私有成员变量 =============
    
    /** @brief 初始化状态标志 */
    bool is_initialized_;
    
    /** @brief 当前错误代码 */
    uint32_t current_error_code_;
    
    /** @brief 当前运动状态，线程安全访问需要加锁 */
    mutable std::mutex motion_state_mutex_;
    robot_base_interfaces::motion_interface::MotionState current_motion_state_;
    
    /** @brief 最后一次运动命令完成时间 */
    std::chrono::steady_clock::time_point last_command_time_;
    
    // ============= ROS2通信组件 =============
    
    /** @brief Go2 API请求发布器，用于发送运动控制命令 */
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr api_request_pub_;
    
    /** @brief Go2运动状态订阅器，接收机器人实时状态 */
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sport_state_sub_;
    
    /** @brief ROS2 Twist命令发布器，用于标准速度控制 */
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    /** @brief API响应订阅器，用于接收Go2命令执行结果 */
    rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr api_response_sub_;

    // ============= Go2组件 =============

    /** @brief Go2通信管理器 */
    std::shared_ptr<Go2Communication> go2_communication_;

    /** @brief Go2消息转换器 */
    std::shared_ptr<Go2MessageConverter> go2_converter_;

    // ============= 回调函数存储 =============
    
    /** @brief 状态变化回调函数 */
    std::function<void(const robot_base_interfaces::motion_interface::MotionState&)> state_callback_;
    
    /** @brief 错误事件回调函数 */
    std::function<void(uint32_t, const std::string&)> error_callback_;
    
    // ============= 私有方法声明 =============
    
    /**
     * @brief 初始化ROS2发布器和订阅器
     */
    void initializeROS2Communications();
    
    /**
     * @brief Go2运动状态回调函数
     * @param msg 接收到的SportModeState消息
     * 
     * 处理Go2机器人发布的实时状态信息，包括：
     * - 当前运动模式和步态
     * - 位置、姿态、速度信息
     * - 足端力和位置
     * - 错误代码和系统状态
     */
    void sportModeStateCallback(const unitree_go::msg::SportModeState::SharedPtr msg);
    
    /**
     * @brief 发送Go2 API命令（基于SportClient模式）
     * @param api_id Go2 API命令ID
     * @param json_params JSON格式的参数
     * @return bool 发送成功返回true
     *
     * 封装Go2 API请求的发送过程，包括：
     * - 构造unitree_api::msg::Request消息
     * - 设置正确的请求头信息
     * - 使用JSON格式参数
     * - 发布到Go2 API话题
     */
    bool sendGo2ApiCommand(uint32_t api_id, const nlohmann::json& json_params = nlohmann::json{});

    /**
     * @brief API响应回调函数
     * @param msg 接收到的API响应消息
     *
     * 处理Go2机器人的API命令执行结果：
     * - 检查命令执行状态
     * - 处理错误代码
     * - 触发相应的错误回调
     */
    void apiResponseCallback(const unitree_api::msg::Response::SharedPtr msg);
    
    /**
     * @brief 发送速度控制命令
     * @param vx 前进速度 (m/s)
     * @param vy 侧移速度 (m/s)
     * @param wz 偏航角速度 (rad/s)
     * @return bool 发送成功返回true
     */
    bool sendVelocityCommand(float vx, float vy, float wz);
    
    /**
     * @brief 发送姿态控制命令
     * @param roll 滚转角 (rad)
     * @param pitch 俯仰角 (rad)
     * @param yaw 偏航角 (rad)
     * @param body_height 机身高度 (m)
     * @return bool 发送成功返回true
     */
    bool sendPostureCommand(float roll, float pitch, float yaw, float body_height);
    
    /**
     * @brief 验证速度参数是否在安全范围内
     * @param velocity 待验证的速度向量
     * @return bool 参数有效返回true
     */
    bool validateVelocityLimits(const robot_base_interfaces::motion_interface::Velocity& velocity) const;
    
    /**
     * @brief 验证姿态参数是否在安全范围内
     * @param posture 待验证的姿态
     * @return bool 参数有效返回true
     */
    bool validatePostureLimits(const robot_base_interfaces::motion_interface::Posture& posture) const;
    
    /**
     * @brief 将Go2 SportModeState转换为统一运动状态格式
     * @param sport_state Go2原生状态消息
     * @return robot_base_interfaces::motion_interface::MotionState 统一格式的运动状态
     */
    robot_base_interfaces::motion_interface::MotionState convertSportStateToMotionState(
        const unitree_go::msg::SportModeState::SharedPtr& sport_state) const;
    
    /**
     * @brief 触发状态变化回调
     * @param new_state 新的运动状态
     */
    void triggerStateCallback(const robot_base_interfaces::motion_interface::MotionState& new_state);
    
    /**
     * @brief 触发错误回调
     * @param error_code 错误代码
     * @param error_msg 错误消息
     */
    void triggerErrorCallback(uint32_t error_code, const std::string& error_msg);
    
    // ============= Go2专用常量定义 =============

    /** @brief Go2最大线速度 (m/s) */
    static constexpr float MAX_LINEAR_VELOCITY = 1.5f;

    /** @brief Go2最大角速度 (rad/s) */
    static constexpr float MAX_ANGULAR_VELOCITY = 2.0f;

    /** @brief Go2最大侧移速度 (m/s) */
    static constexpr float MAX_LATERAL_VELOCITY = 0.8f;

    /** @brief Go2最大滚转角 (rad) */
    static constexpr float MAX_ROLL_ANGLE = 0.4f;

    /** @brief Go2最大俯仰角 (rad) */
    static constexpr float MAX_PITCH_ANGLE = 0.4f;

    /** @brief Go2最小机身高度 (m) */
    static constexpr float MIN_BODY_HEIGHT = 0.08f;

    /** @brief Go2最大机身高度 (m) */
    static constexpr float MAX_BODY_HEIGHT = 0.42f;

    /** @brief Go2默认机身高度 (m) */
    static constexpr float DEFAULT_BODY_HEIGHT = 0.32f;

    /** @brief 命令超时时间 (毫秒) */
    static constexpr int COMMAND_TIMEOUT_MS = 5000;

    // ============= Go2基本控制API ID常量（基于通信指南） =============

    static constexpr uint32_t API_ID_DAMP = 1001;               // 阻尼模式
    static constexpr uint32_t API_ID_BALANCE_STAND = 1002;      // 平衡站立
    static constexpr uint32_t API_ID_STOP_MOVE = 1003;          // 停止移动
    static constexpr uint32_t API_ID_STAND_UP = 1004;           // 站起
    static constexpr uint32_t API_ID_STAND_DOWN = 1005;         // 趴下
    static constexpr uint32_t API_ID_RECOVERY_STAND = 1006;     // 恢复站立
    static constexpr uint32_t API_ID_EULER = 1007;              // 欧拉角控制
    static constexpr uint32_t API_ID_MOVE = 1008;               // 运动控制
    static constexpr uint32_t API_ID_SIT = 1009;                // 坐下
    static constexpr uint32_t API_ID_RISE_SIT = 1010;           // 从坐姿起立
    static constexpr uint32_t API_ID_SPEED_LEVEL = 1015;        // 速度等级设置
};

} // namespace go2_adapter
} // namespace robot_adapters
#endif //ROBOT_ADAPTERS__GO2_ADAPTER__GO2_MOTION_CONTROLLER_HPP_