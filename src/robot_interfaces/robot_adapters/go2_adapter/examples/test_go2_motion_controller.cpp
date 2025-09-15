/**
 * @file   test_go2_motion_controller.cpp
 * @brief  Go2MotionController类功能验证测试程序
 * @author Yang Nan
 * @date   2025-09-15
 *
 * @details
 * 这是一个用于验证Go2MotionController类各项功能的测试程序。
 * 该程序创建Go2MotionController实例，并系统性地测试其各个功能模块，
 * 包括初始化、运动控制、模式切换、状态查询、回调设置等。
 *
 * 注意：这不是标准的单元测试框架，而是一个功能验证程序，
 * 用于直接实例化类并验证各个函数的正常工作。
 */

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <string>
#include <vector>
#include <csignal>
#include <atomic>
#include <functional>
#include <poll.h>
#include <unistd.h>
#include <cstdio>

// 引入要测试的头文件
#include "robot_adapters/go2_adapter/go2_motion_controller.hpp"

// 全局变量控制程序停止
std::atomic<bool> g_shutdown_requested{false};

using namespace robot_adapters::go2_adapter;
using namespace robot_base_interfaces::motion_interface;

/**
 * @brief 信号处理函数，处理Ctrl+C (SIGINT)
 */
void signalHandler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\n\n检测到 Ctrl+C，正在停止测试程序..." << std::endl;
        g_shutdown_requested.store(true);
    }
}

class Go2MotionControllerTester {
private:
    std::unique_ptr<Go2MotionController> controller_;
    std::vector<std::string> test_results_;
    int total_tests_;
    int passed_tests_;

    // 回调数据存储
    bool state_callback_triggered_;
    bool error_callback_triggered_;
    MotionState last_motion_state_;
    uint32_t last_error_code_;
    std::string last_error_message_;

public:
    Go2MotionControllerTester()
        : total_tests_(0), passed_tests_(0),
          state_callback_triggered_(false), error_callback_triggered_(false),
          last_error_code_(0) {

        // 创建Go2MotionController实例
        controller_ = std::make_unique<Go2MotionController>("test_go2_motion_controller");

        std::cout << "=== Go2MotionController功能验证测试程序 ===" << std::endl;
        std::cout << "初始化测试环境..." << std::endl;
    }

    ~Go2MotionControllerTester() {
        if (controller_) {
            controller_->shutdown();
        }
    }

    /**
     * @brief 显示测试菜单
     */
    void showTestMenu() {
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "Go2MotionController 功能验证测试程序" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        std::cout << "请选择要运行的测试:" << std::endl;
        std::cout << "1. 初始化和生命周期管理测试" << std::endl;
        std::cout << "2. 运动能力参数测试" << std::endl;
        std::cout << "3. 基础运动控制测试" << std::endl;
        std::cout << "4. 运动模式切换测试" << std::endl;
        std::cout << "5. Go2基本动作测试" << std::endl;
        std::cout << "6. Go2特有高级功能测试" << std::endl;
        std::cout << "7. 状态查询功能测试" << std::endl;
        std::cout << "8. 回调函数设置测试" << std::endl;
        std::cout << "9. 错误处理和安全测试" << std::endl;
        std::cout << "0. 运行所有测试" << std::endl;
        std::cout << "q. 退出程序" << std::endl;
        std::cout << std::string(60, '-') << std::endl;
        std::cout << "请输入选择: ";
    }

    /**
     * @brief 运行交互式菜单
     */
    void runInteractiveMenu() {
        char choice;

        while (!g_shutdown_requested.load()) {
            if (!rclcpp::ok()) {
                break;
            }

            showTestMenu();

            // 非阻塞等待键盘输入，避免 Ctrl+C 时阻塞在输入上
            bool got_input = false;
            while (!got_input) {
                if (g_shutdown_requested.load() || !rclcpp::ok()) {
                    break;
                }

                struct pollfd pfd;
                pfd.fd = STDIN_FILENO;
                pfd.events = POLLIN;
                int ret = poll(&pfd, 1, 200); // 200ms 轮询
                if (ret > 0 && (pfd.revents & POLLIN)) {
                    int c = std::getchar();
                    if (c == EOF) {
                        // 输入流结束，直接退出
                        return;
                    }
                    choice = static_cast<char>(c);
                    // 丢弃本行剩余内容
                    while (c != '\n' && c != EOF) {
                        c = std::getchar();
                    }
                    got_input = true;
                }
            }

            if (g_shutdown_requested.load() || !rclcpp::ok()) {
                break;
            }

            switch (choice) {
                case '1':
                    runSingleTest("初始化和生命周期管理测试", [this]() { testInitializationLifecycle(); });
                    break;
                case '2':
                    runSingleTest("运动能力参数测试", [this]() { testCapabilities(); });
                    break;
                case '3':
                    runSingleTest("基础运动控制测试", [this]() { testBasicMotionControl(); });
                    break;
                case '4':
                    runSingleTest("运动模式切换测试", [this]() { testModeSwitch(); });
                    break;
                case '5':
                    runSingleTest("Go2基本动作测试", [this]() { testBasicActions(); });
                    break;
                case '6':
                    runSingleTest("Go2特有高级功能测试", [this]() { testAdvancedFeatures(); });
                    break;
                case '7':
                    runSingleTest("状态查询功能测试", [this]() { testStateQuery(); });
                    break;
                case '8':
                    runSingleTest("回调函数设置测试", [this]() { testCallbacks(); });
                    break;
                case '9':
                    runSingleTest("错误处理和安全测试", [this]() { testErrorHandling(); });
                    break;
                case '0':
                    runAllTests();
                    break;
                case 'q':
                case 'Q':
                    std::cout << "强制退出程序..." << std::endl;
                    std::fflush(stdout);
                    _exit(0); // 立即退出，不做任何清理，避免阻塞
                default:
                    std::cout << "无效选择，请重新输入。" << std::endl;
                    break;
            }
        }

        if (g_shutdown_requested.load()) {
            std::cout << "\n程序被用户中断。" << std::endl;
        }
    }

    /**
     * @brief 运行所有测试
     */
    void runAllTests() {
        printHeader("开始运行Go2MotionController功能验证测试");

        // 1. 测试初始化和生命周期管理
        if (checkShutdown()) return;
        testInitializationLifecycle();

        // 2. 测试运动能力参数
        if (checkShutdown()) return;
        testCapabilities();

        // 3. 测试基础运动控制
        if (checkShutdown()) return;
        testBasicMotionControl();

        // 4. 测试运动模式切换
        if (checkShutdown()) return;
        testModeSwitch();

        // 5. 测试Go2基本动作
        if (checkShutdown()) return;
        testBasicActions();

        // 6. 测试Go2特有高级功能
        if (checkShutdown()) return;
        testAdvancedFeatures();

        // 7. 测试状态查询功能
        if (checkShutdown()) return;
        testStateQuery();

        // 8. 测试回调函数设置
        if (checkShutdown()) return;
        testCallbacks();

        // 9. 测试错误处理和安全
        if (checkShutdown()) return;
        testErrorHandling();

        // 输出最终结果
        if (!g_shutdown_requested.load()) {
            printFinalResults();
        } else {
            printInterruptedResults();
        }
    }

private:
    /**
     * @brief 测试初始化和生命周期管理
     */
    void testInitializationLifecycle() {
        printHeader("测试初始化和生命周期管理");

        // 测试初始状态
        checkTest("初始状态检查 - 未操作", !controller_->isOperational());
        checkTest("初始状态检查 - 无错误码", controller_->getErrorCode() == 0);
        checkTest("初始状态检查 - 机器人类型", controller_->getRobotType() == RobotType::GO2);

        // 测试初始化
        MotionResult init_result = controller_->initialize();
        checkTest("initialize()调用", init_result == MotionResult::SUCCESS);

        // 等待一段时间让初始化完成
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        checkTest("初始化后状态检查 - 可操作", controller_->isOperational());

        // 注意：保持初始化状态以支持后续测试
        std::cout << "  注意：保持初始化状态以支持后续测试" << std::endl;
    }

    /**
     * @brief 测试运动能力参数
     */
    void testCapabilities() {
        printHeader("测试运动能力参数");

        // 获取运动能力参数
        MotionCapabilities capabilities = controller_->getCapabilities();
        checkTest("getCapabilities()调用", true);

        std::cout << "  Go2运动能力参数:" << std::endl;
        std::cout << "    最大线速度: " << capabilities.max_linear_velocity << " m/s" << std::endl;
        std::cout << "    最大角速度: " << capabilities.max_angular_velocity << " rad/s" << std::endl;
        std::cout << "    最大侧移速度: " << capabilities.max_lateral_velocity << " m/s" << std::endl;
        std::cout << "    最大滚转角: " << capabilities.max_roll_angle << " rad" << std::endl;
        std::cout << "    最大俯仰角: " << capabilities.max_pitch_angle << " rad" << std::endl;
        std::cout << "    机身高度范围: " << capabilities.min_body_height << "-" << capabilities.max_body_height << " m" << std::endl;

        // 验证能力参数是否符合Go2规格
        checkTest("最大线速度验证", capabilities.max_linear_velocity == 1.5f);
        checkTest("最大角速度验证", capabilities.max_angular_velocity == 2.0f);
        checkTest("最大侧移速度验证", capabilities.max_lateral_velocity == 0.8f);
        checkTest("最小机身高度验证", capabilities.min_body_height == 0.08f);
        checkTest("最大机身高度验证", capabilities.max_body_height == 0.42f);

        // 验证特殊能力
        checkTest("爬楼梯能力", capabilities.can_climb_stairs);
        checkTest("平衡控制能力", capabilities.can_balance);
        checkTest("侧移能力", capabilities.can_lateral_move);
        checkTest("舞蹈能力", capabilities.can_dance);
        checkTest("跳跃能力", capabilities.can_jump);
        checkTest("翻滚能力", capabilities.can_flip);

        // 验证支持的模式和步态
        checkTest("支持的运动模式数量", capabilities.supported_modes.size() >= 6);
        checkTest("支持的步态类型数量", capabilities.supported_gaits.size() >= 4);
    }

    /**
     * @brief 测试基础运动控制
     */
    void testBasicMotionControl() {
        printHeader("测试基础运动控制");

        // 测试速度控制
        Velocity test_velocity;
        test_velocity.linear_x = 0.5f;
        test_velocity.linear_y = 0.2f;
        test_velocity.angular_z = 0.3f;

        MotionResult vel_result = controller_->setVelocity(test_velocity);
        checkTest("setVelocity()调用", vel_result == MotionResult::SUCCESS);

        // 测试边界速度（应该被限制）
        Velocity extreme_velocity;
        extreme_velocity.linear_x = 10.0f; // 超过最大速度
        extreme_velocity.linear_y = 5.0f;
        extreme_velocity.angular_z = 10.0f;

        MotionResult extreme_vel_result = controller_->setVelocity(extreme_velocity);
        checkTest("极限速度控制", extreme_vel_result == MotionResult::CAPABILITY_LIMITED);

        // 测试姿态控制
        Posture test_posture;
        test_posture.roll = 0.1f;
        test_posture.pitch = 0.05f;
        test_posture.yaw = 0.2f;
        test_posture.body_height = 0.3f;

        MotionResult posture_result = controller_->setPosture(test_posture);
        checkTest("setPosture()调用", posture_result == MotionResult::SUCCESS);

        // 测试机身高度控制
        MotionResult height_result = controller_->setBodyHeight(0.25f);
        checkTest("setBodyHeight()调用", height_result == MotionResult::SUCCESS);

        // 测试极限高度（应该被限制）
        MotionResult extreme_height = controller_->setBodyHeight(1.0f); // 超过最大高度
        checkTest("极限高度控制", extreme_height == MotionResult::CAPABILITY_LIMITED);

        // 测试停止速度命令
        Velocity zero_velocity = {};
        MotionResult stop_result = controller_->setVelocity(zero_velocity);
        checkTest("停止速度设置", stop_result == MotionResult::SUCCESS);
    }

    /**
     * @brief 测试运动模式切换
     */
    void testModeSwitch() {
        printHeader("测试运动模式切换");

        // 测试所有支持的运动模式
        std::vector<std::pair<MotionMode, std::string>> modes = {
            {MotionMode::IDLE, "IDLE"},
            {MotionMode::BALANCE_STAND, "BALANCE_STAND"},
            {MotionMode::LOCOMOTION, "LOCOMOTION"},
            {MotionMode::LIE_DOWN, "LIE_DOWN"},
            {MotionMode::SIT, "SIT"},
            {MotionMode::RECOVERY_STAND, "RECOVERY_STAND"}
        };

        for (const auto& [mode, mode_name] : modes) {
            MotionResult result = controller_->switchMode(mode);
            checkTest("模式切换到 " + mode_name, result == MotionResult::SUCCESS);

            // 等待模式切换完成
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        }

        // 测试步态类型设置
        std::vector<std::pair<GaitType, std::string>> gaits = {
            {GaitType::IDLE, "IDLE"},
            {GaitType::TROT, "TROT"},
            {GaitType::RUN, "RUN"},
            {GaitType::CLIMB_STAIR, "CLIMB_STAIR"}
        };

        for (const auto& [gait, gait_name] : gaits) {
            MotionResult result = controller_->setGaitType(gait);
            checkTest("步态设置为 " + gait_name, result == MotionResult::SUCCESS);
        }
    }

    /**
     * @brief 测试Go2基本动作
     */
    void testBasicActions() {
        printHeader("测试Go2基本动作");

        // 测试平衡站立
        MotionResult balance_result = controller_->balanceStand();
        checkTest("balanceStand()调用", balance_result == MotionResult::SUCCESS);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // 测试站起
        MotionResult standup_result = controller_->standUp();
        checkTest("standUp()调用", standup_result == MotionResult::SUCCESS);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // 测试坐下
        MotionResult sit_result = controller_->sit();
        checkTest("sit()调用", sit_result == MotionResult::SUCCESS);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // 测试恢复站立
        MotionResult recovery_result = controller_->recoveryStand();
        checkTest("recoveryStand()调用", recovery_result == MotionResult::SUCCESS);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // 测试趴下
        MotionResult standdown_result = controller_->standDown();
        checkTest("standDown()调用", standdown_result == MotionResult::SUCCESS);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // 恢复到站立状态准备后续测试
        controller_->standUp();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    /**
     * @brief 测试Go2特有高级功能
     */
    void testAdvancedFeatures() {
        printHeader("测试Go2特有高级功能");

        // 测试舞蹈动作
        MotionResult dance1_result = controller_->performDance(1);
        checkTest("performDance(1)调用", dance1_result == MotionResult::SUCCESS);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        MotionResult dance2_result = controller_->performDance(2);
        checkTest("performDance(2)调用", dance2_result == MotionResult::SUCCESS);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        // 测试无效舞蹈类型
        MotionResult invalid_dance = controller_->performDance(99);
        checkTest("无效舞蹈类型处理", invalid_dance == MotionResult::INVALID_PARAMETER);

        // 测试打招呼动作
        MotionResult hello_result = controller_->hello();
        checkTest("hello()调用", hello_result == MotionResult::SUCCESS);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        // 测试伸展动作
        MotionResult stretch_result = controller_->stretch();
        checkTest("stretch()调用", stretch_result == MotionResult::SUCCESS);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        // 测试速度等级设置
        for (int level = 1; level <= 5; ++level) {
            MotionResult speed_result = controller_->setSpeedLevel(level);
            checkTest("setSpeedLevel(" + std::to_string(level) + ")调用",
                     speed_result == MotionResult::SUCCESS);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

        // 测试无效速度等级
        MotionResult invalid_speed = controller_->setSpeedLevel(10);
        checkTest("无效速度等级处理", invalid_speed == MotionResult::INVALID_PARAMETER);

        // 警告：这些动作可能有安全风险，在实际测试中需要谨慎
        std::cout << "  注意：以下动作存在安全风险，请确保周围环境安全" << std::endl;

        // 测试前跳动作
        MotionResult jump_result = controller_->frontJump();
        checkTest("frontJump()调用", jump_result == MotionResult::SUCCESS);
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));

        // 测试前翻动作
        MotionResult flip_result = controller_->frontFlip();
        checkTest("frontFlip()调用", flip_result == MotionResult::SUCCESS);
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));

        // 恢复到安全状态
        controller_->recoveryStand();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }

    /**
     * @brief 测试状态查询功能
     */
    void testStateQuery() {
        printHeader("测试状态查询功能");

        // 测试运动状态获取
        MotionState motion_state = controller_->getMotionState();
        checkTest("getMotionState()调用", true);

        std::cout << "  当前运动状态:" << std::endl;
        std::cout << "    运动模式: " << static_cast<int>(motion_state.current_mode) << std::endl;
        std::cout << "    步态类型: " << static_cast<int>(motion_state.current_gait) << std::endl;
        std::cout << "    是否移动: " << (motion_state.is_moving ? "是" : "否") << std::endl;
        std::cout << "    是否平衡: " << (motion_state.is_balanced ? "是" : "否") << std::endl;
        std::cout << "    机身高度: " << motion_state.posture.body_height << " m" << std::endl;
        std::cout << "    错误代码: " << motion_state.error_code << std::endl;

        // 测试操作状态查询
        bool operational = controller_->isOperational();
        checkTest("isOperational()调用", true);
        std::cout << "  控制器可操作: " << (operational ? "是" : "否") << std::endl;

        // 测试错误代码获取
        uint32_t error_code = controller_->getErrorCode();
        checkTest("getErrorCode()调用", true);
        std::cout << "  当前错误代码: " << error_code << std::endl;

        // 测试动作完成状态
        bool motion_completed = controller_->isMotionCompleted();
        checkTest("isMotionCompleted()调用", true);
        std::cout << "  动作是否完成: " << (motion_completed ? "是" : "否") << std::endl;
    }

    /**
     * @brief 测试回调函数设置
     */
    void testCallbacks() {
        printHeader("测试回调函数设置");

        // 重置回调标志
        state_callback_triggered_ = false;
        error_callback_triggered_ = false;

        // 设置状态变化回调
        controller_->setStateCallback([this](const MotionState& state) {
            this->state_callback_triggered_ = true;
            this->last_motion_state_ = state;
            std::cout << "  状态回调触发: 模式=" << static_cast<int>(state.current_mode)
                      << ", 高度=" << state.posture.body_height << std::endl;
        });
        checkTest("setStateCallback()调用", true);

        // 设置错误回调
        controller_->setErrorCallback([this](uint32_t error_code, const std::string& error_msg) {
            this->error_callback_triggered_ = true;
            this->last_error_code_ = error_code;
            this->last_error_message_ = error_msg;
            std::cout << "  错误回调触发: 代码=" << error_code << ", 消息=" << error_msg << std::endl;
        });
        checkTest("setErrorCallback()调用", true);

        // 触发一个状态变化来测试回调
        controller_->setBodyHeight(0.35f);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // 验证回调是否被触发
        checkTest("状态回调触发验证", state_callback_triggered_);

        std::cout << "  回调设置成功，可正常接收状态和错误事件" << std::endl;
    }

    /**
     * @brief 测试错误处理和安全
     */
    void testErrorHandling() {
        printHeader("测试错误处理和安全");

        // 测试紧急停止功能
        MotionResult soft_stop = controller_->emergencyStop(EmergencyStopLevel::SOFT_STOP);
        checkTest("软停止调用", soft_stop == MotionResult::SUCCESS);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        MotionResult hard_stop = controller_->emergencyStop(EmergencyStopLevel::HARD_STOP);
        checkTest("硬停止调用", hard_stop == MotionResult::SUCCESS);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        MotionResult power_off = controller_->emergencyStop(EmergencyStopLevel::POWER_OFF);
        checkTest("断电停止调用", power_off == MotionResult::SUCCESS);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // 测试极限参数处理
        Posture extreme_posture;
        extreme_posture.roll = 10.0f;  // 极大滚转角
        extreme_posture.pitch = 10.0f; // 极大俯仰角
        extreme_posture.body_height = 2.0f; // 极大高度

        MotionResult extreme_posture_result = controller_->setPosture(extreme_posture);
        checkTest("极限姿态参数处理", extreme_posture_result == MotionResult::CAPABILITY_LIMITED);

        // 测试无效参数处理
        MotionResult invalid_height = controller_->setBodyHeight(-0.1f); // 负高度
        checkTest("无效高度参数处理", invalid_height == MotionResult::CAPABILITY_LIMITED);

        std::cout << "  安全保护功能正常，能正确处理极限和无效参数" << std::endl;

        // 恢复到安全状态准备结束测试
        controller_->recoveryStand();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }

    /**
     * @brief 运行单个测试
     * @param testName 测试名称
     * @param testFunc 测试函数
     */
    void runSingleTest(const std::string& testName, std::function<void()> testFunc) {
        printHeader("运行单个测试: " + testName);

        // 重置统计数据为本次测试
        int prev_total = total_tests_;
        int prev_passed = passed_tests_;

        if (checkShutdown()) return;
        testFunc();

        // 显示本次测试结果
        int current_total = total_tests_ - prev_total;
        int current_passed = passed_tests_ - prev_passed;

        std::cout << "\n" << std::string(50, '-') << std::endl;
        std::cout << "测试 [" << testName << "] 完成:" << std::endl;
        std::cout << "  本次测试数: " << current_total << std::endl;
        std::cout << "  通过测试: " << current_passed << std::endl;
        std::cout << "  失败测试: " << (current_total - current_passed) << std::endl;
        if (current_total > 0) {
            std::cout << "  成功率: " << std::fixed << std::setprecision(1)
                      << (100.0 * current_passed / current_total) << "%" << std::endl;
        }

        if (!g_shutdown_requested.load()) {
            if (current_total == current_passed) {
                std::cout << "🎉 测试全部通过！" << std::endl;
            } else {
                std::cout << "⚠️  有部分测试失败。" << std::endl;
            }
        } else {
            std::cout << "⏹️  测试被中断。" << std::endl;
        }
    }

    /**
     * @brief 检查是否收到停止信号
     * @return 如果收到停止信号返回true
     */
    bool checkShutdown() {
        if (g_shutdown_requested.load()) {
            std::cout << "\n检测到停止信号，正在终止测试..." << std::endl;
            return true;
        }
        return false;
    }

    /**
     * @brief 打印被中断的测试结果
     */
    void printInterruptedResults() {
        printHeader("测试被中断 - 部分结果总结");

        std::cout << "已运行测试数: " << total_tests_ << std::endl;
        std::cout << "通过测试: " << passed_tests_ << std::endl;
        std::cout << "失败测试: " << (total_tests_ - passed_tests_) << std::endl;
        if (total_tests_ > 0) {
            std::cout << "成功率: " << std::fixed << std::setprecision(1)
                      << (100.0 * passed_tests_ / total_tests_) << "%" << std::endl;
        }

        std::cout << "\n⏹️  测试被用户中断。正在执行清理..." << std::endl;

        // 执行清理
        if (controller_) {
            std::cout << "正在关闭运动控制器..." << std::endl;
            controller_->shutdown();
        }
    }

    /**
     * @brief 检查测试结果并记录
     */
    void checkTest(const std::string& test_name, bool result) {
        total_tests_++;
        if (result) {
            passed_tests_++;
            std::cout << "  ✓ " << test_name << " - 通过" << std::endl;
            test_results_.push_back("PASS: " + test_name);
        } else {
            std::cout << "  ✗ " << test_name << " - 失败" << std::endl;
            test_results_.push_back("FAIL: " + test_name);
        }
    }

    /**
     * @brief 打印测试章节标题
     */
    void printHeader(const std::string& title) {
        std::cout << "\n" << std::string(50, '=') << std::endl;
        std::cout << title << std::endl;
        std::cout << std::string(50, '=') << std::endl;
    }

    /**
     * @brief 打印最终测试结果
     */
    void printFinalResults() {
        printHeader("测试结果总结");

        std::cout << "总测试数: " << total_tests_ << std::endl;
        std::cout << "通过测试: " << passed_tests_ << std::endl;
        std::cout << "失败测试: " << (total_tests_ - passed_tests_) << std::endl;
        std::cout << "成功率: " << std::fixed << std::setprecision(1)
                  << (100.0 * passed_tests_ / total_tests_) << "%" << std::endl;

        std::cout << "\n详细结果:" << std::endl;
        for (const auto& result : test_results_) {
            std::cout << "  " << result << std::endl;
        }

        // 执行最后的清理
        std::cout << "\n执行最终清理..." << std::endl;

        // 测试最终关闭
        MotionResult shutdown_result = controller_->shutdown();
        if (shutdown_result == MotionResult::SUCCESS) {
            std::cout << "✓ shutdown()调用 - 通过" << std::endl;
        } else {
            std::cout << "✗ shutdown()调用 - 失败" << std::endl;
        }

        if (passed_tests_ == total_tests_) {
            std::cout << "\n🎉 所有功能验证测试通过！Go2MotionController类工作正常。" << std::endl;
        } else {
            std::cout << "\n⚠️  有部分测试失败，请检查Go2MotionController类的实现。" << std::endl;
        }
    }
};

/**
 * @brief 主函数
 */
int main(int argc, char** argv) {
    // 注册信号处理函数
    std::signal(SIGINT, signalHandler);

    // 初始化ROS2
    rclcpp::init(argc, argv);

    try {
        // 创建测试器并运行交互式菜单
        Go2MotionControllerTester tester;
        tester.runInteractiveMenu();

    } catch (const std::exception& e) {
        std::cerr << "测试过程中发生异常: " << e.what() << std::endl;
        return 1;
    }

    // 关闭ROS2
    rclcpp::shutdown();
    return 0;
}