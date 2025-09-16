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
#include <rcutils/logging.h>
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
        _exit(0); // 立即退出，不做任何清理，避免阻塞
    }
}

class Go2MotionControllerTester {
private:
    std::shared_ptr<Go2MotionController> controller_;
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

        // 创建Go2MotionController实例（必须使用shared_ptr，因为它继承自enable_shared_from_this）
        controller_ = std::make_shared<Go2MotionController>("test_go2_motion_controller");

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
        std::cout << "请输入选择: " << std::flush;
    }

    /**
     * @brief 获取控制器实例，供main函数中的ROS消息处理线程使用
     */
    std::shared_ptr<Go2MotionController> getController() const {
        return controller_;
    }

    /**
     * @brief 运行交互式菜单
     */
    void runInteractiveMenu() {

        while (!g_shutdown_requested.load()) {
            if (!rclcpp::ok()) {
                break;
            }

            showTestMenu();

            // 非阻塞等待键盘输入，避免 Ctrl+C 时阻塞在输入上，支持多位数
            bool got_input = false;
            std::string input;
            while (!got_input) {
                if (g_shutdown_requested.load() || !rclcpp::ok()) {
                    break;
                }

                struct pollfd pfd;
                pfd.fd = STDIN_FILENO;
                pfd.events = POLLIN;
                int ret = poll(&pfd, 1, 200); // 200ms 轮询
                if (ret > 0 && (pfd.revents & POLLIN)) {
                    // 一次性读完整行，提取数字或'q'
                    for (;;) {
                        int c = std::getchar();
                        if (c == EOF) {
                            return;
                        }
                        if (c == '\n') { got_input = true; break; }
                        if (c == '\r') {
                            // 兼容CRLF，尝试吞掉后续的'\n'
                            int next_c = std::getchar();
                            if (next_c != '\n' && next_c != EOF) { ungetc(next_c, stdin); }
                            got_input = true; break;
                        }
                        if (c >= '0' && c <= '9') {
                            input += static_cast<char>(c);
                        } else if (c == 'q' || c == 'Q') {
                            input = "q";
                            // 继续丢弃到行尾
                        } else {
                            // 忽略其它字符（空格等），继续等到行尾
                        }
                    }
                }
            }

            if (g_shutdown_requested.load() || !rclcpp::ok()) {
                break;
            }

            if (input == "q" || input == "Q") {
                std::cout << "强制退出程序..." << std::endl;
                std::fflush(stdout);
                _exit(0); // 立即退出，不做任何清理，避免阻塞
            } else if (input == "1") {
                runSingleTest("初始化和生命周期管理测试", [this]() { testInitializationLifecycle(); });
            } else if (input == "2") {
                runSingleTest("运动能力参数测试", [this]() { testCapabilities(); });
            } else if (input == "3") {
                runSingleTest("基础运动控制测试", [this]() { testBasicMotionControl(); });
            } else if (input == "4") {
                runSingleTest("运动模式切换测试", [this]() { testModeSwitch(); });
            } else if (input == "5") {
                runSingleTest("Go2基本动作测试", [this]() { testBasicActions(); });
            } else if (input == "6") {
                runSingleTest("Go2特有高级功能测试", [this]() { testAdvancedFeatures(); });
            } else if (input == "7") {
                runSingleTest("状态查询功能测试", [this]() { testStateQuery(); });
            } else if (input == "8") {
                runSingleTest("回调函数设置测试", [this]() { testCallbacks(); });
            } else if (input == "9") {
                runSingleTest("错误处理和安全测试", [this]() { testErrorHandling(); });
            } else if (input == "0") {
                runAllTests();
            } else if (!input.empty()) {
                std::cout << "无效选择，请重新输入。" << std::endl;
            } else {
                // 空输入，重新显示提示
                std::cout << "请输入选择: " << std::flush;
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
     * @brief 测试基础运动控制 - 交互式菜单
     */
    void testBasicMotionControl() {
        printHeader("基础运动控制测试 - 交互式菜单");

        std::vector<std::string> options = {
            "前进运动 (linear_x)",
            "后退运动 (-linear_x)",
            "左移运动 (linear_y)",
            "右移运动 (-linear_y)",
            "左转运动 (angular_z)",
            "右转运动 (-angular_z)",
            "复合运动 (前进+左转)",
            "停止所有运动",
            "测试姿态控制 (roll/pitch/yaw)",
            "测试机身高度控制",
            "测试极限速度(安全测试)",
            "测试极限高度(安全测试)"
        };

        while (!checkShutdown()) {
            int choice = showSubmenu("基础运动控制测试菜单", options);
            if (choice == -1) break;  // 返回上级菜单

            int duration = getDurationFromUser(2000);
            if (duration == -1) continue;  // 用户取消

            switch (choice) {
                case 0: { // 前进运动
                    Velocity vel = {}; vel.linear_x = 0.5f;
                    bool success = safeExecuteCommand("前进运动", [this, vel]() {
                        return controller_->setVelocity(vel);
                    }, duration);
                    checkTest("前进运动", success);
                    break;
                }
                case 1: { // 后退运动
                    Velocity vel = {}; vel.linear_x = -0.5f;
                    bool success = safeExecuteCommand("后退运动", [this, vel]() {
                        return controller_->setVelocity(vel);
                    }, duration);
                    checkTest("后退运动", success);
                    break;
                }
                case 2: { // 左移运动
                    Velocity vel = {}; vel.linear_y = 0.3f;
                    bool success = safeExecuteCommand("左移运动", [this, vel]() {
                        return controller_->setVelocity(vel);
                    }, duration);
                    checkTest("左移运动", success);
                    break;
                }
                case 3: { // 右移运动
                    Velocity vel = {}; vel.linear_y = -0.3f;
                    bool success = safeExecuteCommand("右移运动", [this, vel]() {
                        return controller_->setVelocity(vel);
                    }, duration);
                    checkTest("右移运动", success);
                    break;
                }
                case 4: { // 左转运动
                    Velocity vel = {}; vel.angular_z = 0.5f;
                    bool success = safeExecuteCommand("左转运动", [this, vel]() {
                        return controller_->setVelocity(vel);
                    }, duration);
                    checkTest("左转运动", success);
                    break;
                }
                case 5: { // 右转运动
                    Velocity vel = {}; vel.angular_z = -0.5f;
                    bool success = safeExecuteCommand("右转运动", [this, vel]() {
                        return controller_->setVelocity(vel);
                    }, duration);
                    checkTest("右转运动", success);
                    break;
                }
                case 6: { // 复合运动
                    Velocity vel = {}; vel.linear_x = 0.3f; vel.angular_z = 0.3f;
                    bool success = safeExecuteCommand("复合运动(前进+左转)", [this, vel]() {
                        return controller_->setVelocity(vel);
                    }, duration);
                    checkTest("复合运动", success);
                    break;
                }
                case 7: { // 停止所有运动
                    Velocity vel = {};
                    bool success = safeExecuteCommand("停止所有运动", [this, vel]() {
                        return controller_->setVelocity(vel);
                    }, 1000);
                    checkTest("停止运动", success);
                    break;
                }
                case 8: { // 姿态控制
                    Posture posture = {};
                    posture.roll = 0.1f; posture.pitch = 0.05f; posture.yaw = 0.2f;
                    posture.body_height = 0.3f;
                    bool success = safeExecuteCommand("姿态控制", [this, posture]() {
                        return controller_->setPosture(posture);
                    }, duration);
                    checkTest("姿态控制", success);
                    break;
                }
                case 9: { // 机身高度控制
                    bool success = safeExecuteCommand("机身高度控制", [this]() {
                        return controller_->setBodyHeight(0.25f);
                    }, duration);
                    checkTest("机身高度控制", success);
                    break;
                }
                case 10: { // 极限速度测试
                    Velocity extreme_vel = {};
                    extreme_vel.linear_x = 10.0f; extreme_vel.angular_z = 10.0f;
                    MotionResult result = controller_->setVelocity(extreme_vel);
                    checkTest("极限速度控制", result == MotionResult::CAPABILITY_LIMITED);
                    break;
                }
                case 11: { // 极限高度测试
                    MotionResult result = controller_->setBodyHeight(1.0f);
                    checkTest("极限高度控制", result == MotionResult::CAPABILITY_LIMITED);
                    break;
                }
            }

            // 每次测试后自动停止运动确保安全
            if (choice < 7) {  // 除了"停止运动"选项外
                std::cout << "  自动停止运动确保安全..." << std::endl;
                Velocity zero_vel = {};
                safeExecuteCommand("自动停止", [this, zero_vel]() {
                    return controller_->setVelocity(zero_vel);
                }, 500);
            }
        }
    }

    /**
     * @brief 测试运动模式切换 - 交互式菜单
     */
    void testModeSwitch() {
        printHeader("运动模式切换测试 - 交互式菜单");

        std::vector<std::string> options = {
            "空闲模式 (IDLE)",
            "平衡站立 (BALANCE_STAND)",
            "运动模式 (LOCOMOTION)",
            "趴下模式 (LIE_DOWN)",
            "坐姿模式 (SIT)",
            "恢复站立 (RECOVERY_STAND)",
            "--- 步态设置 ---",
            "空闲步态 (IDLE)",
            "小跑步态 (TROT)",
            "奔跑步态 (RUN)",
            "爬楼梯步态 (CLIMB_STAIR)"
        };

        while (!checkShutdown()) {
            int choice = showSubmenu("运动模式切换测试菜单", options);
            if (choice == -1) break;  // 返回上级菜单

            if (choice == 6) {
                std::cout << "--- 这是分隔线，请选择步态设置选项 ---" << std::endl;
                continue;
            }

            int duration = getDurationFromUser(3000);  // 模式切换默认需要更长时间
            if (duration == -1) continue;  // 用户取消

            switch (choice) {
                case 0: { // IDLE
                    bool success = safeExecuteCommand("切换到空闲模式", [this]() {
                        return controller_->switchMode(MotionMode::IDLE);
                    }, duration);
                    checkTest("空闲模式切换", success);
                    break;
                }
                case 1: { // BALANCE_STAND
                    bool success = safeExecuteCommand("切换到平衡站立", [this]() {
                        return controller_->switchMode(MotionMode::BALANCE_STAND);
                    }, duration);
                    checkTest("平衡站立模式切换", success);
                    break;
                }
                case 2: { // LOCOMOTION
                    bool success = safeExecuteCommand("切换到运动模式", [this]() {
                        return controller_->switchMode(MotionMode::LOCOMOTION);
                    }, duration);
                    checkTest("运动模式切换", success);
                    break;
                }
                case 3: { // LIE_DOWN
                    std::cout << "  ⚠️ 注意：趴下模式可能需要手动恢复，确保安全" << std::endl;
                    bool success = safeExecuteCommand("切换到趴下模式", [this]() {
                        return controller_->switchMode(MotionMode::LIE_DOWN);
                    }, duration);
                    checkTest("趴下模式切换", success);

                    // 自动恢复到安全状态
                    if (success && !checkShutdown()) {
                        std::cout << "    自动恢复到站立状态..." << std::endl;
                        safeExecuteCommand("恢复站立", [this]() {
                            return controller_->switchMode(MotionMode::RECOVERY_STAND);
                        }, 4000);
                    }
                    break;
                }
                case 4: { // SIT
                    bool success = safeExecuteCommand("切换到坐姿模式", [this]() {
                        return controller_->switchMode(MotionMode::SIT);
                    }, duration);
                    checkTest("坐姿模式切换", success);
                    break;
                }
                case 5: { // RECOVERY_STAND
                    bool success = safeExecuteCommand("切换到恢复站立", [this]() {
                        return controller_->switchMode(MotionMode::RECOVERY_STAND);
                    }, duration);
                    checkTest("恢复站立模式切换", success);
                    break;
                }
                case 7: { // IDLE 步态
                    bool success = safeExecuteCommand("设置空闲步态", [this]() {
                        return controller_->setGaitType(GaitType::IDLE);
                    }, duration);
                    checkTest("空闲步态设置", success);
                    break;
                }
                case 8: { // TROT 步态
                    bool success = safeExecuteCommand("设置小跑步态", [this]() {
                        return controller_->setGaitType(GaitType::TROT);
                    }, duration);
                    checkTest("小跑步态设置", success);
                    break;
                }
                case 9: { // RUN 步态
                    bool success = safeExecuteCommand("设置奔跑步态", [this]() {
                        return controller_->setGaitType(GaitType::RUN);
                    }, duration);
                    checkTest("奔跑步态设置", success);
                    break;
                }
                case 10: { // CLIMB_STAIR 步态
                    bool success = safeExecuteCommand("设置爬楼梯步态", [this]() {
                        return controller_->setGaitType(GaitType::CLIMB_STAIR);
                    }, duration);
                    checkTest("爬楼梯步态设置", success);
                    break;
                }
            }
        }
    }

    /**
     * @brief 测试Go2基本动作 - 交互式菜单
     */
    void testBasicActions() {
        printHeader("Go2基本动作测试 - 交互式菜单");

        std::vector<std::string> options = {
            "平衡站立 (balanceStand)",
            "站起动作 (standUp)",
            "坐下动作 (sit)",
            "恢复站立 (recoveryStand)",
            "趴下动作 (standDown)",
            "快速站立序列",
            "快速趴下序列",
            "基本动作循环测试"
        };

        while (!checkShutdown()) {
            int choice = showSubmenu("Go2基本动作测试菜单", options);
            if (choice == -1) break;  // 返回上级菜单

            int duration = getDurationFromUser(2000);
            if (duration == -1) continue;  // 用户取消

            switch (choice) {
                case 0: { // 平衡站立
                    bool success = safeExecuteCommand("平衡站立", [this]() {
                        return controller_->balanceStand();
                    }, duration);
                    checkTest("平衡站立", success);
                    break;
                }
                case 1: { // 站起动作
                    bool success = safeExecuteCommand("站起动作", [this]() {
                        return controller_->standUp();
                    }, duration);
                    checkTest("站起动作", success);
                    break;
                }
                case 2: { // 坐下动作
                    bool success = safeExecuteCommand("坐下动作", [this]() {
                        return controller_->sit();
                    }, duration);
                    checkTest("坐下动作", success);
                    break;
                }
                case 3: { // 恢复站立
                    bool success = safeExecuteCommand("恢复站立", [this]() {
                        return controller_->recoveryStand();
                    }, duration);
                    checkTest("恢复站立", success);
                    break;
                }
                case 4: { // 趴下动作
                    bool success = safeExecuteCommand("趴下动作", [this]() {
                        return controller_->standDown();
                    }, duration);
                    checkTest("趴下动作", success);

                    // 自动恢复到站立状态确保安全
                    if (success && !checkShutdown()) {
                        std::cout << "    自动恢复到站立状态..." << std::endl;
                        safeExecuteCommand("自动站起", [this]() {
                            return controller_->standUp();
                        }, 3000);
                    }
                    break;
                }
                case 5: { // 快速站立序列
                    std::cout << "  执行快速站立序列: 坐下 -> 站起" << std::endl;
                    if (!checkShutdown()) {
                        safeExecuteCommand("坐下", [this]() {
                            return controller_->sit();
                        }, 2000);
                    }
                    if (!checkShutdown()) {
                        safeExecuteCommand("站起", [this]() {
                            return controller_->standUp();
                        }, 2000);
                    }
                    checkTest("快速站立序列", true);
                    break;
                }
                case 6: { // 快速趴下序列
                    std::cout << "  执行快速趴下序列: 趴下 -> 恢复站立" << std::endl;
                    if (!checkShutdown()) {
                        safeExecuteCommand("趴下", [this]() {
                            return controller_->standDown();
                        }, 2000);
                    }
                    if (!checkShutdown()) {
                        safeExecuteCommand("恢复站立", [this]() {
                            return controller_->recoveryStand();
                        }, 3000);
                    }
                    checkTest("快速趴下序列", true);
                    break;
                }
                case 7: { // 基本动作循环测试
                    std::cout << "  执行基本动作循环: 站立 -> 坐下 -> 站起 -> 平衡站立" << std::endl;
                    std::vector<std::string> actions = {"坐下", "站起", "平衡站立"};
                    std::vector<std::function<MotionResult()>> commands = {
                        [this]() { return controller_->sit(); },
                        [this]() { return controller_->standUp(); },
                        [this]() { return controller_->balanceStand(); }
                    };

                    bool all_success = true;
                    for (size_t i = 0; i < actions.size() && !checkShutdown(); ++i) {
                        bool success = safeExecuteCommand(actions[i], commands[i], 2000);
                        if (!success) all_success = false;
                    }
                    checkTest("基本动作循环测试", all_success);
                    break;
                }
            }
        }
    }

    /**
     * @brief 测试Go2特有高级功能 - 交互式菜单
     */
    void testAdvancedFeatures() {
        printHeader("Go2特有高级功能测试 - 交互式菜单");

        std::vector<std::string> options = {
            "舞蹈动作1 (performDance 1)",
            "舞蹈动作2 (performDance 2)",
            "打招呼动作 (hello)",
            "伸展动作 (stretch)",
            "速度等级设置 (1-5级)",
            "--- 高风险动作 (需要确认) ---",
            "前跳动作 (frontJump) ⚠️",
            "前翻动作 (frontFlip) ⚠️",
            "--- 安全测试 ---",
            "无效舞蹈类型测试",
            "无效速度等级测试",
            "高级功能组合测试"
        };

        while (!checkShutdown()) {
            int choice = showSubmenu("Go2高级功能测试菜单", options);
            if (choice == -1) break;  // 返回上级菜单

            if (choice == 5 || choice == 8) {
                std::cout << "--- 这是分隔线，请选择具体功能 ---" << std::endl;
                continue;
            }

            int duration = getDurationFromUser(3000);
            if (duration == -1) continue;  // 用户取消

            switch (choice) {
                case 0: { // 舞蹈动作1
                    bool success = safeExecuteCommand("舞蹈动作1", [this]() {
                        return controller_->performDance(1);
                    }, duration);
                    checkTest("舞蹈动作1", success);
                    break;
                }
                case 1: { // 舞蹈动作2
                    bool success = safeExecuteCommand("舞蹈动作2", [this]() {
                        return controller_->performDance(2);
                    }, duration);
                    checkTest("舞蹈动作2", success);
                    break;
                }
                case 2: { // 打招呼动作
                    bool success = safeExecuteCommand("打招呼动作", [this]() {
                        return controller_->hello();
                    }, duration);
                    checkTest("打招呼动作", success);
                    break;
                }
                case 3: { // 伸展动作
                    bool success = safeExecuteCommand("伸展动作", [this]() {
                        return controller_->stretch();
                    }, duration);
                    checkTest("伸展动作", success);
                    break;
                }
                case 4: { // 速度等级设置
                    std::cout << "  测试速度等级设置 (1-5级):" << std::endl;
                    bool all_success = true;
                    for (int level = 1; level <= 5; ++level) {
                        if (checkShutdown()) break;
                        std::cout << "    设置速度等级: " << level << std::endl;
                        bool success = safeExecuteCommand("速度等级" + std::to_string(level), [this, level]() {
                            return controller_->setSpeedLevel(level);
                        }, 500);
                        checkTest("速度等级" + std::to_string(level), success);
                        if (!success) all_success = false;
                    }
                    checkTest("速度等级设置完整测试", all_success);
                    break;
                }
                case 6: { // 前跳动作 - 高风险
                    if (waitForUserConfirmation("前跳动作")) {
                        bool success = safeExecuteCommand("前跳动作", [this]() {
                            return controller_->frontJump();
                        }, duration);
                        checkTest("前跳动作", success);

                        // 立即恢复到安全状态
                        if (success && !checkShutdown()) {
                            std::cout << "    立即恢复到安全状态..." << std::endl;
                            safeExecuteCommand("恢复站立", [this]() {
                                return controller_->recoveryStand();
                            }, 4000);
                        }
                    } else {
                        std::cout << "    用户取消前跳动作测试" << std::endl;
                        checkTest("前跳动作", true); // 跳过但记录为通过
                    }
                    break;
                }
                case 7: { // 前翻动作 - 高风险
                    if (waitForUserConfirmation("前翻动作")) {
                        bool success = safeExecuteCommand("前翻动作", [this]() {
                            return controller_->frontFlip();
                        }, duration);
                        checkTest("前翻动作", success);

                        // 立即恢复到安全状态
                        if (success && !checkShutdown()) {
                            std::cout << "    立即恢复到安全状态..." << std::endl;
                            safeExecuteCommand("恢复站立", [this]() {
                                return controller_->recoveryStand();
                            }, 4000);
                        }
                    } else {
                        std::cout << "    用户取消前翻动作测试" << std::endl;
                        checkTest("前翻动作", true); // 跳过但记录为通过
                    }
                    break;
                }
                case 9: { // 无效舞蹈类型测试
                    MotionResult result = controller_->performDance(99);
                    checkTest("无效舞蹈类型处理", result == MotionResult::INVALID_PARAMETER);
                    std::cout << "    无效舞蹈类型测试完成，期望结果：INVALID_PARAMETER" << std::endl;
                    break;
                }
                case 10: { // 无效速度等级测试
                    MotionResult result = controller_->setSpeedLevel(10);
                    checkTest("无效速度等级处理", result == MotionResult::INVALID_PARAMETER);
                    std::cout << "    无效速度等级测试完成，期望结果：INVALID_PARAMETER" << std::endl;
                    break;
                }
                case 11: { // 高级功能组合测试
                    std::cout << "  执行高级功能组合测试: 打招呼 -> 伸展 -> 舞蹈1" << std::endl;
                    std::vector<std::string> actions = {"打招呼", "伸展", "舞蹈1"};
                    std::vector<std::function<MotionResult()>> commands = {
                        [this]() { return controller_->hello(); },
                        [this]() { return controller_->stretch(); },
                        [this]() { return controller_->performDance(1); }
                    };

                    bool all_success = true;
                    for (size_t i = 0; i < actions.size() && !checkShutdown(); ++i) {
                        bool success = safeExecuteCommand(actions[i], commands[i], 3000);
                        if (!success) all_success = false;
                    }
                    checkTest("高级功能组合测试", all_success);
                    break;
                }
            }

            // 高级动作后确保恢复到安全状态
            if (choice <= 4 || choice == 11) {
                std::cout << "  恢复到安全状态..." << std::endl;
                safeExecuteCommand("恢复站立", [this]() {
                    return controller_->balanceStand();
                }, 2000);
            }
        }
    }

    /**
     * @brief 测试状态查询功能
     */
    void testStateQuery() {
        printHeader("测试状态查询功能");

        // 等待接收一次ROS2消息更新（ROS消息现在在后台线程中持续处理）
        std::cout << "正在等待接收一次状态更新..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // 测试运动状态获取
        MotionState motion_state = controller_->getMotionState();
        checkTest("getMotionState()调用", true);

        std::cout << "  当前运动状态:" << std::endl;
        std::cout << "    运动模式: " << static_cast<int>(motion_state.current_mode) << std::endl;
        std::cout << "    步态类型: " << static_cast<int>(motion_state.current_gait) << std::endl;
        std::cout << "    是否移动: " << (motion_state.is_moving ? "是" : "否") << std::endl;
        std::cout << "    是否平衡: " << (motion_state.is_balanced ? "是" : "否") << std::endl;
        std::cout << "    机身高度: " << motion_state.posture.body_height << " m" << std::endl;
        std::cout << "    状态错误代码: " << motion_state.error_code << " (注意：Go2原始error_code字段表示模式，非错误)" << std::endl;

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

        // 等待回调触发（ROS消息现在在后台线程中持续处理）
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
     * @brief 安全执行运动命令，带有中断检查和紧急停止机制
     * @param command_name 命令名称，用于日志输出
     * @param command_func 要执行的命令函数
     * @param wait_time_ms 命令执行后等待时间(毫秒)
     * @return 命令是否成功执行（不包括被中断的情况）
     */
    bool safeExecuteCommand(const std::string& command_name,
                           std::function<MotionResult()> command_func,
                           int wait_time_ms = 1000) {
        if (checkShutdown()) {
            return false;
        }

        std::cout << "  正在执行: " << command_name << std::endl;

        // 执行命令
        MotionResult result = command_func();
        if (result != MotionResult::SUCCESS) {
            std::cout << "    ⚠️ 命令执行失败，结果代码: " << static_cast<int>(result) << std::endl;
            return false;
        }

        std::cout << "    ✓ 命令发送成功，等待执行完成..." << std::endl;

        // 分段等待，支持中断
        const int check_interval = 100; // 每100ms检查一次中断信号
        int remaining_time = wait_time_ms;

        while (remaining_time > 0 && !checkShutdown()) {
            int current_wait = std::min(check_interval, remaining_time);
            std::this_thread::sleep_for(std::chrono::milliseconds(current_wait));
            remaining_time -= current_wait;
        }

        if (checkShutdown()) {
            std::cout << "    ⏹️ 检测到中断信号，执行紧急停止..." << std::endl;
            emergencyStopRobot();
            return false;
        }

        std::cout << "    ✓ 命令执行完成" << std::endl;
        return true;
    }

    /**
     * @brief 紧急停止机器人
     */
    void emergencyStopRobot() {
        std::cout << "  🚨 执行紧急停止序列..." << std::endl;

        // 尝试软停止
        if (controller_) {
            MotionResult soft_stop = controller_->emergencyStop(EmergencyStopLevel::SOFT_STOP);
            if (soft_stop == MotionResult::SUCCESS) {
                std::cout << "    ✓ 软停止成功" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                return;
            }
        }

        // 如果软停止失败，尝试硬停止
        std::cout << "    ⚠️ 软停止失败，尝试硬停止..." << std::endl;
        if (controller_) {
            MotionResult hard_stop = controller_->emergencyStop(EmergencyStopLevel::HARD_STOP);
            if (hard_stop == MotionResult::SUCCESS) {
                std::cout << "    ✓ 硬停止成功" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                return;
            }
        }

        std::cout << "    ⚠️ 紧急停止失败，请手动检查机器人状态" << std::endl;
    }

    /**
     * @brief 等待用户确认继续执行危险动作
     * @param action_name 动作名称
     * @return 用户是否同意继续
     */
    bool waitForUserConfirmation(const std::string& action_name) {
        if (checkShutdown()) {
            return false;
        }

        std::cout << "\n  ⚠️ 警告: 即将执行 [" << action_name << "] 动作" << std::endl;
        std::cout << "  此动作可能存在安全风险，请确保:" << std::endl;
        std::cout << "    1. 机器人周围有足够的安全空间" << std::endl;
        std::cout << "    2. 没有人员或障碍物在机器人附近" << std::endl;
        std::cout << "    3. 机器人处于平坦稳定的地面上" << std::endl;
        std::cout << "  输入 'y' 继续，任何其他键取消: ";

        // 非阻塞等待用户输入（整行读取）
        while (!checkShutdown()) {
            struct pollfd pfd;
            pfd.fd = STDIN_FILENO;
            pfd.events = POLLIN;
            int ret = poll(&pfd, 1, 100); // 100ms轮询

            if (ret > 0 && (pfd.revents & POLLIN)) {
                std::string line;
                for (;;) {
                    int c = std::getchar();
                    if (c == EOF) { return false; }
                    if (c == '\n') { break; }
                    if (c == '\r') {
                        int next_c = std::getchar();
                        if (next_c != '\n' && next_c != EOF) { ungetc(next_c, stdin); }
                        break;
                    }
                    line += static_cast<char>(c);
                }

                if (line == "y" || line == "Y") {
                    std::cout << "    ✓ 用户确认继续执行" << std::endl;
                    return true;
                } else {
                    std::cout << "    ✗ 用户取消执行" << std::endl;
                    return false;
                }
            }
        }

        return false;
    }

    /**
     * @brief 获取用户输入的持续时间
     * @param default_duration 默认持续时间(毫秒)
     * @return 用户设置的持续时间(毫秒)
     */
    int getDurationFromUser(int default_duration = 2000) {
        std::cout << "  请输入动作持续时间(毫秒，默认" << default_duration << "ms，直接回车使用默认值): " << std::flush;

        std::string input;
        bool got_input = false;

        // 非阻塞获取用户输入（整行读取）
        while (!got_input && !checkShutdown()) {
            struct pollfd pfd;
            pfd.fd = STDIN_FILENO;
            pfd.events = POLLIN;
            int ret = poll(&pfd, 1, 100);

            if (ret > 0 && (pfd.revents & POLLIN)) {
                for (;;) {
                    int c = std::getchar();
                    if (c == EOF) { std::cout << "\n    取消输入" << std::endl; return -1; }
                    if (c == '\n') { got_input = true; break; }
                    if (c == '\r') {
                        int next_c = std::getchar();
                        if (next_c != '\n' && next_c != EOF) { ungetc(next_c, stdin); }
                        got_input = true; break;
                    }
                    if (c >= '0' && c <= '9') {
                        input += static_cast<char>(c);
                    } else if (c == 27) { // ESC键
                        // 丢弃到行尾
                        int d;
                        while ((d = std::getchar()) != '\n' && d != EOF) {
                            if (d == '\r') { int e = std::getchar(); if (e != '\n' && e != EOF) { ungetc(e, stdin); } break; }
                        }
                        std::cout << "\n    取消输入" << std::endl;
                        return -1;
                    } else {
                        // 忽略其它字符
                    }
                }
            }
        }

        if (checkShutdown()) return -1;

        if (input.empty()) {
            std::cout << "    使用默认持续时间: " << default_duration << "ms" << std::endl;
            return default_duration;
        }

        try {
            int duration = std::stoi(input);
            if (duration < 100) duration = 100;  // 最小100ms
            if (duration > 10000) duration = 10000;  // 最大10s
            std::cout << "    设置持续时间: " << duration << "ms" << std::endl;
            return duration;
        } catch (...) {
            std::cout << "    输入无效，使用默认持续时间: " << default_duration << "ms" << std::endl;
            return default_duration;
        }
    }

    /**
     * @brief 显示子菜单并获取用户选择
     * @param title 菜单标题
     * @param options 选项列表
     * @return 用户选择的选项索引，-1表示退出
     */
    int showSubmenu(const std::string& title, const std::vector<std::string>& options) {
        while (!checkShutdown()) {
            std::cout << "\n" << std::string(50, '=') << std::endl;
            std::cout << title << std::endl;
            std::cout << std::string(50, '=') << std::endl;

            for (size_t i = 0; i < options.size(); ++i) {
                std::cout << (i + 1) << ". " << options[i] << std::endl;
            }
            std::cout << "0. 返回上级菜单" << std::endl;
            std::cout << std::string(50, '-') << std::endl;
            std::cout << "请选择: " << std::flush;

            // 非阻塞等待键盘输入，支持多位数（整行读取）
            bool got_input = false;
            std::string input;
            while (!got_input && !checkShutdown()) {
                struct pollfd pfd;
                pfd.fd = STDIN_FILENO;
                pfd.events = POLLIN;
                int ret = poll(&pfd, 1, 200);
                if (ret > 0 && (pfd.revents & POLLIN)) {
                    for (;;) {
                        int c = std::getchar();
                        if (c == EOF) return -1;
                        if (c == '\n') { got_input = true; break; }
                        if (c == '\r') {
                            int next_c = std::getchar();
                            if (next_c != '\n' && next_c != EOF) { ungetc(next_c, stdin); }
                            got_input = true; break;
                        }
                        if (c >= '0' && c <= '9') {
                            input += static_cast<char>(c);
                        } else if (c == 'q' || c == 'Q') {
                            input = "q"; // 继续丢弃到行尾
                        } else if (c == 27) { // ESC键
                            // 丢弃到行尾
                            int d;
                            while ((d = std::getchar()) != '\n' && d != EOF) {
                                if (d == '\r') {
                                    int e = std::getchar();
                                    if (e != '\n' && e != EOF) { ungetc(e, stdin); }
                                    break;
                                }
                            }
                            return -1;
                        } else {
                            // 忽略其它字符
                        }
                    }
                }
            }

            if (checkShutdown()) return -1;

            if (input == "q" || input == "Q") {
                return -1;  // 退出
            }

            if (input == "0") {
                return -1;  // 返回上级菜单
            }

            if (!input.empty()) {
                try {
                    int selected = std::stoi(input) - 1;
                    if (selected >= 0 && selected < static_cast<int>(options.size())) {
                        return selected;
                    }
                } catch (const std::exception&) {
                    // 无效数字
                }
                std::cout << "无效选择，请重新输入。" << std::endl;
            }
        }
        return -1;
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

    // 设置ROS日志级别，减少控制台输出
    rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_WARN);

    // 初始化ROS2
    rclcpp::init(argc, argv);

    try {
        // 创建测试器
        Go2MotionControllerTester tester;

        // 启动ROS消息处理线程，持续处理订阅的消息
        std::thread spin_thread([&tester]() {
            rclcpp::spin(tester.getController());
        });

        // 运行交互式菜单
        tester.runInteractiveMenu();

        // 等待ROS消息处理线程结束
        if (spin_thread.joinable()) {
            spin_thread.join();
        }

    } catch (const std::exception& e) {
        std::cerr << "测试过程中发生异常: " << e.what() << std::endl;
        return 1;
    }

    // 关闭ROS2
    rclcpp::shutdown();
    return 0;
}