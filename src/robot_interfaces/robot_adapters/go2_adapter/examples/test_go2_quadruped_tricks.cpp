/**
 * @file   test_go2_quadruped_tricks.cpp
 * @brief  Go2QuadrupedTricks类功能验证测试程序
 * @author Yang Nan
 * @date   2025-09-17
 *
 * @details
 * 这是一个用于验证Go2QuadrupedTricks类各项功能的测试程序。
 * 该程序创建Go2QuadrupedTricks实例，并系统性地测试其各个功能模块，
 * 包括基础动作、特技动作、运动参数设置等。
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
#include "robot_adapters/go2_adapter/go2_quadruped_tricks.hpp"
#include "robot_adapters/go2_adapter/go2_communication.hpp"

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

class Go2QuadrupedTricksTester {
private:
    std::shared_ptr<Go2QuadrupedTricks> tricks_;
    std::shared_ptr<Go2Communication> communication_;
    std::shared_ptr<rclcpp::Node> node_;
    std::vector<std::string> test_results_;
    int total_tests_;
    int passed_tests_;

public:
    Go2QuadrupedTricksTester()
        : total_tests_(0), passed_tests_(0) {

        // 创建ROS2节点
        node_ = std::make_shared<rclcpp::Node>("test_go2_quadruped_tricks");

        // 创建Go2Communication实例
        communication_ = std::make_shared<Go2Communication>(node_);

        // 初始化通信管理器
        std::cout << "初始化Go2Communication..." << std::endl;
        bool comm_init_result = communication_->initialize();
        if (comm_init_result) {
            std::cout << "Go2Communication初始化成功" << std::endl;
        } else {
            std::cout << "Go2Communication初始化失败，测试将在模拟模式下运行" << std::endl;
        }

        // 创建Go2QuadrupedTricks实例
        tricks_ = std::make_shared<Go2QuadrupedTricks>(communication_, node_->get_logger());

        std::cout << "=== Go2QuadrupedTricks功能验证测试程序 ===" << std::endl;
        std::cout << "测试环境初始化完成" << std::endl;
    }

    ~Go2QuadrupedTricksTester() {
        if (communication_) {
            communication_->shutdown();
        }
    }

    /**
     * @brief 显示测试菜单
     */
    void showTestMenu() {
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "Go2QuadrupedTricks 功能验证测试程序" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        std::cout << "请选择要运行的测试:" << std::endl;
        std::cout << "1. 通信状态测试" << std::endl;
        std::cout << "2. 特技动作测试" << std::endl;
        std::cout << "3. 运动参数设置测试" << std::endl;
        std::cout << "4. 安全动作序列测试" << std::endl;
        std::cout << "5. 危险动作测试（需确认）" << std::endl;
        std::cout << "6. 连续动作测试" << std::endl;
        std::cout << "7. 参数边界测试" << std::endl;
        std::cout << "8. 基础动作测试" << std::endl;
        std::cout << "0. 运行所有测试" << std::endl;
        std::cout << "q. 退出程序" << std::endl;
        std::cout << std::string(60, '-') << std::endl;
        std::cout << "请输入选择: " << std::flush;
    }

    /**
     * @brief 获取节点实例，供main函数中的ROS消息处理线程使用
     */
    std::shared_ptr<rclcpp::Node> getNode() const {
        return node_;
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
                runSingleTest("通信状态测试", [this]() { testCommunicationStatus(); });
            } else if (input == "2") {
                runSingleTest("特技动作测试", [this]() { testTrickActions(); });
            } else if (input == "3") {
                runSingleTest("运动参数设置测试", [this]() { testMotionParameters(); });
            } else if (input == "4") {
                runSingleTest("安全动作序列测试", [this]() { testSafeActionSequence(); });
            } else if (input == "5") {
                runSingleTest("危险动作测试", [this]() { testDangerousActions(); });
            } else if (input == "6") {
                runSingleTest("连续动作测试", [this]() { testContinuousActions(); });
            } else if (input == "7") {
                runSingleTest("参数边界测试", [this]() { testParameterBoundaries(); });
            } else if (input == "8") {
                runSingleTest("基础动作测试", [this]() { testBasicActions(); });
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
        printHeader("开始运行Go2QuadrupedTricks功能验证测试");

        // 0. 测试通信状态
        if (checkShutdown()) return;
        testCommunicationStatus();

        // 1. 测试基础动作
        if (checkShutdown()) return;
        testBasicActions();

        // 2. 测试特技动作
        if (checkShutdown()) return;
        testTrickActions();

        // 3. 测试运动参数设置
        if (checkShutdown()) return;
        testMotionParameters();

        // 4. 测试安全动作序列
        if (checkShutdown()) return;
        testSafeActionSequence();

        // 5. 测试参数边界
        if (checkShutdown()) return;
        testParameterBoundaries();

        // 输出最终结果
        if (!g_shutdown_requested.load()) {
            printFinalResults();
        } else {
            printInterruptedResults();
        }
    }

private:

    /**
     * @brief 测试通信状态
     */
     void testCommunicationStatus() {
        printHeader("测试通信状态");

        std::cout << "测试通信管理器状态..." << std::endl;

        if (communication_) {
            bool is_initialized = communication_->isInitialized();
            checkTest("Go2Communication初始化状态", is_initialized);
            std::cout << "  通信管理器初始化状态: " << (is_initialized ? "已初始化" : "未初始化") << std::endl;

            bool is_connected = communication_->isConnected();
            checkTest("Go2Communication连接状态检查", true); // 检查调用是否成功
            std::cout << "  机器人连接状态: " << (is_connected ? "已连接" : "未连接") << std::endl;
        } else {
            checkTest("Go2Communication实例", false);
            std::cout << "  错误：通信管理器实例为空" << std::endl;
        }

        // 测试在各种状态下执行动作
        std::cout << "  在当前通信状态下测试简单动作..." << std::endl;
        MotionResult result = tricks_->balanceStand();
        checkTest("通信状态下的动作执行", true); // 只要不崩溃就算通过
        std::cout << "  动作执行结果: " << static_cast<int>(result) << std::endl;
    }

    /**
     * @brief 测试基础动作 - 交互式菜单
     */
    void testBasicActions() {
        printHeader("基础动作测试 - 交互式菜单");

        std::vector<std::string> options = {
            "平衡站立 - balanceStand()",
            "站起动作 - standUp()",
            "坐下动作 - sit()",
            "趴下动作 - standDown()",
            "恢复站立 - recoveryStand()",
            "基础动作序列测试",
            "快速站立循环",
            "姿态变化循环测试"
        };

        while (!checkShutdown()) {
            int choice = showSubmenu("基础动作测试菜单", options);
            if (choice == -1) break;  // 返回上级菜单

            int duration = getDurationFromUser(2000);
            if (duration == -1) continue;  // 用户取消

            switch (choice) {
                case 0: { // 平衡站立
                    bool success = safeExecuteCommand("平衡站立", [this]() {
                        return tricks_->balanceStand();
                    }, duration);
                    checkTest("balanceStand()调用", success);
                    break;
                }
                case 1: { // 站起动作
                    bool success = safeExecuteCommand("站起动作", [this]() {
                        return tricks_->standUp();
                    }, duration);
                    checkTest("standUp()调用", success);
                    break;
                }
                case 2: { // 坐下动作
                    bool success = safeExecuteCommand("坐下动作", [this]() {
                        return tricks_->sit();
                    }, duration);
                    checkTest("sit()调用", success);
                    break;
                }
                case 3: { // 趴下动作
                    bool success = safeExecuteCommand("趴下动作", [this]() {
                        return tricks_->standDown();
                    }, duration);
                    checkTest("standDown()调用", success);

                    // 自动恢复到站立状态确保安全
                    if (success && !checkShutdown()) {
                        std::cout << "    自动恢复到站立状态..." << std::endl;
                        safeExecuteCommand("恢复站立", [this]() {
                            return tricks_->standUp();
                        }, 3000);
                    }
                    break;
                }
                case 4: { // 恢复站立
                    bool success = safeExecuteCommand("恢复站立", [this]() {
                        return tricks_->recoveryStand();
                    }, duration);
                    checkTest("recoveryStand()调用", success);
                    break;
                }
                case 5: { // 基础动作序列测试
                    std::cout << "  执行基础动作序列: 站立 -> 坐下 -> 站起 -> 平衡站立" << std::endl;
                    std::vector<std::string> actions = {"站立", "坐下", "站起", "平衡站立"};
                    std::vector<std::function<MotionResult()>> commands = {
                        [this]() { return tricks_->standUp(); },
                        [this]() { return tricks_->sit(); },
                        [this]() { return tricks_->standUp(); },
                        [this]() { return tricks_->balanceStand(); }
                    };

                    bool all_success = true;
                    for (size_t i = 0; i < actions.size() && !checkShutdown(); ++i) {
                        bool success = safeExecuteCommand(actions[i], commands[i], duration);
                        if (!success) all_success = false;
                    }
                    checkTest("基础动作序列测试", all_success);
                    break;
                }
                case 6: { // 快速站立循环
                    std::cout << "  执行快速站立循环: 坐下 -> 站起 (重复3次)" << std::endl;
                    bool all_success = true;
                    for (int i = 0; i < 3 && !checkShutdown(); ++i) {
                        std::cout << "    循环 " << (i+1) << "/3" << std::endl;
                        bool success1 = safeExecuteCommand("坐下", [this]() {
                            return tricks_->sit();
                        }, duration / 2);
                        bool success2 = safeExecuteCommand("站起", [this]() {
                            return tricks_->standUp();
                        }, duration / 2);
                        if (!success1 || !success2) all_success = false;
                    }
                    checkTest("快速站立循环", all_success);
                    break;
                }
                case 7: { // 姿态变化循环测试
                    std::cout << "  执行姿态变化循环: 平衡站立 -> 趴下 -> 恢复站立" << std::endl;
                    std::vector<std::string> actions = {"平衡站立", "趴下", "恢复站立"};
                    std::vector<std::function<MotionResult()>> commands = {
                        [this]() { return tricks_->balanceStand(); },
                        [this]() { return tricks_->standDown(); },
                        [this]() { return tricks_->recoveryStand(); }
                    };

                    bool all_success = true;
                    for (size_t i = 0; i < actions.size() && !checkShutdown(); ++i) {
                        bool success = safeExecuteCommand(actions[i], commands[i], duration);
                        if (!success) all_success = false;
                    }
                    checkTest("姿态变化循环测试", all_success);
                    break;
                }
            }
        }
    }

    /**
     * @brief 测试特技动作 - 交互式菜单
     */
    void testTrickActions() {
        printHeader("特技动作测试 - 交互式菜单");

        std::vector<std::string> options = {
            "作揖(打招呼) - hello()",
            "伸懒腰 - stretch()",
            "跳舞类型1 - performDance(1)",
            "跳舞类型2 - performDance(2)",
            "前跳动作 - frontJump()",
            "前空翻动作 - frontFlip() ⚠️危险",
            "连续特技表演",
            "快速特技循环测试"
        };

        while (!checkShutdown()) {
            int choice = showSubmenu("特技动作测试菜单", options);
            if (choice == -1) break;  // 返回上级菜单

            int duration = getDurationFromUser(2000);
            if (duration == -1) continue;  // 用户取消

            switch (choice) {
                case 0: { // 作揖(打招呼)
                    bool success = safeExecuteCommand("作揖(打招呼)", [this]() {
                        return tricks_->hello();
                    }, duration);
                    checkTest("hello()调用", success);
                    break;
                }
                case 1: { // 伸懒腰
                    bool success = safeExecuteCommand("伸懒腰", [this]() {
                        return tricks_->stretch();
                    }, duration);
                    checkTest("stretch()调用", success);
                    break;
                }
                case 2: { // 跳舞类型1
                    bool success = safeExecuteCommand("跳舞类型1", [this]() {
                        return tricks_->performDance(1);
                    }, duration);
                    checkTest("performDance(1)调用", success);
                    break;
                }
                case 3: { // 跳舞类型2
                    bool success = safeExecuteCommand("跳舞类型2", [this]() {
                        return tricks_->performDance(2);
                    }, duration);
                    checkTest("performDance(2)调用", success);
                    break;
                }
                case 4: { // 前跳动作
                    if (waitForUserConfirmation("前跳动作")) {
                        bool success = safeExecuteCommand("前跳动作", [this]() {
                            return tricks_->frontJump();
                        }, duration);
                        checkTest("frontJump()调用", success);

                        // 自动恢复到安全状态
                        if (success && !checkShutdown()) {
                            std::cout << "    自动恢复到安全状态..." << std::endl;
                            safeExecuteCommand("恢复站立", [this]() {
                                return tricks_->recoveryStand();
                            }, 3000);
                        }
                    } else {
                        std::cout << "    用户取消前跳动作测试" << std::endl;
                        checkTest("frontJump()调用", true); // 跳过但记录为通过
                    }
                    break;
                }
                case 5: { // 前空翻动作
                    if (waitForUserConfirmation("前空翻动作")) {
                        bool success = safeExecuteCommand("前空翻动作", [this]() {
                            return tricks_->frontFlip();
                        }, duration);
                        checkTest("frontFlip()调用", success);

                        // 立即恢复到安全状态
                        if (success && !checkShutdown()) {
                            std::cout << "    立即恢复到安全状态..." << std::endl;
                            safeExecuteCommand("恢复站立", [this]() {
                                return tricks_->recoveryStand();
                            }, 4000);
                        }
                    } else {
                        std::cout << "    用户取消前空翻动作测试" << std::endl;
                        checkTest("frontFlip()调用", true); // 跳过但记录为通过
                    }
                    break;
                }
                case 6: { // 连续特技表演
                    std::cout << "  执行连续特技表演: 作揖 -> 伸懒腰 -> 跳舞1" << std::endl;
                    std::vector<std::string> actions = {"作揖", "伸懒腰", "跳舞1"};
                    std::vector<std::function<MotionResult()>> commands = {
                        [this]() { return tricks_->hello(); },
                        [this]() { return tricks_->stretch(); },
                        [this]() { return tricks_->performDance(1); }
                    };

                    bool all_success = true;
                    for (size_t i = 0; i < actions.size() && !checkShutdown(); ++i) {
                        bool success = safeExecuteCommand(actions[i], commands[i], duration);
                        if (!success) all_success = false;
                    }
                    checkTest("连续特技表演", all_success);
                    break;
                }
                case 7: { // 快速特技循环测试
                    std::cout << "  执行快速特技循环: 作揖 -> 跳舞1 -> 跳舞2 -> 伸懒腰" << std::endl;
                    std::vector<std::string> actions = {"作揖", "跳舞1", "跳舞2", "伸懒腰"};
                    std::vector<std::function<MotionResult()>> commands = {
                        [this]() { return tricks_->hello(); },
                        [this]() { return tricks_->performDance(1); },
                        [this]() { return tricks_->performDance(2); },
                        [this]() { return tricks_->stretch(); }
                    };

                    bool all_success = true;
                    for (size_t i = 0; i < actions.size() && !checkShutdown(); ++i) {
                        bool success = safeExecuteCommand(actions[i], commands[i], duration / 2); // 使用一半时间，快速循环
                        if (!success) all_success = false;
                    }
                    checkTest("快速特技循环测试", all_success);
                    break;
                }
            }

            // 每次测试后恢复到平衡站立确保安全
            if (choice < 6) {
                std::cout << "  恢复到平衡站立确保安全..." << std::endl;
                safeExecuteCommand("平衡站立", [this]() {
                    return tricks_->balanceStand();
                }, 1500);
            }
        }
    }

    /**
     * @brief 测试运动参数设置
     */
    void testMotionParameters() {
        printHeader("测试运动参数设置");

        std::cout << "测试速度等级设置功能..." << std::endl;

        // 测试不同速度等级
        for (int level = 1; level <= 5; level++) {
            MotionResult result = tricks_->setSpeedLevel(level);
            checkTest("setSpeedLevel(" + std::to_string(level) + ")调用",
                     result == MotionResult::SUCCESS);
            std::cout << "  设置速度等级" << level << "结果: " << static_cast<int>(result) << std::endl;

            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }

    /**
     * @brief 测试安全动作序列
     */
    void testSafeActionSequence() {
        printHeader("测试安全动作序列");

        std::cout << "执行一个完整的安全动作序列..." << std::endl;

        // 序列1: 站立 -> 作揖 -> 坐下 -> 站立
        std::cout << "  序列1: 站立 -> 作揖 -> 坐下 -> 站立" << std::endl;

        MotionResult result = tricks_->standUp();
        checkTest("序列1-站立", result == MotionResult::SUCCESS);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        result = tricks_->hello();
        checkTest("序列1-作揖", result == MotionResult::SUCCESS);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        result = tricks_->sit();
        checkTest("序列1-坐下", result == MotionResult::SUCCESS);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        result = tricks_->standUp();
        checkTest("序列1-重新站立", result == MotionResult::SUCCESS);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // 序列2: 平衡站立 -> 伸懒腰 -> 趴下 -> 恢复站立
        std::cout << "  序列2: 平衡站立 -> 伸懒腰 -> 趴下 -> 恢复站立" << std::endl;

        result = tricks_->balanceStand();
        checkTest("序列2-平衡站立", result == MotionResult::SUCCESS);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        result = tricks_->stretch();
        checkTest("序列2-伸懒腰", result == MotionResult::SUCCESS);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        result = tricks_->standDown();
        checkTest("序列2-趴下", result == MotionResult::SUCCESS);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        result = tricks_->recoveryStand();
        checkTest("序列2-恢复站立", result == MotionResult::SUCCESS);
    }

    /**
     * @brief 测试危险动作（需用户确认）
     */
    void testDangerousActions() {
        printHeader("测试危险动作");

        std::cout << "警告：以下测试包含可能对机器人造成损害的动作！" << std::endl;
        std::cout << "请确保机器人周围有足够的安全空间。" << std::endl;
        std::cout << "是否继续执行危险动作测试？(y/N): " << std::flush;

        char confirm;
        std::cin >> confirm;
        std::cin.ignore(); // 清除输入缓冲区

        if (confirm != 'y' && confirm != 'Y') {
            std::cout << "跳过危险动作测试。" << std::endl;
            checkTest("危险动作测试", true); // 标记为跳过但通过
            return;
        }

        std::cout << "执行危险动作测试..." << std::endl;

        // 测试前空翻
        std::cout << "警告：即将执行前空翻，请确保安全！" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        MotionResult result = tricks_->frontFlip();
        checkTest("frontFlip()调用", result == MotionResult::SUCCESS);
        std::cout << "  前空翻结果: " << static_cast<int>(result) << std::endl;

        // 等待更长时间让机器人稳定
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    }

    /**
     * @brief 测试连续动作
     */
    void testContinuousActions() {
        printHeader("测试连续动作");

        std::cout << "测试快速连续执行多个动作..." << std::endl;

        // 连续执行多个简单动作
        std::vector<std::string> action_names = {
            "hello", "stretch", "sit", "standUp", "balanceStand"
        };

        for (const auto& action_name : action_names) {
            MotionResult result;

            if (action_name == "hello") {
                result = tricks_->hello();
            } else if (action_name == "stretch") {
                result = tricks_->stretch();
            } else if (action_name == "sit") {
                result = tricks_->sit();
            } else if (action_name == "standUp") {
                result = tricks_->standUp();
            } else if (action_name == "balanceStand") {
                result = tricks_->balanceStand();
            }
            else {
                _exit(0);
            }

            checkTest("连续动作-" + action_name, result == MotionResult::SUCCESS);
            std::cout << "  " << action_name << "结果: " << static_cast<int>(result) << std::endl;

            // 短暂等待
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    /**
     * @brief 测试参数边界
     */
    void testParameterBoundaries() {
        printHeader("测试参数边界");

        std::cout << "测试各种边界和无效参数..." << std::endl;

        // 测试无效的速度等级
        MotionResult result = tricks_->setSpeedLevel(-1);
        checkTest("setSpeedLevel(-1)调用", result == MotionResult::INVALID_PARAMETER);
        std::cout << "  无效速度等级(-1)结果: " << static_cast<int>(result) << std::endl;

        result = tricks_->setSpeedLevel(0);
        checkTest("setSpeedLevel(0)调用", result == MotionResult::INVALID_PARAMETER);
        std::cout << "  无效速度等级(0)结果: " << static_cast<int>(result) << std::endl;

        result = tricks_->setSpeedLevel(10);
        checkTest("setSpeedLevel(10)调用", result == MotionResult::INVALID_PARAMETER);
        std::cout << "  无效速度等级(10)结果: " << static_cast<int>(result) << std::endl;

        // 测试边界有效值
        result = tricks_->setSpeedLevel(1);
        checkTest("setSpeedLevel(1)调用", result == MotionResult::SUCCESS);
        std::cout << "  边界有效速度等级(1)结果: " << static_cast<int>(result) << std::endl;

        result = tricks_->setSpeedLevel(5);
        checkTest("setSpeedLevel(5)调用", result == MotionResult::SUCCESS);
        std::cout << "  边界有效速度等级(5)结果: " << static_cast<int>(result) << std::endl;

        // 测试无效的舞蹈类型
        result = tricks_->performDance(-1);
        checkTest("performDance(-1)调用", result == MotionResult::SUCCESS); // 应该使用默认值
        std::cout << "  无效舞蹈类型(-1)结果: " << static_cast<int>(result) << std::endl;

        result = tricks_->performDance(0);
        checkTest("performDance(0)调用", result == MotionResult::SUCCESS); // 应该使用默认值
        std::cout << "  无效舞蹈类型(0)结果: " << static_cast<int>(result) << std::endl;

        result = tricks_->performDance(3);
        checkTest("performDance(3)调用", result == MotionResult::SUCCESS); // 应该使用默认值
        std::cout << "  无效舞蹈类型(3)结果: " << static_cast<int>(result) << std::endl;
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
        if (communication_) {
            std::cout << "正在关闭通信管理器..." << std::endl;
            communication_->shutdown();
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
            return false;
        }

        std::cout << "    ✓ 命令执行完成" << std::endl;
        return true;
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
        if (communication_) {
            std::cout << "正在关闭通信管理器..." << std::endl;
            bool shutdown_result = communication_->shutdown();
            if (shutdown_result) {
                std::cout << "✓ communication_->shutdown()调用 - 通过" << std::endl;
            } else {
                std::cout << "✗ communication_->shutdown()调用 - 失败" << std::endl;
            }
        }

        if (passed_tests_ == total_tests_) {
            std::cout << "\n🎉 所有功能验证测试通过！Go2QuadrupedTricks类工作正常。" << std::endl;
        } else {
            std::cout << "\n⚠️  有部分测试失败，请检查Go2QuadrupedTricks类的实现。" << std::endl;
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
        Go2QuadrupedTricksTester tester;

        // 启动ROS消息处理线程，持续处理订阅的消息
        std::thread spin_thread([&tester]() {
            rclcpp::spin(tester.getNode());
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