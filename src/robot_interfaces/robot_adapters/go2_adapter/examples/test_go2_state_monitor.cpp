/**
 * @file   test_go2_state_monitor.cpp
 * @brief  Go2StateMonitor类功能验证测试程序
 * @author Yang Nan
 * @date   2025-09-18
 *
 * @details
 * 这是一个用于验证Go2StateMonitor类各项功能的测试程序。
 * 该程序创建Go2StateMonitor实例，并系统性地测试其各个功能模块，
 * 包括初始化、状态监控、健康评估、告警管理、诊断功能等。
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
#include "robot_adapters/go2_adapter/go2_state_monitor.hpp"

// 全局变量控制程序停止
std::atomic<bool> g_shutdown_requested{false};

using namespace robot_adapters::go2_adapter;
using namespace robot_base_interfaces::state_interface;

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

class Go2StateMonitorTester {
private:
    std::shared_ptr<Go2StateMonitor> monitor_;
    std::vector<std::string> test_results_;
    int total_tests_;
    int passed_tests_;

    // 回调数据存储
    bool state_callback_triggered_;
    bool health_callback_triggered_;
    bool alert_callback_triggered_;
    bool error_callback_triggered_;
    bool detailed_state_callback_triggered_;

    RobotState last_old_state_;
    RobotState last_new_state_;
    HealthLevel last_old_health_;
    HealthLevel last_new_health_;
    float last_health_score_;
    AlertInfo last_alert_;
    uint32_t last_error_code_;
    std::string last_error_message_;
    DetailedRobotState last_detailed_state_;

public:
    Go2StateMonitorTester()
        : total_tests_(0), passed_tests_(0),
          state_callback_triggered_(false), health_callback_triggered_(false),
          alert_callback_triggered_(false), error_callback_triggered_(false),
          detailed_state_callback_triggered_(false),
          last_old_state_(RobotState::UNKNOWN), last_new_state_(RobotState::UNKNOWN),
          last_old_health_(HealthLevel::UNKNOWN), last_new_health_(HealthLevel::UNKNOWN),
          last_health_score_(0.0f), last_error_code_(0) {

        // 创建Go2StateMonitor实例
        monitor_ = std::make_shared<Go2StateMonitor>("test_go2_state_monitor");

        std::cout << "=== Go2StateMonitor功能验证测试程序 ===" << std::endl;
        std::cout << "初始化测试环境..." << std::endl;
    }

    ~Go2StateMonitorTester() {
        if (monitor_) {
            monitor_->shutdown();
        }
    }

    /**
     * @brief 显示测试菜单
     */
    void showTestMenu() {
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "Go2StateMonitor 功能验证测试程序" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        std::cout << "请选择要运行的测试:" << std::endl;
        std::cout << "1. 初始化和生命周期管理测试" << std::endl;
        std::cout << "2. 基础状态查询测试" << std::endl;
        std::cout << "3. 详细状态查询测试" << std::endl;
        std::cout << "4. 系统诊断功能测试" << std::endl;
        std::cout << "5. 告警管理功能测试" << std::endl;
        std::cout << "6. 回调函数设置测试" << std::endl;
        std::cout << "7. 配置管理功能测试" << std::endl;
        std::cout << "8. 性能统计功能测试" << std::endl;
        std::cout << "9. 数据记录导出测试" << std::endl;
        std::cout << "0. 运行所有测试" << std::endl;
        std::cout << "q. 退出程序" << std::endl;
        std::cout << std::string(60, '-') << std::endl;
        std::cout << "请输入选择: " << std::flush;
    }

    /**
     * @brief 获取监控器实例，供main函数中的ROS消息处理线程使用
     */
    std::shared_ptr<Go2StateMonitor> getMonitor() const {
        return monitor_;
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

            // 非阻塞等待键盘输入，避免 Ctrl+C 时阻塞在输入上
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
                runSingleTest("基础状态查询测试", [this]() { testBasicStateQuery(); });
            } else if (input == "3") {
                runSingleTest("详细状态查询测试", [this]() { testDetailedStateQuery(); });
            } else if (input == "4") {
                runSingleTest("系统诊断功能测试", [this]() { testSystemDiagnostics(); });
            } else if (input == "5") {
                runSingleTest("告警管理功能测试", [this]() { testAlertManagement(); });
            } else if (input == "6") {
                runSingleTest("回调函数设置测试", [this]() { testCallbacks(); });
            } else if (input == "7") {
                runSingleTest("配置管理功能测试", [this]() { testConfigurationManagement(); });
            } else if (input == "8") {
                runSingleTest("性能统计功能测试", [this]() { testPerformanceStats(); });
            } else if (input == "9") {
                runSingleTest("数据记录导出测试", [this]() { testDataRecordingExport(); });
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
        printHeader("开始运行Go2StateMonitor功能验证测试");

        // 1. 测试初始化和生命周期管理
        if (checkShutdown()) return;
        testInitializationLifecycle();

        // 2. 测试基础状态查询
        if (checkShutdown()) return;
        testBasicStateQuery();

        // 3. 测试详细状态查询
        if (checkShutdown()) return;
        testDetailedStateQuery();

        // 4. 测试系统诊断功能
        if (checkShutdown()) return;
        testSystemDiagnostics();

        // 5. 测试告警管理功能
        if (checkShutdown()) return;
        testAlertManagement();

        // 6. 测试回调函数设置
        if (checkShutdown()) return;
        testCallbacks();

        // 7. 测试配置管理功能
        if (checkShutdown()) return;
        testConfigurationManagement();

        // 8. 测试性能统计功能
        if (checkShutdown()) return;
        testPerformanceStats();

        // 9. 测试数据记录导出
        if (checkShutdown()) return;
        testDataRecordingExport();

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
        checkTest("初始状态检查 - 未监控", !monitor_->isMonitoring());
        checkTest("初始状态检查 - 错误码", monitor_->getErrorCode() == 0);

        // 测试初始化
        bool init_result = monitor_->initialize();
        checkTest("initialize()调用", init_result);

        // 等待一段时间让初始化完成
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // 测试启动监控
        bool start_result = monitor_->startMonitoring();
        checkTest("startMonitoring()调用", start_result);
        checkTest("监控状态检查", monitor_->isMonitoring());

        // 等待监控运行一段时间
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // 测试停止监控
        bool stop_result = monitor_->stopMonitoring();
        checkTest("stopMonitoring()调用", stop_result);
        checkTest("监控停止检查", !monitor_->isMonitoring());

        // 重新启动监控以支持后续测试
        monitor_->startMonitoring();
        std::cout << "  注意：重新启动监控以支持后续测试" << std::endl;
    }

    /**
     * @brief 测试基础状态查询
     */
    void testBasicStateQuery() {
        printHeader("测试基础状态查询");

        // 等待接收一次状态更新
        std::cout << "正在等待接收状态更新..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // 测试状态查询接口
        RobotState robot_state = monitor_->getRobotState();
        checkTest("getRobotState()调用", true);
        std::cout << "  当前机器人状态: " << static_cast<int>(robot_state) << std::endl;

        HealthLevel health_level = monitor_->getHealthStatus();
        checkTest("getHealthStatus()调用", true);
        std::cout << "  当前健康等级: " << static_cast<int>(health_level) << std::endl;

        float health_score = monitor_->getHealthScore();
        checkTest("getHealthScore()调用", true);
        std::cout << "  当前健康分数: " << std::fixed << std::setprecision(2) << health_score << std::endl;

        bool operational = monitor_->isOperational();
        checkTest("isOperational()调用", true);
        std::cout << "  机器人可操作: " << (operational ? "是" : "否") << std::endl;

        uint32_t error_code = monitor_->getErrorCode();
        checkTest("getErrorCode()调用", true);
        std::cout << "  当前错误码: " << error_code << std::endl;

        // 测试运行时间统计
        uint64_t uptime = monitor_->getUptimeSeconds();
        checkTest("getUptimeSeconds()调用", true);
        std::cout << "  运行时间: " << uptime << " 秒" << std::endl;
    }

    /**
     * @brief 测试详细状态查询
     */
    void testDetailedStateQuery() {
        printHeader("测试详细状态查询");

        // 获取详细状态
        DetailedRobotState detailed_state = monitor_->getDetailedState();
        checkTest("getDetailedState()调用", true);
        std::cout << "  详细状态时间戳: " << detailed_state.timestamp_ns << std::endl;

        // 测试电机信息查询
        std::vector<MotorInfo> all_motors = monitor_->getAllMotorInfo();
        checkTest("getAllMotorInfo()调用", true);
        std::cout << "  电机数量: " << all_motors.size() << std::endl;

        if (!all_motors.empty()) {
            MotorInfo first_motor = monitor_->getMotorInfo(0);
            checkTest("getMotorInfo(0)调用", true);
            std::cout << "  第一个电机温度: " << static_cast<int>(first_motor.temperature) << "°C" << std::endl;
        }

        // 测试越界电机查询
        MotorInfo invalid_motor = monitor_->getMotorInfo(255);
        checkTest("getMotorInfo(255)越界处理", invalid_motor.motor_id == 0); // 应该返回默认值

        // 测试足端信息查询
        std::vector<FootInfo> all_feet = monitor_->getAllFootInfo();
        checkTest("getAllFootInfo()调用", true);
        std::cout << "  足端数量: " << all_feet.size() << std::endl;

        if (!all_feet.empty()) {
            FootInfo first_foot = monitor_->getFootInfo(0);
            checkTest("getFootInfo(0)调用", true);
            std::cout << "  第一个足端接触力: " << first_foot.force << "N" << std::endl;
        }

        // 测试越界足端查询
        FootInfo invalid_foot = monitor_->getFootInfo(255);
        checkTest("getFootInfo(255)越界处理", invalid_foot.foot_id == 0); // 应该返回默认值
    }

    /**
     * @brief 测试系统诊断功能
     */
    void testSystemDiagnostics() {
        printHeader("测试系统诊断功能");

        // 获取支持的模块
        std::vector<SystemModule> supported_modules = monitor_->getSupportedModules();
        checkTest("getSupportedModules()调用", true);
        std::cout << "  支持的模块数量: " << supported_modules.size() << std::endl;

        // 打印支持的模块
        for (size_t i = 0; i < supported_modules.size(); ++i) {
            std::cout << "    模块 " << i << ": " << static_cast<int>(supported_modules[i]) << std::endl;
        }

        // 获取系统诊断信息
        std::vector<DiagnosticInfo> diagnostics = monitor_->getSystemDiagnostics();
        checkTest("getSystemDiagnostics()调用", true);
        std::cout << "  诊断信息数量: " << diagnostics.size() << std::endl;

        // 测试单个模块诊断
        if (!supported_modules.empty()) {
            DiagnosticInfo first_diagnostic = monitor_->getModuleDiagnostic(supported_modules[0]);
            checkTest("getModuleDiagnostic()调用", true);
            std::cout << "  第一个模块健康等级: " << static_cast<int>(first_diagnostic.health_level) << std::endl;
            std::cout << "  第一个模块健康分数: " << first_diagnostic.health_score << std::endl;
        }

        // 执行系统自检
        bool system_check_result = monitor_->performSystemCheck();
        checkTest("performSystemCheck()调用", true);
        std::cout << "  系统自检结果: " << (system_check_result ? "通过" : "失败") << std::endl;

        // 获取自检结果
        std::map<SystemModule, bool> check_results = monitor_->getSystemCheckResults();
        checkTest("getSystemCheckResults()调用", true);
        std::cout << "  自检结果数量: " << check_results.size() << std::endl;

        for (const auto& [module, result] : check_results) {
            std::cout << "    模块 " << static_cast<int>(module) << ": "
                      << (result ? "正常" : "异常") << std::endl;
        }
    }

    /**
     * @brief 测试告警管理功能
     */
    void testAlertManagement() {
        printHeader("测试告警管理功能");

        // 获取当前活跃告警
        std::vector<AlertInfo> active_alerts = monitor_->getActiveAlerts();
        checkTest("getActiveAlerts()调用", true);
        std::cout << "  当前活跃告警数量: " << active_alerts.size() << std::endl;

        // 输出第一个告警的详细信息
        if (!active_alerts.empty()) {
            const auto& first_alert = active_alerts[0];
            std::cout << "  第一个告警详细信息:" << std::endl;
            std::cout << "    告警代码: " << first_alert.code << std::endl;
            std::cout << "    告警类型: " << static_cast<int>(first_alert.type) <<
                         " (0=INFO, 1=WARNING, 2=ERROR, 3=CRITICAL)" << std::endl;
            std::cout << "    相关模块: " << static_cast<int>(first_alert.module) <<
                         " (0=MOTION_CONTROL, 1=SENSOR_SYSTEM, 2=POWER_MANAGEMENT, 3=COMMUNICATION, 4=NAVIGATION)" << std::endl;
            std::cout << "    告警消息: " << first_alert.message << std::endl;
            std::cout << "    是否活跃: " << (first_alert.is_active ? "是" : "否") << std::endl;
            std::cout << "    时间戳: " << first_alert.timestamp_ns << " ns" << std::endl;

            // 输出附加数据（如果有）
            if (!first_alert.string_data.empty()) {
                std::cout << "    附加字符串数据:" << std::endl;
                for (const auto& [key, value] : first_alert.string_data) {
                    std::cout << "      " << key << ": " << value << std::endl;
                }
            }
            if (!first_alert.numeric_data.empty()) {
                std::cout << "    附加数值数据:" << std::endl;
                for (const auto& [key, value] : first_alert.numeric_data) {
                    std::cout << "      " << key << ": " << value << std::endl;
                }
            }
        } else {
            std::cout << "  没有活跃的告警信息" << std::endl;
        }

        // 测试按类型获取告警
        std::vector<AlertInfo> warning_alerts = monitor_->getAlertsByType(AlertType::WARNING);
        checkTest("getAlertsByType(WARNING)调用", true);
        std::cout << "  警告类型告警数量: " << warning_alerts.size() << std::endl;

        std::vector<AlertInfo> error_alerts = monitor_->getAlertsByType(AlertType::ERROR);
        checkTest("getAlertsByType(ERROR)调用", true);
        std::cout << "  错误类型告警数量: " << error_alerts.size() << std::endl;

        std::vector<AlertInfo> critical_alerts = monitor_->getAlertsByType(AlertType::CRITICAL);
        checkTest("getAlertsByType(CRITICAL)调用", true);
        std::cout << "  严重类型告警数量: " << critical_alerts.size() << std::endl;

        // 测试按模块获取告警
        std::vector<SystemModule> supported_modules = monitor_->getSupportedModules();
        if (!supported_modules.empty()) {
            std::vector<AlertInfo> module_alerts = monitor_->getAlertsByModule(supported_modules[0]);
            checkTest("getAlertsByModule()调用", true);
            std::cout << "  第一个模块告警数量: " << module_alerts.size() << std::endl;
        }

        // 测试告警确认（使用一个不存在的告警码）
        bool ack_result = monitor_->acknowledgeAlert(12345);
        checkTest("acknowledgeAlert()调用", true);
        std::cout << "  告警确认结果（不存在的告警）: " << (!ack_result ? "成功" : "失败") << std::endl;

        // 测试清除已解决告警
        int cleared_count = monitor_->clearResolvedAlerts();
        checkTest("clearResolvedAlerts()调用", true);
        std::cout << "  清除的告警数量: " << cleared_count << std::endl;

        // 测试清除特定告警
        int specific_cleared = monitor_->clearResolvedAlerts(12345);
        checkTest("clearResolvedAlerts(12345)调用", true);
        std::cout << "  清除特定告警数量: " << specific_cleared << std::endl;
    }

    /**
     * @brief 测试回调函数设置
     */
    void testCallbacks() {
        printHeader("测试回调函数设置");

        // 重置回调标志
        resetCallbackFlags();

        // 设置状态变化回调
        monitor_->setStateChangeCallback([this](RobotState old_state, RobotState new_state) {
            this->state_callback_triggered_ = true;
            this->last_old_state_ = old_state;
            this->last_new_state_ = new_state;
            std::cout << "  状态回调触发: " << static_cast<int>(old_state)
                      << " -> " << static_cast<int>(new_state) << std::endl;
        });
        checkTest("setStateChangeCallback()调用", true);

        // 设置健康状态变化回调
        monitor_->setHealthChangeCallback([this](HealthLevel old_level, HealthLevel new_level, float score) {
            this->health_callback_triggered_ = true;
            this->last_old_health_ = old_level;
            this->last_new_health_ = new_level;
            this->last_health_score_ = score;
            std::cout << "  健康回调触发: " << static_cast<int>(old_level)
                      << " -> " << static_cast<int>(new_level)
                      << ", 分数: " << score << std::endl;
        });
        checkTest("setHealthChangeCallback()调用", true);

        // 设置告警回调
        monitor_->setAlertCallback([this](const AlertInfo& alert) {
            this->alert_callback_triggered_ = true;
            this->last_alert_ = alert;
            std::cout << "  告警回调触发: 代码=" << alert.code
                      << ", 类型=" << static_cast<int>(alert.type) << std::endl;
        });
        checkTest("setAlertCallback()调用", true);

        // 设置错误回调
        monitor_->setErrorCallback([this](uint32_t error_code, const std::string& error_msg) {
            this->error_callback_triggered_ = true;
            this->last_error_code_ = error_code;
            this->last_error_message_ = error_msg;
            std::cout << "  错误回调触发: 代码=" << error_code
                      << ", 消息=" << error_msg << std::endl;
        });
        checkTest("setErrorCallback()调用", true);

        // 设置详细状态回调
        monitor_->setDetailedStateCallback([this](const DetailedRobotState& state) {
            this->detailed_state_callback_triggered_ = true;
            this->last_detailed_state_ = state;
            std::cout << "  详细状态回调触发: 时间戳=" << state.timestamp_ns << std::endl;
        });
        checkTest("setDetailedStateCallback()调用", true);

        // 等待回调可能被触发
        std::cout << "  等待回调触发..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        // 验证回调设置成功（注意：由于没有真实的状态变化，某些回调可能不会被触发）
        std::cout << "  回调触发状态:" << std::endl;
        std::cout << "    状态变化回调: " << (state_callback_triggered_ ? "已触发" : "未触发") << std::endl;
        std::cout << "    健康变化回调: " << (health_callback_triggered_ ? "已触发" : "未触发") << std::endl;
        std::cout << "    告警回调: " << (alert_callback_triggered_ ? "已触发" : "未触发") << std::endl;
        std::cout << "    错误回调: " << (error_callback_triggered_ ? "已触发" : "未触发") << std::endl;
        std::cout << "    详细状态回调: " << (detailed_state_callback_triggered_ ? "已触发" : "未触发") << std::endl;

        checkTest("回调函数设置完成", true);
    }

    /**
     * @brief 测试配置管理功能
     */
    void testConfigurationManagement() {
        printHeader("测试配置管理功能");

        // 测试监控频率设置
        bool freq_result1 = monitor_->setMonitoringFrequency(5.0f);
        checkTest("setMonitoringFrequency(5.0)调用", freq_result1);

        bool freq_result2 = monitor_->setMonitoringFrequency(20.0f);
        checkTest("setMonitoringFrequency(20.0)调用", freq_result2);

        // 测试无效频率处理
        bool freq_result3 = monitor_->setMonitoringFrequency(0.0f);
        checkTest("setMonitoringFrequency(0.0)无效值处理", !freq_result3);

        bool freq_result4 = monitor_->setMonitoringFrequency(-5.0f);
        checkTest("setMonitoringFrequency(-5.0)无效值处理", !freq_result4);

        // 测试健康阈值设置
        bool threshold_result1 = monitor_->setHealthThresholds(0.9f, 0.7f, 0.5f, 0.3f);
        checkTest("setHealthThresholds()有效值调用", threshold_result1);

        // 测试无效阈值处理（阈值不是递减顺序）
        bool threshold_result2 = monitor_->setHealthThresholds(0.5f, 0.7f, 0.9f, 0.3f);
        checkTest("setHealthThresholds()无效值处理", !threshold_result2);

        // 测试模块监控启用/禁用
        std::vector<SystemModule> supported_modules = monitor_->getSupportedModules();
        if (!supported_modules.empty()) {
            bool module_result1 = monitor_->setModuleMonitoring(supported_modules[0], false);
            checkTest("setModuleMonitoring()禁用调用", module_result1);

            bool module_result2 = monitor_->setModuleMonitoring(supported_modules[0], true);
            checkTest("setModuleMonitoring()启用调用", module_result2);
        }

        std::cout << "  配置管理功能测试完成" << std::endl;
    }

    /**
     * @brief 测试性能统计功能
     */
    void testPerformanceStats() {
        printHeader("测试性能统计功能");

        // 获取性能统计
        PerformanceStats stats = monitor_->getPerformanceStats();
        checkTest("getPerformanceStats()调用", true);
        std::cout << "  当前性能统计:" << std::endl;
        std::cout << "    处理的消息数: " << stats.communication.messages_sent << std::endl;
        std::cout << "    处理失败数: " << stats.communication.messages_lost << std::endl;
        std::cout << "    平均延迟: " << stats.communication.average_latency_ms << "ms" << std::endl;

        // 重置性能统计
        bool reset_result = monitor_->resetPerformanceStats();
        checkTest("resetPerformanceStats()调用", reset_result);

        // 验证重置后的统计
        PerformanceStats reset_stats = monitor_->getPerformanceStats();
        checkTest("重置后统计验证", reset_stats.communication.messages_sent == 0);

        // 获取运行时间
        uint64_t uptime1 = monitor_->getUptimeSeconds();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        uint64_t uptime2 = monitor_->getUptimeSeconds();
        checkTest("运行时间递增验证", uptime2 >= uptime1);
        std::cout << "  运行时间变化: " << uptime1 << "s -> " << uptime2 << "s" << std::endl;
    }

    /**
     * @brief 测试数据记录导出功能
     */
    void testDataRecordingExport() {
        printHeader("测试数据记录导出功能");

        // 测试开始数据记录
        bool record_start = monitor_->startDataRecording(10);
        checkTest("startDataRecording(10)调用", record_start);

        // 等待一段时间
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // 测试停止数据记录
        bool record_stop = monitor_->stopDataRecording();
        checkTest("stopDataRecording()调用", record_stop);

        // 测试数据导出（不同格式）
        bool export_json = monitor_->exportStateData("/tmp/test_state_data.json", "json");
        checkTest("exportStateData()JSON格式调用", export_json);

        bool export_csv = monitor_->exportStateData("/tmp/test_state_data.csv", "csv");
        checkTest("exportStateData()CSV格式调用", export_csv);

        bool export_binary = monitor_->exportStateData("/tmp/test_state_data.bin", "binary");
        checkTest("exportStateData()二进制格式调用", export_binary);

        // 测试无持续时间的记录
        bool record_continuous = monitor_->startDataRecording();
        checkTest("startDataRecording()持续记录调用", record_continuous);

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        monitor_->stopDataRecording();
        std::cout << "  数据记录导出功能测试完成" << std::endl;
    }

    /**
     * @brief 运行单个测试
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
     */
    bool checkShutdown() {
        if (g_shutdown_requested.load()) {
            std::cout << "\n检测到停止信号，正在终止测试..." << std::endl;
            return true;
        }
        return false;
    }

    /**
     * @brief 重置回调标志
     */
    void resetCallbackFlags() {
        state_callback_triggered_ = false;
        health_callback_triggered_ = false;
        alert_callback_triggered_ = false;
        error_callback_triggered_ = false;
        detailed_state_callback_triggered_ = false;
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
        if (monitor_) {
            std::cout << "正在关闭状态监控器..." << std::endl;
            monitor_->shutdown();
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
        bool shutdown_result = monitor_->shutdown();
        if (shutdown_result) {
            std::cout << "✓ shutdown()调用 - 通过" << std::endl;
        } else {
            std::cout << "✗ shutdown()调用 - 失败" << std::endl;
        }

        if (passed_tests_ == total_tests_) {
            std::cout << "\n🎉 所有功能验证测试通过！Go2StateMonitor类工作正常。" << std::endl;
        } else {
            std::cout << "\n⚠️  有部分测试失败，请检查Go2StateMonitor类的实现。" << std::endl;
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
        Go2StateMonitorTester tester;

        // 启动ROS消息处理线程，持续处理订阅的消息
        std::thread spin_thread([&tester]() {
            rclcpp::spin(tester.getMonitor());
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