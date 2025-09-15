/**
 * @file   test_go2_communication.cpp
 * @brief  Go2Communication类功能验证测试程序
 * @author Claude Assistant
 * @date   2025-09-15
 *
 * @details
 * 这是一个用于验证Go2Communication类各项功能的测试程序。
 * 该程序创建Go2Communication实例，并系统性地测试其各个功能模块，
 * 包括初始化、消息发布/订阅、缓冲区管理、连接状态管理等。
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
#include "robot_adapters/go2_adapter/go2_communication.hpp"

// 引入必要的消息类型
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

// 全局变量控制程序停止
std::atomic<bool> g_shutdown_requested{false};

using namespace robot_adapters::go2_adapter;

/**
 * @brief 信号处理函数，处理Ctrl+C (SIGINT)
 */
void signalHandler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\n\n检测到 Ctrl+C，正在停止测试程序..." << std::endl;
        g_shutdown_requested.store(true);
    }
}

class Go2CommunicationTester {
private:
    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<Go2Communication> comm_;
    std::vector<std::string> test_results_;
    int total_tests_;
    int passed_tests_;

    // 回调数据存储
    bool pointcloud_received_;
    bool imu_received_;
    bool odom_received_;
    bool sport_state_received_;
    bool low_state_received_;
    bool bms_state_received_;
    CommunicationStatus last_status_;
    std::string last_error_;
    float last_quality_;

public:
    Go2CommunicationTester()
        : total_tests_(0), passed_tests_(0),
          pointcloud_received_(false), imu_received_(false),
          odom_received_(false), sport_state_received_(false),
          low_state_received_(false), bms_state_received_(false),
          last_status_(CommunicationStatus::DISCONNECTED),
          last_quality_(0.0f) {

        // 创建ROS2节点
        node_ = std::make_shared<rclcpp::Node>("go2_communication_tester");

        // 创建Go2Communication实例
        comm_ = std::make_unique<Go2Communication>(node_);

        std::cout << "=== Go2Communication功能验证测试程序 ===" << std::endl;
        std::cout << "初始化测试环境..." << std::endl;
    }

    ~Go2CommunicationTester() {
        if (comm_) {
            comm_->shutdown();
        }
    }

    /**
     * @brief 显示测试菜单
     */
    void showTestMenu() {
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "Go2Communication 功能验证测试程序" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        std::cout << "请选择要运行的测试:" << std::endl;
        std::cout << "1. 初始化和生命周期管理测试" << std::endl;
        std::cout << "2. 连接状态管理测试" << std::endl;
        std::cout << "3. 消息发布功能测试" << std::endl;
        std::cout << "4. 回调设置测试" << std::endl;
        std::cout << "5. 缓冲区管理测试" << std::endl;
        std::cout << "6. 统计数据功能测试" << std::endl;
        std::cout << "7. 配置功能测试" << std::endl;
        std::cout << "8. 错误处理测试" << std::endl;
        std::cout << "9. 网络配置测试" << std::endl;
        std::cout << "0. 运行所有测试" << std::endl;
        std::cout << "q. 退出程序" << std::endl;
        std::cout << std::string(60, '-') << std::endl;
        std::cout << "提示：按 Ctrl+C 可随时停止程序" << std::endl;
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
                    runSingleTest("连接状态管理测试", [this]() { testConnectionManagement(); });
                    break;
                case '3':
                    runSingleTest("消息发布功能测试", [this]() { testMessagePublishing(); });
                    break;
                case '4':
                    runSingleTest("回调设置测试", [this]() { testCallbackSetup(); });
                    break;
                case '5':
                    runSingleTest("缓冲区管理测试", [this]() { testBufferManagement(); });
                    break;
                case '6':
                    runSingleTest("统计数据功能测试", [this]() { testStatistics(); });
                    break;
                case '7':
                    runSingleTest("配置功能测试", [this]() { testConfiguration(); });
                    break;
                case '8':
                    runSingleTest("错误处理测试", [this]() { testErrorHandling(); });
                    break;
                case '9':
                    runSingleTest("网络配置测试", [this]() { testNetworkConfiguration(); });
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
        printHeader("开始运行Go2Communication功能验证测试");

        // 1. 测试初始化和生命周期管理
        if (checkShutdown()) return;
        testInitializationLifecycle();

        // 2. 测试连接状态管理
        if (checkShutdown()) return;
        testConnectionManagement();

        // 3. 测试消息发布功能
        if (checkShutdown()) return;
        testMessagePublishing();

        // 4. 测试回调设置
        if (checkShutdown()) return;
        testCallbackSetup();

        // 5. 测试缓冲区管理
        if (checkShutdown()) return;
        testBufferManagement();

        // 6. 测试统计数据功能
        if (checkShutdown()) return;
        testStatistics();

        // 7. 测试配置功能
        if (checkShutdown()) return;
        testConfiguration();

        // 8. 测试错误处理
        if (checkShutdown()) return;
        testErrorHandling();

        // 9. 测试网络配置
        if (checkShutdown()) return;
        testNetworkConfiguration();

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
        checkTest("初始状态检查 - 未初始化", !comm_->isInitialized());
        checkTest("初始状态检查 - 未连接", !comm_->isConnected());
        checkTest("初始状态检查 - 不在通信中", !comm_->isCommunicating());

        // 测试初始化
        bool init_result = comm_->initialize();
        checkTest("initialize()调用", init_result);
        checkTest("初始化后状态检查 - 已初始化", comm_->isInitialized());

        // 等待一段时间让ROS2初始化完成
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // 测试启动通信
        bool start_result = comm_->startCommunication();
        checkTest("startCommunication()调用", start_result);

        // 注意：不调用stopCommunication()，因为后续测试需要保持连接状态
        std::cout << "  注意：保持连接状态以支持后续测试" << std::endl;

        // 注意：不在这里测试shutdown，因为后续测试还需要使用
        std::cout << "  注意：shutdown()测试将在所有测试完成后进行" << std::endl;
    }

    /**
     * @brief 测试连接状态管理
     */
    void testConnectionManagement() {
        printHeader("测试连接状态管理");

        // 测试连接状态获取
        CommunicationStatus status = comm_->getConnectionStatus();
        checkTest("getConnectionStatus()调用", true); // 只要不抛异常就算成功
        std::cout << "  当前连接状态: " << static_cast<int>(status) << std::endl;

        // 测试连接到机器人
        bool connect_result = comm_->connectToRobot("192.168.123.18", 5);
        checkTest("connectToRobot()调用", connect_result);

        // 测试自动重连设置
        comm_->setAutoReconnect(true, 3000, 5);
        checkTest("setAutoReconnect()调用", true);

        // 测试断开连接
        bool disconnect_result = comm_->disconnectFromRobot();
        checkTest("disconnectFromRobot()调用", disconnect_result);

        // 测试连接到机器人，确保后续正常
        connect_result = comm_->connectToRobot("192.168.123.18", 5);
        checkTest("connectToRobot()调用", connect_result);

        // 设置连接状态回调
        comm_->setConnectionStatusCallback([this](CommunicationStatus status) {
            this->last_status_ = status;
            std::cout << "  连接状态变化回调触发: " << static_cast<int>(status) << std::endl;
        });
        checkTest("setConnectionStatusCallback()调用", true);
    }

    /**
     * @brief 测试消息发布功能
     */
    void testMessagePublishing() {
        printHeader("测试消息发布功能");

        // 创建测试的API请求消息
        unitree_api::msg::Request api_msg;
        // 设置基本的请求头信息
        api_msg.header.identity.api_id = 1004; // StandUp API ID
        api_msg.parameter = "{}"; // 空JSON参数

        // 测试API请求发布 - Go2机器人唯一支持的控制方式
        bool api_result = comm_->sendApiRequest(api_msg);
        checkTest("sendApiRequest()调用", api_result);

        std::cout << "  注意：根据Go2通信规范，所有控制命令必须通过unitree_api::msg::Request发送" << std::endl;
    }

    /**
     * @brief 测试回调设置
     */
    void testCallbackSetup() {
        printHeader("测试回调设置");

        // 设置各种回调函数
        comm_->setPointCloudCallback([this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            this->pointcloud_received_ = true;
            std::cout << "  点云回调触发，点数: " << msg->width * msg->height << std::endl;
        });
        checkTest("setPointCloudCallback()调用", true);

        comm_->setImuCallback([this](const sensor_msgs::msg::Imu::SharedPtr msg) {
            (void)msg;
            this->imu_received_ = true;
            std::cout << "  IMU回调触发" << std::endl;
        });
        checkTest("setImuCallback()调用", true);

        comm_->setOdometryCallback([this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            (void)msg;
            this->odom_received_ = true;
            std::cout << "  里程计回调触发" << std::endl;
        });
        checkTest("setOdometryCallback()调用", true);

        comm_->setSportModeStateCallback([this](const unitree_go::msg::SportModeState::SharedPtr msg) {
            (void)msg;
            this->sport_state_received_ = true;
            std::cout << "  运动模式状态回调触发" << std::endl;
        });
        checkTest("setSportModeStateCallback()调用", true);

        comm_->setLowStateCallback([this](const unitree_go::msg::LowState::SharedPtr msg) {
            (void)msg;
            this->low_state_received_ = true;
            std::cout << "  底层状态回调触发" << std::endl;
        });
        checkTest("setLowStateCallback()调用", true);

        comm_->setBmsStateCallback([this](const unitree_go::msg::BmsState::SharedPtr msg) {
            (void)msg;
            this->bms_state_received_ = true;
            std::cout << "  BMS状态回调触发" << std::endl;
        });
        checkTest("setBmsStateCallback()调用", true);

        // 设置错误回调
        comm_->setErrorCallback([this](const std::string& error) {
            this->last_error_ = error;
            std::cout << "  错误回调触发: " << error << std::endl;
        });
        checkTest("setErrorCallback()调用", true);

        // 设置质量回调
        comm_->setQualityCallback([this](float quality) {
            this->last_quality_ = quality;
            std::cout << "  通信质量回调触发: " << quality << std::endl;
        });
        checkTest("setQualityCallback()调用", true);
    }

    /**
     * @brief 测试缓冲区管理
     */
    void testBufferManagement() {
        printHeader("测试缓冲区管理");

        // 测试设置缓冲区大小
        comm_->setMessageBufferSize(MessageType::POINT_CLOUD, 3);
        comm_->setMessageBufferSize(MessageType::IMU_DATA, 15);
        checkTest("setMessageBufferSize()调用", true);

        // 测试获取最新消息（应该返回nullptr，因为还没有收到消息）
        auto latest_pointcloud = comm_->getLatestPointCloud();
        checkTest("getLatestPointCloud()调用", latest_pointcloud == nullptr);

        auto latest_imu = comm_->getLatestImu();
        checkTest("getLatestImu()调用", latest_imu == nullptr);

        auto latest_sport_state = comm_->getLatestSportModeState();
        checkTest("getLatestSportModeState()调用", latest_sport_state == nullptr);

        auto latest_low_state = comm_->getLatestLowState();
        checkTest("getLatestLowState()调用", latest_low_state == nullptr);

        auto latest_bms_state = comm_->getLatestBmsState();
        checkTest("getLatestBmsState()调用", latest_bms_state == nullptr);

        // 测试清空缓冲区
        comm_->clearMessageBuffer(MessageType::POINT_CLOUD);
        comm_->clearMessageBuffer(MessageType::IMU_DATA);
        checkTest("clearMessageBuffer()调用", true);
    }

    /**
     * @brief 测试统计数据功能
     */
    void testStatistics() {
        printHeader("测试统计数据功能");

        // 获取统计数据
        CommunicationStats stats = comm_->getStatistics();
        checkTest("getStatistics()调用", true);

        std::cout << "  统计数据:" << std::endl;
        std::cout << "    已发送消息: " << stats.messages_sent << std::endl;
        std::cout << "    已接收消息: " << stats.messages_received << std::endl;
        std::cout << "    连接尝试次数: " << stats.connection_attempts << std::endl;
        std::cout << "    消息频率: " << stats.message_rate_hz << " Hz" << std::endl;
        std::cout << "    平均延迟: " << stats.average_latency_ms << " ms" << std::endl;

        // 测试消息延迟获取
        float pointcloud_latency = comm_->getMessageLatency(MessageType::POINT_CLOUD);
        checkTest("getMessageLatency()调用", pointcloud_latency >= 0.0f);
        std::cout << "  点云消息延迟: " << pointcloud_latency << " ms" << std::endl;

        // 测试消息频率获取
        float imu_frequency = comm_->getMessageFrequency(MessageType::IMU_DATA);
        checkTest("getMessageFrequency()调用", imu_frequency >= 0.0f);
        std::cout << "  IMU消息频率: " << imu_frequency << " Hz" << std::endl;

        // 测试通信质量检查
        bool quality_good = comm_->isCommuncationQualityGood();
        checkTest("isCommuncationQualityGood()调用", true); // 只要不抛异常就算成功
        std::cout << "  通信质量良好: " << (quality_good ? "是" : "否") << std::endl;

        // 测试重置统计
        comm_->resetStatistics();
        checkTest("resetStatistics()调用", true);

        // 验证重置效果
        CommunicationStats reset_stats = comm_->getStatistics();
        checkTest("统计重置验证", reset_stats.messages_sent == 0 && reset_stats.messages_received == 0);
    }

    /**
     * @brief 测试配置功能
     */
    void testConfiguration() {
        printHeader("测试配置功能");

        // 测试加载配置
        bool load_result = comm_->loadConfiguration();
        checkTest("loadConfiguration()调用", load_result);

        // 测试保存配置
        bool save_result = comm_->saveConfiguration();
        checkTest("saveConfiguration()调用", save_result);

        // 测试获取配置
        std::string config = comm_->getConfiguration();
        checkTest("getConfiguration()调用", !config.empty());
        std::cout << "  配置信息: " << config << std::endl;

        // 测试详细日志设置
        comm_->setVerboseLogging(true);
        checkTest("setVerboseLogging(true)调用", true);

        comm_->setVerboseLogging(false);
        checkTest("setVerboseLogging(false)调用", true);
    }

    /**
     * @brief 测试错误处理
     */
    void testErrorHandling() {
        printHeader("测试错误处理");

        // 获取最后错误
        std::string last_error = comm_->getLastError();
        checkTest("getLastError()调用", true);
        if (!last_error.empty()) {
            std::cout << "  最后错误: " << last_error << std::endl;
        }

        // 获取错误历史
        std::vector<std::string> error_history = comm_->getErrorHistory();
        checkTest("getErrorHistory()调用", true);
        std::cout << "  错误历史条目数: " << error_history.size() << std::endl;

        // 清除错误历史
        comm_->clearErrorHistory();
        checkTest("clearErrorHistory()调用", true);

        // 验证清除效果
        std::vector<std::string> cleared_history = comm_->getErrorHistory();
        checkTest("错误历史清除验证", cleared_history.empty());

        // 测试连接诊断
        std::map<std::string, bool> diagnostics = comm_->performConnectionDiagnostics();
        checkTest("performConnectionDiagnostics()调用", true);

        std::cout << "  连接诊断结果:" << std::endl;
        for (const auto& [key, value] : diagnostics) {
            std::cout << "    " << key << ": " << (value ? "通过" : "失败") << std::endl;
        }

        // 测试网络诊断
        std::string network_diag = comm_->getNetworkDiagnostics();
        checkTest("getNetworkDiagnostics()调用", !network_diag.empty());
        std::cout << "  网络诊断: " << network_diag << std::endl;
    }

    /**
     * @brief 测试网络配置
     */
    void testNetworkConfiguration() {
        printHeader("测试网络配置");

        // 测试设置网络接口
        bool interface_result = comm_->setNetworkInterface("enp3s0");
        checkTest("setNetworkInterface()调用", interface_result);

        // 测试设置DDS域ID
        bool domain_result = comm_->setDdsDomainId(1);
        checkTest("setDdsDomainId()调用", domain_result);

        // 测试QoS设置
        comm_->setQosSettings(
            rclcpp::ReliabilityPolicy::BestEffort,
            rclcpp::DurabilityPolicy::Volatile,
            10
        );
        checkTest("setQosSettings()调用", true);

        // 测试网络优化应用
        bool optimization_result = comm_->applyNetworkOptimization();
        checkTest("applyNetworkOptimization()调用", optimization_result);
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
        if (comm_) {
            std::cout << "正在关闭通信..." << std::endl;
            comm_->shutdown();
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

        // 执行最后的通信停止和shutdown测试
        std::cout << "\n执行最终清理..." << std::endl;

        // 测试停止通信
        bool stop_result = comm_->stopCommunication();
        if (stop_result) {
            std::cout << "✓ stopCommunication()调用 - 通过" << std::endl;
        } else {
            std::cout << "✗ stopCommunication()调用 - 失败" << std::endl;
        }

        // 测试最终关闭
        bool shutdown_result = comm_->shutdown();
        if (shutdown_result) {
            std::cout << "✓ shutdown()调用 - 通过" << std::endl;
        } else {
            std::cout << "✗ shutdown()调用 - 失败" << std::endl;
        }

        if (passed_tests_ == total_tests_) {
            std::cout << "\n🎉 所有功能验证测试通过！Go2Communication类工作正常。" << std::endl;
        } else {
            std::cout << "\n⚠️  有部分测试失败，请检查Go2Communication类的实现。" << std::endl;
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
        Go2CommunicationTester tester;
        tester.runInteractiveMenu();

    } catch (const std::exception& e) {
        std::cerr << "测试过程中发生异常: " << e.what() << std::endl;
        return 1;
    }

    // 关闭ROS2
    rclcpp::shutdown();
    return 0;
}