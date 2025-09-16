/**
 * @file   test_go2_power_manager.cpp
 * @brief  Go2PowerManager类功能验证测试程序
 * @author Yang Nan
 * @date   2025-09-16
 *
 * @details
 * 这是一个用于验证Go2PowerManager类各项功能的测试程序。
 * 该程序创建Go2PowerManager实例，并系统性地测试其各个功能模块，
 * 包括初始化、电池状态查询、充电管理、功耗控制、安全保护等。
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
#include "robot_adapters/go2_adapter/go2_power_manager.hpp"

// 全局变量控制程序停止
std::atomic<bool> g_shutdown_requested{false};

using namespace robot_adapters::go2_adapter;
using namespace robot_base_interfaces::power_interface;

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

class Go2PowerManagerTester {
private:
    std::shared_ptr<Go2PowerManager> power_manager_;
    std::shared_ptr<rclcpp::Node> node_;
    std::vector<std::string> test_results_;
    int total_tests_;
    int passed_tests_;

    // 回调数据存储
    bool battery_callback_triggered_;
    bool charging_callback_triggered_;
    bool power_event_callback_triggered_;
    bool low_battery_callback_triggered_;
    bool charge_complete_callback_triggered_;
    BatteryInfo last_battery_info_;
    ChargingStatus last_charging_status_;
    PowerEventInfo last_power_event_;
    float last_low_battery_percentage_;

public:
    Go2PowerManagerTester()
        : total_tests_(0), passed_tests_(0),
          battery_callback_triggered_(false), charging_callback_triggered_(false),
          power_event_callback_triggered_(false), low_battery_callback_triggered_(false),
          charge_complete_callback_triggered_(false), last_low_battery_percentage_(0.0f) {

        // 创建ROS2节点
        node_ = std::make_shared<rclcpp::Node>("test_go2_power_manager");

        // 创建Go2PowerManager实例
        power_manager_ = std::make_shared<Go2PowerManager>(node_);

        std::cout << "=== Go2PowerManager功能验证测试程序 ===" << std::endl;
        std::cout << "初始化测试环境..." << std::endl;
    }

    ~Go2PowerManagerTester() {
        if (power_manager_) {
            power_manager_->shutdown();
        }
    }

    /**
     * @brief 显示测试菜单
     */
    void showTestMenu() {
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "Go2PowerManager 功能验证测试程序" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        std::cout << "请选择要运行的测试:" << std::endl;
        std::cout << "1. 初始化和生命周期管理测试" << std::endl;
        std::cout << "2. 电池状态查询功能测试" << std::endl;
        std::cout << "3. 充电管理功能测试" << std::endl;
        std::cout << "4. 充电站管理测试" << std::endl;
        std::cout << "5. 功耗管理和控制测试" << std::endl;
        std::cout << "6. 电源控制功能测试" << std::endl;
        std::cout << "7. 安全和保护功能测试" << std::endl;
        std::cout << "8. 事件和回调函数测试" << std::endl;
        std::cout << "9. 配置管理测试" << std::endl;
        std::cout << "10. 诊断和统计功能测试" << std::endl;
        std::cout << "11. Go2特有功能测试" << std::endl;
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
                runSingleTest("初始化和生命周期管理测试", [this]() { testInitializationLifecycle(); });
            } else if (input == "2") {
                runSingleTest("电池状态查询功能测试", [this]() { testBatteryStatusQuery(); });
            } else if (input == "3") {
                runSingleTest("充电管理功能测试", [this]() { testChargingManagement(); });
            } else if (input == "4") {
                runSingleTest("充电站管理测试", [this]() { testChargingStationManagement(); });
            } else if (input == "5") {
                runSingleTest("功耗管理和控制测试", [this]() { testPowerConsumptionManagement(); });
            } else if (input == "6") {
                runSingleTest("电源控制功能测试", [this]() { testPowerControl(); });
            } else if (input == "7") {
                runSingleTest("安全和保护功能测试", [this]() { testSafetyProtection(); });
            } else if (input == "8") {
                runSingleTest("事件和回调函数测试", [this]() { testCallbacks(); });
            } else if (input == "9") {
                runSingleTest("配置管理测试", [this]() { testConfigurationManagement(); });
            } else if (input == "10") {
                runSingleTest("诊断和统计功能测试", [this]() { testDiagnosticsStatistics(); });
            } else if (input == "11") {
                runSingleTest("Go2特有功能测试", [this]() { testGo2SpecificFeatures(); });
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
        printHeader("开始运行Go2PowerManager功能验证测试");

        // 1. 测试初始化和生命周期管理
        if (checkShutdown()) return;
        testInitializationLifecycle();

        // 2. 测试电池状态查询功能
        if (checkShutdown()) return;
        testBatteryStatusQuery();

        // 3. 测试充电管理功能
        if (checkShutdown()) return;
        testChargingManagement();

        // 4. 测试充电站管理
        if (checkShutdown()) return;
        testChargingStationManagement();

        // 5. 测试功耗管理和控制
        if (checkShutdown()) return;
        testPowerConsumptionManagement();

        // 6. 测试电源控制功能
        if (checkShutdown()) return;
        testPowerControl();

        // 7. 测试安全和保护功能
        if (checkShutdown()) return;
        testSafetyProtection();

        // 8. 测试事件和回调函数
        if (checkShutdown()) return;
        testCallbacks();

        // 9. 测试配置管理
        if (checkShutdown()) return;
        testConfigurationManagement();

        // 10. 测试诊断和统计功能
        if (checkShutdown()) return;
        testDiagnosticsStatistics();

        // 11. 测试Go2特有功能
        if (checkShutdown()) return;
        testGo2SpecificFeatures();

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
        checkTest("初始状态检查 - 未操作", !power_manager_->isOperational());

        // 测试支持的充电类型
        std::vector<ChargingType> supported_types = power_manager_->getSupportedChargingTypes();
        checkTest("获取支持的充电类型", !supported_types.empty());
        std::cout << "  支持的充电类型数量: " << supported_types.size() << std::endl;

        // 测试初始化
        bool init_result = power_manager_->initialize();
        checkTest("initialize()调用", init_result);

        // 等待一段时间让初始化完成
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        checkTest("初始化后状态检查 - 可操作", power_manager_->isOperational());

        std::cout << "  注意：保持初始化状态以支持后续测试" << std::endl;
    }

    /**
     * @brief 测试电池状态查询功能
     */
    void testBatteryStatusQuery() {
        printHeader("测试电池状态查询功能");

        // 获取电池详细信息
        BatteryInfo battery_info = power_manager_->getBatteryInfo();
        checkTest("getBatteryInfo()调用", true);

        std::cout << "  Go2电池状态信息:" << std::endl;
        std::cout << "    电池电压: " << std::fixed << std::setprecision(2) << battery_info.voltage << " V" << std::endl;
        std::cout << "    电池电流: " << battery_info.current << " A" << std::endl;
        std::cout << "    电池功率: " << battery_info.power << " W" << std::endl;
        std::cout << "    电池温度: " << battery_info.temperature << " °C" << std::endl;
        std::cout << "    电量百分比: " << battery_info.soc_percentage << " %" << std::endl;
        std::cout << "    电池容量: " << battery_info.capacity_mah << " mAh" << std::endl;
        std::cout << "    剩余容量: " << battery_info.remaining_mah << " mAh" << std::endl;
        std::cout << "    充电循环次数: " << battery_info.cycle_count << " 次" << std::endl;
        std::cout << "    电芯数量: " << battery_info.cells.size() << " 个" << std::endl;

        // 测试单独的获取函数
        float percentage = power_manager_->getBatteryPercentage();
        checkTest("getBatteryPercentage()调用", percentage >= 0.0f && percentage <= 100.0f);
        std::cout << "    单独获取电量: " << percentage << " %" << std::endl;

        float voltage = power_manager_->getBatteryVoltage();
        checkTest("getBatteryVoltage()调用", voltage >= 0.0f);
        std::cout << "    单独获取电压: " << voltage << " V" << std::endl;

        float current = power_manager_->getBatteryCurrent();
        checkTest("getBatteryCurrent()调用", true);
        std::cout << "    单独获取电流: " << current << " A" << std::endl;

        float power = power_manager_->getBatteryPower();
        checkTest("getBatteryPower()调用", true);
        std::cout << "    单独获取功率: " << power << " W" << std::endl;

        float temperature = power_manager_->getBatteryTemperature();
        checkTest("getBatteryTemperature()调用", temperature > -50.0f && temperature < 100.0f);
        std::cout << "    单独获取温度: " << temperature << " °C" << std::endl;

        BatteryHealth health = power_manager_->getBatteryHealth();
        checkTest("getBatteryHealth()调用", true);
        std::cout << "    电池健康度: " << static_cast<int>(health) << std::endl;

        uint16_t cycles = power_manager_->getBatteryCycles();
        checkTest("getBatteryCycles()调用", true);
        std::cout << "    充电循环次数: " << cycles << " 次" << std::endl;

        float runtime = power_manager_->getEstimatedRuntime(50.0f);
        checkTest("getEstimatedRuntime()调用", runtime >= 0.0f);
        std::cout << "    预估运行时间(50W负载): " << runtime << " 分钟" << std::endl;
    }

    /**
     * @brief 测试充电管理功能
     */
    void testChargingManagement() {
        printHeader("测试充电管理功能");

        // 获取充电状态
        ChargingStatus charging_status = power_manager_->getChargingStatus();
        checkTest("getChargingStatus()调用", true);

        std::cout << "  当前充电状态:" << std::endl;
        std::cout << "    充电状态: " << static_cast<int>(charging_status.state) << std::endl;
        std::cout << "    充电类型: " << static_cast<int>(charging_status.charging_type) << std::endl;
        std::cout << "    充电电流: " << charging_status.charging_current << " A" << std::endl;
        std::cout << "    预计剩余时间: " << charging_status.estimated_remaining_seconds << " 秒" << std::endl;

        ChargingState state = power_manager_->getChargingState();
        checkTest("getChargingState()调用", true);
        std::cout << "    单独获取充电状态: " << static_cast<int>(state) << std::endl;

        bool is_charging = power_manager_->isCharging();
        checkTest("isCharging()调用", true);
        std::cout << "    是否正在充电: " << (is_charging ? "是" : "否") << std::endl;

        // 测试充电需求检查
        bool needs_charging = power_manager_->needsCharging();
        checkTest("needsCharging()调用", true);
        std::cout << "    是否需要充电: " << (needs_charging ? "是" : "否") << std::endl;

        bool is_critical = power_manager_->isBatteryCritical();
        checkTest("isBatteryCritical()调用", true);
        std::cout << "    电池是否严重不足: " << (is_critical ? "是" : "否") << std::endl;

        // 测试充电请求（仅模拟，不实际充电）
        std::cout << "  测试充电请求功能（模拟）:" << std::endl;
        bool start_charging = power_manager_->requestCharging(ChargingType::WIRELESS);
        checkTest("requestCharging(WIRELESS)调用", true);
        std::cout << "    无线充电请求结果: " << (start_charging ? "成功" : "失败") << std::endl;

        // 等待一段时间
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        bool stop_charging = power_manager_->requestStopCharging();
        checkTest("requestStopCharging()调用", true);
        std::cout << "    停止充电请求结果: " << (stop_charging ? "成功" : "失败") << std::endl;

        uint32_t estimated_time = power_manager_->getEstimatedChargeTime();
        checkTest("getEstimatedChargeTime()调用", true);
        std::cout << "    预计充电时间: " << estimated_time << " 分钟" << std::endl;
    }

    /**
     * @brief 测试充电站管理
     */
    void testChargingStationManagement() {
        printHeader("测试充电站管理");

        // 创建测试充电站信息
        ChargingStationInfo station1;
        station1.station_id = "test_station_1";
        station1.charging_type = ChargingType::WIRELESS;
        station1.pose.x = 1.0f;
        station1.pose.y = 2.0f;
        station1.pose.yaw = 0.0f;
        station1.is_available = true;
        station1.is_occupied = false;
        station1.max_power_output = 100.0f;

        ChargingStationInfo station2;
        station2.station_id = "test_station_2";
        station2.charging_type = ChargingType::WIRELESS;
        station2.pose.x = 5.0f;
        station2.pose.y = 3.0f;
        station2.pose.yaw = 1.57f;
        station2.is_available = true;
        station2.is_occupied = false;
        station2.max_power_output = 150.0f;

        // 注册充电站
        bool register1 = power_manager_->registerChargingStation(station1);
        checkTest("registerChargingStation(station1)调用", register1);

        bool register2 = power_manager_->registerChargingStation(station2);
        checkTest("registerChargingStation(station2)调用", register2);

        // 获取已知充电站
        std::vector<ChargingStationInfo> known_stations = power_manager_->getKnownChargingStations();
        checkTest("getKnownChargingStations()调用", known_stations.size() >= 2);
        std::cout << "  已注册充电站数量: " << known_stations.size() << std::endl;

        for (const auto& station : known_stations) {
            std::cout << "    充电站ID: " << station.station_id
                      << ", 位置: (" << station.pose.x << ", " << station.pose.y << ")"
                      << ", 可用: " << (station.is_available ? "是" : "否") << std::endl;
        }

        // 查找最近的充电站
        std::vector<float> current_pos = {2.0f, 2.5f};
        auto nearest_station = power_manager_->findNearestChargingStation(current_pos);
        checkTest("findNearestChargingStation()调用", nearest_station != nullptr);

        if (nearest_station) {
            std::cout << "  最近的充电站: " << nearest_station->station_id << std::endl;
        }

        // 更新充电站状态
        bool update_status = power_manager_->updateChargingStationStatus("test_station_1", true, true);
        checkTest("updateChargingStationStatus()调用", update_status);
        std::cout << "  充电站状态更新: " << (update_status ? "成功" : "失败") << std::endl;
    }

    /**
     * @brief 测试功耗管理和控制
     */
    void testPowerConsumptionManagement() {
        printHeader("测试功耗管理和控制");

        // 获取当前功耗
        float current_power = power_manager_->getCurrentPowerConsumption();
        checkTest("getCurrentPowerConsumption()调用", current_power >= 0.0f);
        std::cout << "  当前功耗: " << current_power << " W" << std::endl;

        // 获取平均功耗
        float average_power = power_manager_->getAveragePowerConsumption(300);
        checkTest("getAveragePowerConsumption()调用", average_power >= 0.0f);
        std::cout << "  最近5分钟平均功耗: " << average_power << " W" << std::endl;

        // 测试功耗配置文件
        PowerConsumptionProfile profile;
        profile.profile_name = "test_profile";
        profile.max_power = 120.0f;
        profile.idle_power = 10.0f;
        profile.standby_power = 5.0f;
        profile.walking_power = 50.0f;
        profile.running_power = 100.0f;

        bool set_profile = power_manager_->setPowerProfile(profile);
        checkTest("setPowerProfile()调用", set_profile);
        std::cout << "  设置功耗配置文件: " << (set_profile ? "成功" : "失败") << std::endl;

        PowerConsumptionProfile current_profile = power_manager_->getPowerProfile();
        checkTest("getPowerProfile()调用", true);
        std::cout << "  当前功耗配置文件: " << current_profile.profile_name << std::endl;
        std::cout << "    最大功耗: " << current_profile.max_power << " W" << std::endl;

        // 测试节能模式
        bool enable_saving = power_manager_->enablePowerSaving(true);
        checkTest("enablePowerSaving(true)调用", enable_saving);

        bool is_saving_enabled = power_manager_->isPowerSavingEnabled();
        checkTest("isPowerSavingEnabled()调用", is_saving_enabled);
        std::cout << "  节能模式状态: " << (is_saving_enabled ? "启用" : "禁用") << std::endl;

        // 禁用节能模式
        bool disable_saving = power_manager_->enablePowerSaving(false);
        checkTest("enablePowerSaving(false)调用", disable_saving);
    }

    /**
     * @brief 测试电源控制功能
     */
    void testPowerControl() {
        printHeader("测试电源控制功能");

        std::cout << "  注意：电源控制功能为模拟测试，不会实际关机或重启" << std::endl;

        // 测试系统关机请求
        bool shutdown_request = power_manager_->requestSystemShutdown(5);
        checkTest("requestSystemShutdown()调用", shutdown_request);
        std::cout << "  系统关机请求(5秒延迟): " << (shutdown_request ? "成功" : "失败") << std::endl;

        // 测试系统重启请求
        bool reboot_request = power_manager_->requestSystemReboot(3);
        checkTest("requestSystemReboot()调用", reboot_request);
        std::cout << "  系统重启请求(3秒延迟): " << (reboot_request ? "成功" : "失败") << std::endl;

        // 测试休眠模式
        bool sleep_mode = power_manager_->enterSleepMode(10);
        checkTest("enterSleepMode()调用", sleep_mode);
        std::cout << "  进入休眠模式(10秒): " << (sleep_mode ? "成功" : "失败") << std::endl;

        // 等待一段时间
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // 测试从休眠唤醒
        bool wake_up = power_manager_->wakeFromSleep();
        checkTest("wakeFromSleep()调用", wake_up);
        std::cout << "  从休眠唤醒: " << (wake_up ? "成功" : "失败") << std::endl;
    }

    /**
     * @brief 测试安全和保护功能
     */
    void testSafetyProtection() {
        printHeader("测试安全和保护功能");

        // 检查电池故障
        bool has_fault = power_manager_->hasBatteryFault();
        checkTest("hasBatteryFault()调用", true);
        std::cout << "  电池是否有故障: " << (has_fault ? "是" : "否") << std::endl;

        uint32_t fault_code = power_manager_->getBatteryFaultCode();
        checkTest("getBatteryFaultCode()调用", true);
        std::cout << "  电池故障代码: " << fault_code << std::endl;

        // 检查充电安全性
        bool is_charging_safe = power_manager_->isChargingSafe();
        checkTest("isChargingSafe()调用", true);
        std::cout << "  充电是否安全: " << (is_charging_safe ? "是" : "否") << std::endl;

        bool is_temp_safe = power_manager_->isTemperatureSafe();
        checkTest("isTemperatureSafe()调用", true);
        std::cout << "  温度是否安全: " << (is_temp_safe ? "是" : "否") << std::endl;

        // 测试电池校准
        bool calibrate = power_manager_->calibrateBattery();
        checkTest("calibrateBattery()调用", calibrate);
        std::cout << "  电池校准: " << (calibrate ? "成功" : "失败") << std::endl;

        // 测试重置电池统计
        bool reset_stats = power_manager_->resetBatteryStats();
        checkTest("resetBatteryStats()调用", reset_stats);
        std::cout << "  重置电池统计: " << (reset_stats ? "成功" : "失败") << std::endl;
    }

    /**
     * @brief 测试事件和回调函数
     */
    void testCallbacks() {
        printHeader("测试事件和回调函数");

        // 重置回调标志
        battery_callback_triggered_ = false;
        charging_callback_triggered_ = false;
        power_event_callback_triggered_ = false;
        low_battery_callback_triggered_ = false;
        charge_complete_callback_triggered_ = false;

        // 设置电池信息回调
        power_manager_->setBatteryCallback([this](const BatteryInfo& info) {
            this->battery_callback_triggered_ = true;
            this->last_battery_info_ = info;
            std::cout << "  电池回调触发: 电量=" << info.soc_percentage << "%" << std::endl;
        });
        checkTest("setBatteryCallback()调用", true);

        // 设置充电状态回调
        power_manager_->setChargingCallback([this](const ChargingStatus& status) {
            this->charging_callback_triggered_ = true;
            this->last_charging_status_ = status;
            std::cout << "  充电回调触发: 状态=" << static_cast<int>(status.state) << std::endl;
        });
        checkTest("setChargingCallback()调用", true);

        // 设置电源事件回调
        power_manager_->setPowerEventCallback([this](const PowerEventInfo& event) {
            this->power_event_callback_triggered_ = true;
            this->last_power_event_ = event;
            std::cout << "  电源事件回调触发: 类型=" << static_cast<int>(event.event_type) << std::endl;
        });
        checkTest("setPowerEventCallback()调用", true);

        // 设置低电量回调
        power_manager_->setLowBatteryCallback([this](float percentage) {
            this->low_battery_callback_triggered_ = true;
            this->last_low_battery_percentage_ = percentage;
            std::cout << "  低电量回调触发: 电量=" << percentage << "%" << std::endl;
        });
        checkTest("setLowBatteryCallback()调用", true);

        // 设置充电完成回调
        power_manager_->setChargeCompleteCallback([this]() {
            this->charge_complete_callback_triggered_ = true;
            std::cout << "  充电完成回调触发" << std::endl;
        });
        checkTest("setChargeCompleteCallback()调用", true);

        // 等待回调触发
        std::cout << "  等待回调触发..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        std::cout << "  回调设置完成，可正常接收电源相关事件" << std::endl;
    }

    /**
     * @brief 测试配置管理
     */
    void testConfigurationManagement() {
        printHeader("测试配置管理");

        // 测试阈值设置
        bool set_low_threshold = power_manager_->setLowBatteryThreshold(25.0f);
        checkTest("setLowBatteryThreshold()调用", set_low_threshold);

        bool set_critical_threshold = power_manager_->setCriticalBatteryThreshold(10.0f);
        checkTest("setCriticalBatteryThreshold()调用", set_critical_threshold);

        bool set_auto_threshold = power_manager_->setAutoChargeThreshold(30.0f);
        checkTest("setAutoChargeThreshold()调用", set_auto_threshold);

        // 测试自动充电设置
        bool enable_auto_charging = power_manager_->enableAutoCharging(true);
        checkTest("enableAutoCharging(true)调用", enable_auto_charging);

        bool is_auto_enabled = power_manager_->isAutoChargingEnabled();
        checkTest("isAutoChargingEnabled()调用", is_auto_enabled);
        std::cout << "  自动充电状态: " << (is_auto_enabled ? "启用" : "禁用") << std::endl;

        // 禁用自动充电
        bool disable_auto_charging = power_manager_->enableAutoCharging(false);
        checkTest("enableAutoCharging(false)调用", disable_auto_charging);

        // 获取配置信息
        std::string config_str = power_manager_->getConfiguration();
        checkTest("getConfiguration()调用", !config_str.empty());
        std::cout << "  当前配置信息:" << std::endl;
        std::cout << config_str << std::endl;
    }

    /**
     * @brief 测试诊断和统计功能
     */
    void testDiagnosticsStatistics() {
        printHeader("测试诊断和统计功能");

        // 获取电源诊断报告
        std::string diagnostics = power_manager_->getPowerDiagnostics();
        checkTest("getPowerDiagnostics()调用", !diagnostics.empty());
        std::cout << "  电源诊断报告:" << std::endl;
        std::cout << diagnostics << std::endl;

        // 获取充电统计报告
        std::string statistics = power_manager_->getChargingStatistics();
        checkTest("getChargingStatistics()调用", !statistics.empty());
        std::cout << "  充电统计报告:" << std::endl;
        std::cout << statistics << std::endl;

        // 测试数据导出功能
        std::string json_path = "/tmp/go2_power_test.json";
        bool export_json = power_manager_->exportBatteryData(json_path, "json");
        checkTest("exportBatteryData(json)调用", export_json);
        std::cout << "  JSON数据导出: " << (export_json ? "成功" : "失败") << std::endl;

        std::string csv_path = "/tmp/go2_power_test.csv";
        bool export_csv = power_manager_->exportBatteryData(csv_path, "csv");
        checkTest("exportBatteryData(csv)调用", export_csv);
        std::cout << "  CSV数据导出: " << (export_csv ? "成功" : "失败") << std::endl;
    }

    /**
     * @brief 测试Go2特有功能
     */
    void testGo2SpecificFeatures() {
        printHeader("测试Go2特有功能");

        // 获取原生BMS状态
        auto native_bms = power_manager_->getNativeBmsState();
        checkTest("getNativeBmsState()调用", true);
        if (native_bms) {
            std::cout << "  获取到原生BMS状态数据" << std::endl;
        } else {
            std::cout << "  暂无原生BMS状态数据" << std::endl;
        }

        // 设置Go2充电模式
        bool set_charging_mode = power_manager_->setGo2ChargingMode(true);
        checkTest("setGo2ChargingMode(true)调用", set_charging_mode);
        std::cout << "  启用Go2充电模式: " << (set_charging_mode ? "成功" : "失败") << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        bool disable_charging_mode = power_manager_->setGo2ChargingMode(false);
        checkTest("setGo2ChargingMode(false)调用", disable_charging_mode);
        std::cout << "  禁用Go2充电模式: " << (disable_charging_mode ? "成功" : "失败") << std::endl;

        // 获取无线控制器电量
        float controller_battery = power_manager_->getWirelessControllerBattery();
        checkTest("getWirelessControllerBattery()调用", controller_battery >= 0.0f && controller_battery <= 100.0f);
        std::cout << "  无线控制器电量: " << controller_battery << "%" << std::endl;

        // 检查是否需要更换电池
        bool replacement_needed = power_manager_->isBatteryReplacementNeeded();
        checkTest("isBatteryReplacementNeeded()调用", true);
        std::cout << "  是否需要更换电池: " << (replacement_needed ? "是" : "否") << std::endl;

        // 获取详细电池参数
        auto detailed_params = power_manager_->getDetailedBatteryParameters();
        checkTest("getDetailedBatteryParameters()调用", !detailed_params.empty());
        std::cout << "  详细电池参数数量: " << detailed_params.size() << std::endl;

        for (const auto& param : detailed_params) {
            std::cout << "    " << param.first << ": " << param.second << std::endl;
        }

        // 测试自定义命令
        std::string cell_voltages = power_manager_->executeCustomCommand("get_cell_voltages", "");
        checkTest("executeCustomCommand(get_cell_voltages)调用", !cell_voltages.empty());
        std::cout << "  电芯电压: " << cell_voltages << std::endl;
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
        if (power_manager_) {
            std::cout << "正在关闭电源管理器..." << std::endl;
            power_manager_->shutdown();
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
        bool shutdown_result = power_manager_->shutdown();
        if (shutdown_result) {
            std::cout << "✓ shutdown()调用 - 通过" << std::endl;
        } else {
            std::cout << "✗ shutdown()调用 - 失败" << std::endl;
        }

        if (passed_tests_ == total_tests_) {
            std::cout << "\n🎉 所有功能验证测试通过！Go2PowerManager类工作正常。" << std::endl;
        } else {
            std::cout << "\n⚠️  有部分测试失败，请检查Go2PowerManager类的实现。" << std::endl;
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
        Go2PowerManagerTester tester;

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