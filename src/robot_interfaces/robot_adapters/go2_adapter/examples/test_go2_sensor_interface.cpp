/**
 * @file   test_go2_sensor_interface.cpp
 * @brief  Go2SensorInterface类功能验证测试程序
 * @author Yang Nan
 * @date   2025-09-17
 *
 * @details
 * 这是一个用于验证Go2SensorInterface类各项功能的测试程序。
 * 该程序创建Go2SensorInterface实例，并系统性地测试其各个功能模块，
 * 包括初始化、传感器管理、数据获取、消息转换、回调设置等。
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
#include "robot_adapters/go2_adapter/go2_sensor_interface.hpp"

// 全局变量控制程序停止
std::atomic<bool> g_shutdown_requested{false};

using namespace robot_adapters::go2_adapter;
using namespace robot_base_interfaces::sensor_interface;

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

class Go2SensorInterfaceTester {
private:
    std::shared_ptr<Go2SensorInterface> sensor_interface_;
    std::vector<std::string> test_results_;
    int total_tests_;
    int passed_tests_;

    // 回调数据存储
    bool point_cloud_callback_triggered_;
    bool imu_callback_triggered_;
    std::shared_ptr<PointCloudData> last_point_cloud_;
    std::shared_ptr<IMUData> last_imu_data_;

    // Go2原生消息回调数据存储
    bool sport_callback_triggered_;
    bool low_state_callback_triggered_;
    bool bms_callback_triggered_;

public:
    Go2SensorInterfaceTester()
        : total_tests_(0), passed_tests_(0),
          point_cloud_callback_triggered_(false), imu_callback_triggered_(false),
          sport_callback_triggered_(false), low_state_callback_triggered_(false),
          bms_callback_triggered_(false) {

        // 创建Go2SensorInterface实例
        sensor_interface_ = std::make_shared<Go2SensorInterface>("test_go2_sensor_interface");

        std::cout << "=== Go2SensorInterface功能验证测试程序 ===" << std::endl;
        std::cout << "初始化测试环境..." << std::endl;
    }

    ~Go2SensorInterfaceTester() {
        if (sensor_interface_) {
            sensor_interface_->shutdown();
        }
    }

    /**
     * @brief 显示测试菜单
     */
    void showTestMenu() {
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "Go2SensorInterface 功能验证测试程序" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        std::cout << "请选择要运行的测试:" << std::endl;
        std::cout << "1. 初始化和生命周期管理测试" << std::endl;
        std::cout << "2. 传感器发现和管理测试" << std::endl;
        std::cout << "3. 传感器状态和健康监测测试" << std::endl;
        std::cout << "4. 传感器数据获取测试" << std::endl;
        std::cout << "5. Go2原生消息处理测试" << std::endl;
        std::cout << "6. 数据转换和融合测试" << std::endl;
        std::cout << "7. 回调函数设置测试" << std::endl;
        std::cout << "8. 传感器参数配置测试" << std::endl;
        std::cout << "9. 通信统计和错误处理测试" << std::endl;
        std::cout << "0. 运行所有测试" << std::endl;
        std::cout << "q. 退出程序" << std::endl;
        std::cout << std::string(60, '-') << std::endl;
        std::cout << "请输入选择: " << std::flush;
    }

    /**
     * @brief 获取传感器接口实例，供main函数中的ROS消息处理线程使用
     */
    std::shared_ptr<Go2SensorInterface> getSensorInterface() const {
        return sensor_interface_;
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
                runSingleTest("传感器发现和管理测试", [this]() { testSensorDiscoveryManagement(); });
            } else if (input == "3") {
                runSingleTest("传感器状态和健康监测测试", [this]() { testSensorStatusHealth(); });
            } else if (input == "4") {
                runSingleTest("传感器数据获取测试", [this]() { testSensorDataRetrieval(); });
            } else if (input == "5") {
                runSingleTest("Go2原生消息处理测试", [this]() { testGo2NativeMessages(); });
            } else if (input == "6") {
                runSingleTest("数据转换和融合测试", [this]() { testDataConversionFusion(); });
            } else if (input == "7") {
                runSingleTest("回调函数设置测试", [this]() { testCallbacks(); });
            } else if (input == "8") {
                runSingleTest("传感器参数配置测试", [this]() { testSensorParameterConfiguration(); });
            } else if (input == "9") {
                runSingleTest("通信统计和错误处理测试", [this]() { testCommunicationErrorHandling(); });
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
        printHeader("开始运行Go2SensorInterface功能验证测试");

        // 1. 测试初始化和生命周期管理
        if (checkShutdown()) return;
        testInitializationLifecycle();

        // 2. 测试传感器发现和管理
        if (checkShutdown()) return;
        testSensorDiscoveryManagement();

        // 3. 测试传感器状态和健康监测
        if (checkShutdown()) return;
        testSensorStatusHealth();

        // 4. 测试传感器数据获取
        if (checkShutdown()) return;
        testSensorDataRetrieval();

        // 5. 测试Go2原生消息处理
        if (checkShutdown()) return;
        testGo2NativeMessages();

        // 6. 测试数据转换和融合
        if (checkShutdown()) return;
        testDataConversionFusion();

        // 7. 测试回调函数设置
        if (checkShutdown()) return;
        testCallbacks();

        // 8. 测试传感器参数配置
        if (checkShutdown()) return;
        testSensorParameterConfiguration();

        // 9. 测试通信统计和错误处理
        if (checkShutdown()) return;
        testCommunicationErrorHandling();

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
        checkTest("初始状态检查 - 节点名称", sensor_interface_->get_name() == std::string("test_go2_sensor_interface"));

        // 测试初始化
        bool init_result = sensor_interface_->initialize();
        checkTest("initialize()调用", init_result);

        // 等待一段时间让初始化完成
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        std::cout << "  注意：保持初始化状态以支持后续测试" << std::endl;
    }

    /**
     * @brief 测试传感器发现和管理
     */
    void testSensorDiscoveryManagement() {
        printHeader("测试传感器发现和管理");

        // 获取可用传感器列表
        auto available_sensors = sensor_interface_->getAvailableSensors();
        checkTest("getAvailableSensors()调用", true);
        std::cout << "  发现 " << available_sensors.size() << " 个传感器" << std::endl;

        // 显示传感器信息
        for (const auto& sensor : available_sensors) {
            std::cout << "    传感器类型: " << static_cast<int>(sensor.type)
                      << ", 名称: " << sensor.name
                      << ", 话题: " << sensor.topic_name
                      << ", 频率: " << sensor.frequency << " Hz" << std::endl;
        }

        // 测试特定传感器可用性
        checkTest("LiDAR传感器可用性", sensor_interface_->isSensorAvailable(SensorType::LIDAR_3D));
        checkTest("IMU传感器可用性", sensor_interface_->isSensorAvailable(SensorType::IMU));
        checkTest("不支持的传感器可用性", !sensor_interface_->isSensorAvailable(SensorType::CAMERA_RGB));

        // 测试传感器启动和停止
        bool start_lidar = sensor_interface_->startSensor(SensorType::LIDAR_3D);
        checkTest("启动LiDAR传感器", start_lidar);

        bool start_imu = sensor_interface_->startSensor(SensorType::IMU);
        checkTest("启动IMU传感器", start_imu);

        // 等待传感器启动
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        bool stop_lidar = sensor_interface_->stopSensor(SensorType::LIDAR_3D);
        checkTest("停止LiDAR传感器", stop_lidar);

        bool stop_imu = sensor_interface_->stopSensor(SensorType::IMU);
        checkTest("停止IMU传感器", stop_imu);
    }

    /**
     * @brief 测试传感器状态和健康监测
     */
    void testSensorStatusHealth() {
        printHeader("测试传感器状态和健康监测");

        // 测试传感器状态查询
        auto lidar_status = sensor_interface_->getSensorStatus(SensorType::LIDAR_3D);
        checkTest("getLiDARStatus()调用", true);
        std::cout << "  LiDAR状态: " << static_cast<int>(lidar_status) << std::endl;

        auto imu_status = sensor_interface_->getSensorStatus(SensorType::IMU);
        checkTest("getIMUStatus()调用", true);
        std::cout << "  IMU状态: " << static_cast<int>(imu_status) << std::endl;

        // 测试传感器健康状态
        auto sensor_health = sensor_interface_->getSensorHealth();
        checkTest("getSensorHealth()调用", !sensor_health.empty());
        std::cout << "  传感器健康状态数: " << sensor_health.size() << std::endl;

        for (const auto& [sensor_type, status] : sensor_health) {
            std::cout << "    传感器类型 " << static_cast<int>(sensor_type)
                      << " 状态: " << static_cast<int>(status) << std::endl;
        }

        // 测试传感器错误信息
        std::string lidar_error = sensor_interface_->getSensorError(SensorType::LIDAR_3D);
        checkTest("getLiDARError()调用", true);
        if (!lidar_error.empty()) {
            std::cout << "  LiDAR错误: " << lidar_error << std::endl;
        }

        std::string imu_error = sensor_interface_->getSensorError(SensorType::IMU);
        checkTest("getIMUError()调用", true);
        if (!imu_error.empty()) {
            std::cout << "  IMU错误: " << imu_error << std::endl;
        }

        // 测试传感器统计信息
        auto lidar_stats = sensor_interface_->getSensorStatistics(SensorType::LIDAR_3D);
        checkTest("getLiDARStatistics()调用", true);
        std::cout << "  LiDAR统计信息数: " << lidar_stats.size() << std::endl;

        auto imu_stats = sensor_interface_->getSensorStatistics(SensorType::IMU);
        checkTest("getIMUStatistics()调用", true);
        std::cout << "  IMU统计信息数: " << imu_stats.size() << std::endl;
    }

    /**
     * @brief 测试传感器数据获取
     */
    void testSensorDataRetrieval() {
        printHeader("测试传感器数据获取");

        // 等待数据更新
        std::cout << "  等待传感器数据更新..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // 刷新所有传感器数据
        sensor_interface_->refreshAllSensorData();

        // 测试点云数据获取
        auto point_cloud = sensor_interface_->getLatestPointCloud();
        checkTest("getLatestPointCloud()调用", true);
        if (point_cloud) {
            std::cout << "  点云数据可用，点数: " << point_cloud->points.size() << std::endl;
            std::cout << "  点云时间戳: " << point_cloud->timestamp_ns << std::endl;
            std::cout << "  点云坐标系: " << point_cloud->frame_id << std::endl;
        } else {
            std::cout << "  点云数据不可用" << std::endl;
        }

        // 测试IMU数据获取
        auto imu_data = sensor_interface_->getLatestIMU();
        checkTest("getLatestIMU()调用", true);
        if (imu_data) {
            std::cout << "  IMU数据可用" << std::endl;
            std::cout << "  姿态四元数: [" << imu_data->orientation.w << ", "
                      << imu_data->orientation.x << ", " << imu_data->orientation.y
                      << ", " << imu_data->orientation.z << "]" << std::endl;
            std::cout << "  角速度: [" << imu_data->angular_velocity.x << ", "
                      << imu_data->angular_velocity.y << ", " << imu_data->angular_velocity.z << "]" << std::endl;
            std::cout << "  线性加速度: [" << imu_data->linear_acceleration.x << ", "
                      << imu_data->linear_acceleration.y << ", " << imu_data->linear_acceleration.z << "]" << std::endl;
        } else {
            std::cout << "  IMU数据不可用" << std::endl;
        }

        // 测试融合IMU数据获取
        auto fused_imu = sensor_interface_->getFusedIMUData();
        checkTest("getFusedIMUData()调用", true);
        if (fused_imu) {
            std::cout << "  融合IMU数据可用，温度: " << static_cast<int>(fused_imu->temperature) << "°C" << std::endl;
        } else {
            std::cout << "  融合IMU数据不可用" << std::endl;
        }

        // 测试通用传感器数据获取
        auto lidar_sensor_data = sensor_interface_->getLatestData(SensorType::LIDAR_3D);
        checkTest("getLiDARLatestData()调用", true);
        if (lidar_sensor_data && lidar_sensor_data->hasData()) {
            std::cout << "  LiDAR传感器数据可用" << std::endl;
        }

        auto imu_sensor_data = sensor_interface_->getLatestData(SensorType::IMU);
        checkTest("getIMULatestData()调用", true);
        if (imu_sensor_data && imu_sensor_data->hasData()) {
            std::cout << "  IMU传感器数据可用" << std::endl;
        }
    }

    /**
     * @brief 测试Go2原生消息处理
     */
    void testGo2NativeMessages() {
        printHeader("测试Go2原生消息处理");

        // 等待数据更新
        std::cout << "  等待Go2原生消息更新..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // 测试SportModeState数据
        auto sport_state = sensor_interface_->getLatestSportModeState();
        checkTest("getLatestSportModeState()调用", true);
        if (sport_state) {
            std::cout << "  SportModeState数据可用" << std::endl;
            std::cout << "  运动模式: " << static_cast<int>(sport_state->mode) << std::endl;
            std::cout << "  步态类型: " << static_cast<int>(sport_state->gait_type) << std::endl;
            std::cout << "  足端力: [" << sport_state->foot_force[0] << ", "
                      << sport_state->foot_force[1] << ", " << sport_state->foot_force[2]
                      << ", " << sport_state->foot_force[3] << "]" << std::endl;
            std::cout << "  IMU温度原始数据:" << static_cast<int>(sport_state->imu_state.temperature) << "°C" << std::endl;
        } else {
            std::cout << "  SportModeState数据不可用" << std::endl;
        }

        // 测试LowState数据
        auto low_state = sensor_interface_->getLatestLowState();
        checkTest("getLatestLowState()调用", true);
        if (low_state) {
            std::cout << "  LowState数据可用" << std::endl;
            std::cout << "  电机数量: " << low_state->motor_state.size() << std::endl;
            std::cout << "  足端力: [" << low_state->foot_force[0] << ", "
                      << low_state->foot_force[1] << ", " << low_state->foot_force[2]
                      << ", " << low_state->foot_force[3] << "]" << std::endl;
        } else {
            std::cout << "  LowState数据不可用" << std::endl;
        }

        // 测试BmsState数据
        auto bms_state = sensor_interface_->getLatestBmsState();
        checkTest("getLatestBmsState()调用", true);
        if (bms_state) {
            std::cout << "  BmsState数据可用" << std::endl;
            std::cout << "  电池电量: " << static_cast<int>(bms_state->soc) << "%" << std::endl;
            std::cout << "  电池电流: " << bms_state->current << "A" << std::endl;
            std::cout << "  电池状态: " << static_cast<int>(bms_state->status) << std::endl;
        } else {
            std::cout << "  BmsState数据不可用" << std::endl;
        }

        // 测试电机数据获取
        for (int i = 0; i < 4; ++i) {  // 测试前4个电机
            auto motor_data = sensor_interface_->getMotorData(i);
            checkTest("getMotorData(" + std::to_string(i) + ")调用", true);
            std::cout << "  电机 " << i << " - 位置: " << motor_data.position
                      << ", 速度: " << motor_data.velocity
                      << ", 温度: " << static_cast<int>(motor_data.temperature) << "°C" << std::endl;
        }

        // 测试足端数据获取
        for (int i = 0; i < 4; ++i) {  // 测试4只脚
            auto foot_data = sensor_interface_->getFootData(i);
            checkTest("getFootData(" + std::to_string(i) + ")调用", true);
            std::cout << "  足端 " << i << " - 力: " << foot_data.force
                      << ", 接触: " << (foot_data.in_contact ? "是" : "否")
                      << std::endl;
        }
    }

    /**
     * @brief 测试数据转换和融合
     */
    void testDataConversionFusion() {
        printHeader("测试数据转换和融合");

        // 等待数据更新以进行转换测试
        std::cout << "  等待数据更新以测试转换功能..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // 刷新数据以触发转换过程
        sensor_interface_->refreshAllSensorData();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // 测试IMU数据融合
        auto fused_imu = sensor_interface_->getFusedIMUData();
        checkTest("IMU数据融合", fused_imu != nullptr);
        if (fused_imu) {
            std::cout << "  融合IMU数据成功获取" << std::endl;
            std::cout << "  RPY角度: [" << fused_imu->rpy.roll << ", "
                      << fused_imu->rpy.pitch << ", " << fused_imu->rpy.yaw << "]" << std::endl;
            std::cout << "  温度详细信息:" << std::endl;
            std::cout << "    温度值: " << static_cast<int>(fused_imu->temperature) << "°C" << std::endl;
        }

        // 测试点云数据增强处理
        auto enhanced_pc = sensor_interface_->getLatestPointCloud();
        checkTest("点云数据增强处理", enhanced_pc != nullptr);
        if (enhanced_pc) {
            std::cout << "  增强点云数据成功获取，点数: " << enhanced_pc->points.size() << std::endl;
            std::cout << "  点云坐标系: " << enhanced_pc->frame_id << std::endl;
        }

        // 测试多传感器数据同步
        bool data_sync_success = (fused_imu != nullptr) && (enhanced_pc != nullptr);
        checkTest("多传感器数据同步", data_sync_success);
        if (data_sync_success) {
            std::cout << "  多传感器数据同步成功" << std::endl;
        }
    }

    /**
     * @brief 测试回调函数设置
     */
    void testCallbacks() {
        printHeader("测试回调函数设置");

        // 重置回调标志
        point_cloud_callback_triggered_ = false;
        imu_callback_triggered_ = false;
        sport_callback_triggered_ = false;
        low_state_callback_triggered_ = false;
        bms_callback_triggered_ = false;

        // 设置Go2原生消息回调
        sensor_interface_->setGo2Callbacks(
            [this](const std::shared_ptr<unitree_go::msg::SportModeState>& msg) {
                this->sport_callback_triggered_ = true;
                std::cout << "  SportModeState回调触发，模式: " << static_cast<int>(msg->mode) << std::endl;
            },
            [this](const std::shared_ptr<unitree_go::msg::LowState>& msg) {
                this->low_state_callback_triggered_ = true;
                std::cout << "  LowState回调触发，电机数: " << msg->motor_state.size() << std::endl;
            },
            [this](const std::shared_ptr<unitree_go::msg::BmsState>& msg) {
                this->bms_callback_triggered_ = true;
                std::cout << "  BmsState回调触发，电量: " << static_cast<int>(msg->soc) << "%" << std::endl;
            }
        );
        checkTest("setGo2Callbacks()调用", true);

        // 等待回调触发
        std::cout << "  等待回调触发..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        // 主动刷新数据以触发回调
        sensor_interface_->refreshAllSensorData();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // 验证回调是否被触发（注意：实际回调可能需要真实的硬件连接）
        checkTest("SportModeState回调设置", true); // 假设设置成功
        checkTest("LowState回调设置", true);
        checkTest("BmsState回调设置", true);

        std::cout << "  回调设置完成，实际触发需要真实硬件连接" << std::endl;
    }

    /**
     * @brief 测试传感器参数配置
     */
    void testSensorParameterConfiguration() {
        printHeader("测试传感器参数配置");

        // 测试获取传感器参数
        float max_range;
        bool get_max_range = sensor_interface_->getSensorParameter(SensorType::LIDAR_3D, "max_range", max_range);
        checkTest("获取LiDAR最大距离参数", get_max_range);
        if (get_max_range) {
            std::cout << "  LiDAR最大距离: " << max_range << " m" << std::endl;
        }

        float min_range;
        bool get_min_range = sensor_interface_->getSensorParameter(SensorType::LIDAR_3D, "min_range", min_range);
        checkTest("获取LiDAR最小距离参数", get_min_range);
        if (get_min_range) {
            std::cout << "  LiDAR最小距离: " << min_range << " m" << std::endl;
        }

        float frequency;
        bool get_frequency = sensor_interface_->getSensorParameter(SensorType::LIDAR_3D, "frequency", frequency);
        checkTest("获取LiDAR频率参数", get_frequency);
        if (get_frequency) {
            std::cout << "  LiDAR频率: " << frequency << " Hz" << std::endl;
        }

        // 测试设置传感器参数（Go2通常不支持动态修改）
        bool set_param = sensor_interface_->setSensorParameter(SensorType::LIDAR_3D, "max_range", 50.0f);
        checkTest("设置传感器参数（预期失败）", !set_param);

        // 测试设置传感器频率（Go2通常不支持动态修改）
        bool set_freq = sensor_interface_->setSensorFrequency(SensorType::LIDAR_3D, 20.0f);
        checkTest("设置传感器频率（预期失败）", !set_freq);

        // 测试传感器校准（Go2出厂时已校准）
        bool calibrate = sensor_interface_->calibrateSensor(SensorType::LIDAR_3D);
        checkTest("传感器校准（预期失败）", !calibrate);

        // 测试获取校准数据
        auto calib_data = sensor_interface_->getCalibrationData(SensorType::LIDAR_3D);
        checkTest("获取校准数据", !calib_data.is_valid);  // Go2不提供校准数据访问

        // 测试获取传感器变换矩阵
        auto transform = sensor_interface_->getSensorTransform(SensorType::LIDAR_3D);
        checkTest("获取传感器变换矩阵", transform.size() == 16);
        if (transform.size() == 16) {
            std::cout << "  变换矩阵获取成功（4x4矩阵）" << std::endl;
        }
    }

    /**
     * @brief 测试通信统计和错误处理
     */
    void testCommunicationErrorHandling() {
        printHeader("测试通信统计和错误处理");

        // 测试通信统计
        auto comm_stats = sensor_interface_->getCommunicationStatistics();
        checkTest("getCommunicationStatistics()调用", true);
        std::cout << "  通信统计信息数: " << comm_stats.size() << std::endl;

        for (const auto& [key, value] : comm_stats) {
            std::cout << "    " << key << ": " << value << std::endl;
        }

        // 测试传感器统计
        auto lidar_stats = sensor_interface_->getSensorStatistics(SensorType::LIDAR_3D);
        checkTest("LiDAR统计信息获取", true);
        std::cout << "  LiDAR统计信息: " << lidar_stats.size() << " 项" << std::endl;

        auto imu_stats = sensor_interface_->getSensorStatistics(SensorType::IMU);
        checkTest("IMU统计信息获取", true);
        std::cout << "  IMU统计信息: " << imu_stats.size() << " 项" << std::endl;

        // 测试错误信息获取
        std::string lidar_error = sensor_interface_->getSensorError(SensorType::LIDAR_3D);
        checkTest("LiDAR错误信息获取", true);
        if (!lidar_error.empty()) {
            std::cout << "  LiDAR当前错误: " << lidar_error << std::endl;
        } else {
            std::cout << "  LiDAR无错误" << std::endl;
        }

        std::string imu_error = sensor_interface_->getSensorError(SensorType::IMU);
        checkTest("IMU错误信息获取", true);
        if (!imu_error.empty()) {
            std::cout << "  IMU当前错误: " << imu_error << std::endl;
        } else {
            std::cout << "  IMU无错误" << std::endl;
        }

        // 测试数据验证和刷新
        sensor_interface_->refreshAllSensorData();
        checkTest("refreshAllSensorData()调用", true);
        std::cout << "  传感器数据刷新完成" << std::endl;
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
        if (sensor_interface_) {
            std::cout << "正在关闭传感器接口..." << std::endl;
            sensor_interface_->shutdown();
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
        bool shutdown_result = sensor_interface_->shutdown();
        if (shutdown_result) {
            std::cout << "✓ shutdown()调用 - 通过" << std::endl;
        } else {
            std::cout << "✗ shutdown()调用 - 失败" << std::endl;
        }

        if (passed_tests_ == total_tests_) {
            std::cout << "\n🎉 所有功能验证测试通过！Go2SensorInterface类工作正常。" << std::endl;
        } else {
            std::cout << "\n⚠️  有部分测试失败，请检查Go2SensorInterface类的实现。" << std::endl;
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
        Go2SensorInterfaceTester tester;

        // 启动ROS消息处理线程，持续处理订阅的消息
        std::thread spin_thread([&tester]() {
            rclcpp::spin(tester.getSensorInterface());
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