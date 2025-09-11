# robot_base_interfaces 基础抽象接口层

## 概述

`robot_base_interfaces` 是宇树Go2四足机器人自主导航系统中的**核心抽象接口层**，定义了统一的机器人硬件抽象接口。该模块采用面向接口编程的设计理念，通过抽象基类定义标准接口，为不同类型的机器人提供统一的访问方式，是整个导航系统的基础架构组件。

## 在项目中的重要作用

### 1. 架构核心地位

`robot_base_interfaces` 在整个四层架构体系中位于**机器人接口层的基础**，承担以下关键职责：

```
应用层 (task_manager, map_manager, fleet_manager)
        ↓
算法层 (slam_manager, path_planner, motion_controller)  
        ↓
ROS2通信层 (标准ROS2消息接口)
        ↓
机器人接口层 (robot_adapters)
        ↓
robot_base_interfaces (统一抽象接口) ← 当前模块
        ↓
硬件层 (Go2机器人本体)
```

### 2. 统一抽象作用

- **屏蔽硬件差异**：为上层算法提供统一的机器人控制和感知接口
- **支持多机器人平台**：通过抽象接口设计，预留对其他四足机器人的扩展能力
- **简化上层开发**：导航、SLAM、感知等模块只需面向抽象接口编程
- **提高系统可维护性**：接口变更不影响上层业务逻辑

## 目录结构详解

```
robot_base_interfaces/
├── motion_interface/           # 运动控制抽象接口
│   ├── include/robot_base_interfaces/motion_interface/
│   │   ├── i_motion_controller.hpp      # 运动控制器接口
│   │   ├── motion_types.hpp             # 运动相关数据类型
│   │   └── i_quadruped_tricks.hpp       # 四足机器人特技接口扩展
│   ├── src/
│   │   └── motion_types.cpp             # 数据类型实现
│   ├── test/                            # 单元测试
│   ├── CMakeLists.txt                   # 构建配置
│   └── package.xml                      # ROS2包描述
│
├── sensor_interface/           # 传感器数据抽象接口
│   ├── include/robot_base_interfaces/sensor_interface/
│   │   ├── i_sensor_interface.hpp       # 传感器接口
│   │   └── sensor_types.hpp             # 传感器数据类型
│   ├── src/
│   │   └── sensor_types.cpp             # 数据类型实现
│   ├── test/                            # 单元测试
│   ├── CMakeLists.txt
│   └── package.xml
│
├── state_interface/            # 状态监控抽象接口
│   ├── include/robot_base_interfaces/state_interface/
│   │   ├── i_state_monitor.hpp          # 状态监控器接口
│   │   └── state_types.hpp        # 机器人状态类型
│   ├── src/
│   │   └── state_types.cpp        # 状态类型实现
│   ├── test/                            # 单元测试
│   ├── CMakeLists.txt
│   └── package.xml
│
├── power_interface/            # 电源管理抽象接口
│   ├── include/robot_base_interfaces/power_interface/
│   │   ├── i_power_manager.hpp          # 电源管理器接口
│   │   └── power_types.hpp              # 电源相关数据类型
│   ├── src/
│   │   └── power_types.cpp              # 数据类型实现
│   ├── test/                            # 单元测试
│   ├── CMakeLists.txt
│   └── package.xml
│
├── common/                     # 通用组件
│   └── result.hpp                       # 强类型结果与错误码定义
│
└── README.md                   # 本文档
```

## 核心接口详解

### 1. 运动控制接口 (motion_interface)

#### IMotionController 接口

`IMotionController` 是运动控制的核心抽象接口，定义了机器人运动控制的标准方法：

**核心功能分组**：

- **基础运动控制**：`setVelocity()`, `setPosture()`, `setBodyHeight()`, `emergencyStop()`
- **运动模式管理**：`switchMode()`, `setGaitType()`, `balanceStand()`, `standUp()`, `standDown()`
- **Go2特有功能**：`performDance()`, `frontFlip()`, `frontJump()`, `hello()`, `stretch()`
- **状态查询**：`getMotionState()`, `isOperational()`, `getErrorCode()`, `isMotionCompleted()`
- **回调机制**：`setStateCallback()`, `setErrorCallback()`

**设计特点**：

- 完全覆盖Go2的运动API（对应unitree_api的Request消息）
- 通过虚函数提供默认实现，支持能力有限的机器人
- 包含参数验证的保护方法，确保命令安全性
- 支持异步回调，实时获取运动状态变化

**数据类型**：

- `Velocity`: 三维速度控制（线性x/y，角速度z）
- `Posture`: 姿态控制（Roll/Pitch/Yaw + 机身高度）
- `MotionMode`: 运动模式枚举（待机、平衡站立、移动等）
- `GaitType`: 步态类型（小跑、慢走、跳跃等）
- `MotionCapabilities`: 机器人运动能力描述

### 2. 传感器接口 (sensor_interface)

#### ISensorInterface 接口

传感器接口提供统一的传感器数据访问方式，重点适配Go2的传感器配置：

**主要传感器支持**：

- **Livox Mid360 3D激光雷达**：`getLatestPointCloud()`, `setPointCloudCallback()`
- **内置IMU传感器**：`getLatestIMU()`, `setIMUCallback()`
- **预留扩展**：2D激光雷达、摄像头等

**功能特性**：

- 传感器生命周期管理：`initialize()`, `startSensor()`, `stopSensor()`
- 实时数据获取：支持轮询和回调两种模式
- 传感器配置：`setSensorParameter()`, `setSensorFrequency()`
- 坐标系管理：`getSensorTransform()`, `setSensorTransform()`
- 校准支持：`calibrateSensor()`, `getCalibrationData()`
- 健康监控：`getSensorHealth()`, `getSensorError()`, `getSensorStatistics()`

**数据类型**：

- `PointCloudData`: 三维点云数据
- `IMUData`: 惯性测量单元数据
- `SensorInfo`: 传感器信息描述
- `CalibrationData`: 传感器校准数据

### 3. 状态监控接口 (state_interface)

#### IStateMonitor 接口

状态监控接口提供完整的机器人状态监控能力，对应Go2的SportModeState和LowState：

**监控范围**：

- **基础状态**：`getRobotState()`, `getHealthStatus()`, `isOperational()`
- **详细硬件状态**：`getDetailedState()`, `getMotorInfo()`, `getFootInfo()`
- **系统诊断**：`performSystemCheck()`, `getSystemDiagnostics()`
- **告警管理**：`getActiveAlerts()`, `clearResolvedAlerts()`, `acknowledgeAlert()`
- **性能统计**：`getPerformanceStats()`, `getUptimeSeconds()`

**Go2特有功能**：

- **20个电机状态监控**：温度、转矩、位置、速度
- **4个足端状态监控**：接触状态、力反馈
- **BMS电池管理**：集成到整体健康评估

**事件驱动**：

- `setStateChangeCallback()`: 状态变化通知
- `setHealthChangeCallback()`: 健康状态变化
- `setAlertCallback()`: 新告警事件
- `setErrorCallback()`: 错误事件

### 4. 电源管理接口 (power_interface)

#### IPowerManager 接口

电源管理接口负责电池监控、充电管理和功耗优化：

**电池监控**：

- **详细电池信息**：`getBatteryInfo()`, `getBatteryPercentage()`, `getBatteryVoltage()`
- **健康评估**：`getBatteryHealth()`, `getBatteryCycles()`, `hasBatteryFault()`
- **续航预估**：`getEstimatedRuntime()`, `getEstimatedChargeTime()`

**充电管理**：

- **充电控制**：`requestCharging()`, `requestStopCharging()`, `isCharging()`
- **充电站管理**：`registerChargingStation()`, `findNearestChargingStation()`
- **安全检查**：`isChargingSafe()`, `isTemperatureSafe()`

**功耗优化**：

- **功耗监控**：`getCurrentPowerConsumption()`, `getAveragePowerConsumption()`
- **节能模式**：`enablePowerSaving()`, `setPowerProfile()`
- **系统电源控制**：`requestSystemShutdown()`, `enterSleepMode()`

## 设计原则与特色

### 1. 面向Go2优化，预留扩展

- **完全适配Go2**：接口设计完全覆盖Go2的硬件能力和API
- **扩展性设计**：通过虚函数默认实现和能力查询机制支持其他机器人
- **优雅降级**：不支持的功能返回合理的默认值或错误码

### 2. 强类型安全

- **枚举类型**：使用强类型枚举避免魔法数字
- **结果类型**：提供 `Result<T>`模板类实现强类型返回值
- **参数验证**：内置参数范围检查和能力验证

### 3. 异步友好

- **回调机制**：支持状态变化、数据更新、错误事件的异步通知
- **线程安全**：接口设计考虑多线程环境，明确线程语义
- **非阻塞设计**：避免长时间阻塞调用

### 4. 运维友好

- **诊断支持**：完整的健康监控、错误诊断、性能统计
- **配置管理**：支持运行时参数调整和能力配置
- **数据导出**：支持状态数据导出，便于分析调试

## 与Go2硬件的对应关系

### 运动控制映射

| 接口方法           | Go2 API             | 功能说明                |
| ------------------ | ------------------- | ----------------------- |
| `setVelocity()`  | unitree_api Request | 速度控制，对应ROS Twist |
| `balanceStand()` | API Code: 1002      | 平衡站立                |
| `standUp()`      | API Code: 1004      | 站起动作                |
| `standDown()`    | API Code: 1005      | 趴下动作                |
| `sit()`          | API Code: 1009      | 坐下动作                |
| `performDance()` | API Code: 1007/1008 | 舞蹈1/舞蹈2             |
| `frontFlip()`    | API Code: 1030      | 前翻动作                |
| `frontJump()`    | API Code: 1031      | 前跳动作                |

### 传感器数据映射

| 接口方法                  | Go2话题            | 数据类型                 |
| ------------------------- | ------------------ | ------------------------ |
| `getLatestPointCloud()` | `/utlidar/cloud` | sensor_msgs::PointCloud2 |
| `getLatestIMU()`        | `/imu/data`      | sensor_msgs::Imu         |
| `getLatestOdometry()`   | 来自SportModeState | nav_msgs::Odometry       |

### 状态监控映射

| 接口方法             | Go2消息              | 说明         |
| -------------------- | -------------------- | ------------ |
| `getRobotState()`  | SportModeState       | 运动状态     |
| `getMotorInfo()`   | LowState.motor_state | 20个电机状态 |
| `getFootInfo()`    | LowState.foot_force  | 足端力反馈   |
| `getBatteryInfo()` | LowState.bms         | 电池管理系统 |

## 使用示例

### 1. 运动控制示例

```cpp
#include "robot_base_interfaces/motion_interface/i_motion_controller.hpp"

// 获取运动控制器实例（由具体适配器提供）
auto motion_controller = robot_adapter->getMotionController();

// 检查运动能力
auto capabilities = motion_controller->getCapabilities();
std::cout << "最大线速度: " << capabilities.max_linear_velocity << " m/s" << std::endl;

// 设置机器人运动
Velocity cmd_vel;
cmd_vel.linear_x = 0.5;  // 前进0.5m/s
cmd_vel.angular_z = 0.2; // 左转0.2rad/s

if (motion_controller->setVelocity(cmd_vel) == MotionResult::SUCCESS) {
    std::cout << "运动命令发送成功" << std::endl;
}

// 执行特殊动作
if (motion_controller->hello() == MotionResult::SUCCESS) {
    std::cout << "Go2正在打招呼" << std::endl;
}
```

### 2. 传感器数据获取

```cpp
#include "robot_base_interfaces/sensor_interface/i_sensor_interface.hpp"

auto sensor_interface = robot_adapter->getSensorInterface();

// 设置点云数据回调
sensor_interface->setPointCloudCallback([](const auto& cloud) {
    std::cout << "收到点云数据: " << cloud->points.size() << " 个点" << std::endl;
    // 处理点云数据...
});

// 获取最新IMU数据
auto imu_data = sensor_interface->getLatestIMU();
if (imu_data) {
    std::cout << "IMU线加速度: " 
              << imu_data->linear_acceleration.x << ", "
              << imu_data->linear_acceleration.y << ", " 
              << imu_data->linear_acceleration.z << std::endl;
}
```

### 3. 状态监控示例

```cpp
#include "robot_base_interfaces/state_interface/i_state_monitor.hpp"

auto state_monitor = robot_adapter->getStateMonitor();

// 检查机器人健康状态
auto health = state_monitor->getHealthStatus();
auto score = state_monitor->getHealthScore();
std::cout << "健康评分: " << score << "/1.0" << std::endl;

// 获取电机状态
for (int i = 0; i < 20; ++i) {
    auto motor_info = state_monitor->getMotorInfo(i);
    if (motor_info.temperature > 60.0f) {
        std::cout << "警告: 电机" << i << "温度过高: " 
                  << motor_info.temperature << "°C" << std::endl;
    }
}

// 设置状态变化回调
state_monitor->setStateChangeCallback([](RobotState old_state, RobotState new_state) {
    std::cout << "机器人状态变化: " << (int)old_state << " -> " << (int)new_state << std::endl;
});
```

### 4. 电源管理示例

```cpp
#include "robot_base_interfaces/power_interface/i_power_manager.hpp"

auto power_manager = robot_adapter->getPowerManager();

// 检查电池状态
auto battery_info = power_manager->getBatteryInfo();
std::cout << "电池电量: " << battery_info.percentage << "%" << std::endl;
std::cout << "预计续航: " << power_manager->getEstimatedRuntime() << " 分钟" << std::endl;

// 检查是否需要充电
if (power_manager->needsCharging()) {
    std::cout << "电量不足，需要充电" << std::endl;
  
    // 查找最近的充电站
    std::vector<float> current_pos = {1.0f, 2.0f, 0.0f};
    auto nearest_station = power_manager->findNearestChargingStation(current_pos);
  
    if (nearest_station) {
        std::cout << "找到充电站: " << nearest_station->name << std::endl;
        power_manager->requestCharging();
    }
}
```

## 构建和集成

### 依赖关系

```xml
<!-- package.xml -->
<depend>rclcpp</depend>
<depend>std_msgs</depend>
<depend>geometry_msgs</depend>
<depend>sensor_msgs</depend>
<depend>nav_msgs</depend>
```

### CMake集成

```cmake
# 在上层包的CMakeLists.txt中
find_package(robot_base_interfaces REQUIRED)

target_link_libraries(your_node
  robot_base_interfaces::motion_interface
  robot_base_interfaces::sensor_interface
  robot_base_interfaces::state_interface  
  robot_base_interfaces::power_interface
)
```

### 编译命令

```bash
# 单独编译基础接口层
colcon build --packages-select robot_base_interfaces

# 编译所有接口相关包
colcon build --packages-up-to robot_base_interfaces
```

## 扩展开发指南

### 1. 添加新机器人支持

当需要支持新的四足机器人时，只需要：

1. **实现接口类**：继承四个抽象接口并实现具体方法
2. **定义能力矩阵**：在配置文件中描述机器人具体能力
3. **适配消息格式**：实现与新机器人通信协议的转换
4. **注册到工厂**：在适配器工厂中添加新机器人类型

### 2. 扩展传感器支持

```cpp
// 继承ISensorInterface并添加新传感器方法
class CustomSensorInterface : public ISensorInterface {
public:
    // 添加深度相机支持
    virtual std::shared_ptr<DepthImageData> getLatestDepthImage() const = 0;
    virtual void setDepthImageCallback(std::function<void(const auto&)> callback) = 0;
};
```

### 3. 自定义运动模式

```cpp
// 利用executeCustomCommand扩展接口
auto result = motion_controller->executeCustomCommand(
    "custom_gait", 
    R"({"speed": 1.2, "step_height": 0.08, "frequency": 2.5})"
);
```

## 测试框架

每个接口模块都包含完整的单元测试：

```bash
# 运行所有测试
colcon test --packages-select robot_base_interfaces

# 查看测试结果
colcon test-result --verbose
```

测试覆盖：

- **接口完整性测试**：确保所有虚函数都有合理实现
- **参数验证测试**：验证输入参数的边界检查
- **错误处理测试**：测试异常情况下的行为
- **回调机制测试**：验证异步回调的正确性

## 性能考虑

### 1. 内存管理

- 使用智能指针管理对象生命周期
- 避免不必要的数据拷贝，优先使用共享指针
- 大型传感器数据使用移动语义

### 2. 实时性保证

- 关键路径避免动态内存分配
- 回调函数保持轻量，避免阻塞
- 提供同步和异步两种API访问模式

### 3. 线程安全

- 接口方法标明线程安全性
- 回调函数可能在不同线程执行
- 建议上层使用者实现适当的同步机制

## 故障诊断

### 常见问题及解决方案

1. **编译错误：找不到头文件**

   ```bash
   # 确保正确设置了include路径
   source install/setup.bash
   ```
2. **运行时错误：接口不支持某功能**

   ```cpp
   // 先检查能力再调用方法
   if (motion_controller->getCapabilities().supports_dancing) {
       motion_controller->performDance();
   }
   ```
3. **回调函数不触发**

   - 检查是否正确调用了 `startSensor()`或 `startMonitoring()`
   - 确认回调函数不包含耗时操作
4. **内存泄漏**

   - 使用智能指针管理资源
   - 在析构函数中正确清理回调注册

## 版本历史

- **v1.0.0** (2025): 初始版本，完整支持Go2机器人
- **v1.1.0** (计划): 添加更多传感器类型支持
- **v2.0.0** (计划): 支持多机器人协作接口

## 贡献指南

1. 遵循C++14标准和Google代码风格
2. 每个公共接口都需要详细的Doxygen注释
3. 新功能必须包含对应的单元测试
4. 更新相关文档和示例代码

## 许可证

本项目采用Apache 2.0许可证，详见LICENSE文件。

---

**robot_base_interfaces** 作为整个导航系统的接口基础，为Go2四足机器人的自主导航提供了坚实的抽象层支撑。通过统一的接口设计，不仅简化了上层算法的开发，也为未来支持更多机器人平台奠定了良好的架构基础。
