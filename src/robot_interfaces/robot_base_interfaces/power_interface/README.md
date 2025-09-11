# power_interface

## 1. 概述

`power_interface` 是机器人基础接口层中负责电源和充电管理的软件包。它定义了一套标准的C++抽象接口和数据类型，用于监控机器人的电池状态、控制充电过程以及管理电源模式。该接口的设计旨在兼容不同品牌和型号的机器人及其充电桩，特别是针对宇树Go2的无线充电功能进行了适配。

通过将电源管理的具体实现与上层应用（如自主充电任务调度、系统状态监控）分离，本软件包使得整个机器人系统在电源管理方面具有更好的模块化和可扩展性。

## 2. 核心设计

### 2.1. 抽象接口 (`IPowerManager`)

核心抽象基类 `IPowerManager` (`include/robot_base_interfaces/power_interface/i_power_manager.hpp`) 定义了所有电源管理和充电控制的API。主要功能包括：

- **初始化与配置**: `initialize()`, `shutdown()`。
- **状态查询**:
  - `getBatteryState()`: 获取当前电池的详细状态，包括电量百分比、电压、电流、温度等。
  - `getPowerMode()`: 获取机器人当前的电源模式（如正常工作、低功耗、充电中）。
  - `isCharging()`: 快速检查机器人是否正在充电。
- **充电控制**:
  - `startCharging()`: 命令机器人开始充电。这可能涉及与充电桩的通信、对位或启动无线充电等操作。
  - `stopCharging()`: 命令机器人停止充电。
- **电源模式管理**:
  - `setPowerMode(PowerMode mode)`: 切换机器人的电源模式，以在不同任务需求下平衡性能和续航。
- **异步事件回调**:
  - `setBatteryStateCallback()`: 注册电池状态变化的实时回调，用于上层应用监控电量。
  - `setChargingStateCallback()`: 注册充电状态（开始、结束、异常）变化的回调。

### 2.2. 数据类型 (`power_types.hpp`)

所有与电源和充电相关的数据结构、枚举和常量都定义在 `include/robot_base_interfaces/power_interface/power_types.hpp` 中。关键类型包括：

- **枚举**:
  - `PowerMode`: 电源模式（`NORMAL`, `LOW_POWER`, `SLEEP`, `CHARGING`）。
  - `ChargingStatus`: 充电状态（`NOT_CHARGING`, `CHARGING`, `CHARGED`, `ERROR`）。
  - `PowerSupplyStatus`: 电源供应状态，用于区分电池、适配器或USB供电。
  - `CommandResult`: 命令执行结果的通用枚举。
- **数据结构**:
  - `BatteryState`: 描述电池状态的完整结构体，包含电压、电流、温度、剩余容量、电量百分比等，兼容`sensor_msgs/BatteryState`消息。
  - `ChargingStationInfo`: （预留）用于描述充电桩信息，如ID、位置、类型等。

## 3. 使用方法

1.  **依赖**: 在需要监控电源或控制充电的软件包的 `package.xml` 和 `CMakeLists.txt` 中添加对 `power_interface` 的依赖。

2.  **获取接口实例**: 上层应用通过 `AdapterFactory` 获取一个指向 `IPowerManager` 实现的智能指针。工厂会根据当前连接的机器人自动选择并实例化正确的适配器（如 `Go2PowerAdapter`）。

3.  **调用接口**:
    ```cpp
    #include <robot_base_interfaces/power_interface/i_power_manager.hpp>
    #include <memory>

    // 通过工厂获取电源管理器实例
    std::shared_ptr<robot_base_interfaces::power_interface::IPowerManager> power_manager;

    // 1. 初始化
    if (power_manager->initialize() != CommandResult::SUCCESS) {
        // 错误处理
    }

    // 2. 获取电池状态
    auto battery_state = power_manager->getBatteryState();
    if (battery_state.percentage < 0.2) {
        // 电量低于20%，触发自主充电任务
    }

    // 3. 注册电量变化回调
    power_manager->setBatteryStateCallback([](const auto& state) {
        // 在UI上更新电量显示
        // 注意：回调在独立线程执行，需保证线程安全
    });

    // 4. 开始充电
    if (!power_manager->isCharging()) {
        power_manager->startCharging();
    }
    ```

## 4. 如何扩展

要为一个新的机器人平台或充电桩添加支持，需要执行以下步骤：

1.  **创建新的适配器类**: 创建一个新类，继承自 `IPowerManager`。
2.  **实现虚函数**: 在新类中，根据新平台的SDK或通信协议，具体实现 `IPowerManager` 的所有纯虚函数。例如，`getBatteryState` 函数需要从机器人的底层状态消息中解析出电池信息并填充到 `BatteryState` 结构体中；`startCharging` 则可能需要发送一个特定的CAN总线消息或调用一个HTTP服务。
3.  **注册到工厂**: 将新的适配器类注册到 `AdapterFactory` 中，使其能够被自动发现和实例化。

## 5. 编译与测试

- **编译**:
  ```bash
  colcon build --packages-select power_interface
  ```
- **测试**:
  ```bash
  colcon test --packages-select power_interface
  ```
  测试用例位于 `test` 目录下，主要验证数据结构的默认值和编译时属性。
