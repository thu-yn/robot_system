# state_interface

## 1. 概述

`state_interface` 是机器人基础接口层中负责全面状态监控、诊断和告警管理的核心软件包。它定义了一套标准的C++抽象接口和统一的数据模型，用于聚合、分析和分发来自机器人各个子系统（运动、电源、传感器、计算单元等）的状态信息。

该接口的设计完全覆盖了宇树Go2机器人的`SportModeState`和`LowState`两种核心状态信息，并将其整合为一个统一、详细的机器人状态模型。其目标是为上层应用（如任务调度、系统监控UI、远程运维平台）提供一个单一、可靠的状态信息来源，同时保持对不同机器人平台的良好兼容性和扩展性。

## 2. 核心设计

### 2.1. 抽象接口 (`IStateMonitor`)

核心抽象基类 `IStateMonitor` (`include/robot_base_interfaces/state_interface/i_state_monitor.hpp`) 定义了所有状态监控、诊断和告警处理的API。这是一个功能非常全面的接口，主要包括：

- **生命周期管理**: `initialize()`, `shutdown()`, `startMonitoring()`, `stopMonitoring()`。
- **综合状态查询**:
  - `getRobotState()`: 获取一个高级、概括性的机器人状态（如`ACTIVE`, `CHARGING`, `ERROR`）。
  - `getHealthStatus()` / `getHealthScore()`: 获取量化的系统健康等级和评分。
  - `isOperational()`: 快速判断机器人是否处于可执行任务的状态。
- **详细状态查询**:
  - `getDetailedState()`: 获取一个包含所有子系统信息的、非常详细的`DetailedRobotState`快照。
  - `getMotorInfo()`, `getFootInfo()`: 获取特定硬件单元（如电机、足端）的详细信息。
- **系统诊断**:
  - `getSystemDiagnostics()`: 获取所有已注册模块的诊断信息。
  - `performSystemCheck()`: 主动触发一次全面的系统自检。
- **告警管理**:
  - `getActiveAlerts()`: 获取当前所有未解决的告警。
  - `clearResolvedAlerts()`, `acknowledgeAlert()`: 管理告警生命周期。
- **性能统计**: `getPerformanceStats()` 获取长期运行的性能指标，如总里程、平均功耗等。
- **异步事件回调**: 提供丰富的回调机制，用于实时响应状态变化、健康度下降、新告警等事件。
  - `setStateChangeCallback()`, `setHealthChangeCallback()`, `setAlertCallback()`, `setDetailedStateCallback()`。
- **配置与数据记录**: 支持动态配置监控频率、健康阈值，并能将状态数据记录和导出到文件。

### 2.2. 数据类型 (`state_types.hpp`)

所有与状态、诊断、告警相关的数据结构、枚举和常量都定义在 `include/robot_base_interfaces/state_interface/state_types.hpp` 中。关键类型包括：

- **枚举**:
  - `RobotState`: 机器人高级状态枚举。
  - `HealthLevel`: 健康度等级（`EXCELLENT` 到 `CRITICAL`）。
  - `AlertType`: 告警类型（`INFO`, `WARNING`, `ERROR`）。
  - `SystemModule`: 系统模块枚举，用于区分告警和诊断信息的来源。
- **核心数据结构**:
  - `DetailedRobotState`: 这是一个非常核心的聚合结构体，它整合了来自运动、电源、传感器、通信和系统资源等所有子模块的状态信息，形成一个完整的机器人状态快照。
  - `MotorInfo`, `FootInfo`: 分别描述单个电机和足端的详细状态。
  - `AlertInfo`: 描述一个告警事件的完整信息。
  - `DiagnosticInfo`: 描述一个系统模块的诊断状态和关键指标。
  - `PerformanceStats`: 描述机器人长期运行的性能统计数据。

## 3. 使用方法

1.  **依赖**: 在需要监控机器人状态的软件包的 `package.xml` 和 `CMakeLists.txt` 中添加对 `state_interface` 的依赖。

2.  **获取接口实例**: 上层应用通过 `AdapterFactory` 获取一个指向 `IStateMonitor` 实现的智能指针。工厂会自动实例化与当前机器人匹配的适配器（如 `Go2StateMonitorAdapter`）。

3.  **调用接口**:
    ```cpp
    #include <robot_base_interfaces/state_interface/i_state_monitor.hpp>
    #include <memory>

    // 通过工厂获取状态监控器实例
    std::shared_ptr<robot_base_interfaces::state_interface::IStateMonitor> state_monitor;

    // 1. 初始化并开始监控
    if (state_monitor->initialize()) {
        state_monitor->startMonitoring();
    }

    // 2. 注册新告警回调，用于关键事件处理
    state_monitor->setAlertCallback([](const auto& alert) {
        if (alert.type >= AlertType::ERROR) {
            // 触发紧急停止或上报严重错误
        }
    });

    // 3. 注册详细状态回调，用于UI更新
    state_monitor->setDetailedStateCallback([](const auto& detailed_state) {
        // 更新UI显示CPU温度、内存使用率、电机状态等
        // 注意：回调在独立线程执行，需保证线程安全
    });

    // 4. 在任务开始前检查机器人是否可操作
    if (!state_monitor->isOperational()) {
        // 机器人处于错误或维护状态，拒绝执行任务
    }
    ```

## 4. 如何扩展

要为一个新的机器人平台添加状态监控支持，需要执行以下步骤：

1.  **创建新的适配器类**: 创建一个新类，继承自 `IStateMonitor`。
2.  **实现虚函数**: 在新类中，根据新平台的SDK或通信协议，具体实现 `IStateMonitor` 的所有纯虚函数。核心工作是：
    - 订阅机器人底层的各种状态消息。
    - 在内部回调中，将接收到的平台特定数据填充到统一的 `DetailedRobotState` 结构体中。
    - 根据预设规则，从 `DetailedRobotState` 中计算出综合的 `RobotState` 和 `HealthLevel`。
    - 实现告警生成逻辑，例如当电机温度超过阈值时，创建一个 `AlertInfo` 对象并触发告警回调。
3.  **注册到工厂**: 将新的适配器类注册到 `AdapterFactory` 中，使其能够被自动发现和实例化。

## 5. 编译与测试

- **编译**:
  ```bash
  colcon build --packages-select state_interface
  ```
- **测试**:
  ```bash
  colcon test --packages-select state_interface
  ```
  测试用例位于 `test` 目录下，主要验证数据结构的默认值和编译时属性。
