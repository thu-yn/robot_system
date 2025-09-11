# motion_interface

## 1. 概述

`motion_interface` 是机器人基础接口层中负责运动控制的核心软件包。它定义了一套标准的C++抽象接口和数据类型，用于控制机器人的移动、姿态、步态和特殊动作。该接口的设计完全兼容宇树Go2机器人的全部运动能力，并为扩展支持其他不同类型的机器人（如其他四足机器人、轮式机器人等）预留了清晰的结构。

上层应用（如导航、任务调度）通过调用本接口与机器人硬件进行解耦，从而实现“一次开发，多平台适用”的目标。

## 2. 核心设计

### 2.1. 抽象接口 (`IMotionController`)

核心抽象基类 `IMotionController` (`include/robot_base_interfaces/motion_interface/i_motion_controller.hpp`) 定义了所有运动控制的API。主要功能包括：

- **初始化与配置**: `initialize()`, `shutdown()`, `getCapabilities()`
- **基础运动控制**:
  - `setVelocity(const Velocity& velocity)`: 控制机器人的线速度和角速度，兼容`geometry_msgs/Twist`。
  - `setPosture(const Posture& posture)`: 控制机身的姿态（翻滚、俯仰、偏航）和高度。
  - `setBodyHeight(float height)`: 单独设置机身高度。
- **模式切换与状态控制**:
  - `switchMode(MotionMode mode)`: 切换运动模式（如站立、行走、趴下等）。
  - `setGaitType(GaitType gait)`: 设定步态（如小跑、奔跑、爬楼梯）。
  - `balanceStand()`, `standUp()`, `standDown()`, `sit()`: 执行原子化的动作。
- **安全机制**: `emergencyStop()` 用于紧急停止。
- **状态查询**:
  - `getMotionState()`: 获取详细的当前运动状态。
  - `isOperational()`: 检查控制器是否就绪。
- **异步事件回调**:
  - `setStateCallback()`: 注册运动状态变化的实时回调。
  - `setErrorCallback()`: 注册错误事件的回调。

### 2.2. 扩展接口 (`IQuadrupedTricks`)

为了保持基础运动接口的通用性，将四足机器人特有的“特技”动作（如跳舞、作揖、翻滚）分离到独立的扩展接口 `IQuadrupedTricks` (`include/robot_base_interfaces/motion_interface/i_quadruped_tricks.hpp`) 中。需要使用这些高级功能的应用可以动态查询并使用此接口。

### 2.3. 数据类型 (`motion_types.hpp`)

所有与运动相关的数据结构、枚举和常量都定义在 `include/robot_base_interfaces/motion_interface/motion_types.hpp` 中。关键类型包括：

- **枚举**:
  - `RobotType`: 机器人平台类型（`GO2`, `SPOT`, `ANYMAL`, `GENERIC`）。
  - `MotionMode`: 运动模式（`IDLE`, `LOCOMOTION`, `BALANCE_STAND` 等）。
  - `GaitType`: 步态类型（`TROT`, `RUN`, `CLIMB_STAIR` 等）。
  - `MotionResult`: 命令执行结果（`SUCCESS`, `INVALID_PARAMETER` 等）。
- **数据结构**:
  - `Velocity`: 包含线速度和角速度的6D向量。
  - `Posture`: 描述机身姿态和高度。
  - `MotionCapabilities`: 描述机器人运动能力的结构体，包括最大速度、姿态限制、支持的模式和步态等。
  - `MotionState`: 包含机器人当前模式、姿态、速度、足端力等信息的完整状态快照。

## 3. 使用方法

1.  **依赖**: 在需要控制机器人的软件包的 `package.xml` 和 `CMakeLists.txt` 中添加对 `motion_interface` 的依赖。

2.  **获取接口实例**: 上层应用通过 `AdapterFactory`（适配器工厂）获取一个指向 `IMotionController` 实现的智能指针。工厂会根据当前连接的机器人自动选择并实例化正确的适配器（如 `Go2MotionAdapter`）。

3.  **调用接口**:
    ```cpp
    #include <robot_base_interfaces/motion_interface/i_motion_controller.hpp>
    #include <memory>

    // 通过工厂获取控制器实例
    std::shared_ptr<robot_base_interfaces::motion_interface::IMotionController> motion_controller;

    // 1. 初始化
    if (motion_controller->initialize() != MotionResult::SUCCESS) {
        // 错误处理
    }

    // 2. 获取并检查能力
    auto caps = motion_controller->getCapabilities();
    if (caps.can_lateral_move) {
        // ...
    }

    // 3. 设置速度
    robot_base_interfaces::motion_interface::Velocity cmd_vel;
    cmd_vel.linear_x = 0.5; // m/s
    cmd_vel.angular_z = 0.2; // rad/s
    motion_controller->setVelocity(cmd_vel);

    // 4. 注册状态回调
    motion_controller->setStateCallback([](const auto& state) {
        // 处理实时状态更新
        // 注意：回调在独立线程执行，需保证线程安全
    });
    ```

## 4. 如何扩展

要支持一个新的机器人平台，需要执行以下步骤：

1.  **创建新的适配器类**: 创建一个新类，继承自 `IMotionController`。
2.  **实现虚函数**: 在新类中，根据新平台的SDK或通信协议，具体实现 `IMotionController` 的所有纯虚函数。例如，`setVelocity` 函数需要将标准的速度指令转换为该平台特定的指令格式并发送。
3.  **实现能力描述**: `getCapabilities` 函数需要返回一个准确描述新平台运动能力的 `MotionCapabilities` 对象。
4.  **注册到工厂**: 将新的适配器类注册到 `AdapterFactory` 中，使其能够被自动发现和实例化。

## 5. 编译与测试

- **编译**:
  ```bash
  colcon build --packages-select motion_interface
  ```
- **测试**:
  ```bash
  colcon test --packages-select motion_interface
  ```
  测试用例位于 `test` 目录下，主要验证数据结构的默认值和编译时属性。

