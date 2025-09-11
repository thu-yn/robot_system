# Go2 Adapter Package

## 1. 概述 (Overview)

`go2_adapter` 是专为 Unitree Go2 四足机器人设计的 ROS2 适配器包。

作为 `robot_interfaces` 抽象层的一个具体实现，本包的核心职责是**将上层应用发出的标准化指令（如速度、姿态）转换为 Go2 机器人可以理解的特定 ROS2 消息格式，并将 Go2 的传感器数据和状态信息转换为系统统一的标准化消息格式**。

它充当了通用机器人控制系统与 Go2 机器人硬件之间的“翻译官”和“驱动程序”，完全遵循了 `robot_interfaces` 定义的适配器设计模式，实现了上层应用与特定机器人平台的完全解耦。

## 2. 核心职责 (Core Responsibilities)

- **实现标准接口**: 继承并实现 `robot_interfaces` 中定义的所有核心抽象接口 (`IMotionController`, `ISensorInterface`, `IStateMonitor`, `IPowerManager`)。
- **数据双向转换**: 在标准 ROS2 消息 (如 `geometry_msgs::msg::Twist`) 和 Unitree Go2 的专有消息 (如 `unitree_api::msg::Request`) 之间进行双向转换。
- **管理通信**: 负责与 Go2 机器人进行 ROS2 通信的所有细节，包括话题的订阅、发布，以及服务质量（QoS）的配置。
- **生命周期管理**: 管理与机器人连接的整个生命周期，包括初始化、健康检查、心跳维持和安全关闭。

## 3. 文件详解 (File-by-File Breakdown)

本包遵循职责分离原则，将不同功能模块化到独立的文件中，结构清晰，易于维护。

---

### `go2_adapter.hpp` / `go2_adapter.cpp`

- **职责**: **适配器主控类 (Orchestrator)**
- **详细说明**:
    - 这是 `go2_adapter` 包的入口和核心，定义了 `Go2Adapter` 类。
    - `Go2Adapter` 类继承自 `IRobotAdapter` 接口，是工厂模式创建的主要对象。
    - 负责**初始化并持有**所有其他功能模块（如 `Go2MotionController`, `Go2SensorInterface` 等）的实例。
    - 通过 `getMotionController()`, `getSensorInterface()` 等方法，向上层提供对具体功能模块的访问。
    - 管理整个适配器的生命周期，确保所有模块按顺序正确地初始化和关闭。

---

### `go2_motion_controller.hpp` / `go2_motion_controller.cpp`

- **职责**: **运动控制实现**
- **详细说明**:
    - 实现了 `IMotionController` 抽象接口。
    - **接收**上层应用发来的标准化速度指令 (`geometry_msgs::msg::Twist`)。
    - **调用** `Go2MessageConverter` 将标准速度指令**转换**为 Go2 特定的 `unitree_api::msg::Request` 消息。
    - **发布**转换后的消息到 `/api/sport/request` 话题，从而直接控制机器人行走、转向等。
    - **订阅** `/sportmodestate` 话题，以获取机器人当前的运动状态（如速度、模式等），并将其提供给上层。

---

### `go2_sensor_interface.hpp` / `go2_sensor_interface.cpp`

- **职责**: **传感器数据接口实现**
- **详细说明**:
    - 实现了 `ISensorInterface` 抽象接口。
    - **订阅** Go2 发布的原始传感器数据，主要包括：
        - `/utlidar/cloud`: Livox Mid360 激光雷达的点云数据 (`sensor_msgs::msg::PointCloud2`)。
        - `/imu/data`: IMU 惯性测量单元数据 (`sensor_msgs::msg::Imu`)。
    - 对原始数据进行必要的**预处理**、**坐标变换**和**时间戳同步**。
    - 将处理后的数据封装成系统标准格式，供上层感知算法（如 SLAM、障碍物检测）使用。

---

### `go2_state_monitor.hpp` / `go2_state_monitor.cpp`

- **职责**: **机器人状态监控实现**
- **详细说明**:
    - 实现了 `IStateMonitor` 抽象接口。
    - **订阅** `/lowstate` 话题，这是 Go2 机器人发布的核心底层状态信息。
    - **解析** `lowstate` 消息，提取关键的机器人健康和状态信息，例如：
        - 各关节电机的角度、速度、力矩。
        - 电机温度。
        - 足端接触状态。
        - 各种错误或警告标志位。
    - 将这些底层状态聚合成上层应用可以理解的、标准化的机器人健康报告和状态更新。

---

### `go2_power_manager.hpp` / `go2_power_manager.cpp`

- **职责**: **电源管理实现**
- **详细说明**:
    - 实现了 `IPowerManager` 抽象接口。
    - 同样从 `/lowstate` 话题中**解析**关于电池的信息，包括：
        - 电池电压 (Voltage)
        - 电池电流 (Current)
        - 电量百分比 (Percentage)
    - 提供获取电池当前状态、估算剩余电量等标准接口。
    - 为上层的自主充电决策系统提供关键数据支持。

---

### `go2_communication.hpp` / `go2_communication.cpp`

- **职责**: **底层通信管理**
- **详细说明**:
    - 这是一个辅助模块，封装了所有与 ROS2 通信相关的实现细节。
    - 负责创建、配置和管理本节点所有的**发布者 (Publisher)** 和**订阅者 (Subscriber)**。
    - 统一处理 ROS2 的服务质量（QoS）配置，确保与 Go2 机器人的 DDS 通信稳定、可靠。
    - 监控与机器人的通信连接状态，处理可能出现的通信超时或重连逻辑。

---

### `go2_message_converter.hpp` / `go2_message_converter.cpp`

- **职责**: **消息格式转换器**
- **详细说明**:
    - 这是一个核心的辅助模块，是适配器模式中的“翻译”核心。
    - 提供了静态工具函数，用于在**系统标准消息**和 **Go2 专用消息**之间进行双向转换。
    - **示例**:
        - `fromTwist(const geometry_msgs::msg::Twist&)`: 将标准 `Twist` 消息转换为 `unitree_api::msg::Request`。
        - `toBatteryState(const unitree_go::msg::LowState&)`: 从 `LowState` 消息中提取并转换为标准的电池状态消息。
    - 将转换逻辑集中在此处，使得其他模块（如 `Go2MotionController`）的职责更纯粹。

---

### `go2_quadruped_tricks.hpp` / `go2_quadruped_tricks.cpp`

- **职责**: **四足机器人特殊动作**
- **详细说明**:
    - 这是一个扩展功能模块，用于实现标准运动控制之外的、Go2 特有的高级或趣味性动作。
    - 通过向 `/api/sport/request` 发送特定的模式码 (mode) 和参数来实现，例如：
        - `dance()`: 跳舞
        - `flip()`: 后空翻
        - `standUp()` / `lieDown()`: 起立/趴下
    - 这些接口通常由上层应用在特定场景下调用，不属于常规导航和运动控制的一部分。

## 4. ROS2 API

#### 订阅的话题 (Subscribed Topics)

- `/cmd_vel` (`geometry_msgs::msg::Twist`): 接收上层应用发布的标准速度控制指令。
- `/sportmodestate` (`unitree_go::msg::SportModeState`): 获取机器人的高级运动状态。
- `/lowstate` (`unitree_go::msg::LowState`): 获取机器人底层硬件状态，包括关节、电池、IMU 等。
- `/utlidar/cloud` (`sensor_msgs::msg::PointCloud2`): 接收激光雷达点云数据。
- `/imu/data` (`sensor_msgs::msg::Imu`): 接收 IMU 数据。

#### 发布的话题 (Published Topics)

- `/api/sport/request` (`unitree_api::msg::Request`): 向 Go2 机器人发送转换后的运动控制指令。
- `/robot_state` (`robot_common/RobotState`): 发布标准化的机器人整体状态。
- `/battery_info` (`robot_common/BatteryInfo`): 发布标准化的电池信息。
- (可能) `/processed_lidar` (`sensor_msgs::msg::PointCloud2`): 发布经过预处理的激光雷达数据。

## 5. 配置 (Configuration)

本包的行为通常由位于 `configs/robot_configs/` 目录下的 YAML 文件驱动。这些配置文件定义了：
- 运动限制（最大速度、角速度等）。
- 传感器参数（坐标系、滤波器开关等）。
- 通信话题名称。

## 6. 依赖 (Dependencies)

- `robot_interfaces`: 依赖其中定义的抽象接口。
- `robot_common`: 依赖其中定义的标准消息和工具。
- `unitree_ros2` (`unitree_api`, `unitree_go`): 依赖 Go2 官方提供的消息定义。
- `rclcpp`: ROS2 C++ 客户端库。
- `geometry_msgs`, `sensor_msgs`: ROS2 标准消息库。
