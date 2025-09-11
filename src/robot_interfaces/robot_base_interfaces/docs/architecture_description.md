# Robot Base Interfaces 架构设计说明

**版本**: 2.0
**更新时间**: 2024年12月
**作者**: Robot Navigation Team

## 文档概述

本文档全面阐述 `robot_base_interfaces` 模块的架构设计、实现原理和应用方式。该模块作为四足机器人自主导航系统的核心抽象层，为Go2机器人及其他机器人平台提供了统一的硬件抽象接口。通过深入分析接口设计哲学、具体实现和扩展机制，为开发者提供完整的理解和使用指南。

---

## 1. 架构定位与核心价值

### 1.1 系统架构中的关键位置

`robot_base_interfaces` 在四层系统架构中占据基础核心地位：

```
┌─────────────────────────────────────────────────────────────┐
│                    应用层 (Application Layer)                │
│  ┌───────────────┐ ┌───────────────┐ ┌───────────────────┐  │
│  │ TaskManager   │ │ MapManager    │ │ ChargingScheduler │  │
│  └───────────────┘ └───────────────┘ └───────────────────┘  │
└─────────────────────────┬───────────────────────────────────┘
                          │ 业务逻辑调用
┌─────────────────────────▼───────────────────────────────────┐
│                    算法层 (Algorithm Layer)                  │
│  ┌───────────────┐ ┌───────────────┐ ┌───────────────────┐  │
│  │ SlamManager   │ │ PathPlanner   │ │ ObstacleAvoidance │  │
│  └───────────────┘ └───────────────┘ └───────────────────┘  │
└─────────────────────────┬───────────────────────────────────┘
                          │ 算法实现调用
┌─────────────────────────▼───────────────────────────────────┐
│                  ROS2通信层 (ROS2 Layer)                     │
│    标准ROS2消息类型 (geometry_msgs, sensor_msgs, nav_msgs)    │
└─────────────────────────┬───────────────────────────────────┘
                          │ 消息转换和分发
┌─────────────────────────▼───────────────────────────────────┐
│                 机器人接口层 (Robot Interface Layer)          │
│    ┌─────────────────────────────────────────────────────┐  │
│    │         robot_base_interfaces (核心抽象层)            │  │
│    │  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐    │  │
│    │  │Motion       │ │Sensor       │ │State        │    │  │
│    │  │Interface    │ │Interface    │ │Interface    │    │  │
│    │  └─────────────┘ └─────────────┘ └─────────────┘    │  │
│    │                 ┌─────────────┐                     │  │
│    │                 │Power        │                     │  │
│    │                 │Interface    │                     │  │
│    │                 └─────────────┘                     │  │
│    └─────────────────┬───────────────────────────────────┘  │
│                      │ 适配器实现                             │
│    ┌─────────────────▼───────────────────────────────────┐  │
│    │              Robot Adapters                         │  │
│    │  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐    │  │
│    │  │Go2Adapter   │ │SpotAdapter  │ │CustomAdapter│    │  │
│    │  └─────────────┘ └─────────────┘ └─────────────┘    │  │
│    └─────────────────────────────────────────────────────┘  │
└─────────────────────────┬───────────────────────────────────┘
                         │ 硬件通信协议
┌─────────────────────────▼───────────────────────────────────┐
│                    硬件层 (Hardware Layer)                   │
│           Go2机器人本体 / 其他四足机器人平台                     │
└─────────────────────────────────────────────────────────────┘
```

### 1.2 核心设计哲学：抽象与统一

**面向接口编程 (Interface-Oriented Programming)**：该模块严格遵循依赖倒置原则，通过抽象接口定义系统契约，使上层模块依赖于稳定的抽象而非易变的实现。

**关键价值体现**：

- **硬件无关性**：上层算法无需感知具体机器人类型
- **热插拔能力**：运行时动态切换不同机器人适配器
- **开发并行性**：接口定义完成后，各层可独立开发测试
- **系统可测试性**：通过Mock实现可进行无硬件单元测试

### 1.3 设计原则实践

**依赖倒置原则 (DIP)**：

```cpp
// 上层模块不依赖具体实现
class NavigationController {
    IMotionControllerPtr motion_controller_;  // 依赖抽象
    ISensorInterfacePtr sensor_interface_;    // 依赖抽象
  
    void moveRobot() {
        // 通过接口调用，无需关心具体实现
        motion_controller_->setVelocity({1.0f, 0.0f, 0.5f});
    }
};
```

**接口隔离原则 (ISP)**：

- 将庞大的机器人接口拆分为4个专门化接口
- 客户端只依赖其需要的接口部分
- 特殊功能通过扩展接口提供（如 `IQuadrupedTricks`）

---

## 2. 模块结构与组织架构

### 2.1 整体目录结构

```
robot_base_interfaces/
├── docs/                           # 架构文档目录
│   └── architecture_description.md # 本架构说明文档
├── motion_interface/               # 运动控制抽象接口
│   ├── include/robot_base_interfaces/motion_interface/
│   │   ├── i_motion_controller.hpp      # 核心运动控制接口
│   │   ├── motion_types.hpp             # 运动相关数据类型定义  
│   │   └── i_quadruped_tricks.hpp       # 四足机器人扩展接口
│   ├── src/
│   │   └── motion_types.cpp             # 数据类型实现
│   ├── test/                            # 单元测试
│   ├── CMakeLists.txt
│   └── package.xml
├── sensor_interface/               # 传感器数据抽象接口
│   ├── include/robot_base_interfaces/sensor_interface/
│   │   ├── i_sensor_interface.hpp       # 核心传感器接口
│   │   └── sensor_types.hpp             # 传感器数据类型定义
│   ├── src/
│   │   └── sensor_types.cpp             # 数据类型实现
│   ├── test/                            # 单元测试
│   ├── CMakeLists.txt
│   └── package.xml
├── state_interface/                # 状态监控抽象接口
│   ├── include/robot_base_interfaces/state_interface/
│   │   ├── i_state_monitor.hpp          # 核心状态监控接口
│   │   └── state_types.hpp              # 状态数据类型定义
│   ├── src/
│   │   └── state_types.cpp              # 数据类型实现  
│   ├── test/                            # 单元测试
│   ├── CMakeLists.txt
│   └── package.xml
├── power_interface/                # 电源管理抽象接口
│   ├── include/robot_base_interfaces/power_interface/
│   │   ├── i_power_manager.hpp          # 核心电源管理接口
│   │   └── power_types.hpp              # 电源数据类型定义
│   ├── src/
│   │   └── power_types.cpp              # 数据类型实现
│   ├── test/                            # 单元测试
│   ├── CMakeLists.txt
│   └── package.xml
├── common/                         # 通用组件
│   └── result.hpp                       # 强类型结果与错误码定义
└── README.md                       # 模块使用说明
```

### 2.2 模块化设计特点

**独立性**：每个接口模块独立编译，具有独立的包定义
**可选性**：上层应用可选择性依赖所需接口模块
**扩展性**：新增接口类型无需修改现有模块
**一致性**：所有接口遵循统一的设计模式和命名规范

---

## 3. 核心接口深度分析

### 3.1 运动控制接口 (IMotionController)

#### 接口设计架构

`IMotionController` 是运动控制的核心抽象，完全基于Go2机器人能力设计，同时为其他机器人预留扩展空间。

**类层次结构**：

```cpp
class IMotionController {
public:
    // ============= 核心接口组 =============
  
    // 初始化管理
    virtual MotionResult initialize() = 0;
    virtual MotionResult shutdown() = 0;
    virtual MotionCapabilities getCapabilities() const = 0;
  
    // 基础运动控制 (所有机器人通用)
    virtual MotionResult setVelocity(const Velocity& velocity) = 0;
    virtual MotionResult setPosture(const Posture& posture) = 0;
    virtual MotionResult emergencyStop(EmergencyStopLevel level) = 0;
  
    // 运动模式管理 (基于Go2设计)
    virtual MotionResult switchMode(MotionMode mode) = 0;
    virtual MotionResult setGaitType(GaitType gait) = 0;
    virtual MotionResult balanceStand() = 0;
    virtual MotionResult standUp() = 0;
    virtual MotionResult standDown() = 0;
  
    // Go2特有高级功能 (其他机器人可返回CAPABILITY_LIMITED)
    virtual MotionResult performDance(int dance_type = 1) = 0;
    virtual MotionResult frontFlip() = 0;
    virtual MotionResult frontJump() = 0;
    virtual MotionResult hello() = 0;
  
    // 状态查询与监控
    virtual MotionState getMotionState() const = 0;
    virtual bool isOperational() const = 0;
    virtual uint32_t getErrorCode() const = 0;
  
    // 异步事件回调
    virtual void setStateCallback(std::function<void(const MotionState&)> callback) = 0;
    virtual void setErrorCallback(std::function<void(uint32_t, const std::string&)> callback) = 0;
  
    // 扩展接口
    virtual MotionResult executeCustomCommand(const std::string& command_name, 
                                            const std::string& parameters = "");
};
```

#### 核心数据类型设计

**Velocity结构体**：

```cpp
struct Velocity {
    float linear_x = 0.0f;   // 前进速度 (m/s) 
    float linear_y = 0.0f;   // 侧移速度 (m/s, Go2支持)
    float linear_z = 0.0f;   // 垂直速度 (预留)
    float angular_x = 0.0f;  // 滚转角速度 (预留)
    float angular_y = 0.0f;  // 俯仰角速度 (预留) 
    float angular_z = 0.0f;  // 偏航角速度 (rad/s)
};
```

**MotionCapabilities结构体**：

```cpp
struct MotionCapabilities {
    // Go2具体能力参数
    float max_linear_velocity = 1.5f;    // Go2: 1.5 m/s
    float max_angular_velocity = 2.0f;   // Go2: 2.0 rad/s
    float max_lateral_velocity = 0.8f;   // Go2侧移能力
  
    // 特殊能力标志
    bool can_climb_stairs = true;        // Go2爬楼梯能力
    bool can_dance = true;               // Go2舞蹈能力
    bool can_jump = true;                // Go2跳跃能力
    bool can_flip = true;                // Go2翻滚能力
  
    // 支持的模式和步态
    std::vector<MotionMode> supported_modes;
    std::vector<GaitType> supported_gaits;
};
```

#### Go2适配实现示例

**关键实现点**：

```cpp
// Go2MotionController.cpp 实现摘要
MotionResult Go2MotionController::setVelocity(const Velocity& velocity) {
    // 1. 参数验证 - 确保在Go2能力范围内
    if (!validateVelocity(velocity)) {
        return MotionResult::CAPABILITY_LIMITED;
    }
  
    // 2. 构造Go2特定的API请求
    auto request_msg = std::make_unique<unitree_api::msg::Request>();
    request_msg->header.identity.api_id = 1003; // Go2移动控制API ID
  
    // 3. 参数编码为Go2格式
    std::string params = std::format("{},{},{}", 
                                   velocity.linear_x, 
                                   velocity.linear_y, 
                                   velocity.angular_z);
    request_msg->parameter = params;
  
    // 4. 通过ROS2发布到Go2
    api_request_pub_->publish(std::move(request_msg));
  
    // 5. 更新内部状态并触发回调
    updateMotionState();
  
    return MotionResult::SUCCESS;
}
```

### 3.2 传感器接口 (ISensorInterface)

#### 接口架构设计

`ISensorInterface` 提供统一的传感器数据访问方式，重点适配Go2的传感器配置：

**核心功能组织**：

```cpp
class ISensorInterface {
public:
    // ============= 生命周期管理 =============
    virtual bool initialize() = 0;
    virtual bool shutdown() = 0;
    virtual std::vector<SensorInfo> getAvailableSensors() const = 0;
  
    // ============= 数据获取接口 =============
  
    // 通用数据获取
    virtual std::shared_ptr<SensorData> getLatestData(SensorType sensor_type) const = 0;
  
    // Go2主要传感器专用接口
    virtual std::shared_ptr<PointCloudData> getLatestPointCloud() const = 0;  // Livox Mid360
    virtual std::shared_ptr<IMUData> getLatestIMU() const = 0;                // 内置IMU
  
    // 扩展传感器接口(默认返回nullptr)
    virtual std::shared_ptr<LaserScanData> getLatestLaserScan() const;        // 2D激光雷达
    virtual std::shared_ptr<ImageData> getLatestImage(int camera_id = 0) const; // 摄像头
  
    // ============= 回调接口 =============
    virtual void setPointCloudCallback(std::function<void(const std::shared_ptr<PointCloudData>&)> callback) = 0;
    virtual void setIMUCallback(std::function<void(const std::shared_ptr<IMUData>&)> callback) = 0;
  
    // ============= 传感器控制 =============
    virtual bool startSensor(SensorType sensor_type) = 0;
    virtual bool setSensorFrequency(SensorType sensor_type, float frequency) = 0;
    virtual bool calibrateSensor(SensorType sensor_type) = 0;
};
```

#### 传感器数据类型设计

**统一基类设计**：

```cpp
struct SensorData {
    uint64_t timestamp_ns = 0;           // 统一时间戳
    std::string frame_id;                // 坐标系ID
    bool is_valid = false;               // 数据有效性
  
    virtual ~SensorData() = default;
    virtual bool hasData() const { return is_valid; }
    virtual size_t getDataSize() const = 0;
};

struct PointCloudData : public SensorData {
    struct Point3D {
        float x, y, z;
        float intensity = 0.0f;          // 强度信息
        uint8_t ring = 0;                // 线束编号(多线激光雷达)
    };
  
    std::vector<Point3D> points;
    float min_range = 0.0f;              // 有效距离范围
    float max_range = 100.0f;
  
    size_t getDataSize() const override { return points.size(); }
};

struct IMUData : public SensorData {
    struct Vector3 { float x, y, z; };
    struct Quaternion { float x, y, z, w; };
  
    Quaternion orientation;              // 方向四元数
    Vector3 angular_velocity;            // 角速度
    Vector3 linear_acceleration;         // 线加速度
  
    // 协方差矩阵(可选)
    std::array<float, 9> orientation_covariance;
    std::array<float, 9> angular_velocity_covariance; 
    std::array<float, 9> linear_acceleration_covariance;
  
    size_t getDataSize() const override { return 1; }
};
```

### 3.3 状态监控接口 (IStateMonitor)

#### 全面状态监控架构

`IStateMonitor` 对应Go2的完整状态监控能力，包括SportModeState和LowState：

**状态监控层次**：

```cpp
class IStateMonitor {
public:
    // ============= 基础状态监控 =============
    virtual RobotState getRobotState() const = 0;           // 基础状态
    virtual HealthLevel getHealthStatus() const = 0;        // 健康等级  
    virtual float getHealthScore() const = 0;               // 健康评分(0-1)
    virtual bool isOperational() const = 0;                 // 可操作性
  
    // ============= Go2详细状态监控 =============
    virtual DetailedRobotState getDetailedState() const = 0;         // 详细状态
    virtual MotorInfo getMotorInfo(uint8_t motor_id) const = 0;       // 单个电机状态
    virtual std::vector<MotorInfo> getAllMotorInfo() const = 0;      // 全部20个电机
    virtual FootInfo getFootInfo(uint8_t foot_id) const = 0;         // 单个足端状态  
    virtual std::vector<FootInfo> getAllFootInfo() const = 0;       // 全部4个足端
  
    // ============= 系统诊断 =============
    virtual std::vector<DiagnosticInfo> getSystemDiagnostics() const = 0;
    virtual bool performSystemCheck() = 0;                  // 系统自检
    virtual std::map<SystemModule, bool> getSystemCheckResults() const = 0;
  
    // ============= 告警管理 =============
    virtual std::vector<AlertInfo> getActiveAlerts() const = 0;
    virtual std::vector<AlertInfo> getAlertsByType(AlertType type) const = 0;
    virtual int clearResolvedAlerts(uint32_t alert_code = 0) = 0;
  
    // ============= 性能统计 =============
    virtual PerformanceStats getPerformanceStats() const = 0;
    virtual uint64_t getUptimeSeconds() const = 0;
  
    // ============= 事件回调 =============
    virtual void setStateChangeCallback(
        std::function<void(RobotState old_state, RobotState new_state)> callback) = 0;
    virtual void setHealthChangeCallback(
        std::function<void(HealthLevel old_level, HealthLevel new_level, float score)> callback) = 0;
    virtual void setAlertCallback(
        std::function<void(const AlertInfo& alert)> callback) = 0;
};
```

#### Go2状态数据映射

**电机状态监控**：

```cpp
struct MotorInfo {
    uint8_t motor_id;                    // 电机ID (0-19)
    float temperature;                   // 温度 (°C)
    float torque;                        // 转矩 (N⋅m)
    float position;                      // 位置 (rad)
    float velocity;                      // 速度 (rad/s)
    uint32_t fault_code;                 // 故障代码
    bool is_enabled;                     // 使能状态
  
    // Go2 LowState.motor_state 映射
    static MotorInfo fromGo2MotorState(const unitree_go::msg::MotorState& state, uint8_t id);
};

struct FootInfo {
    uint8_t foot_id;                     // 足端ID (0-3: FL,FR,RL,RR)  
    float force_z;                       // 垂直力 (N)
    bool contact;                        // 接触状态
    Position position;                   // 足端位置
    Velocity velocity;                   // 足端速度
  
    // Go2 LowState.foot_force 和 foot_position_body 映射
    static FootInfo fromGo2FootData(const unitree_go::msg::LowState& state, uint8_t id);
};
```

### 3.4 电源管理接口 (IPowerManager)

#### 完整电源管理体系

`IPowerManager` 基于Go2的BMS(电池管理系统)设计，支持完整的电源生命周期管理：

**电源管理功能组织**：

```cpp
class IPowerManager {
public:
    // ============= 电池状态查询 =============
    virtual BatteryInfo getBatteryInfo() const = 0;         // 完整电池信息
    virtual float getBatteryPercentage() const = 0;         // 电量百分比
    virtual float getBatteryVoltage() const = 0;            // 电压
    virtual float getBatteryCurrent() const = 0;            // 电流 (正充负放)
    virtual float getBatteryTemperature() const = 0;        // 温度
    virtual BatteryHealth getBatteryHealth() const = 0;     // 健康状态
  
    // ============= 充电管理 =============
    virtual ChargingStatus getChargingStatus() const = 0;   // 充电状态
    virtual bool requestCharging(ChargingType type = ChargingType::WIRELESS) = 0;
    virtual bool requestStopCharging() = 0;
    virtual bool needsCharging(float threshold = -1.0f) const = 0;
    virtual uint32_t getEstimatedChargeTime() const = 0;    // 预计充满时间
  
    // ============= 充电站管理 =============
    virtual bool registerChargingStation(const ChargingStationInfo& station_info) = 0;
    virtual std::shared_ptr<ChargingStationInfo> findNearestChargingStation(
        const std::vector<float>& current_position) const = 0;
  
    // ============= 功耗管理 =============
    virtual float getCurrentPowerConsumption() const = 0;   // 当前功耗
    virtual float getAveragePowerConsumption(uint32_t duration_seconds = 60) const = 0;
    virtual bool setPowerProfile(const PowerConsumptionProfile& profile) = 0;
    virtual bool enablePowerSaving(bool enable) = 0;
  
    // ============= 电源控制 =============
    virtual bool requestSystemShutdown(uint32_t delay_seconds = 5) = 0;
    virtual bool enterSleepMode(uint32_t duration_seconds = 0) = 0;
  
    // ============= 安全保护 =============
    virtual bool hasBatteryFault() const = 0;
    virtual bool isChargingSafe() const = 0;
    virtual bool isTemperatureSafe() const = 0;
    virtual bool calibrateBattery() = 0;
};
```

#### Go2电源数据结构

**电池信息结构**：

```cpp
struct BatteryInfo {
    float percentage = 0.0f;             // 电量百分比 (0-100)
    float voltage = 0.0f;                // 电压 (V)
    float current = 0.0f;                // 电流 (A, 正充负放)
    float power = 0.0f;                  // 功率 (W)
    float temperature = 0.0f;            // 温度 (°C)
    uint16_t cycles = 0;                 // 循环次数
    BatteryHealth health = BatteryHealth::UNKNOWN;
    uint32_t fault_code = 0;             // 故障代码
    uint64_t timestamp_ns = 0;           // 时间戳
  
    // Go2 BmsState 映射构造
    static BatteryInfo fromGo2BmsState(const unitree_go::msg::BmsState& bms_state);
};

enum class BatteryHealth {
    EXCELLENT = 0,    // >90% 优秀
    GOOD = 1,         // 70-90% 良好  
    FAIR = 2,         // 50-70% 一般
    POOR = 3,         // 30-50% 较差
    CRITICAL = 4,     // <30% 严重
    UNKNOWN = 5       // 未知
};

struct ChargingStationInfo {
    std::string station_id;              // 充电站ID
    std::string name;                    // 显示名称
    std::vector<float> position;         // 位置 [x,y,z]
    ChargingType type;                   // 充电类型
    bool is_available = true;            // 是否可用
    bool is_occupied = false;            // 是否被占用
    float max_power = 0.0f;              // 最大充电功率 (W)
  
    // 计算与当前位置的距离
    float distanceFrom(const std::vector<float>& current_pos) const;
};
```

---

## 4. 高级设计特性

### 4.1 强类型安全设计

#### Result`<T>` 模板类

为提高接口可靠性，引入强类型Result模板：

```cpp
template <typename T>
struct Result {
    bool ok = false;
    ErrorCode code = ErrorCode::OK;
    std::string message;
    std::optional<T> value;
  
    static Result<T> success(T v) {
        Result<T> r;
        r.ok = true;
        r.code = ErrorCode::OK;
        r.value = std::move(v);
        return r;
    }
  
    static Result<T> failure(ErrorCode c, std::string msg = "") {
        Result<T> r;
        r.ok = false;
        r.code = c;
        r.message = std::move(msg);
        return r;
    }
};

// 使用示例
auto result = sensor_interface->getLatestPointCloudResult();
if (result.ok) {
    auto point_cloud = result.value.value();
    // 处理点云数据...
} else {
    RCLCPP_ERROR(logger_, "获取点云失败: %s", result.message.c_str());
}
```

#### 枚举类型设计

所有枚举使用强类型enum class，避免隐式转换：

```cpp
enum class MotionResult {
    SUCCESS = 0,
    INVALID_PARAMETER = 1,
    CAPABILITY_LIMITED = 2,
    EMERGENCY_STOP = 3,
    COMMUNICATION_ERROR = 4,
    UNKNOWN_ERROR = 99
};

enum class RobotType {
    GO2 = 0,
    SPOT = 1,
    ANYMAL = 2,
    GENERIC = 99,
    UNKNOWN = 255
};
```

### 4.2 异步编程模型

#### 事件驱动架构

所有接口支持回调机制，实现事件驱动编程：

```cpp
// 运动控制状态变化回调
motion_controller->setStateCallback([this](const MotionState& state) {
    if (state.current_mode == MotionMode::EMERGENCY_STOP) {
        handleEmergencyStop();
    }
    updateNavigationState(state);
});

// 传感器数据流回调
sensor_interface->setPointCloudCallback([this](const auto& cloud_data) {
    // 高频数据处理，需要高效实现
    obstacle_detector_->processPointCloud(cloud_data);
});

// 健康状态监控回调
state_monitor->setHealthChangeCallback([this](HealthLevel old_level, HealthLevel new_level, float score) {
    if (new_level == HealthLevel::CRITICAL) {
        requestMaintenanceAlert();
    }
    publishHealthStatus(new_level, score);
});
```

#### 线程安全设计

**回调线程语义**：

- 回调函数可能在内部工作线程中执行
- 调用方需确保回调函数线程安全
- 建议回调中只进行数据拷贝和事件通知，复杂处理移交独立线程

```cpp
class SafeCallbackHandler {
private:
    std::mutex callback_mutex_;
    std::queue<MotionState> state_queue_;
    std::thread processing_thread_;
  
public:
    void onMotionStateChange(const MotionState& state) {
        // 快速拷贝，避免阻塞回调线程
        {
            std::lock_guard<std::mutex> lock(callback_mutex_);
            state_queue_.push(state);
        }
      
        // 通知处理线程
        condition_variable_.notify_one();
    }
  
    void processingLoop() {
        while (running_) {
            std::unique_lock<std::mutex> lock(callback_mutex_);
            condition_variable_.wait(lock, [this] { return !state_queue_.empty() || !running_; });
          
            while (!state_queue_.empty()) {
                auto state = state_queue_.front();
                state_queue_.pop();
                lock.unlock();
              
                // 复杂处理逻辑
                processMotionState(state);
              
                lock.lock();
            }
        }
    }
};
```

### 4.3 能力发现与适配

#### 动态能力查询

通过能力查询机制实现运行时适配：

```cpp
// 查询机器人具体能力
auto motion_capabilities = motion_controller->getCapabilities();

// 根据能力调整控制策略
if (motion_capabilities.can_lateral_move) {
    // Go2支持侧移，可以使用更复杂的路径规划
    path_planner_->enableLateralMovement(true);
    max_velocity_.linear_y = motion_capabilities.max_lateral_velocity;
} else {
    // 其他机器人可能不支持侧移
    path_planner_->enableLateralMovement(false);
    max_velocity_.linear_y = 0.0f;
}

// 检查特殊功能支持
if (motion_capabilities.can_climb_stairs) {
    navigation_modes_.insert(NavigationMode::STAIR_CLIMBING);
}

// 根据传感器能力调整感知策略  
auto available_sensors = sensor_interface->getAvailableSensors();
for (const auto& sensor_info : available_sensors) {
    if (sensor_info.type == SensorType::LIDAR_3D) {
        enablePointCloudProcessing(true);
        lidar_range_ = sensor_info.max_range;
    }
}
```

#### 优雅降级机制

对于不支持的功能，提供优雅降级：

```cpp
// Go2专有功能的优雅降级示例
MotionResult performSpecialAction() {
    // 尝试执行Go2特有动作
    auto result = motion_controller_->hello();
  
    if (result == MotionResult::CAPABILITY_LIMITED) {
        // 不支持该动作，使用替代方案
        RCLCPP_WARN(logger_, "机器人不支持打招呼动作，使用替代行为");
        return motion_controller_->setPosture({0.0f, -0.2f, 0.0f, 0.35f}); // 点头动作
    }
  
    return result;
}

// 传感器数据获取的降级处理
std::shared_ptr<ObstacleMap> buildObstacleMap() {
    // 首选：使用3D点云构建障碍物地图
    auto point_cloud = sensor_interface_->getLatestPointCloud();
    if (point_cloud && point_cloud->points.size() > 100) {
        return obstacle_mapper_->fromPointCloud(point_cloud);
    }
  
    // 降级：使用2D激光雷达数据
    auto laser_scan = sensor_interface_->getLatestLaserScan();
    if (laser_scan) {
        return obstacle_mapper_->fromLaserScan(laser_scan);
    }
  
    // 最后降级：基于运动状态的简单避障
    RCLCPP_WARN(logger_, "传感器数据不可用，使用基础避障模式");
    return obstacle_mapper_->fromMotionState(motion_controller_->getMotionState());
}
```

---

## 5. Go2机器人完整适配分析

### 5.1 Go2硬件能力映射

#### 运动控制能力完整映射

| 接口方法            | Go2 API ID | 功能说明     | 参数格式          |
| ------------------- | ---------- | ------------ | ----------------- |
| `setVelocity()`   | 1003       | 速度控制     | `"vx,vy,vyaw"`  |
| `balanceStand()`  | 1002       | 平衡站立     | 无参数            |
| `standUp()`       | 1004       | 站起动作     | 无参数            |
| `standDown()`     | 1005       | 趴下动作     | 无参数            |
| `sit()`           | 1009       | 坐下动作     | 无参数            |
| `recoveryStand()` | 1006       | 恢复站立     | 无参数            |
| `performDance(1)` | 1007       | 舞蹈1        | 无参数            |
| `performDance(2)` | 1008       | 舞蹈2        | 无参数            |
| `hello()`         | 1016       | 打招呼动作   | 无参数            |
| `stretch()`       | 1017       | 伸展动作     | 无参数            |
| `frontFlip()`     | 1030       | 前翻动作     | 无参数            |
| `frontJump()`     | 1031       | 前跳动作     | 无参数            |
| `setSpeedLevel()` | 1015       | 速度等级设置 | `"level"` (1-9) |

#### 传感器数据完整映射

| 接口方法                  | Go2话题/消息                  | 数据类型                     | 频率  |
| ------------------------- | ----------------------------- | ---------------------------- | ----- |
| `getLatestPointCloud()` | `/utlidar/cloud`            | `sensor_msgs::PointCloud2` | ~10Hz |
| `getLatestIMU()`        | `/imu/data` 或 LowState.imu | `sensor_msgs::Imu`         | 500Hz |
| `getLatestOdometry()`   | SportModeState计算            | `nav_msgs::Odometry`       | 500Hz |

#### 状态监控完整映射

| 接口方法             | Go2消息源      | 具体字段                                         | 更新频率 |
| -------------------- | -------------- | ------------------------------------------------ | -------- |
| `getRobotState()`  | SportModeState | `mode`, `gait_type`, `position`            | 500Hz    |
| `getMotorInfo()`   | LowState       | `motor_state[0-19]`                            | 500Hz    |
| `getFootInfo()`    | LowState       | `foot_force[0-3]`, `foot_position_body[0-3]` | 500Hz    |
| `getBatteryInfo()` | LowState       | `bms` (BmsState)                               | 100Hz    |

### 5.2 Go2适配器实现架构

#### 完整适配器类设计

```cpp
class Go2Adapter : public IRobotAdapter {
private:
    // 各接口的具体实现
    std::unique_ptr<Go2MotionController> motion_controller_;
    std::unique_ptr<Go2SensorInterface> sensor_interface_;  
    std::unique_ptr<Go2StateMonitor> state_monitor_;
    std::unique_ptr<Go2PowerManager> power_manager_;
  
    // Go2通信组件
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr api_request_pub_;
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sport_state_sub_;
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  
    // 状态缓存和同步
    std::mutex state_mutex_;
    unitree_go::msg::SportModeState latest_sport_state_;
    unitree_go::msg::LowState latest_low_state_;
  
public:
    bool initialize() override {
        // 1. 初始化ROS2通信
        setupRos2Communication();
      
        // 2. 创建各接口实现
        motion_controller_ = std::make_unique<Go2MotionController>(shared_from_this());
        sensor_interface_ = std::make_unique<Go2SensorInterface>(shared_from_this());
        state_monitor_ = std::make_unique<Go2StateMonitor>(shared_from_this());
        power_manager_ = std::make_unique<Go2PowerManager>(shared_from_this());
      
        // 3. 初始化各接口
        return motion_controller_->initialize() &&
               sensor_interface_->initialize() &&
               state_monitor_->initialize() &&
               power_manager_->initialize();
    }
  
    // 接口获取方法
    IMotionController* getMotionController() override { return motion_controller_.get(); }
    ISensorInterface* getSensorInterface() override { return sensor_interface_.get(); }
    IStateMonitor* getStateMonitor() override { return state_monitor_.get(); }
    IPowerManager* getPowerManager() override { return power_manager_.get(); }
  
    RobotType getRobotType() const override { return RobotType::GO2; }
  
private:
    void setupRos2Communication() {
        // 发布器设置
        api_request_pub_ = this->create_publisher<unitree_api::msg::Request>(
            "/api/sport/request", 10);
          
        // 订阅器设置
        sport_state_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
            "/sportmodestate", 10,
            std::bind(&Go2Adapter::sportStateCallback, this, std::placeholders::_1));
          
        low_state_sub_ = this->create_subscription<unitree_go::msg::LowState>(
            "/lowstate", 10,
            std::bind(&Go2Adapter::lowStateCallback, this, std::placeholders::_1));
          
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/utlidar/cloud", 10,
            std::bind(&Go2Adapter::lidarCallback, this, std::placeholders::_1));
    }
  
    void sportStateCallback(const unitree_go::msg::SportModeState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_sport_state_ = *msg;
      
        // 通知所有子接口状态更新
        motion_controller_->onSportStateUpdate(*msg);
        state_monitor_->onSportStateUpdate(*msg);
    }
  
    void lowStateCallback(const unitree_go::msg::LowState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_low_state_ = *msg;
      
        // 通知相关子接口
        state_monitor_->onLowStateUpdate(*msg);
        power_manager_->onBmsStateUpdate(msg->bms);
    }
};
```

---

## 6. 实际应用场景与代码示例

### 6.1 导航系统集成示例

#### 完整导航控制器实现

```cpp
class NavigationController : public rclcpp::Node {
private:
    // 通过接口依赖，而非具体实现
    IMotionControllerPtr motion_controller_;
    ISensorInterfacePtr sensor_interface_;
    IStateMonitorPtr state_monitor_;
    IPowerManagerPtr power_manager_;
  
    // 导航组件
    std::unique_ptr<PathPlanner> path_planner_;
    std::unique_ptr<ObstacleDetector> obstacle_detector_;
  
public:
    NavigationController(IRobotAdapterPtr robot_adapter) 
        : Node("navigation_controller") {
        // 获取所有需要的接口
        motion_controller_ = robot_adapter->getMotionController();
        sensor_interface_ = robot_adapter->getSensorInterface();
        state_monitor_ = robot_adapter->getStateMonitor();
        power_manager_ = robot_adapter->getPowerManager();
      
        // 初始化导航组件
        initializeNavigation();
      
        // 设置回调
        setupCallbacks();
    }
  
private:
    void initializeNavigation() {
        // 根据机器人能力配置路径规划器
        auto motion_caps = motion_controller_->getCapabilities();
      
        PathPlannerConfig config;
        config.max_linear_velocity = motion_caps.max_linear_velocity;
        config.max_angular_velocity = motion_caps.max_angular_velocity;
        config.can_move_lateral = motion_caps.can_lateral_move;
        config.can_climb_stairs = motion_caps.can_climb_stairs;
      
        path_planner_ = std::make_unique<PathPlanner>(config);
      
        // 根据传感器能力配置障碍物检测
        auto available_sensors = sensor_interface_->getAvailableSensors();
      
        ObstacleDetectorConfig obs_config;
        for (const auto& sensor_info : available_sensors) {
            if (sensor_info.type == SensorType::LIDAR_3D) {
                obs_config.use_3d_lidar = true;
                obs_config.lidar_max_range = sensor_info.max_range;
            }
        }
      
        obstacle_detector_ = std::make_unique<ObstacleDetector>(obs_config);
    }
  
    void setupCallbacks() {
        // 点云数据处理
        sensor_interface_->setPointCloudCallback(
            [this](const std::shared_ptr<PointCloudData>& cloud) {
                obstacle_detector_->updatePointCloud(cloud);
              
                // 检查是否需要紧急停止
                if (obstacle_detector_->hasImmediateObstacle()) {
                    motion_controller_->emergencyStop(EmergencyStopLevel::SOFT_STOP);
                }
            });
      
        // 状态监控
        state_monitor_->setHealthChangeCallback(
            [this](HealthLevel old_level, HealthLevel new_level, float score) {
                if (new_level == HealthLevel::CRITICAL) {
                    RCLCPP_ERROR(this->get_logger(), "机器人健康状态危急，停止导航");
                    stopNavigation();
                }
            });
      
        // 电源管理
        power_manager_->setLowBatteryCallback(
            [this](float percentage) {
                if (percentage < 20.0f) {
                    RCLCPP_WARN(this->get_logger(), "电量不足%.1f%%，寻找充电站", percentage);
                    auto nearest_station = power_manager_->findNearestChargingStation(getCurrentPosition());
                    if (nearest_station) {
                        navigateToChargingStation(*nearest_station);
                    }
                }
            });
    }
  
    void navigateToGoal(const geometry_msgs::msg::PoseStamped& goal) {
        // 生成路径
        auto current_pose = getCurrentPose();
        auto path = path_planner_->planPath(current_pose, goal);
      
        if (!path) {
            RCLCPP_ERROR(this->get_logger(), "路径规划失败");
            return;
        }
      
        // 执行路径跟随
        for (const auto& waypoint : path->waypoints) {
            if (!executeWaypoint(waypoint)) {
                RCLCPP_ERROR(this->get_logger(), "路径执行失败");
                break;
            }
        }
    }
  
    bool executeWaypoint(const Waypoint& waypoint) {
        // 计算控制指令
        auto current_pose = getCurrentPose();
        auto velocity_cmd = calculateVelocityCommand(current_pose, waypoint);
      
        // 安全检查
        if (!isSafeToMove(velocity_cmd)) {
            RCLCPP_WARN(this->get_logger(), "检测到障碍物，暂停移动");
            motion_controller_->setVelocity({0.0f, 0.0f, 0.0f});
            return false;
        }
      
        // 发送运动指令
        auto result = motion_controller_->setVelocity(velocity_cmd);
        if (result != MotionResult::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "运动指令执行失败: %d", static_cast<int>(result));
            return false;
        }
      
        return true;
    }
  
    bool isSafeToMove(const Velocity& velocity) {
        // 检查障碍物
        if (obstacle_detector_->hasObstacleInPath(velocity)) {
            return false;
        }
      
        // 检查机器人状态
        if (!state_monitor_->isOperational()) {
            return false;
        }
      
        // 检查电量
        if (power_manager_->isBatteryCritical()) {
            return false;
        }
      
        return true;
    }
  
    std::vector<float> getCurrentPosition() {
        auto motion_state = motion_controller_->getMotionState();
        return {motion_state.position.x, motion_state.position.y, motion_state.position.z};
    }
  
    void navigateToChargingStation(const ChargingStationInfo& station) {
        RCLCPP_INFO(this->get_logger(), "导航到充电站: %s", station.name.c_str());
      
        geometry_msgs::msg::PoseStamped charging_goal;
        charging_goal.pose.position.x = station.position[0];
        charging_goal.pose.position.y = station.position[1];
        charging_goal.pose.position.z = station.position[2];
      
        navigateToGoal(charging_goal);
      
        // 到达后开始充电
        power_manager_->requestCharging(station.type);
    }
};
```

### 6.2 多机器人支持示例

#### 工厂模式与适配器切换

```cpp
class MultiRobotNavigationSystem {
private:
    std::map<std::string, IRobotAdapterPtr> robot_adapters_;
    std::map<std::string, std::unique_ptr<NavigationController>> nav_controllers_;
  
public:
    bool addRobot(const std::string& robot_id, RobotType robot_type) {
        // 通过工厂创建适配器
        auto adapter = RobotAdapterFactory::createAdapter(robot_type);
        if (!adapter || !adapter->initialize()) {
            RCLCPP_ERROR(rclcpp::get_logger("multi_robot_system"), 
                        "无法初始化机器人: %s", robot_id.c_str());
            return false;
        }
      
        // 创建导航控制器
        auto nav_controller = std::make_unique<NavigationController>(adapter);
      
        // 保存引用
        robot_adapters_[robot_id] = adapter;
        nav_controllers_[robot_id] = std::move(nav_controller);
      
        RCLCPP_INFO(rclcpp::get_logger("multi_robot_system"), 
                   "成功添加机器人: %s, 类型: %d", robot_id.c_str(), static_cast<int>(robot_type));
        return true;
    }
  
    bool navigateRobot(const std::string& robot_id, const geometry_msgs::msg::PoseStamped& goal) {
        auto it = nav_controllers_.find(robot_id);
        if (it == nav_controllers_.end()) {
            return false;
        }
      
        it->second->navigateToGoal(goal);
        return true;
    }
  
    void printRobotStatus() {
        for (const auto& [robot_id, adapter] : robot_adapters_) {
            auto motion_controller = adapter->getMotionController();
            auto state_monitor = adapter->getStateMonitor();
            auto power_manager = adapter->getPowerManager();
          
            std::cout << "机器人 " << robot_id << ":\n";
            std::cout << "  类型: " << getRobotTypeName(adapter->getRobotType()) << "\n";
            std::cout << "  状态: " << (motion_controller->isOperational() ? "正常" : "异常") << "\n";
            std::cout << "  健康评分: " << state_monitor->getHealthScore() << "\n";
            std::cout << "  电量: " << power_manager->getBatteryPercentage() << "%\n";
          
            // 打印机器人特定能力
            auto capabilities = motion_controller->getCapabilities();
            std::cout << "  能力:\n";
            std::cout << "    最大速度: " << capabilities.max_linear_velocity << " m/s\n";
            std::cout << "    可以侧移: " << (capabilities.can_lateral_move ? "是" : "否") << "\n";
            std::cout << "    可以跳舞: " << (capabilities.can_dance ? "是" : "否") << "\n";
            std::cout << "\n";
        }
    }
  
private:
    std::string getRobotTypeName(RobotType type) {
        switch (type) {
            case RobotType::GO2: return "宇树Go2";
            case RobotType::SPOT: return "波士顿动力Spot";
            case RobotType::ANYMAL: return "ANYmal";
            case RobotType::GENERIC: return "通用四足机器人";
            default: return "未知类型";
        }
    }
};

// 使用示例
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
  
    MultiRobotNavigationSystem system;
  
    // 添加不同类型的机器人
    system.addRobot("go2_001", RobotType::GO2);
    system.addRobot("spot_001", RobotType::SPOT);        // 假设已实现Spot适配器
    system.addRobot("custom_001", RobotType::GENERIC);   // 假设已实现通用适配器
  
    // 打印机器人状态
    system.printRobotStatus();
  
    // 为每个机器人分配导航任务
    geometry_msgs::msg::PoseStamped goal1;
    goal1.pose.position.x = 5.0;
    goal1.pose.position.y = 0.0;
    system.navigateRobot("go2_001", goal1);
  
    geometry_msgs::msg::PoseStamped goal2;  
    goal2.pose.position.x = 0.0;
    goal2.pose.position.y = 5.0;
    system.navigateRobot("spot_001", goal2);
  
    rclcpp::spin(std::make_shared<rclcpp::Node>("multi_robot_node"));
    rclcpp::shutdown();
  
    return 0;
}
```

---

## 7. 扩展性与未来发展

### 7.1 新机器人平台适配

#### 适配器开发指南

为新机器人平台开发适配器需遵循以下步骤：

**1. 能力分析与映射**

```cpp
// 1. 分析目标机器人能力
class SpotMotionCapabilities {
public:
    static MotionCapabilities getCapabilities() {
        MotionCapabilities caps;
      
        // Spot 具体能力参数
        caps.max_linear_velocity = 1.6f;     // Spot: 1.6 m/s
        caps.max_angular_velocity = 2.0f;    // 与Go2相近
        caps.max_lateral_velocity = 0.5f;    // Spot侧移能力较弱
      
        // Spot特殊能力
        caps.can_climb_stairs = true;        // Spot爬楼梯能力强  
        caps.can_dance = false;              // Spot不支持舞蹈
        caps.can_jump = false;               // Spot不支持跳跃
        caps.can_flip = false;               // Spot不支持翻滚
      
        // 支持的模式 (映射到标准模式)
        caps.supported_modes = {
            MotionMode::IDLE,
            MotionMode::BALANCE_STAND,
            MotionMode::LOCOMOTION,
            MotionMode::LIE_DOWN,
            MotionMode::SIT
        };
      
        caps.supported_gaits = {
            GaitType::IDLE,
            GaitType::TROT,
            GaitType::RUN,
            GaitType::CLIMB_STAIR,
            GaitType::DOWN_STAIR
        };
      
        return caps;
    }
};
```

**2. 通信协议适配**

```cpp
class SpotMotionController : public IMotionController {
private:
    // Spot SDK通信组件
    std::unique_ptr<bosdyn::client::RobotCommandClient> command_client_;
    std::unique_ptr<bosdyn::client::RobotStateClient> state_client_;
  
public:
    MotionResult setVelocity(const Velocity& velocity) override {
        // 1. 验证参数
        if (!validateVelocity(velocity)) {
            return MotionResult::CAPABILITY_LIMITED;
        }
      
        // 2. 转换为Spot格式
        bosdyn::api::SE2Velocity se2_velocity;
        se2_velocity.mutable_linear()->set_x(velocity.linear_x);
        se2_velocity.mutable_linear()->set_y(velocity.linear_y);
        se2_velocity.set_angular(velocity.angular_z);
      
        // 3. 构造Spot运动指令
        bosdyn::api::RobotCommand robot_command;
        auto* mobility_command = robot_command.mutable_mobility_command();
        auto* se2_trajectory = mobility_command->mutable_se2_trajectory_request()->mutable_trajectory();
      
        // 4. 发送到Spot
        auto result = command_client_->RobotCommand(robot_command);
      
        if (result) {
            return MotionResult::SUCCESS;
        } else {
            return MotionResult::COMMUNICATION_ERROR;
        }
    }
  
    MotionResult performDance(int dance_type) override {
        // Spot不支持舞蹈，返回能力限制
        (void)dance_type;
        return MotionResult::CAPABILITY_LIMITED;
    }
  
    MotionCapabilities getCapabilities() const override {
        return SpotMotionCapabilities::getCapabilities();
    }
};
```

**3. 数据格式转换**

```cpp
class SpotSensorInterface : public ISensorInterface {
private:
    std::unique_ptr<bosdyn::client::ImageClient> image_client_;
    std::unique_ptr<bosdyn::client::PointCloudClient> pointcloud_client_;
  
public:
    std::shared_ptr<PointCloudData> getLatestPointCloud() const override {
        // 1. 从Spot获取点云数据  
        auto spot_pointcloud = pointcloud_client_->GetPointCloud();
        if (!spot_pointcloud) {
            return nullptr;
        }
      
        // 2. 转换为统一格式
        auto unified_cloud = std::make_shared<PointCloudData>();
        unified_cloud->frame_id = "velodyne";  // Spot使用Velodyne激光雷达
        unified_cloud->timestamp_ns = spot_pointcloud->acquisition_time().nanos();
      
        // 3. 转换点数据
        for (const auto& spot_point : spot_pointcloud->data().points()) {
            PointCloudData::Point3D point;
            point.x = spot_point.x();
            point.y = spot_point.y();  
            point.z = spot_point.z();
            unified_cloud->points.push_back(point);
        }
      
        unified_cloud->is_valid = true;
        return unified_cloud;
    }
};
```

### 7.2 接口功能扩展

#### 新接口类型添加

```cpp
// 示例：添加机械臂控制接口
namespace robot_base_interfaces {
namespace manipulator_interface {

struct JointState {
    std::vector<float> positions;    // 关节位置 (rad)
    std::vector<float> velocities;   // 关节速度 (rad/s)  
    std::vector<float> efforts;      // 关节力矩 (N⋅m)
    uint64_t timestamp_ns = 0;
};

struct EndEffectorPose {
    float x, y, z;                   // 位置 (m)
    float roll, pitch, yaw;          // 姿态 (rad)
};

class IManipulatorInterface {
public:
    virtual ~IManipulatorInterface() = default;
  
    // 关节控制
    virtual bool moveToJointPositions(const std::vector<float>& positions) = 0;
    virtual bool setJointVelocities(const std::vector<float>& velocities) = 0;
  
    // 末端执行器控制
    virtual bool moveToCartesianPose(const EndEffectorPose& pose) = 0;
    virtual bool openGripper() = 0;
    virtual bool closeGripper() = 0;
  
    // 状态查询
    virtual JointState getJointState() const = 0;
    virtual EndEffectorPose getEndEffectorPose() const = 0;
  
    // 安全控制
    virtual bool emergencyStop() = 0;
    virtual bool isOperational() const = 0;
  
    virtual std::string getVersion() const { return "1.0.0"; }
};

} // namespace manipulator_interface
} // namespace robot_base_interfaces

// 更新IRobotAdapter接口
class IRobotAdapter {
public:
    // 原有接口...
    virtual IMotionController* getMotionController() = 0;
    virtual ISensorInterface* getSensorInterface() = 0;
    virtual IStateMonitor* getStateMonitor() = 0;
    virtual IPowerManager* getPowerManager() = 0;
  
    // 新增机械臂接口 (可选)
    virtual IManipulatorInterface* getManipulatorInterface() {
        return nullptr; // 默认不支持机械臂
    }
  
    // 能力查询
    virtual bool hasManipulator() const {
        return getManipulatorInterface() != nullptr;
    }
};
```

#### 上层应用适配

```cpp
class AdvancedNavigationController : public NavigationController {
private:
    IManipulatorInterfacePtr manipulator_;
  
public:
    AdvancedNavigationController(IRobotAdapterPtr robot_adapter) 
        : NavigationController(robot_adapter) {
        // 检查是否支持机械臂
        if (robot_adapter->hasManipulator()) {
            manipulator_ = robot_adapter->getManipulatorInterface();
            RCLCPP_INFO(this->get_logger(), "检测到机械臂，启用高级操作功能");
        }
    }
  
    void performPickAndPlace(const geometry_msgs::msg::Point& object_position,
                           const geometry_msgs::msg::Point& target_position) {
        if (!manipulator_) {
            RCLCPP_WARN(this->get_logger(), "机器人不支持机械臂操作");
            return;
        }
      
        // 1. 导航到物体附近
        geometry_msgs::msg::PoseStamped approach_pose;
        approach_pose.pose.position = object_position;
        navigateToGoal(approach_pose);
      
        // 2. 机械臂抓取
        EndEffectorPose grasp_pose;
        grasp_pose.x = object_position.x;
        grasp_pose.y = object_position.y;
        grasp_pose.z = object_position.z;
      
        manipulator_->moveToCartesianPose(grasp_pose);
        manipulator_->closeGripper();
      
        // 3. 导航到目标位置
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.pose.position = target_position;
        navigateToGoal(target_pose);
      
        // 4. 机械臂放置
        EndEffectorPose place_pose;
        place_pose.x = target_position.x;
        place_pose.y = target_position.y;  
        place_pose.z = target_position.z;
      
        manipulator_->moveToCartesianPose(place_pose);
        manipulator_->openGripper();
    }
};
```

### 7.3 性能优化方向

#### 零拷贝数据传递

```cpp
// 优化大数据量传感器数据传递
template<typename T>
class ZeroCopyDataBuffer {
private:
    std::shared_ptr<T> data_;
    std::atomic<bool> ready_{false};
  
public:
    void updateData(std::shared_ptr<T> new_data) {
        data_ = std::move(new_data);  // 移动语义，避免拷贝
        ready_.store(true);
    }
  
    std::shared_ptr<T> getData() const {
        if (ready_.load()) {
            return data_;  // 返回共享指针，零拷贝
        }
        return nullptr;
    }
};

class OptimizedSensorInterface : public ISensorInterface {
private:
    ZeroCopyDataBuffer<PointCloudData> pointcloud_buffer_;
  
public:
    std::shared_ptr<PointCloudData> getLatestPointCloud() const override {
        return pointcloud_buffer_.getData();  // 零拷贝返回
    }
  
    void onPointCloudReceived(sensor_msgs::msg::PointCloud2::SharedPtr ros_cloud) {
        // 直接在ROS回调中构造统一数据格式
        auto unified_cloud = std::make_shared<PointCloudData>();
        convertRosToUnified(ros_cloud, unified_cloud);  // 一次转换
      
        pointcloud_buffer_.updateData(std::move(unified_cloud));  // 移动语义
    }
};
```

#### 异步执行优化

```cpp
class AsyncMotionController : public IMotionController {
private:
    std::thread command_thread_;
    std::queue<std::function<void()>> command_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
  
public:
    MotionResult setVelocity(const Velocity& velocity) override {
        // 验证参数
        if (!validateVelocity(velocity)) {
            return MotionResult::CAPABILITY_LIMITED;
        }
      
        // 异步执行实际命令
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            command_queue_.push([this, velocity]() {
                executeVelocityCommand(velocity);
            });
        }
        queue_cv_.notify_one();
      
        return MotionResult::SUCCESS;  // 立即返回，不等待执行完成
    }
  
private:
    void commandExecutionLoop() {
        while (running_) {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this] { return !command_queue_.empty() || !running_; });
          
            while (!command_queue_.empty()) {
                auto command = command_queue_.front();
                command_queue_.pop();
                lock.unlock();
              
                // 执行命令
                command();
              
                lock.lock();
            }
        }
    }
  
    void executeVelocityCommand(const Velocity& velocity) {
        // 实际的硬件通信逻辑
        sendToHardware(velocity);
    }
};
```

---

## 8. 测试与质量保证

### 8.1 单元测试框架

#### Mock实现示例

```cpp
class MockMotionController : public IMotionController {
private:
    MotionState mock_state_;
    MotionCapabilities mock_capabilities_;
    std::function<void(const MotionState&)> state_callback_;
  
public:
    MockMotionController() {
        // 设置默认能力
        mock_capabilities_.max_linear_velocity = 1.5f;
        mock_capabilities_.can_dance = true;
        mock_capabilities_.can_jump = true;
    }
  
    MotionResult initialize() override { return MotionResult::SUCCESS; }
    MotionResult shutdown() override { return MotionResult::SUCCESS; }
  
    MotionCapabilities getCapabilities() const override {
        return mock_capabilities_;
    }
  
    MotionResult setVelocity(const Velocity& velocity) override {
        if (!validateVelocity(velocity)) {
            return MotionResult::CAPABILITY_LIMITED;
        }
      
        // 模拟状态更新
        mock_state_.velocity = velocity;
        mock_state_.is_moving = (velocity.linear_x != 0 || velocity.linear_y != 0 || velocity.angular_z != 0);
      
        // 触发回调
        if (state_callback_) {
            state_callback_(mock_state_);
        }
      
        return MotionResult::SUCCESS;
    }
  
    MotionResult performDance(int dance_type) override {
        (void)dance_type;
        return MotionResult::SUCCESS;  // 模拟成功执行
    }
  
    MotionState getMotionState() const override { return mock_state_; }
    bool isOperational() const override { return true; }
    uint32_t getErrorCode() const override { return 0; }
  
    void setStateCallback(std::function<void(const MotionState&)> callback) override {
        state_callback_ = callback;
    }
  
    // 测试辅助方法
    void setMockCapabilities(const MotionCapabilities& capabilities) {
        mock_capabilities_ = capabilities;
    }
  
    void triggerErrorState(uint32_t error_code) {
        // 模拟错误状态，用于测试错误处理
        mock_state_.error_code = error_code;
        if (state_callback_) {
            state_callback_(mock_state_);
        }
    }
};
```

#### 测试用例示例

```cpp
class MotionControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        mock_motion_controller_ = std::make_shared<MockMotionController>();
        navigation_controller_ = std::make_unique<NavigationController>(
            createMockAdapter(mock_motion_controller_));
    }
  
    std::shared_ptr<MockMotionController> mock_motion_controller_;
    std::unique_ptr<NavigationController> navigation_controller_;
  
private:
    IRobotAdapterPtr createMockAdapter(IMotionControllerPtr motion_controller) {
        auto mock_adapter = std::make_shared<MockRobotAdapter>();
        mock_adapter->setMotionController(motion_controller);
        return mock_adapter;
    }
};

TEST_F(MotionControllerTest, SetVelocityWithinLimits) {
    // 测试正常速度设置
    Velocity test_velocity{1.0f, 0.5f, 0.8f};
  
    auto result = mock_motion_controller_->setVelocity(test_velocity);
  
    EXPECT_EQ(result, MotionResult::SUCCESS);
  
    auto motion_state = mock_motion_controller_->getMotionState();
    EXPECT_FLOAT_EQ(motion_state.velocity.linear_x, 1.0f);
    EXPECT_FLOAT_EQ(motion_state.velocity.linear_y, 0.5f);
    EXPECT_FLOAT_EQ(motion_state.velocity.angular_z, 0.8f);
    EXPECT_TRUE(motion_state.is_moving);
}

TEST_F(MotionControllerTest, SetVelocityExceedsLimits) {
    // 测试超限速度设置
    Velocity excessive_velocity{5.0f, 0.0f, 0.0f};  // 超过1.5m/s限制
  
    auto result = mock_motion_controller_->setVelocity(excessive_velocity);
  
    EXPECT_EQ(result, MotionResult::CAPABILITY_LIMITED);
}

TEST_F(MotionControllerTest, DanceCapabilityCheck) {
    // 测试舞蹈能力
    auto capabilities = mock_motion_controller_->getCapabilities();
    EXPECT_TRUE(capabilities.can_dance);
  
    auto result = mock_motion_controller_->performDance(1);
    EXPECT_EQ(result, MotionResult::SUCCESS);
}

TEST_F(MotionControllerTest, StateCallbackTriggering) {
    // 测试状态回调机制
    bool callback_triggered = false;
    MotionState received_state;
  
    mock_motion_controller_->setStateCallback([&](const MotionState& state) {
        callback_triggered = true;
        received_state = state;
    });
  
    Velocity test_velocity{0.5f, 0.0f, 0.2f};
    mock_motion_controller_->setVelocity(test_velocity);
  
    EXPECT_TRUE(callback_triggered);
    EXPECT_FLOAT_EQ(received_state.velocity.linear_x, 0.5f);
    EXPECT_FLOAT_EQ(received_state.velocity.angular_z, 0.2f);
}

TEST_F(MotionControllerTest, ErrorHandling) {
    // 测试错误处理
    bool error_callback_triggered = false;
    uint32_t received_error_code = 0;
  
    mock_motion_controller_->setErrorCallback([&](uint32_t error_code, const std::string& msg) {
        error_callback_triggered = true;
        received_error_code = error_code;
    });
  
    mock_motion_controller_->triggerErrorState(123);
  
    EXPECT_TRUE(error_callback_triggered);
    EXPECT_EQ(received_error_code, 123);
}

// 集成测试
TEST_F(MotionControllerTest, NavigationIntegration) {
    // 测试导航控制器集成
    geometry_msgs::msg::PoseStamped goal;
    goal.pose.position.x = 2.0;
    goal.pose.position.y = 1.0;
  
    bool navigation_completed = false;
  
    // 模拟导航完成
    mock_motion_controller_->setStateCallback([&](const MotionState& state) {
        if (std::abs(state.position.x - 2.0) < 0.1 && 
            std::abs(state.position.y - 1.0) < 0.1) {
            navigation_completed = true;
        }
    });
  
    navigation_controller_->navigateToGoal(goal);
  
    // 验证导航逻辑...
}
```

### 8.2 集成测试

#### 硬件在环测试

```cpp
class HardwareIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 连接真实Go2机器人
        go2_adapter_ = RobotAdapterFactory::createAdapter(RobotType::GO2);
        ASSERT_TRUE(go2_adapter_->initialize());
      
        motion_controller_ = go2_adapter_->getMotionController();
        sensor_interface_ = go2_adapter_->getSensorInterface();
        state_monitor_ = go2_adapter_->getStateMonitor();
        power_manager_ = go2_adapter_->getPowerManager();
    }
  
    void TearDown() override {
        // 确保机器人处于安全状态
        motion_controller_->emergencyStop(EmergencyStopLevel::SOFT_STOP);
        go2_adapter_->shutdown();
    }
  
    IRobotAdapterPtr go2_adapter_;
    IMotionControllerPtr motion_controller_;
    ISensorInterfacePtr sensor_interface_;
    IStateMonitorPtr state_monitor_;
    IPowerManagerPtr power_manager_;
};

TEST_F(HardwareIntegrationTest, BasicMovementTest) {
    // 测试基础移动功能
    ASSERT_TRUE(motion_controller_->isOperational());
  
    // 站起
    auto result = motion_controller_->standUp();
    EXPECT_EQ(result, MotionResult::SUCCESS);
  
    std::this_thread::sleep_for(std::chrono::seconds(3));
  
    // 前进
    result = motion_controller_->setVelocity({0.3f, 0.0f, 0.0f});
    EXPECT_EQ(result, MotionResult::SUCCESS);
  
    std::this_thread::sleep_for(std::chrono::seconds(2));
  
    // 停止
    result = motion_controller_->setVelocity({0.0f, 0.0f, 0.0f});
    EXPECT_EQ(result, MotionResult::SUCCESS);
}

TEST_F(HardwareIntegrationTest, SensorDataTest) {
    // 测试传感器数据获取
    ASSERT_TRUE(sensor_interface_->initialize());
  
    // 启动传感器
    EXPECT_TRUE(sensor_interface_->startSensor(SensorType::LIDAR_3D));
    EXPECT_TRUE(sensor_interface_->startSensor(SensorType::IMU));
  
    std::this_thread::sleep_for(std::chrono::seconds(1));
  
    // 获取点云数据
    auto point_cloud = sensor_interface_->getLatestPointCloud();
    ASSERT_NE(point_cloud, nullptr);
    EXPECT_GT(point_cloud->points.size(), 100);  // 至少有100个点
  
    // 获取IMU数据
    auto imu_data = sensor_interface_->getLatestIMU();
    ASSERT_NE(imu_data, nullptr);
    EXPECT_TRUE(imu_data->is_valid);
}

TEST_F(HardwareIntegrationTest, StateMonitoringTest) {
    // 测试状态监控
    ASSERT_TRUE(state_monitor_->initialize());
    ASSERT_TRUE(state_monitor_->startMonitoring());
  
    std::this_thread::sleep_for(std::chrono::seconds(1));
  
    // 检查基础状态
    EXPECT_TRUE(state_monitor_->isOperational());
  
    auto health_score = state_monitor_->getHealthScore();
    EXPECT_GE(health_score, 0.0f);
    EXPECT_LE(health_score, 1.0f);
  
    // 检查电机状态
    for (int i = 0; i < 20; ++i) {
        auto motor_info = state_monitor_->getMotorInfo(i);
        EXPECT_LT(motor_info.temperature, 80.0f);  // 温度不应过高
    }
  
    // 检查足端状态
    auto foot_info = state_monitor_->getAllFootInfo();
    EXPECT_EQ(foot_info.size(), 4);
}

TEST_F(HardwareIntegrationTest, PowerManagementTest) {
    // 测试电源管理
    ASSERT_TRUE(power_manager_->isOperational());
  
    // 获取电池信息
    auto battery_info = power_manager_->getBatteryInfo();
    EXPECT_GT(battery_info.percentage, 0.0f);
    EXPECT_LE(battery_info.percentage, 100.0f);
    EXPECT_GT(battery_info.voltage, 20.0f);  // Go2电池电压应大于20V
  
    // 测试功耗监控
    auto power_consumption = power_manager_->getCurrentPowerConsumption();
    EXPECT_GT(power_consumption, 0.0f);
    EXPECT_LT(power_consumption, 200.0f);  // 合理的功耗范围
}
```

---

## 9. 部署与运维

### 9.1 构建系统配置

#### CMake配置

```cmake
# robot_base_interfaces/CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
project(robot_base_interfaces)

# 编译器要求
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# 各子模块
add_subdirectory(motion_interface)
add_subdirectory(sensor_interface)
add_subdirectory(state_interface)
add_subdirectory(power_interface)

# 测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  add_subdirectory(test)
endif()

# 安装
install(DIRECTORY common/
  DESTINATION include/${PROJECT_NAME}/common/)

# 导出
ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs)

ament_package()
```

#### 子模块CMake示例

```cmake
# motion_interface/CMakeLists.txt
cmake_minimum_required(VERSION 3.8)

# 创建接口库
add_library(motion_interface INTERFACE)
target_include_directories(motion_interface
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(motion_interface INTERFACE cxx_std_17)

# 实现库 (数据类型实现)
add_library(motion_types src/motion_types.cpp)
target_include_directories(motion_types
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(motion_types PUBLIC cxx_std_17)

# 安装
install(TARGETS motion_interface motion_types
  EXPORT motion_interface_targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include/
)

# 导出目标
ament_export_targets(motion_interface_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp geometry_msgs)

# 测试
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(test_motion_types
    test/test_motion_types.cpp
  )
  target_link_libraries(test_motion_types motion_types)
  
  ament_add_gtest(test_i_motion_controller  
    test/test_i_motion_controller.cpp
  )
  target_link_libraries(test_i_motion_controller motion_interface motion_types)
endif()

ament_package()
```

### 9.2 包管理与依赖

#### package.xml配置

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>robot_base_interfaces</name>
  <version>2.0.0</version>
  <description>Robot base interfaces for unified hardware abstraction</description>
  <maintainer email="navigation@robot.com">Robot Navigation Team</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- 核心ROS2依赖 -->
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>

  <!-- 测试依赖 -->
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  <test_depend>ament_cmake_gtest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

#### 使用配置

```cmake
# 使用robot_base_interfaces的项目CMakeLists.txt
find_package(robot_base_interfaces REQUIRED)

add_executable(my_navigation_node src/navigation_node.cpp)

# 链接所需接口
target_link_libraries(my_navigation_node
  robot_base_interfaces::motion_interface
  robot_base_interfaces::sensor_interface
  robot_base_interfaces::state_interface
  robot_base_interfaces::power_interface
)
```

### 9.3 文档与维护

#### 自动化文档生成

```python
#!/usr/bin/env python3
"""
自动生成接口文档的脚本
"""

import os
import re
from pathlib import Path

def extract_interface_info(header_file):
    """从头文件提取接口信息"""
    with open(header_file, 'r') as f:
        content = f.read()
  
    # 提取类名
    class_match = re.search(r'class\s+(\w+Interface)\s*{', content)
    if not class_match:
        return None
  
    class_name = class_match.group(1)
  
    # 提取虚函数
    virtual_methods = re.findall(r'virtual\s+[\w:]+\s+(\w+)\s*\([^)]*\)[^;]*;', content)
  
    # 提取注释
    comments = re.findall(r'/\*\*\s*(.*?)\s*\*/', content, re.DOTALL)
  
    return {
        'class_name': class_name,
        'methods': virtual_methods,
        'comments': comments,
        'file_path': header_file
    }

def generate_interface_doc(interface_info):
    """生成接口文档"""
    doc = f"""
# {interface_info['class_name']} 接口文档

**文件位置**: `{interface_info['file_path']}`

## 核心方法

"""
  
    for method in interface_info['methods']:
        doc += f"- `{method}()`\n"
  
    doc += f"""
## 总计

该接口定义了 {len(interface_info['methods'])} 个虚方法。

详细API说明请参考头文件中的文档注释。
"""
  
    return doc

def main():
    """主函数"""
    base_dir = Path(__file__).parent.parent
    interfaces_dir = base_dir / "include" / "robot_base_interfaces"
  
    all_interfaces = []
  
    # 遍历所有接口目录
    for interface_dir in interfaces_dir.iterdir():
        if interface_dir.is_dir():
            header_files = list(interface_dir.glob("i_*.hpp"))
            for header_file in header_files:
                interface_info = extract_interface_info(header_file)
                if interface_info:
                    all_interfaces.append(interface_info)
  
    # 生成总体文档
    overview_doc = """# Robot Base Interfaces API 文档

## 接口总览

"""
  
    for interface in all_interfaces:
        overview_doc += f"- [{interface['class_name']}](#{interface['class_name'].lower()}): {len(interface['methods'])} 方法\n"
  
    overview_doc += "\n## 详细接口文档\n\n"
  
    for interface in all_interfaces:
        overview_doc += generate_interface_doc(interface)
        overview_doc += "\n---\n\n"
  
    # 写入文档文件
    doc_file = base_dir / "docs" / "api_reference.md"
    with open(doc_file, 'w') as f:
        f.write(overview_doc)
  
    print(f"API文档已生成: {doc_file}")

if __name__ == "__main__":
    main()
```

---

## 10. 总结与展望

### 10.1 架构优势总结

`robot_base_interfaces` 作为四足机器人自主导航系统的基础抽象层，具备以下核心优势：

**设计优势**:

- **统一抽象**: 提供四个专业化接口，覆盖机器人核心功能域
- **Go2优化**: 完全基于宇树Go2机器人能力设计，确保功能完备性
- **扩展友好**: 通过虚函数默认实现和能力查询机制支持其他机器人
- **类型安全**: 使用强类型枚举和Result`<T>`模板提高可靠性

**工程优势**:

- **模块化**: 四个独立接口模块可按需依赖
- **可测试**: 通过Mock实现支持无硬件单元测试
- **高性能**: 支持零拷贝数据传递和异步处理
- **生产就绪**: 包含完整的构建、测试、文档和部署支持

**业务优势**:

- **降低开发复杂度**: 上层算法无需关注硬件细节
- **提高开发效率**: 支持并行开发和快速原型验证
- **增强系统可靠性**: 统一错误处理和状态监控机制
- **降低维护成本**: 清晰的接口边界和职责分离

### 10.2 技术创新点

1. **完备的四足机器人抽象模型**: 首次系统性地将四足机器人功能抽象为运动、感知、状态、电源四个核心领域
2. **Go2原生适配设计**: 接口设计完全贴合Go2硬件能力，确保零损耗的功能映射
3. **优雅降级机制**: 通过能力查询和默认实现，优雅支持能力不同的机器人平台
4. **强类型安全保证**: 引入Result`<T>`模板和强类型枚举，显著提高接口可靠性
5. **事件驱动架构**: 全面的回调机制支持，实现高效的异步编程模型

### 10.3 应用效果

该架构设计已在实际Go2机器人导航系统中得到验证：

- **开发效率提升**: 导航算法开发时间缩短40%
- **代码复用率**: 核心导航逻辑可直接应用于其他四足机器人
- **系统稳定性**: 统一的错误处理和状态监控显著提高系统鲁棒性
- **测试覆盖度**: Mock实现使单元测试覆盖率达到85%以上

### 10.4 未来发展方向

#### 近期计划 (6个月内)

1. **接口功能增强**:

   - 添加机械臂控制接口支持
   - 增强传感器融合能力
   - 扩展自主充电决策算法
2. **性能优化**:

   - 实现完全零拷贝数据流
   - 优化高频回调的线程调度
   - 增加实时性能监控指标
3. **工具链完善**:

   - 开发接口代码自动生成工具
   - 完善Mock框架和测试工具
   - 增加性能分析和诊断工具

#### 中期目标 (1年内)

1. **多机器人平台支持**:

   - 完成Boston Dynamics Spot适配器
   - 开发ANYmal机器人适配器
   - 建立机器人能力认证体系
2. **云端集成**:

   - 支持远程监控和诊断
   - 实现云端能力配置和更新
   - 开发多机器人协作接口
3. **标准化推进**:

   - 参与ROS2四足机器人标准制定
   - 开源核心接口设计供社区使用
   - 建立机器人抽象接口最佳实践

#### 长期愿景 (3年内)

1. **通用机器人抽象平台**:

   - 扩展支持轮式、履带式机器人
   - 建立通用机器人能力描述语言
   - 实现跨平台机器人应用生态
2. **AI驱动的适配优化**:

   - 机器学习驱动的性能调优
   - 智能化的硬件能力发现
   - 自适应的接口参数优化
3. **产业标准建立**:

   - 推动行业标准化接口规范
   - 建立机器人互操作性认证体系
   - 构建开放的机器人软件生态

---

## 附录

### 附录A: 完整API参考

详细的API参考文档请参考各接口头文件中的doxygen注释：

- [IMotionController](../motion_interface/include/robot_base_interfaces/motion_interface/i_motion_controller.hpp)
- [ISensorInterface](../sensor_interface/include/robot_base_interfaces/sensor_interface/i_sensor_interface.hpp)
- [IStateMonitor](../state_interface/include/robot_base_interfaces/state_interface/i_state_monitor.hpp)
- [IPowerManager](../power_interface/include/robot_base_interfaces/power_interface/i_power_manager.hpp)

### 附录B: Go2机器人技术规格

| 参数类别           | 规格参数                               |
| ------------------ | -------------------------------------- |
| **基本参数** | 站立尺寸: 70×31×40cm, 重量: 15kg     |
| **运动能力** | 最大速度: 1.5m/s, 最大角速度: 2.0rad/s |
| **传感器**   | Livox Mid360 3D激光雷达, 内置IMU       |
| **电源系统** | 15000mAh锂电池, 无线充电支持           |
| **通信接口** | ROS2 Humble, Cyclone DDS 0.10.2        |

### 附录C: 常见问题解答

**Q: 如何为新机器人添加适配器？**
A: 参考第7.1节"新机器人平台适配"，按照三步流程：能力分析、通信协议适配、数据格式转换。

**Q: 接口回调函数的线程安全如何保证？**
A: 回调函数可能在内部线程执行，调用方需保证回调函数线程安全。建议只在回调中进行数据拷贝和事件通知。

**Q: 如何处理机器人不支持的功能？**
A: 使用能力查询机制检查功能支持，对不支持的功能实现优雅降级或替代方案。

**Q: 性能优化有哪些建议？**
A: 使用零拷贝数据传递、异步执行、合理的缓冲区设计。避免在回调函数中执行耗时操作。

### 附录D: 参考资料

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [宇树Go2开发文档](https://support.unitree.com/home/zh/developer)
- [Design Patterns: Elements of Reusable Object-Oriented Software](https://en.wikipedia.org/wiki/Design_Patterns)
- [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines)

---

**文档版权**: Apache 2.0 License
**联系方式**: thu.yangnan@outlook.com
**最后更新**: 2025年9月
