# robot_factory 文件夹详细分析

## 概述

`robot_factory`文件夹是ROS2机器人系统中实现**工厂模式**的核心模块，负责机器人的自动检测、适配器创建和能力管理。该模块采用现代C++设计模式，为多机器人平台提供统一的创建和管理接口。

## 目录结构

```
src/robot_interfaces/robot_factory/
├── robot_detector/                 # 机器人检测模块
│   ├── include/robot_factory/robot_detector/
│   │   ├── robot_detector.hpp      # 机器人检测器主类
│   │   └── robot_types.hpp         # 机器人类型定义
│   ├── src/
│   │   ├── robot_detector.cpp      # 检测器实现
│   │   └── robot_types.cpp         # 类型定义实现
│   ├── test/
│   │   ├── test_robot_detector.cpp # 检测器单元测试
│   │   └── test_robot_types.cpp    # 类型定义测试
│   ├── CMakeLists.txt              # 构建配置
│   └── package.xml                 # ROS2包描述
│
├── adapter_factory/                # 适配器工厂模块
│   ├── include/robot_factory/adapter_factory/
│   │   ├── robot_adapter_factory.hpp   # 适配器工厂主类
│   │   ├── i_robot_adapter.hpp         # 机器人适配器基类接口
│   │   └── adapter_registry.hpp        # 适配器注册管理
│   ├── src/
│   │   ├── robot_adapter_factory.cpp   # 工厂实现
│   │   ├── i_robot_adapter.cpp         # 基类实现
│   │   └── adapter_registry.cpp        # 注册管理实现
│   ├── test/
│   │   ├── test_adapter_factory.cpp    # 工厂单元测试
│   │   └── test_adapter_registry.cpp   # 注册管理测试
│   ├── CMakeLists.txt
│   └── package.xml
│
└── capability_manager/             # 能力管理模块
    ├── include/robot_factory/capability_manager/
    │   ├── capability_manager.hpp      # 能力管理器主类
    │   ├── capability_definitions.hpp  # 能力定义结构
    │   └── capability_loader.hpp       # 能力配置加载器
    ├── src/
    │   ├── capability_manager.cpp      # 管理器实现
    │   ├── capability_definitions.cpp  # 能力定义实现
    │   └── capability_loader.cpp       # 配置加载实现
    ├── config/
    │   ├── capability_matrix.yaml      # 机器人能力矩阵配置
    │   └── default_capabilities.yaml   # 默认能力配置
    ├── test/
    │   ├── test_capability_manager.cpp # 能力管理测试
    │   └── test_capability_loader.cpp  # 加载器测试
    ├── CMakeLists.txt
    └── package.xml
```

---

## 模块一：robot_detector（机器人检测模块）

### 1.1 robot_detector.hpp

**作用**: 机器人自动检测器的核心类，负责自动识别连接的机器人类型

**主要功能**:
- 通过多种方式检测机器人类型（环境变量、网络发现、配置文件、ROS2话题）
- 提供Go2机器人的专用检测方法
- 支持多机器人系统的检测
- 提供检测结果缓存和验证机制

**核心接口设计**:
```cpp
class RobotDetector {
public:
    // 主要检测接口
    RobotDetectionResult detectRobot();                 // 自动检测机器人类型
    RobotDetectionResult detectSpecificRobot(RobotType target_type);
    std::vector<RobotDetectionResult> detectAllRobots(); // 多机器人检测
    bool isRobotOnline(RobotType robot_type);
    
    // 配置管理
    void setNetworkDetectionConfig(const NetworkDetectionConfig& config);
    void enableDetectionMethod(const std::string& method_name, bool enabled);
    
    // 具体检测方法
    RobotDetectionResult detectByEnvironment();         // 环境变量检测
    RobotDetectionResult detectByNetwork();             // 网络发现检测
    RobotDetectionResult detectByConfigFile();          // 配置文件检测
    RobotDetectionResult detectByROS2Topics();          // ROS2话题检测
    
    // Go2专用方法
    RobotDetectionResult detectGo2Robot();
    bool checkGo2NetworkConnection(const std::string& ip_address);
    std::vector<std::string> checkGo2ROS2Topics();
    std::map<std::string, std::string> getGo2RobotInfo(const std::string& ip_address);
};
```

**数据结构设计**:
```cpp
struct RobotDetectionResult {
    bool is_detected = false;
    RobotType robot_type = RobotType::GENERIC;
    std::string robot_name;
    std::string robot_model;
    std::string serial_number;
    std::string firmware_version;
    std::string network_address;
    float detection_confidence = 0.0f;
    std::string detection_method;
    std::vector<std::string> evidence;
};

struct NetworkDetectionConfig {
    struct {
        std::vector<std::string> ip_addresses = {"192.168.123.18"};
        std::vector<int> ports = {8080, 8081};
        std::vector<std::string> ros2_topics = {
            "/sportmodestate", "/lowstate", "/utlidar/cloud",
            "/api/sport/request", "/wirelesscontroller"
        };
        std::string dds_domain = "cyclonedds";
        std::string network_interface = "eth0";
    } go2;
    
    int network_timeout_ms = 3000;
    int max_ping_attempts = 3;
    int topic_discovery_timeout_ms = 5000;
};
```

**如何实现**:
1. 实现多级检测策略：环境变量 → 网络发现 → 配置文件 → 硬件特征
2. Go2特定检测：ping 192.168.123.18，检查unitree特有ROS2话题
3. 添加检测结果缓存机制，避免重复检测
4. 提供置信度评分系统，综合多种证据判断结果可靠性

### 1.2 robot_types.hpp

**作用**: 定义支持的机器人类型枚举和相关常量

**内容设计**:
```cpp
namespace robot_factory::robot_detector {

enum class RobotType {
    UNKNOWN = 0,    // 未知机器人
    GENERIC = 1,    // 通用机器人
    GO2 = 2,        // Unitree Go2
    SPOT = 3,       // Boston Dynamics Spot (预留)
    ANYMAL = 4,     // ANYbotics ANYmal (预留)
    CUSTOM = 99     // 自定义机器人类型
};

// 机器人类型常量
const std::map<RobotType, std::string> ROBOT_TYPE_NAMES = {
    {RobotType::UNKNOWN, "Unknown"},
    {RobotType::GENERIC, "Generic"},
    {RobotType::GO2, "Unitree Go2"},
    {RobotType::SPOT, "Boston Dynamics Spot"},
    {RobotType::ANYMAL, "ANYbotics ANYmal"}
};

// Go2特定常量
namespace go2_constants {
    const std::string DEFAULT_IP = "192.168.123.18";
    const int DEFAULT_PORT = 8080;
    const std::vector<std::string> SIGNATURE_TOPICS = {
        "/sportmodestate", "/lowstate", "/api/sport/request"
    };
    const std::string DEVICE_TYPE = "quadruped";
    const std::string MANUFACTURER = "Unitree";
}

} // namespace
```

### 1.3 robot_detector.cpp

**作用**: 检测器的具体实现

**实现重点**:
1. **检测策略实现**：
   ```cpp
   RobotDetectionResult RobotDetector::detectRobot() {
       // 1. 检查缓存
       if (isCacheValid()) return getCachedResult();
       
       // 2. 环境变量检测
       auto env_result = detectByEnvironment();
       if (env_result.is_detected && env_result.detection_confidence > 0.9) {
           return env_result;
       }
       
       // 3. 网络检测
       auto network_result = detectByNetwork();
       if (network_result.is_detected) {
           return network_result;
       }
       
       // 4. 配置文件检测
       auto config_result = detectByConfigFile();
       return config_result.is_detected ? config_result : RobotDetectionResult();
   }
   ```

2. **Go2网络检测实现**：
   ```cpp
   bool RobotDetector::checkGo2NetworkConnection(const std::string& ip_address) {
       // Ping测试
       if (!pingAddress(ip_address, 3000)) return false;
       
       // 端口连通性测试
       if (!checkPortOpen(ip_address, 8080, 2000)) return false;
       
       // ROS2话题检测
       auto topics = checkGo2ROS2Topics();
       return topics.size() >= 3; // 至少检测到3个特征话题
   }
   ```

---

## 模块二：adapter_factory（适配器工厂模块）

### 2.1 robot_adapter_factory.hpp

**作用**: 适配器工厂类，根据机器人类型创建对应的适配器实例

**核心设计理念**:
- 工厂模式：根据机器人类型动态创建适配器
- 注册机制：支持运行时注册新的适配器类型
- 配置驱动：通过配置参数控制适配器创建行为
- 统计监控：提供创建统计和诊断信息

**主要接口**:
```cpp
class RobotAdapterFactory {
public:
    // 适配器创建函数类型
    using AdapterCreator = std::function<IRobotAdapterPtr(const AdapterCreationConfig&)>;
    
    // 主要创建接口
    AdapterCreationResult createAdapter(const AdapterCreationConfig& config = {});
    AdapterCreationResult createAdapter(RobotType robot_type, const AdapterCreationConfig& config = {});
    AdapterCreationResult createAdapterFromDetection(const RobotDetectionResult& detection_result,
                                                    const AdapterCreationConfig& config = {});
    
    // 多机器人支持
    std::vector<AdapterCreationResult> createMultipleAdapters(
        const std::vector<std::pair<RobotType, AdapterCreationConfig>>& configs);
    
    // 扩展和注册
    bool registerAdapterCreator(RobotType robot_type, AdapterCreator creator);
    bool unregisterAdapterCreator(RobotType robot_type);
    bool isRobotTypeSupported(RobotType robot_type) const;
    
    // Go2专用接口
    AdapterCreationResult createGo2Adapter(const AdapterCreationConfig& config = {});
    AdapterCreationResult createGo2AdapterWithDefaults(const std::string& robot_ip = "192.168.123.18",
                                                       const std::string& network_interface = "eth0");
    
    // 静态便利方法
    static IRobotAdapterPtr quickCreateGo2Adapter(const std::string& robot_ip = "192.168.123.18",
                                                  const std::string& network_interface = "eth0");
    static IRobotAdapterPtr autoCreateAdapter();
    static RobotAdapterFactory& getInstance(); // 单例模式
};
```

**配置结构设计**:
```cpp
struct AdapterCreationConfig {
    // 网络配置
    std::string robot_ip = "192.168.123.18";
    int robot_port = 8080;
    std::string network_interface = "eth0";
    int connection_timeout_ms = 5000;
    
    // DDS配置 (Go2特有)
    std::string rmw_implementation = "rmw_cyclonedds_cpp";
    std::string cyclonedds_uri;
    
    // 配置文件路径
    std::string config_file_path;
    
    // 调试选项
    bool enable_debug_logging = false;
    bool auto_reconnect = true;
    int max_reconnect_attempts = 3;
    
    // 扩展配置
    std::map<std::string, std::string> custom_parameters;
};

struct AdapterCreationResult {
    bool success = false;
    IRobotAdapterPtr adapter;
    std::string error_message;
    int error_code = 0;
    RobotType detected_robot_type = RobotType::GENERIC;
    std::string adapter_version;
    std::string creation_time;
};
```

### 2.2 i_robot_adapter.hpp

**作用**: 机器人适配器的基类接口，定义所有适配器必须实现的标准接口

**接口设计**:
```cpp
class IRobotAdapter {
public:
    virtual ~IRobotAdapter() = default;
    
    // 生命周期管理
    virtual bool initialize() = 0;
    virtual bool shutdown() = 0;
    virtual bool isInitialized() const = 0;
    virtual bool isConnected() const = 0;
    
    // 接口获取
    virtual IMotionController* getMotionController() = 0;
    virtual ISensorInterface* getSensorInterface() = 0;
    virtual IStateMonitor* getStateMonitor() = 0;
    virtual IPowerManager* getPowerManager() = 0;
    
    // 元信息
    virtual RobotType getRobotType() const = 0;
    virtual std::string getRobotName() const = 0;
    virtual std::string getAdapterVersion() const = 0;
    virtual RobotCapabilities getRobotCapabilities() const = 0;
    
    // 连接管理
    virtual bool connect() = 0;
    virtual bool disconnect() = 0;
    virtual bool reconnect() = 0;
    virtual ConnectionStatus getConnectionStatus() const = 0;
    
    // 健康检查
    virtual bool healthCheck() = 0;
    virtual std::vector<std::string> getDiagnosticInfo() const = 0;
    
    // 配置管理
    virtual bool loadConfig(const std::string& config_path) = 0;
    virtual bool saveConfig(const std::string& config_path) const = 0;
    
protected:
    // 通用状态
    bool initialized_ = false;
    bool connected_ = false;
    RobotType robot_type_ = RobotType::GENERIC;
    std::string robot_name_;
    std::string adapter_version_ = "1.0.0";
};
```

### 2.3 adapter_registry.hpp

**作用**: 适配器注册管理器，负责管理和维护适配器创建器的注册

**功能设计**:
```cpp
class AdapterRegistry {
public:
    // 单例模式
    static AdapterRegistry& getInstance();
    
    // 注册管理
    bool registerCreator(RobotType robot_type, 
                        const std::string& creator_name,
                        AdapterCreator creator,
                        const std::string& version = "1.0.0");
    bool unregisterCreator(RobotType robot_type);
    
    // 查询功能
    bool hasCreator(RobotType robot_type) const;
    AdapterCreator getCreator(RobotType robot_type) const;
    std::vector<RobotType> getRegisteredTypes() const;
    std::map<RobotType, std::string> getCreatorVersions() const;
    
    // 批量操作
    void registerDefaultCreators();
    void clearAllCreators();
    
    // 诊断功能
    std::string getRegistryStatus() const;
    std::vector<std::string> validateAllCreators() const;

private:
    struct CreatorInfo {
        AdapterCreator creator;
        std::string name;
        std::string version;
        std::chrono::system_clock::time_point registration_time;
    };
    
    std::map<RobotType, CreatorInfo> creators_;
    mutable std::mutex registry_mutex_;
};
```

---

## 模块三：capability_manager（能力管理模块）

### 3.1 capability_manager.hpp

**作用**: 机器人能力管理器，负责加载、管理和匹配机器人能力信息

**核心功能**:
- 从配置文件加载机器人能力信息
- 提供能力查询和比较服务
- 根据能力信息调整系统参数
- 支持能力热更新和动态配置

**接口设计**:
```cpp
class CapabilityManager {
public:
    CapabilityManager();
    
    // 配置加载
    bool loadCapabilities(const std::string& config_path = "");
    bool reloadCapabilities();
    bool saveCapabilities(const std::string& config_path) const;
    
    // 能力查询
    RobotCapabilities getRobotCapabilities(RobotType robot_type) const;
    bool hasCapability(RobotType robot_type, const std::string& capability_name) const;
    std::vector<std::string> getAvailableCapabilities(RobotType robot_type) const;
    
    // 能力比较和匹配
    CapabilityComparisonResult compareCapabilities(RobotType type1, RobotType type2) const;
    std::vector<RobotType> findRobotsWithCapability(const std::string& capability_name) const;
    double calculateCapabilityScore(RobotType robot_type, const CapabilityRequirements& requirements) const;
    
    // 推荐系统
    RecommendationResult recommendRobotForTask(const TaskRequirements& task_requirements) const;
    ConfigurationRecommendation recommendConfiguration(RobotType robot_type, 
                                                      const TaskRequirements& task_requirements) const;
    
    // 能力验证
    ValidationResult validateCapabilities(RobotType robot_type) const;
    std::vector<std::string> checkCapabilityConsistency() const;
    
    // 动态更新
    bool updateCapability(RobotType robot_type, const std::string& capability_name, 
                         const CapabilityValue& value);
    bool addCustomRobotType(RobotType robot_type, const RobotCapabilities& capabilities);
};
```

### 3.2 capability_definitions.hpp

**作用**: 定义机器人能力的完整数据结构

**能力结构设计**:
```cpp
namespace robot_factory::capability_manager {

// 运动能力
struct MotionCapabilities {
    double max_linear_velocity;      // 最大线速度 (m/s)
    double max_angular_velocity;     // 最大角速度 (rad/s)
    double max_acceleration;         // 最大加速度 (m/s²)
    double turning_radius;           // 最小转弯半径 (m)
    bool can_climb_stairs;           // 是否可爬楼梯
    bool can_balance_on_two_legs;    // 是否可双腿平衡
    std::vector<std::string> locomotion_modes; // 运动模式列表
    
    // 四足机器人特有能力
    struct {
        bool can_trot;               // 是否支持小跑模式
        bool can_bound;              // 是否支持跳跃模式
        bool can_crawl;              // 是否支持爬行模式
        double max_step_height;      // 最大跨越高度 (m)
    } quadruped_specific;
};

// 传感器能力
struct SensorCapabilities {
    // LiDAR能力
    struct {
        bool has_2d_lidar;
        bool has_3d_lidar;
        double max_range;            // 最大测距 (m)
        double resolution;           // 角分辨率 (度)
        double accuracy;             // 测距精度 (m)
        int scan_frequency;          // 扫描频率 (Hz)
    } lidar;
    
    // 摄像头能力
    struct {
        int rgb_camera_count;
        int depth_camera_count;
        std::string max_resolution;  // 最大分辨率
        int max_framerate;           // 最大帧率 (fps)
        bool has_stereo_vision;      // 是否有立体视觉
    } camera;
    
    // IMU能力
    struct {
        int dof;                     // 自由度 (6DOF/9DOF)
        int sampling_rate;           // 采样率 (Hz)
        double gyro_range;           // 陀螺仪量程 (度/s)
        double accel_range;          // 加速度计量程 (g)
        bool has_magnetometer;       // 是否有磁力计
    } imu;
    
    // 其他传感器
    bool has_gps;
    bool has_encoders;
    bool has_force_sensors;
    bool has_temperature_sensors;
};

// 电源能力
struct PowerCapabilities {
    double battery_capacity;         // 电池容量 (mAh)
    double operating_voltage;        // 工作电压 (V)
    std::string charging_type;       // 充电类型 (wireless/contact/inductive)
    int charging_time_minutes;       // 充电时间 (分钟)
    double estimated_runtime_hours;  // 预计运行时间 (小时)
    bool supports_hot_swap;          // 是否支持热插拔
    
    // 电源管理特性
    bool has_power_monitoring;
    bool has_low_power_mode;
    bool has_hibernation_mode;
    bool can_estimate_remaining_time;
};

// 通信能力
struct CommunicationCapabilities {
    // 网络通信
    std::vector<std::string> supported_networks; // wifi, ethernet, 4g, 5g
    std::vector<std::string> supported_protocols; // tcp, udp, websocket
    bool has_bluetooth;
    bool has_remote_control;
    
    // ROS2通信
    std::string default_rmw_implementation;
    std::vector<std::string> supported_rmw;
    bool supports_multi_domain;
    
    // DDS配置
    std::string preferred_dds;
    int max_participants;
    bool supports_security;
};

// 计算能力
struct ProcessingCapabilities {
    // 硬件规格
    std::string cpu_architecture;   // ARM64, x86_64
    int cpu_cores;
    int ram_gb;
    int storage_gb;
    bool has_gpu;                   // 是否有GPU加速
    
    // AI/ML支持
    bool supports_tensorflow;
    bool supports_pytorch;
    bool supports_tensorrt;
    bool supports_openvino;
    
    // 实时性能
    double max_control_frequency;   // 最大控制频率 (Hz)
    double typical_latency_ms;      // 典型延迟 (ms)
};

// 物理属性
struct PhysicalProperties {
    // 尺寸 (米)
    double length;
    double width;
    double height;
    double weight;                  // 重量 (kg)
    
    // 环境适应性
    struct {
        double min_temperature;     // 最低工作温度 (°C)
        double max_temperature;     // 最高工作温度 (°C)
        int ip_rating;             // IP防护等级
        bool waterproof;
        bool dustproof;
    } environmental;
    
    // 负载能力
    double max_payload;            // 最大负载 (kg)
    std::vector<std::string> mounting_points; // 安装点
};

// 综合能力结构
struct RobotCapabilities {
    RobotType robot_type;
    std::string robot_model;
    std::string manufacturer;
    std::string firmware_version;
    
    MotionCapabilities motion;
    SensorCapabilities sensors;
    PowerCapabilities power;
    CommunicationCapabilities communication;
    ProcessingCapabilities processing;
    PhysicalProperties physical;
    
    // 元数据
    std::string last_updated;
    std::string config_version;
    std::map<std::string, std::string> custom_properties;
};

} // namespace
```

### 3.3 capability_loader.hpp

**作用**: 能力配置文件的加载和解析器

**功能实现**:
```cpp
class CapabilityLoader {
public:
    // 配置文件格式枚举
    enum class ConfigFormat {
        YAML,
        JSON,
        XML
    };
    
    // 加载接口
    std::map<RobotType, RobotCapabilities> loadCapabilities(const std::string& file_path, 
                                                            ConfigFormat format = ConfigFormat::YAML);
    bool loadCapabilitiesFromString(const std::string& config_content, 
                                   ConfigFormat format,
                                   std::map<RobotType, RobotCapabilities>& capabilities);
    
    // 保存接口
    bool saveCapabilities(const std::map<RobotType, RobotCapabilities>& capabilities,
                         const std::string& file_path,
                         ConfigFormat format = ConfigFormat::YAML);
    
    // 验证接口
    ValidationResult validateConfigFile(const std::string& file_path);
    ValidationResult validateCapabilities(const RobotCapabilities& capabilities);
    
    // 配置合并
    std::map<RobotType, RobotCapabilities> mergeConfigurations(
        const std::vector<std::string>& config_files);
    
    // 默认配置生成
    RobotCapabilities generateDefaultCapabilities(RobotType robot_type);
    bool generateDefaultConfigFile(const std::string& output_path, 
                                  const std::vector<RobotType>& robot_types);
};
```

### 3.4 capability_matrix.yaml

**作用**: 机器人能力矩阵的配置文件

**配置文件结构**:
```yaml
# 机器人能力矩阵配置文件
config_version: "1.0.0"
last_updated: "2024-01-01T00:00:00Z"

robots:
  go2:
    robot_type: 2
    robot_model: "Go2"
    manufacturer: "Unitree"
    firmware_version: "1.0.0"
    
    motion_capabilities:
      max_linear_velocity: 1.5      # m/s
      max_angular_velocity: 2.0     # rad/s
      max_acceleration: 2.0         # m/s²
      turning_radius: 0.5          # m
      can_climb_stairs: true
      can_balance_on_two_legs: false
      locomotion_modes: ["walk", "trot", "bound"]
      
      quadruped_specific:
        can_trot: true
        can_bound: true
        can_crawl: true
        max_step_height: 0.3       # m
    
    sensor_capabilities:
      lidar:
        has_2d_lidar: false
        has_3d_lidar: true
        max_range: 40.0            # m
        resolution: 0.2            # degrees
        accuracy: 0.02             # m
        scan_frequency: 10         # Hz
      
      camera:
        rgb_camera_count: 5
        depth_camera_count: 1
        max_resolution: "1920x1080"
        max_framerate: 30          # fps
        has_stereo_vision: true
      
      imu:
        dof: 6
        sampling_rate: 1000        # Hz
        gyro_range: 2000           # degrees/s
        accel_range: 16            # g
        has_magnetometer: false
      
      has_gps: false
      has_encoders: true
      has_force_sensors: true
      has_temperature_sensors: true
    
    power_capabilities:
      battery_capacity: 15000      # mAh
      operating_voltage: 25.2      # V
      charging_type: "wireless"
      charging_time_minutes: 120
      estimated_runtime_hours: 4.0
      supports_hot_swap: false
      
      has_power_monitoring: true
      has_low_power_mode: true
      has_hibernation_mode: true
      can_estimate_remaining_time: true
    
    communication_capabilities:
      supported_networks: ["wifi", "ethernet"]
      supported_protocols: ["tcp", "udp"]
      has_bluetooth: true
      has_remote_control: true
      
      default_rmw_implementation: "rmw_cyclonedds_cpp"
      supported_rmw: ["rmw_cyclonedds_cpp", "rmw_fastrtps_cpp"]
      supports_multi_domain: true
      
      preferred_dds: "cyclonedds"
      max_participants: 100
      supports_security: false
    
    processing_capabilities:
      cpu_architecture: "ARM64"
      cpu_cores: 4
      ram_gb: 8
      storage_gb: 64
      has_gpu: false
      
      supports_tensorflow: true
      supports_pytorch: false
      supports_tensorrt: false
      supports_openvino: false
      
      max_control_frequency: 1000.0  # Hz
      typical_latency_ms: 5.0
    
    physical_properties:
      length: 0.845                # m
      width: 0.405                 # m
      height: 0.32                 # m
      weight: 15.0                 # kg
      
      environmental:
        min_temperature: -10        # °C
        max_temperature: 45         # °C
        ip_rating: 54
        waterproof: false
        dustproof: true
      
      max_payload: 3.0             # kg
      mounting_points: ["back_mount", "head_mount"]

  # 预留其他机器人类型的配置
  spot:
    robot_type: 3
    robot_model: "Spot"
    manufacturer: "Boston Dynamics"
    # ... (具体配置)
    
  anymal:
    robot_type: 4
    robot_model: "ANYmal C"
    manufacturer: "ANYbotics"
    # ... (具体配置)
```

---

## 集成和测试

### 测试文件结构

每个模块都应包含完整的单元测试：

```cpp
// test_robot_detector.cpp 示例
#include <gtest/gtest.h>
#include "robot_factory/robot_detector/robot_detector.hpp"

class RobotDetectorTest : public ::testing::Test {
protected:
    void SetUp() override {
        detector_ = std::make_unique<RobotDetector>();
    }
    
    std::unique_ptr<RobotDetector> detector_;
};

TEST_F(RobotDetectorTest, DetectGo2Robot) {
    // 模拟Go2环境
    setenv("ROBOT_TYPE", "go2", 1);
    
    auto result = detector_->detectRobot();
    EXPECT_TRUE(result.is_detected);
    EXPECT_EQ(result.robot_type, RobotType::GO2);
    EXPECT_GT(result.detection_confidence, 0.5);
}

TEST_F(RobotDetectorTest, NetworkDetection) {
    // 测试网络检测功能
    bool go2_online = detector_->checkGo2NetworkConnection("192.168.123.18");
    // 注意：此测试需要实际的Go2机器人或模拟环境
}

TEST_F(RobotDetectorTest, MultipleRobotsDetection) {
    auto results = detector_->detectAllRobots();
    // 验证多机器人检测结果
}
```

---

## 实现建议

### 开发顺序
1. **第一阶段**：实现`robot_types.hpp`和基础数据结构
2. **第二阶段**：实现`robot_detector.hpp`的核心检测功能
3. **第三阶段**：实现`capability_manager.hpp`和配置加载
4. **第四阶段**：实现`adapter_factory.hpp`的工厂模式
5. **第五阶段**：集成测试和错误处理完善

### 关键技术点
1. **异步检测**: 使用std::async实现非阻塞检测
2. **缓存机制**: 避免重复检测，提高性能
3. **配置热更新**: 支持运行时重新加载配置
4. **错误恢复**: 完善的错误处理和恢复机制
5. **日志系统**: 详细的调试和运行日志

### 依赖关系
- **系统依赖**: ROS2, yaml-cpp, gtest
- **内部依赖**: robot_base_interfaces
- **网络库**: 用于ping和端口检测的网络工具

---

## 总结

`robot_factory`模块是整个机器人系统的"大脑"，负责：

1. **智能检测**: 自动识别连接的机器人类型和状态
2. **动态创建**: 根据检测结果创建合适的适配器实例  
3. **能力管理**: 管理和查询机器人的各种能力信息
4. **扩展支持**: 为未来新机器人类型预留扩展接口

该模块的设计充分体现了**工厂模式**、**策略模式**和**适配器模式**的优势，为多机器人平台的统一管理提供了强有力的技术基础。通过精心设计的接口和配置驱动的架构，新机器人类型的接入将变得简单高效。