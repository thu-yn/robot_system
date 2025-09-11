# Go2机器人接口信息汇总

## 进行第一步基础抽象接口设计时，需要了解的Unitree_ros2具体信息

基于您要实现基础抽象接口层的目标，以下是从Unitree_ros2中需要重点了解的核心信息：

---

## 1. 核心通信架构

### 1.1 DDS通信机制

- **底层协议**：Go2基于CycloneDDS 0.10.2版本实现通信
- **ROS2兼容性**：支持直接使用ROS2消息进行通信，无需SDK封装层
- **网络配置**：机器人IP为192.168.123.18，开发机设置为192.168.123.99

### 1.2 消息包结构

```
cyclonedds_ws/unitree/
├── unitree_api/          # API请求响应消息
├── unitree_go/           # Go2机器人专用消息
└── unitree_hg/           # G1/H1机器人消息（Go2不需要）
```

---

## 2. 运动控制接口（最关键）

### 2.1 控制消息格式

- **话题名称**：`/api/sport/request`
- **消息类型**：`unitree_api::msg::Request`
- **控制机制**：请求-响应模式（request/response）

### 2.2 SportModeState状态消息

- **话题名称**：`/sportmodestate`
- **消息类型**：`unitree_go::msg::SportModeState`
- **核心字段**：
  ```
  TimeSpec stamp              # 时间戳
  uint32 error_code           # 错误代码
  IMUState imu_state          # IMU状态
  uint8 mode                  # 运动模式
  float32 progress            # 动作执行进度
  uint8 gait_type            # 步态类型
  float32 foot_raise_height   # 足端抬起高度
  float32[3] position         # 位置信息
  float32 body_height         # 机身高度
  float32[3] velocity         # 速度信息
  float32 yaw_speed          # 偏航角速度
  float32[4] range_obstacle   # 障碍物距离
  int16[4] foot_force        # 足端力
  float32[12] foot_position_body  # 足端位置
  float32[12] foot_speed_body     # 足端速度
  ```

### 2.3 运动模式定义

```
0. idle（待机）
1. balanceStand（平衡站立）
2. pose（姿态）
3. locomotion（移动）
4. reserve（保留）
5. lieDown（趴下）
6. jointLock（关节锁定）
7. damping（阻尼）
8. recoveryStand（恢复站立）
9. reserve（保留）
10. sit（坐下）
11. frontFlip（前翻）
12. frontJump（前跳）
13. frontPounce（前扑）
```

### 2.4 步态类型

```
0. idle（待机）
1. trot（小跑）
2. run（奔跑）  
3. climb stair（爬楼梯）
4. forwardDownStair（下楼梯）
9. adjust（调整）
```

---

## 3. 传感器数据接口

### 3.1 激光雷达数据

- **话题名称**：`/utlidar/cloud`
- **消息类型**：`sensor_msgs::PointCloud2`
- **坐标系**：`utlidar_lidar`
- **设备类型**：Livox Mid360

### 3.2 IMU数据

- **包含在SportModeState中**：`imu_state`字段
- **数据内容**：姿态角（RPY）、角速度、线性加速度

### 3.3 低级状态数据

- **话题名称**：`/lowstate` 或 `/lf/lowstate`
- **消息类型**：`unitree_go::msg::LowState`
- **内容**：电机状态、电源信息、详细传感器数据

---

## 4. 电源管理接口

### 4.1 电池信息

- **数据来源**：LowState消息中的电源部分
- **关键参数**：
  - 电池电压
  - 电池容量百分比
  - 充电状态
  - 功耗信息

### 4.2 充电控制

- **充电类型**：Go2支持无线充电
- **控制方式**：通过API请求实现充电行为控制

---

## 5. 网络通信配置

### 5.1 网络设置要求

```bash
# 开发机网络配置
IP: 192.168.123.99
Mask: 255.255.255.0
Interface: 根据实际情况（如eth0, enp3s0等）

# Go2机器人IP
Robot IP: 192.168.123.18
```

### 5.2 DDS配置

```bash
# 必须使用CycloneDDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CYCLONEDDS><Domain><General><Interfaces>
    <NetworkInterface name="eth0" priority="default" multicast="default" />
</Interfaces></General></Domain></CYCLONEDDS>'
```

---

## 6. 消息转换需求分析

### 6.1 ROS2标准消息 → Go2原生消息

```
geometry_msgs/Twist → unitree_api::msg::Request
nav_msgs/Odometry ← unitree_go::msg::SportModeState  
sensor_msgs/LaserScan ← sensor_msgs::PointCloud2（需要转换）
sensor_msgs/Imu ← SportModeState.imu_state
sensor_msgs/BatteryState ← unitree_go::msg::LowState
```

### 6.2 坐标系转换需求

- **机器人坐标系**：Go2内部坐标系
- **ROS2标准坐标系**：ENU（东北天）
- **激光雷达坐标系**：`utlidar_lidar`
- **需要建立完整的TF变换树**

---

## 7. 开发环境要求

### 7.1 基础依赖

```bash
# ROS2版本支持
- ROS2 Foxy（官方推荐）
- ROS2 Humble（社区支持）

# 必要组件
- CycloneDDS 0.10.2+
- unitree_ros2官方库
- 网络连接工具
```

### 7.2 编译配置

```bash
# 编译前环境设置
source /opt/ros/foxy/setup.bash
colcon build --symlink-install

# 运行时环境设置  
source ~/unitree_ros2/setup.sh
```

---

## 8. 关键实现注意事项

### 8.1 时间同步

- **关键点**：确保所有传感器数据时间戳一致
- **方法**：使用ROS2时间同步机制
- **时间源**：机器人内部时钟为主时间源

### 8.2 消息频率

- **控制频率**：建议2ms（500Hz）
- **状态反馈**：SportModeState约30-50Hz
- **传感器数据**：激光雷达10-20Hz，IMU 100Hz+

### 8.3 安全机制

- **紧急停止**：必须实现急停功能
- **状态检查**：实时监控机器人状态和错误代码
- **通信异常处理**：网络断开、消息丢失的处理机制

### 8.4 单位转换

- **线性速度**：ROS2使用m/s，Go2内部单位需要确认
- **角速度**：ROS2使用rad/s，Go2内部单位需要确认
- **坐标系**：右手坐标系 vs 左手坐标系转换

---

## 9. 调试和验证方法

### 9.1 基础连接测试

```bash
# 检查网络连通性
ping 192.168.123.18

# 检查话题发布
ros2 topic list
ros2 topic echo /sportmodestate

# 检查DDS通信
ros2 node list
```

### 9.2 消息验证

```bash
# 查看消息结构
ros2 interface show unitree_go/msg/SportModeState
ros2 interface show unitree_api/msg/Request

# 测试控制指令
ros2 topic pub /api/sport/request unitree_api/msg/Request "..."
```

---

## 10. 第一阶段具体实现建议

### 10.1 数据类型定义优先级

1. **Velocity类型**：对应ROS2 Twist消息
2. **Posture类型**：对应机器人姿态控制
3. **SensorData类型**：统一传感器数据格式
4. **RobotState类型**：对应SportModeState
5. **BatteryInfo类型**：对应电源状态

### 10.2 接口设计重点

1. **IMotionController**：重点关注Go2的运动控制特性
2. **ISensorInterface**：重点关注Livox激光雷达和IMU
3. **IStateMonitor**：重点关注SportModeState解析
4. **IPowerManager**：重点关注LowState中的电源部分

这些信息为您设计基础抽象接口提供了充分的技术基础，确保接口设计既能满足Go2的具体需求，又具备足够的通用性支持未来扩展。
