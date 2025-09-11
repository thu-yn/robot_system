# sensor_interface

## 1. 概述

`sensor_interface` 是机器人基础接口层中负责传感器数据接入和管理的软件包。它为机器人搭载的各类传感器（如激光雷达、IMU、相机、里程计等）定义了一套标准的C++抽象接口和统一的数据类型。该接口旨在将原始传感器数据的获取和预处理过程进行封装，为上层应用（如SLAM、感知、导航）提供一个清晰、稳定、与硬件无关的数据源。

本软件包的设计重点在于统一数据格式和坐标系，并为不同品牌和型号的传感器提供可插拔的适配器，从而极大地提升了整个机器人系统的可移植性和可扩展性。

## 2. 核心设计

### 2.1. 抽象接口 (`ISensorInterface`)

核心抽象基类 `ISensorInterface` (`include/robot_base_interfaces/sensor_interface/i_sensor_interface.hpp`) 定义了所有传感器数据访问的API。主要功能包括：

- **初始化与配置**: `initialize()`, `shutdown()`。
- **数据流控制**: `start()` 和 `stop()` 用于启动和停止传感器的数据流。
- **数据回调注册**: 为每种传感器类型提供了独立的回调注册函数，上层应用可以按需订阅。
  - `setImuCallback()`: 订阅IMU数据。
  - `setLidarScanCallback()`: 订阅2D激光雷达数据 (`sensor_msgs/LaserScan`)。
  - `setPointCloudCallback()`: 订阅3D点云数据 (`sensor_msgs/PointCloud2`)。
  - `setOdometryCallback()`: 订阅里程计数据 (`nav_msgs/Odometry`)。
  - `setImageCallback()`: 订阅图像数据 (`sensor_msgs/Image`)。
  - `setGnssCallback()`: 订阅GNSS/GPS数据。
- **状态查询**:
  - `getSensorStatus()`: 获取特定传感器的当前状态（如`OK`, `ERROR`, `NOT_AVAILABLE`）。
  - `getSensorIntrinsics()` / `getSensorExtrinsics()`: 获取传感器的内参和外参（相对于`base_link`的变换）。

### 2.2. 数据类型 (`sensor_types.hpp`)

所有与传感器相关的数据结构、枚举和常量都定义在 `include/robot_base_interfaces/sensor_interface/sensor_types.hpp` 中。关键类型包括：

- **枚举**:
  - `SensorType`: 传感器类型（`IMU`, `LIDAR_2D`, `LIDAR_3D`, `CAMERA`, `ODOMETRY`, `GNSS`）。
  - `SensorStatus`: 传感器状态。
- **数据结构**:
  - `ImuData`, `LidarScanData`, `PointCloudData`, `OdometryData`, `ImageData`, `GnssData`: 这些是ROS标准消息类型的别名或轻量级封装，用于在接口层内部传递数据。
  - `CameraIntrinsics`: 描述相机内参，如焦距、主点、畸变系数。
  - `Transform`: 描述传感器外参，即相对于机器人基座的`TF`变换。

## 3. 使用方法

1.  **依赖**: 在需要使用传感器数据的软件包（如`slam_toolbox`, `perception_module`）的 `package.xml` 和 `CMakeLists.txt` 中添加对 `sensor_interface` 的依赖。

2.  **获取接口实例**: 上层应用通过 `AdapterFactory` 获取一个指向 `ISensorInterface` 实现的智能指针。工厂会根据当前连接的机器人自动选择并实例化正确的适配器（如 `Go2SensorAdapter`）。

3.  **调用接口**:
    ```cpp
    #include <robot_base_interfaces/sensor_interface/i_sensor_interface.hpp>
    #include <memory>

    // 通过工厂获取传感器接口实例
    std::shared_ptr<robot_base_interfaces::sensor_interface::ISensorInterface> sensor_interface;

    // 1. 初始化
    if (sensor_interface->initialize() != CommandResult::SUCCESS) {
        // 错误处理
    }

    // 2. 注册点云数据回调
    sensor_interface->setPointCloudCallback([](const auto& pc_msg) {
        // 在这里处理接收到的点云数据
        // 例如：将其发布到ROS网络，或直接用于障碍物检测
        // 注意：回调在独立线程执行，需保证线程安全
    });

    // 3. 注册IMU数据回调
    sensor_interface->setImuCallback([](const auto& imu_msg) {
        // 处理IMU数据，用于姿态估计
    });

    // 4. 启动传感器数据流
    sensor_interface->start();
    ```

## 4. 如何扩展

要为一个新的机器人平台或一个新的传感器添加支持，需要执行以下步骤：

1.  **创建新的适配器类**: 创建一个新类，继承自 `ISensorInterface`。
2.  **实现虚函数**: 在新类中，根据新平台的SDK或通信协议，具体实现 `ISensorInterface` 的所有纯虚函数。这通常意味着需要：
    - 在 `initialize()` 中，连接到传感器或订阅其原始数据话题。
    - 在数据接收的内部回调中，将原始数据转换为标准的数据类型（如 `sensor_msgs/PointCloud2`）。
    - 调用通过 `setPointCloudCallback` 等函数注册的外部回调，将转换后的标准数据传递给上层应用。
    - 实现 `getSensorExtrinsics` 以提供正确的TF变换。
3.  **注册到工厂**: 将新的适配器类注册到 `AdapterFactory` 中，使其能够被自动发现和实例化。

## 5. 编译与测试

- **编译**:
  ```bash
  colcon build --packages-select sensor_interface
  ```
- **测试**:
  ```bash
  colcon test --packages-select sensor_interface
  ```
  测试用例位于 `test` 目录下，主要验证数据结构的默认值和编译时属性。
