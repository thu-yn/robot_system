# ROS2 Package: robot_detector

## 概述

`robot_detector` 是机器人适配器工厂（Robot Factory）模式中的第一个关键模块。它的核心职责是**自动检测和识别**网络中连接的机器人类型。通过采用多策略检测机制，该模块能够为上层应用提供可靠的机器人识别结果，是实现多机器人平台无缝切换和统一管理的基础。

该包实现了多种检测方法，包括环境变量、网络扫描和ROS2话题探测，并为Unitree Go2等特定机器人提供了专门的检测逻辑。

## 文件结构与作用

```
.
├── include/robot_detector/
│   ├── robot_detector.hpp      # 核心检测器类定义
│   └── robot_types.hpp         # 机器人相关类型、枚举和数据结构定义
├── src/
│   ├── robot_detector.cpp      # 检测器核心逻辑的实现
│   └── robot_types.cpp         # 机器人类型的实现（如名称映射）
├── CMakeLists.txt              # CMake 构建配置文件
└── package.xml                 # ROS2 包元信息文件
```

### 详细文件说明

#### `include/robot_detector/robot_detector.hpp`

- **作用**: 定义了 `RobotDetector` 类的接口。这是与外部交互的主要入口。
- **核心功能**:
    - 声明了主要的检测方法，如 `detectRobot()`（自动检测）、`detectSpecificRobot()`（检测特定类型）和 `detectAllRobots()`（检测所有机器人）。
    - 定义了用于配置网络检测参数和启用/禁用特定检测方法的接口。
    - 内部声明了多种私有检测策略，如 `detectByEnvironment()`、`detectByNetwork()` 等。
    - 包含一个简单的缓存机制（`Cache`结构体），以避免在短时间内重复执行耗时的检测操作。

#### `include/robot_detector/robot_types.hpp`

- **作用**: 定义了整个机器人检测流程中使用的核心数据结构、枚举和常量。
- **核心定义**:
    - `RobotType` (enum class): 枚举了系统支持的机器人类型，如 `GO2`、`SPOT`、`ANYMAL` 等，为机器人身份提供了一个明确的类型安全标识。
    - `RobotDetectionResult` (struct): 封装了一次检测操作的结果，包含是否检测到、机器人类型、网络地址、置信度、检测方法等详细信息。
    - `NetworkDetectionConfig` (struct): 结构化的配置参数，用于网络检测，特别是针对Go2机器人的IP地址、端口和特征话题列表。
    - `go2_constants`: 包含专门用于Go2机器人检测的常量，如默认IP地址和特征ROS2话题。

#### `src/robot_detector.cpp`

- **作用**: `RobotDetector` 类的具体实现。
- **实现细节**:
    - `detectRobot()`: 实现了多级检测策略的调用流程，例如，它会按顺序尝试环境变量检测、网络检测等，并根据置信度返回最可靠的结果。
    - `detectByEnvironment()`: 实现从系统环境变量（如 `ROBOT_TYPE`）中获取机器人类型的逻辑。
    - `detectByNetwork()` 和 `detectGo2Robot()`: 实现了通过网络探测（目前为模拟的ping和ROS2话题检查）来识别Go2机器人的逻辑。
    - **缓存管理**: 实现了 `isCacheValid()` 和 `updateCache()` 等函数，用于管理检测结果的缓存，提高重复调用的性能。

#### `src/robot_types.cpp`

- **作用**: 为 `robot_types.hpp` 中声明的变量提供实现。
- **实现细节**:
    - 初始化 `ROBOT_TYPE_NAMES` 这个 `std::map`，提供了从 `RobotType` 枚举到其字符串名称（如 "Unitree Go2"）的映射。
    - 初始化 `go2_constants` 命名空间中的常量值。

#### `CMakeLists.txt`

- **作用**: 定义了 `robot_detector` 包的构建规则。
- **主要配置**:
    - 使用 `find_package` 查找 `rclcpp` 和其他机器人接口（`motion_interface` 等）依赖。
    - 将 `src/robot_detector.cpp` 和 `src/robot_types.cpp` 编译成一个名为 `robot_detector` 的共享库。
    - 使用 `target_include_directories` 将 `include` 目录声明为公共头文件目录。
    - 使用 `ament_target_dependencies` 链接所需的依赖项。
    - 使用 `install` 命令来安装库文件和头文件，以便其他ROS包可以使用此包。

#### `package.xml`

- **作用**: ROS2包的清单文件，定义了包的元信息和依赖关系。
- **主要内容**:
    - **`<name>`**: 包名称 `robot_detector`。
    - **`<version>`**: 版本号。
    - **`<description>`**: 包的功能描述。
    - **`<buildtool_depend>`**: 构建工具依赖 `ament_cmake`。
    - **`<build_depend>` 和 `<exec_depend>`**: 声明了编译和运行时的依赖项，如 `rclcpp` 和其他自定义接口包。

## 如何使用

其他ROS节点或库可以通过链接 `robot_detector` 库并包含 `robot_detector/robot_detector.hpp` 头文件来使用此功能。

**示例**:
```cpp
#include "robot_detector/robot_detector.hpp"
#include <iostream>

int main() {
    robot_factory::robot_detector::RobotDetector detector;
    
    // 设置环境变量以模拟检测
    setenv("ROBOT_TYPE", "go2", 1);

    robot_factory::robot_detector::RobotDetectionResult result = detector.detectRobot();

    if (result.is_detected) {
        std::cout << "机器人已检测到!" << std::endl;
        std::cout << "类型: " << robot_factory::robot_detector::ROBOT_TYPE_NAMES.at(result.robot_type) << std::endl;
        std::cout << "检测方法: " << result.detection_method << std::endl;
    } else {
        std::cout << "未检测到任何支持的机器人。" << std::endl;
    }

    return 0;
}
```
