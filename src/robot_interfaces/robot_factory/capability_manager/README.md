# ROS2 Package: capability_manager

## 概述

`capability_manager` 是机器人适配器工厂（Robot Factory）模式中的数据驱动核心。它的主要职责是**加载、管理和查询**不同机器人平台的详细能力参数。这些参数被定义在一个外部的YAML配置文件中，使得系统可以在不修改代码的情况下，轻松地更新或添加新机器人的能力信息。

通过将机器人的具体能力（如最大速度、传感器类型、电池容量等）与核心业务逻辑解耦，该模块使得上层应用（如导航、任务规划）能够根据机器人的实际能力来动态调整其行为。例如，一个任务规划器可以查询能力管理器，以确定当前机器人是否具备执行特定任务（如爬楼梯）所需的能力。

## 文件结构与作用

```
.
├── config/
│   ├── capability_matrix.yaml    # 定义了已知机器人详细能力的YAML文件
│   └── default_capabilities.yaml # 为通用或未知机器人提供的默认能力配置
├── include/capability_manager/
│   ├── capability_manager.hpp      # 核心能力管理器类
│   ├── capability_loader.hpp       # 从YAML文件加载能力的加载器类
│   └── capability_definitions.hpp  # 所有能力参数的数据结构定义
├── src/
│   ├── capability_manager.cpp      # 管理器类的逻辑实现
│   ├── capability_loader.cpp       # 加载器类的逻辑实现
│   └── capability_definitions.cpp  # (通常为空) 数据结构定义的实现文件
├── CMakeLists.txt                  # CMake 构建配置文件
└── package.xml                     # ROS2 包元信息文件
```

### 详细文件说明

#### `config/capability_matrix.yaml`

- **作用**: 这是能力管理器的核心数据文件。它以人类可读的YAML格式，详细描述了每种已知机器人的能力矩阵。
- **结构**: 文件包含一个 `robots` 列表，每个机器人（如 `go2`）作为一个条目，下面详细列出了其 `motion`（运动）、`sensors`（传感器）、`power`（电源）等方面的具体参数。

#### `config/default_capabilities.yaml`

- **作用**: 提供一个备用的、通用的机器人能力配置。当系统中没有找到特定机器人的详细配置时，会使用这里的默认值。

#### `include/capability_manager/capability_definitions.hpp`

- **作用**: 定义了与YAML配置文件结构相对应的C++数据结构。
- **核心定义**:
    - `MotionCapabilities`, `SensorCapabilities`, `PowerCapabilities` 等结构体，分别对应能力矩阵中的各个部分。
    - `RobotCapabilities` (struct): 这是一个顶层结构体，聚合了上述所有具体的能力结构，完整地代表了一个机器人的全部能力信息。

#### `include/capability_manager/capability_loader.hpp`

- **作用**: 定义了 `CapabilityLoader` 类，负责解析YAML配置文件并将其内容填充到C++数据结构中。
- **核心功能**:
    - `loadCapabilities()`: 接受一个文件路径，使用 `yaml-cpp` 库加载并解析YAML文件，返回一个从 `RobotType` 到 `RobotCapabilities` 的映射（`std::map`）。

#### `include/capability_manager/capability_manager.hpp`

- **作用**: 定义了 `CapabilityManager` 单例类，它是与外部交互、查询机器人能力的主要入口。
- **核心功能**:
    - `getInstance()`: 获取能力管理器的全局唯一实例。
    - `loadCapabilities()`: 初始化管理器，调用 `CapabilityLoader` 来加载配置文件。
    - `getRobotCapabilities()`: 根据 `RobotType` 查询并返回一个机器人的完整能力结构。
    - `hasCapability()` 和 `findRobotsWithCapability()`: 提供更高级的查询功能，用于检查特定能力或找到具备某项能力的机器人。

#### `src/capability_loader.cpp`

- **作用**: `CapabilityLoader` 类的具体实现。
- **实现细节**:
    - 使用 `ament_index_cpp::get_package_share_directory` 来自动查找默认的 `capability_matrix.yaml` 配置文件路径，这使得包安装后也能正确定位配置文件。
    - 使用 `try-catch` 块来处理文件加载和解析过程中可能出现的 `YAML::Exception` 异常。
    - `parseYamlNode()`: 遍历YAML文件的 `robots` 节点，并将每个机器人的参数逐一填充到 `RobotCapabilities` 结构体中。

#### `src/capability_manager.cpp`

- **作用**: `CapabilityManager` 类的具体实现。
- **实现细节**:
    - 内部维护一个 `std::map<RobotType, RobotCapabilities>` 作为能力的内存数据库。
    - `loadCapabilities()` 方法确保能力只被加载一次。
    - `getRobotCapabilities()` 在找不到特定机器人的配置时，会尝试返回一个通用的（`GENERIC`）后备配置。

#### `CMakeLists.txt`

- **作用**: 定义了 `capability_manager` 包的构建规则。
- **主要配置**:
    - 使用 `pkg_check_modules` 来查找系统上安装的 `yaml-cpp` 库，并链接它。
    - 依赖 `ament_index_cpp` 以便在运行时定位包共享目录中的配置文件。

#### `package.xml`

- **作用**: ROS2包的清单文件，声明了包的元信息和依赖。

## 设计模式

- **数据驱动设计 (Data-Driven Design)**: 整个模块的核心逻辑是围绕外部配置文件（`capability_matrix.yaml`）构建的。这使得非开发人员（如机器人操作员）也可以通过修改YAML文件来调整系统行为，而无需重新编译代码。
- **单例模式 (Singleton Pattern)**: `CapabilityManager` 作为单例，为整个系统提供了一个统一的、全局的能力查询访问点。
- **策略模式 (Strategy Pattern) 的基础**: 该模块为实现策略模式提供了数据基础。其他模块可以查询 `CapabilityManager` 以获取机器人的能力，然后根据这些能力选择最合适的算法或行为策略。
