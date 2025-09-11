# ROS2 Package: adapter_factory

## 概述

`adapter_factory` 是机器人适配器工厂（Robot Factory）模式的核心，扮演着“工厂”本身的角色。它的主要职责是根据 `robot_detector` 包的检测结果，**实例化并返回一个与特定机器人相匹配的适配器实例**。

此包采用了经典的工厂设计模式和注册表（Registry）模式。它定义了一个所有机器人适配器都必须遵守的通用接口（`IRobotAdapter`），并提供了一个中央工厂类（`RobotAdapterFactory`）来创建具体的适配器。通过注册机制，系统可以轻松扩展，支持新的机器人类型，而无需修改工厂核心代码。

## 文件结构与作用

```
.
├── include/adapter_factory/
│   ├── i_robot_adapter.hpp         # 机器人适配器的通用抽象基类（接口）
│   ├── robot_adapter_factory.hpp   # 核心工厂类，用于创建适配器实例
│   └── adapter_registry.hpp        # 适配器创建函数的注册表（Registry）
├── src/
│   ├── i_robot_adapter.cpp         # 接口基类的（通常为空）实现文件
│   ├── robot_adapter_factory.cpp   # 工厂类的逻辑实现
│   └── adapter_registry.cpp        # 注册表类的逻辑实现
├── CMakeLists.txt                  # CMake 构建配置文件
└── package.xml                     # ROS2 包元信息文件
```

### 详细文件说明

#### `include/adapter_factory/i_robot_adapter.hpp`

- **作用**: 定义了 `IRobotAdapter` 纯虚基类，它是一个接口，统一了所有机器人适配器的标准行为。
- **核心功能**:
    - **生命周期管理**: `initialize()`, `shutdown()`
    - **连接管理**: `connect()`, `disconnect()`, `isConnected()`
    - **接口获取**: `getMotionController()`, `getSensorInterface()` 等，用于获取机器人不同功能的具体接口。
    - **元信息**: `getRobotType()`, `getRobotName()` 等，用于查询适配器自身的信息。
    - **配置与诊断**: `loadConfig()`, `healthCheck()`
- **关键定义**:
    - `AdapterCreationConfig` (struct): 定义了创建适配器时可以传入的配置参数，如IP地址、网络接口、超时时间等。
    - `IRobotAdapterPtr`: 使用 `std::shared_ptr` 定义了适配器对象的智能指针类型。

#### `include/adapter_factory/robot_adapter_factory.hpp`

- **作用**: 定义了 `RobotAdapterFactory` 单例类，这是创建适配器的主要入口。
- **核心功能**:
    - `getInstance()`: 获取工厂的全局唯一实例。
    - `createAdapter()`: 多个重载版本，可以根据 `RobotType`、`RobotDetectionResult` 或通过自动检测来创建适配器。
    - `registerAdapterCreator()`: 提供了向注册表注册新的适配器创建函数的便捷方法。
    - `quickCreateGo2Adapter()` 和 `autoCreateAdapter()`: 提供了用于快速创建常用或自动检测的适配器的静态便利函数。
- **关键定义**:
    - `AdapterCreationResult` (struct): 封装了适配器创建操作的结果，包含成功与否、适配器指针、错误信息等。

#### `include/adapter_factory/adapter_registry.hpp`

- **作用**: 定义了 `AdapterRegistry` 单例类，它维护一个从 `RobotType` 到其对应创建函数的映射。
- **核心功能**:
    - `getInstance()`: 获取注册表的全局唯一实例。
    - `registerCreator()`: 核心注册方法，将一个 `RobotType` 与一个能够创建相应适配器的函数（`AdapterCreator`）关联起来。
    - `getCreator()`: 根据 `RobotType` 查找并返回对应的创建函数。
    - `hasCreator()`: 检查某个 `RobotType` 是否已有注册的创建函数。

#### `src/robot_adapter_factory.cpp`

- **作用**: `RobotAdapterFactory` 类的具体实现。
- **实现细节**:
    - `createAdapter()` 的逻辑：首先调用 `AdapterRegistry` 来获取指定 `RobotType` 的创建函数，然后执行该函数以生成适配器实例。
    - 如果 `createAdapter()` 在未指定 `RobotType` 的情况下被调用，它会先使用 `robot_detector` 来自动检测机器人，然后再进行创建。

#### `src/adapter_registry.cpp`

- **作用**: `AdapterRegistry` 类的具体实现。
- **实现细节**:
    - 内部使用一个 `std::map` 来存储 `RobotType` 和 `CreatorInfo`（包含创建函数、版本等信息）的映射。
    - 使用 `std::mutex` 来保护注册表的并发访问，确保线程安全。

#### `src/i_robot_adapter.cpp`

- **作用**: `IRobotAdapter` 接口的实现文件。由于 `IRobotAdapter` 是一个纯接口类，此文件通常是空的。它的存在是为了将来可能需要为基类添加非纯虚函数或通用实现时，提供一个放置代码的位置。

#### `CMakeLists.txt`

- **作用**: 定义了 `adapter_factory` 包的构建规则。
- **主要配置**:
    - 依赖 `robot_detector` 和 `capability_manager` 等其他工厂模块。
    - 将 `src` 目录下的所有 `.cpp` 文件编译成 `adapter_factory` 共享库。
    - 链接所有必要的依赖项。

#### `package.xml`

- **作用**: ROS2包的清单文件。
- **主要内容**: 声明了对 `robot_detector` 和 `capability_manager` 包的编译和执行依赖。

## 设计模式

- **工厂模式 (Factory Pattern)**: `RobotAdapterFactory` 类根据输入（`RobotType`）来创建不同类型的适配器对象，隐藏了对象的创建逻辑。
- **单例模式 (Singleton Pattern)**: `RobotAdapterFactory` 和 `AdapterRegistry` 都通过 `getInstance()` 提供全局唯一的实例，方便在系统各处访问。
- **注册表模式 (Registry Pattern)**: `AdapterRegistry` 作为中心化的注册表，允许在运行时动态地添加（注册）新的机器人适配器类型，极大地提高了系统的可扩展性。
- **适配器模式 (Adapter Pattern)**: 每个具体的机器人适配器（如 `Go2Adapter`）都将遵循 `IRobotAdapter` 接口，将特定机器人的SDK或通信协议“适配”成统一的接口，从而对系统其余部分隐藏了底层实现的差异。
