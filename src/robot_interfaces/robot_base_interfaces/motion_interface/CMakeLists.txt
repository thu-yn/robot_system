cmake_minimum_required(VERSION 3.8)
project(motion_interface)

# ========== 编译器设置 ==========
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置C++标准为C++17（ROS2 Humble要求）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ========== ROS2包依赖查找 ==========
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ========== 依赖列表定义 ==========
set(DEPENDENCIES
  rclcpp
  std_msgs
  geometry_msgs
)

# ========== 库目标构建 ==========

# 运动控制数据类型库（包含实际的cpp实现）
add_library(${PROJECT_NAME}
  src/motion_types.cpp
)

# 设置目标属性
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 链接ROS2依赖
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})

# ========== 单元测试构建 ==========
# 暂时禁用测试以完成基本构建
if(BUILD_TESTING)
  # 查找测试依赖
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  # 代码风格检查
  ament_lint_auto_find_test_dependencies()
  
  # 运动类型单元测试
  ament_add_gtest(test_motion_types
    test/test_motion_types.cpp
  )
  if(TARGET test_motion_types)
    target_link_libraries(test_motion_types ${PROJECT_NAME})
    ament_target_dependencies(test_motion_types ${DEPENDENCIES})
  endif()
endif()

# ========== 安装配置 ==========

# 安装库文件
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装头文件
install(DIRECTORY
  include/
  DESTINATION include/
)

# ========== Ament导出设置 ==========

# 导出依赖信息
ament_export_dependencies(${DEPENDENCIES})

# 导出包含目录
ament_export_include_directories(include)

# 导出库
ament_export_libraries(${PROJECT_NAME})

# 导出目标
ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)

# ========== 包完成 ==========
ament_package()

# ========== 构建信息输出 ==========
message(STATUS "========================================")
message(STATUS "Motion Interface Package Configuration:")
message(STATUS "  - Project: ${PROJECT_NAME}")
message(STATUS "  - Version: 1.0.0")  
message(STATUS "  - C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  - Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "========================================")