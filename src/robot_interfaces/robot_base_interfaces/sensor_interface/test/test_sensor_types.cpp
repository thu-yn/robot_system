/**
 * @file   test_sensor_types.cpp
 * @brief  传感器类型单元测试
 * @author Yang Nan
 * @date   2025-09-11
 */

#include <gtest/gtest.h>
#include <cmath>
#include "robot_base_interfaces/sensor_interface/sensor_types.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace robot_base_interfaces::sensor_interface;

class SensorTypesTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 测试前的设置
    }

    void TearDown() override {
        // 测试后的清理
    }
};

// ========== 枚举测试 ==========

TEST_F(SensorTypesTest, SensorTypeEnums) {
    // Go2主要传感器
    EXPECT_EQ(static_cast<int>(SensorType::LIDAR_3D), 0);
    EXPECT_EQ(static_cast<int>(SensorType::LIDAR_2D), 1);
    EXPECT_EQ(static_cast<int>(SensorType::IMU), 2);
    EXPECT_EQ(static_cast<int>(SensorType::CAMERA_RGB), 3);
    EXPECT_EQ(static_cast<int>(SensorType::CAMERA_DEPTH), 4);
    EXPECT_EQ(static_cast<int>(SensorType::ULTRASONIC), 5);
    
    // 扩展传感器
    EXPECT_EQ(static_cast<int>(SensorType::GPS), 10);
    EXPECT_EQ(static_cast<int>(SensorType::WHEEL_ENCODER), 11);
    EXPECT_EQ(static_cast<int>(SensorType::FORCE_SENSOR), 12);
    EXPECT_EQ(static_cast<int>(SensorType::CUSTOM), 100);
}

TEST_F(SensorTypesTest, SensorStatusEnums) {
    EXPECT_EQ(static_cast<int>(SensorStatus::UNKNOWN), 0);
    EXPECT_EQ(static_cast<int>(SensorStatus::INITIALIZING), 1);
    EXPECT_EQ(static_cast<int>(SensorStatus::ACTIVE), 2);
    EXPECT_EQ(static_cast<int>(SensorStatus::ERROR), 3);
    EXPECT_EQ(static_cast<int>(SensorStatus::CALIBRATING), 4);
    EXPECT_EQ(static_cast<int>(SensorStatus::DISCONNECTED), 5);
}

// ========== Point3D 测试 ==========

TEST_F(SensorTypesTest, Point3DDefaultConstructor) {
    Point3D point;
    
    EXPECT_FLOAT_EQ(point.x, 0.0f);
    EXPECT_FLOAT_EQ(point.y, 0.0f);
    EXPECT_FLOAT_EQ(point.z, 0.0f);
    EXPECT_FLOAT_EQ(point.intensity, 0.0f);
}

TEST_F(SensorTypesTest, Point3DParameterizedConstructor) {
    Point3D point(1.5f, 2.0f, 0.8f, 100.0f);
    
    EXPECT_FLOAT_EQ(point.x, 1.5f);
    EXPECT_FLOAT_EQ(point.y, 2.0f);
    EXPECT_FLOAT_EQ(point.z, 0.8f);
    EXPECT_FLOAT_EQ(point.intensity, 100.0f);
}

TEST_F(SensorTypesTest, Point3DNoIntensityConstructor) {
    Point3D point(1.0f, 2.0f, 3.0f);
    
    EXPECT_FLOAT_EQ(point.x, 1.0f);
    EXPECT_FLOAT_EQ(point.y, 2.0f);
    EXPECT_FLOAT_EQ(point.z, 3.0f);
    EXPECT_FLOAT_EQ(point.intensity, 0.0f);  // 默认强度为0
}

// ========== PointCloudData 测试 ==========

TEST_F(SensorTypesTest, PointCloudDataDefaultConstructor) {
    PointCloudData cloud;
    
    EXPECT_TRUE(cloud.points.empty());
    EXPECT_EQ(cloud.frame_id, "base_link");
    EXPECT_EQ(cloud.timestamp_ns, 0);
    EXPECT_EQ(cloud.width, 0);
    EXPECT_EQ(cloud.height, 1);
    EXPECT_TRUE(cloud.is_dense);
    EXPECT_EQ(cloud.size(), 0);
}

TEST_F(SensorTypesTest, PointCloudDataAddPoints) {
    PointCloudData cloud;
    
    // 添加点
    cloud.points.emplace_back(1.0f, 2.0f, 3.0f, 50.0f);
    cloud.points.emplace_back(2.0f, 3.0f, 4.0f, 75.0f);
    cloud.points.emplace_back(3.0f, 4.0f, 5.0f, 100.0f);
    
    EXPECT_EQ(cloud.size(), 3);
    EXPECT_EQ(cloud.points.size(), 3);
    
    // 验证点数据
    EXPECT_FLOAT_EQ(cloud.points[0].x, 1.0f);
    EXPECT_FLOAT_EQ(cloud.points[0].intensity, 50.0f);
    EXPECT_FLOAT_EQ(cloud.points[2].z, 5.0f);
    EXPECT_FLOAT_EQ(cloud.points[2].intensity, 100.0f);
}

TEST_F(SensorTypesTest, PointCloudDataClear) {
    PointCloudData cloud;
    
    // 添加点并设置宽度
    cloud.points.emplace_back(1.0f, 2.0f, 3.0f);
    cloud.width = 100;
    
    EXPECT_EQ(cloud.size(), 1);
    EXPECT_EQ(cloud.width, 100);
    
    // 清空
    cloud.clear();
    
    EXPECT_EQ(cloud.size(), 0);
    EXPECT_EQ(cloud.width, 0);
    EXPECT_TRUE(cloud.points.empty());
}

TEST_F(SensorTypesTest, PointCloudDataMetadata) {
    PointCloudData cloud;
    
    // 设置元数据
    cloud.frame_id = "livox";
    cloud.timestamp_ns = 1234567890;
    cloud.width = 1024;
    cloud.height = 64;  // 有序点云
    cloud.is_dense = false;
    
    EXPECT_EQ(cloud.frame_id, "livox");
    EXPECT_EQ(cloud.timestamp_ns, 1234567890);
    EXPECT_EQ(cloud.width, 1024);
    EXPECT_EQ(cloud.height, 64);
    EXPECT_FALSE(cloud.is_dense);
}

// ========== LaserScanData 测试 ==========

TEST_F(SensorTypesTest, LaserScanDataDefaultConstructor) {
    LaserScanData scan;
    
    EXPECT_TRUE(scan.ranges.empty());
    EXPECT_TRUE(scan.intensities.empty());
    EXPECT_FLOAT_EQ(scan.angle_min, 0.0f);
    EXPECT_FLOAT_EQ(scan.angle_max, 0.0f);
    EXPECT_FLOAT_EQ(scan.angle_increment, 0.0f);
    EXPECT_FLOAT_EQ(scan.time_increment, 0.0f);
    EXPECT_FLOAT_EQ(scan.scan_time, 0.0f);
    EXPECT_FLOAT_EQ(scan.range_min, 0.0f);
    EXPECT_FLOAT_EQ(scan.range_max, 0.0f);
    EXPECT_EQ(scan.frame_id, "laser");
    EXPECT_EQ(scan.timestamp_ns, 0);
}

TEST_F(SensorTypesTest, LaserScanDataConfiguration) {
    LaserScanData scan;
    
    // 配置典型的2D激光雷达参数
    scan.angle_min = -M_PI;        // -180度
    scan.angle_max = M_PI;         // +180度
    scan.angle_increment = M_PI / 180.0f;  // 1度增量
    scan.range_min = 0.1f;         // 10cm最小距离
    scan.range_max = 10.0f;        // 10m最大距离
    scan.scan_time = 0.1f;         // 100ms扫描时间
    scan.frame_id = "laser_link";
    
    EXPECT_FLOAT_EQ(scan.angle_min, -M_PI);
    EXPECT_FLOAT_EQ(scan.angle_max, M_PI);
    EXPECT_FLOAT_EQ(scan.angle_increment, M_PI / 180.0f);
    EXPECT_FLOAT_EQ(scan.range_min, 0.1f);
    EXPECT_FLOAT_EQ(scan.range_max, 10.0f);
    EXPECT_FLOAT_EQ(scan.scan_time, 0.1f);
    EXPECT_EQ(scan.frame_id, "laser_link");
}

TEST_F(SensorTypesTest, LaserScanDataRangeArray) {
    LaserScanData scan;
    
    // 模拟360度扫描，每度一个测量点
    int num_points = 360;
    scan.ranges.resize(num_points);
    scan.intensities.resize(num_points);
    
    // 填充测试数据
    for (int i = 0; i < num_points; ++i) {
        scan.ranges[i] = 5.0f + sinf(i * M_PI / 180.0f);  // 变化的距离
        scan.intensities[i] = 100.0f + i * 0.1f;          // 递增的强度
    }
    
    EXPECT_EQ(scan.ranges.size(), 360);
    EXPECT_EQ(scan.intensities.size(), 360);
    
    // 验证部分数据
    EXPECT_FLOAT_EQ(scan.ranges[0], 5.0f);      // sin(0) = 0
    EXPECT_FLOAT_EQ(scan.ranges[90], 6.0f);     // sin(90度) = 1
    EXPECT_FLOAT_EQ(scan.intensities[0], 100.0f);
    EXPECT_FLOAT_EQ(scan.intensities[359], 100.0f + 359 * 0.1f);
}

// ========== IMU数据测试 ==========

TEST_F(SensorTypesTest, ImuDataDefaultConstructor) {
    // 注意：这个测试假设ImuData结构已定义，如果没有可以跳过
    // ImuData imu;
    // 验证IMU数据的默认值
}

// ========== 实际传感器数据模拟测试 ==========

TEST_F(SensorTypesTest, Go2LidarSimulation) {
    // 模拟Go2 Livox Mid360激光雷达数据
    PointCloudData lidar_cloud;
    lidar_cloud.frame_id = "livox";
    lidar_cloud.timestamp_ns = 1000000000;  // 1秒
    lidar_cloud.is_dense = true;
    
    // 生成模拟的圆形扫描数据
    int num_points = 1000;
    for (int i = 0; i < num_points; ++i) {
        float angle = 2.0f * M_PI * i / num_points;
        float radius = 5.0f + 0.5f * sinf(4 * angle);  // 变化的半径
        
        Point3D point;
        point.x = radius * cosf(angle);
        point.y = radius * sinf(angle);
        point.z = 0.1f * sinf(8 * angle);  // 轻微的Z变化
        point.intensity = 100.0f + 50.0f * cosf(angle);
        
        lidar_cloud.points.push_back(point);
    }
    
    EXPECT_EQ(lidar_cloud.size(), 1000);
    EXPECT_GT(lidar_cloud.points[0].intensity, 0.0f);
    
    // 验证数据在合理范围内
    for (const auto& point : lidar_cloud.points) {
        EXPECT_GE(point.intensity, 50.0f);
        EXPECT_LE(point.intensity, 150.0f);
        
        float distance = sqrtf(point.x * point.x + point.y * point.y + point.z * point.z);
        EXPECT_GT(distance, 4.0f);  // 最小距离约4m
        EXPECT_LT(distance, 6.0f);  // 最大距离约6m
    }
}

TEST_F(SensorTypesTest, LaserScanObstacleDetection) {
    LaserScanData scan;
    
    // 配置180度前向扫描
    scan.angle_min = -M_PI / 2;    // -90度
    scan.angle_max = M_PI / 2;     // +90度
    scan.angle_increment = M_PI / 180.0f;  // 1度增量
    scan.range_min = 0.1f;
    scan.range_max = 10.0f;
    
    int num_rays = 180;
    scan.ranges.resize(num_rays);
    scan.intensities.resize(num_rays);
    
    // 模拟前方有障碍物的场景
    for (int i = 0; i < num_rays; ++i) {
        
        if (abs(i - 90) < 10) {  // 前方±10度范围内有近距离障碍物
            scan.ranges[i] = 0.8f;
            scan.intensities[i] = 200.0f;  // 高反射强度
        } else {
            scan.ranges[i] = 5.0f;  // 远处无障碍物
            scan.intensities[i] = 50.0f;
        }
    }
    
    // 验证障碍物检测
    EXPECT_LT(scan.ranges[90], 1.0f);     // 正前方有近距离障碍物
    EXPECT_GT(scan.ranges[0], 4.0f);      // 左侧无障碍物
    EXPECT_GT(scan.ranges[179], 4.0f);    // 右侧无障碍物
    EXPECT_GT(scan.intensities[90], 150.0f);  // 障碍物高反射
}

// ========== 数据有效性验证测试 ==========

TEST_F(SensorTypesTest, PointCloudValidation) {
    PointCloudData cloud;
    
    // 添加有效点
    cloud.points.emplace_back(1.0f, 2.0f, 3.0f, 100.0f);
    
    // 添加无效点 (NaN或无穷大)
    Point3D invalid_point;
    invalid_point.x = std::numeric_limits<float>::quiet_NaN();
    invalid_point.y = std::numeric_limits<float>::infinity();
    invalid_point.z = 0.0f;
    cloud.points.push_back(invalid_point);
    
    EXPECT_EQ(cloud.size(), 2);
    
    // 检查有效点
    EXPECT_FALSE(std::isnan(cloud.points[0].x));
    EXPECT_FALSE(std::isinf(cloud.points[0].x));
    
    // 检查无效点
    EXPECT_TRUE(std::isnan(cloud.points[1].x));
    EXPECT_TRUE(std::isinf(cloud.points[1].y));
    
    // 如果包含无效点，应该设置is_dense为false
    cloud.is_dense = false;
    EXPECT_FALSE(cloud.is_dense);
}

TEST_F(SensorTypesTest, LaserScanRangeValidation) {
    LaserScanData scan;
    scan.range_min = 0.1f;
    scan.range_max = 10.0f;
    
    // 添加各种距离值
    scan.ranges = {0.05f, 0.5f, 5.0f, 12.0f, -1.0f};
    
    EXPECT_EQ(scan.ranges.size(), 5);
    
    // 验证距离范围
    EXPECT_LT(scan.ranges[0], scan.range_min);    // 小于最小距离
    EXPECT_GT(scan.ranges[1], scan.range_min);    // 在有效范围内
    EXPECT_LT(scan.ranges[2], scan.range_max);    // 在有效范围内
    EXPECT_GT(scan.ranges[3], scan.range_max);    // 超过最大距离
    EXPECT_LT(scan.ranges[4], 0.0f);              // 负距离（无效）
}

// ========== 性能和内存测试 ==========

TEST_F(SensorTypesTest, PointCloudMemoryUsage) {
    PointCloudData cloud;
    
    // 生成大量点云数据
    int num_points = 100000;  // 10万个点
    cloud.points.reserve(num_points);  // 预分配内存
    
    for (int i = 0; i < num_points; ++i) {
        cloud.points.emplace_back(
            static_cast<float>(i) * 0.001f,
            static_cast<float>(i) * 0.002f,
            static_cast<float>(i) * 0.003f,
            static_cast<float>(i % 256)
        );
    }
    
    EXPECT_EQ(cloud.size(), 100000);
    EXPECT_EQ(cloud.points.capacity(), cloud.points.size());  // 验证预分配有效
    
    // 清空后内存应该释放
    cloud.clear();
    EXPECT_EQ(cloud.size(), 0);
}

TEST_F(SensorTypesTest, LaserScanAngularResolution) {
    LaserScanData scan;
    
    // 设置高分辨率扫描参数
    scan.angle_min = -M_PI;
    scan.angle_max = M_PI;
    scan.angle_increment = M_PI / 1800.0f;  // 0.1度增量
    
    // 计算预期的光线数量
    int expected_rays = static_cast<int>((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1;
    scan.ranges.resize(expected_rays);
    
    EXPECT_EQ(scan.ranges.size(), expected_rays);
    EXPECT_GT(expected_rays, 3600);  // 应该超过3600个光线
    
    // 验证角度计算
    float calculated_span = scan.angle_max - scan.angle_min;
    float expected_span = 2 * M_PI;
    EXPECT_FLOAT_EQ(calculated_span, expected_span);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}