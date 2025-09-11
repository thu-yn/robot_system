/**
 * @file test_motion_types.cpp
 * @brief 运动类型单元测试
 * @details
 *  - 本文件使用 GoogleTest（gtest）编写“功能单元测试”。
 *  - 在 CMake 中通过 ament_cmake_gtest 的 ament_add_gtest() 将本源文件编译为一个测试可执行文件（例如 test_motion_types）。
 *  - 当你在工作区执行 `colcon test` 时，ament 会调用 ctest；ctest 再调用 ament 的 run_test.py 脚本运行该可执行文件，
 *    并附带 `--gtest_output=xml:...` 参数，把 gtest 的结果导出到 test_results/ 目录，供 `colcon test-result` 汇总显示。
 *  - 你在 latest_test/motion_interface/stdout_stderr.log 中看到的 “[ RUN ] / [ OK ]” 就是 gtest 的标准输出。
 * 
 * 流程总览：
 *  CMakeLists.txt:
 *    if(BUILD_TESTING)
 *      find_package(ament_cmake_gtest REQUIRED)
 *      ament_add_gtest(test_motion_types test/test_motion_types.cpp)
 *      target_link_libraries(test_motion_types ${PROJECT_NAME})
 *    endif()
 *  运行：
 *    colcon build --cmake-args -DBUILD_TESTING=ON
 *    colcon test --packages-select motion_interface
 *    colcon test-result --verbose
 * 
 * 其中 `ament_add_gtest` 会：
 *  - 生成测试目标（可执行文件）
 *  - 将其注册到 ctest 测试列表中（这样 `ctest -N` 能列出）
 *  - 由 `colcon test` 驱动 ctest 执行，并收集 XML 结果
 * 
 * 提示：
 *  - 只有功能测试（gtest）会在这里显示 `[ PASSED ]`/`[ FAILED ]`。
 *  - 代码规范类测试（ament_lint_*）也是以“测试”的形式运行，但它们不是 gtest，它们的失败不会出现在本可执行的输出里，
 *    而是在对应的 ament_xxx 的测试项中报告（可在 test_result 或对应 stdout_stderr.log 中查看）。
 * 
 * @author Claude Code
 * @date 2024
 */

 #include <gtest/gtest.h>  // 引入 GoogleTest 主头文件，提供 TEST/TEST_F/EXPECT_* 宏
 #include <cmath>
 #include "robot_base_interfaces/motion_interface/motion_types.hpp"  // 被测头文件（库由 CMake 目标 ${PROJECT_NAME} 提供）
 
 #ifndef M_PI
 #define M_PI 3.14159265358979323846
 #endif
 
 // 为了书写简洁，使用命名空间别名（注意：在 cpplint 规则较严格的项目中，可能建议避免 using-directive）
 using namespace robot_base_interfaces::motion_interface;
 
 /**
  * @brief 测试夹具（Fixture）
  * @details
  *  - 继承 ::testing::Test，可以在 SetUp()/TearDown() 中统一做前置/清理工作。
  *  - 使用 TEST_F(类名, 用例名) 绑定该夹具；同一夹具的用例共享相同的测试环境初始化逻辑。
  */
 class MotionTypesTest : public ::testing::Test {
 protected:
     void SetUp() override {
         // 测试前的设置（每条用例开始前调用）
     }
 
     void TearDown() override {
         // 测试后的清理（每条用例结束后调用）
     }
 };
 
 // ========== Velocity 测试 ==========
 // 每个 TEST_F 宏就是一条独立的测试用例；gtest 会逐条运行并统计通过/失败数量。
 
 TEST_F(MotionTypesTest, VelocityDefaultConstructor) {
     // 验证默认构造的成员初值
     Velocity vel;
     EXPECT_FLOAT_EQ(vel.linear_x, 0.0f);
     EXPECT_FLOAT_EQ(vel.linear_y, 0.0f);
     EXPECT_FLOAT_EQ(vel.linear_z, 0.0f);
     EXPECT_FLOAT_EQ(vel.angular_x, 0.0f);
     EXPECT_FLOAT_EQ(vel.angular_y, 0.0f);
     EXPECT_FLOAT_EQ(vel.angular_z, 0.0f);
 }
 
 TEST_F(MotionTypesTest, VelocityParameterizedConstructor) {
     // 验证带参构造，注意该类型的三参构造只设置 x/y 线速度和 z 角速度，其余保持默认
     Velocity vel(1.0f, 0.5f, 0.8f);
     EXPECT_FLOAT_EQ(vel.linear_x, 1.0f);
     EXPECT_FLOAT_EQ(vel.linear_y, 0.5f);
     EXPECT_FLOAT_EQ(vel.angular_z, 0.8f);
     EXPECT_FLOAT_EQ(vel.linear_z, 0.0f);
     EXPECT_FLOAT_EQ(vel.angular_x, 0.0f);
     EXPECT_FLOAT_EQ(vel.angular_y, 0.0f);
 }
 
 // ========== Posture 测试 ==========
 
 TEST_F(MotionTypesTest, PostureDefaultConstructor) {
     // 验证默认姿态的初值（含 Go2 默认身高）
     Posture posture;
     EXPECT_FLOAT_EQ(posture.roll, 0.0f);
     EXPECT_FLOAT_EQ(posture.pitch, 0.0f);
     EXPECT_FLOAT_EQ(posture.yaw, 0.0f);
     EXPECT_FLOAT_EQ(posture.body_height, 0.32f); // Go2默认高度
 }
 
 TEST_F(MotionTypesTest, PostureParameterizedConstructor) {
     // 验证带参构造可覆盖 roll/pitch/yaw/body_height
     Posture posture(0.1f, 0.2f, 0.3f, 0.25f);
     EXPECT_FLOAT_EQ(posture.roll, 0.1f);
     EXPECT_FLOAT_EQ(posture.pitch, 0.2f);
     EXPECT_FLOAT_EQ(posture.yaw, 0.3f);
     EXPECT_FLOAT_EQ(posture.body_height, 0.25f);
 }
 
 // ========== Position 测试 ==========
 
 TEST_F(MotionTypesTest, PositionDefaultConstructor) {
     // 默认坐标为 0
     Position pos;
     EXPECT_FLOAT_EQ(pos.x, 0.0f);
     EXPECT_FLOAT_EQ(pos.y, 0.0f);
     EXPECT_FLOAT_EQ(pos.z, 0.0f);
 }
 
 TEST_F(MotionTypesTest, PositionParameterizedConstructor) {
     // 带参构造设置 x/y/z
     Position pos(1.0f, 2.0f, 3.0f);
     EXPECT_FLOAT_EQ(pos.x, 1.0f);
     EXPECT_FLOAT_EQ(pos.y, 2.0f);
     EXPECT_FLOAT_EQ(pos.z, 3.0f);
 }
 
 // ========== MotionCapabilities 测试 ==========
 
 TEST_F(MotionTypesTest, MotionCapabilitiesDefaults) {
     // 验证 Go2 的能力默认值（与被测头文件中的默认成员初值保持一致）
     MotionCapabilities caps;
     
     // 速度、姿态与机身高度的上/下限
     EXPECT_FLOAT_EQ(caps.max_linear_velocity, 1.5f);
     EXPECT_FLOAT_EQ(caps.max_angular_velocity, 2.0f);
     EXPECT_FLOAT_EQ(caps.max_lateral_velocity, 0.8f);
     EXPECT_FLOAT_EQ(caps.max_roll_angle, 0.4f);
     EXPECT_FLOAT_EQ(caps.max_pitch_angle, 0.4f);
     EXPECT_FLOAT_EQ(caps.min_body_height, 0.08f);
     EXPECT_FLOAT_EQ(caps.max_body_height, 0.42f);
     
     // 特殊能力开关
     EXPECT_TRUE(caps.can_climb_stairs);
     EXPECT_TRUE(caps.can_balance);
     EXPECT_TRUE(caps.can_lateral_move);
     EXPECT_TRUE(caps.can_dance);
     EXPECT_TRUE(caps.can_jump);
     EXPECT_TRUE(caps.can_flip);
     
     // 支持的模式/步态：通过容器包含性检查验证
     EXPECT_FALSE(caps.supported_modes.empty());
     EXPECT_TRUE(std::find(caps.supported_modes.begin(), caps.supported_modes.end(), 
                          MotionMode::BALANCE_STAND) != caps.supported_modes.end());
     EXPECT_TRUE(std::find(caps.supported_modes.begin(), caps.supported_modes.end(), 
                          MotionMode::LOCOMOTION) != caps.supported_modes.end());
     
     EXPECT_FALSE(caps.supported_gaits.empty());
     EXPECT_TRUE(std::find(caps.supported_gaits.begin(), caps.supported_gaits.end(), 
                          GaitType::TROT) != caps.supported_gaits.end());
 }
 
 // ========== MotionState 测试 ==========
 
 TEST_F(MotionTypesTest, MotionStateDefaultConstructor) {
     // 验证状态默认值，以及 4 足端容器初始化
     MotionState state;
     
     EXPECT_EQ(state.current_mode, MotionMode::IDLE);
     EXPECT_EQ(state.current_gait, GaitType::IDLE);
     EXPECT_FALSE(state.is_moving);
     EXPECT_FALSE(state.is_balanced);
     EXPECT_FLOAT_EQ(state.motion_progress, 0.0f);
     EXPECT_FLOAT_EQ(state.foot_raise_height, 0.09f);
     
     // Go2 足端数据：应初始化为长度 4
     EXPECT_EQ(state.foot_forces.size(), 4);
     EXPECT_EQ(state.foot_positions.size(), 4);
     EXPECT_EQ(state.foot_velocities.size(), 4);
     EXPECT_EQ(state.range_obstacles.size(), 4);
     
     for (int i = 0; i < 4; ++i) {
         EXPECT_FLOAT_EQ(state.foot_forces[i], 0.0f);
         EXPECT_FLOAT_EQ(state.range_obstacles[i], 0.0f);
     }
     
     EXPECT_EQ(state.timestamp_ns, 0);
     EXPECT_EQ(state.error_code, 0);
 }
 
 // ========== 枚举值测试 ==========
 // 通过 static_cast<int> 检查枚举底层值是否与约定一致（方便与设备协议/外部接口对接）
 
 TEST_F(MotionTypesTest, MotionModeEnums) {
     EXPECT_EQ(static_cast<int>(MotionMode::IDLE), 0);
     EXPECT_EQ(static_cast<int>(MotionMode::BALANCE_STAND), 1);
     EXPECT_EQ(static_cast<int>(MotionMode::POSE), 2);
     EXPECT_EQ(static_cast<int>(MotionMode::LOCOMOTION), 3);
     EXPECT_EQ(static_cast<int>(MotionMode::LIE_DOWN), 5);
     EXPECT_EQ(static_cast<int>(MotionMode::SIT), 10);
 }
 
 TEST_F(MotionTypesTest, GaitTypeEnums) {
     EXPECT_EQ(static_cast<int>(GaitType::IDLE), 0);
     EXPECT_EQ(static_cast<int>(GaitType::TROT), 1);
     EXPECT_EQ(static_cast<int>(GaitType::RUN), 2);
     EXPECT_EQ(static_cast<int>(GaitType::CLIMB_STAIR), 3);
     EXPECT_EQ(static_cast<int>(GaitType::DOWN_STAIR), 4);
     EXPECT_EQ(static_cast<int>(GaitType::ADJUST), 9);
 }
 
 TEST_F(MotionTypesTest, RobotTypeEnums) {
     EXPECT_EQ(static_cast<int>(RobotType::GO2), 0);
     EXPECT_EQ(static_cast<int>(RobotType::SPOT), 1);
     EXPECT_EQ(static_cast<int>(RobotType::ANYMAL), 2);
     EXPECT_EQ(static_cast<int>(RobotType::GENERIC), 99);
 }
 
 TEST_F(MotionTypesTest, MotionResultEnums) {
     EXPECT_EQ(static_cast<int>(MotionResult::SUCCESS), 0);
     EXPECT_EQ(static_cast<int>(MotionResult::INVALID_PARAMETER), 1);
     EXPECT_EQ(static_cast<int>(MotionResult::CAPABILITY_LIMITED), 2);
     EXPECT_EQ(static_cast<int>(MotionResult::EMERGENCY_STOP), 3);
     EXPECT_EQ(static_cast<int>(MotionResult::COMMUNICATION_ERROR), 4);
     EXPECT_EQ(static_cast<int>(MotionResult::UNKNOWN_ERROR), 99);
 }
 
 TEST_F(MotionTypesTest, EmergencyStopLevelEnums) {
     EXPECT_EQ(static_cast<int>(EmergencyStopLevel::SOFT_STOP), 0);
     EXPECT_EQ(static_cast<int>(EmergencyStopLevel::HARD_STOP), 1);
     EXPECT_EQ(static_cast<int>(EmergencyStopLevel::POWER_OFF), 2);
 }
 
 // ========== 边界值/一致性测试 ==========
 
 TEST_F(MotionTypesTest, VelocityLimits) {
     // 使用能力上限构造速度，检查传参到成员的映射是否正确
     MotionCapabilities caps;
     
     Velocity max_vel(caps.max_linear_velocity, caps.max_lateral_velocity, caps.max_angular_velocity);
     EXPECT_FLOAT_EQ(max_vel.linear_x, 1.5f);
     EXPECT_FLOAT_EQ(max_vel.linear_y, 0.8f);
     EXPECT_FLOAT_EQ(max_vel.angular_z, 2.0f);
     
     // 负值边界（有些系统允许负速度）
     Velocity neg_vel(-caps.max_linear_velocity, -caps.max_lateral_velocity, -caps.max_angular_velocity);
     EXPECT_FLOAT_EQ(neg_vel.linear_x, -1.5f);
     EXPECT_FLOAT_EQ(neg_vel.linear_y, -0.8f);
     EXPECT_FLOAT_EQ(neg_vel.angular_z, -2.0f);
 }
 
 TEST_F(MotionTypesTest, PostureLimits) {
     // 验证姿态/高度边界
     MotionCapabilities caps;
     
     Posture max_posture(caps.max_roll_angle, caps.max_pitch_angle, M_PI, caps.max_body_height);
     EXPECT_FLOAT_EQ(max_posture.roll, 0.4f);
     EXPECT_FLOAT_EQ(max_posture.pitch, 0.4f);
     EXPECT_FLOAT_EQ(max_posture.yaw, M_PI);
     EXPECT_FLOAT_EQ(max_posture.body_height, 0.42f);
     
     Posture min_height_posture(0.0f, 0.0f, 0.0f, caps.min_body_height);
     EXPECT_FLOAT_EQ(min_height_posture.body_height, 0.08f);
 }
 
 TEST_F(MotionTypesTest, Go2SpecificParameters) {
     // 与文档/规格对齐性的检查（防回归）
     MotionCapabilities caps;
     
     EXPECT_FLOAT_EQ(caps.max_linear_velocity, 1.5f);  // Go2规格: 1.5 m/s
     EXPECT_FLOAT_EQ(caps.max_angular_velocity, 2.0f); // Go2规格: 2.0 rad/s
     EXPECT_FLOAT_EQ(caps.min_body_height, 0.08f);     // Go2最低高度
     EXPECT_FLOAT_EQ(caps.max_body_height, 0.42f);     // Go2最高高度
     
     Posture default_posture;
     EXPECT_FLOAT_EQ(default_posture.body_height, 0.32f); // Go2默认站立高度
 }
 
 /**
  * @brief gtest 入口
  * @details
  *  - gtest 提供了自己的 main；这里显式实现 main 也可（两者选其一）。
  *  - `::testing::InitGoogleTest` 会解析命令行参数（例如 --gtest_filter=...），
  *    run_test.py 会注入 `--gtest_output=xml:...`，用于把结果写入 XML 文件。
  *  - `RUN_ALL_TESTS()` 返回 0 表示所有用例通过，非 0 表示有失败；ctest/colcon 据此判定测试通过/失败。
  */
 int main(int argc, char **argv) {
     ::testing::InitGoogleTest(&argc, argv);
     return RUN_ALL_TESTS();
 }