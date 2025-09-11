/**
 * @file test_i_motion_controller.cpp
 * @brief 运动控制器接口单元测试
 * @author Claude Code
 * @date 2024
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "robot_base_interfaces/motion_interface/i_motion_controller.hpp"

using namespace robot_base_interfaces::motion_interface;
using ::testing::Return;
using ::testing::_;

// ========== Mock 运动控制器 ==========

class MockMotionController : public IMotionController {
public:
    MOCK_METHOD(MotionResult, initialize, (), (override));
    MOCK_METHOD(MotionResult, shutdown, (), (override));
    MOCK_METHOD(MotionCapabilities, getCapabilities, (), (const, override));
    
    MOCK_METHOD(MotionResult, setVelocity, (const Velocity& velocity), (override));
    MOCK_METHOD(MotionResult, setPosture, (const Posture& posture), (override));
    MOCK_METHOD(MotionResult, setBodyHeight, (float height), (override));
    MOCK_METHOD(MotionResult, emergencyStop, (EmergencyStopLevel level), (override));
    
    MOCK_METHOD(MotionResult, switchMode, (MotionMode mode), (override));
    MOCK_METHOD(MotionResult, setGaitType, (GaitType gait), (override));
    MOCK_METHOD(MotionResult, balanceStand, (), (override));
    MOCK_METHOD(MotionResult, standUp, (), (override));
    MOCK_METHOD(MotionResult, standDown, (), (override));
    MOCK_METHOD(MotionResult, sit, (), (override));
    MOCK_METHOD(MotionResult, recoveryStand, (), (override));
    
    MOCK_METHOD(MotionResult, performDance, (int dance_type), (override));
    MOCK_METHOD(MotionResult, frontFlip, (), (override));
    MOCK_METHOD(MotionResult, frontJump, (), (override));
    MOCK_METHOD(MotionResult, hello, (), (override));
    MOCK_METHOD(MotionResult, stretch, (), (override));
    MOCK_METHOD(MotionResult, setSpeedLevel, (int level), (override));
    
    MOCK_METHOD(MotionState, getMotionState, (), (const, override));
    MOCK_METHOD(bool, isOperational, (), (const, override));
    MOCK_METHOD(uint32_t, getErrorCode, (), (const, override));
    MOCK_METHOD(bool, isMotionCompleted, (), (const, override));
    
    MOCK_METHOD(void, setStateCallback, (std::function<void(const MotionState&)> callback), (override));
    MOCK_METHOD(void, setErrorCallback, (std::function<void(uint32_t, const std::string&)> callback), (override));
    
    MOCK_METHOD(RobotType, getRobotType, (), (const, override));
    MOCK_METHOD(std::string, getVersion, (), (const, override));
};

// ========== 测试fixture ==========

class MotionControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        mock_controller = std::make_shared<MockMotionController>();
    }

    void TearDown() override {
        mock_controller.reset();
    }

    std::shared_ptr<MockMotionController> mock_controller;
    
    // 创建默认的Go2能力参数
    MotionCapabilities createGo2Capabilities() {
        MotionCapabilities caps;
        caps.max_linear_velocity = 1.5f;
        caps.max_angular_velocity = 2.0f;
        caps.max_lateral_velocity = 0.8f;
        caps.max_roll_angle = 0.4f;
        caps.max_pitch_angle = 0.4f;
        caps.min_body_height = 0.08f;
        caps.max_body_height = 0.42f;
        return caps;
    }
};

// ========== 初始化和配置测试 ==========

TEST_F(MotionControllerTest, InitializeSuccess) {
    EXPECT_CALL(*mock_controller, initialize())
        .WillOnce(Return(MotionResult::SUCCESS));

    auto result = mock_controller->initialize();
    EXPECT_EQ(result, MotionResult::SUCCESS);
}

TEST_F(MotionControllerTest, InitializeFailure) {
    EXPECT_CALL(*mock_controller, initialize())
        .WillOnce(Return(MotionResult::COMMUNICATION_ERROR));

    auto result = mock_controller->initialize();
    EXPECT_EQ(result, MotionResult::COMMUNICATION_ERROR);
}

TEST_F(MotionControllerTest, ShutdownSuccess) {
    EXPECT_CALL(*mock_controller, shutdown())
        .WillOnce(Return(MotionResult::SUCCESS));

    auto result = mock_controller->shutdown();
    EXPECT_EQ(result, MotionResult::SUCCESS);
}

TEST_F(MotionControllerTest, GetCapabilities) {
    auto expected_caps = createGo2Capabilities();
    
    EXPECT_CALL(*mock_controller, getCapabilities())
        .WillOnce(Return(expected_caps));

    auto caps = mock_controller->getCapabilities();
    EXPECT_FLOAT_EQ(caps.max_linear_velocity, 1.5f);
    EXPECT_FLOAT_EQ(caps.max_angular_velocity, 2.0f);
    EXPECT_FLOAT_EQ(caps.min_body_height, 0.08f);
    EXPECT_FLOAT_EQ(caps.max_body_height, 0.42f);
}

// ========== 基础运动控制测试 ==========

TEST_F(MotionControllerTest, SetVelocityValid) {
    Velocity vel(1.0f, 0.5f, 1.0f);
    
    EXPECT_CALL(*mock_controller, setVelocity(vel))
        .WillOnce(Return(MotionResult::SUCCESS));

    auto result = mock_controller->setVelocity(vel);
    EXPECT_EQ(result, MotionResult::SUCCESS);
}

TEST_F(MotionControllerTest, SetVelocityInvalid) {
    // 超出最大速度限制的速度
    Velocity vel(3.0f, 2.0f, 5.0f);
    
    EXPECT_CALL(*mock_controller, setVelocity(vel))
        .WillOnce(Return(MotionResult::CAPABILITY_LIMITED));

    auto result = mock_controller->setVelocity(vel);
    EXPECT_EQ(result, MotionResult::CAPABILITY_LIMITED);
}

TEST_F(MotionControllerTest, SetPostureValid) {
    Posture posture(0.1f, 0.2f, 0.3f, 0.32f);
    
    EXPECT_CALL(*mock_controller, setPosture(posture))
        .WillOnce(Return(MotionResult::SUCCESS));

    auto result = mock_controller->setPosture(posture);
    EXPECT_EQ(result, MotionResult::SUCCESS);
}

TEST_F(MotionControllerTest, SetBodyHeightValid) {
    float height = 0.32f;
    
    EXPECT_CALL(*mock_controller, setBodyHeight(height))
        .WillOnce(Return(MotionResult::SUCCESS));

    auto result = mock_controller->setBodyHeight(height);
    EXPECT_EQ(result, MotionResult::SUCCESS);
}

TEST_F(MotionControllerTest, SetBodyHeightInvalid) {
    float height = 0.5f; // 超出最大高度
    
    EXPECT_CALL(*mock_controller, setBodyHeight(height))
        .WillOnce(Return(MotionResult::CAPABILITY_LIMITED));

    auto result = mock_controller->setBodyHeight(height);
    EXPECT_EQ(result, MotionResult::CAPABILITY_LIMITED);
}

TEST_F(MotionControllerTest, EmergencyStopSoft) {
    EXPECT_CALL(*mock_controller, emergencyStop(EmergencyStopLevel::SOFT_STOP))
        .WillOnce(Return(MotionResult::SUCCESS));

    auto result = mock_controller->emergencyStop(EmergencyStopLevel::SOFT_STOP);
    EXPECT_EQ(result, MotionResult::SUCCESS);
}

TEST_F(MotionControllerTest, EmergencyStopHard) {
    EXPECT_CALL(*mock_controller, emergencyStop(EmergencyStopLevel::HARD_STOP))
        .WillOnce(Return(MotionResult::SUCCESS));

    auto result = mock_controller->emergencyStop(EmergencyStopLevel::HARD_STOP);
    EXPECT_EQ(result, MotionResult::SUCCESS);
}

// ========== 运动模式控制测试 ==========

TEST_F(MotionControllerTest, SwitchModeBalanceStand) {
    EXPECT_CALL(*mock_controller, switchMode(MotionMode::BALANCE_STAND))
        .WillOnce(Return(MotionResult::SUCCESS));

    auto result = mock_controller->switchMode(MotionMode::BALANCE_STAND);
    EXPECT_EQ(result, MotionResult::SUCCESS);
}

TEST_F(MotionControllerTest, SwitchModeLocomotion) {
    EXPECT_CALL(*mock_controller, switchMode(MotionMode::LOCOMOTION))
        .WillOnce(Return(MotionResult::SUCCESS));

    auto result = mock_controller->switchMode(MotionMode::LOCOMOTION);
    EXPECT_EQ(result, MotionResult::SUCCESS);
}

TEST_F(MotionControllerTest, SetGaitTypeTrot) {
    EXPECT_CALL(*mock_controller, setGaitType(GaitType::TROT))
        .WillOnce(Return(MotionResult::SUCCESS));

    auto result = mock_controller->setGaitType(GaitType::TROT);
    EXPECT_EQ(result, MotionResult::SUCCESS);
}

TEST_F(MotionControllerTest, BalanceStand) {
    EXPECT_CALL(*mock_controller, balanceStand())
        .WillOnce(Return(MotionResult::SUCCESS));

    auto result = mock_controller->balanceStand();
    EXPECT_EQ(result, MotionResult::SUCCESS);
}

TEST_F(MotionControllerTest, StandUp) {
    EXPECT_CALL(*mock_controller, standUp())
        .WillOnce(Return(MotionResult::SUCCESS));

    auto result = mock_controller->standUp();
    EXPECT_EQ(result, MotionResult::SUCCESS);
}

TEST_F(MotionControllerTest, StandDown) {
    EXPECT_CALL(*mock_controller, standDown())
        .WillOnce(Return(MotionResult::SUCCESS));

    auto result = mock_controller->standDown();
    EXPECT_EQ(result, MotionResult::SUCCESS);
}

TEST_F(MotionControllerTest, Sit) {
    EXPECT_CALL(*mock_controller, sit())
        .WillOnce(Return(MotionResult::SUCCESS));

    auto result = mock_controller->sit();
    EXPECT_EQ(result, MotionResult::SUCCESS);
}

TEST_F(MotionControllerTest, RecoveryStand) {
    EXPECT_CALL(*mock_controller, recoveryStand())
        .WillOnce(Return(MotionResult::SUCCESS));

    auto result = mock_controller->recoveryStand();
    EXPECT_EQ(result, MotionResult::SUCCESS);
}

// ========== Go2特有功能测试 ==========

TEST_F(MotionControllerTest, PerformDance1) {
    EXPECT_CALL(*mock_controller, performDance(1))
        .WillOnce(Return(MotionResult::SUCCESS));

    auto result = mock_controller->performDance(1);
    EXPECT_EQ(result, MotionResult::SUCCESS);
}

TEST_F(MotionControllerTest, PerformDance2) {
    EXPECT_CALL(*mock_controller, performDance(2))
        .WillOnce(Return(MotionResult::SUCCESS));

    auto result = mock_controller->performDance(2);
    EXPECT_EQ(result, MotionResult::SUCCESS);
}

TEST_F(MotionControllerTest, FrontFlip) {
    EXPECT_CALL(*mock_controller, frontFlip())
        .WillOnce(Return(MotionResult::SUCCESS));

    auto result = mock_controller->frontFlip();
    EXPECT_EQ(result, MotionResult::SUCCESS);
}

TEST_F(MotionControllerTest, FrontJump) {
    EXPECT_CALL(*mock_controller, frontJump())
        .WillOnce(Return(MotionResult::SUCCESS));

    auto result = mock_controller->frontJump();
    EXPECT_EQ(result, MotionResult::SUCCESS);
}

TEST_F(MotionControllerTest, Hello) {
    EXPECT_CALL(*mock_controller, hello())
        .WillOnce(Return(MotionResult::SUCCESS));

    auto result = mock_controller->hello();
    EXPECT_EQ(result, MotionResult::SUCCESS);
}

TEST_F(MotionControllerTest, Stretch) {
    EXPECT_CALL(*mock_controller, stretch())
        .WillOnce(Return(MotionResult::SUCCESS));

    auto result = mock_controller->stretch();
    EXPECT_EQ(result, MotionResult::SUCCESS);
}

TEST_F(MotionControllerTest, SetSpeedLevelValid) {
    int level = 5; // 1-9之间的有效值
    
    EXPECT_CALL(*mock_controller, setSpeedLevel(level))
        .WillOnce(Return(MotionResult::SUCCESS));

    auto result = mock_controller->setSpeedLevel(level);
    EXPECT_EQ(result, MotionResult::SUCCESS);
}

TEST_F(MotionControllerTest, SetSpeedLevelInvalid) {
    int level = 15; // 超出范围
    
    EXPECT_CALL(*mock_controller, setSpeedLevel(level))
        .WillOnce(Return(MotionResult::INVALID_PARAMETER));

    auto result = mock_controller->setSpeedLevel(level);
    EXPECT_EQ(result, MotionResult::INVALID_PARAMETER);
}

// ========== 状态查询测试 ==========

TEST_F(MotionControllerTest, GetMotionState) {
    MotionState expected_state;
    expected_state.current_mode = MotionMode::BALANCE_STAND;
    expected_state.current_gait = GaitType::IDLE;
    expected_state.is_moving = false;
    expected_state.is_balanced = true;
    
    EXPECT_CALL(*mock_controller, getMotionState())
        .WillOnce(Return(expected_state));

    auto state = mock_controller->getMotionState();
    EXPECT_EQ(state.current_mode, MotionMode::BALANCE_STAND);
    EXPECT_EQ(state.current_gait, GaitType::IDLE);
    EXPECT_FALSE(state.is_moving);
    EXPECT_TRUE(state.is_balanced);
}

TEST_F(MotionControllerTest, IsOperationalTrue) {
    EXPECT_CALL(*mock_controller, isOperational())
        .WillOnce(Return(true));

    EXPECT_TRUE(mock_controller->isOperational());
}

TEST_F(MotionControllerTest, IsOperationalFalse) {
    EXPECT_CALL(*mock_controller, isOperational())
        .WillOnce(Return(false));

    EXPECT_FALSE(mock_controller->isOperational());
}

TEST_F(MotionControllerTest, GetErrorCodeNoError) {
    EXPECT_CALL(*mock_controller, getErrorCode())
        .WillOnce(Return(0));

    EXPECT_EQ(mock_controller->getErrorCode(), 0);
}

TEST_F(MotionControllerTest, GetErrorCodeWithError) {
    uint32_t error_code = 1001;
    
    EXPECT_CALL(*mock_controller, getErrorCode())
        .WillOnce(Return(error_code));

    EXPECT_EQ(mock_controller->getErrorCode(), error_code);
}

TEST_F(MotionControllerTest, IsMotionCompleted) {
    EXPECT_CALL(*mock_controller, isMotionCompleted())
        .WillOnce(Return(true));

    EXPECT_TRUE(mock_controller->isMotionCompleted());
}

// ========== 回调测试 ==========

TEST_F(MotionControllerTest, SetStateCallback) {
    auto callback = [](const MotionState& state) {
        // 测试回调函数
    };
    
    EXPECT_CALL(*mock_controller, setStateCallback(_))
        .Times(1);

    mock_controller->setStateCallback(callback);
}

TEST_F(MotionControllerTest, SetErrorCallback) {
    auto callback = [](uint32_t code, const std::string& msg) {
        // 测试回调函数
    };
    
    EXPECT_CALL(*mock_controller, setErrorCallback(_))
        .Times(1);

    mock_controller->setErrorCallback(callback);
}

// ========== 扩展接口测试 ==========

TEST_F(MotionControllerTest, GetRobotType) {
    EXPECT_CALL(*mock_controller, getRobotType())
        .WillOnce(Return(RobotType::GO2));

    EXPECT_EQ(mock_controller->getRobotType(), RobotType::GO2);
}

TEST_F(MotionControllerTest, GetVersion) {
    std::string version = "1.0.0";
    
    EXPECT_CALL(*mock_controller, getVersion())
        .WillOnce(Return(version));

    EXPECT_EQ(mock_controller->getVersion(), version);
}

// ========== 集成测试场景 ==========

TEST_F(MotionControllerTest, CompleteStandupSequence) {
    // 模拟完整的站立序列
    ::testing::InSequence seq;
    
    EXPECT_CALL(*mock_controller, initialize())
        .WillOnce(Return(MotionResult::SUCCESS));
    
    EXPECT_CALL(*mock_controller, isOperational())
        .WillOnce(Return(true));
    
    EXPECT_CALL(*mock_controller, switchMode(MotionMode::BALANCE_STAND))
        .WillOnce(Return(MotionResult::SUCCESS));
    
    EXPECT_CALL(*mock_controller, standUp())
        .WillOnce(Return(MotionResult::SUCCESS));
    
    EXPECT_CALL(*mock_controller, isMotionCompleted())
        .WillOnce(Return(true));

    // 执行站立序列
    EXPECT_EQ(mock_controller->initialize(), MotionResult::SUCCESS);
    EXPECT_TRUE(mock_controller->isOperational());
    EXPECT_EQ(mock_controller->switchMode(MotionMode::BALANCE_STAND), MotionResult::SUCCESS);
    EXPECT_EQ(mock_controller->standUp(), MotionResult::SUCCESS);
    EXPECT_TRUE(mock_controller->isMotionCompleted());
}

TEST_F(MotionControllerTest, CompleteMovementSequence) {
    // 模拟完整的移动序列
    ::testing::InSequence seq;
    
    EXPECT_CALL(*mock_controller, switchMode(MotionMode::LOCOMOTION))
        .WillOnce(Return(MotionResult::SUCCESS));
    
    EXPECT_CALL(*mock_controller, setGaitType(GaitType::TROT))
        .WillOnce(Return(MotionResult::SUCCESS));
    
    Velocity vel(1.0f, 0.0f, 0.5f);
    EXPECT_CALL(*mock_controller, setVelocity(vel))
        .WillOnce(Return(MotionResult::SUCCESS));
    
    EXPECT_CALL(*mock_controller, emergencyStop(EmergencyStopLevel::SOFT_STOP))
        .WillOnce(Return(MotionResult::SUCCESS));

    // 执行移动序列
    EXPECT_EQ(mock_controller->switchMode(MotionMode::LOCOMOTION), MotionResult::SUCCESS);
    EXPECT_EQ(mock_controller->setGaitType(GaitType::TROT), MotionResult::SUCCESS);
    EXPECT_EQ(mock_controller->setVelocity(vel), MotionResult::SUCCESS);
    EXPECT_EQ(mock_controller->emergencyStop(EmergencyStopLevel::SOFT_STOP), MotionResult::SUCCESS);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}