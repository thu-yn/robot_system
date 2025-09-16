# Go2机器人ROS2通信接口文档

## 1. 通信架构概述

### 1.1 技术栈

- **ROS2版本**: Humble
- **DDS实现**: Cyclone DDS 0.10.2
- **通信方式**: 直接ROS2话题通信
- **网络连接**: 有线以太网

### 1.2 通信优势

- **低延迟**: 15-25ms系统控制延迟
- **高性能**: 直接DDS通信，避免SDK转换开销
- **标准化**: 使用ROS2标准消息格式
- **零拷贝**: 利用DDS共享内存机制

---

## 2. 机器人运动状态获取

### 2.1 SportModeState消息格式

Go2机器人通过 `unitree_go::msg::SportModeState`消息提供完整的运动状态信息：

```
// 消息结构定义
struct SportModeState {
    TimeSpec stamp;                    // 时间戳
    uint32 error_code;                 // 当前模式 (注意：字段名易误解，实际表示模式而非错误代码)
    IMUState imu_state;               // IMU状态数据
    uint8 mode;                       // 当前运动模式
    float32 progress;                 // 动作执行进度
    uint8 gait_type;                  // 步态类型
    float32 foot_raise_height;        // 抬腿高度
    float32[3] position;              // 机器人位置 (x,y,z)
    float32 body_height;              // 机体高度
    float32[3] velocity;              // 线速度 (vx,vy,vz)
    float32 yaw_speed;                // 偏航角速度
    float32[4] range_obstacle;        // 障碍物距离范围
    int16[4] foot_force;              // 四足端力传感器数值
    float32[12] foot_position_body;   // 足端相对机体位置 (每足3个坐标)
    float32[12] foot_speed_body;      // 足端相对机体速度 (每足3个速度)
};
```

### 2.2 IMU状态数据

```
struct IMUState {
    std::array<float, 4> quaternion;     // 四元数姿态 (w,x,y,z)
    std::array<float, 3> gyroscope;      // 角速度 (rad/s)
    std::array<float, 3> accelerometer;  // 线加速度 (m/s²)
    std::array<float, 3> rpy;            // 欧拉角 (roll,pitch,yaw) (rad)
    int8_t temperature;                  // IMU温度 (°C)
};
```

### 2.3 当前模式（error_code）定义

| 模式值    | 状态机名称                        |
| --------- | --------------------------------- |
| 100       | 灵动                              |
| 1001      | 阻尼                              |
| 1002      | 站立锁定                          |
| 1004/2006 | 蹲下                              |
| 1006      | 打招呼/伸懒腰/舞蹈/拜年/比心/开心 |
| 1007      | 坐下                              |
| 1008      | 前跳                              |
| 1009      | 扑人                              |
| 1013      | 平衡站立                          |
| 1015      | 常规行走                          |
| 1016      | 常规跑步                          |
| 1017      | 常规续航                          |
| 1091      | 摆姿势                            |
| 2007      | 闪避                              |
| 2008      | 并腿跑                            |
| 2009      | 跳跃跑                            |
| 2010      | 经典                              |
| 2011      | 倒立                              |
| 2012      | 前空翻                            |
| 2013      | 后空翻                            |
| 2014      | 左空翻                            |
| 2016      | 交叉步                            |
| 2017      | 直立                              |
| 2019      | 牵引                              |

### 2.4 运动模式(mode)定义

| 模式值 | 模式名称      | 说明         |
| ------ | ------------- | ------------ |
| 0      | idle          | 默认站立状态 |
| 1      | balanceStand  | 平衡站立     |
| 2      | pose          | 姿态控制     |
| 3      | locomotion    | 运动模式     |
| 4      | reserve       | 预留         |
| 5      | lieDown       | 趴下         |
| 6      | jointLock     | 关节锁定     |
| 7      | damping       | 阻尼模式     |
| 8      | recoveryStand | 恢复站立     |
| 9      | reserve       | 预留         |
| 10     | sit           | 坐下         |
| 11     | frontFlip     | 前空翻       |
| 12     | frontJump     | 前跳         |
| 13     | frontPounce   | 前扑         |

### 2.5步态类型(gait_type)定义

| 步态值 | 步态名称         | 说明       |
| ------ | ---------------- | ---------- |
| 0      | idle             | 静止状态   |
| 1      | trot             | 小跑步态   |
| 2      | run              | 奔跑步态   |
| 3      | climb stair      | 爬楼梯步态 |
| 4      | forwardDownStair | 下楼梯步态 |
| 9      | adjust           | 调整步态   |

---

## 3. 机器人运动状态订阅实现

### 3.1 话题说明

Go2机器人提供两种频率的状态话题：

- **高频话题**: `"sportmodestate"` (500Hz)
- **低频话题**: `"lf/sportmodestate"` (较低频率，适合一般应用)

### 3.2 订阅节点实现

```cpp
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

class MotionStateSuber : public rclcpp::Node {
public:
    MotionStateSuber() : Node("motion_state_suber") {
        // 选择话题频率
        const auto *topic_name = HIGH_FREQ ? "sportmodestate" : "lf/sportmodestate";
  
        // 创建订阅者
        suber_ = this->create_subscription<unitree_go::msg::SportModeState>(
            topic_name, 10,
            [this](const unitree_go::msg::SportModeState::SharedPtr data) {
                topic_callback(data);
            });
    }

private:
    void topic_callback(const unitree_go::msg::SportModeState::SharedPtr &data) {
        // 处理接收到的状态数据
        process_robot_state(data);
    }
  
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr suber_;
};
```

### 3.3 状态数据处理

```cpp
void process_robot_state(const unitree_go::msg::SportModeState::SharedPtr &data) {
    // 1. 基本运动信息
    RCLCPP_INFO(this->get_logger(),
                "Motion Mode: %d, Gait Type: %d, Foot Height: %.3f",
                data->mode, data->gait_type, data->foot_raise_height);
  
    // 2. 位置和速度信息
    RCLCPP_INFO(this->get_logger(),
                "Position: [%.3f, %.3f, %.3f], Height: %.3f",
                data->position[0], data->position[1], data->position[2],
                data->body_height);
        
    RCLCPP_INFO(this->get_logger(),
                "Velocity: [%.3f, %.3f, %.3f], Yaw Speed: %.3f",
                data->velocity[0], data->velocity[1], data->velocity[2],
                data->yaw_speed);
  
    // 3. IMU数据处理
    const auto& imu = data->imu_state;
    RCLCPP_INFO(this->get_logger(),
                "IMU - Roll: %.3f, Pitch: %.3f, Yaw: %.3f",
                imu.rpy[0], imu.rpy[1], imu.rpy[2]);
  
    // 4. 足端状态处理
    process_foot_states(data);
  
    // 5. 安全状态检查
    check_safety_status(data);
}
```

### 3.4 足端状态处理

```cpp
void process_foot_states(const unitree_go::msg::SportModeState::SharedPtr &data) {
    // 足端编号：0-前左，1-前右，2-后左，3-后右
    for (int foot = 0; foot < 4; foot++) {
        int base_idx = foot * 3;
  
        // 足端位置 (相对机体坐标系)
        float pos_x = data->foot_position_body[base_idx];
        float pos_y = data->foot_position_body[base_idx + 1];
        float pos_z = data->foot_position_body[base_idx + 2];
  
        // 足端速度 (相对机体坐标系)
        float vel_x = data->foot_speed_body[base_idx];
        float vel_y = data->foot_speed_body[base_idx + 1];
        float vel_z = data->foot_speed_body[base_idx + 2];
  
        // 足端力
        float force = data->foot_force[foot];
  
        RCLCPP_DEBUG(this->get_logger(),
                    "Foot %d - Pos:[%.3f,%.3f,%.3f], Vel:[%.3f,%.3f,%.3f], Force:%.1f",
                    foot, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, force);
    }
}
```

---

## 4. 机器人低层状态获取

### 4.1 LowState消息格式

Go2机器人通过 `unitree_go::msg::LowState`消息提供底层硬件状态信息：

```
// LowState消息结构定义
struct LowState {
    uint8[2] head;                    // 消息头
    uint8 level_flag;                 // 等级标志
    uint8 frame_reserve;              // 帧保留字段
    uint32[2] sn;                     // 序列号
    uint32[2] version;                // 版本信息
    uint16 bandwidth;                 // 带宽信息
    IMUState imu_state;               // IMU状态数据
    MotorState[20] motor_state;       // 20个电机状态（Go2实际使用12个）
    BmsState bms_state;               // 电池管理系统状态
    int16[4] foot_force;              // 足端接触力原始值
    int16[4] foot_force_est;          // 足端接触力估计值
    uint32 tick;                      // 时间戳计数
    uint8[40] wireless_remote;        // 无线遥控器数据
    uint8 bit_flag;                   // 位标志
    float32 adc_reel;                 // ADC读数
    int8 temperature_ntc1;            // NTC温度传感器1
    int8 temperature_ntc2;            // NTC温度传感器2
    float32 power_v;                  // 电池电压 (V)
    float32 power_a;                  // 电池电流 (A)
    uint16[4] fan_frequency;          // 风扇频率
    uint32 reserve;                   // 保留字段
    uint32 crc;                       // 校验码
};
```

### 4.2 电机状态数据

```
struct MotorState {
    uint8 mode;                       // 电机运行模式
    float32 q;                        // 关节位置反馈 (rad)
    float32 dq;                       // 关节速度反馈 (rad/s)
    float32 ddq;                      // 关节加速度反馈 (rad/s²)
    float32 tau_est;                  // 关节力矩估计 (N·m)
    float32 q_raw;                    // 原始位置数据（暂未使用）
    float32 dq_raw;                   // 原始速度数据（暂未使用）
    float32 ddq_raw;                  // 原始加速度数据（暂未使用）
    int8 temperature;                 // 电机温度 (°C)
    uint32 lost;                      // 通信丢失计数
    uint32[2] reserve;                // 保留字段
};
```

### 4.3 电池管理系统状态

```
struct BmsState {
    uint8 version_high;               // 电池版本高位
    uint8 version_low;                // 电池版本低位
    uint8 status;                     // 电池状态
    /*
    电池状态定义：
    - 0:SAFE,（未开启电池）
    - 1：WAKE_UP,（唤醒事件）
    - 6：PRECHG,（电池预充电中）
    - 7：CHG,（电池正常充电中）
    - 8：DCHG,（电池正常放电中）
    - 9：SELF_DCHG,（电池自放电中）
    - 11：ALARM,（电池存在警告）
    - 12：RESET_ALARM,（等待按键复位警告中）
    - 13：AUTO_RECOVERY （复位中）
    */
    uint8 soc;                        // 电池电量 (1%-100%)
    int32 current;                    // 充放电电流 (mA) 正值充电，负值放电
    uint16 cycle;                     // 充电循环次数
    int8[2] bq_ntc;                   // 电池内部NTC温度 [BAT1, BAT2]
    int8[2] mcu_ntc;                  // 电池MCU NTC温度 [RES, MOS]
    uint16[15] cell_vol;              // 15节电池单体电压 (mV)
};
```

### 4.4 电机编号定义

Go2机器人12个关节电机编号对应关系：

| 电机编号 | 关节名称       | 位置描述     |
| -------- | -------------- | ------------ |
| 0        | FR_hip_joint   | 前右髋关节   |
| 1        | FR_thigh_joint | 前右大腿关节 |
| 2        | FR_calf_joint  | 前右小腿关节 |
| 3        | FL_hip_joint   | 前左髋关节   |
| 4        | FL_thigh_joint | 前左大腿关节 |
| 5        | FL_calf_joint  | 前左小腿关节 |
| 6        | RR_hip_joint   | 后右髋关节   |
| 7        | RR_thigh_joint | 后右大腿关节 |
| 8        | RR_calf_joint  | 后右小腿关节 |
| 9        | RL_hip_joint   | 后左髋关节   |
| 10       | RL_thigh_joint | 后左大腿关节 |
| 11       | RL_calf_joint  | 后左小腿关节 |

---

## 5. 机器人低层状态订阅实现

### 5.1 话题选择

Go2提供两种频率的低层状态话题：

- **高频话题**: `"lowstate"` (500Hz)
- **低频话题**: `"lf/lowstate"` (较低频率)

### 5.2 低层状态订阅节点

```cpp
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/imu_state.hpp"
#include "unitree_go/msg/motor_state.hpp"

class LowStateSubscriber : public rclcpp::Node {
public:
    LowStateSubscriber() : Node("low_state_subscriber") {
        // 选择话题频率
        const auto *topic_name = HIGH_FREQ ? "lowstate" : "lf/lowstate";
  
        // 创建订阅者
        subscriber_ = this->create_subscription<unitree_go::msg::LowState>(
            topic_name, 10,
            [this](const unitree_go::msg::LowState::SharedPtr data) {
                process_low_state(data);
            });
    }

private:
    void process_low_state(const unitree_go::msg::LowState::SharedPtr &data) {
        // 处理IMU数据
        if (INFO_IMU) {
            process_imu_data(data->imu_state);
        }
  
        // 处理电机状态
        if (INFO_MOTOR) {
            process_motor_states(data->motor_state);
        }
  
        // 处理足端力数据
        if (INFO_FOOT_FORCE) {
            process_foot_forces(data);
        }
  
        // 处理电池状态
        if (INFO_BATTERY) {
            process_battery_state(data);
        }
  
        // 处理BMS状态
        process_bms_state(data->bms_state);
    }
  
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr subscriber_;
  
    // 配置标志
    static constexpr bool INFO_IMU = true;
    static constexpr bool INFO_MOTOR = true;
    static constexpr bool INFO_FOOT_FORCE = true;
    static constexpr bool INFO_BATTERY = true;
    static constexpr bool HIGH_FREQ = false;
};
```

### 5.3 IMU数据处理

```cpp
void process_imu_data(const unitree_go::msg::IMUState &imu) {
    // 欧拉角 (ZYX顺序，相对于机体坐标系)
    RCLCPP_INFO(this->get_logger(),
                "IMU Euler: Roll=%.3f, Pitch=%.3f, Yaw=%.3f (rad)",
                imu.rpy[0], imu.rpy[1], imu.rpy[2]);
  
    // 四元数姿态
    RCLCPP_DEBUG(this->get_logger(),
                 "IMU Quaternion: w=%.3f, x=%.3f, y=%.3f, z=%.3f",
                 imu.quaternion[0], imu.quaternion[1], 
                 imu.quaternion[2], imu.quaternion[3]);
  
    // 陀螺仪原始数据
    RCLCPP_DEBUG(this->get_logger(),
                 "IMU Gyro: wx=%.3f, wy=%.3f, wz=%.3f (rad/s)",
                 imu.gyroscope[0], imu.gyroscope[1], imu.gyroscope[2]);
  
    // 加速度计原始数据
    RCLCPP_DEBUG(this->get_logger(),
                 "IMU Accel: ax=%.3f, ay=%.3f, az=%.3f (m/s²)",
                 imu.accelerometer[0], imu.accelerometer[1], 
                 imu.accelerometer[2]);
}
```

### 5.4 电机状态处理

```cpp
void process_motor_states(const std::array<unitree_go::msg::MotorState, 20> &motors) {
    // Go2实际使用前12个电机
    for (int i = 0; i < 12; i++) {
        const auto &motor = motors[i];
  
        RCLCPP_DEBUG(this->get_logger(),
                    "Motor[%d]: pos=%.3f rad, vel=%.3f rad/s, "
                    "acc=%.3f rad/s², torque=%.3f N·m, temp=%d°C",
                    i, motor.q, motor.dq, motor.ddq, 
                    motor.tau_est, motor.temperature);
  
        // 电机安全检查
        check_motor_safety(i, motor);
    }
}

void check_motor_safety(int motor_id, const unitree_go::msg::MotorState &motor) {
    // 温度检查
    if (motor.temperature > 70) {
        RCLCPP_WARN(this->get_logger(),
                   "Motor[%d] temperature high: %d°C", motor_id, motor.temperature);
    }
  
    // 通信检查
    if (motor.lost > 100) {
        RCLCPP_ERROR(this->get_logger(),
                    "Motor[%d] communication lost count: %d", motor_id, motor.lost);
    }
  
    // 力矩检查
    if (abs(motor.tau_est) > 50.0) {  // 根据实际情况调整阈值
        RCLCPP_WARN(this->get_logger(),
                   "Motor[%d] high torque: %.3f N·m", motor_id, motor.tau_est);
    }
}
```

### 5.5 足端力处理

```cpp
void process_foot_forces(const unitree_go::msg::LowState::SharedPtr &data) {
    // 原始接触力数据
    RCLCPP_DEBUG(this->get_logger(),
                "Foot Forces: [%d, %d, %d, %d]",
                data->foot_force[0], data->foot_force[1],
                data->foot_force[2], data->foot_force[3]);
  
    // 估计接触力数据
    RCLCPP_DEBUG(this->get_logger(),
                "Estimated Forces: [%d, %d, %d, %d]",
                data->foot_force_est[0], data->foot_force_est[1],
                data->foot_force_est[2], data->foot_force_est[3]);
  
    // 接触检测
    for (int i = 0; i < 4; i++) {
        bool in_contact = abs(data->foot_force_est[i]) > contact_threshold_;
        foot_contact_states_[i] = in_contact;
  
        if (in_contact != last_contact_states_[i]) {
            std::string foot_names[] = {"FR", "FL", "RR", "RL"};
            RCLCPP_INFO(this->get_logger(),
                       "Foot %s contact changed: %s",
                       foot_names[i].c_str(),
                       in_contact ? "ON" : "OFF");
        }
        last_contact_states_[i] = in_contact;
    }
}

private:
    int16_t contact_threshold_ = 50;  // 接触力阈值
    bool foot_contact_states_[4] = {false};
    bool last_contact_states_[4] = {false};
```

### 5.6 电池状态处理

```cpp
void process_battery_state(const unitree_go::msg::LowState::SharedPtr &data) {
    float voltage = data->power_v;
    float current = data->power_a;
  
    RCLCPP_INFO(this->get_logger(),
                "Battery: Voltage=%.2fV, Current=%.3fA, Power=%.2fW",
                voltage, current, voltage * current);
  
    // 电池电压安全检查
    if (voltage < low_voltage_threshold_) {
        RCLCPP_WARN(this->get_logger(),
                   "Low battery voltage: %.2fV", voltage);
    }
}

void process_bms_state(const unitree_go::msg::BmsState &bms) {
    // 电池电量
    RCLCPP_INFO(this->get_logger(),
                "Battery SOC: %d%%, Status: %d, Cycles: %d",
                bms.soc, bms.status, bms.cycle);
  
    // 充放电状态
    if (bms.current > 0) {
        RCLCPP_INFO(this->get_logger(),
                   "Battery charging at %.3fA", bms.current / 1000.0f);
    } else if (bms.current < 0) {
        RCLCPP_INFO(this->get_logger(),
                   "Battery discharging at %.3fA", -bms.current / 1000.0f);
    }
  
    // 电池温度监控
    RCLCPP_DEBUG(this->get_logger(),
                "Battery Temps: BQ[%d,%d]°C, MCU[%d,%d]°C",
                bms.bq_ntc[0], bms.bq_ntc[1],
                bms.mcu_ntc[0], bms.mcu_ntc[1]);
  
    // 单体电压监控
    process_cell_voltages(bms.cell_vol);
}

void process_cell_voltages(const std::array<uint16_t, 15> &cell_voltages) {
    uint16_t min_vol = *std::min_element(cell_voltages.begin(), cell_voltages.end());
    uint16_t max_vol = *std::max_element(cell_voltages.begin(), cell_voltages.end());
  
    RCLCPP_DEBUG(this->get_logger(),
                "Cell Voltages: Min=%dmV, Max=%dmV, Diff=%dmV",
                min_vol, max_vol, max_vol - min_vol);
  
    // 电池均衡检查
    if ((max_vol - min_vol) > cell_imbalance_threshold_) {
        RCLCPP_WARN(this->get_logger(),
                   "Battery cell imbalance detected: %dmV difference",
                   max_vol - min_vol);
    }
}

private:
    float low_voltage_threshold_ = 20.0f;  // 低电压阈值
    uint16_t cell_imbalance_threshold_ = 100;  // 单体电压差阈值(mV)
```

---

## 6. 机器人遥控器状态获取

### 6.1 WirelessController消息格式

Go2机器人通过 `unitree_go::msg::WirelessController`消息提供遥控器状态信息：

```
// WirelessController消息结构定义
struct WirelessController {
    float32 lx;                       // 左摇杆X轴值 (-1.0 ~ 1.0)
    float32 ly;                       // 左摇杆Y轴值 (-1.0 ~ 1.0)
    float32 rx;                       // 右摇杆X轴值 (-1.0 ~ 1.0)
    float32 ry;                       // 右摇杆Y轴值 (-1.0 ~ 1.0)
    uint16 keys;                      // 按键状态位掩码
};
```

### 6.2 摇杆坐标系定义

```
左摇杆 (lx, ly)               右摇杆 (rx, ry)
       ly                           ry
        ↑                            ↑
        |                            |
lx ← ---+--- → lx            rx ← ---+--- → rx
        |                            |
        ↓                            ↓
       ly                           ry

取值范围: [-1.0, 1.0]
中性位置: (0.0, 0.0)
```

### 6.3 按键编码定义

Go2遥控器按键采用位掩码编码方式：

| 按键   | 位位置 | 数值   | 描述     |
| ------ | ------ | ------ | -------- |
| R1     | bit 0  | 0x0001 | 右肩键1  |
| L1     | bit 1  | 0x0002 | 左肩键1  |
| Start  | bit 2  | 0x0004 | 开始键   |
| Select | bit 3  | 0x0008 | 选择键   |
| R2     | bit 4  | 0x0010 | 右肩键2  |
| L2     | bit 5  | 0x0020 | 左肩键2  |
| F1     | bit 6  | 0x0040 | 功能键1  |
| F2     | bit 7  | 0x0080 | 功能键2  |
| A      | bit 8  | 0x0100 | A键      |
| B      | bit 9  | 0x0200 | B键      |
| X      | bit 10 | 0x0400 | X键      |
| Y      | bit 11 | 0x0800 | Y键      |
| UP     | bit 12 | 0x1000 | 方向键上 |
| RIGHT  | bit 13 | 0x2000 | 方向键右 |
| DOWN   | bit 14 | 0x4000 | 方向键下 |
| LEFT   | bit 15 | 0x8000 | 方向键左 |

---

## 7. 机器人遥控器状态订阅实现

### 7.1 基础订阅节点

```cpp
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/wireless_controller.hpp"

class WirelessControllerSubscriber : public rclcpp::Node {
public:
    WirelessControllerSubscriber() : Node("wireless_controller_subscriber") {
        // 订阅遥控器状态话题
        subscriber_ = this->create_subscription<unitree_go::msg::WirelessController>(
            "/wirelesscontroller", 10,
            [this](const unitree_go::msg::WirelessController::SharedPtr data) {
                process_controller_input(data);
            });
  
        RCLCPP_INFO(this->get_logger(), "Wireless controller subscriber started");
    }

private:
    void process_controller_input(const unitree_go::msg::WirelessController::SharedPtr &data) {
        // 处理摇杆输入
        process_joystick_input(data);
  
        // 处理按键输入
        process_button_input(data);
  
        // 状态更新和事件触发
        update_controller_state(data);
    }
  
    rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr subscriber_;
};
```

### 7.2 摇杆输入处理

```cpp
void process_joystick_input(const unitree_go::msg::WirelessController::SharedPtr &data) {
    // 获取摇杆值
    float left_x = data->lx;
    float left_y = data->ly;
    float right_x = data->rx;
    float right_y = data->ry;
  
    // 死区处理 - 消除小幅抖动
    const float deadzone = 0.05f;
    left_x = apply_deadzone(left_x, deadzone);
    left_y = apply_deadzone(left_y, deadzone);
    right_x = apply_deadzone(right_x, deadzone);
    right_y = apply_deadzone(right_y, deadzone);
  
    // 左摇杆通常用于控制机器人移动
    if (abs(left_x) > 0.01f || abs(left_y) > 0.01f) {
        RCLCPP_DEBUG(this->get_logger(),
                    "Left Joystick: X=%.3f, Y=%.3f", left_x, left_y);
  
        // 转换为机器人运动命令
        convert_to_motion_command(left_x, left_y);
    }
  
    // 右摇杆通常用于控制机器人旋转或视角
    if (abs(right_x) > 0.01f || abs(right_y) > 0.01f) {
        RCLCPP_DEBUG(this->get_logger(),
                    "Right Joystick: X=%.3f, Y=%.3f", right_x, right_y);
  
        // 转换为旋转命令
        convert_to_rotation_command(right_x, right_y);
    }
}

float apply_deadzone(float value, float deadzone) {
    if (abs(value) < deadzone) {
        return 0.0f;
    }
    // 重新映射到 [-1, 1] 范围，消除死区影响
    float sign = value > 0 ? 1.0f : -1.0f;
    return sign * (abs(value) - deadzone) / (1.0f - deadzone);
}

void convert_to_motion_command(float lx, float ly) {
    // 将摇杆输入转换为线速度命令
    // ly: 前进/后退 (正值前进，负值后退)
    // lx: 左右平移 (正值右移，负值左移)
  
    float max_linear_vel = 1.0f;  // m/s
    float linear_x = ly * max_linear_vel;
    float linear_y = lx * max_linear_vel;
  
    // 发布运动命令 (这里需要结合后续的运动控制部分)
    RCLCPP_INFO(this->get_logger(),
               "Motion Command: linear_x=%.3f, linear_y=%.3f",
               linear_x, linear_y);
}

void convert_to_rotation_command(float rx, float ry) {
    // rx通常用于偏航控制
    float max_angular_vel = 2.0f;  // rad/s
    float angular_z = rx * max_angular_vel;
  
    RCLCPP_INFO(this->get_logger(),
               "Rotation Command: angular_z=%.3f", angular_z);
}
```

### 7.3 按键输入处理

```cpp
void process_button_input(const unitree_go::msg::WirelessController::SharedPtr &data) {
    uint16_t keys = data->keys;
  
    // 按键状态检测
    bool r1_pressed = check_button_pressed(keys, 0x0001);
    bool l1_pressed = check_button_pressed(keys, 0x0002);
    bool start_pressed = check_button_pressed(keys, 0x0004);
    bool select_pressed = check_button_pressed(keys, 0x0008);
    bool r2_pressed = check_button_pressed(keys, 0x0010);
    bool l2_pressed = check_button_pressed(keys, 0x0020);
    bool a_pressed = check_button_pressed(keys, 0x0100);
    bool b_pressed = check_button_pressed(keys, 0x0200);
    bool x_pressed = check_button_pressed(keys, 0x0400);
    bool y_pressed = check_button_pressed(keys, 0x0800);
  
    // 方向键检测
    bool up_pressed = check_button_pressed(keys, 0x1000);
    bool right_pressed = check_button_pressed(keys, 0x2000);
    bool down_pressed = check_button_pressed(keys, 0x4000);
    bool left_pressed = check_button_pressed(keys, 0x8000);
  
    // 按键事件处理
    handle_button_events(keys);
  
    // 组合键检测
    detect_key_combinations(keys);
}

bool check_button_pressed(uint16_t keys, uint16_t button_mask) {
    return (keys & button_mask) != 0;
}

void handle_button_events(uint16_t current_keys) {
    // 检测按键状态变化
    uint16_t pressed = current_keys & (~last_keys_);      // 新按下的键
    uint16_t released = (~current_keys) & last_keys_;     // 新释放的键
  
    if (pressed != 0) {
        RCLCPP_INFO(this->get_logger(), "Keys pressed: 0x%04X", pressed);
  
        // 具体按键事件处理
        if (pressed & 0x0004) {  // Start键
            RCLCPP_INFO(this->get_logger(), "Start button pressed - System ready!");
            handle_start_button();
        }
  
        if (pressed & 0x0008) {  // Select键
            RCLCPP_INFO(this->get_logger(), "Select button pressed - Emergency stop!");
            handle_emergency_stop();
        }
  
        if (pressed & 0x0100) {  // A键
            RCLCPP_INFO(this->get_logger(), "A button pressed - Confirm action");
            handle_confirm_action();
        }
  
        if (pressed & 0x0200) {  // B键
            RCLCPP_INFO(this->get_logger(), "B button pressed - Cancel action");
            handle_cancel_action();
        }
    }
  
    if (released != 0) {
        RCLCPP_DEBUG(this->get_logger(), "Keys released: 0x%04X", released);
    }
  
    last_keys_ = current_keys;
}

void detect_key_combinations(uint16_t keys) {
    // L1 + R1: 模式切换
    if ((keys & 0x0003) == 0x0003) {
        RCLCPP_INFO(this->get_logger(), "Mode switch combination detected");
        handle_mode_switch();
    }
  
    // L2 + R2: 紧急停止
    if ((keys & 0x0030) == 0x0030) {
        RCLCPP_WARN(this->get_logger(), "Emergency stop combination detected!");
        handle_emergency_stop();
    }
  
    // Start + Select: 系统复位
    if ((keys & 0x000C) == 0x000C) {
        RCLCPP_WARN(this->get_logger(), "System reset combination detected");
        handle_system_reset();
    }
}

private:
    uint16_t last_keys_ = 0;  // 上一帧按键状态
```

### 7.4 控制器状态管理

```cpp
class ControllerStateManager {
public:
    enum ControlMode {
        MANUAL_CONTROL,      // 手动遥控模式
        AUTONOMOUS_NAV,      // 自主导航模式
        EMERGENCY_STOP,      // 紧急停止模式
        IDLE                 // 空闲模式
    };
  
    void update_state(const unitree_go::msg::WirelessController::SharedPtr &data) {
        current_controller_data_ = *data;
        last_update_time_ = std::chrono::steady_clock::now();
  
        // 连接状态检查
        check_connection_status();
  
        // 模式切换逻辑
        update_control_mode();
    }
  
    bool is_controller_active() const {
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>
                       (now - last_update_time_);
        return duration.count() < 1000;  // 1秒内有数据更新
    }
  
    ControlMode get_current_mode() const { return current_mode_; }
  
    std::pair<float, float> get_motion_command() const {
        if (current_mode_ != MANUAL_CONTROL) {
            return {0.0f, 0.0f};
        }
  
        // 应用死区和缩放
        float linear_x = apply_deadzone(current_controller_data_.ly, 0.05f);
        float angular_z = apply_deadzone(current_controller_data_.rx, 0.05f);
  
        // 速度限制
        linear_x *= max_linear_velocity_;
        angular_z *= max_angular_velocity_;
  
        return {linear_x, angular_z};
    }

private:
    unitree_go::msg::WirelessController current_controller_data_;
    std::chrono::steady_clock::time_point last_update_time_;
    ControlMode current_mode_ = IDLE;
    float max_linear_velocity_ = 1.0f;   // m/s
    float max_angular_velocity_ = 2.0f;  // rad/s
  
    void check_connection_status() {
        if (!is_controller_active()) {
            RCLCPP_WARN(rclcpp::get_logger("controller_manager"),
                       "Controller connection lost - switching to safe mode");
            current_mode_ = EMERGENCY_STOP;
        }
    }
  
    void update_control_mode() {
        uint16_t keys = current_controller_data_.keys;
  
        // 紧急停止检查
        if ((keys & 0x0030) == 0x0030) {  // L2 + R2
            current_mode_ = EMERGENCY_STOP;
            return;
        }
  
        // 模式切换逻辑
        if (keys & 0x0004) {  // Start键 - 激活手动控制
            current_mode_ = MANUAL_CONTROL;
        } else if (keys & 0x0008) {  // Select键 - 激活自主导航
            current_mode_ = AUTONOMOUS_NAV;
        }
    }
};
```

### 7.5 遥控器输入滤波

```cpp
class ControllerInputFilter {
public:
    ControllerInputFilter(float alpha = 0.2f) : filter_alpha_(alpha) {
        // 初始化滤波器
        filtered_lx_ = filtered_ly_ = filtered_rx_ = filtered_ry_ = 0.0f;
    }
  
    unitree_go::msg::WirelessController filter_input(
        const unitree_go::msg::WirelessController &raw_input) {
  
        unitree_go::msg::WirelessController filtered;
  
        // 低通滤波器 - 减少噪声和抖动
        filtered_lx_ = low_pass_filter(raw_input.lx, filtered_lx_);
        filtered_ly_ = low_pass_filter(raw_input.ly, filtered_ly_);
        filtered_rx_ = low_pass_filter(raw_input.rx, filtered_rx_);
        filtered_ry_ = low_pass_filter(raw_input.ry, filtered_ry_);
  
        filtered.lx = filtered_lx_;
        filtered.ly = filtered_ly_;
        filtered.rx = filtered_rx_;
        filtered.ry = filtered_ry_;
        filtered.keys = raw_input.keys;  // 按键不需要滤波
  
        return filtered;
    }

private:
    float filter_alpha_;
    float filtered_lx_, filtered_ly_, filtered_rx_, filtered_ry_;
  
    float low_pass_filter(float new_value, float old_value) {
        return filter_alpha_ * new_value + (1.0f - filter_alpha_) * old_value;
    }
};
```

### 7.6 完整的遥控器处理节点

```cpp
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/wireless_controller.hpp"
#include "geometry_msgs/msg/twist.hpp"

class TeleopControlNode : public rclcpp::Node {
public:
    TeleopControlNode() : Node("teleop_control_node") {
        // 订阅遥控器状态
        controller_subscriber_ = this->create_subscription<unitree_go::msg::WirelessController>(
            "/wirelesscontroller", 10,
            [this](const unitree_go::msg::WirelessController::SharedPtr msg) {
                handle_controller_input(msg);
            });
  
        // 发布运动命令 (这部分在下一节详细说明)
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  
        // 定时器 - 定期检查连接状态
        status_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() { check_controller_status(); });
  
        RCLCPP_INFO(this->get_logger(), "Teleop control node initialized");
    }

private:
    void handle_controller_input(const unitree_go::msg::WirelessController::SharedPtr &msg) {
        // 更新状态管理器
        state_manager_.update_state(msg);
  
        // 滤波处理
        auto filtered = input_filter_.filter_input(*msg);
  
        // 根据当前模式处理输入
        if (state_manager_.get_current_mode() == ControllerStateManager::MANUAL_CONTROL) {
            auto [linear_x, angular_z] = state_manager_.get_motion_command();
            publish_motion_command(linear_x, angular_z);
        }
    }
  
    void publish_motion_command(float linear_x, float angular_z) {
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = linear_x;
        twist_msg.angular.z = angular_z;
  
        cmd_publisher_->publish(twist_msg);
  
        RCLCPP_DEBUG(this->get_logger(),
                    "Published cmd_vel: linear=%.3f, angular=%.3f",
                    linear_x, angular_z);
    }
  
    void check_controller_status() {
        if (!state_manager_.is_controller_active()) {
            // 控制器失联，发布零速度命令
            publish_motion_command(0.0f, 0.0f);
        }
    }
  
    // ROS2组件
    rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr controller_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    rclcpp::TimerBase::SharedPtr status_timer_;
  
    // 状态管理和滤波
    ControllerStateManager state_manager_;
    ControllerInputFilter input_filter_;
};
```

---

## 8. 机器人运动控制命令介绍

### 8.1 控制架构概述

Go2机器人采用请求-响应模式进行运动控制，通过订阅 `/api/sport/request`话题并发送 `unitree_api::msg::Request`消息实现高层运动控制。系统使用SportClient类封装所有运动指令的生成和发送。

```mermaid
flowchart LR
    A[控制节点] --> B[SportClient]
    B --> C[Request消息]
    C --> D["/api/sport/request"]
    D --> E[Go2机器人]
    E --> F["/api/sport/response"]
    F --> A
```

### 8.2 Request消息结构详解

```cpp
// unitree_api::msg::Request 消息结构
struct Request {
    RequestHeader header;             // 消息头
    std::string parameter;            // JSON格式参数字符串
    std::vector<uint8_t> binary;      // 二进制数据
};

// RequestHeader结构
struct RequestHeader {
    RequestIdentity identity;         // API身份标识
    RequestLease lease;              // 租约信息
    RequestPolicy policy;            // 策略配置
};

// RequestIdentity结构
struct RequestIdentity {
    int64_t id;                      // 请求ID
    int64_t api_id;                  // API标识符
};

// RequestLease结构  
struct RequestLease {
    int64_t id;                      // 租约ID
};

// RequestPolicy结构
struct RequestPolicy {
    int32_t priority;                // 优先级
    bool noreply;                    // 是否需要回复
};
```

### 8.3 Response消息结构详解

```cpp
// unitree_api::msg::Response 消息结构
struct Response {
    ResponseHeader header;           // 响应消息头
    std::string data;               // JSON格式响应数据
    std::vector<int8_t> binary;     // 二进制响应数据
};

// ResponseHeader结构
struct ResponseHeader {
    RequestIdentity identity;        // 对应请求的身份标识
    ResponseStatus status;           // 响应状态
};

// ResponseStatus结构
struct ResponseStatus {
    int32_t code;                   // 状态码
};
```

### 8.4 SportClient类详解

`SportClient`是Go2运动控制的核心类，封装了所有运动指令的生成和发送。基于源码分析，其工作原理如下：

```cpp
class SportClient {
public:
    SportClient(rclcpp::Node* node) {
        // 创建请求发布者
        req_puber_ = node->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
    }
  
    // 基础运动控制方法
    void Damp(unitree_api::msg::Request &req);
    void BalanceStand(unitree_api::msg::Request &req);
    void Move(unitree_api::msg::Request &req, float vx, float vy, float vyaw);
    void Euler(unitree_api::msg::Request &req, float roll, float pitch, float yaw);
    // ... 其他方法

private:
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber_;
};
```

#### 8.4.1 无参数指令实现模式

对于不需要参数的简单指令，SportClient直接设置API ID并发布：

```cpp
void SportClient::Damp(unitree_api::msg::Request &req) {
    req.header.identity.api_id = ROBOT_SPORT_API_ID_DAMP;
    req_puber_->publish(req);
}

void SportClient::BalanceStand(unitree_api::msg::Request &req) {
    req.header.identity.api_id = ROBOT_SPORT_API_ID_BALANCESTAND;
    req_puber_->publish(req);
}

void SportClient::StandUp(unitree_api::msg::Request &req) {
    req.header.identity.api_id = ROBOT_SPORT_API_ID_STANDUP;
    req_puber_->publish(req);
}
```

#### 8.4.2 带参数指令实现模式

对于需要参数的指令，SportClient使用nlohmann::json库将参数序列化为JSON字符串：

```cpp
void SportClient::Move(unitree_api::msg::Request &req, float vx, float vy, float vyaw) {
    nlohmann::json js;
    js["x"] = vx;
    js["y"] = vy; 
    js["z"] = vyaw;
    req.parameter = js.dump();
    req.header.identity.api_id = ROBOT_SPORT_API_ID_MOVE;
    req_puber_->publish(req);
}

void SportClient::Euler(unitree_api::msg::Request &req, float roll, float pitch, float yaw) {
    nlohmann::json js;
    js["x"] = roll;
    js["y"] = pitch;
    js["z"] = yaw;
    req.parameter = js.dump();
    req.header.identity.api_id = ROBOT_SPORT_API_ID_EULER;
    req_puber_->publish(req);
}

void SportClient::SpeedLevel(unitree_api::msg::Request &req, int level) {
    nlohmann::json js;
    js["data"] = level;
    req.parameter = js.dump();
    req.header.identity.api_id = ROBOT_SPORT_API_ID_SPEEDLEVEL;
    req_puber_->publish(req);
}
```

#### 8.4.3 布尔参数指令实现模式

对于开关类型的指令，参数通过"data"字段传递：

```cpp
void SportClient::SwitchJoystick(unitree_api::msg::Request &req, bool flag) {
    nlohmann::json js;
    js["data"] = flag;
    req.parameter = js.dump();
    req.header.identity.api_id = ROBOT_SPORT_API_ID_SWITCHJOYSTICK;
    req_puber_->publish(req);
}

void SportClient::HandStand(unitree_api::msg::Request &req, bool flag) {
    nlohmann::json js;
    js["data"] = flag;
    req.parameter = js.dump();
    req.header.identity.api_id = ROBOT_SPORT_API_ID_HANDSTAND;
    req_puber_->publish(req);
}
```

### 8.5 运动控制节点实现

基于SportClient的正确使用方式，完整的运动控制节点实现如下：

```cpp
#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ros2_sport_client.h"

class Go2MotionController : public rclcpp::Node {
public:
    Go2MotionController() : Node("go2_motion_controller") {
        // 创建请求发布者
        req_publisher_ = this->create_publisher<unitree_api::msg::Request>(
            "/api/sport/request", 10);
  
        // 创建响应订阅者
        resp_subscriber_ = this->create_subscription<unitree_api::msg::Response>(
            "/api/sport/response", 10,
            [this](const unitree_api::msg::Response::SharedPtr msg) {
                handle_response(msg);
            });
  
        // 订阅Twist消息进行速度控制
        cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                handle_velocity_command(msg);
            });
  
        // 初始化SportClient，传入节点指针
        sport_client_ = std::make_unique<SportClient>(this);
  
        RCLCPP_INFO(this->get_logger(), "Go2 Motion Controller initialized");
  
        // 初始化机器人到站立状态
        initialize_robot();
    }

private:
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_publisher_;
    rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr resp_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    std::unique_ptr<SportClient> sport_client_;
  
    void initialize_robot() {
        // 等待一段时间让系统初始化
        rclcpp::sleep_for(std::chrono::seconds(2));
  
        // 创建请求消息
        unitree_api::msg::Request req;
  
        // 站起机器人
        sport_client_->StandUp(req);
        RCLCPP_INFO(this->get_logger(), "Command sent: StandUp");
  
        // 等待站起完成后切换到平衡站立
        rclcpp::sleep_for(std::chrono::seconds(3));
        sport_client_->BalanceStand(req);
        RCLCPP_INFO(this->get_logger(), "Command sent: BalanceStand");
    }
  
    void handle_velocity_command(const geometry_msgs::msg::Twist::SharedPtr &msg) {
        float vx = msg->linear.x;
        float vy = msg->linear.y;
        float vyaw = msg->angular.z;
  
        // 速度限制
        vx = std::clamp(vx, -2.0f, 2.0f);
        vy = std::clamp(vy, -1.0f, 1.0f);
        vyaw = std::clamp(vyaw, -2.0f, 2.0f);
  
        // 创建请求消息并发送运动命令
        unitree_api::msg::Request req;
        sport_client_->Move(req, vx, vy, vyaw);
  
        RCLCPP_DEBUG(this->get_logger(),
                    "Motion command sent: vx=%.3f, vy=%.3f, vyaw=%.3f",
                    vx, vy, vyaw);
    }
  
    void handle_response(const unitree_api::msg::Response::SharedPtr &msg) {
        // 处理机器人响应
        RCLCPP_DEBUG(this->get_logger(), 
                    "Received response for API ID: %ld, Status: %d", 
                    msg->header.identity.api_id, msg->header.status.code);
  
        // 根据状态码处理不同情况
        if (msg->header.status.code != 0) {
            RCLCPP_WARN(this->get_logger(),
                       "Command failed with status code: %d", 
                       msg->header.status.code);
        }
    }
};
```

### 8.6 实际使用示例

基于源码的正确实现方式，以下是一个完整的姿态控制示例：

```cpp
// 创建运动控制节点
class PoseControlExample : public rclcpp::Node {
public:
    PoseControlExample() : Node("pose_control_example") {
        // 初始化SportClient
        sport_client_ = std::make_unique<SportClient>(this);
  
        // 创建定时器，定期执行姿态控制
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() { execute_pose_control(); });
    }
  
private:
    std::unique_ptr<SportClient> sport_client_;
    rclcpp::TimerBase::SharedPtr control_timer_;
  
    void execute_pose_control() {
        static float phase = 0.0f;
        phase += 0.1f;
  
        // 计算正弦波姿态
        float roll = 0.2f * sin(phase);
        float pitch = 0.1f * cos(phase);
        float yaw = 0.0f;
  
        // 创建请求并发送欧拉角控制命令
        unitree_api::msg::Request req;
        sport_client_->Euler(req, roll, pitch, yaw);
  
        RCLCPP_DEBUG(this->get_logger(),
                    "Pose command: roll=%.3f, pitch=%.3f, yaw=%.3f",
                    roll, pitch, yaw);
    }
};
```

---

## 9. 机器人基础运动控制API

### 9.1 基本姿态控制

#### 9.1.1 站立控制

```cpp
class BasicMotionController {
public:
    BasicMotionController(rclcpp::Node* node) : sport_client_(node) {}
  
    // 平衡站立 - API ID: 1002
    void balance_stand() {
        unitree_api::msg::Request req;
        sport_client_.BalanceStand(req);
        RCLCPP_INFO(rclcpp::get_logger("motion"), "Command: Balance Stand");
    }
  
    // 站起 - API ID: 1004  
    void stand_up() {
        unitree_api::msg::Request req;
        sport_client_.StandUp(req);
        RCLCPP_INFO(rclcpp::get_logger("motion"), "Command: Stand Up");
    }
  
    // 趴下 - API ID: 1005
    void stand_down() {
        unitree_api::msg::Request req;
        sport_client_.StandDown(req);
        RCLCPP_INFO(rclcpp::get_logger("motion"), "Command: Stand Down");
    }
  
    // 恢复站立 - API ID: 1006
    void recovery_stand() {
        unitree_api::msg::Request req;
        sport_client_.RecoveryStand(req);
        RCLCPP_INFO(rclcpp::get_logger("motion"), "Command: Recovery Stand");
    }
  
    // 坐下 - API ID: 1009
    void sit() {
        unitree_api::msg::Request req;
        sport_client_.Sit(req);
        RCLCPP_INFO(rclcpp::get_logger("motion"), "Command: Sit");
    }
  
    // 从坐姿起立 - API ID: 1010
    void rise_sit() {
        unitree_api::msg::Request req;
        sport_client_.RiseSit(req);
        RCLCPP_INFO(rclcpp::get_logger("motion"), "Command: Rise from Sit");
    }

private:
    SportClient sport_client_;
};
```

#### 9.1.2 紧急控制

```cpp
// 阻尼模式 - API ID: 1001
void enable_damp_mode() {
    unitree_api::msg::Request req;
    sport_client_.Damp(req);
    RCLCPP_WARN(rclcpp::get_logger("motion"), "Command: Damp Mode Enabled");
}

// 停止移动 - API ID: 1003
void stop_move() {
    unitree_api::msg::Request req;
    sport_client_.StopMove(req);
    RCLCPP_INFO(rclcpp::get_logger("motion"), "Command: Stop Move");
}
```

### 9.2 运动控制API

#### 9.2.1 速度控制

```cpp
// 移动控制 - API ID: 1008
void move_robot(float vx, float vy, float vyaw) {
    unitree_api::msg::Request req;
    sport_client_.Move(req, vx, vy, vyaw);
  
    RCLCPP_DEBUG(rclcpp::get_logger("motion"),
                "Command: Move - vx=%.3f, vy=%.3f, vyaw=%.3f",
                vx, vy, vyaw);
}

// 速度等级设置 - API ID: 1015
void set_speed_level(int level) {
    if (level < 1 || level > 5) {
        RCLCPP_ERROR(rclcpp::get_logger("motion"),
                    "Invalid speed level: %d (valid range: 1-5)", level);
        return;
    }
  
    unitree_api::msg::Request req;
    sport_client_.SpeedLevel(req, level);
  
    RCLCPP_INFO(rclcpp::get_logger("motion"),
               "Command: Speed Level = %d", level);
}
```

#### 9.2.2 姿态控制

```cpp
// 欧拉角姿态控制 - API ID: 1007
void set_euler_angles(float roll, float pitch, float yaw) {
    // 角度范围检查
    if (abs(roll) > 0.5 || abs(pitch) > 0.5 || abs(yaw) > M_PI) {
        RCLCPP_ERROR(rclcpp::get_logger("motion"),
                    "Euler angles out of safe range!");
        return;
    }
  
    unitree_api::msg::Request req;
    sport_client_.Euler(req, roll, pitch, yaw);
  
    RCLCPP_INFO(rclcpp::get_logger("motion"),
               "Command: Euler - roll=%.3f, pitch=%.3f, yaw=%.3f",
               roll, pitch, yaw);
}

// 姿态模式开关 - API ID: 1028
void enable_pose_mode(bool enable) {
    unitree_api::msg::Request req;
    sport_client_.Pose(req, enable);
  
    RCLCPP_INFO(rclcpp::get_logger("motion"),
               "Command: Pose Mode %s", enable ? "Enabled" : "Disabled");
}
```

---

## 10. 机器人步态控制API

### 10.1 基础步态

```cpp
class GaitController {
public:
    GaitController(rclcpp::Node* node) : sport_client_(node) {}
  
    // 静态行走 - API ID: 1061
    void static_walk() {
        unitree_api::msg::Request req;
        sport_client_.StaticWalk(req);
        RCLCPP_INFO(rclcpp::get_logger("gait"), "Gait: Static Walk");
    }
  
    // 小跑步态 - API ID: 1062  
    void trot_run() {
        unitree_api::msg::Request req;
        sport_client_.TrotRun(req);
        RCLCPP_INFO(rclcpp::get_logger("gait"), "Gait: Trot Run");
    }
  
    // 节能步态 - API ID: 1063
    void economic_gait() {
        unitree_api::msg::Request req;
        sport_client_.EconomicGait(req);
        RCLCPP_INFO(rclcpp::get_logger("gait"), "Gait: Economic");
    }

private:
    SportClient sport_client_;
};
```

### 10.2 高级步态

```cpp
// 自由行走 - API ID: 2045
void free_walk() {
    unitree_api::msg::Request req;
    sport_client_.FreeWalk(req);
    RCLCPP_INFO(rclcpp::get_logger("gait"), "Gait: Free Walk");
}

// 经典行走 - API ID: 2049
void classic_walk(bool enable) {
    unitree_api::msg::Request req;
    sport_client_.ClassicWalk(req, enable);
    RCLCPP_INFO(rclcpp::get_logger("gait"), 
               "Gait: Classic Walk %s", enable ? "Enabled" : "Disabled");
}

// 直立行走 - API ID: 2050
void walk_upright(bool enable) {
    unitree_api::msg::Request req;
    sport_client_.WalkUpright(req, enable);
    RCLCPP_INFO(rclcpp::get_logger("gait"),
               "Gait: Walk Upright %s", enable ? "Enabled" : "Disabled");
}

// 交叉步态 - API ID: 2051  
void cross_step(bool enable) {
    unitree_api::msg::Request req;
    sport_client_.CrossStep(req, enable);
    RCLCPP_INFO(rclcpp::get_logger("gait"),
               "Gait: Cross Step %s", enable ? "Enabled" : "Disabled");
}
```

---

## 11. 机器人动作表演API

### 11.1 特技动作

```cpp
class StuntController {
public:
    StuntController(rclcpp::Node* node) : sport_client_(node) {}
  
    // 前空翻 - API ID: 1030
    void front_flip() {
        unitree_api::msg::Request req;
        sport_client_.FrontFlip(req);
        RCLCPP_INFO(rclcpp::get_logger("stunt"), "Performing: Front Flip");
    }
  
    // 后空翻 - API ID: 2043
    void back_flip() {
        unitree_api::msg::Request req;
        sport_client_.BackFlip(req);  
        RCLCPP_INFO(rclcpp::get_logger("stunt"), "Performing: Back Flip");
    }
  
    // 左空翻 - API ID: 2041
    void left_flip() {
        unitree_api::msg::Request req;
        sport_client_.LeftFlip(req);
        RCLCPP_INFO(rclcpp::get_logger("stunt"), "Performing: Left Flip");
    }
  
    // 前跳 - API ID: 1031
    void front_jump() {
        unitree_api::msg::Request req;
        sport_client_.FrontJump(req);
        RCLCPP_INFO(rclcpp::get_logger("stunt"), "Performing: Front Jump");
    }
  
    // 前扑 - API ID: 1032
    void front_pounce() {
        unitree_api::msg::Request req;
        sport_client_.FrontPounce(req);
        RCLCPP_INFO(rclcpp::get_logger("stunt"), "Performing: Front Pounce");
    }
  
    // 倒立 - API ID: 2044
    void hand_stand(bool enable) {
        unitree_api::msg::Request req;
        sport_client_.HandStand(req, enable);
        RCLCPP_INFO(rclcpp::get_logger("stunt"),
                   "Hand Stand %s", enable ? "Started" : "Stopped");
    }

private:
    SportClient sport_client_;
};
```

### 11.2 舞蹈和表情

```cpp
class EntertainmentController {
public:
    EntertainmentController(rclcpp::Node* node) : sport_client_(node) {}
  
    // 舞蹈1 - API ID: 1022
    void dance1() {
        unitree_api::msg::Request req;
        sport_client_.Dance1(req);
        RCLCPP_INFO(rclcpp::get_logger("entertainment"), "Performing: Dance 1");
    }
  
    // 舞蹈2 - API ID: 1023  
    void dance2() {
        unitree_api::msg::Request req;
        sport_client_.Dance2(req);
        RCLCPP_INFO(rclcpp::get_logger("entertainment"), "Performing: Dance 2");
    }
  
    // 打招呼 - API ID: 1016
    void hello() {
        unitree_api::msg::Request req;
        sport_client_.Hello(req);
        RCLCPP_INFO(rclcpp::get_logger("entertainment"), "Performing: Hello");
    }
  
    // 伸展 - API ID: 1017
    void stretch() {
        unitree_api::msg::Request req;
        sport_client_.Stretch(req);
        RCLCPP_INFO(rclcpp::get_logger("entertainment"), "Performing: Stretch");
    }
  
    // 心形手势 - API ID: 1036
    void heart_gesture() {
        unitree_api::msg::Request req;
        sport_client_.Heart(req);
        RCLCPP_INFO(rclcpp::get_logger("entertainment"), "Performing: Heart");
    }
  
    // 满足表情 - API ID: 1020
    void content_expression() {
        unitree_api::msg::Request req;
        sport_client_.Content(req);
        RCLCPP_INFO(rclcpp::get_logger("entertainment"), "Expression: Content");
    }
  
    // 刨地动作 - API ID: 1029
    void scrape() {
        unitree_api::msg::Request req;
        sport_client_.Scrape(req);
        RCLCPP_INFO(rclcpp::get_logger("entertainment"), "Performing: Scrape");
    }

private:
    SportClient sport_client_;
};
```

---

## 12. 机器人系统配置API

### 12.1 控制器配置

```cpp
class SystemController {
public:
    SystemController(rclcpp::Node* node) : sport_client_(node) {}
  
    // 遥控器开关 - API ID: 1027
    void switch_joystick(bool enable) {
        unitree_api::msg::Request req;
        sport_client_.SwitchJoystick(req, enable);
        RCLCPP_INFO(rclcpp::get_logger("system"),
                   "Joystick %s", enable ? "Enabled" : "Disabled");
    }
  
    // 自动恢复设置 - API ID: 2054
    void set_auto_recovery(bool enable) {
        unitree_api::msg::Request req;
        sport_client_.AutoRecoverySet(req, enable);
        RCLCPP_INFO(rclcpp::get_logger("system"),
                   "Auto Recovery %s", enable ? "Enabled" : "Disabled");
    }
  
    // 获取自动恢复状态 - API ID: 2055
    bool get_auto_recovery() {
        unitree_api::msg::Request req;
        bool status = false;
        sport_client_.AutoRecoveryGet(req, status);
        RCLCPP_INFO(rclcpp::get_logger("system"),
                   "Auto Recovery Status: %s", status ? "Enabled" : "Disabled");
        return status;
    }
  
    // 切换避障模式 - API ID: 2058
    void switch_avoid_mode() {
        unitree_api::msg::Request req;
        sport_client_.SwitchAvoidMode(req);
        RCLCPP_INFO(rclcpp::get_logger("system"), "Switched Avoid Mode");
    }

private:
    SportClient sport_client_;
};
```

### 12.2 高级运动模式

```cpp
// 自由跳跃 - API ID: 2047
void free_jump(bool enable) {
    unitree_api::msg::Request req;
    sport_client_.FreeJump(req, enable);
    RCLCPP_INFO(rclcpp::get_logger("advanced"),
               "Free Jump %s", enable ? "Enabled" : "Disabled");
}

// 自由避障 - API ID: 2048  
void free_avoid(bool enable) {
    unitree_api::msg::Request req;
    sport_client_.FreeAvoid(req, enable);
    RCLCPP_INFO(rclcpp::get_logger("advanced"),
               "Free Avoid %s", enable ? "Enabled" : "Disabled");
}

// 自由奔跑 - API ID: 2046
void free_bound(bool enable) {
    unitree_api::msg::Request req;
    sport_client_.FreeBound(req, enable);
    RCLCPP_INFO(rclcpp::get_logger("advanced"),
               "Free Bound %s", enable ? "Enabled" : "Disabled");
}
```

---

## 13. API ID速查表

### 13.1 基础控制API

| API ID | 功能          | 参数           | 描述         |
| ------ | ------------- | -------------- | ------------ |
| 1001   | Damp          | 无             | 阻尼模式     |
| 1002   | BalanceStand  | 无             | 平衡站立     |
| 1003   | StopMove      | 无             | 停止移动     |
| 1004   | StandUp       | 无             | 站起         |
| 1005   | StandDown     | 无             | 趴下         |
| 1006   | RecoveryStand | 无             | 恢复站立     |
| 1007   | Euler         | roll,pitch,yaw | 欧拉角控制   |
| 1008   | Move          | vx,vy,vyaw     | 运动控制     |
| 1009   | Sit           | 无             | 坐下         |
| 1010   | RiseSit       | 无             | 从坐姿起立   |
| 1015   | SpeedLevel    | level          | 速度等级设置 |

### 13.2 表演动作API

| API ID | 功能        | 参数 | 描述     |
| ------ | ----------- | ---- | -------- |
| 1016   | Hello       | 无   | 打招呼   |
| 1017   | Stretch     | 无   | 伸展     |
| 1020   | Content     | 无   | 满足表情 |
| 1022   | Dance1      | 无   | 舞蹈1    |
| 1023   | Dance2      | 无   | 舞蹈2    |
| 1029   | Scrape      | 无   | 刨地     |
| 1030   | FrontFlip   | 无   | 前空翻   |
| 1031   | FrontJump   | 无   | 前跳     |
| 1032   | FrontPounce | 无   | 前扑     |
| 1036   | Heart       | 无   | 心形手势 |

### 13.3 高级功能API

| API ID | 功能            | 参数 | 描述             |
| ------ | --------------- | ---- | ---------------- |
| 2041   | LeftFlip        | 无   | 左空翻           |
| 2043   | BackFlip        | 无   | 后空翻           |
| 2044   | HandStand       | flag | 倒立             |
| 2045   | FreeWalk        | 无   | 自由行走         |
| 2046   | FreeBound       | flag | 自由奔跑         |
| 2047   | FreeJump        | flag | 自由跳跃         |
| 2048   | FreeAvoid       | flag | 自由避障         |
| 2049   | ClassicWalk     | flag | 经典行走         |
| 2050   | WalkUpright     | flag | 直立行走         |
| 2051   | CrossStep       | flag | 交叉步态         |
| 2054   | AutoRecoverySet | flag | 自动恢复设置     |
| 2055   | AutoRecoveryGet | 无   | 获取自动恢复状态 |
| 2058   | SwitchAvoidMode | 无   | 切换避障模式     |
