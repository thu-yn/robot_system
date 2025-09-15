# Unitree ROS2 消息汇总

本文档汇总了 `src/third_party/go_ros2/cyclonedds_ws/src/unitree` 目录下定义的所有ROS2消息类型，包括 `unitree_api`、`unitree_go` 和 `unitree_hg` 三个包。

---

## 1. `unitree_api`

此包定义了与机器人进行高级交互的通用API请求和响应消息。

### `unitree_api/msg/Request.msg`

通用API请求消息。

```
RequestHeader header
string parameter
uint8[] binary
```

### `unitree_api/msg/RequestHeader.msg`

请求消息头。

```
RequestIdentity identity
RequestLease lease
RequestPolicy policy
```

### `unitree_api/msg/RequestIdentity.msg`

请求身份标识。

```
int64  id
int64 api_id
```

### `unitree_api/msg/RequestLease.msg`

请求租约信息。

```
int64 id
```

### `unitree_api/msg/RequestPolicy.msg`

请求策略。

```
int32 priority
bool noreply
```

### `unitree_api/msg/Response.msg`

通用API响应消息。

```
ResponseHeader header
string data
int8[] binary
```

### `unitree_api/msg/ResponseHeader.msg`

响应消息头。

```
RequestIdentity identity
ResponseStatus status
```

### `unitree_api/msg/ResponseStatus.msg`

响应状态码。

```
int32 code
```

---

## 2. `unitree_go`

此包定义了专用于Go2机器人的消息。

### `unitree_go/msg/AudioData.msg`

音频数据。

```
uint64 time_frame
uint8[] data
```

### `unitree_go/msg/BmsCmd.msg`

电池管理系统（BMS）指令。

```
uint8 off
uint8[3] reserve
```

### `unitree_go/msg/BmsState.msg`

BMS状态。

```
uint8 version_high
uint8 version_low
uint8 status
uint8 soc
int32 current
uint16 cycle
int8[2] bq_ntc
int8[2] mcu_ntc
uint16[15] cell_vol
```

### `unitree_go/msg/Error.msg`

错误信息。

```
uint32 source
uint32 state
```

### `unitree_go/msg/Go2FrontVideoData.msg`

Go2前置摄像头视频数据。

```
uint64 time_frame
uint8[] video720p
uint8[] video360p
uint8[] video180p 
```

### `unitree_go/msg/HeightMap.msg`

地形高度图。

```
# Header
float64 stamp         # timestamp
string frame_id      # world frame id

# Map info
float32 resolution     # The map resolution [m/cell]
uint32 width  # Map width along x-axis [cells]
uint32 height # Map height alonge y-axis [cells]
float32[2] origin      # Map frame origin xy-position [m], the xyz-axis direction of map frame is aligned with the world frame

# Map data, in x-major order, starting with [0,0], ending with [width, height]
# For a cell whose 2d-array-index is [ix, iy]，
#    its position in world frame is: [ix * resolution + origin[0], iy * resolution + origin[1]]
#    its cell value is: data[width * iy + ix]
float32[] data
```

### `unitree_go/msg/IMUState.msg`

IMU状态。

```
float32[4] quaternion
float32[3] gyroscope
float32[3] accelerometer
float32[3] rpy
int8 temperature
```

### `unitree_go/msg/InterfaceConfig.msg`

接口配置。

```
uint8 mode
uint8 value
uint8[2] reserve
```

### `unitree_go/msg/LidarState.msg`

激光雷达状态。

```
float64 stamp
string firmware_version
string software_version
string sdk_version
float32 sys_rotation_speed
float32 com_rotation_speed
uint8 error_state
float32 cloud_frequency
float32 cloud_packet_loss_rate
uint32 cloud_size
uint32 cloud_scan_num
float32 imu_frequency
float32 imu_packet_loss_rate
float32[3] imu_rpy
float64 serial_recv_stamp
uint32 serial_buffer_size
uint32 serial_buffer_read
```

### `unitree_go/msg/LowCmd.msg`

底层控制指令。

```
uint8[2] head
uint8 level_flag
uint8 frame_reserve
uint32[2] sn
uint32[2] version
uint16 bandwidth
MotorCmd[20] motor_cmd
BmsCmd bms_cmd
uint8[40] wireless_remote
uint8[12] led
uint8[2] fan
uint8 gpio
uint32 reserve
uint32 crc
```

### `unitree_go/msg/LowState.msg`

底层状态信息。

```
uint8[2] head
uint8 level_flag
uint8 frame_reserve
uint32[2] sn
uint32[2] version
uint16 bandwidth
IMUState imu_state
MotorState[20] motor_state
BmsState bms_state
int16[4] foot_force
int16[4] foot_force_est
uint32 tick
uint8[40] wireless_remote
uint8 bit_flag
float32 adc_reel
int8 temperature_ntc1
int8 temperature_ntc2
float32 power_v
float32 power_a
uint16[4] fan_frequency
uint32 reserve
uint32 crc
```

### `unitree_go/msg/MotorCmd.msg`

单个电机指令。

```
uint8 mode
float32 q
float32 dq
float32 tau
float32 kp
float32 kd
uint32[3] reserve
```

### `unitree_go/msg/MotorCmds.msg`

多个电机指令集合。

```
MotorCmd[] cmds
```

### `unitree_go/msg/MotorState.msg`

单个电机状态。

```
uint8 mode
float32 q
float32 dq
float32 ddq
float32 tau_est
float32 q_raw
float32 dq_raw
float32 ddq_raw
int8 temperature
uint32 lost
uint32[2] reserve
```

### `unitree_go/msg/MotorStates.msg`

多个电机状态集合。

```
MotorState[] states
```

### `unitree_go/msg/PathPoint.msg`

路径点信息。

```
float32 t_from_start
float32 x
float32 y
float32 yaw
float32 vx
float32 vy
float32 vyaw
```

### `unitree_go/msg/Req.msg`

自定义请求消息。

```
string uuid
string body
```

### `unitree_go/msg/Res.msg`

自定义响应消息。

```
string uuid
uint8[] data
string body
```

### `unitree_go/msg/SportModeCmd.msg`

运动模式指令。

```
uint8 mode
uint8 gait_type
uint8 speed_level
float32 foot_raise_height
float32 body_height
float32[2] position
float32[3] euler
float32[2] velocity
float32 yaw_speed
BmsCmd bms_cmd
PathPoint[30] path_point
```

### `unitree_go/msg/SportModeState.msg`

运动模式状态。

```
TimeSpec stamp
uint32 error_code
IMUState imu_state
uint8 mode
float32 progress
uint8 gait_type
float32 foot_raise_height
float32[3] position
float32 body_height
float32[3] velocity
float32 yaw_speed
float32[4] range_obstacle
int16[4] foot_force
float32[12] foot_position_body
float32[12] foot_speed_body
```

### `unitree_go/msg/TimeSpec.msg`

时间戳。

```
# Time indicates a specific point in time, relative to a clock's 0 point.
# The seconds component, valid over all int32 values.
int32 sec
# The nanoseconds component, valid in the range [0, 10e9).
uint32 nanosec
```

### `unitree_go/msg/UwbState.msg`

UWB状态。

```
uint8[2] version
uint8 channel
uint8 joy_mode
float32 orientation_est
float32 pitch_est
float32 distance_est
float32 yaw_est
float32 tag_roll
float32 tag_pitch
float32 tag_yaw
float32 base_roll
float32 base_pitch
float32 base_yaw
float32[2] joystick
uint8 error_state
uint8 buttons
uint8 enabled_from_app
```

### `unitree_go/msg/UwbSwitch.msg`

UWB开关。

```
uint8 enabled
```

### `unitree_go/msg/WirelessController.msg`

无线手柄。

```
float32 lx
float32 ly
float32 rx
float32 ry
uint16 keys
```
