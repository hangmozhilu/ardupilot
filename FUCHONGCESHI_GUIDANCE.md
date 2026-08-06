俯冲测试模式说明：
已更新 [mode_fuchongceshi.cpp](file:///d:/APM/ardupilot/ArduPlane/mode_fuchongceshi.cpp) 中像素归一化注释，并整理了完整制导算法与变量清单。

## 坐标系与像素归一化

外部 AI 模块已将像素坐标系原点设置在画面中心，传入的 `Camera_x`、`Camera_y` 为**相对于画面中心的像素偏移**：

```cpp
// 像素坐标系原点位于画面中心
// AI模块输出的是相对于画面中心的像素偏移，无需再减 half_width/half_height
// Pixel coordinate origin is at image center. The AI module sends pixel offsets
// relative to center, so no subtraction of half width/height is needed.
const float nx = target.camera_x / (CAMERA_WIDTH_PX * 0.5f);   // [-1, 1], 右正
const float ny = target.camera_y / (CAMERA_HEIGHT_PX * 0.5f); // [-1, 1], 上正
```

## FUCHONGCESHI 模式制导算法总览

### 1. 输入数据

| 变量 | 来源 | 说明 |
|------|------|------|
| `target.camera_x` | 串口 | 目标相对画面中心的水平像素偏移，右正 |
| `target.camera_y` | 串口 | 目标相对画面中心的垂直像素偏移，上正 |
| `target.gimbal_yaw_cdeg` | 串口 | 云台偏航框架角，0.01°，右正 |
| `target.gimbal_pitch_cdeg` | 串口 | 云台俯仰框架角，0.01°，抬头正 |
| `target.confidence` | 串口 | 目标置信度，0.0~1.0 |
| `target.object_active` | 串口 | 追踪标志：0x11不追踪，0x01/0x22追踪 |

### 2. 关键参数

| 参数 | 符号 | 默认值 | 作用 |
|------|------|--------|------|
| `GA_HFOV` | `hfov` | 60° | 相机水平视场角 |
| `GA_VFOV` | `vfov` | 45° | 相机垂直视场角 |
| `GA_DIVE_PITCH` | `dive_pitch` | -18° | 后备固定俯冲角 |
| `GA_ROLL_LIM` | `roll_lim` | 45° | 滚转限制 |
| `GA_PITCH_MIN/MAX` | `pitch_min/max` | -60°/+10° | 俯仰限制 |
| `GA_KP_ROLL` | `kp_roll` | 1.0 | 滚转P增益 |
| `GA_KD_ROLL` | `kd_roll` | 0.5 | 滚转D增益 |
| `GA_PNG_N` | `png_n` | 3.0 | 比例导航增益 |
| `GA_KP_PITCH` | `kp_pitch` | 1.0 | 俯仰P增益 |
| `GA_TGT_ALT` | `tgt_alt` | 0 | 目标海拔高度 (m) |
| `GA_GSC_RNG` | `gain_sched_range` | 200m | 增益调度参考距离 |
| `GA_TERM_RNG` | `terminal_range` | 50m | 终端制导切换距离 |
| `GA_RUD_MIX` | `rudder_mix` | 0.0 | 方向舵混合增益 |
| `GA_TIMEOUT_MS` | `timeout_ms` | 2000ms | 目标丢失超时 |
| `GA_LOSS_ACT` | `loss_action` | 0 | 丢失动作 |

### 3. 制导流程

```
读取串口帧 → 解析目标 → 检查自动切换条件
    ↓
[目标有效?] 否 → handle_target_loss()
    ↓ 是
像素归一化 (nx, ny)
    ↓
滚转去旋转：R(-roll) * [nx; ny]
    ↓
像素角计算：atan(nx * tan(hfov/2)), atan(ny * tan(vfov/2))
    ↓
体轴LOS角：los_yaw = gimbal_yaw + pixel_yaw
           los_pitch = gimbal_pitch + pixel_pitch
    ↓
Alpha-Beta滤波：filtered_bearing, filtered_elevation, filtered_range
                 并估计角速率
    ↓
杆臂补偿：扣除云台安装位置带来的虚假LOS运动
    ↓
角速率去陀螺耦合：true_bearing_rate = filtered_bearing_rate + gyro.z
    ↓
[slant_range >= GA_TERM_RNG?] 否 → 终端纯追踪制导
    ↓ 是
远距离PNG制导 + 增益调度
    ↓
姿态限幅：nav_roll_cd, nav_pitch_cd
    ↓
油门 = THR_MAX，方向舵 = roll_norm * GA_RUD_MIX
```

### 4. 控制律

**远距离比例导航制导：**
```cpp
true_bearing_rate = filtered_bearing_rate + ahrs.get_gyro().z;
roll_cmd = gain_scale * (kp_roll * bearing_error
                        + kd_roll * true_bearing_rate
                        + png_n   * true_bearing_rate);
pitch_cmd = compute_adaptive_dive_pitch(filtered_range, los_pitch_earth);
```

**终端纯追踪制导：**
```cpp
roll_cmd = 0.5f * bearing_error;  // 限幅 ±15°
pitch_cmd = compute_adaptive_dive_pitch(filtered_range, los_pitch_earth);
```

**自适应俯冲角：**
```cpp
depression_angle = -los_pitch_earth;  // 正值=向下看
horizontal_dist = range * cos(depression_angle);
height_above_target = current_AMSL - GA_TGT_ALT;
adaptive_dive = atan2(-height_above_target, horizontal_dist);
```

### 5. 输出

| 输出 | 变量 | 说明 |
|------|------|------|
| 滚转指令 | `plane.nav_roll_cd` | centideg |
| 俯仰指令 | `plane.nav_pitch_cd` | centideg |
| 油门 | `plane.throttle` | 最大油门 |
| 方向舵 | `output_rudder_and_steering()` | 归一化 ±1 |

### 6. 安全机制

- 自动模式切换：`object_active != 0x11 && confidence >= 0.7`
- 模式内目标有效：`object_active != 0x11 && confidence >= 0.3`
- 目标丢失：平飞姿态，按 `GA_LOSS_ACT` 执行
- 遥控器超控：滚转/俯仰摇杆 >15% 时切换手动控制
