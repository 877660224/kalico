# Klipper 挤出压力预读控制系统

## 概述

本模块实现了基于 GCode 预读的挤出压力闭环控制系统，通过实时监测挤出压力并根据打印速度动态调整目标压力，优化打印质量。

### 主要功能

1. **GCode 预读解析** - 提前读取后续打印路径的速度参数
2. **压力-速度校准** - 建立挤出速度与压力的关系模型
3. **实时压力调整** - PID 闭环控制 + 前馈补偿
4. **反馈验证系统** - 监测偏差并动态补偿

---

## 安装配置

### 1. 硬件要求

- Klipper 固件打印机
- 压力传感器 (Load Cell) - 推荐使用 ADS1220 或 HX711
- 传感器安装：将压力传感器安装在挤出机适当位置，能够测量挤出压力

### 2. 配置示例

在 `printer.cfg` 中添加以下配置：

```ini
# ========================================
# 压力传感器配置 (Load Cell)
# ========================================
[load_cell]
sensor_type: ads1220
cs_pin: PA4
data_ready_pin: PA3
scale: 1.0
offset: 0.0

# ========================================
# 挤出压力预读控制系统
# ========================================
[extrusion_pressure_control]
# === 基本参数 ===
nozzle_diameter: 0.4              # 喷嘴直径
nozzle_length: 2.0                # 喷嘴长度
material_type: PLA                # 材料类型: PLA/PETG/ABS/TPU/NYLON

# === 压力模型参数 ===
pressure_coefficient: 1.0         # 压力系数
base_pressure: 5.0                # 基础压力
max_pressure: 50.0                # 最大压力
# 多项式系数: P = a0 + a1*v + a2*v²
poly_coef_a0: 2.0
poly_coef_a1: 0.5
poly_coef_a2: 0.01

# === 预读参数 ===
lookahead_depth: 30               # 预读深度 (移动段数量)
velocity_change_threshold: 5.0    # 速度变化阈值 (mm/s)

# === PID 控制参数 ===
pid_kp: 0.5                       # 比例系数
pid_ki: 0.1                       # 积分系数
pid_kd: 0.02                      # 微分系数

# === 控制参数 ===
target_pressure: 10.0             # 目标压力
pressure_tolerance: 0.5           # 压力容差
adjustment_method: extrude_factor # 调整方法: extrude_factor/rotation_distance
control_mode: closed_loop         # 控制模式: open_loop/closed_loop/hybrid
control_period: 0.01              # 控制周期 (秒)

# === 前馈控制 ===
use_feedforward: True             # 启用前馈控制
feedforward_gain: 0.1             # 前馈增益

# === 反馈验证 ===
history_size: 100                 # 历史记录大小
warning_threshold: 2.0            # 警告阈值
critical_threshold: 5.0           # 严重阈值
learning_rate: 0.01               # 学习率

# === 校准参数 ===
calibration_speeds: 10,20,30,40,50,60  # 校准速度列表
calibration_duration: 5.0         # 每个速度的校准持续时间
calibration_distance: 50.0        # 校准移动距离
```

---

## GCode 命令参考

### PRESSURE_CONTROL_ENABLE

启用挤出压力控制。

```gcode
PRESSURE_CONTROL_ENABLE
```

### PRESSURE_CONTROL_DISABLE

禁用挤出压力控制。

```gcode
PRESSURE_CONTROL_DISABLE
```

### SET_TARGET_PRESSURE

设置目标挤出压力。

**参数:**
- `PRESSURE`: 目标压力值

```gcode
SET_TARGET_PRESSURE PRESSURE=12.0
```

### PRESSURE_CONTROL_STATUS

获取压力控制系统状态。

```gcode
PRESSURE_CONTROL_STATUS
```

**输出示例:**
```
Extrusion Pressure Control Status:
  Enabled: True
  Control Mode: closed_loop
  Target Pressure: 10.00
  Current Pressure: 9.85
  Error: 0.15
  Extrude Factor: 1.002
  Current Velocity: 40.0 mm/s
  Current Extrude Rate: 4.00 mm/s
  Calibration Points: 6
  Trend: stable
  Mean Error: 0.125
  RMS Error: 0.180
```

### PRESSURE_CALIBRATE

启动自动压力校准程序。

```gcode
PRESSURE_CALIBRATE
```

校准过程会自动以不同速度打印，并记录对应的压力值，建立速度-压力模型。

### PRESSURE_ADD_CALIBRATION_POINT

手动添加校准点。

**参数:**
- `RATE`: 挤出速率
- `PRESSURE`: 对应压力

```gcode
PRESSURE_ADD_CALIBRATION_POINT RATE=4.0 PRESSURE=12.5
```

### PRESSURE_CLEAR_CALIBRATION

清除所有校准数据。

```gcode
PRESSURE_CLEAR_CALIBRATION
```

### SET_PRESSURE_PID

设置 PID 控制参数。

**参数:**
- `KP`: 比例系数
- `KI`: 积分系数
- `KD`: 微分系数

```gcode
SET_PRESSURE_PID KP=0.6 KI=0.12 KD=0.025
```

### SET_PRESSURE_MATERIAL

设置材料类型。

**参数:**
- `MATERIAL`: 材料类型 (PLA/PETG/ABS/TPU/NYLON)

```gcode
SET_PRESSURE_MATERIAL MATERIAL=PETG
```

### PRESSURE_RESET_STATS

重置统计数据。

```gcode
PRESSURE_RESET_STATS
```

---

## 使用流程

### 1. 首次使用 - 校准流程

```gcode
; 1. 归零并预热
G28
M140 S60
M104 S210
M190 S60
M109 S210

; 2. 压力传感器归零
LOAD_CELL_TARE

; 3. 启动自动校准
PRESSURE_CALIBRATE

; 校准完成后会自动建立速度-压力模型
```

### 2. 手动校准 (可选)

如果自动校准不适用，可以手动添加校准点：

```gcode
; 以不同速度打印并记录压力

; 速度 20mm/s
G1 X50 E5 F1200
; 等待稳定后记录
PRESSURE_ADD_CALIBRATION_POINT RATE=2.0 PRESSURE=8.5

; 速度 40mm/s
G1 X100 E10 F2400
PRESSURE_ADD_CALIBRATION_POINT RATE=4.0 PRESSURE=12.0

; 速度 60mm/s
G1 X150 E15 F3600
PRESSURE_ADD_CALIBRATION_POINT RATE=6.0 PRESSURE=15.5
```

### 3. 正常打印流程

```gcode
; 打印开始
G28
M140 S60
M104 S210
M190 S60
M109 S210

; 启用压力控制
PRESSURE_CONTROL_ENABLE
SET_TARGET_PRESSURE PRESSURE=10.0

; 开始打印
; ... GCode 打印指令 ...

; 打印结束
PRESSURE_CONTROL_DISABLE
```

### 4. 打印中动态调整

```gcode
; 第一层增加压力确保粘附
SET_TARGET_PRESSURE PRESSURE=12.0
; ... 第一层 ...

; 后续层恢复正常
SET_TARGET_PRESSURE PRESSURE=10.0
; ... 后续层 ...

; 精细特征降低压力
SET_TARGET_PRESSURE PRESSURE=8.0
; ... 精细特征 ...

; 恢复正常
SET_TARGET_PRESSURE PRESSURE=10.0
```

---

## 参数调优指南

### PID 参数调整

1. **初始设置**: 使用默认值 `KP=0.5, KI=0.1, KD=0.02`
2. **观察响应**: 使用 `PRESSURE_CONTROL_STATUS` 查看误差
3. **调整方法**:
   - 如果响应太慢 → 增加 KP
   - 如果有稳态误差 → 增加 KI
   - 如果有振荡 → 增加 KD 或减小 KP

### 前馈增益调整

前馈增益 `feedforward_gain` 决定了速度变化时的压力预调整量：

- **值太小**: 速度变化时压力响应滞后
- **值太大**: 速度变化时压力过冲
- **推荐范围**: 0.05 - 0.2

### 预读深度调整

`lookahead_depth` 决定了提前读取的移动段数量：

- **值太小**: 无法及时预测速度变化
- **值太大**: 占用更多内存，响应可能延迟
- **推荐范围**: 20 - 50

---

## 工作原理

### 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                    GCode 输入                                    │
│  G1 X100 Y100 E10 F1800                                         │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│              1. GCode 预读解析模块                               │
│  - 提取速度参数: F1800 → 30mm/s                                 │
│  - 计算挤出速率: E10/100mm × 30mm/s = 3mm/s                     │
│  - 检测速度变化并触发前馈                                        │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│              2. 压力-速度模型                                    │
│  - 查询校准表或使用理论模型                                      │
│  - 计算目标压力: P_target = f(velocity, extrude_rate)           │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│              3. 实时压力调整                                     │
│  - PID 闭环控制                                                 │
│  - 前馈补偿                                                     │
│  - 调整 extrude_factor 或 rotation_distance                     │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│              4. 反馈验证系统                                     │
│  - 读取实际压力                                                 │
│  - 计算误差并记录统计                                           │
│  - 自适应学习优化模型                                           │
└─────────────────────────────────────────────────────────────────┘
```

### 控制算法

```
目标压力计算:
  P_target = P_base + P_calibration(extrude_rate) + P_compensation

PID 控制:
  output = KP × error + KI × ∫error + KD × d(error)/dt

前馈补偿:
  feedforward = velocity_change × feedforward_gain

最终调整:
  extrude_factor = 1.0 + (output + feedforward) / 100
```

---

## 故障排除

### 问题: 压力读数为 0 或异常

**可能原因:**
1. 压力传感器未正确配置
2. 传感器未归零

**解决方法:**
```gcode
LOAD_CELL_TARE          ; 归零
LOAD_CELL_READ          ; 检查读数
```

### 问题: 压力控制不稳定

**可能原因:**
1. PID 参数不合适
2. 前馈增益过大

**解决方法:**
```gcode
; 降低前馈增益
; 在配置文件中设置 feedforward_gain: 0.05

; 调整 PID
SET_PRESSURE_PID KP=0.3 KI=0.05 KD=0.01
```

### 问题: 速度变化时压力响应滞后

**可能原因:**
1. 预读深度不足
2. 前馈增益过小

**解决方法:**
```gcode
; 在配置文件中增加预读深度
lookahead_depth: 40

; 增加前馈增益
feedforward_gain: 0.15
```

---

## API 接口

### Webhooks 查询

可以通过 Webhooks API 查询压力控制状态：

```json
{
  "method": "objects/query",
  "params": {
    "objects": {
      "extrusion_pressure_control": null
    }
  }
}
```

**响应示例:**
```json
{
  "result": {
    "status": {
      "extrusion_pressure_control": {
        "enabled": true,
        "target_pressure": 10.0,
        "current_pressure": 9.85,
        "error": 0.15,
        "extrude_factor": 1.002,
        "velocity": 40.0,
        "extrude_rate": 4.0,
        "calibration_points": 6,
        "trend": "stable"
      }
    }
  }
}
```

---

## 注意事项

1. **首次使用前必须校准** - 校准数据决定了压力控制的准确性
2. **更换材料后重新校准** - 不同材料的压力特性不同
3. **定期检查传感器** - 确保压力传感器工作正常
4. **监控误差统计** - 使用 `PRESSURE_CONTROL_STATUS` 定期检查
5. **温度影响** - 温度变化会影响材料粘度，可能需要调整目标压力

---

## 版本历史

- v1.0.0 - 初始版本
  - GCode 预读解析
  - 压力-速度校准
  - PID 闭环控制
  - 前馈补偿
  - 反馈验证系统

---

## 许可证

本模块遵循 GNU GPLv3 许可证。
