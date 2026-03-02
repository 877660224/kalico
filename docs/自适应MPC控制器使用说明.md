# 自适应MPC控制器使用说明

## 概述

自适应MPC控制器是在标准MPC控制器基础上增加了数据驱动的参数自动辨识功能。系统能够在运行过程中持续采集温度数据，评估数据质量，并周期性地优化热参数，从而实现更精确的温度控制。

### 核心特性

- **数据质量评估**：自动评估采集数据的质量等级（A/B/C/D）
- **分级存储**：按数据质量分级存储，优先使用高质量数据进行参数辨识
- **周期性更新**：可配置的参数更新间隔和触发条件
- **参数验证**：新参数需通过物理合理性验证才能应用
- **版本管理**：保留参数历史版本，支持回滚
- **平滑过渡**：参数更新采用平滑过渡策略，避免突变

## 控制器类型

系统提供三种MPC控制器类型：

### 1. 标准MPC (`mpc`)

传统的模型预测控制，使用固定参数。

### 2. 自适应MPC (`adaptive_mpc`)

具备参数自动辨识功能的MPC控制器，始终运行在自适应模式。

### 3. 管理式MPC (`managed_mpc`)

支持运行时在标准MPC和自适应MPC之间动态切换的控制器管理器。

## 配置说明

### 基础MPC参数

所有MPC控制器类型共享以下基础参数：

```ini
[pid_profile default my_heater]
control: mpc
block_heat_capacity: 15.0        # 加热块热容 (J/K)
ambient_transfer: 0.08           # 环境散热系数 (W/K)
heater_power: 40.0               # 加热器功率 (W)
sensor_responsiveness: 0.012     # 传感器响应系数
target_reach_time: 2.0           # 目标到达时间 (s)
smoothing: 0.83                  # 平滑系数
```

### 自适应MPC额外参数

使用 `adaptive_mpc` 或 `managed_mpc` 时可配置以下参数：

```ini
[pid_profile default my_heater]
control: adaptive_mpc

# 自适应功能开关
adaptive_enabled: 1              # 0=禁用, 1=启用

# 数据采集阈值
adaptive_min_data: 1000          # 触发更新所需最小数据量
adaptive_min_hq_data: 300        # 触发更新所需最小高质量数据量

# 更新策略
adaptive_interval: 600.0         # 定时更新间隔 (秒)
adaptive_max_change: 0.5         # 单次参数最大变化比例 (50%)
adaptive_rmse_threshold: 2.0     # RMSE阈值触发更新 (°C)

# 容错机制
adaptive_max_failures: 3         # 连续失败次数上限

# 参数平滑
adaptive_smoothing: 1            # 0=禁用平滑, 1=启用平滑
adaptive_smoothing_tau: 30.0     # 平滑时间常数 (秒)
```

### 管理式MPC额外参数

使用 `managed_mpc` 时可配置：

```ini
[pid_profile default my_heater]
control: managed_mpc
initial_mode: adaptive           # 初始模式: standard 或 adaptive

# 包含所有自适应MPC参数
adaptive_enabled: 1
adaptive_min_data: 1000
# ... 其他自适应参数
```

## G-code命令

### ADAPTIVE_MPC_SET

设置自适应MPC参数。

```
ADAPTIVE_MPC_SET HEATER=<加热器> [ENABLED=<0|1>] [UPDATE_INTERVAL=<秒>] [RMSE_THRESHOLD=<温度>]
```

**示例：**
```
ADAPTIVE_MPC_SET HEATER=extruder ENABLED=1
ADAPTIVE_MPC_SET HEATER=extruder UPDATE_INTERVAL=300 RMSE_THRESHOLD=1.5
```

### ADAPTIVE_MPC_STATUS

获取自适应MPC状态信息。

```
ADAPTIVE_MPC_STATUS HEATER=<加热器>
```

**输出示例：**
```
自适应MPC状态 (extruder):
  启用状态: 启用
  正在适应: 否
  数据量: A级=1250, B级=800, C级=450
  当前RMSE: 0.85°C
  参数版本: 3
  上次更新: 120秒前
  连续失败: 0次
```

### ADAPTIVE_MPC_SWITCH

切换自适应MPC的启用状态。

```
ADAPTIVE_MPC_SWITCH HEATER=<加热器> MODE=<adaptive|standard>
```

**示例：**
```
ADAPTIVE_MPC_SWITCH HEATER=extruder MODE=standard
ADAPTIVE_MPC_SWITCH HEATER=extruder MODE=adaptive
```

### MPC_SWITCH

在标准MPC和自适应MPC之间切换（仅 `managed_mpc` 类型）。

```
MPC_SWITCH HEATER=<加热器> MODE=<standard|adaptive> [STATE_TRANSFER=<0|1>]
```

**参数说明：**
- `MODE`: 目标模式，`standard` 或 `adaptive`
- `STATE_TRANSFER`: 是否传递状态（默认1）

**示例：**
```
MPC_SWITCH HEATER=extruder MODE=adaptive
MPC_SWITCH HEATER=extruder MODE=standard STATE_TRANSFER=1
```

### MPC_MODE

获取或设置当前MPC模式（仅 `managed_mpc` 类型）。

```
MPC_MODE HEATER=<加热器> [MODE=<standard|adaptive>]
```

**示例：**
```
; 查询当前模式
MPC_MODE HEATER=extruder

; 设置模式
MPC_MODE HEATER=extruder MODE=adaptive
```

### MPC_MANAGER_STATUS

获取MPC管理器状态（仅 `managed_mpc` 类型）。

```
MPC_MANAGER_STATUS HEATER=<加热器>
```

**输出示例：**
```
MPC控制器管理器状态 (extruder):
  当前模式: adaptive
  活动控制器: adaptive_mpc
  切换次数: 2
  状态传递: 启用
  切换历史:
    - standard -> adaptive: 成功, 2.35ms
    - adaptive -> standard: 成功, 1.89ms
```

## 数据质量评估

系统根据温度变化率和功率变化率评估数据质量：

| 等级 | 温度变化率 (°C/s) | 功率变化率 (W/s) | 权重 | 说明 |
|------|------------------|------------------|------|------|
| A | ≥ 0.5 | ≥ 5.0 | 1.0 | 高动态数据，最适合参数辨识 |
| B | ≥ 0.1 | ≥ 1.0 | 0.6 | 中等动态数据 |
| C | ≥ 0.02 | - | 0.3 | 低动态数据 |
| D | < 0.02 | < 1.0 | - | 稳态数据，不用于辨识 |

## 参数更新流程

```
┌─────────────────┐
│  数据采集       │
│  (持续进行)     │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  质量评估       │
│  分级存储       │
└────────┬────────┘
         │
         ▼
┌─────────────────┐     否
│  检查触发条件   │────────────┐
└────────┬────────┘            │
         │ 是                  │
         ▼                     │
┌─────────────────┐            │
│  参数优化       │            │
│  (后台计算)     │            │
└────────┬────────┘            │
         │                     │
         ▼                     │
┌─────────────────┐            │
│  参数验证       │            │
└────────┬────────┘            │
         │ 通过                │
         ▼                     │
┌─────────────────┐            │
│  应用新参数     │            │
│  (平滑过渡)     │            │
└────────┬────────┘            │
         │                     │
         ▼                     │
┌─────────────────┐◄───────────┘
│  继续监控       │
└─────────────────┘
```

## 触发条件

参数更新在以下任一条件满足时触发：

1. **数据量触发**
   - 总数据量 ≥ `adaptive_min_data`
   - 高质量数据量 ≥ `adaptive_min_hq_data`

2. **定时触发**
   - 距上次更新时间 ≥ `adaptive_interval`

3. **误差触发**
   - 当前RMSE > `adaptive_rmse_threshold`

## 参数验证

新参数必须通过以下验证才能应用：

1. **正值验证**：所有参数必须为正值
2. **变化幅度验证**：单次变化不超过 `adaptive_max_change`
3. **物理合理性验证**：参数在合理物理范围内

## 故障恢复

当参数更新连续失败达到 `adaptive_max_failures` 次时，系统自动回滚到上一个有效参数版本。

## 使用建议

### 首次使用

1. 使用标准MPC进行初始调参
2. 切换到自适应MPC进行参数优化
3. 监控RMSE指标确认参数收敛

### 日常使用

1. 使用 `managed_mpc` 类型以便灵活切换
2. 定期检查 `ADAPTIVE_MPC_STATUS` 输出
3. 在工况变化后可手动触发更新

### 故障排查

1. 检查数据采集是否正常
2. 确认高质量数据量充足
3. 查看日志中的参数更新记录
4. 必要时手动切换到标准模式

## 注意事项

1. 参数辨识需要足够的动态数据，建议在温度变化过程中采集
2. 避免在稳态温度下长时间运行，此时数据质量较低
3. 参数更新间隔不宜过短，避免频繁计算影响控制性能
4. 首次使用时建议设置较小的 `adaptive_max_change` 以观察效果

## 示例配置

### 完整配置示例

```ini
[extruder]
step_pin: PF0
dir_pin: PF1
# ... 其他配置 ...

[pid_profile default extruder]
control: managed_mpc

# 基础MPC参数
block_heat_capacity: 15.0
ambient_transfer: 0.08
heater_power: 40.0
sensor_responsiveness: 0.012
target_reach_time: 2.0
smoothing: 0.83

# 自适应参数
adaptive_enabled: 1
adaptive_min_data: 1000
adaptive_min_hq_data: 300
adaptive_interval: 600.0
adaptive_max_change: 0.3
adaptive_rmse_threshold: 2.0
adaptive_max_failures: 3
adaptive_smoothing: 1
adaptive_smoothing_tau: 30.0

# 管理器参数
initial_mode: adaptive

# 环境传感器
ambient_temp_sensor: temperature_sensor chamber
cooling_fan: fan
```

### 启动G-code示例

```gcode
; 打印开始时检查状态
MPC_MANAGER_STATUS HEATER=extruder

; 预热阶段使用标准MPC
MPC_SWITCH HEATER=extruder MODE=standard

; 等待温度稳定
M109 S200

; 切换到自适应模式
MPC_SWITCH HEATER=extruder MODE=adaptive

; 开始打印...
```

### 打印结束G-code示例

```gcode
; 打印结束，查看最终状态
ADAPTIVE_MPC_STATUS HEATER=extruder

; 关闭加热器
M104 S0
```
