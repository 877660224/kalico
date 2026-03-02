# 自适应MPC控制器 API 文档

**版本**: 1.0  
**最后更新**: 2026-03-01  
**作者**: Kalico开发团队

---

## 目录

1. [概述](#概述)
2. [类索引](#类索引)
3. [AdaptiveMPC类](#adaptivempc类)
   - [类概述](#adaptivempc类概述)
   - [构造函数](#adaptivempc构造函数)
   - [公共方法](#adaptivempc公共方法)
   - [属性说明](#adaptivempc属性说明)
   - [使用示例](#adaptivempc使用示例)
4. [MPCControllerManager类](#mpccontrollermanager类)
   - [类概述](#mpccontrollermanager类概述)
   - [构造函数](#mpccontrollermanager构造函数)
   - [公共方法](#mpccontrollermanager公共方法)
   - [属性说明](#mpccontrollermanager属性说明)
   - [使用示例](#mpccontrollermanager使用示例)
5. [G-code命令参考](#g-code命令参考)
6. [异常处理](#异常处理)
7. [配置参考](#配置参考)

---

## 概述

本文档描述了Kalico温度控制系统中的自适应MPC控制器相关API。该模块提供以下核心功能：

- **AdaptiveMPC**: 数据驱动的自适应参数辨识与MPC控制集成
- **MPCControllerManager**: 标准MPC与自适应MPC之间的无缝切换管理

### 模块架构

```
┌─────────────────────────────────────────────────────────────┐
│                    温度控制系统架构                          │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌─────────────────┐                                        │
│  │     Heater      │◄──── 温度采样回调                      │
│  └────────┬────────┘                                        │
│           │                                                 │
│           ▼                                                 │
│  ┌─────────────────┐     ┌─────────────────┐               │
│  │ MPCController   │────►│  ControlMPC     │ (标准模式)     │
│  │    Manager      │     └─────────────────┘               │
│  │                 │                                        │
│  │                 │     ┌─────────────────┐               │
│  │                 │────►│  AdaptiveMPC    │ (自适应模式)   │
│  └─────────────────┘     └────────┬────────┘               │
│                                   │                         │
│                                   ▼                         │
│                          ┌─────────────────┐               │
│                          │   ControlMPC    │ (基础控制器)   │
│                          └─────────────────┘               │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 类索引

| 类名 | 描述 | 继承关系 |
|------|------|----------|
| `AdaptiveMPC` | 自适应MPC控制器 | 无（组合ControlMPC） |
| `MPCControllerManager` | MPC控制器管理器 | 无（组合ControlMPC和AdaptiveMPC） |

---

## AdaptiveMPC类

### AdaptiveMPC类概述

`AdaptiveMPC`类实现数据驱动的自适应参数辨识与MPC控制的集成。通过持续采集温度数据并评估数据质量，周期性地优化热参数，实现更精确的温度控制。

**设计目的**: 传统MPC控制器使用固定参数，无法适应热系统特性的变化。本类通过数据驱动的参数辨识方法，实现参数的自动优化和更新。

**核心特性**:
- 参数辨识与实时控制解耦
- 数据质量评估与分级存储（A/B/C/D四级）
- 周期性批量参数更新
- 参数验证与版本管理
- 平滑过渡机制
- 故障自动恢复

### AdaptiveMPC构造函数

#### `__init__(profile, heater, load_clean=False, register=True)`

初始化自适应MPC控制器实例。

**参数**:

| 参数名 | 类型 | 必需 | 默认值 | 描述 |
|--------|------|------|--------|------|
| `profile` | dict | 是 | - | 控制器配置参数字典 |
| `heater` | Heater | 是 | - | 加热器对象实例 |
| `load_clean` | bool | 否 | False | 是否以清洁状态加载 |
| `register` | bool | 否 | True | 是否注册G-code命令 |

**profile字典必需键**:

| 键名 | 类型 | 描述 |
|------|------|------|
| `block_heat_capacity` | float | 加热块热容 (J/K) |
| `ambient_transfer` | float | 环境散热系数 (W/K) |
| `heater_power` | float | 加热器功率 (W) |
| `sensor_responsiveness` | float | 传感器响应系数 |

**profile字典可选键**:

| 键名 | 类型 | 默认值 | 描述 |
|------|------|--------|------|
| `adaptive_enabled` | bool | True | 是否启用自适应功能 |
| `adaptive_min_data` | int | 1000 | 最小数据量阈值 |
| `adaptive_min_hq_data` | int | 300 | 最小高质量数据量 |
| `adaptive_interval` | float | 600.0 | 更新间隔(秒) |
| `adaptive_max_change` | float | 0.5 | 最大参数变化比例 |
| `adaptive_rmse_threshold` | float | 2.0 | RMSE阈值(°C) |
| `adaptive_max_failures` | int | 3 | 最大连续失败次数 |
| `adaptive_smoothing` | bool | True | 是否启用参数平滑 |
| `adaptive_smoothing_tau` | float | 30.0 | 平滑时间常数(秒) |

**返回值**: None

**异常**: 无主动抛出异常

**示例**:
```python
profile = {
    "block_heat_capacity": 15.0,
    "ambient_transfer": 0.08,
    "heater_power": 40.0,
    "sensor_responsiveness": 0.012,
    "adaptive_enabled": True,
    "adaptive_interval": 300.0,
}
controller = AdaptiveMPC(profile, heater, load_clean=False)
```

### AdaptiveMPC公共方法

#### `temperature_update(read_time, temp, target_temp)`

温度更新回调，主要控制入口。

**参数**:

| 参数名 | 类型 | 描述 |
|--------|------|------|
| `read_time` | float | 温度读取时间戳（reactor时间，秒） |
| `temp` | float | 当前温度值（°C） |
| `target_temp` | float | 目标温度值（°C） |

**返回值**: None

**副作用**:
- 更新数据缓冲区
- 更新温度误差窗口
- 调用基础MPC控制器执行控制

---

#### `check_busy(eventtime, smoothed_temp, target_temp)`

检查控制器是否忙碌。

**参数**:

| 参数名 | 类型 | 描述 |
|--------|------|------|
| `eventtime` | float | 当前事件时间戳 |
| `smoothed_temp` | float | 平滑后的温度值（°C） |
| `target_temp` | float | 目标温度值（°C） |

**返回值**: `bool` - True表示正在调节，False表示已达到稳态

---

#### `get_status(eventtime)`

获取控制器完整状态信息。

**参数**:

| 参数名 | 类型 | 描述 |
|--------|------|------|
| `eventtime` | float | 当前事件时间戳 |

**返回值**: `dict` - 状态信息字典

**返回字典结构**:

| 键名 | 类型 | 描述 |
|------|------|------|
| `enabled` | bool | 自适应功能启用状态 |
| `adapting` | bool | 是否正在进行参数适应 |
| `data_count_a` | int | A级数据量 |
| `data_count_b` | int | B级数据量 |
| `data_count_c` | int | C级数据量 |
| `current_rmse` | float | 当前预测误差RMSE |
| `param_version` | int | 参数版本号 |
| `last_update` | float | 距上次更新的时间（秒） |
| `failures` | int | 连续失败次数 |
| `controller_mode` | str | 当前模式 |

---

#### `get_profile()`

获取控制器配置参数。

**返回值**: `dict` - 配置参数字典

---

#### `get_type()`

获取控制器类型标识。

**返回值**: `str` - "adaptive_mpc"

---

#### `update_smooth_time()`

更新平滑时间参数。

**返回值**: None

---

#### `cmd_ADAPTIVE_MPC_SET(gcmd)`

处理ADAPTIVE_MPC_SET命令，设置自适应参数。

**参数**:

| 参数名 | 类型 | 描述 |
|--------|------|------|
| `gcmd` | GCodeCommand | G-code命令对象 |

**G-code参数**:

| 参数 | 类型 | 描述 |
|------|------|------|
| `ENABLED` | int | 启用状态（0/1） |
| `UPDATE_INTERVAL` | float | 更新间隔（秒） |
| `RMSE_THRESHOLD` | float | RMSE阈值（°C） |

---

#### `cmd_ADAPTIVE_MPC_STATUS(gcmd)`

处理ADAPTIVE_MPC_STATUS命令，获取状态信息。

**参数**:

| 参数名 | 类型 | 描述 |
|--------|------|------|
| `gcmd` | GCodeCommand | G-code命令对象 |

---

#### `cmd_ADAPTIVE_MPC_SWITCH(gcmd)`

处理ADAPTIVE_MPC_SWITCH命令，切换控制器模式。

**参数**:

| 参数名 | 类型 | 描述 |
|--------|------|------|
| `gcmd` | GCodeCommand | G-code命令对象 |

**G-code参数**:

| 参数 | 类型 | 描述 |
|------|------|------|
| `MODE` | str | 目标模式（adaptive/standard） |

### AdaptiveMPC属性说明

| 属性名 | 类型 | 描述 |
|--------|------|------|
| `profile` | dict | 控制器配置参数字典 |
| `heater` | Heater | 加热器对象实例 |
| `base_mpc` | ControlMPC | 基础MPC控制器实例 |
| `data_buffer` | dict | 数据缓存，按质量分级存储 |
| `data_buffer_lock` | threading.Lock | 数据缓冲区线程锁 |
| `param_history` | list | 参数版本历史记录 |
| `current_params` | dict | 当前使用的热参数 |
| `is_adapting` | bool | 是否正在进行参数适应 |
| `adaptation_enabled` | bool | 自适应功能是否启用 |
| `last_adaptation_time` | float | 上次参数更新时间戳 |
| `adaptation_failures` | int | 连续参数更新失败次数 |
| `temp_error_window` | list | 温度预测误差滑动窗口 |

### AdaptiveMPC使用示例

#### 基本使用

```python
# 创建控制器实例
profile = {
    "block_heat_capacity": 15.0,
    "ambient_transfer": 0.08,
    "heater_power": 40.0,
    "sensor_responsiveness": 0.012,
    "adaptive_enabled": True,
}
controller = AdaptiveMPC(profile, heater)

# 在温度回调中使用
def temperature_callback(read_time, temp, target_temp):
    controller.temperature_update(read_time, temp, target_temp)
```

#### 查询状态

```python
# 获取状态信息
status = controller.get_status(reactor.monotonic())
print(f"当前RMSE: {status['current_rmse']:.2f}°C")
print(f"数据量: A={status['data_count_a']}, B={status['data_count_b']}")
```

---

## MPCControllerManager类

### MPCControllerManager类概述

`MPCControllerManager`类实现标准MPC与自适应MPC之间的无缝切换功能。通过组合模式同时持有两种控制器实例，支持在运行时动态切换控制器类型。

**设计目的**: 在实际3D打印过程中，不同阶段对温度控制的需求不同。本类提供了灵活的切换机制，无需重启系统即可切换控制器。

**核心特性**:
- 运行时动态切换
- 状态平滑传递
- 切换过程无扰动
- 完整的事件记录
- 双控制器并行

### MPCControllerManager构造函数

#### `__init__(profile, heater, load_clean=False, register=True)`

初始化MPC控制器管理器实例。

**参数**:

| 参数名 | 类型 | 必需 | 默认值 | 描述 |
|--------|------|------|--------|------|
| `profile` | dict | 是 | - | 控制器配置参数字典 |
| `heater` | Heater | 是 | - | 加热器对象实例 |
| `load_clean` | bool | 否 | False | 是否以清洁状态加载 |
| `register` | bool | 否 | True | 是否注册G-code命令 |

**profile额外键**:

| 键名 | 类型 | 默认值 | 描述 |
|------|------|--------|------|
| `initial_mode` | str | "adaptive" | 初始模式 |

**返回值**: None

**示例**:
```python
profile = {
    "initial_mode": "adaptive",
    "block_heat_capacity": 15.0,
    # ... 其他参数
}
manager = MPCControllerManager(profile, heater)
```

### MPCControllerManager公共方法

#### `temperature_update(read_time, temp, target_temp)`

温度更新回调，委托给活动控制器。

**参数**: 同AdaptiveMPC

**返回值**: None

---

#### `check_busy(eventtime, smoothed_temp, target_temp)`

检查控制器是否忙碌。

**参数**: 同AdaptiveMPC

**返回值**: `bool`

---

#### `get_status(eventtime)`

获取控制器状态信息。

**返回值**: `dict` - 包含管理器相关状态

---

#### `get_manager_status(eventtime)`

获取管理器详细状态信息。

**返回值**: `dict` - 包含切换历史等详细信息

---

#### `get_type()`

获取控制器类型标识。

**返回值**: `str` - 格式为"managed_{mode}_mpc"

---

#### `cmd_MPC_SWITCH(gcmd)`

处理MPC_SWITCH命令，切换控制器类型。

**G-code参数**:

| 参数 | 类型 | 描述 |
|------|------|------|
| `MODE` | str | 目标模式（standard/adaptive） |
| `STATE_TRANSFER` | int | 是否传递状态（0/1） |

---

#### `cmd_MPC_MODE(gcmd)`

处理MPC_MODE命令，查询或设置当前模式。

**G-code参数**:

| 参数 | 类型 | 描述 |
|------|------|------|
| `MODE` | str | 目标模式（可选） |

---

#### `cmd_MPC_MANAGER_STATUS(gcmd)`

处理MPC_MANAGER_STATUS命令，获取管理器状态。

### MPCControllerManager属性说明

| 属性名 | 类型 | 描述 |
|--------|------|------|
| `profile` | dict | 控制器配置参数字典 |
| `heater` | Heater | 加热器对象实例 |
| `standard_mpc` | ControlMPC | 标准MPC控制器实例 |
| `adaptive_mpc` | AdaptiveMPC | 自适应MPC控制器实例 |
| `current_mode` | str | 当前模式（"standard"或"adaptive"） |
| `active_controller` | object | 当前活动的控制器实例 |
| `switch_history` | list | 切换历史记录列表 |
| `switch_in_progress` | bool | 是否正在进行切换操作 |
| `state_transfer_enabled` | bool | 是否启用状态传递 |

### MPCControllerManager使用示例

#### 基本使用

```python
# 创建管理器实例
profile = {
    "initial_mode": "adaptive",
    "block_heat_capacity": 15.0,
    # ... 其他参数
}
manager = MPCControllerManager(profile, heater)

# 在温度回调中使用
def temperature_callback(read_time, temp, target_temp):
    manager.temperature_update(read_time, temp, target_temp)
```

#### 运行时切换

```python
# 通过G-code切换
# MPC_SWITCH HEATER=extruder MODE=standard

# 或通过代码切换
manager._perform_switch("standard")
```

---

## G-code命令参考

### ADAPTIVE_MPC_SET

设置自适应MPC参数。

**语法**:
```
ADAPTIVE_MPC_SET HEATER=<name> [ENABLED=<0|1>] [UPDATE_INTERVAL=<seconds>] [RMSE_THRESHOLD=<temp>]
```

**示例**:
```
ADAPTIVE_MPC_SET HEATER=extruder ENABLED=1
ADAPTIVE_MPC_SET HEATER=extruder UPDATE_INTERVAL=300 RMSE_THRESHOLD=1.5
```

---

### ADAPTIVE_MPC_STATUS

获取自适应MPC状态。

**语法**:
```
ADAPTIVE_MPC_STATUS HEATER=<name>
```

---

### ADAPTIVE_MPC_SWITCH

切换自适应MPC模式。

**语法**:
```
ADAPTIVE_MPC_SWITCH HEATER=<name> MODE=<adaptive|standard>
```

---

### MPC_SWITCH

切换MPC控制器类型（仅managed_mpc）。

**语法**:
```
MPC_SWITCH HEATER=<name> MODE=<standard|adaptive> [STATE_TRANSFER=<0|1>]
```

---

### MPC_MODE

查询或设置MPC模式（仅managed_mpc）。

**语法**:
```
MPC_MODE HEATER=<name> [MODE=<standard|adaptive>]
```

---

### MPC_MANAGER_STATUS

获取MPC管理器状态（仅managed_mpc）。

**语法**:
```
MPC_MANAGER_STATUS HEATER=<name>
```

---

## 异常处理

### 常见异常

| 异常类型 | 触发场景 | 处理方式 |
|----------|----------|----------|
| `gcmd.error` | G-code参数无效 | 检查参数格式和取值范围 |
| `config_section.error` | 配置项无效 | 检查配置文件语法 |

### 错误恢复机制

1. **参数更新失败**: 自动回滚到上一有效版本
2. **连续失败**: 达到阈值后暂停更新，记录日志
3. **切换失败**: 自动回滚到原控制器

---

## 配置参考

### 完整配置示例

```ini
[extruder]
# 加热器基础配置
heater_pin: PF6
sensor_type: EPCOS 100K B57560G104F
sensor_pin: PF3
min_temp: 0
max_temp: 300

[pid_profile default extruder]
# 控制器类型
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

### 参数范围建议

| 参数 | 最小值 | 最大值 | 推荐值 | 单位 |
|------|--------|--------|--------|------|
| `block_heat_capacity` | 5 | 100 | 10-20 | J/K |
| `ambient_transfer` | 0.01 | 1.0 | 0.05-0.15 | W/K |
| `adaptive_interval` | 60 | 3600 | 300-600 | s |
| `adaptive_max_change` | 0.1 | 0.8 | 0.3-0.5 | - |
| `adaptive_rmse_threshold` | 0.5 | 5.0 | 1.5-2.5 | °C |

---

## 附录

### 数据质量等级定义

| 等级 | 温度变化率 (°C/s) | 功率变化率 (W/s) | 权重 | 说明 |
|------|------------------|------------------|------|------|
| A | ≥ 0.5 | ≥ 5.0 | 1.0 | 高动态数据 |
| B | ≥ 0.1 | ≥ 1.0 | 0.6 | 中等动态数据 |
| C | ≥ 0.02 | - | 0.3 | 低动态数据 |
| D | < 0.02 | < 1.0 | - | 稳态数据（不存储） |

### 版本历史

| 版本 | 日期 | 变更说明 |
|------|------|----------|
| 1.0 | 2026-03-01 | 初始版本 |

---

*本文档由Kalico开发团队维护*
