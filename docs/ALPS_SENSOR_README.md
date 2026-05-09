# ALPS USB 串口压力传感器

## 概述

本模块为通过 USB 串口通信的 ALPS 压力传感器提供支持。传感器输出格式为 `a=**,b=**`，其中：
- `a`：原始传感器数据
- `b`：卡尔曼滤波后的数据（本实现使用此值）

传感器以固定的 **2600 Hz** 采样率运行，无法修改。

## 配置

将以下内容添加到打印机配置文件中：

```ini
[load_cell alps_pressure]
sensor_type: alps_serial
serial_port: /dev/ttyUSB0    # Linux: /dev/ttyUSB0, Windows: COM3, macOS: /dev/cu.usbserial-*
baudrate: 9600                # 根据传感器规格调整
timeout: 1.0                  # 串口超时时间（秒）
```

### 配置选项

| 参数 | 必需 | 默认值 | 描述 |
|-----------|----------|---------|-------------|
| `sensor_type` | 是 | - | 必须为 `alps_serial` |
| `serial_port` | 是 | - | 串口路径（例如 `/dev/ttyUSB0`、`COM3`） |
| `baudrate` | 否 | 9600 | 串口波特率 |
| `timeout` | 否 | 1.0 | 串口读取超时时间（秒） |

## 校准

将传感器添加到配置后，必须进行校准：

1. **传感器归零**（移除所有负载）：
   ```
   LOAD_CELL_TARE LOAD_CELL=alps_pressure
   ```

2. **启动交互式校准**：
   ```
   LOAD_CELL_CALIBRATE LOAD_CELL=alps_pressure
   ```

3. 按照屏幕提示操作：
   - 在无负载状态下运行 `TARE`
   - 施加已知重量并运行 `CALIBRATE GRAMS=nnn`
   - 运行 `ACCEPT` 保存校准结果

4. **保存配置**：
   ```
   SAVE_CONFIG
   ```

## 使用命令

- `LOAD_CELL_READ LOAD_CELL=alps_pressure` - 读取传感器数据
- `LOAD_CELL_TARE LOAD_CELL=alps_pressure` - 设置零点
- `LOAD_CELL_CALIBRATE LOAD_CELL=alps_pressure` - 启动校准向导
- `LOAD_CELL_DIAGNOSTIC LOAD_CELL=alps_pressure` - 检查传感器健康状态

## 状态变量

传感器通过 API 提供以下状态变量：

```json
{
  "force_g": 0.0,           // 当前力值（克）
  "min_force_g": 0.0,       // 缓冲区最小力值
  "max_force_g": 0.0,       // 缓冲区最大力值
  "is_calibrated": true,    // 传感器是否已校准
  "counts_per_gram": 1.0,   // 校准系数
  "reference_tare_counts": 0,
  "tare_counts": 0,
  "tare_force": 0.0
}
```

## 故障排除

### 传感器未检测到
- 验证串口路径是否正确
- 检查权限（Linux：将用户添加到 `dialout` 组）
- 确保没有其他进程正在使用该串口

### 未接收到数据
- 验证波特率是否与传感器规格匹配
- 检查接线和连接
- 使用 `LOAD_CELL_DIAGNOSTIC` 检查错误

### 校准问题
- 归零前确保传感器稳定
- 使用适当的校准重量（50g - 25kg）
- 检查读数是否饱和

## 技术细节

- **采样率**：固定为 2600 Hz（2.6 kHz）
- **数据格式**：使用正则表达式解析 `a=**,b=**` 模式
- **滤波数据**：使用 `b` 值（卡尔曼滤波后）
- **缓冲区大小**：5200 个样本（2600 Hz 下的 2 秒）
- **分发间隔**：每 100ms 批量更新客户端

## 依赖项

本模块需要 `pyserial` 库，该库应该已经安装在 Klipper 环境中。
