# =============================================================================
# 温度数据采集模块 - 用于热参数辨识
# Temperature Data Collector for Thermal Parameter Identification
# =============================================================================
#
# Copyright (C) 2024
#
# 本文件依据 GNU GPLv3 许可证分发。
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# =============================================================================
# 模块概述 (Module Overview)
# =============================================================================
#
# 本模块用于3D打印机加热系统的热参数辨识，提供两种实验方法：
#
# 1. 稳态阶梯响应实验 (Steady-State Staircase Characterization)
#    - 目的：分离并量化系统的静态热损耗参数
#    - 辨识参数：线性散热系数 K_lin（对流+传导）、辐射系数 K_rad
#    - 原理：在热平衡状态下，输入功率等于散热功率
#
# 2. 高频动态激励实验 (PRBS Dynamic Excitation)
#    - 目的：辨识系统的动态惯性参数
#    - 辨识参数：热容 C、接触热阻 R
#    - 原理：通过伪随机二进制序列激发系统的频率响应特性
#
# =============================================================================
# 热学模型 (Thermal Model)
# =============================================================================
#
# 热损耗模型：
#   P_loss = K_lin × (T - T_amb) + K_rad × (T⁴ - T_amb⁴)
#
# 多节点热网络模型：
#   [加热芯] --R_hb--> [加热块] --R_bs--> [传感器]
#       |                  |                  |
#     C_h                C_b                C_s
#
# 其中：
#   - K_lin: 线性散热系数 (W/K)，包含对流和喉管传导
#   - K_rad: 辐射系数 (W/K⁴)，等于 εσA
#   - C_h, C_b, C_s: 各节点的热容 (J/K)
#   - R_hb, R_bs: 节点间的接触热阻 (K/W)
#
# =============================================================================

import csv
import logging
import math
import os
import random
import threading
import time

# =============================================================================
# 全局常量定义 (Global Constants)
# =============================================================================

# PWM引脚最小时间间隔（秒），确保MCU有足够时间处理命令
PIN_MIN_TIME = 0.100

# 默认环境温度（摄氏度），用于热损耗计算
AMBIENT_TEMP = 25.0


# =============================================================================
# 核心类：温度数据采集器
# Core Class: Temperature Data Collector
# =============================================================================

class TemperatureDataCollector:
    """
    温度数据采集器类
    
    该类实现温度数据的实时采集、存储和实验管理功能。支持：
    - 手动数据采集模式
    - 稳态阶梯响应自动实验
    - PRBS动态激励自动实验
    - CSV格式数据存储
    
    属性：
        printer: 打印机主对象引用
        reactor: 事件反应器，用于定时器管理
        gcode: G-code命令处理器
        data_dir: 数据存储目录路径
        default_sample_rate: 默认采样率 (Hz)
        max_heater_power: 加热器最大功率 (W)
        is_collecting: 是否正在采集数据
        data_buffer: 数据缓存列表
        heater: 当前监控的加热器对象
    """
    
    def __init__(self, config):
        """
        初始化温度数据采集器
        
        参数：
            config: 配置对象，包含printer.cfg中的配置参数
            
        配置参数说明：
            data_dir: 数据存储目录，默认 ~/printer_data/temp_data
            sample_rate: 默认采样率 (Hz)，默认 10.0
            max_heater_power: 加热器最大功率 (W)，默认 60.0
        """
        # 获取打印机核心对象
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")

        # 从配置文件读取参数
        # data_dir: 数据存储目录，支持 ~ 展开为用户主目录
        self.data_dir = os.path.expanduser(
            config.get("data_dir", "~/printer_data/temp_data")
        )
        # sample_rate: 采样频率，单位 Hz，影响数据采集的时间分辨率
        self.default_sample_rate = config.getfloat("sample_rate", 10.0, above=0.0)
        # max_heater_power: 加热器额定功率，用于将PWM占空比转换为实际功率(W)
        self.max_heater_power = config.getfloat("max_heater_power", 60.0, above=0.0)

        # 采集状态变量
        self.is_collecting = False                    # 是否正在采集
        self.collection_lock = threading.Lock()       # 线程锁，保护数据缓冲区
        self.current_experiment = None                # 当前实验名称
        self.data_buffer = []                         # 数据缓存列表
        self.sample_timer = None                      # 采样定时器句柄
        self.start_time = 0.0                         # 实验开始时间

        # 加热器相关对象
        self.heater = None                            # 加热器对象引用
        self.heater_name = None                       # 加热器名称
        self.pwm_pin = None                           # PWM引脚对象（保留扩展用）
        self.sensor = None                            # 温度传感器对象（保留扩展用）

        # 注册事件处理器
        # klippy:ready - 打印机初始化完成时触发
        self.printer.register_event_handler(
            "klippy:ready", self._handle_ready
        )
        # klippy:shutdown - 打印机关闭时触发，用于清理资源
        self.printer.register_event_handler(
            "klippy:shutdown", self._handle_shutdown
        )

        # 注册G-code命令
        # 每个命令对应一个处理方法，desc属性提供命令帮助信息
        self.gcode.register_command(
            "TEMP_DATA_COLLECT",
            self.cmd_TEMP_DATA_COLLECT,
            desc=self.cmd_TEMP_DATA_COLLECT_help,
        )
        self.gcode.register_command(
            "TEMP_DATA_STOP",
            self.cmd_TEMP_DATA_STOP,
            desc=self.cmd_TEMP_DATA_STOP_help,
        )
        self.gcode.register_command(
            "STEADY_STATE_CALIBRATE",
            self.cmd_STEADY_STATE_CALIBRATE,
            desc=self.cmd_STEADY_STATE_CALIBRATE_help,
        )
        self.gcode.register_command(
            "PRBS_CALIBRATE",
            self.cmd_PRBS_CALIBRATE,
            desc=self.cmd_PRBS_CALIBRATE_help,
        )
        self.gcode.register_command(
            "TEMP_DATA_STATUS",
            self.cmd_TEMP_DATA_STATUS,
            desc=self.cmd_TEMP_DATA_STATUS_help,
        )

        # 创建数据存储目录
        # 如果目录不存在则自动创建
        if not os.path.exists(self.data_dir):
            try:
                os.makedirs(self.data_dir)
            except OSError as e:
                logging.warning("无法创建数据目录: %s", e)

    # =========================================================================
    # 事件处理方法 (Event Handlers)
    # =========================================================================

    def _handle_ready(self):
        """
        打印机就绪事件处理器
        
        在打印机初始化完成后调用，可用于执行需要其他模块已加载的初始化操作。
        当前为空实现，保留用于未来扩展。
        """
        pass

    def _handle_shutdown(self):
        """
        打打印机关闭事件处理器
        
        在打印机关闭时调用，确保停止正在进行的采集任务，
        防止数据丢失或加热器异常。
        """
        self._stop_collection()

    # =========================================================================
    # 辅助方法 (Helper Methods)
    # =========================================================================

    def _get_heater(self, heater_name):
        """
        获取加热器对象
        
        参数：
            heater_name: 加热器名称，如 "extruder" 或 "heater_bed"
            
        返回：
            加热器对象引用
            
        异常：
            如果加热器不存在，将抛出配置错误异常
        """
        pheaters = self.printer.lookup_object("heaters")
        return pheaters.lookup_heater(heater_name)

    def _get_sensor_temp(self):
        """
        获取当前传感器温度
        
        返回：
            当前温度值（摄氏度），如果加热器未初始化则返回 0.0
        """
        if self.heater is None:
            return 0.0
        # get_temp() 返回 (当前温度, 目标温度) 元组
        return self.heater.get_temp(self.reactor.monotonic())[0]

    def _get_pwm_value(self):
        """
        获取当前PWM占空比
        
        返回：
            当前PWM占空比 (0.0-1.0)，如果加热器未初始化则返回 0.0
        """
        if self.heater is None:
            return 0.0
        return getattr(self.heater, "last_pwm_value", 0.0)

    def _set_heater_power(self, power):
        """
        设置加热器功率（开环控制）
        
        该方法直接设置PWM占空比，绕过PID控制器，
        用于PRBS实验中的开环功率控制。
        
        参数：
            power: PWM占空比 (0.0-1.0)，表示最大功率的百分比
            
        注意：
            此方法用于开环控制，不经过PID调节，
            使用时需确保有适当的安全保护措施。
        """
        if self.heater is None:
            return
        current_time = self.reactor.monotonic()
        # 计算MCU打印时间，添加最小延迟确保命令能被处理
        print_time = (
            self.heater.mcu_pwm.get_mcu().estimated_print_time(current_time)
            + PIN_MIN_TIME
        )
        self.heater.set_pwm(print_time, power)

    # =========================================================================
    # 数据采集核心方法 (Data Collection Core Methods)
    # =========================================================================

    def _sample_callback(self, eventtime):
        """
        采样回调函数 - 采集单次数据样本
        
        该方法在每次采样时刻被调用，采集温度、PWM、目标温度等数据，
        并将样本存入数据缓冲区。
        
        参数：
            eventtime: 事件时间戳（reactor单调时间）
            
        采集的数据字段：
            - time: 采样时间戳
            - temperature: 当前温度 (°C)
            - pwm: PWM占空比 (0.0-1.0)
            - target: 目标温度 (°C)
            - power_watts: 计算功率 (W) = pwm × max_heater_power
            - experiment: 实验名称（可选）
        """
        if not self.is_collecting:
            return
        
        # 采集各项数据
        temp = self._get_sensor_temp()
        pwm = self._get_pwm_value()
        target = getattr(self.heater, "target_temp", 0.0) if self.heater else 0.0

        # 构建数据样本字典
        sample = {
            "time": eventtime,
            "temperature": temp,
            "pwm": pwm,
            "target": target,
            "power_watts": pwm * self.max_heater_power,  # PWM转换为实际功率
        }

        # 如果有实验名称，添加到样本中
        if self.current_experiment is not None:
            sample["experiment"] = self.current_experiment

        # 使用线程锁保护数据缓冲区，防止并发写入问题
        with self.collection_lock:
            self.data_buffer.append(sample)

    def _start_collection(self, experiment_name=None):
        """
        启动数据采集
        
        初始化采集状态，清空数据缓冲区，启动采样定时器。
        
        参数：
            experiment_name: 实验名称，用于标识数据来源
            
        返回：
            bool: True 表示成功启动，False 表示已有采集任务在进行
            
        注意：
            该方法使用线程锁确保状态变更的原子性。
        """
        with self.collection_lock:
            # 检查是否已有采集任务
            if self.is_collecting:
                return False
            
            # 初始化采集状态
            self.is_collecting = True
            self.current_experiment = experiment_name
            self.data_buffer = []
            self.start_time = self.reactor.monotonic()

        # 注册采样定时器
        # 定时器周期 = 1 / 采样率
        self.sample_timer = self.reactor.register_timer(
            self._sample_callback_timer, self.reactor.NOW
        )
        return True

    def _sample_callback_timer(self, eventtime):
        """
        采样定时器回调函数
        
        该方法由reactor定时器调用，执行采样并返回下次调用时间。
        
        参数：
            eventtime: 当前事件时间
            
        返回：
            float: 下次调用的时间戳，实现周期性采样
        """
        self._sample_callback(eventtime)
        # 计算下次采样时间 = 当前时间 + 采样周期
        return eventtime + (1.0 / self.default_sample_rate)

    def _stop_collection(self):
        """
        停止数据采集
        
        停止采样定时器，设置采集状态为False。
        注意：该方法不保存数据，需调用 _save_data_to_csv() 单独保存。
        """
        self.is_collecting = False
        if self.sample_timer is not None:
            self.reactor.unregister_timer(self.sample_timer)
            self.sample_timer = None

    def _save_data_to_csv(self, filename):
        """
        将采集数据保存到CSV文件
        
        参数：
            filename: 输出文件名（不含路径）
            
        返回：
            bool: True 表示保存成功，False 表示保存失败或无数据
            
        输出格式：
            CSV文件，包含以下列：
            - time: 相对时间（秒，从实验开始计时）
            - temperature: 温度 (°C)
            - pwm: PWM占空比
            - target: 目标温度 (°C)
            - power_watts: 功率 (W)
            - experiment: 实验名称
        """
        if not self.data_buffer:
            return False

        filepath = os.path.join(self.data_dir, filename)
        try:
            with open(filepath, "w", newline="") as csvfile:
                # 定义CSV列名
                fieldnames = [
                    "time",
                    "temperature",
                    "pwm",
                    "target",
                    "power_watts",
                    "experiment",
                ]
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()

                # 计算起始时间，将绝对时间转换为相对时间
                start_time = self.data_buffer[0]["time"]
                for sample in self.data_buffer:
                    row = {
                        "time": sample["time"] - start_time,  # 相对时间
                        "temperature": sample["temperature"],
                        "pwm": sample["pwm"],
                        "target": sample["target"],
                        "power_watts": sample["power_watts"],
                        "experiment": sample.get("experiment", ""),
                    }
                    writer.writerow(row)
            return True
        except Exception as e:
            logging.error("保存CSV数据失败: %s", e)
            return False

    # =========================================================================
    # G-code 命令处理方法 (G-code Command Handlers)
    # =========================================================================

    # 命令帮助文本
    cmd_TEMP_DATA_COLLECT_help = "启动温度数据采集"

    def cmd_TEMP_DATA_COLLECT(self, gcmd):
        """
        处理 TEMP_DATA_COLLECT 命令 - 启动手动数据采集
        
        用法：
            TEMP_DATA_COLLECT [HEATER=<加热器名>] [SAMPLE_RATE=<采样率>] [EXPERIMENT=<实验名>]
            
        参数：
            HEATER: 加热器名称，默认 "extruder"
            SAMPLE_RATE: 采样频率 (Hz)，默认使用配置值
            EXPERIMENT: 实验名称，用于数据标识
            
        示例：
            TEMP_DATA_COLLECT HEATER=extruder SAMPLE_RATE=20 EXPERIMENT=manual_test
        """
        # 解析命令参数
        heater_name = gcmd.get("HEATER", "extruder")
        sample_rate = gcmd.get_float("SAMPLE_RATE", self.default_sample_rate, above=0.0)
        experiment_name = gcmd.get("EXPERIMENT", "manual_collection")

        # 获取加热器对象
        try:
            self.heater = self._get_heater(heater_name)
            self.heater_name = heater_name
        except Exception as e:
            raise gcmd.error(f"未找到加热器 '{heater_name}': {e}")

        # 更新采样率
        self.default_sample_rate = sample_rate

        # 启动采集
        if self._start_collection(experiment_name):
            gcmd.respond_info(
                f"已启动 '{heater_name}' 的温度数据采集，"
                f"采样率 {sample_rate} Hz，实验名称: {experiment_name}"
            )
        else:
            gcmd.respond_info("数据采集已在进行中。")

    cmd_TEMP_DATA_STOP_help = "停止温度数据采集并保存到文件"

    def cmd_TEMP_DATA_STOP(self, gcmd):
        """
        处理 TEMP_DATA_STOP 命令 - 停止采集并保存数据
        
        用法：
            TEMP_DATA_STOP [FILENAME=<文件名>]
            
        参数：
            FILENAME: 输出文件名，默认自动生成（temp_data_<时间戳>.csv）
            
        示例：
            TEMP_DATA_STOP FILENAME=my_experiment.csv
        """
        filename = gcmd.get("FILENAME", f"temp_data_{int(time.time())}.csv")

        # 停止采集
        self._stop_collection()

        # 保存数据
        if self._save_data_to_csv(filename):
            filepath = os.path.join(self.data_dir, filename)
            gcmd.respond_info(
                f"数据采集已停止。已保存 {len(self.data_buffer)} 个样本到 {filepath}"
            )
        else:
            gcmd.respond_info("无数据可保存或保存失败。")

        # 清理状态
        with self.collection_lock:
            self.data_buffer = []
            self.current_experiment = None

    cmd_TEMP_DATA_STATUS_help = "获取温度数据采集状态"

    def cmd_TEMP_DATA_STATUS(self, gcmd):
        """
        处理 TEMP_DATA_STATUS 命令 - 显示当前采集状态
        
        用法：
            TEMP_DATA_STATUS
            
        输出信息：
            - 当前状态（采集中/空闲）
            - 当前实验名称
            - 已采集样本数
            - 采样率
            - 数据存储目录
        """
        status = "采集中" if self.is_collecting else "空闲"
        experiment = self.current_experiment or "无"
        samples = len(self.data_buffer)

        gcmd.respond_info(
            f"温度数据采集器状态:\n"
            f"  状态: {status}\n"
            f"  当前实验: {experiment}\n"
            f"  已采集样本: {samples}\n"
            f"  采样率: {self.default_sample_rate} Hz\n"
            f"  数据目录: {self.data_dir}"
        )

    cmd_STEADY_STATE_CALIBRATE_help = "运行稳态阶梯响应校准实验"

    def cmd_STEADY_STATE_CALIBRATE(self, gcmd):
        """
        处理 STEADY_STATE_CALIBRATE 命令 - 稳态阶梯响应实验
        
        该实验通过测量不同温度点下的平衡功率，辨识热损耗参数。
        
        原理说明：
            在热平衡状态下，dT/dt = 0，输入功率等于散热功率：
            P_in = P_loss(T) = K_lin × (T - T_amb) + K_rad × (T⁴ - T_amb⁴)
            
        用法：
            STEADY_STATE_CALIBRATE [HEATER=<加热器>] [TEMP_POINTS=<温度点>] 
                                   [TOLERANCE=<容差>] [DURATION=<持续时间>] 
                                   [FILENAME=<文件名>]
            
        参数：
            HEATER: 加热器名称，默认 "extruder"
            TEMP_POINTS: 温度设定点序列，逗号分隔，默认 "50,100,150,200,250,300"
            TOLERANCE: 稳定性判断容差 (°C)，默认 0.1
            DURATION: 稳定持续时间要求 (秒)，默认 180
            FILENAME: 输出文件名
            
        示例：
            STEADY_STATE_CALIBRATE HEATER=extruder TEMP_POINTS=50,100,150,200,250,300,350,400,450
        """
        # 解析参数
        heater_name = gcmd.get("HEATER", "extruder")
        temp_points = gcmd.get("TEMP_POINTS", "50,100,150,200,250,300")
        stability_tolerance = gcmd.get_float("TOLERANCE", 1, above=0.0)
        stability_duration = gcmd.get_float("DURATION", 180.0, above=0.0)
        filename = gcmd.get(
            "FILENAME", f"steady_state_{int(time.time())}.csv"
        )

        # 获取加热器
        try:
            self.heater = self._get_heater(heater_name)
            self.heater_name = heater_name
        except Exception as e:
            raise gcmd.error(f"未找到加热器 '{heater_name}': {e}")

        # 解析温度点序列
        try:
            temps = [float(t.strip()) for t in temp_points.split(",")]
        except ValueError:
            raise gcmd.error("TEMP_POINTS 格式无效。请使用逗号分隔的数值。")

        gcmd.respond_info(
            f"启动 '{heater_name}' 的稳态阶梯响应校准\n"
            f"温度设定点: {temps}\n"
            f"稳定性容差: {stability_tolerance}°C\n"
            f"稳定持续时间: {stability_duration}s"
        )

        # 启动数据采集
        self._start_collection("steady_state_staircase")

        results = []
        try:
            # 遍历每个温度设定点
            for target_temp in temps:
                gcmd.respond_info(f"加热至 {target_temp}°C...")
                
                # 执行单个温度点的稳态测量
                self._run_steady_state_point(
                    target_temp, stability_tolerance, stability_duration, gcmd
                )

                # 计算该温度点的平均功率和温度
                avg_power = self._calculate_average_power(stability_duration)
                avg_temp = self._calculate_average_temp(stability_duration)

                # 记录结果
                results.append(
                    {
                        "target_temp": target_temp,
                        "avg_temp": avg_temp,
                        "avg_power": avg_power,
                    }
                )
                gcmd.respond_info(
                    f"温度 {target_temp}°C 稳态结果: "
                    f"平均温度={avg_temp:.2f}°C, 平均功率={avg_power:.2f}W"
                )

        except Exception as e:
            gcmd.respond_raw(f"!! 校准中断: {e}")
        finally:
            # 确保关闭加热器
            self._set_heater_power(0.0)
            self._stop_collection()
            self._save_data_to_csv(filename)

        # 保存汇总结果
        if results:
            self._save_steady_state_results(results, filename)

        gcmd.respond_info(
            f"稳态阶梯响应校准完成。数据已保存到 {filename}"
        )

    def _run_steady_state_point(
        self, target_temp, tolerance, duration, gcmd
    ):
        """
        执行单个温度点的稳态测量
        
        等待加热器达到目标温度并稳定指定时间。
        
        参数：
            target_temp: 目标温度 (°C)
            tolerance: 温度波动容差 (°C)，用于判断是否稳定
            duration: 需要保持稳定的持续时间 (秒)
            gcmd: G-code命令对象，用于输出信息
            
        稳定判据：
            温度波动范围 ≤ tolerance，且持续时间 ≥ duration
        """
        # 使用heaters模块的温度设置功能（带PID控制）
        pheaters = self.printer.lookup_object("heaters")
        pheaters.set_temperature(self.heater, target_temp, wait=True)

        gcmd.respond_info(f"等待 {target_temp}°C 温度稳定...")

        # 稳定性监测变量
        stable_start = None           # 开始稳定的时刻
        check_interval = 1.0          # 检查间隔（秒）
        last_temps = []               # 最近温度记录

        while True:
            current_temp = self._get_sensor_temp()

            # 记录温度历史
            last_temps.append(current_temp)
            # 保持历史记录长度不超过检查窗口
            if len(last_temps) > int(duration / check_interval):
                last_temps.pop(0)

            # 检查稳定性
            if len(last_temps) >= 3:
                temp_range = max(last_temps) - min(last_temps)
                if temp_range <= tolerance:
                    # 温度波动在容差范围内
                    if stable_start is None:
                        stable_start = self.reactor.monotonic()
                    elif (self.reactor.monotonic() - stable_start) >= duration:
                        # 已稳定足够长时间
                        gcmd.respond_info(
                            f"温度 {target_temp}°C 已达到稳定状态"
                        )
                        return
                else:
                    # 温度波动超出容差，重置稳定计时
                    stable_start = None

            # 等待下一次检查
            self.reactor.pause(self.reactor.monotonic() + check_interval)

    def _calculate_average_power(self, duration):
        """
        计算指定时间窗口内的平均功率
        
        参数：
            duration: 时间窗口长度（秒）
            
        返回：
            float: 平均功率 (W)
        """
        if not self.data_buffer:
            return 0.0

        # 获取最近N个样本（N = duration × sample_rate）
        recent_samples = self.data_buffer[-int(duration * self.default_sample_rate) :]
        if not recent_samples:
            return 0.0

        return sum(s["power_watts"] for s in recent_samples) / len(recent_samples)

    def _calculate_average_temp(self, duration):
        """
        计算指定时间窗口内的平均温度
        
        参数：
            duration: 时间窗口长度（秒）
            
        返回：
            float: 平均温度 (°C)
        """
        if not self.data_buffer:
            return 0.0

        recent_samples = self.data_buffer[-int(duration * self.default_sample_rate) :]
        if not recent_samples:
            return 0.0

        return sum(s["temperature"] for s in recent_samples) / len(recent_samples)

    def _save_steady_state_results(self, results, base_filename):
        """
        保存稳态实验汇总结果
        
        将各温度点的平均温度和功率保存到单独的结果文件。
        
        参数：
            results: 结果列表，每项包含 target_temp, avg_temp, avg_power
            base_filename: 基础文件名，结果文件名在此基础上添加 _results 后缀
        """
        filename = base_filename.replace(".csv", "_results.csv")
        filepath = os.path.join(self.data_dir, filename)

        try:
            with open(filepath, "w", newline="") as csvfile:
                fieldnames = ["target_temp", "avg_temp", "avg_power"]
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                for r in results:
                    writer.writerow(r)
        except Exception as e:
            logging.error("保存稳态结果失败: %s", e)

    cmd_PRBS_CALIBRATE_help = "运行PRBS动态激励校准实验"

    def cmd_PRBS_CALIBRATE(self, gcmd):
        """
        处理 PRBS_CALIBRATE 命令 - PRBS动态激励实验
        
        该实验通过伪随机二进制序列功率信号，辨识系统的动态热参数。
        
        原理说明：
            PRBS信号包含丰富的频率成分，能激发系统的动态响应特性：
            - 短脉冲（0.2-1s）：激发小热容节点（加热芯）的快速响应
            - 长脉冲（5-20s）：反映整体热容和传导滞后
            
        用法：
            PRBS_CALIBRATE [HEATER=<加热器>] [DURATION=<总时长>] 
                          [MIN_PULSE=<最小脉宽>] [MAX_PULSE=<最大脉宽>]
                          [POWER_LEVELS=<功率电平>] [SAMPLE_RATE=<采样率>]
                          [FILENAME=<文件名>]
            
        参数：
            HEATER: 加热器名称，默认 "extruder"
            DURATION: 实验总时长（秒），默认 300
            MIN_PULSE: 最小脉冲宽度（秒），默认 0.2
            MAX_PULSE: 最大脉冲宽度（秒），默认 10.0
            POWER_LEVELS: 功率电平序列，逗号分隔，默认 "0.0,1.0"
            SAMPLE_RATE: 采样频率 (Hz)，默认 20.0
            FILENAME: 输出文件名
            
        示例：
            PRBS_CALIBRATE HEATER=extruder DURATION=600 MIN_PULSE=0.2 MAX_PULSE=15 POWER_LEVELS=0.0,0.5,1.0
        """
        # 解析参数
        heater_name = gcmd.get("HEATER", "extruder")
        duration = gcmd.get_float("DURATION", 300.0, above=0.0)
        min_pulse = gcmd.get_float("MIN_PULSE", 0.2, above=0.0)
        max_pulse = gcmd.get_float("MAX_PULSE", 10.0, above=0.0, minval=min_pulse)
        power_levels = gcmd.get("POWER_LEVELS", "0.0,1.0")
        sample_rate = gcmd.get_float("SAMPLE_RATE", 20.0, above=0.0)
        filename = gcmd.get("FILENAME", f"prbs_data_{int(time.time())}.csv")

        # 获取加热器
        try:
            self.heater = self._get_heater(heater_name)
            self.heater_name = heater_name
        except Exception as e:
            raise gcmd.error(f"未找到加热器 '{heater_name}': {e}")

        # 解析功率电平
        try:
            powers = [float(p.strip()) for p in power_levels.split(",")]
        except ValueError:
            raise gcmd.error(
                "POWER_LEVELS 格式无效。请使用逗号分隔的数值。"
            )

        gcmd.respond_info(
            f"启动 '{heater_name}' 的PRBS动态激励校准\n"
            f"实验时长: {duration}s\n"
            f"脉冲宽度范围: {min_pulse}s - {max_pulse}s\n"
            f"功率电平: {powers}\n"
            f"采样率: {sample_rate} Hz"
        )

        # 设置采样率并启动采集
        self.default_sample_rate = sample_rate
        self._start_collection("prbs_dynamic")

        # 生成PRBS序列
        prbs_sequence = self._generate_prbs_sequence(
            duration, min_pulse, max_pulse, powers
        )

        try:
            # 执行PRBS序列
            start_time = self.reactor.monotonic()
            for pulse_power, pulse_duration in prbs_sequence:
                if not self.is_collecting:
                    break

                # 设置功率（开环控制）
                self._set_heater_power(pulse_power)

                # 等待脉冲结束
                pulse_end = start_time + pulse_duration
                while self.reactor.monotonic() < pulse_end:
                    if not self.is_collecting:
                        break
                    self.reactor.pause(
                        self.reactor.monotonic() + 0.01
                    )

                start_time = pulse_end

        except Exception as e:
            gcmd.respond_raw(f"!! PRBS校准中断: {e}")
        finally:
            # 确保关闭加热器
            self._set_heater_power(0.0)
            self._stop_collection()
            self._save_data_to_csv(filename)

        gcmd.respond_info(
            f"PRBS动态激励校准完成。数据已保存到 {filename}"
        )

    def _generate_prbs_sequence(self, duration, min_pulse, max_pulse, powers):
        """
        生成伪随机二进制序列 (PRBS)
        
        生成一个随机的功率脉冲序列，用于系统辨识。
        
        参数：
            duration: 序列总时长（秒）
            min_pulse: 最小脉冲宽度（秒）
            max_pulse: 最大脉冲宽度（秒）
            powers: 可选功率电平列表
            
        返回：
            list: [(功率, 持续时间), ...] 元组列表
            
        注意：
            该序列满足"持续激励条件"，保证辨识结果收敛且唯一。
        """
        sequence = []
        total_time = 0.0

        while total_time < duration:
            # 随机生成脉冲宽度
            pulse_duration = random.uniform(min_pulse, max_pulse)
            # 确保不超过总时长
            pulse_duration = min(pulse_duration, duration - total_time)

            # 随机选择功率电平
            pulse_power = random.choice(powers)

            sequence.append((pulse_power, pulse_duration))
            total_time += pulse_duration

        return sequence

    def get_status(self, eventtime):
        """
        获取模块状态信息
        
        该方法供API Server和宏调用，返回当前采集状态。
        
        参数：
            eventtime: 事件时间（未使用，但必须保留以符合接口规范）
            
        返回：
            dict: 状态信息字典，包含：
                - is_collecting: 是否正在采集
                - current_experiment: 当前实验名称
                - samples_collected: 已采集样本数
                - sample_rate: 当前采样率
                - data_directory: 数据存储目录
        """
        return {
            "is_collecting": self.is_collecting,
            "current_experiment": self.current_experiment or "",
            "samples_collected": len(self.data_buffer),
            "sample_rate": self.default_sample_rate,
            "data_directory": self.data_dir,
        }


# =============================================================================
# 热参数估计器类
# Thermal Parameter Estimator Class
# =============================================================================

class ThermalParameterEstimator:
    """
    热参数估计器类
    
    该类提供从采集数据估计热参数的功能，支持：
    - 稳态数据分析：估计线性散热系数和辐射系数
    - PRBS数据分析：提供外部工具分析建议
    
    数学模型：
        P_loss = K_lin × (T - T_amb) + K_rad × (T⁴ - T_amb⁴)
        
    其中：
        K_lin: 线性散热系数 (W/K)
        K_rad: 辐射系数 (W/K⁴)
    """
    
    def __init__(self, config):
        """
        初始化热参数估计器
        
        参数：
            config: 配置对象
        """
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")

        # 注册G-code命令
        self.gcode.register_command(
            "ESTIMATE_THERMAL_PARAMS",
            self.cmd_ESTIMATE_THERMAL_PARAMS,
            desc=self.cmd_ESTIMATE_THERMAL_PARAMS_help,
        )

    cmd_ESTIMATE_THERMAL_PARAMS_help = "从采集数据估计热参数"

    def cmd_ESTIMATE_THERMAL_PARAMS(self, gcmd):
        """
        处理 ESTIMATE_THERMAL_PARAMS 命令 - 估计热参数
        
        用法：
            ESTIMATE_THERMAL_PARAMS DATA_FILE=<数据文件> [METHOD=<方法>]
            
        参数：
            DATA_FILE: 数据文件路径
            METHOD: 估计方法，可选 "steady_state" 或 "prbs"，默认 "steady_state"
            
        示例：
            ESTIMATE_THERMAL_PARAMS DATA_FILE=~/printer_data/temp_data/steady_state_xxx.csv METHOD=steady_state
        """
        data_file = gcmd.get("DATA_FILE")
        method = gcmd.get("METHOD", "steady_state")

        if method == "steady_state":
            self._estimate_from_steady_state(data_file, gcmd)
        elif method == "prbs":
            self._estimate_from_prbs(data_file, gcmd)
        else:
            raise gcmd.error(f"未知的估计方法: {method}")

    def _estimate_from_steady_state(self, data_file, gcmd):
        """
        从稳态数据估计热参数
        
        使用最小二乘法拟合热损耗模型：
        P_loss = K_lin × (T - T_amb) + K_rad × (T⁴ - T_amb⁴)
        
        参数：
            data_file: 稳态实验结果文件路径
            gcmd: G-code命令对象
        """
        # 加载稳态结果文件
        try:
            results_file = data_file.replace(".csv", "_results.csv")
            results = self._load_csv(results_file)
        except Exception as e:
            raise gcmd.error(f"加载数据文件失败: {e}")

        if not results:
            raise gcmd.error("数据文件中未找到稳态结果")

        # 提取温度和功率数据
        temps = [float(r["avg_temp"]) for r in results]
        powers = [float(r["avg_power"]) for r in results]

        # 拟合热损耗模型
        k_lin, k_rad = self._fit_heat_loss_model(temps, powers)

        gcmd.respond_info(
            f"热参数估计结果:\n"
            f"  线性散热系数 (K_lin): {k_lin:.6f} W/K\n"
            f"  辐射系数 (K_rad): {k_rad:.9f} W/K^4\n"
            f"  热损耗模型: P_loss = K_lin × (T - T_amb) + K_rad × (T⁴ - T_amb⁴)"
        )

    def _fit_heat_loss_model(self, temps, powers):
        """
        拟合热损耗模型参数
        
        使用最小二乘法求解二元线性回归问题：
        y = a×x1 + b×x2
        
        其中：
            y = P_loss (功率)
            x1 = T - T_amb (线性项)
            x2 = T⁴ - T_amb⁴ (辐射项)
            a = K_lin
            b = K_rad
            
        参数：
            temps: 温度列表 (°C)
            powers: 功率列表 (W)
            
        返回：
            tuple: (K_lin, K_rad) 热损耗系数
        """
        n = len(temps)
        if n < 2:
            return 0.0, 0.0

        # 转换为开尔文温度
        T_amb = AMBIENT_TEMP + 273.15
        T_kelvin = [t + 273.15 for t in temps]

        # 计算回归所需的各项求和
        # 线性项：x1 = T - T_amb
        sum_x1 = sum(T - T_amb for T in T_kelvin)
        # 辐射项：x2 = T⁴ - T_amb⁴
        sum_x2 = sum(T**4 - T_amb**4 for T in T_kelvin)
        # 功率：y = P
        sum_y = sum(powers)
        # 平方项
        sum_x1x1 = sum((T - T_amb) ** 2 for T in T_kelvin)
        sum_x2x2 = sum((T**4 - T_amb**4) ** 2 for T in T_kelvin)
        # 交叉项
        sum_x1x2 = sum(
            (T - T_amb) * (T**4 - T_amb**4) for T in T_kelvin
        )
        # 与功率的乘积
        sum_x1y = sum(
            (T_kelvin[i] - T_amb) * powers[i] for i in range(n)
        )
        sum_x2y = sum(
            (T_kelvin[i] ** 4 - T_amb**4) * powers[i] for i in range(n)
        )

        # 求解正规方程组
        # 行列式
        det = sum_x1x1 * sum_x2x2 - sum_x1x2**2
        if abs(det) < 1e-15:
            # 矩阵奇异，退化为单参数估计
            return sum_y / sum_x1 if sum_x1 != 0 else 0.0, 0.0

        # 克拉默法则求解
        k_lin = (sum_x1y * sum_x2x2 - sum_x2y * sum_x1x2) / det
        k_rad = (sum_x2y * sum_x1x1 - sum_x1y * sum_x1x2) / det

        return k_lin, k_rad

    def _estimate_from_prbs(self, data_file, gcmd):
        """
        从PRBS数据估计热参数
        
        PRBS数据分析需要更复杂的系统辨识方法，
        此方法提供数据加载和建议信息。
        
        参数：
            data_file: PRBS实验数据文件路径
            gcmd: G-code命令对象
        """
        try:
            data = self._load_csv(data_file)
        except Exception as e:
            raise gcmd.error(f"加载数据文件失败: {e}")

        if not data:
            raise gcmd.error("文件中未找到数据")

        gcmd.respond_info(
            f"已加载 {len(data)} 个PRBS数据样本。\n"
            f"建议使用外部工具进行高级系统辨识分析。"
        )

        gcmd.respond_info(
            f"推荐分析方法:\n"
            f"  1. 从阶跃响应识别时间常数\n"
            f"  2. 使用ARX/ARMAX模型估计传递函数\n"
            f"  3. 应用子空间辨识方法建立多节点模型"
        )

    def _load_csv(self, filepath):
        """
        加载CSV数据文件
        
        参数：
            filepath: CSV文件路径
            
        返回：
            list: 数据行列表，每行为字典格式
        """
        data = []
        try:
            with open(filepath, "r") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    data.append(row)
        except Exception as e:
            logging.error("加载CSV失败: %s", e)
        return data


# =============================================================================
# 模块加载入口 (Module Entry Points)
# =============================================================================

def load_config(config):
    """
    模块加载入口函数
    
    当配置文件中存在 [temp_data_collector] 配置节时，
    Klippy会自动调用此函数加载模块。
    
    参数：
        config: 配置对象
        
    返回：
        TemperatureDataCollector 实例
    """
    return TemperatureDataCollector(config)


def load_config_prefix(config):
    """
    带前缀的模块加载入口函数
    
    当配置文件中存在 [thermal_estimator] 或类似带前缀的配置节时，
    Klippy会调用此函数。
    
    参数：
        config: 配置对象
        
    返回：
        ThermalParameterEstimator 实例
    """
    return ThermalParameterEstimator(config)
