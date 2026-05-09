# =============================================================================
# 通用多对象温度数据采集模块 - Universal Multi-Object Temperature Collector
# =============================================================================
#
# Copyright (C) 2024-2025
# 本文件依据 GNU GPLv3 许可证分发。
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# =============================================================================
# 模块概述 (Module Overview)
# =============================================================================
#
# 本模块提供通用的多对象温度数据采集功能，支持：
#   - heater_generic 对象（加热器，含温度和目标温度）
#   - temperature_sensor 对象（纯传感器，支持多实例配置）
#   - 可扩展的其他温度相关对象类型
#
# 功能特性：
#   1. 通过外部cfg配置文件管理对象（添加/移除/修改）
#   2. 周期性采集所有配置对象的温度数据
#   3. CSV格式存储（时间戳、对象标识、温度值、目标温度等）
#   4. GCode命令触发采集操作
#   5. 模块化架构，便于扩展新对象类型
#
# =============================================================================

import csv
import logging
import os
import threading
import time
from datetime import datetime

# =============================================================================
# 全局常量定义 (Global Constants)
# =============================================================================

DEFAULT_SAMPLE_RATE = 5.0  # 默认采样率 (Hz)
DEFAULT_DATA_DIR = "~/printer_data/temp_multi"  # 默认数据目录


# =============================================================================
# 数据源适配器基类 (Data Source Adapter Base Class)
# =============================================================================


class DataSourceAdapter:
    """
    数据源适配器基类

    定义了所有数据源必须实现的接口。每种类型的温度对象
    都需要有一个对应的适配器实现。

    支持的对象类型：
        - heater_generic: 加热器对象，提供当前温度和目标温度
        - temperature_sensor: 纯传感器对象，仅提供当前温度
        - 自定义: 用户可通过继承此类添加新的数据源类型
    """

    def __init__(self, name, obj):
        self.name = name  # 对象标识名称
        self.obj = obj  # Klipper对象引用
        self.enabled = True  # 是否启用采集

    def get_temperature(self, eventtime):
        """获取当前温度值"""
        raise NotImplementedError("子类必须实现 get_temperature 方法")

    def get_target_temperature(self, eventtime):
        """获取目标温度值（可选）"""
        return None

    def get_extra_data(self, eventtime):
        """获取额外数据（如PWM功率等）"""
        return {}

    def get_object_type(self):
        """返回对象类型标识"""
        return "unknown"


# =============================================================================
# 加热器适配器 (Heater Adapter)
# =============================================================================


class HeaterAdapter(DataSourceAdapter):
    """
    加热器数据源适配器

    用于 heater_generic 和其他加热器类型对象。
    提供当前温度、目标温度和PWM功率信息。
    """

    def __init__(self, name, obj):
        super().__init__(name, obj)

    def get_temperature(self, eventtime):
        try:
            temp, _ = self.obj.get_temp(eventtime)
            return temp
        except Exception:
            return None

    def get_target_temperature(self, eventtime):
        try:
            _, target = self.obj.get_temp(eventtime)
            return target
        except Exception:
            return None

    def get_extra_data(self, eventtime):
        extra = {}
        try:
            pwm = getattr(self.obj, "last_pwm_value", None)
            if pwm is not None:
                extra["pwm"] = pwm
                max_power = getattr(self.obj, "get_max_power", lambda: 1.0)()
                extra["power_watts"] = pwm * max_power
        except Exception:
            pass
        return extra

    def get_object_type(self):
        return "heater"


# =============================================================================
# 温度传感器适配器 (Temperature Sensor Adapter)
# =============================================================================


class TemperatureSensorAdapter(DataSourceAdapter):
    """
    纯温度传感器数据源适配器

    用于 temperature_sensor 类型对象。
    仅提供当前温度值，无目标温度控制功能。
    支持多个实例同时配置。
    """

    def __init__(self, name, obj):
        super().__init__(name, obj)

    def get_temperature(self, eventtime):
        try:
            status = self.obj.get_status(eventtime)
            return status.get("temperature")
        except Exception:
            return None

    def get_target_temperature(self, eventtime):
        return None

    def get_extra_data(self, eventtime):
        extra = {}
        try:
            status = self.obj.get_status(eventtime)
            if "measured_min_temp" in status:
                extra["min_temp"] = status["measured_min_temp"]
            if "measured_max_temp" in status:
                extra["max_temp"] = status["measured_max_temp"]
        except Exception:
            pass
        return extra

    def get_object_type(self):
        return "sensor"


# =============================================================================
# 核心类：通用多对象温度数据采集器
# Core Class: Universal Multi-Object Temperature Collector
# =============================================================================


class UniversalTempCollector:
    """
    通用多对象温度数据采集器

    该类实现对多个温度对象的统一管理和周期性数据采集。

    属性：
        printer: 打印机主对象引用
        reactor: 事件反应器，用于定时器管理
        gcode: G-code命令处理器
        data_dir: 数据存储目录路径
        sample_rate: 默认采样率 (Hz)
        sources: 已注册的数据源字典 {name: adapter}
        is_collecting: 是否正在采集数据
        data_buffer: 数据缓存列表
    """

    # 适配器工厂映射表 - 定义对象类型到适配器的对应关系
    ADAPTER_FACTORIES = {
        "heater": HeaterAdapter,
        "heater_generic": HeaterAdapter,
        "sensor": TemperatureSensorAdapter,
        "temperature_sensor": TemperatureSensorAdapter,
    }

    def __init__(self, config):
        """
        初始化通用多对象温度数据采集器

        参数：
            config: 配置对象，包含printer.cfg中的配置参数

        配置参数说明：
            data_dir: 数据存储目录，默认 ~/printer_data/temp_multi
            sample_rate: 默认采样率 (Hz)，默认 5.0
            auto_start: 是否自动开始采集，默认 False
        """
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")

        # 从配置文件读取基本参数
        self.data_dir = os.path.expanduser(
            config.get("data_dir", DEFAULT_DATA_DIR)
        )
        self.default_sample_rate = config.getfloat(
            "sample_rate", DEFAULT_SAMPLE_RATE, above=0.1
        )
        self.auto_start = config.getboolean("auto_start", False)

        # 数据源管理
        self.sources = {}  # {name: DataSourceAdapter}
        self.source_configs = []  # 配置列表 [{name, type, ...}, ...]

        # 采集状态变量
        self.is_collecting = False
        self.collection_lock = threading.Lock()
        self.data_buffer = []
        self.sample_timer = None
        self.start_time = 0.0
        self.sample_count = 0

        # 当前使用的采样率（可运行时修改）
        self.current_sample_rate = self.default_sample_rate

        # 注册事件处理器
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.printer.register_event_handler(
            "klippy:shutdown", self._handle_shutdown
        )

        # 解析配置中的数据源定义
        self._parse_source_configs(config)

        # 注册G-code命令
        self._register_commands()

        # 创建数据存储目录
        if not os.path.exists(self.data_dir):
            try:
                os.makedirs(self.data_dir)
            except OSError as e:
                logging.warning("无法创建数据目录: %s", e)

        logging.info(
            "UniversalTempCollector initialized: dir=%s, rate=%.1f Hz",
            self.data_dir,
            self.default_sample_rate,
        )

    def _parse_source_configs(self, config):
        """
        解析配置文件中的数据源定义

        支持两种配置方式：
        1. 内联配置：直接在 [universal_temp_collector] 中定义
           SOURCES="heater:extruder,sensor:sensor_ntc,sensor:sensor_thermo"
        2. 分节配置：使用 [universal_temp_collector SOURCE_xxx] 子节

        参数：
            config: 主配置对象
        """
        # 方式1: 从SOURCES参数解析内联配置
        sources_str = config.get("SOURCES", "")
        if sources_str:
            for source_def in sources_str.split(","):
                source_def = source_def.strip()
                if ":" in source_def and source_def:
                    src_type, src_name = source_def.split(":", 1)
                    src_type = src_type.strip().lower()
                    src_name = src_name.strip()
                    self.source_configs.append(
                        {"name": src_name, "type": src_type, "enabled": True}
                    )
                    logging.info(
                        "  添加内联数据源: %s (%s)", src_name, src_type
                    )

        # 方式2: 扫描子节配置 [universal_temp_collector SOURCE_xxx]
        # 这部分在 _handle_ready 中处理，因为需要等待所有对象加载完成

    def _handle_ready(self):
        """打印机就绪事件处理器 - 初始化所有数据源"""
        pheaters = self.printer.lookup_object("heaters", None)

        # 遍历已注册的数据源配置并创建适配器
        for src_config in self.source_configs:
            name = src_config["name"]
            src_type = src_config["type"].lower()

            if not src_config.get("enabled", True):
                continue

            try:
                adapter = self._create_adapter(name, src_type)
                if adapter:
                    self.sources[name] = adapter
                    logging.info(
                        "成功注册数据源 '%s' (类型: %s)", name, src_type
                    )
            except Exception as e:
                logging.warning("无法注册数据源 '%s': %s", name, e)

        # 如果配置了自动启动且存在有效数据源
        if self.auto_start and self.sources:
            logging.info("自动启动数据采集...")
            self._start_collection()

    def _handle_shutdown(self):
        """打印机关闭事件处理器"""
        self._stop_collection()

    def _create_adapter(self, name, src_type):
        """
        创建数据源适配器

        参数：
            name: 对象名称（如 extruder, sensor_ntc）
            src_type: 对象类型（heater/sensor/heater_generic/temperature_sensor）

        返回：
            DataSourceAdapter 实例或 None
        """
        # 确定要查找的Klipper对象名
        lookup_name = name

        # 根据类型确定工厂类
        factory_class = self.ADAPTER_FACTORIES.get(src_type)
        if factory_class is None:
            raise ValueError(f"未知的数据源类型: {src_type}")

        # 获取Klipper对象
        obj = None
        if src_type in ("heater", "heater_generic"):
            try:
                pheaters = self.printer.lookup_object("heaters")
                obj = pheaters.lookup_heater(name)
            except Exception as e:
                logging.error("找不到加热器 '%s': %s", name, e)
                return None
        elif src_type in ("sensor", "temperature_sensor"):
            try:
                full_name = f"temperature_sensor {name}"
                obj = self.printer.lookup_object(full_name)
            except Exception as e:
                logging.error("找不到温度传感器 '%s': %s", name, e)
                return None

        if obj is None:
            return None

        return factory_class(name, obj)

    def add_source(self, name, src_type):
        """
        运行时动态添加数据源

        参数：
            name: 对象名称
            src_type: 对象类型

        返回：
            bool: 是否添加成功
        """
        if name in self.sources:
            logging.warning("数据源 '%s' 已存在", name)
            return False

        try:
            adapter = self._create_adapter(name, src_type)
            if adapter:
                self.sources[name] = adapter
                self.source_configs.append(
                    {"name": name, "type": src_type, "enabled": True}
                )
                logging.info("动态添加数据源: %s (%s)", name, src_type)
                return True
        except Exception as e:
            logging.error("添加数据源失败: %s", e)

        return False

    def remove_source(self, name):
        """
        移除数据源

        参数：
            name: 要移除的对象名称

        返回：
            bool: 是否移除成功
        """
        if name in self.sources:
            del self.sources[name]

            # 同时从配置列表中移除
            self.source_configs = [
                c for c in self.source_configs if c.get("name") != name
            ]
            logging.info("已移除数据源: %s", name)
            return True
        return False

    def enable_source(self, name, enabled=True):
        """启用/禁用指定数据源的采集"""
        if name in self.sources:
            self.sources[name].enabled = enabled
            return True
        return False

    # =========================================================================
    # 数据采集核心方法 (Data Collection Core Methods)
    # =========================================================================

    def _sample_callback(self, eventtime):
        """
        采样回调函数 - 采集所有数据源的一次快照

        该方法在每个采样时刻被调用，遍历所有启用的数据源，
        采集其温度数据并存入缓冲区。

        参数：
            eventtime: 事件时间戳（reactor单调时间）
        """
        if not self.is_collecting:
            return

        current_time = time.time()

        for name, adapter in self.sources.items():
            if not adapter.enabled:
                continue

            try:
                temp = adapter.get_temperature(eventtime)
                target = adapter.get_target_temperature(eventtime)
                extra = adapter.get_extra_data(eventtime)

                if temp is not None:
                    sample = {
                        "timestamp": current_time,
                        "eventtime": eventtime,
                        "object_name": name,
                        "object_type": adapter.get_object_type(),
                        "temperature": temp,
                        "target_temperature": target
                        if target is not None
                        else "",
                        **extra,
                    }

                    with self.collection_lock:
                        self.data_buffer.append(sample)

                    self.sample_count += 1

            except Exception as e:
                logging.debug("采集数据源 '%s' 失败: %s", name, e)

    def _start_collection(self, experiment_name=None):
        """
        启动数据采集

        参数：
            experiment_name: 实验名称（可选）

        返回：
            bool: 是否成功启动
        """
        with self.collection_lock:
            if self.is_collecting:
                return False

            if not self.sources:
                logging.warning("没有可用的数据源，无法启动采集")
                return False

            self.is_collecting = True
            self.data_buffer = []
            self.start_time = self.reactor.monotonic()
            self.sample_count = 0

        self.sample_timer = self.reactor.register_timer(
            self._sample_callback_timer, self.reactor.NOW
        )

        active_count = sum(1 for s in self.sources.values() if s.enabled)
        logging.info(
            "数据采集已启动: %d 个活跃数据源, 采样率 %.1f Hz",
            active_count,
            self.current_sample_rate,
        )
        return True

    def _stop_collection(self):
        """停止数据采集"""
        self.is_collecting = False

        if self.sample_timer is not None:
            self.reactor.unregister_timer(self.sample_timer)
            self.sample_timer = None

        logging.info("数据采集已停止: 共采集 %d 个样本", self.sample_count)

    def _sample_callback_timer(self, eventtime):
        """采样定时器回调"""
        self._sample_callback(eventtime)

        if self.is_collecting:
            return eventtime + (1.0 / self.current_sample_rate)
        return self.reactor.NEVER

    def _save_data_to_csv(self, filename=None):
        """
        将采集数据保存到CSV文件

        参数：
            filename: 输出文件名（不含路径），如果为None则自动生成

        返回：
            tuple: (是否成功, 文件路径或错误信息)
        """
        if not self.data_buffer:
            return False, "无数据可保存"

        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
            filename = f"temp_multi_{timestamp}.csv"

        filepath = os.path.join(self.data_dir, filename)

        try:
            # 收集所有可能的字段名
            all_fields = set()
            for sample in self.data_buffer:
                all_fields.update(sample.keys())

            # 确保字段顺序
            fieldnames = [
                "timestamp",
                "object_name",
                "object_type",
                "temperature",
                "target_temperature",
            ]

            # 添加额外字段（按字母排序）
            extra_fields = sorted(
                [
                    f
                    for f in all_fields
                    if f not in fieldnames and f != "eventtime"
                ]
            )
            fieldnames.extend(extra_fields)

            with open(filepath, "w", newline="", encoding="utf-8") as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()

                start_time = self.data_buffer[0].get("timestamp", 0)

                for sample in self.data_buffer:
                    row = dict(sample)
                    # 计算相对时间
                    if "timestamp" in row and start_time:
                        row["relative_time"] = round(
                            row["timestamp"] - start_time, 6
                        )

                    writer.writerow(row)

            return True, filepath

        except Exception as e:
            error_msg = f"保存CSV数据失败: {e}"
            logging.error(error_msg)
            return False, error_msg

    # =========================================================================
    # G-code 命令处理方法 (G-code Command Handlers)
    # =========================================================================

    def _register_commands(self):
        """注册所有G-code命令"""
        self.gcode.register_command(
            "TEMP_MULTI_COLLECT",
            self.cmd_TEMP_MULTI_COLLECT,
            desc=self.cmd_TEMP_MULTI_COLLECT_help,
        )
        self.gcode.register_command(
            "TEMP_MULTI_STOP",
            self.cmd_TEMP_MULTI_STOP,
            desc=self.cmd_TEMP_MULTI_STOP_help,
        )
        self.gcode.register_command(
            "TEMP_MULTI_STATUS",
            self.cmd_TEMP_MULTI_STATUS,
            desc=self.cmd_TEMP_MULTI_STATUS_help,
        )
        self.gcode.register_command(
            "TEMP_MULTI_ADD_SOURCE",
            self.cmd_TEMP_MULTI_ADD_SOURCE,
            desc=self.cmd_TEMP_MULTI_ADD_SOURCE_help,
        )
        self.gcode.register_command(
            "TEMP_MULTI_REMOVE_SOURCE",
            self.cmd_TEMP_MULTI_REMOVE_SOURCE,
            desc=self.cmd_TEMP_MULTI_REMOVE_SOURCE_help,
        )
        self.gcode.register_command(
            "TEMP_MULTI_LIST_SOURCES",
            self.cmd_TEMP_MULTI_LIST_SOURCES,
            desc=self.cmd_TEMP_MULTI_LIST_SOURCES_help,
        )

    cmd_TEMP_MULTI_COLLECT_help = "启动多对象温度数据采集"

    def cmd_TEMP_MULTI_COLLECT(self, gcmd):
        """
        处理 TEMP_MULTI_COLLECT 命令 - 启动数据采集

        用法：
            TEMP_MULTI_COLLECT [SAMPLE_RATE=<采样率>] [EXPERIMENT=<实验名>]
                          [SOURCES=<对象列表>]

        参数：
            SAMPLE_RATE: 采样频率 (Hz)，默认使用配置值
            EXPERIMENT: 实验名称（用于日志记录）
            SOURCES: 指定要采集的对象（逗号分隔），为空则采集全部

        示例：
            TEMP_MULTI_COLLECT SAMPLE_RATE=10 EXPERIMENT=bed_leveling
            TEMP_MULTI_COLLECT SOURCES=extruder,heater_bed
        """
        sample_rate = gcmd.get_float(
            "SAMPLE_RATE", self.default_sample_rate, above=0.1
        )
        experiment_name = gcmd.get("EXPERIMENT", "manual")
        sources_filter = gcmd.get("SOURCES", "").strip()

        # 应用过滤（如果有指定）
        if sources_filter:
            filter_list = [
                s.strip() for s in sources_filter.split(",") if s.strip()
            ]
            for name, adapter in self.sources.items():
                adapter.enabled = name in filter_list
        else:
            # 如果没有过滤条件，启用所有源
            for adapter in self.sources.values():
                adapter.enabled = True

        self.current_sample_rate = sample_rate

        if self._start_collection(experiment_name):
            active_sources = [n for n, a in self.sources.items() if a.enabled]
            gcmd.respond_info(
                f"多对象温度数据采集已启动:\n"
                f"  采样率: {sample_rate} Hz\n"
                f"  数据源 ({len(active_sources)}个): {', '.join(active_sources)}\n"
                f"  实验: {experiment_name}"
            )
        else:
            gcmd.respond_info("启动失败（可能已在采集中或无可用数据源）")

    cmd_TEMP_MULTI_STOP_help = "停止多对象温度数据采集并保存"

    def cmd_TEMP_MULTI_STOP(self, gcmd):
        """
        处理 TEMP_MULTI_STOP 命令 - 停止采集并保存数据

        用法：
            TEMP_MULTI_STOP [FILENAME=<文件名>] [SHOW_PREVIEW=<0|1>]

        参数：
            FILENAME: 输出文件名，默认自动生成
            SHOW_PREVIEW: 是否显示数据预览摘要，默认 1

        示例：
            TEMP_MULTI_STOP FILENAME=my_experiment.csv
        """
        filename = gcmd.get("FILENAME", "")
        show_preview = gcmd.get_int("SHOW_PREVIEW", 1, minval=0, maxval=1) == 1

        self._stop_collection()

        fname = filename if filename else None
        success, result = self._save_data_to_csv(fname)

        if success:
            gcmd.respond_info(
                f"数据已保存: {result}\n总样本数: {len(self.data_buffer)}"
            )

            if show_preview and self.data_buffer:
                self._show_data_preview(gcmd)
        else:
            gcmd.respond_info(f"保存失败: {result}")

        with self.collection_lock:
            self.data_buffer = []

    def _show_data_preview(self, gcmd):
        """显示数据预览摘要"""
        if not self.data_buffer:
            return

        # 按对象统计
        stats = {}
        for sample in self.data_buffer:
            name = sample["object_name"]
            temp = sample["temperature"]

            if name not in stats:
                stats[name] = {
                    "count": 0,
                    "temps": [],
                    "min": float("inf"),
                    "max": float("-inf"),
                }

            stats[name]["count"] += 1
            stats[name]["temps"].append(temp)
            stats[name]["min"] = min(stats[name]["min"], temp)
            stats[name]["max"] = max(stats[name]["max"], temp)

        preview_lines = ["\n数据预览摘要:"]
        preview_lines.append("-" * 50)

        for name, stat in sorted(stats.items()):
            avg = (
                sum(stat["temps"]) / len(stat["temps"]) if stat["temps"] else 0
            )
            preview_lines.append(
                f"  {name}: "
                f"N={stat['count']}, "
                f"Avg={avg:.1f}°C, "
                f"Min={stat['min']:.1f}°C, "
                f"Max={stat['max']:.1f}°C"
            )

        preview_lines.append("-" * 50)
        preview_lines.append(f"总计: {len(self.data_buffer)} 个样本")

        gcmd.respond_info("\n".join(preview_lines))

    cmd_TEMP_MULTI_STATUS_help = "获取多对象温度采集状态"

    def cmd_TEMP_MULTI_STATUS(self, gcmd):
        """
        处理 TEMP_MULTI_STATUS 命令 - 显示当前状态和实时温度

        用法：
            TEMP_MULTI_STATUS [SHOW_VALUES=<0|1>]

        参数：
            SHOW_VALUES: 是否显示各对象的当前温度值，默认 1
        """
        show_values = gcmd.get_int("SHOW_VALUES", 1, minval=0, maxval=1) == 1

        status_text = ["多对象温度采集器状态:"]
        status_text.append("=" * 50)
        status_text.append(
            f"  采集状态: {'采集中' if self.is_collecting else '空闲'}"
        )
        status_text.append(f"  采样率: {self.current_sample_rate} Hz")
        status_text.append(f"  总样本数: {self.sample_count}")
        status_text.append(f"  数据目录: {self.data_dir}")
        status_text.append(f"  已注册数据源: {len(self.sources)} 个")

        if show_values:
            status_text.append("")
            status_text.append("  各对象当前温度:")
            eventtime = self.reactor.monotonic()

            for name, adapter in sorted(self.sources.items()):
                try:
                    temp = adapter.get_temperature(eventtime)
                    target = adapter.get_target_temperature(eventtime)
                    type_name = adapter.get_object_type()
                    status_icon = "●" if adapter.enabled else "○"

                    if temp is not None:
                        temp_str = f"{temp:.1f}"
                    else:
                        temp_str = "N/A"

                    target_str = (
                        f", 目标: {target:.1f}" if target is not None else ""
                    )

                    status_text.append(
                        f"    {status_icon} [{type_name}] {name}: "
                        f"{temp_str}°C{target_str}"
                    )
                except Exception as e:
                    status_text.append(f"    ○ {name}: 错误 - {e}")

        gcmd.respond_info("\n".join(status_text))

    cmd_TEMP_MULTI_ADD_SOURCE_help = "动态添加数据源"

    def cmd_TEMP_MULTI_ADD_SOURCE(self, gcmd):
        """
        处理 TEMP_MULTI_ADD_SOURCE 命令 - 动态添加数据源

        用法：
            TEMP_MULTI_ADD_SOURCE NAME=<对象名> TYPE=<类型>

        参数：
            NAME: Klipper对象名称（如 extruder, sensor_ntc）
            TYPE: 对象类型 (heater / sensor / heater_generic / temperature_sensor)

        示例：
            TEMP_MULTI_ADD_SOURCE NAME=sensor_chamber TYPE=sensor
            TEMP_MULTI_ADD_SOURCE NAME=heater_bed TYPE=heater
        """
        name = gcmd.get("NAME", "")
        src_type = gcmd.get("TYPE", "").lower()

        if not name or not src_type:
            raise gcmd.error("必须指定 NAME 和 TYPE 参数")

        valid_types = list(self.ADAPTER_FACTORIES.keys())
        if src_type not in valid_types:
            raise gcmd.error(
                f"无效的类型 '{src_type}'，有效类型: {valid_types}"
            )

        if self.add_source(name, src_type):
            gcmd.respond_info(f"已添加数据源: {name} ({src_type})")
        else:
            gcmd.respond_info(f"添加失败: {name} 可能已存在或不可用")

    cmd_TEMP_MULTI_REMOVE_SOURCE_help = "移除数据源"

    def cmd_TEMP_MULTI_REMOVE_SOURCE(self, gcmd):
        """
        处理 TEMP_MULTI_REMOVE_SOURCE 命令 - 移除数据源

        用法：
            TEMP_MULTI_REMOVE_SOURCE NAME=<对象名>

        示例：
            TEMP_MULTI_REMOVE_SOURCE NAME=sensor_chamber
        """
        name = gcmd.get("NAME", "")

        if not name:
            raise gcmd.error("必须指定 NAME 参数")

        if self.remove_source(name):
            gcmd.respond_info(f"已移除数据源: {name}")
        else:
            gcmd.respond_info(f"移除失败: {name} 不存在")

    cmd_TEMP_MULTI_LIST_SOURCES_help = "列出所有已注册的数据源"

    def cmd_TEMP_MULTI_LIST_SOURCES(self, gcmd):
        """
        处理 TEMP_MULTI_LIST_SOURCES 命令 - 列出所有数据源
        """
        lines = ["已注册的数据源:"]
        lines.append("-" * 60)
        lines.append(f"{'名称':<25} {'类型':<12} {'状态':<8}")
        lines.append("-" * 60)

        for name, adapter in sorted(self.sources.items()):
            status = "启用" if adapter.disabled else "禁用"
            type_name = adapter.get_object_type()
            lines.append(f"{name:<25} {type_name:<12} {status:<8}")

        lines.append("-" * 60)
        lines.append(f"总计: {len(self.sources)} 个数据源")

        gcmd.respond_info("\n".join(lines))

    def get_status(self, eventtime):
        """
        获取模块状态信息（供API调用）

        返回：
            dict: 状态信息字典
        """
        source_statuses = {}
        for name, adapter in self.sources.items():
            source_statuses[name] = {
                "type": adapter.get_object_type(),
                "enabled": adapter.enabled,
                "temperature": adapter.get_temperature(eventtime),
                "target": adapter.get_target_temperature(eventtime),
            }

        return {
            "is_collecting": self.is_collecting,
            "sample_rate": self.current_sample_rate,
            "total_samples": self.sample_count,
            "data_directory": self.data_dir,
            "source_count": len(self.sources),
            "sources": source_statuses,
        }


# =============================================================================
# 模块加载入口 (Module Entry Points)
# =============================================================================


def load_config(config):
    """
    模块加载入口函数

    当配置文件中存在 [universal_temp_collector] 配置节时调用。

    参数：
        config: 配置对象

    返回：
        UniversalTempCollector 实例
    """
    return UniversalTempCollector(config)


def load_config_prefix(config):
    """
    带前缀的模块加载入口函数

    当配置文件中存在 [universal_temp_collector xxx] 配置节时调用。
    支持创建多个采集器实例。

    参数：
        config: 配置对象

    返回：
        UniversalTempCollector 实例
    """
    return UniversalTempCollector(config)
