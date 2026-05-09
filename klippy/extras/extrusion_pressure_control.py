# 挤出压力预读控制系统
# Extrusion Pressure Control with GCode Lookahead
#
# Copyright (C) 2024
#
# 本文件遵循 GNU GPLv3 许可证分发。
# This file may be distributed under the terms of the GNU GPLv3 license.

import collections
import logging
import math

######################################################################
# PID 控制器
# PID Controller
######################################################################


class PIDController:
    """
    PID控制器类 - 实现比例-积分-微分控制算法
    
    用于根据压力误差计算调整输出，实现闭环压力控制。
    输出 = Kp*误差 + Ki*积分 + Kd*微分
    """

    def __init__(self, kp, ki, kd, output_min=-float("inf"), output_max=float("inf")):
        """
        初始化PID控制器
        
        参数:
            kp: 比例系数 - 决定对当前误差的响应强度
            ki: 积分系数 - 消除稳态误差
            kd: 微分系数 - 抑制误差变化率，减少超调
            output_min: 输出最小值限制
            output_max: 输出最大值限制
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral = 0.0          # 积分项累积值
        self.last_error = 0.0        # 上一次的误差值
        self.last_time = None        # 上一次更新时间

    def reset(self):
        """重置PID控制器状态，清空积分项和历史数据"""
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None

    def update(self, error, current_time):
        """
        更新PID控制器并计算输出值
        
        参数:
            error: 当前误差值 (目标值 - 实际值)
            current_time: 当前时间戳
            
        返回:
            控制输出值，用于调整挤出因子
        """
        # 首次调用时初始化时间
        if self.last_time is None:
            self.last_time = current_time
            self.last_error = error
            return 0.0
            
        # 计算时间间隔
        dt = current_time - self.last_time
        if dt <= 0:
            return 0.0
            
        # 计算比例项: Kp * 误差
        p_term = self.kp * error
        
        # 更新积分项并限制范围防止积分饱和
        self.integral += error * dt
        self.integral = max(-100, min(100, self.integral))  # 限制积分范围
        i_term = self.ki * self.integral
        
        # 计算微分项: Kd * (误差变化率)
        d_term = self.kd * (error - self.last_error) / dt
        
        # 计算总输出
        output = p_term + i_term + d_term
        
        # 限制输出范围
        output = max(self.output_min, min(self.output_max, output))
        
        # 保存当前状态用于下次计算
        self.last_time = current_time
        self.last_error = error
        
        return output


######################################################################
# 压力-速度模型
# Pressure-Velocity Model
######################################################################


class PressureVelocityModel:
    """
    压力-速度关系模型类
    
    建立挤出速度与挤出压力之间的数学关系模型，支持:
    1. 理论模型计算 - 基于流体力学原理
    2. 经验多项式模型 - 基于实验数据拟合
    3. 查表法 - 基于校准数据
    4. 自适应学习 - 根据实际误差动态调整
    """
    
    # 预定义材料参数数据库
    # 包含不同材料的粘度参考值、参考温度、温度系数和压力系数
    MATERIAL_PARAMS = {
        "PLA": {
            "viscosity_ref": 1000,        # 参考粘度 (Pa·s)
            "temp_ref": 210,              # 参考温度 (°C)
            "temp_coefficient": 0.03,     # 温度粘度系数
            "pressure_coefficient": 1.0,  # 压力系数
        },
        "PETG": {
            "viscosity_ref": 1500,
            "temp_ref": 240,
            "temp_coefficient": 0.025,
            "pressure_coefficient": 1.2,
        },
        "ABS": {
            "viscosity_ref": 2000,
            "temp_ref": 250,
            "temp_coefficient": 0.02,
            "pressure_coefficient": 1.3,
        },
        "TPU": {
            "viscosity_ref": 3000,
            "temp_ref": 230,
            "temp_coefficient": 0.035,
            "pressure_coefficient": 1.5,
        },
        "NYLON": {
            "viscosity_ref": 2500,
            "temp_ref": 260,
            "temp_coefficient": 0.018,
            "pressure_coefficient": 1.4,
        },
    }

    def __init__(self, config):
        """
        从配置初始化压力-速度模型
        
        参数:
            config: Klipper配置对象
        """
        self.printer = config.get_printer()
        
        # 喷嘴参数
        self.nozzle_diameter = config.getfloat("nozzle_diameter", 0.4)  # 喷嘴直径
        self.nozzle_length = config.getfloat("nozzle_length", 2.0)      # 喷嘴长度
        
        # 材料配置
        self.material_type = config.get("material_type", "PLA")
        self.material_params = self.MATERIAL_PARAMS.get(
            self.material_type, self.MATERIAL_PARAMS["PLA"]
        )
        
        # 压力模型参数
        self.pressure_coefficient = config.getfloat(
            "pressure_coefficient", self.material_params["pressure_coefficient"]
        )
        self.base_pressure = config.getfloat("base_pressure", 5.0)      # 基础压力
        self.max_pressure = config.getfloat("max_pressure", 50.0)       # 最大压力
        
        # 多项式经验模型系数: P = a0 + a1*v + a2*v²
        self.poly_coefficients = [
            config.getfloat("poly_coef_a0", 2.0),
            config.getfloat("poly_coef_a1", 0.5),
            config.getfloat("poly_coef_a2", 0.01),
        ]
        
        # 查表法配置
        self.use_lookup_table = config.getboolean("use_lookup_table", False)
        self.lookup_table = self._build_lookup_table(config)
        
        # 校准数据表 (运行时填充)
        self.calibration_table = {}
        
        # 自适应补偿参数
        self.model_correction_factor = 1.0   # 模型修正因子
        self.bias_compensation = 0.0         # 偏差补偿值

    def _build_lookup_table(self, config):
        """
        从配置构建压力查表
        
        配置格式: "速度1:压力1, 速度2:压力2, ..."
        例如: "10:5.0, 20:8.0, 30:12.0"
        """
        table_str = config.get("pressure_lookup_table", None)
        if table_str is None:
            return None
            
        table = []
        for entry in table_str.split(","):
            entry = entry.strip()
            if ":" in entry:
                v, p = entry.split(":")
                table.append((float(v), float(p)))
                
        # 按速度排序
        return sorted(table, key=lambda x: x[0]) if table else None

    def calculate_target_pressure(
        self, velocity, extrude_rate, temperature=None, use_calibration=True
    ):
        """
        计算目标挤出压力
        
        参数:
            velocity: 打印速度
            extrude_rate: 挤出速率
            temperature: 挤出温度 (可选，默认使用材料参考温度)
            use_calibration: 是否使用校准数据
            
        返回:
            计算得到的目标压力值
        """
        # 优先使用校准数据 (最准确)
        if use_calibration and self.calibration_table:
            calibrated_pressure = self._lookup_calibration(extrude_rate)
            if calibrated_pressure is not None:
                return calibrated_pressure
                
        # 其次使用查表法
        if self.use_lookup_table and self.lookup_table:
            base_pressure = self._lookup_pressure(extrude_rate)
        else:
            # 使用理论模型计算
            base_pressure = self._calculate_theoretical_pressure(
                velocity, extrude_rate, temperature
            )
            
        # 应用自适应补偿
        adjusted_pressure = (
            base_pressure * self.model_correction_factor + self.bias_compensation
        )
        
        # 限制压力范围
        return max(self.base_pressure, min(self.max_pressure, adjusted_pressure))

    def _calculate_theoretical_pressure(self, velocity, extrude_rate, temperature):
        """
        使用理论模型计算压力
        
        基于流体力学原理，考虑材料粘度、温度等因素
        """
        # 使用参考温度如果未指定
        if temperature is None:
            temperature = self.material_params["temp_ref"]
            
        # 计算温度修正因子 (温度越高，粘度越低)
        temp_diff = temperature - self.material_params["temp_ref"]
        temp_factor = math.exp(-self.material_params["temp_coefficient"] * temp_diff)
        
        # 理论压力计算 (简化模型)
        theoretical_pressure = (
            self.pressure_coefficient
            * extrude_rate
            * self.material_params["viscosity_ref"]
            * temp_factor
            / 1000.0
        )
        
        # 经验多项式修正
        empirical_correction = (
            self.poly_coefficients[0]
            + self.poly_coefficients[1] * extrude_rate
            + self.poly_coefficients[2] * extrude_rate ** 2
        )
        
        # 综合理论值和经验值 (加权平均)
        target_pressure = theoretical_pressure * 0.5 + empirical_correction * 0.5
        
        return target_pressure

    def _lookup_pressure(self, extrude_rate):
        """
        从预定义查表中查找压力值
        
        使用线性插值处理查表之间的值
        """
        if not self.lookup_table:
            return self.base_pressure
            
        # 边界处理
        if extrude_rate <= self.lookup_table[0][0]:
            return self.lookup_table[0][1]
        if extrude_rate >= self.lookup_table[-1][0]:
            return self.lookup_table[-1][1]
            
        # 线性插值查找
        for i in range(len(self.lookup_table) - 1):
            v1, p1 = self.lookup_table[i]
            v2, p2 = self.lookup_table[i + 1]
            if v1 <= extrude_rate <= v2:
                t = (extrude_rate - v1) / (v2 - v1) if v2 != v1 else 0
                return p1 + t * (p2 - p1)
                
        return self.base_pressure

    def _lookup_calibration(self, extrude_rate):
        """
        从校准数据表中查找压力值
        
        使用线性插值处理校准点之间的值
        """
        if not self.calibration_table:
            return None
            
        sorted_keys = sorted(self.calibration_table.keys())
        
        # 边界处理
        if extrude_rate <= sorted_keys[0]:
            return self.calibration_table[sorted_keys[0]]
        if extrude_rate >= sorted_keys[-1]:
            return self.calibration_table[sorted_keys[-1]]
            
        # 线性插值查找
        for i in range(len(sorted_keys) - 1):
            v1, v2 = sorted_keys[i], sorted_keys[i + 1]
            if v1 <= extrude_rate <= v2:
                p1, p2 = self.calibration_table[v1], self.calibration_table[v2]
                t = (extrude_rate - v1) / (v2 - v1) if v2 != v1 else 0
                return p1 + t * (p2 - p1)
                
        return None

    def add_calibration_point(self, extrude_rate, pressure):
        """
        添加校准数据点
        
        参数:
            extrude_rate: 挤出速率
            pressure: 对应的压力值
        """
        self.calibration_table[extrude_rate] = pressure
        logging.info(
            "添加压力校准点: 挤出速率=%.2f, 压力=%.2f"
            % (extrude_rate, pressure)
        )

    def clear_calibration(self):
        """清除所有校准数据和自适应补偿参数"""
        self.calibration_table = {}
        self.model_correction_factor = 1.0
        self.bias_compensation = 0.0
        logging.info("压力校准数据已清除")

    def update_material(self, material_type):
        """
        更新材料类型
        
        参数:
            material_type: 材料类型 (PLA/PETG/ABS/TPU/NYLON)
        """
        if material_type in self.MATERIAL_PARAMS:
            self.material_type = material_type
            self.material_params = self.MATERIAL_PARAMS[material_type]
            self.pressure_coefficient = self.material_params["pressure_coefficient"]
            logging.info("压力模型材料已更新: %s" % material_type)

    def apply_adaptive_compensation(self, error, learning_rate=0.01):
        """
        应用自适应补偿
        
        根据误差动态调整模型参数，实现自学习优化
        
        参数:
            error: 当前压力误差
            learning_rate: 学习率
        """
        # 更新偏差补偿
        self.bias_compensation += error * learning_rate
        self.bias_compensation = max(-5.0, min(5.0, self.bias_compensation))
        
        # 当误差较大时调整模型修正因子
        if abs(error) > 1.0:
            correction = 1.0 + error * learning_rate * 0.1
            self.model_correction_factor *= correction
            self.model_correction_factor = max(0.5, min(2.0, self.model_correction_factor))

    def get_status(self, eventtime):
        """获取模型状态信息，用于Webhooks API查询"""
        return {
            "material_type": self.material_type,
            "pressure_coefficient": self.pressure_coefficient,
            "base_pressure": self.base_pressure,
            "max_pressure": self.max_pressure,
            "calibration_points": len(self.calibration_table),
            "model_correction_factor": self.model_correction_factor,
            "bias_compensation": self.bias_compensation,
        }


######################################################################
# GCode 预读解析器
# GCode Lookahead Reader
######################################################################


class GCodeLookaheadReader:
    """
    GCode预读解析器类
    
    通过监听Klipper的移动队列事件，提前读取后续移动段的信息，
    提取速度参数用于前馈控制。
    """
    
    def __init__(self, config):
        """
        从配置初始化预读解析器
        
        参数:
            config: Klipper配置对象
        """
        self.printer = config.get_printer()
        
        # 预读配置
        self.lookahead_depth = config.getint("lookahead_depth", 20, minval=5, maxval=100)
        self.move_buffer = collections.deque(maxlen=self.lookahead_depth)
        
        # 速度变化检测阈值
        self.velocity_change_threshold = config.getfloat(
            "velocity_change_threshold", 5.0, minval=1.0, maxval=50.0
        )
        
        # 当前状态
        self.last_velocity = 0.0
        self.last_extrude_rate = 0.0
        self.pending_adjustments = []
        
        # 注册事件处理器
        self.printer.register_event_handler("klippy:ready", self._handle_ready)

    def _handle_ready(self):
        """Klipper就绪后获取必要对象的引用"""
        self.toolhead = self.printer.lookup_object("toolhead")
        self.gcode_move = self.printer.lookup_object("gcode_move")

    def on_move_queued(self, move):
        """
        移动段入队事件处理
        
        当新的移动段加入队列时被调用，提取速度信息并检测速度变化
        
        参数:
            move: 移动对象
            
        返回:
            如果检测到显著速度变化，返回包含速度信息的字典；否则返回None
        """
        # 忽略非运动移动
        if not move.is_kinematic_move:
            return None
            
        # 提取移动信息
        move_info = self._extract_move_info(move)
        self.move_buffer.append(move_info)
        
        # 检测速度变化
        velocity_change = abs(move_info["velocity"] - self.last_velocity)
        
        result = None
        if velocity_change > self.velocity_change_threshold:
            # 速度变化超过阈值，返回调整信息
            result = {
                "velocity": move_info["velocity"],
                "extrude_rate": move_info["extrude_rate"],
                "velocity_change": velocity_change,
                "move_d": move_info["move_d"],
            }
            
        # 更新当前状态
        self.last_velocity = move_info["velocity"]
        self.last_extrude_rate = move_info["extrude_rate"]
        
        return result

    def _extract_move_info(self, move):
        """
        从移动对象中提取关键信息
        
        参数:
            move: 移动对象
            
        返回:
            包含移动信息的字典
        """
        axes_d = move.axes_d          # 各轴移动距离
        axes_r = move.axes_r          # 各轴移动比率
        move_d = move.move_d          # 总移动距离
        
        # 计算巡航速度 (最大稳定速度)
        cruise_velocity = math.sqrt(move.max_cruise_v2)
        
        # 计算挤出比率和挤出速率
        extrude_ratio = axes_r[3] if move_d > 0 else 0.0
        extrude_rate = cruise_velocity * extrude_ratio
        
        return {
            "move_d": move_d,
            "velocity": cruise_velocity,
            "extrude_ratio": extrude_ratio,
            "extrude_rate": extrude_rate,
            "axes_d": list(axes_d),
            "start_pos": move.start_pos,
            "end_pos": move.end_pos,
            "accel": move.accel,
            "min_move_t": move.min_move_t,
        }

    def get_current_extrude_rate(self):
        """获取当前挤出速率"""
        return self.last_extrude_rate

    def get_current_velocity(self):
        """获取当前打印速度"""
        return self.last_velocity

    def get_lookahead_info(self):
        """获取预读状态信息"""
        return {
            "buffer_size": len(self.move_buffer),
            "last_velocity": self.last_velocity,
            "last_extrude_rate": self.last_extrude_rate,
            "pending_adjustments": len(self.pending_adjustments),
        }

    def get_status(self, eventtime):
        """获取状态信息，用于Webhooks API查询"""
        return self.get_lookahead_info()


######################################################################
# 反馈验证器
# Feedback Validator
######################################################################


class FeedbackValidator:
    """
    反馈验证器类
    
    监测压力控制效果，提供:
    1. 误差统计 - 平均误差、标准差、最大/最小误差
    2. 趋势分析 - 判断控制效果是否改善或恶化
    3. 阈值检测 - 警告和严重错误检测
    """
    
    def __init__(self, config):
        """
        从配置初始化反馈验证器
        
        参数:
            config: Klipper配置对象
        """
        self.printer = config.get_printer()
        
        # 历史记录配置
        self.history_size = config.getint("history_size", 100, minval=10, maxval=1000)
        self.error_history = collections.deque(maxlen=self.history_size)
        
        # 阈值配置
        self.warning_threshold = config.getfloat("warning_threshold", 2.0)
        self.critical_threshold = config.getfloat("critical_threshold", 5.0)
        
        # 学习参数
        self.learning_rate = config.getfloat("learning_rate", 0.01)
        
        # 趋势分析参数
        self.trend_window = config.getint("trend_window", 10)
        self.trend_threshold = config.getfloat("trend_threshold", 0.5)
        
        # 统计数据
        self.stats = {
            "total_samples": 0,
            "sum_error": 0.0,
            "sum_error_sq": 0.0,
            "max_error": 0.0,
            "min_error": 0.0,
        }

    def record_sample(self, target, actual, error, timestamp):
        """
        记录采样数据
        
        参数:
            target: 目标压力
            actual: 实际压力
            error: 误差值
            timestamp: 时间戳
        """
        sample = {
            "target": target,
            "actual": actual,
            "error": error,
            "timestamp": timestamp,
        }
        self.error_history.append(sample)
        
        # 更新统计数据
        self.stats["total_samples"] += 1
        self.stats["sum_error"] += error
        self.stats["sum_error_sq"] += error ** 2
        self.stats["max_error"] = max(self.stats["max_error"], error)
        self.stats["min_error"] = min(self.stats["min_error"], error)

    def analyze_trend(self):
        """
        分析误差趋势
        
        使用线性回归计算误差变化趋势
        
        返回:
            "improving" - 误差在减小
            "worsening" - 误差在增大
            "stable" - 误差稳定
        """
        if len(self.error_history) < self.trend_window:
            return "stable"
            
        # 获取最近的误差数据
        recent_errors = [s["error"] for s in list(self.error_history)[-self.trend_window:]]
        
        # 线性回归计算斜率
        n = len(recent_errors)
        x_mean = (n - 1) / 2.0
        y_mean = sum(recent_errors) / n
        
        numerator = sum(
            (i - x_mean) * (recent_errors[i] - y_mean) for i in range(n)
        )
        denominator = sum((i - x_mean) ** 2 for i in range(n))
        
        if denominator == 0:
            return "stable"
            
        slope = numerator / denominator
        
        # 根据斜率判断趋势
        if slope < -self.trend_threshold:
            return "improving"
        elif slope > self.trend_threshold:
            return "worsening"
        else:
            return "stable"

    def check_thresholds(self, error):
        """
        检查误差是否超过阈值
        
        参数:
            error: 误差值
            
        返回:
            "critical" - 超过严重阈值
            "warning" - 超过警告阈值
            "normal" - 正常范围
        """
        abs_error = abs(error)
        if abs_error > self.critical_threshold:
            return "critical"
        elif abs_error > self.warning_threshold:
            return "warning"
        return "normal"

    def get_statistics(self):
        """
        获取统计数据
        
        返回:
            包含统计信息的字典，或None(如果无数据)
        """
        if self.stats["total_samples"] == 0:
            return None
            
        n = self.stats["total_samples"]
        mean_error = self.stats["sum_error"] / n
        
        # 计算方差和标准差
        variance = (self.stats["sum_error_sq"] / n) - (mean_error ** 2)
        std_error = math.sqrt(max(0, variance))
        
        return {
            "total_samples": n,
            "mean_error": mean_error,
            "std_error": std_error,
            "max_error": self.stats["max_error"],
            "min_error": self.stats["min_error"],
            "rms_error": math.sqrt(self.stats["sum_error_sq"] / n),
        }

    def reset_statistics(self):
        """重置所有统计数据"""
        self.error_history.clear()
        self.stats = {
            "total_samples": 0,
            "sum_error": 0.0,
            "sum_error_sq": 0.0,
            "max_error": 0.0,
            "min_error": 0.0,
        }

    def get_status(self, eventtime):
        """获取状态信息，用于Webhooks API查询"""
        stats = self.get_statistics()
        return {
            "statistics": stats,
            "current_trend": self.analyze_trend(),
            "history_size": len(self.error_history),
        }


######################################################################
# 校准辅助器
# Calibration Helper
######################################################################


class PressureCalibrationHelper:
    """
    压力校准辅助器类
    
    提供自动压力校准功能，通过在不同速度下测量压力
    建立速度-压力关系模型。
    """
    
    def __init__(self, config):
        """
        从配置初始化校准辅助器
        
        参数:
            config: Klipper配置对象
        """
        self.printer = config.get_printer()
        self.config = config
        
        # 校准速度列表 (mm/s)
        self.calibration_speeds = config.get(
            "calibration_speeds", "10,20,30,40,50,60"
        ).split(",")
        self.calibration_speeds = [float(s.strip()) for s in self.calibration_speeds]
        
        # 校准参数
        self.calibration_duration = config.getfloat("calibration_duration", 5.0, minval=1.0)
        self.calibration_distance = config.getfloat("calibration_distance", 50.0, minval=10.0)
        
        # 校准状态
        self.is_calibrating = False
        self.calibration_index = 0
        self.calibration_data = []

    def start_calibration(self, pressure_control):
        """
        启动自动校准流程
        
        参数:
            pressure_control: 压力控制主对象
        """
        if self.is_calibrating:
            return
            
        self.is_calibrating = True
        self.calibration_index = 0
        self.calibration_data = []
        self.pressure_control = pressure_control
        
        # 开始第一步校准
        self._run_next_calibration_step()

    def _run_next_calibration_step(self):
        """执行下一个校准步骤"""
        # 检查是否完成所有速度的校准
        if self.calibration_index >= len(self.calibration_speeds):
            self._finish_calibration()
            return
            
        # 获取当前校准速度
        speed = self.calibration_speeds[self.calibration_index]
        
        # 开始校准步骤
        self.pressure_control.start_calibration_step(speed)
        
        # 注册定时回调收集数据
        reactor = self.printer.get_reactor()
        reactor.register_callback(
            lambda e: self._collect_calibration_data(speed, e),
            reactor.monotonic() + self.calibration_duration,
        )

    def _collect_calibration_data(self, speed, eventtime):
        """
        收集校准数据
        
        参数:
            speed: 当前校准速度
            eventtime: 事件时间
        """
        if not self.is_calibrating:
            return
            
        # 获取平均压力和挤出速率
        avg_pressure = self.pressure_control.get_average_pressure()
        extrude_rate = self.pressure_control.get_current_extrude_rate()
        
        # 记录校准数据
        self.calibration_data.append(
            {"speed": speed, "extrude_rate": extrude_rate, "pressure": avg_pressure}
        )
        
        # 结束当前校准步骤
        self.pressure_control.end_calibration_step()
        
        # 移动到下一个速度
        self.calibration_index += 1
        self._run_next_calibration_step()

    def _finish_calibration(self):
        """完成校准，将数据写入模型"""
        self.is_calibrating = False
        
        # 获取压力模型并清除旧校准数据
        model = self.pressure_control.pressure_model
        model.clear_calibration()
        
        # 添加所有校准点
        for data in self.calibration_data:
            model.add_calibration_point(data["extrude_rate"], data["pressure"])
            
        # 输出完成信息
        self.pressure_control.gcode.respond_info(
            "压力校准完成，共 %d 个校准点"
            % len(self.calibration_data)
        )


######################################################################
# 挤出压力控制主类
# Main Pressure Control Class
######################################################################


class ExtrusionPressureControl:
    """
    挤出压力控制主类
    
    整合所有组件，实现完整的挤出压力闭环控制:
    - GCode预读解析
    - 压力-速度模型
    - PID闭环控制
    - 前馈补偿
    - 反馈验证
    - 自动校准
    """
    
    def __init__(self, config):
        """
        从配置初始化压力控制系统
        
        参数:
            config: Klipper配置对象
        """
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name().split()[-1]
        
        # 初始化各组件
        self.pressure_model = PressureVelocityModel(config)
        self.lookahead_reader = GCodeLookaheadReader(config)
        self.feedback_validator = FeedbackValidator(config)
        self.calibration_helper = PressureCalibrationHelper(config)
        
        # PID参数
        self.pid_kp = config.getfloat("pid_kp", 0.5)
        self.pid_ki = config.getfloat("pid_ki", 0.1)
        self.pid_kd = config.getfloat("pid_kd", 0.02)
        
        # 创建PID控制器
        self.pid_controller = PIDController(
            kp=self.pid_kp,
            ki=self.pid_ki,
            kd=self.pid_kd,
            output_min=-20.0,
            output_max=20.0,
        )
        
        # 控制参数
        self.target_pressure = config.getfloat("target_pressure", 10.0)
        self.pressure_tolerance = config.getfloat("pressure_tolerance", 0.5)
        self.adjustment_method = config.get("adjustment_method", "extrude_factor")
        self.min_adjustment_interval = config.getfloat("min_adjustment_interval", 0.05)
        
        # 前馈控制参数
        self.use_feedforward = config.getboolean("use_feedforward", True)
        self.feedforward_gain = config.getfloat("feedforward_gain", 0.1)
        
        # 控制周期
        self.control_period = config.getfloat("control_period", 0.01)
        
        # 运行状态
        self.enabled = False
        self.control_mode = config.get("control_mode", "closed_loop")
        self.current_pressure = 0.0
        self.current_extrude_factor = 1.0
        self.last_adjustment_time = 0.0
        
        # 压力采样缓冲
        self.pressure_samples = collections.deque(maxlen=100)
        
        # 校准状态
        self.is_calibration_step = False
        self.calibration_pressure_samples = []
        
        # 注册事件处理器
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.printer.register_event_handler("toolhead:move_queued", self._on_move_queued)
        
        # 注册GCode命令
        self._register_commands()

    def _handle_ready(self):
        """Klipper就绪后获取必要对象的引用"""
        self.toolhead = self.printer.lookup_object("toolhead")
        self.gcode_move = self.printer.lookup_object("gcode_move")
        self.gcode = self.printer.lookup_object("gcode")
        self.load_cell = self.printer.lookup_object("load_cell", None)
        self.extruder = self.printer.lookup_object("extruder", None)
        
        # 启动控制循环定时器
        self.control_timer = self.reactor.register_timer(
            self._control_loop, self.reactor.NOW
        )

    def _on_move_queued(self, move):
        """
        移动段入队事件处理
        
        参数:
            move: 移动对象
        """
        if not self.enabled:
            return
            
        # 调用预读解析器处理移动
        move_info = self.lookahead_reader.on_move_queued(move)
        
        # 如果检测到速度变化且启用前馈，应用前馈控制
        if move_info and self.use_feedforward:
            self._apply_feedforward(move_info)

    def _register_commands(self):
        """注册所有GCode命令"""
        gcode = self.printer.lookup_object("gcode")
        
        # 注册带参数的命令
        gcode.register_mux_command(
            "PRESSURE_CONTROL_ENABLE",
            "PRESSURE_CONTROL",
            self.name,
            self.cmd_ENABLE,
            desc=self.cmd_ENABLE_help,
        )
        gcode.register_mux_command(
            "PRESSURE_CONTROL_DISABLE",
            "PRESSURE_CONTROL",
            self.name,
            self.cmd_DISABLE,
            desc=self.cmd_DISABLE_help,
        )
        gcode.register_mux_command(
            "SET_TARGET_PRESSURE",
            "PRESSURE_CONTROL",
            self.name,
            self.cmd_SET_TARGET,
            desc=self.cmd_SET_TARGET_help,
        )
        gcode.register_mux_command(
            "PRESSURE_CONTROL_STATUS",
            "PRESSURE_CONTROL",
            self.name,
            self.cmd_STATUS,
            desc=self.cmd_STATUS_help,
        )
        gcode.register_mux_command(
            "PRESSURE_CALIBRATE",
            "PRESSURE_CONTROL",
            self.name,
            self.cmd_CALIBRATE,
            desc=self.cmd_CALIBRATE_help,
        )
        gcode.register_mux_command(
            "PRESSURE_ADD_CALIBRATION_POINT",
            "PRESSURE_CONTROL",
            self.name,
            self.cmd_ADD_CALIBRATION_POINT,
            desc=self.cmd_ADD_CALIBRATION_POINT_help,
        )
        gcode.register_mux_command(
            "PRESSURE_CLEAR_CALIBRATION",
            "PRESSURE_CONTROL",
            self.name,
            self.cmd_CLEAR_CALIBRATION,
            desc=self.cmd_CLEAR_CALIBRATION_help,
        )
        gcode.register_mux_command(
            "SET_PRESSURE_PID",
            "PRESSURE_CONTROL",
            self.name,
            self.cmd_SET_PID,
            desc=self.cmd_SET_PID_help,
        )
        gcode.register_mux_command(
            "SET_PRESSURE_MATERIAL",
            "PRESSURE_CONTROL",
            self.name,
            self.cmd_SET_MATERIAL,
            desc=self.cmd_SET_MATERIAL_help,
        )
        gcode.register_mux_command(
            "PRESSURE_RESET_STATS",
            "PRESSURE_CONTROL",
            self.name,
            self.cmd_RESET_STATS,
            desc=self.cmd_RESET_STATS_help,
        )
        
        # 如果是默认名称，也注册无参数版本
        if self.name == "extrusion_pressure_control":
            gcode.register_command(
                "PRESSURE_CONTROL_ENABLE",
                self.cmd_ENABLE,
                desc=self.cmd_ENABLE_help,
            )
            gcode.register_command(
                "PRESSURE_CONTROL_DISABLE",
                self.cmd_DISABLE,
                desc=self.cmd_DISABLE_help,
            )

    cmd_ENABLE_help = "启用挤出压力控制"

    def cmd_ENABLE(self, gcmd):
        """启用挤出压力控制"""
        self.enabled = True
        self.pid_controller.reset()
        gcmd.respond_info("挤出压力控制已启用")

    cmd_DISABLE_help = "禁用挤出压力控制"

    def cmd_DISABLE(self, gcmd):
        """禁用挤出压力控制"""
        self.enabled = False
        # 恢复默认挤出因子
        self.gcode_move.extrude_factor = 1.0
        self.current_extrude_factor = 1.0
        gcmd.respond_info("挤出压力控制已禁用")

    cmd_SET_TARGET_help = "设置目标挤出压力"

    def cmd_SET_TARGET(self, gcmd):
        """设置目标挤出压力"""
        self.target_pressure = gcmd.get_float("PRESSURE", self.target_pressure)
        gcmd.respond_info("目标压力已设置为 %.2f" % self.target_pressure)

    cmd_STATUS_help = "获取压力控制状态"

    def cmd_STATUS(self, gcmd):
        """获取并显示压力控制系统状态"""
        status = self.get_status(self.reactor.monotonic())
        stats = status.get("statistics", {})
        
        gcmd.respond_info(
            "挤出压力控制系统状态:\n"
            "  启用状态: %s\n"
            "  控制模式: %s\n"
            "  目标压力: %.2f\n"
            "  当前压力: %.2f\n"
            "  压力误差: %.2f\n"
            "  挤出因子: %.3f\n"
            "  当前速度: %.1f mm/s\n"
            "  挤出速率: %.2f mm/s\n"
            "  校准点数: %d\n"
            "  误差趋势: %s\n"
            "  平均误差: %.3f\n"
            "  RMS误差: %.3f"
            % (
                status["enabled"],
                status["control_mode"],
                status["target_pressure"],
                status["current_pressure"],
                status["error"],
                status["extrude_factor"],
                status["velocity"],
                status["extrude_rate"],
                status["calibration_points"],
                status["trend"],
                stats.get("mean_error", 0),
                stats.get("rms_error", 0),
            )
        )

    cmd_CALIBRATE_help = "启动自动压力校准"

    def cmd_CALIBRATE(self, gcmd):
        """启动自动压力校准程序"""
        if self.is_calibration_step:
            gcmd.respond_info("校准已在进行中")
            return
            
        gcmd.respond_info("开始压力校准...")
        self.calibration_helper.start_calibration(self)

    cmd_ADD_CALIBRATION_POINT_help = "手动添加校准点"

    def cmd_ADD_CALIBRATION_POINT(self, gcmd):
        """手动添加校准数据点"""
        extrude_rate = gcmd.get_float("RATE")
        pressure = gcmd.get_float("PRESSURE")
        self.pressure_model.add_calibration_point(extrude_rate, pressure)
        gcmd.respond_info(
            "已添加校准点: 挤出速率=%.2f, 压力=%.2f"
            % (extrude_rate, pressure)
        )

    cmd_CLEAR_CALIBRATION_help = "清除压力校准数据"

    def cmd_CLEAR_CALIBRATION(self, gcmd):
        """清除所有压力校准数据"""
        self.pressure_model.clear_calibration()
        gcmd.respond_info("压力校准数据已清除")

    cmd_SET_PID_help = "设置压力控制PID参数"

    def cmd_SET_PID(self, gcmd):
        """设置PID控制参数"""
        self.pid_kp = gcmd.get_float("KP", self.pid_kp)
        self.pid_ki = gcmd.get_float("KI", self.pid_ki)
        self.pid_kd = gcmd.get_float("KD", self.pid_kd)
        
        # 重新创建PID控制器
        self.pid_controller = PIDController(
            kp=self.pid_kp,
            ki=self.pid_ki,
            kd=self.pid_kd,
            output_min=-20.0,
            output_max=20.0,
        )
        
        gcmd.respond_info(
            "PID参数已设置: KP=%.3f, KI=%.3f, KD=%.3f"
            % (self.pid_kp, self.pid_ki, self.pid_kd)
        )

    cmd_SET_MATERIAL_help = "设置材料类型"

    def cmd_SET_MATERIAL(self, gcmd):
        """设置材料类型"""
        material = gcmd.get("MATERIAL", "PLA").upper()
        self.pressure_model.update_material(material)
        gcmd.respond_info("材料已设置为: %s" % material)

    cmd_RESET_STATS_help = "重置压力控制统计数据"

    def cmd_RESET_STATS(self, gcmd):
        """重置统计数据"""
        self.feedback_validator.reset_statistics()
        gcmd.respond_info("压力控制统计数据已重置")

    def _control_loop(self, eventtime):
        """
        主控制循环
        
        定时执行，实现闭环压力控制
        
        参数:
            eventtime: 事件时间
            
        返回:
            下次执行时间
        """
        # 未启用时降低检查频率
        if not self.enabled:
            return eventtime + 0.1
            
        # 读取当前压力
        self.current_pressure = self._read_pressure()
        
        # 非校准状态下记录压力样本
        if not self.is_calibration_step:
            self.pressure_samples.append(self.current_pressure)
            
        # 计算控制输出
        control_output = 0.0
        
        if self.control_mode in ["closed_loop", "hybrid"]:
            # 计算误差
            error = self.target_pressure - self.current_pressure
            
            # PID控制计算
            control_output = self.pid_controller.update(error, eventtime)
            
            # 记录反馈数据
            self.feedback_validator.record_sample(
                self.target_pressure, self.current_pressure, error, eventtime
            )
            
            # 检查阈值
            threshold_status = self.feedback_validator.check_thresholds(error)
            if threshold_status == "critical":
                logging.warning(
                    "严重压力误差: %.2f (目标: %.2f, 实际: %.2f)"
                    % (error, self.target_pressure, self.current_pressure)
                )
                
            # 分析趋势并应用自适应补偿
            trend = self.feedback_validator.analyze_trend()
            if trend == "worsening" and abs(error) > 1.0:
                self.pressure_model.apply_adaptive_compensation(error)
                
        # 应用调整
        if abs(control_output) > 0.1:
            self._apply_adjustment(control_output, eventtime)
            
        return eventtime + self.control_period

    def _read_pressure(self):
        """
        读取当前压力值
        
        从压力传感器获取当前压力读数
        
        返回:
            当前压力值
        """
        if self.load_cell is None:
            return 0.0
            
        try:
            # 从load_cell获取力值并转换为压力
            force = self.load_cell.counts_to_grams(self.load_cell.avg_counts())
            if force is None:
                return self.current_pressure
                
            # 力到压力的转换 (可根据实际安装调整系数)
            pressure = force * 1.0
            return pressure
            
        except Exception:
            return self.current_pressure

    def _apply_feedforward(self, move_info):
        """
        应用前馈控制
        
        根据预读的速度信息提前调整目标压力
        
        参数:
            move_info: 移动信息字典
        """
        velocity_change = move_info.get("velocity_change", 0.0)
        
        if abs(velocity_change) > 5.0:
            extrude_rate = move_info.get("extrude_rate", 0.0)
            velocity = move_info.get("velocity", 0.0)
            
            # 根据模型计算新的目标压力
            new_target = self.pressure_model.calculate_target_pressure(
                velocity=velocity, extrude_rate=extrude_rate
            )
            
            self.target_pressure = new_target
            
            logging.debug(
                "前馈控制: 速度=%.1f, 挤出速率=%.2f, 目标压力=%.2f"
                % (velocity, extrude_rate, new_target)
            )

    def _apply_adjustment(self, adjustment, eventtime):
        """
        应用压力调整
        
        根据控制输出调整挤出参数
        
        参数:
            adjustment: 调整值
            eventtime: 事件时间
        """
        # 检查调整间隔
        if eventtime - self.last_adjustment_time < self.min_adjustment_interval:
            return
            
        self.last_adjustment_time = eventtime
        
        # 根据配置的调整方法应用调整
        if self.adjustment_method == "extrude_factor":
            self._apply_extrude_factor_adjustment(adjustment)
        elif self.adjustment_method == "rotation_distance":
            self._apply_rotation_distance_adjustment(adjustment)

    def _apply_extrude_factor_adjustment(self, adjustment):
        """
        通过挤出因子调整压力
        
        参数:
            adjustment: 调整值
        """
        # 计算新的挤出因子
        new_factor = self.current_extrude_factor + adjustment / 100.0
        new_factor = max(0.5, min(1.5, new_factor))  # 限制范围
        
        # 应用到gcode_move
        self.gcode_move.extrude_factor = new_factor
        self.current_extrude_factor = new_factor

    def _apply_rotation_distance_adjustment(self, adjustment):
        """
        通过旋转距离调整压力
        
        参数:
            adjustment: 调整值
        """
        if self.extruder and self.extruder.extruder_stepper:
            current_dist = self.extruder.extruder_stepper.get_rotation_distance()
            new_dist = current_dist * (1.0 - adjustment / 100.0)
            self.extruder.extruder_stepper.set_rotation_distance(new_dist)

    def start_calibration_step(self, speed):
        """
        开始校准步骤
        
        参数:
            speed: 校准速度
        """
        self.is_calibration_step = True
        self.calibration_pressure_samples = []

    def end_calibration_step(self):
        """结束校准步骤"""
        self.is_calibration_step = False

    def get_average_pressure(self):
        """
        获取平均压力
        
        返回:
            校准期间的平均压力值
        """
        if not self.calibration_pressure_samples:
            return self.current_pressure
            
        return sum(self.calibration_pressure_samples) / len(
            self.calibration_pressure_samples
        )

    def get_current_extrude_rate(self):
        """获取当前挤出速率"""
        return self.lookahead_reader.get_current_extrude_rate()

    def get_status(self, eventtime):
        """
        获取系统状态
        
        用于Webhooks API查询
        
        参数:
            eventtime: 事件时间
            
        返回:
            状态信息字典
        """
        stats = self.feedback_validator.get_statistics()
        model_status = self.pressure_model.get_status(eventtime)
        
        return {
            "enabled": self.enabled,
            "control_mode": self.control_mode,
            "target_pressure": self.target_pressure,
            "current_pressure": self.current_pressure,
            "error": self.target_pressure - self.current_pressure,
            "extrude_factor": self.current_extrude_factor,
            "velocity": self.lookahead_reader.get_current_velocity(),
            "extrude_rate": self.lookahead_reader.get_current_extrude_rate(),
            "calibration_points": model_status["calibration_points"],
            "trend": self.feedback_validator.analyze_trend(),
            "statistics": stats,
        }


def load_config(config):
    """
    模块加载入口函数
    
    Klipper在加载模块时调用此函数
    
    参数:
        config: Klipper配置对象
        
    返回:
        ExtrusionPressureControl实例
    """
    return ExtrusionPressureControl(config)
