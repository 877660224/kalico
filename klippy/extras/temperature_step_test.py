# Temperature step response test for system identification
#
# Copyright (C) 2024  Kalico authors
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
import os
from .control_mpc import MpcCalibrate, TuningControl, ControlMPC
from .heaters import ControlBangBang

class LoggingControlWrapper:
    """
    控制器包装器，为不支持日志记录的控制器添加日志功能
    """
    def __init__(self, heater, wrapped_control=None):
        self.wrapped_control = wrapped_control or ControlBangBang({"max_delta": 2.0}, heater)
        self.heater = heater
        # 日志现在包含时间、温度和PWM值
        self.log = []
        self.logging = False
        
    def temperature_update(self, read_time, temp, target_temp):
        if self.logging:
            # 在温度更新时记录时间、温度和目标温度
            pwm_value = getattr(self.heater, 'last_pwm_value', 0.0)  # 尝试获取最后的PWM值
            self.log.append((read_time, temp, pwm_value, target_temp))
        # 调用被包装的控制器的temperature_update方法
        if hasattr(self.wrapped_control, 'temperature_update'):
            return self.wrapped_control.temperature_update(read_time, temp, target_temp)
        else:
            # 如果没有被包装的控制器，则只是设置PWM为0
            self.heater.set_pwm(read_time, 0.0)
    
    def set_output(self, value, target):
        if hasattr(self.wrapped_control, 'set_output'):
            # 调用原控制器的set_output方法并记录返回值
            result = self.wrapped_control.set_output(value, target)
            # 保存PWM值以供温度更新时使用
            setattr(self.heater, 'last_pwm_value', value)
            return result
        else:
            # 如果没有被包装的控制器，直接设置目标温度
            self.heater.set_temp(target)
    
    def check_busy(self, eventtime, smoothed_temp, target_temp):
        if hasattr(self.wrapped_control, 'check_busy'):
            return self.wrapped_control.check_busy(eventtime, smoothed_temp, target_temp)
        else:
            # 如果没有被包装的控制器，总是返回False（不忙）
            return False
    
    def get_profile(self):
        if hasattr(self.wrapped_control, 'get_profile'):
            return self.wrapped_control.get_profile()
        else:
            return {"name": "wrapped"}
    
    def get_type(self):
        if hasattr(self.wrapped_control, 'get_type'):
            return self.wrapped_control.get_type()
        else:
            return "wrapped"


class TempStepTest(MpcCalibrate):
    def __init__(self, config):
        # 从配置中获取printer对象
        self.printer = config.get_printer()
        
        # 从配置中获取加热器名称并查找对应的加热器对象
        heater_name = config.get_name().split()[-1]  # 获取配置名称的最后一部分，例如"extruder"
        if heater_name == 'temperature_step_test':
            heater_name = config.get("heater")  # 从配置中获取heater参数
        
        # 获取加热器对象
        pheaters = self.printer.lookup_object("heaters")
        self.heater = pheaters.lookup_heater(heater_name)
        
        # 获取原始控制器
        self.orig_control = self.heater.get_control()
        
        # 调用父类构造函数，传递必需的参数
        super().__init__(self.printer, self.heater, self.orig_control)
        
        # 注册G-code命令
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command(
            "TEMP_STEP_TEST",
            "HEATER",
            heater_name,
            self.cmd_TEMP_STEP_TEST,
            desc=self.cmd_TEMP_STEP_TEST_help,
        )
        gcode.register_mux_command(
            "TEMP_SIM_TEST",
            "HEATER",
            heater_name,
            self.cmd_TEMP_SIM_TEST,
            desc=self.cmd_TEMP_SIM_TEST_help,
        )

    cmd_TEMP_STEP_TEST_help = "Run temperature step test with different controllers"
    cmd_TEMP_SIM_TEST_help = "Run temperature simulation test"

    def cmd_TEMP_STEP_TEST(self, gcmd):
        # 注册gcode命令
        self.run(gcmd)

    def cmd_TEMP_SIM_TEST(self, gcmd):
        self.sim_run()

    def run(self, gcmd):
        # 测试运行
        target_temp = gcmd.get_float("TARGET", 60, minval=1)
        duration = gcmd.get_float("DURATION", 60, minval=10)
        controller_type = gcmd.get("CONTROLLER", "neno").lower()  # 默认为neno（继电器控制器）
        threshold_temp = gcmd.get_float("THRESHOLD", max(50.0, min(100, target_temp - 100.0)))
        
        gcmd.respond_info(f"Starting step test with {controller_type} controller, target temp {target_temp}°C for {duration}s")
        
        # 根据G-code参数选择控制器
        if controller_type == "pid":
            # 使用当前加热器的原始控制器（假设是PID）
            control = LoggingControlWrapper(self.heater, self.orig_control)
        elif controller_type == "mpc":
            # 使用当前加热器的原始控制器（假设是MPC）
            control = LoggingControlWrapper(self.heater, self.orig_control)
        elif controller_type == "bangbang":
            # 使用BangBang控制器
            control = LoggingControlWrapper(self.heater, ControlBangBang({"max_delta": 2.0}, self.heater))
        else:  # 默认为neno（继电器控制器），即TuningControl
            control = LoggingControlWrapper(self.heater, TuningControl(self.heater))
        
        # 替换加热器的控制对象为带日志功能的包装器
        old_control = self.heater.set_control(control)
        
        try:
            # 执行阶跃测试
            log = self.step_test(gcmd, target_temp, control, duration)
            
            # 保存数据到文件（包含温度和PWM数据）
            self.save_heatup_data(gcmd, log)
            
            gcmd.respond_info(f"Step test with {controller_type} controller completed. Collected {len(log)} samples.")
            
        finally:
            # 恢复原始控制器
            self.heater.set_control(old_control)
            self.heater.alter_target(0.0)

    def step_test(self, gcmd, target_temp, control, duration=60):
        """
        执行温度阶跃响应测试
        :param gcmd: G-code命令对象
        :param target_temp: 目标温度
        :param control: 控制器对象
        :param duration: 测试持续时间（秒）
        :return: 采集到的温度数据 [(时间戳, 温度), ...]
        """
        gcmd.respond_info(f"Starting step test with {control.get_type()} controller, "
                          f"target temp {target_temp}°C")
        
        # 首先加热到25度并等待稳定
        gcmd.respond_info("Pre-heating to 25°C and waiting for stabilization...")
        control.set_output(self.heater.get_max_power(), 25.0)

        # 使用wait_stable等待温度在25度附近稳定
        # 设置目标温度为25，允许小的波动范围
        self.heater.set_temp(25.0)  # 临时设置加热器目标为25度
        gcmd.respond_info("Waiting for temperature to stabilize at 25°C...")
        self.wait_stable(3)  # 等待温度在目标值附近循环3次

        # 启动日志记录
        control.logging = True
        
        # 设置目标温度并开始阶跃测试
        gcmd.respond_info(f"Starting step test to target temperature: {target_temp}°C")
        control.set_output(self.heater.get_max_power(), target_temp)
        
        # 记录到达目标温度的时间
        start_time = self.printer.reactor.monotonic()
        reached_target_time = [None]  # 使用列表来在闭包中修改值
        temp_reached = [False]  # 标记是否已达到目标温度
        
        # 定义升温阶段的检查函数
        def ramp_check(eventtime):
            temp, _ = self.heater.get_temp(eventtime)
            gcmd.respond_info(f"Ramping up - Time: {(eventtime - start_time):.1f}s, Temperature: {temp:.2f}°C", 
                              log=False)
            
            # 检查是否达到目标温度
            if not temp_reached[0] and temp >= target_temp * 0.98:  # 达到目标温度的98%
                reached_target_time[0] = eventtime
                temp_reached[0] = True
                gcmd.respond_info(f"Reached target temperature at {temp:.2f}°C")
            
            # 如果已达到目标或超过持续时间，则结束升温阶段
            return (not temp_reached[0]) and (eventtime - start_time < duration)

        # 使用wait_while进行升温阶段
        self.printer.wait_while(ramp_check)
        
        # 如果还没有达到目标温度，记录当前时间作为达到目标的时间
        if reached_target_time[0] is None:
            reached_target_time[0] = self.printer.reactor.monotonic()
            gcmd.respond_info("Did not reach target temperature within duration, continuing sampling...")
        
        # 继续采样60秒
        gcmd.respond_info("Continuing sampling for 60 seconds after reaching target...")
        continue_sampling_until = reached_target_time[0] + 60.0
        
        # 定义采样阶段的检查函数
        def sampling_check(eventtime):
            temp, _ = self.heater.get_temp(eventtime)
            gcmd.respond_info(f"Post-target sampling - Time: {(eventtime - start_time):.1f}s, Temperature: {temp:.2f}°C", 
                              log=False)
            return eventtime < continue_sampling_until

        # 使用wait_while进行采样阶段
        self.printer.wait_while(sampling_check)

        # 添加冷却阶段，在等待阶段后让温度降到室温附近
        gcmd.respond_info("Cooling down to room temperature (~30°C) after stabilization...")
        control.set_output(0.0, 0.0)  # 关闭加热器
        
        # 记录开始冷却的时间
        start_cooling_time = self.printer.reactor.monotonic()
        
        # 定义冷却阶段的检查函数
        def cooling_check(eventtime):
            temp, _ = self.heater.get_temp(eventtime)
            gcmd.respond_info(f"Cooling down - Temperature: {temp:.2f}°C", 
                              log=False)
            # 冷却直到温度降到30度或最长3分钟
            time_elapsed = eventtime - start_cooling_time
            return temp > 30.0 and time_elapsed < 180.0

        # 使用wait_while进行冷却阶段
        self.printer.wait_while(cooling_check)
        
        # 停止记录
        control.logging = False
        self.heater.alter_target(0.0)
        
        # 获取并返回数据
        log = control.log[:]
        control.log = []  # 清空日志
        
        gcmd.respond_info(f"Step test completed. Collected {len(log)} samples.")
        return log

    # 函数测试
    def sim_run(self):
        samples = self.sim_data()
        # 修复错误的self.self引用
        self.save_heatup_data(None, samples)

    # 生成模拟响应数据
    def sim_data(self):
        # 生成模拟的温度响应数据
        import time
        start_time = time.time()
        data = []
        for i in range(100):
            t = start_time + i * 0.5
            # 简单的一阶系统响应模拟
            temp = 25 + 50 * (1 - 0.9 ** i)  # 模拟加热过程
            pwm = 0.5  # 模拟一个固定的PWM值
            target = 75.0  # 模拟目标温度
            data.append((t, temp, pwm, target))
        return data

    # 重写保存热数据的方法以处理四元组数据（时间，温度，PWM值，目标温度）
    def save_heatup_data(self, gcmd, samples):
        """
        保存热测试数据到CSV文件（包含温度和PWM数据）
        """
        import os
        import time
        
        # 获取家目录并创建数据路径
        home_dir = os.path.expanduser("~")
        data_dir = os.path.join(home_dir, "printer_data", "data")
        
        # 创建目录（包括父目录）
        os.makedirs(data_dir, exist_ok=True)
        
        # 生成带时间戳的文件名
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"temp_pwm_data_{self.heater.get_name()}_{timestamp}.csv"
        file_path = os.path.join(data_dir, filename)
        
        try:
            # 将数据写入CSV
            with open(file_path, 'w') as f:
                # 写入标题
                f.write("time,temperature,pwm_value,target_temp\n")
                
                # 写入数据 - 如果有样本则转换为相对时间
                if samples:
                    start_time = samples[0][0]
                    for t, temp, pwm, target in samples:
                        rel_time = t - start_time
                        f.write(f"{rel_time:.3f},{temp:.2f},{pwm:.4f},{target:.2f}\n")
                else:
                    logging.warning("No samples to save in save_heatup_data")
            
            # 记录日志并向用户响应
            logging.info(f"Heatup data saved to: {file_path}")
            gcmd.respond_info(f"Heatup test data (with PWM) saved to {file_path}")
        except Exception as e:
            logging.error(f"Failed to save heatup data: {e}")
            gcmd.respond_info(f"Warning: Failed to save heatup data: {e}")


def load_config(config):
    return TempStepTest(config)