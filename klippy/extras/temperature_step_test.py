# Temperature step response test for system identification
#
# Copyright (C) 2024  Kalico authors
#
# This file may be distributed under the terms of the GNU GPLv3 license.

"""
温度阶跃响应测试模块

该模块用于采集热端的满功率阶跃响应数据，用于系统辨识和控制器设计

功能特点：
- 支持多热端系统，可指定任意热端进行测试
- 直接控制加热功率，绕过常规温度控制器
- 自动记录时间、温度和PWM值
- 测试结果保存为CSV文件，便于后续分析
- 测试完成后自动恢复原控制器
- 完整的错误处理和安全保护

使用方法：
1. 将文件放置在klippy/extras/目录下，Klipper会自动加载
2. 通过G-Code命令TEMP_STEP_TEST触发测试
3. 测试完成后，在指定路径查看CSV结果文件

CSV文件格式：
    time,temp,pwm
    0.000,25.00,1.000
    0.100,25.10,1.000
    0.200,25.20,1.000
    ...

适用场景：
- 系统辨识和建模
- 控制器参数优化
- 热端性能测试
- 温度传感器校准
"""

import logging, os
from .control_mpc import TuningControl


class TempStepTest:
    """
    温度阶跃响应测试类，用于采集满功率阶跃响应数据
    
    实现无控制器下的热端响应数据采集，将测试结果保存到CSV文件
    
    配置示例：
        该模块无需额外配置，仅需将文件放置在klippy/extras/目录下即可自动加载
    
    G-Code使用示例：
        # 测试默认挤出机，持续60秒，结果保存到默认文件
        TEMP_STEP_TEST
        
        # 测试指定挤出机，持续120秒，结果保存到自定义文件
        TEMP_STEP_TEST HEATER=extruder DURATION=120 FILE=/tmp/extruder_step.csv
        
        # 测试加热床，持续30秒
        TEMP_STEP_TEST HEATER=bed DURATION=30 FILE=/tmp/bed_step.csv
        
        # 测试多挤出机系统中的第二个挤出机
        TEMP_STEP_TEST HEATER=extruder1 DURATION=60 FILE=/tmp/extruder1_step.csv
    
    参数说明：
        HEATER: 要测试的热端名称，如extruder、bed、chamber等，默认extruder
        DURATION: 测试持续时间，单位为秒，必须大于0，默认60秒
        FILE: 测试结果输出文件路径，默认/tmp/step_response.csv
    """
    
    def __init__(self, config):
        """
        初始化温度阶跃响应测试类
        
        Args:
            config: 配置对象
        """
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")
        self.heaters = self.printer.lookup_object("heaters")
        
        # 注册G-code命令
        self.gcode.register_command(
            "TEMP_STEP_TEST",
            self.cmd_TEMP_STEP_TEST,
            desc=self.cmd_TEMP_STEP_TEST_help,
        )
    
    def cmd_TEMP_STEP_TEST(self, gcmd):
        """
        处理TEMP_STEP_TEST G-code命令
        
        Args:
            gcmd: G-code命令对象
        """
        # 获取命令参数
        heater_name = gcmd.get("HEATER", "extruder")
        duration = gcmd.get_float("DURATION", 60.0, above=0.0)
        output_file = gcmd.get("FILE", "/tmp/step_response.csv")
        
        try:
            # 查找加热器对象
            heater = self.heaters.lookup_heater(heater_name)
        except self.printer.config_error as e:
            raise gcmd.error(str(e))
        
        # 创建测试控制器
        test_control = StepTestControl(heater, duration, output_file)
        
        # 保存原控制器
        old_control = heater.control
        
        try:
            # 切换到测试控制器
            heater.set_control(test_control)
            
            # 开始测试
            gcmd.respond_info(f"Starting step response test for {heater_name}")
            gcmd.respond_info(f"Duration: {duration} seconds")
            gcmd.respond_info(f"Output file: {output_file}")
            
            # 启动满功率加热
            test_control.start_test()
            
            # 等待测试完成
            while not test_control.is_done():
                self.printer.get_reactor().pause(self.printer.get_reactor().monotonic() + 0.5)
            
            gcmd.respond_info("Step response test completed")
            gcmd.respond_info(f"Results saved to {output_file}")
            
        except Exception as e:
            gcmd.respond_info(f"Step response test failed: {str(e)}")
            logging.exception("Step response test error")
        finally:
            # 恢复原控制器
            heater.set_control(old_control)
            # 关闭加热
            heater.set_temp(0.0)
    
    cmd_TEMP_STEP_TEST_help = "Run a temperature step response test for system identification"


class StepTestControl(TuningControl):
    """
    阶跃测试控制器，继承自TuningControl
    
    实现无控制器下的热端响应数据采集，直接控制加热功率并记录温度数据
    
    工作原理：
        1. 继承自TuningControl，绕过常规温度控制器
        2. 以满功率加热热端，记录温度随时间的变化
        3. 测试期间记录时间、温度和PWM值
        4. 测试结束后自动关闭加热并保存数据
        5. 测试完成后恢复原温度控制器
    
    数据输出格式：
        CSV文件格式，包含三列：
        - time: 相对时间，单位为秒，精确到小数点后3位
        - temp: 温度值，单位为摄氏度，精确到小数点后2位
        - pwm: PWM值，范围0-1，精确到小数点后3位
    
    安全特性：
        - 测试结束后自动关闭加热
        - 异常情况下恢复原控制器
        - 完整的错误处理和日志记录
    """
    
    def __init__(self, heater, duration, output_file):
        """
        初始化阶跃测试控制器
        
        Args:
            heater: 加热器对象
            duration: 测试持续时间（秒）
            output_file: 输出文件路径
        """
        super().__init__(heater)
        self.duration = duration
        self.output_file = output_file
        self.start_time = None
        self.data = []
        self.done = False
        
    def start_test(self):
        """
        开始阶跃测试
        """
        # 记录开始时间
        self.start_time = self.heater.reactor.monotonic()
        # 设置满功率加热
        self.set_output(self.heater.max_power, 0.0)
        # 开始记录数据
        self.logging = True
    
    def temperature_update(self, read_time, temp, target_temp):
        """
        温度更新回调函数，记录温度数据
        
        Args:
            read_time: 当前时间
            temp: 传感器温度
            target_temp: 目标温度
        """
        # 调用父类方法，设置PWM值
        super().temperature_update(read_time, temp, target_temp)
        
        # 记录数据
        if self.logging:
            # 计算相对时间
            relative_time = read_time - self.start_time
            # 记录时间、温度和PWM值
            self.data.append((relative_time, temp, self.heater.last_pwm_value))
            
            # 检查测试是否结束
            if relative_time >= self.duration:
                self._finish_test(read_time)
    
    def _finish_test(self, read_time):
        """
        结束测试，保存数据
        
        Args:
            read_time: 当前时间
        """
        # 停止记录数据
        self.logging = False
        # 关闭加热
        self.set_output(0.0, 0.0)
        # 保存数据到CSV文件
        self._save_data()
        # 标记测试完成
        self.done = True
    
    def _save_data(self):
        """
        保存测试数据到CSV文件
        """
        try:
            # 确保输出目录存在
            os.makedirs(os.path.dirname(self.output_file), exist_ok=True)
            
            # 写入CSV文件
            with open(self.output_file, 'w') as f:
                # 写入表头
                f.write("time,temp,pwm\n")
                # 写入数据
                for time, temp, pwm in self.data:
                    f.write(f"{time:.3f},{temp:.2f},{pwm:.3f}\n")
            
            logging.info(f"Step response data saved to {self.output_file}")
        except Exception as e:
            logging.exception(f"Failed to save step response data: {str(e)}")
    
    def is_done(self):
        """
        检查测试是否完成
        
        Returns:
            bool: 测试是否完成
        """
        return self.done


def load_config(config):
    """
    加载配置，注册阶跃测试对象
    
    Args:
        config: 配置对象
    """
    return TempStepTest(config)


