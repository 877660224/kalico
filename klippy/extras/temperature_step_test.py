# Temperature step response test for system identification
#
# Copyright (C) 2024  Kalico authors
#
# This file may be distributed under the terms of the GNU GPLv3 license.



import logging, os
from .control_mpc import MpcCalibrate , TuningControl , ControlMPC


class TempStepTest(MpcCalibrate):
    
    def __init__(self, config):
        pass
    
    def cmd_TEMP_STEP_TEST(self, gcmd):
        # 注册gcode命令
        self.run(gcmd)
        pass

    def run(self,gcmd):
        # 测试运行

        # 指令指定控制器
        control = TuningControl(self.heater)
        
        pass

    



def load_config(config):
    return TempStepTest(config)


