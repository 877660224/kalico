#!/usr/bin/env python3
"""
快速测试脚本，验证 ALPS 传感器数据解析
"""
import re

# 测试传感器中使用的正则表达式模式（支持整数和小数，逗号后允许空格）
data_pattern = re.compile(r'a=(-?\d+\.?\d*),\s*b=(-?\d+\.?\d*)')

# 测试用例
test_lines = [
    "a=-268600, b=-273529",      # 实际传感器数据格式（整数，有空格）
    "a=-268600,b=-273529",       # 无空格格式
    "a=123.45,b=678.90",         # 小数格式
    "a=-123.45, b=678.90",       # 负数小数，有空格
    "a=0,b=0",                   # 零值
]

print("测试 ALPS 传感器数据解析...\n")

for line in test_lines:
    match = data_pattern.search(line)
    if match:
        a_value = float(match.group(1))
        b_value = float(match.group(2))
        print(f"✓ 行: '{line}'")
        print(f"  a (原始) = {a_value}")
        print(f"  b (滤波) = {b_value}\n")
    else:
        print(f"✗ 解析失败: '{line}'\n")

print("测试完成！")
