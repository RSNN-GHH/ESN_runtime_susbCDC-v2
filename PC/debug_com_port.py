#!/usr/bin/env python3
"""
调试脚本：检测可用的COM端口和PICO板
"""

import serial
from serial.tools import list_ports
import sys

def main():
    print("="*60)
    print("PICO ESP32 COM端口调试工具")
    print("="*60)
    
    print("\n1️⃣ 检查所有可用的COM端口:\n")
    
    ports = list(list_ports.comports())
    
    if not ports:
        print("❌ 没有找到任何COM端口！")
        print("\n请检查:")
        print("  • PICO板是否通过USB连接到电脑")
        print("  • USB驱动是否正确安装")
        print("  • 在设备管理器中查看可用的COM端口")
        return
    
    print(f"找到 {len(ports)} 个COM端口:\n")
    
    for i, port in enumerate(ports, 1):
        print(f"  [{i}] {port.device}")
        print(f"      描述: {port.description}")
        print(f"      硬件ID: {port.hwid}")
        print()
    
    # 尝试连接到每个端口
    print("\n2️⃣ 尝试连接到每个COM端口:\n")
    
    for port in ports:
        print(f"尝试连接到 {port.device}...")
        try:
            ser = serial.Serial(port.device, 115200, timeout=1)
            print(f"  ✅ {port.device} 连接成功!")
            ser.close()
        except Exception as e:
            print(f"  ❌ {port.device} 连接失败: {e}")
        print()
    
    print("\n3️⃣ 建议:")
    print("  • 如果看到 'CH340' 或 'USB-SERIAL' 等字样，那就是PICO板")
    print("  • 在 esn.py 中修改 spikeActivation 初始化时使用正确的COM端口")
    print("  • 例如: sA = spikeActivation(frame_size, count_win)")
    print("         result = sA.init(com_port='COM1', max_retries=3)")

if __name__ == "__main__":
    main()
