# 文件说明：将机械臂末端（工具坐标系）相对与基座的坐标写入文件，用于手眼标定

import time
from pymycobot.mycobot import MyCobot
import serial.tools.list_ports

# 定义要筛选的字符串
usb_string = "USB"

# 获取包含指定字符串的所有串口
ports = serial.tools.list_ports.grep(usb_string)

# # 遍历所有串口，打印串口的名称和描述
for port, desc, hwid in sorted(ports):
    mc = MyCobot(port,115200)
    break

# 初始化基座坐标系和末端坐标系
mc.set_world_reference([0, 0, 0, 0, 0, 0])
mc.set_reference_frame(0)
mc.set_tool_reference([0, 0, 0, 0, 0, 0]) 
mc.set_end_type(0)

# mc.set_tool_reference([0, 0, 1000, 0, 0, 0])   
# mc.set_end_type(1)     # 0-末端坐标系，1-工具坐标系

# mc.set_fresh_mode(0)
# time.sleep(1)
# mc.send_angles([0, -10, -50, -5, 0, 0], 50)
# time.sleep(1)

coords = mc.get_coords()
print(coords)

# 定义要写入的文件名
file_name = "coords.txt"

# 打开文件以追加模式写入，如果文件不存在则创建
with open(file_name, "a") as file:
    # 将 coords 列表中的值转换为字符串并写入文件
    coords_str = ", ".join(map(str, coords))  # 将列表中的值转换为字符串，并用逗号分隔
    file.write("[" + coords_str + "]" + "," + "\n") 