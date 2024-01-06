'''
这个机械臂控制脚本用于眼在手外的情况下。
'''

import argparse
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Coord
import numpy as np
from pose_matrix_to_vector import pose_matrix_to_vector
import time

mc = MyCobot("COM3", 115200)


# ###################### 输入 ######################
# # 位姿估计算法生成的4*4位姿矩阵（物体相对于相机的4*4transformation_matrix）
# Transformation_Matrix = np.array([[0.866, -0.5, 0, 2],
#                                   [0.5, 0.866, 0, 3],
#                                   [0, 0, 1, 1],
#                                   [0, 0, 0, 1]])

# # 选择操作模式： 0-单参数坐标控制，1-多参数坐标控制
# mode = 0
# ###################################################

# 终端输入
# -t "[[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]", 注意一定要加双引号""
parser = argparse.ArgumentParser()
parser.add_argument("-t", "--transformation_matrix", type=str, default="[[-0.66, -0.59, 0.46, 171.1], [-0.75, 0.49, -0.43, -275.9], [0.034, -0.635, -0.77, 237.8], [0, 0, 0, 0]]", required=True, help="位姿估计算法生成的4*4位姿矩阵（物体相对于相机的4*4transformation_matrix）")
parser.add_argument("-m", "--mode", default=0, type=int, required=True, help="选择操作模式： 0-单参数坐标控制，1-多参数坐标控制")
args = parser.parse_args()

Transformation_Matrix = np.array(eval(args.transformation_matrix))
mode = args.mode

# 初始化基座坐标系和末端坐标系
mc.set_world_reference([0, 0, 0, 0, 0, 0])
mc.set_reference_frame(0)
mc.set_tool_reference([0, 0, 0, 0, 0, 0]) 
mc.set_end_type(0)

# # 设置世界坐标系（可以将世界坐标系设置为机械臂基座坐标系）
# mc.set_world_reference([0, 0, 0, 0, 0, 0])
# mc.set_reference_frame(1)    # 0-基座坐标系，1-世界坐标系
# coords1 = mc.get_world_reference()

# # 设置工具坐标系（可以将工具坐标系设置为机械臂末端坐标系）
tool_z = 120    # 只需要修改z轴的值即可
mc.set_tool_reference([0, 0, tool_z, 0, 0, 0])   
mc.set_end_type(1)     # 0-末端坐标系，1-工具坐标系
coords2 = mc.get_tool_reference()


# 初始化机械臂
mc.set_gripper_state(0, 80)    # 0-打开，1-关闭
mc.set_fresh_mode(0)
time.sleep(1)
mc.send_angles([0, -10, -50, -5, 0, 0], 50)
time.sleep(3)
speed = 100

# 获取当前头部的坐标以及姿态
coords = mc.get_coords()
print(coords)

# 将4*4位姿矩阵(物体相对于相机)转换为6维位姿向量(物体相对于基座坐标系)
pose_vector = pose_matrix_to_vector(Transformation_Matrix)

if mode ==1:
    # 让头部以非线性的方式到达pose这个坐标，以及保持这个姿态，速度为40
    mc.send_coords(pose_vector, speed, 0)
    time.sleep(3)
    mc.set_gripper_state(1, 80)
else:
    # 让头部到达pose这个坐标，以及保持这个姿态，速度为40
    mc.send_coord(1, pose_vector[0], speed)
    mc.send_coord(2, pose_vector[1], speed)
    mc.send_coord(3, pose_vector[2], speed)
    mc.send_coord(4, pose_vector[3], speed)
    mc.send_coord(5, pose_vector[4], speed)
    mc.send_coord(6, pose_vector[5], speed)
    time.sleep(3)
    mc.set_gripper_state(1, 80)

# 回到初始位置
mc.send_angles([0, -10, -50, -5, 0, 0], 50)
