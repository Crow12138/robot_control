# 文件说明：将欧拉角转换为旋转矩阵：R = Rz * Ry * Rx
#          将旋转矩阵转换为欧拉角（旋转顺序为ZYX）

import numpy as np
import transforms3d

def euler_to_rotation_matrix(rx, ry, rz):  # rx, ry, rz是欧拉角，单位是度
    # 把角度转换为弧度
    rx = np.radians(rx)
    ry = np.radians(ry)
    rz = np.radians(rz)

    # 计算旋转矩阵Rz 、 Ry 、 Rx
    Rx = transforms3d.axangles.axangle2mat([1, 0, 0], rx)
    Ry = transforms3d.axangles.axangle2mat([0, 1, 0], ry)
    Rz = transforms3d.axangles.axangle2mat([0, 0, 1], rz)

    # 计算旋转矩阵R = Rz * Ry * Rx
    rotation_matrix = np.dot(Rz, np.dot(Ry, Rx))

    return rotation_matrix

def rotation_matrix_to_euler_angles(rotation_matrix):
    # 使用ZYX顺序提取旋转角度（Rz * Ry * Rx）
    euler_angles_rad = transforms3d.euler.mat2euler(rotation_matrix, 'sxyz')

    # 将角度从弧度转换为角度
    euler_angles_deg = np.degrees(euler_angles_rad)
    
    # 提取出三个角度
    roll = euler_angles_deg[0]   # rx_degrees
    pitch = euler_angles_deg[1] # ry_degrees
    yaw = euler_angles_deg[2]    # rz_degrees

    return roll, pitch, yaw

if __name__ == '__main__':
    # 示例用法：
    rx_degrees = 45
    ry_degrees = 80
    rz_degrees = 30

    rotation_matrix = euler_to_rotation_matrix(rx_degrees, ry_degrees, rz_degrees)
    print(rotation_matrix)
    
    # 用您的3x3旋转矩阵替换以下示例矩阵
    rotation_matrix = np.array([
        [ 0.15038373,  0.24951573,  0.95662251],
        [ 0.08682409,  0.96055456, -0.26419032],
        [-0.98480775,  0.1227878,   0.1227878 ]
    ])

    roll, pitch, yaw = rotation_matrix_to_euler_angles(rotation_matrix)
    print("Roll(rx_degrees):", roll)
    print("Pitch(ry_degrees):", pitch)  
    print("Yaw(rz_degrees):", yaw)
