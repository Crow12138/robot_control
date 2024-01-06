# 文件说明：四元数与旋转矩阵的相互转换

import numpy as np
import transforms3d

def quaternion_to_rotation_matrix(quaternion):
    # 将四元数转换为旋转矩阵
    rotation_matrix = transforms3d.quaternions.quat2mat(quaternion)
    
    return rotation_matrix

def rotation_matrix_to_quaternion(rotation_matrix):
    # 将旋转矩阵转换为四元数
    quaternion = transforms3d.quaternions.mat2quat(rotation_matrix)

    return quaternion

# 示例用法
if __name__ == "__main__":
    # 用您的四元数替换以下示例四元数
    quaternion = [0.74728276, 0.12946174, 0.64949654, -0.05442774] # 这里可以替换为您的四元数

    rotation_matrix = quaternion_to_rotation_matrix(quaternion)
    print("旋转矩阵：\n", rotation_matrix)
    
    # 用您的旋转矩阵替换以下示例矩阵
    rotation_matrix = np.array([
        [ 0.15038373,  0.24951573,  0.95662251],
        [ 0.08682409,  0.96055456, -0.26419032],
        [-0.98480775,  0.1227878,   0.1227878 ]
    ])
    
    quaternion = rotation_matrix_to_quaternion(rotation_matrix)
    print("四元数：", quaternion)


