# 文件说明：将机械臂末端相对于机械臂基底的位姿向量转换为末端相对于基底的旋转矩阵和平移向量


import numpy as np
from EulerRotationConversion import euler_to_rotation_matrix


def pose_vectors_to_end2base_transforms(pose_vectors):
    # 提取旋转矩阵和平移向量
    R_end2bases = []
    t_end2bases = []

    # 迭代遍历每个位姿的旋转矩阵和平移向量
    for pose_vector in pose_vectors:
        # 提取旋转矩阵和平移向量
        R_end2base = euler_to_rotation_matrix(pose_vector[3], pose_vector[4], pose_vector[5])
        t_end2base = pose_vector[:3]

        # 提取旋转矩阵和平移向量
        R_end2bases.append(R_end2base)
        t_end2bases.append(t_end2base)

    return R_end2bases, t_end2bases


# 示例用法：
if __name__ == "__main__":
    # 输入位姿数据
    pose_vectors = np.array([[1, 2, 3, 30, 30, 30],
                             [7, 8, 9, 30, 30, 30],
                             [13, 14, 15, 30, 30, 30],
                             [19, 20, 21, 30, 30, 30]])

    # 将位姿向量转换为末端坐标系下的旋转矩阵和平移向量
    R_end2bases, t_end2bases = pose_vectors_to_end2base_transforms(pose_vectors)

    # 打印旋转矩阵和平移向量
    print("旋转矩阵：")
    print(R_end2bases)
    print("平移向量：")
    print(t_end2bases)