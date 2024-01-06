# 文件说明：将机械臂末端相对于机械臂基底的位姿向量转换为基底相对于末端的旋转矩阵和平移向量


import numpy as np
from EulerRotationConversion import euler_to_rotation_matrix

def pose_vectors_to_base2end_transforms(pose_vectors):
    # 提取旋转矩阵和平移向量
    R_base2ends = []
    t_base2ends = []

    # 迭代的到每个位姿的旋转矩阵和平移向量
    for pose_vector in pose_vectors:
        # 提取旋转矩阵和平移向量
        R_end2base = euler_to_rotation_matrix(pose_vector[3], pose_vector[4], pose_vector[5])
        t_end2base = pose_vector[:3]

        # 将旋转矩阵和平移向量组合成齐次位姿矩阵
        pose_matrix = np.eye(4)
        pose_matrix[:3, :3] = R_end2base
        pose_matrix[:3, 3] = t_end2base

        # 求位姿矩阵的逆矩阵
        pose_matrix_inv = np.linalg.inv(pose_matrix)

        # 提取旋转矩阵和平移向量
        R_base2end = pose_matrix_inv[:3, :3]
        t_base2end = pose_matrix_inv[:3, 3]

        # 将平移向量转换为列向量(3*1)
        t_base2end = t_base2end.reshape(3, 1)

        # 将旋转矩阵和平移向量保存到列表
        R_base2ends.append(R_base2end)
        t_base2ends.append(t_base2end)

    # # 打印旋转矩阵和平移向量的形状
    # print(np.array(R_base2ends).shape)
    # print(np.array(t_base2ends).shape)
    
    return R_base2ends, t_base2ends


# 示例用法：
if __name__ == "__main__":
    # 输入位姿数据
    pose_vectors = np.array([[1, 2, 3, 30, 30, 30],
                            [7, 8, 9, 30, 30, 30],
                            [13, 14, 15, 30, 30, 30],
                            [19, 20, 21, 30, 30, 30]])
    
    # 将位姿向量转换为基底坐标系下的旋转矩阵和平移向量
    R_base2ends, t_base2ends = pose_vectors_to_base2end_transforms(pose_vectors)

    # 打印旋转矩阵和平移向量
    print("旋转矩阵：")
    print(R_base2ends)
    print("平移向量：")
    print(t_base2ends)
    




