'''
文件说明：将位姿估计算法生成的4*4位姿矩阵（物体相对于相机的4*4transformation_matrix）
转换为6维位姿向量（translation_vector + euler_angles）（物体相对于基底的6维位姿向量）
'''

import numpy as np
from compute_transform_matrix import compute_transform_matrix
from EulerRotationConversion import rotation_matrix_to_euler_angles

def pose_matrix_to_vector(Transformation_Matrix, Camera_to_Base = np.array([[1, 0, 0, 2],
                                                                            [0, 1, 0, 1],
                                                                            [0, 0, 1, 0],
                                                                            [0, 0, 0, 1]])):   # Camera_to_Base是相机相对于基底的4*4transformation_matrix，需要根据实际情况修改
    # 物体相对于基底的4*4transformation_matrix
    transformation_matrix = compute_transform_matrix(Transformation_Matrix, Camera_to_Base)

    # 提取出旋转矩阵
    rotation_matrix = transformation_matrix[0:3, 0:3]

    # 提取出平移向量
    translation_vector = transformation_matrix[0:3, 3]

    # 将旋转矩阵转换为欧拉角
    rx, ry, rz = rotation_matrix_to_euler_angles(rotation_matrix)


    # 将translaton_vector和欧拉角拼成一个6维向量
    pose = np.array([translation_vector[0], translation_vector[1], translation_vector[2], rx, ry, rz])

    return pose

if __name__ == "__main__":
    # 示例用法：
    # 位姿估计算法生成的4*4位姿矩阵（物体相对于相机的4*4transformation_matrix）
    Transformation_Matrix = np.array([[0.866, -0.5, 0, 2],
                                    [0.5, 0.866, 0, 3],
                                    [0, 0, 1, 1],
                                    [0, 0, 0, 1]])

    # 相机相对于机械臂基底的4*4transformation_matrix
    Camera_to_Base = np.array([[1, 0, 0, 2],
                            [0, 1, 0, 1],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
    
    # 将4*4位姿矩阵转换为6维位姿向量
    pose = pose_matrix_to_vector(Transformation_Matrix, Camera_to_Base)

    # 打印6维位姿向量
    print("Pose:", pose)

