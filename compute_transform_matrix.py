

import numpy as np

def compute_transform_matrix(X, Y):
    A = np.matmul(Y, X)
    return A

if __name__ == '__main__':
    # 假设X是c相对于b的转换矩阵（4x4齐次坐标变换矩阵）
    X = np.array([[0.866, -0.5, 0, 2],
                [0.5, 0.866, 0, 3],
                [0, 0, 1, 1],
                [0, 0, 0, 1]])

    # 假设Y是b相对于a的转换矩阵（4x4齐次坐标变换矩阵）
    Y = np.array([[1, 0, 0, 2],
                [0, 1, 0, 1],
                [0, 0, 1, 0],
                [0, 0, 0, 1]])

    # 调用算法，计算c相对于a的转换矩阵A
    A = compute_transform_matrix(X, Y)

    print("c相对于a的转换矩阵A:")
    print(A)
