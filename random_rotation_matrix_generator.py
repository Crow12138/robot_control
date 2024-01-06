# 文件说明：随机生成旋转矩阵


import numpy as np

def random_rotation_matrix():
    # 第一步: 从标准正态分布中随机生成一个 3x3 矩阵
    random_matrix = np.random.randn(3, 3)
    
    # 第二步: 使用 QR 分解来生成一个旋转矩阵（QR分解是矩阵分解的一种重要方法,它将一个矩阵A分解为一个正交矩阵Q和一个上三角矩阵R）
    q, _ = np.linalg.qr(random_matrix)
    return q


# 示例用法：
rotation_matrix = random_rotation_matrix()

print("随机旋转矩阵:")
for row in rotation_matrix:
    row_str = "[" + ", ".join(str(element) for element in row) + "],"
    print(row_str)


