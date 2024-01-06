# 文件说明：棋盘格手眼标定（眼在手内）

'''
在执行手眼标定时，需要将标定板放置某一固定位置，并同时将相机固定在机械臂末端。
接着控制机械臂末端位于不同的位置，记录下此时机械臂相对于基座的位姿，并使用相机拍摄标定板上的棋盘格图像。
将图像放入./images文件夹中，并将位姿信息输入到chessboard_handeye_calibration.py文件的pose_vectors变量中。
最后运行chessboard_handeye_calibration.py，即可得到相机相对于机械臂末端的位姿矩阵。
'''

import cv2
import numpy as np
import glob
from pose_vectors_to_end2base_transforms import pose_vectors_to_end2base_transforms


#################### 输入 ##########################################################################################################
# 输入位姿数据
pose_vectors = np.array([[222.8, 14.3, 373.4, -151.08, 23.19, -90.93],
                         [310.0, -36.1, 222.3, -146.52, 16.32, -91.03],
                         [251.5, -92.9, 241.5, -152.12, 27.33, -111.35],
                         [185.3, 18.6, 393.5, -144.52, 15.31, -97.8],
                         [290.3, 15.7, 191.5, -133.26, 6.99, -85.7],
                         [271.2, 153.1, 292.7, -137.01, -5.45, -103.4],
                         [243.5, 147.7, 158.5, -127.87, -3.89, -107.1],
                         [223.6, 178.4, 264.8, -134.69, 5.08, -126.14],
                         [155.6, 128.7, 310.7, -138.55, -15.48, -82.4],
                         [67.5, 116.7, 291.5, -129.01, 9.17, -110.31],
                         [131.8, -53.7, 327.2, -131.56, 17.73, -87.48],
                         [158.8, 1.5, 299.7, -127.55, 3.27, -82.63],
                         [179.5, 114.3, 273.2, -132.26, 10.39, -109.83],
                         [287.3, 63.1, 310.2, -148.74, 5.9, -105.59]])

# 定义棋盘格参数
square_size = 20.0  # 假设格子的边长为30mm
pattern_size = (6, 4)   # 在这个例子中，假设标定板有9个内角点和6个内角点

# 导入相机内参和畸变参数
# 焦距 fx, fy, 光心 cx, cy
# 畸变系数 k1, k2 
fx, fy, cx, cy = 2374.36884607104, 2383.07925178301, 1303.05974172261, 688.609636507929
k1, k2 = 0.0928659452689156, -0.433262929710012  
K = np.array([[fx, 0, cx], 
              [0, fy, cy], 
              [0, 0, 1]], dtype=np.float64)   # K为相机内参矩阵
dist_coeffs = np.array([k1, k2, 0, 0], dtype=np.float64)   # 畸变系数

# 所有图像的路径
images = glob.glob('./images/*.jpg')
###########################################################################################################################


# 准备位姿数据
obj_points = []  # 用于保存世界坐标系中的三维点
img_points = []  # 用于保存图像平面上的二维点

# 创建棋盘格3D坐标
objp = np.zeros((np.prod(pattern_size), 3), dtype=np.float32)
objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) * square_size


# 迭代处理图像
for image in images:
  img = cv2.imread(image)   # 读取图像
  gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)   # RGB图像转换为灰度图像
  
  # 棋盘格检测
  ret, corners = cv2.findChessboardCorners(gray, pattern_size)
  
  if ret:
    # 如果成功检测到棋盘格，添加图像平面上的二维点和世界坐标系中的三维点到列表
    obj_points.append(objp)
    img_points.append(corners)

    # 绘制并显示角点
    cv2.drawChessboardCorners(img, pattern_size, corners, ret)
    cv2.namedWindow('img', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('img', 640, 480)
    cv2.imshow('img', img)
    cv2.waitKey(500)

cv2.destroyAllWindows()

# # 打印obj_point和img_point的形状
# print(np.array(obj_points).shape)
# print(np.array(img_points).shape)


# 求解标定板位姿
R_board2cameras = []  # 用于保存旋转矩阵
t_board2cameras = []  # 用于保存平移向量
# 迭代的到每张图片相对于相机的位姿
for i in range(len(images)):
  # rvec：标定板相对于相机坐标系的旋转向量
  # t_board2camera：标定板相对于相机坐标系的平移向量
  ret, rvec, t_board2camera = cv2.solvePnP(obj_points[i], img_points[i], K, dist_coeffs) 

  # 将旋转向量(rvec)转换为旋转矩阵
  # R：标定板相对于相机坐标系的旋转矩阵
  R_board2camera, _ = cv2.Rodrigues(rvec)   # 输出：R为旋转矩阵和旋转向量的关系  输入：rvec为旋转向量

  # 将标定板相对于相机坐标系的旋转矩阵和平移向量保存到列表
  R_board2cameras.append(R_board2camera)
  t_board2cameras.append(t_board2camera)

# # 打印R_board2cameras和t_board2cameras的形状
# print(np.array(R_board2cameras).shape)
# print(np.array(t_board2cameras).shape)

# 求解手眼标定
# R_end2bases：机械臂末端相对于机械臂基座的旋转矩阵
# t_end2bases：机械臂末端相对于机械臂基座的平移向量
R_end2bases, t_end2bases = pose_vectors_to_end2base_transforms(pose_vectors)

# R_camera2end：相机相对于机械臂末端的旋转矩阵
# t_camera2end：相机相对于机械臂末端的平移向量
R_camera2end, t_camera2end = cv2.calibrateHandEye(R_end2bases, t_end2bases, 
                                                    R_board2cameras, t_board2cameras,
                                                    method=cv2.CALIB_HAND_EYE_TSAI)

# 将旋转矩阵和平移向量组合成齐次位姿矩阵
T_camera2end = np.eye(4)
T_camera2end[:3, :3] = R_camera2end
T_camera2end[:3, 3] = t_camera2end.reshape(3)

# 输出相机相对于机械臂末端的旋转矩阵和平移向量
print("Camera to end rotation matrix:")
print(R_camera2end)
print("Camera to end translation vector:") 
print(t_camera2end)

# 输出相机相对于机械臂末端的位姿矩阵
print("Camera to end pose matrix:")
np.set_printoptions(suppress=True)  # suppress参数用于禁用科学计数法
print(T_camera2end)