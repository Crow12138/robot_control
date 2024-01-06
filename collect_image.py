import pyrealsense2 as rs
import cv2
import numpy as np
import matplotlib.pyplot as plt
import os
import open3d as o3d


def display_color_and_depth_images(color_image, depth_image):
    """
    展示彩色图像和深度图像。

    参数:
    color_image -- 彩色图像
    depth_image -- 深度图像
    """
 
    # 展示彩色图像
    plt.figure(figsize=(8, 6))
    plt.imshow(cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB))  # 转换颜色空间，如果需要的话
    plt.title("Color Image")
    plt.axis('off')  # 关闭坐标轴

    # 展示深度图像
    plt.figure(figsize=(8, 6))
    plt.imshow(depth_image, cmap='gray')  # 使用灰度颜色映射
    plt.title("Depth Image")
    plt.axis('off')  # 关闭坐标轴

    plt.show()
# 使用该函数时，传入color_image和depth_image
# display_color_and_depth_images(color_image, depth_image)

def save_color_and_depth_images(color_image, depth_image):
    """
    将彩色图像和深度图像保存到指定的文件夹。

    参数:
    color_image -- 要保存的彩色图像。
    depth_image -- 要保存的深度图像。
    """
    # 设置文件名，例如：color_image_001.png
    color_filename = "color_image_{:03d}.png".format(n)
    depth_filename = "depth_image_{:03d}.png".format(n)

    # 确保文件夹存在，如果不存在则创建
    images_folder = "images"
    depths_folder = "depths"

    if not os.path.exists(images_folder):
        os.makedirs(images_folder)

    if not os.path.exists(depths_folder):
        os.makedirs(depths_folder)

    # 设置文件名
    color_image_fullpath = os.path.join(images_folder, color_filename)
    depth_image_fullpath = os.path.join(depths_folder, depth_filename)

    # 保存彩色图像
    cv2.imwrite(color_image_fullpath, color_image)

    # 保存深度图像
    cv2.imwrite(depth_image_fullpath, depth_image)
# 使用该函数时，传入color_image和depth_image
# save_color_and_depth_images(color_image, depth_image)



# 相机配置
pipeline = rs.pipeline() # 创建一个管道
config = rs.config() # 创建一个配置
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30) # 设置深度图像，分辨率640x480，像素格式z16，帧率30
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30) # 设置彩色图像，分辨率640x480，像素格式bgr8，帧率30
profile = pipeline.start(config) # 启动管道，并且返回profile对象

# 相机深度参数，包括精度以及 depth_scale
depth_sensor = profile.get_device().first_depth_sensor() # 获取深度传感器
depth_sensor.set_option(rs.option.visual_preset, 3) # 设置深度传感器的工作模式
depth_scale = depth_sensor.get_depth_scale() # depth_scale默认值为0.001。像素值乘以depth_scale就是以米为单位的距离。
clipping_distance_in_meters = 8  # 设置深度图像的剪切距离，单位为米。相机剪裁距离之外的物体就会被剪切掉。
clipping_distance = clipping_distance_in_meters / depth_scale # 剪切距离转换为剪切深度

# color和depth对齐
align_to = rs.stream.color
align = rs.align(align_to)

n = 0

while True:
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)

    aligned_depth_frame = aligned_frames.get_depth_frame()  # 对齐后的深度图
    color_frame = aligned_frames.get_color_frame()  # 对齐后的彩色图

    if not aligned_depth_frame or not color_frame:
        continue

    # 读取图像
    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    
    # 展示视频
    # cv2.imshow('color_image', color_image)
    # cv2.imshow('depth_image', depth_image) 
    # # 按下 q 键退出
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break
    
    # 显示图像
    display_color_and_depth_images(color_image, depth_image)
    
    # 保存图像
    save_color_and_depth_images(color_image, depth_image)
    n += 1
    
    # # 读取内参
    # intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
    # distortion_coeffs = intrinsics.coeffs
    # o3d_inter = o3d.camera.PinholeCameraIntrinsic(intrinsics.width, intrinsics.height, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy)
    # print(o3d_inter.intrinsic_matrix)
    # print(distortion_coeffs)
    # [[609.23474121   0.         321.21426392]
    #  [  0.         609.21148682 238.9261322 ]
    #  [  0.           0.           1.        ]]
