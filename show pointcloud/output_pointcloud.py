import math

import numpy as np
import cv2
import open3d as o3d

# 定义旋转角度和平移向量
roll = -1.5812880487804877
pitch = -0.006151537921052633
yaw = 1.5866386842105265
tran_x = -0.18489978048780487
tran_y = -0.030279151625
tran_z = -0.07672592926829268

# 创建旋转矩阵
R_x = np.array([[1, 0, 0],
                [0, np.cos(roll), -np.sin(roll)],
                [0, np.sin(roll), np.cos(roll)]])

R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                [0, 1, 0],
                [-np.sin(pitch), 0, np.cos(pitch)]])

R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                [np.sin(yaw), np.cos(yaw), 0],
                [0, 0, 1]])

# 组合旋转矩阵
Rotation1 = R_z @ R_y @ R_x

# 创建4x4变换矩阵
transformMatrix_c2l = np.eye(4)
transformMatrix_c2l[:3, :3] = Rotation1
transformMatrix_c2l[:3, 3] = [tran_x, tran_y, tran_z]

# 计算逆变换矩阵
transformMatrix_l2c = np.linalg.inv(transformMatrix_c2l)


# 读取点云
def read_point_cloud_from_pcd(file_path):
    pcd = o3d.io.read_point_cloud(file_path)
    return pcd


# 定义相机内参矩阵
def initialize_camera_matrix():
    Camera_inMat = np.zeros((3, 3), dtype=np.float64)
    Camera_inMat[0, 0] = 415.2405
    Camera_inMat[0, 2] = 629.6892
    Camera_inMat[1, 1] = 416.0930
    Camera_inMat[1, 2] = 340.6051
    Camera_inMat[2, 2] = 1
    return Camera_inMat


def project_and_color_pcd(pcd, camera_matrix, transform_matrix, image):
    points = np.asarray(pcd.points)
    colors = []

    for point in points:
        point_homogeneous = np.append(point, 1)  # 转换为齐次坐标
        transformed_point = transform_matrix @ point_homogeneous  # 应用变换
        pixel = camera_matrix @ transformed_point[:3]  # 映射到图像平面
        x = int(pixel[0] / pixel[2])
        y = int(pixel[1] / pixel[2])

        # 确保点在图像范围内
        if 0 <= x < image.shape[1] and 0 <= y < image.shape[0] and pixel[2] >= 0:
            color = image[y, x]  # 提取像素颜色
            colors.append(color[::-1] / 255.0)  # BGR到RGB，归一化
        else:
            colors.append([0, 0, 0])  # 如果点超出图像范围，赋予黑色

    # 更新点云颜色
    pcd.colors = o3d.utility.Vector3dVector(colors)


def save_colored_pcd(pcd, output_path):
    o3d.io.write_point_cloud(output_path, pcd, write_ascii=True)


def visualize_colored_point_cloud(file_path):
    # 读取点云
    pcd = o3d.io.read_point_cloud(file_path)

    # 检查点云是否为空
    if pcd.is_empty():
        print("Loaded point cloud is empty.")
        return

    # 可视化点云
    print("Displaying the colored point cloud...")
    o3d.visualization.draw_geometries([pcd], window_name="Colored Point Cloud", point_show_normal=True)


def main():
    file_path = 'D:\image_distort\show pointcloud\pose3_full.pcd'
    image_path = 'D:\image_distort\show pointcloud\pose3.png'
    output_path = 'colored_output.pcd'

    # 读取点云和图像
    pcd = o3d.io.read_point_cloud(file_path)
    image = cv2.imread(image_path)

    # 你可以根据实际需要定义相机矩阵和变换矩阵
    camera_matrix = initialize_camera_matrix()  # 定义或读取你的相机内参

    # 项目点云到图像并为点云着色
    project_and_color_pcd(pcd, camera_matrix, transformMatrix_l2c, image)

    # 保存着色后的点云
    save_colored_pcd(pcd, output_path)

    print(f"Colored point cloud saved to {output_path}")
    visualize_colored_point_cloud(output_path)


if __name__ == '__main__':
    main()