import numpy as np
import cv2
import open3d as o3d
import math

import numpy as np
import yaml

width = 1280
height = 720
# 定义彩虹色映射表
colmap = [
    [0, 0, 0.5385], [0, 0, 0.6154], [0, 0, 0.6923], [0, 0, 0.7692],
    [0, 0, 0.8462], [0, 0, 0.9231], [0, 0, 1.0000], [0, 0.0769, 1.0000],
    [0, 0.1538, 1.0000], [0, 0.2308, 1.0000], [0, 0.3846, 1.0000], [0, 0.4615, 1.0000],
    [0, 0.5385, 1.0000], [0, 0.6154, 1.0000], [0, 0.6923, 1.0000], [0, 0.7692, 1.0000],
    [0, 0.8462, 1.0000], [0, 0.9231, 1.0000], [0, 1.0000, 1.0000], [0.0769, 1.0000, 0.9231],
    [0.1538, 1.0000, 0.8462], [0.2308, 1.0000, 0.7692], [0.3077, 1.0000, 0.6923], [0.3846, 1.0000, 0.6154],
    [0.4615, 1.0000, 0.5385], [0.5385, 1.0000, 0.4615], [0.6154, 1.0000, 0.3846], [0.6923, 1.0000, 0.3077],
    [0.7692, 1.0000, 0.2308], [0.8462, 1.0000, 0.1538], [0.9231, 1.0000, 0.0769], [1.0000, 1.0000, 0],
    [1.0000, 0.9231, 0], [1.0000, 0.8462, 0], [1.0000, 0.7692, 0], [1.0000, 0.6923, 0],
    [1.0000, 0.6154, 0], [1.0000, 0.5385, 0], [1.0000, 0.4615, 0], [1.0000, 0.3846, 0],
    [1.0000, 0.3077, 0], [1.0000, 0.2308, 0], [1.0000, 0.1538, 0], [1.0000, 0.0769, 0],
    [1.0000, 0, 0], [0.9231, 0, 0], [0.8462, 0, 0], [0.7692, 0, 0],
    [0.6923, 0, 0], [0.6154, 0, 0]
]


def read_camera_parameters_from_yaml(filename):
    with open(filename, 'r') as file:
        params = yaml.safe_load(file)

    # Extract rotation and translation parameters
    roll = params['rotation']['roll']
    pitch = params['rotation']['pitch']
    yaw = params['rotation']['yaw']
    tran_x = params['translation']['x']
    tran_y = params['translation']['y']
    tran_z = params['translation']['z']

    # Create rotation matrices
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])
    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])
    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])

    # Combine rotation matrices
    Rotation = R_z @ R_y @ R_x

    # Create 4x4 transformation matrix
    transformMatrix = np.eye(4)
    transformMatrix[:3, :3] = Rotation
    transformMatrix[:3, 3] = [tran_x, tran_y, tran_z]

    # Extract intrinsic camera matrix parameters
    f_x = params['intrinsic_matrix']['focal_length_x']
    f_y = params['intrinsic_matrix']['focal_length_y']
    c_x = params['intrinsic_matrix']['center_x']
    c_y = params['intrinsic_matrix']['center_y']

    # Create camera intrinsic matrix
    Camera_inMat = np.array([
        [f_x, 0, c_x],
        [0, f_y, c_y],
        [0, 0, 1]
    ], dtype=np.float64)

    transformMatrix_l2c = np.linalg.inv(transformMatrix)

    dist_coeffs = np.array([params["distort_param"]["k1"], params["distort_param"]["k2"], params["distort_param"]["p1"],
                            params["distort_param"]["p2"]], dtype=np.float64)

    return transformMatrix_l2c, Camera_inMat, dist_coeffs


# 读取点云
def read_point_cloud_from_pcd(file_path):
    pcd = o3d.io.read_point_cloud(file_path)
    return pcd


# 将点云投影到图像平面并应用畸变
def project_to_distored_image(pcd, camera_matrix, dist_coeffs, transform_matrix, image):
    points = np.asarray(pcd.points)
    distances_info = []
    for point in points:
        point_homogeneous = np.append(point, 1)  # 转换为齐次坐标
        transformed_point = transform_matrix @ point_homogeneous  # 应用变换

        camera_point = transform_matrix @ point_homogeneous  # 应用变换
        pixel = camera_matrix @ transformed_point[:3]  # 映射到图像平面
        x = int(pixel[0] / pixel[2])
        y = int(pixel[1] / pixel[2])

        a, b = transformed_point[0] / transformed_point[2], transformed_point[1] / transformed_point[2]
        r = math.sqrt(a ** 2 + b ** 2)

        theta = math.atan(r)
        theta_d = theta * (1 + dist_coeffs[0] * theta ** 2 + dist_coeffs[1] * theta ** 4 + dist_coeffs[
            2] * theta ** 6 + dist_coeffs[3] * theta ** 8)

        x_prime = (theta_d / r) * a
        y_prime = (theta_d / r) * b

        u = camera_matrix[0][0] * x_prime + camera_matrix[0][2]
        v = camera_matrix[1][1] * y_prime + camera_matrix[1][2]

        if u < 0 or v < 0 or u >= width or v >= height or pixel[2] < 0:
            continue

        dist = np.linalg.norm(transformed_point[:3])
        distances_info.append((u, v, dist))

    distances_info = np.array(distances_info)
    min_dist, max_dist = distances_info[:, 2].min(), distances_info[:, 2].max()
    for u_tmp, y_tmp, dist in distances_info:
        rangie_idx = int(min(round(((dist - min_dist) / (max_dist - min_dist)) * 49), 49))
        cv2.circle(image, (int(u_tmp), int(y_tmp)), 2, (
            255 * colmap[49 - rangie_idx][0], 255 * colmap[49 - rangie_idx][1], 255 * colmap[49 - rangie_idx][2]), -1)


# 将点云投影到图像平面
def project_to_undistored_image(pcd, camera_matrix, transform_matrix, image):
    points = np.asarray(pcd.points)

    distances_info = []
    for point in points:
        point_homogeneous = np.append(point, 1)  # 转换为齐次坐标
        transformed_point = transform_matrix @ point_homogeneous  # 应用变换
        pixel = camera_matrix @ transformed_point[:3]  # 映射到图像平面
        x = int(pixel[0] / pixel[2])
        y = int(pixel[1] / pixel[2])

        if x < 0 or y < 0 or x > width or y > height or transformed_point[2] < 0:
            continue

        dist = np.sqrt(pixel[0] ** 2 + pixel[1] ** 2 + pixel[2] ** 2)

        distances_info.append((x, y, dist))

    distances_info = np.array(distances_info)
    min_dist, max_dist = min(distances_info[:, 2]), max(distances_info[:, 2])
    for x, y, dist in distances_info:
        rangie_idx = int(min(round(((dist - min_dist) / (max_dist - min_dist)) * 49), 49.0))
        cv2.circle(image, (int(x), int(y)), 2, (
            255 * colmap[49 - rangie_idx][0], 255 * colmap[49 - rangie_idx][1], 255 * colmap[49 - rangie_idx][2]), -1)


def project_and_color_pcd_undistort(pcd, camera_matrix, transform_matrix, image):
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


def project_and_color_pcd_distort(pcd, camera_matrix, dist_coeffs, transform_matrix, image):
    points = np.asarray(pcd.points)
    colors = []

    for point in points:
        point_homogeneous = np.append(point, 1)  # 转换为齐次坐标
        transformed_point = transform_matrix @ point_homogeneous  # 应用变换

        camera_point = transform_matrix @ point_homogeneous  # 应用变换
        pixel = camera_matrix @ transformed_point[:3]  # 映射到图像平面
        x = int(pixel[0] / pixel[2])
        y = int(pixel[1] / pixel[2])

        a, b = transformed_point[0] / transformed_point[2], transformed_point[1] / transformed_point[2]
        r = math.sqrt(a ** 2 + b ** 2)

        theta = math.atan(r)
        theta_d = theta * (1 + dist_coeffs[0] * theta ** 2 + dist_coeffs[1] * theta ** 4 + dist_coeffs[
            2] * theta ** 6 + dist_coeffs[3] * theta ** 8)

        x_prime = (theta_d / r) * a
        y_prime = (theta_d / r) * b

        u = camera_matrix[0][0] * x_prime + camera_matrix[0][2]
        v = camera_matrix[1][1] * y_prime + camera_matrix[1][2]

        # 确保点在图像范围内
        if 0 <= u < image.shape[1] and 0 <= v < image.shape[0] and pixel[2] >= 0:
            color = image[int(v), int(u)]  # 提取像素颜色
            colors.append(color[::-1] / 255.0)  # BGR到RGB，归一化
        else:
            colors.append([0, 0, 0])  # 如果点超出图像范围，赋予黑色

    # 更新点云颜色
    pcd.colors = o3d.utility.Vector3dVector(colors)


def get_undistort_image(image, camera_matrix, dist_coeffs):
    R = np.eye(3)
    # Project point cloud to image
    mapx, mapy = cv2.fisheye.initUndistortRectifyMap(camera_matrix, dist_coeffs, R, camera_matrix, image.shape[:2],
                                                     cv2.CV_32FC1)

    undistort_image = cv2.remap(image, mapx, mapy, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT)

    return undistort_image


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


# 主程序
def main():
    filename = 'D:\\pc_visualizer\\show pointcloud\\hunter\\config_hunter.yaml'  # Adjust path as needed

    # Load parameters
    transformMatrix_l2c, camera_matrix, dist_coeffs = read_camera_parameters_from_yaml(filename)
    file_path = 'D:\\pc_visualizer\\show pointcloud\\hunter\\pose9_full.pcd'  # Adjust path as needed
    image_path = 'D:\\pc_visualizer\\show pointcloud\\hunter\\pose9.png'  # Adjust path as needed
    output_path = 'colored_output.pcd'

    pcd = o3d.io.read_point_cloud(file_path)
    image = cv2.imread(image_path)
    if image is None:
        print("Failed to read image.")
        return

    undistort_image = get_undistort_image(image, camera_matrix, dist_coeffs)
    project_to_undistored_image(pcd, camera_matrix, transformMatrix_l2c, undistort_image)

    distort_image = image.copy()
    image_pt = image.copy()
    distort_image_pt = image.copy()

    project_to_distored_image(pcd, camera_matrix, dist_coeffs, transformMatrix_l2c, distort_image)
    cv2.imshow("Projected on undistorted Image", undistort_image)
    cv2.imshow("Projected on distorted Image", distort_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    cv2.imwrite("projected_output.jpg", undistort_image)

    project_and_color_pcd_undistort(pcd, camera_matrix, transformMatrix_l2c, image_pt)
    # 保存着色后的点云
    save_colored_pcd(pcd, output_path)

    project_and_color_pcd_distort(pcd, camera_matrix, dist_coeffs, transformMatrix_l2c, distort_image_pt)
    # 保存着色后的点云
    save_colored_pcd(pcd, output_path)

    print(f"Colored point cloud saved to {output_path}")
    visualize_colored_point_cloud(output_path)


if __name__ == '__main__':
    main()
