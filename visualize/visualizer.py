import numpy as np
import cv2
import open3d as o3d
import math

import numpy as np
import yaml
from config import hunter_config
from tools.undistort_image import get_undistort_image
from tools.read_config import read_parameters_from_yaml
from tools.coordinate_convert import get_transformation_pt

config = hunter_config


# 将点云投影到图像平面并应用畸变
def project_pc_to_image(pcd, camera_matrix, distort_coeffs, distort_type, transform_matrix, image):
    points = np.asarray(pcd.points)
    distances_info = []
    for point in points:
        dist_info = get_transformation_pt(camera_matrix, distort_coeffs, distort_type, transform_matrix, image.shape[1],
                                          image.shape[0], point)
        if dist_info:
            distances_info.append(dist_info)

    distances_info = np.array(distances_info)
    min_dist, max_dist = distances_info[:, 2].min(), distances_info[:, 2].max()
    for u_tmp, y_tmp, dist in distances_info:
        rangie_idx = int(min(round(((dist - min_dist) / (max_dist - min_dist)) * 49), 49))
        cv2.circle(image, (int(u_tmp), int(y_tmp)), 2, (
            255 * config.COLOMAP[49 - rangie_idx][0], 255 * config.COLOMAP[49 - rangie_idx][1],
            255 * config.COLOMAP[49 - rangie_idx][2]), -1)

    return image


def project_and_color_pcd(pcd, camera_matrix, distort_coeffs, distort_type, transform_matrix, image):
    points = np.asarray(pcd.points)
    colors = []

    for point in points:
        dist_info = get_transformation_pt(camera_matrix, distort_coeffs, distort_type, transform_matrix, image.shape[1],
                                          image.shape[0], point)
        if dist_info:
            x, y, _ = dist_info
            color = image[int(y), int(x)]  # 提取像素颜色
            colors.append(color[::-1] / 255.0)  # BGR到RGB，归一化
        else:
            colors.append([0, 0, 0])  # 如果点超出图像范围，赋予黑色

    # 更新点云颜色
    pcd.colors = o3d.utility.Vector3dVector(colors)

    return pcd


def visualize_colored_point_cloud(pcd):
    # 检查点云是否为空
    if pcd.is_empty():
        print("Loaded point cloud is empty.")
        return

    # 可视化点云
    print("Displaying the colored point cloud...")
    o3d.visualization.draw_geometries([pcd], window_name="Colored Point Cloud", point_show_normal=True)


def point_to_image(pcd, camera_matrix, distort_coeffs, distort_type, transformMatrix_l2c, image, undistort_image):
    undistort_image_res = project_pc_to_image(pcd, camera_matrix, distort_coeffs, "undistort", transformMatrix_l2c,
                                              undistort_image.copy())
    distort_image_res = project_pc_to_image(pcd, camera_matrix, distort_coeffs, distort_type, transformMatrix_l2c,
                                            image.copy())

    cv2.imshow("Projected on undistorted Image", undistort_image_res)
    cv2.imshow("Projected on distorted Image", distort_image_res)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    cv2.imwrite(hunter_config.OUTPUT_UNDISTORT_IMAGE_FILE_PATH, undistort_image)
    cv2.imwrite(config.OUTPUT_DISTORT_IMAGE_FILE_PATH, undistort_image)


def color_to_point(pcd, camera_matrix, distort_coeffs, distort_type, transformMatrix_l2c, image, undistort_image):
    undistort_pcd_res = project_and_color_pcd(pcd, camera_matrix, distort_coeffs, "undistort",
                                              transformMatrix_l2c, undistort_image.copy())

    print(f"Colored point cloud saved to {config.OUTPUT_COLOR_PCD_UNDISTORT_FILE_PATH}")
    visualize_colored_point_cloud(undistort_pcd_res)
    o3d.io.write_point_cloud(config.OUTPUT_UNDISTORT_PCD_FILE_PATH, pcd, write_ascii=True)

    distort_pcd_res = project_and_color_pcd(pcd, camera_matrix, distort_coeffs, distort_type,
                                            transformMatrix_l2c,
                                            image.copy())

    print(f"Colored point cloud saved to {hunter_config.OUTPUT_COLOR_PCD_DISTORT_FILE_PATH}")
    visualize_colored_point_cloud(distort_pcd_res)
    o3d.io.write_point_cloud(config.OUTPUT_DISTORT_PCD_FILE_PATH, pcd, write_ascii=True)


if __name__ == '__main__':
    # Load parameters
    transformMatrix_l2c, camera_matrix, distort_coeffs, distort_type = read_parameters_from_yaml(
        hunter_config.CAMERA_FILE_PATH)

    pcd = o3d.io.read_point_cloud(hunter_config.PCD_FILE_PATH)
    image = cv2.imread(config.IMAGE_FILE_PATH)

    if image is None:
        print("Failed to read image.")
        raise FileNotFoundError(f"Failed to read image.{hunter_config.IMAGE_FILE_PATH}")

    undistort_image = get_undistort_image(image.copy(), camera_matrix, distort_coeffs, distort_type)

    point_to_image(pcd, camera_matrix, distort_coeffs, distort_type, transformMatrix_l2c, image, undistort_image)

    color_to_point(pcd, camera_matrix, distort_coeffs, distort_type, transformMatrix_l2c, image, undistort_image)
