import numpy as np
import open3d as o3d

from tools.coordinate_convert import get_transformation_pt
from config import hunter_config
config = hunter_config

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
