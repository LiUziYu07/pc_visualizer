import numpy as np
import cv2

from tools.coordinate_convert import get_transformation_pt
from config import hunter_config
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