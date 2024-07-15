# hunter_config.py

# Pointcloud Projection Param
COLOMAP = [
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

# CAMERA Param
CAMERA_FILE_PATH = 'D:\\pc_visualizer\\data\hunter\\config_hunter.yaml'

# PCD Param
PCD_FILE_PATH = 'D:\\pc_visualizer\\data\\hunter\\pose9_full.pcd'

# IMAGE Param
IMAGE_FILE_PATH = 'D:\\pc_visualizer\\data\\hunter\\pose9.png'

# OUTPUT Param
OUTPUT_DISTORT_IMAGE_FILE_PATH = 'D:\\pc_visualizer\\output\\output_distort.jpg'
OUTPUT_UNDISTORT_IMAGE_FILE_PATH = 'D:\\pc_visualizer\\output\\output_undistort.jpg'

OUTPUT_DISTORT_PCD_FILE_PATH = 'D:\\pc_visualizer\\output\\output_distort.pcd'
OUTPUT_UNDISTORT_PCD_FILE_PATH = 'D:\\pc_visualizer\\output\\output_undistort.pcd'

OUTPUT_COLOR_PCD_UNDISTORT_FILE_PATH = 'D:\\pc_visualizer\\output\\output_color.pcd'
OUTPUT_COLOR_PCD_DISTORT_FILE_PATH = 'D:\\pc_visualizer\\output\\output_color.pcd'
