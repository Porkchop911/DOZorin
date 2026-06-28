#!/usr/bin/python3
# coding=utf8
# @Author: Aiden
# @Date: 2024/12/31
import cv2
import copy
import math
import numpy as np
from sdk import common
from typing import Tuple, List, Optional

# Physical parameters of the robotic arm gripper (unit: meters)(机械臂夹持器的物理参数(单位:米))
GRIPPER_HB = 0.014  
GRIPPER_BC = 0.03 
GRIPPER_ED = 0.037
GRIPPER_DC = 0.022
EDC = math.radians(180 - 21)
GRIPPER_IH = 0.02
GRIPPER_IG = 0.005
LCD = math.acos((GRIPPER_HB - GRIPPER_IG) / GRIPPER_IH)
GRIPPER_EC = (GRIPPER_ED**2 + GRIPPER_DC**2 - 2 * GRIPPER_ED * GRIPPER_DC * math.cos(EDC)) ** 0.5
ECD = math.acos((GRIPPER_DC**2 + GRIPPER_EC**2 - GRIPPER_ED**2) / (2 * GRIPPER_DC * GRIPPER_EC))

def get_gripper_size(angle: float, angle_zero=200) -> Tuple[float, float]:
    """
    Calculate the width and height of the gripper based on the gripper angle(根据夹持器角度计算夹持器的宽度和高度)
    
    Args:
        angle: Gripper angle (0-1000)(夹持器角度(0-1000))
        angle_zero: Gripper angle zero point (default 200)(夹持器角度零点(默认200))
    Returns:
        width: Gripper width(夹持器宽度)
        height: Gripper height(夹持器高度)
    """
    angle = math.radians((angle - angle_zero) / 1000 * 180)
     
    GRIPPER_BJ = math.cos(angle) * GRIPPER_BC
    GRIPPER_HJ = GRIPPER_HB + GRIPPER_BJ
    GRIPPER_LC = math.cos(LCD + ECD) * GRIPPER_EC
    GRIPPER_KE = GRIPPER_HJ - GRIPPER_LC

    GRIPPER_KE = GRIPPER_HB + math.cos(angle) * GRIPPER_BC - math.cos(LCD + ECD) * GRIPPER_EC

    GRIPPER_JC = (GRIPPER_BC**2 - GRIPPER_BJ**2)**0.5
    GRIPPER_LE = (GRIPPER_EC**2 - GRIPPER_LC**2)**0.5
    gripper_depth = GRIPPER_JC + GRIPPER_LE
    gripper_width = 2*GRIPPER_KE
    
    return gripper_width, gripper_depth

def set_gripper_size(width: float) -> int:
    """
    Calculate the required angle for the gripper based on the target width(根据目标宽度计算夹持器需要的角度)
    
    Args:
        width: Target width (meters)(目标宽度(米))
        
    Returns:
        angle: Gripper angle (0-1000)(夹持器角度(0-1000))
    """
    width = width / 2
    a = (width - GRIPPER_HB + math.cos(LCD + ECD) * GRIPPER_EC) / GRIPPER_BC
    a = max(-1.0, min(1.0, a))
    return int(math.degrees(math.acos(a)) / 180 * 1000 + 200)

def world_to_pixels(world_points, K, T):
    """
    Convert world coordinate points to pixel coordinates(将世界坐标点转换为像素坐标)
    Args:
        world_points: List of world coordinate points(世界坐标点列表)
        K: Camera intrinsic matrix(相机内参矩阵)
        T: Extrinsic matrix [R|t](外参矩阵 [R|t])
    Returns:
        pixel_points: List of pixel coordinate points(像素坐标点列表)
    """
    pixel_points = []
    for wp in world_points:
        # Convert world coordinates to homogeneous coordinates(将世界坐标转换为齐次坐标)
        world_homo = np.append(wp, 1).reshape(4, 1)
        # Convert to the camera coordinate system through the extrinsic matrix(通过外参矩阵转换到相机坐标系)
        camera_point = np.dot(T, world_homo)
        # Project to the pixel plane(投影到像素平面)
        pixel_homo = np.dot(K, camera_point[:3])
        # Normalize(归一化)
        pixel = (pixel_homo / pixel_homo[2])[:2].reshape(-1)
        pixel_points.append(pixel)
    return pixel_points

def calculate_pixel_length(world_length, K, T):
    """
    Calculate the corresponding length in pixel coordinates for the length in world coordinates(计算世界坐标中的长度在像素坐标中的对应长度)
    Args:
        world_length: Length in world coordinates(世界坐标中的长度)
        K: Camera intrinsic matrix(相机内参矩阵)
        T: Extrinsic matrix(外参矩阵)
    Returns:
        pixel_length: Length in pixel coordinates(像素坐标中的长度)
    """
    # Define the starting point and direction(定义起始点和方向)
    start_point = np.array([0, 0, 0])  # Starting point(起始点)
    direction = np.array([0, 1, 0])  # Y direction(y方向)

    # Calculate the end point coordinates(计算终点坐标)
    end_point = start_point + direction * world_length
    # Convert the two endpoints to pixel coordinates(转换两个端点到像素坐标)
    pixels = world_to_pixels([start_point, end_point], K, T)
    # Calculate the pixel distance(计算像素距离)
    pixel_length = np.linalg.norm(pixels[1] - pixels[0])

    return int(pixel_length)

def get_plane_values(depth_image: np.ndarray, 
                    plane: Tuple[float, float, float, float],
                    intrinsic_matrix: np.ndarray) -> np.ndarray:
    """
    Calculate the distance from each point in the depth image to the plane(计算深度图像中每个点到平面的距离)
    
    Args:
        depth_image: Depth image(深度图像)
        plane: Plane equation parameters (a,b,c,d)(平面方程参数(a,b,c,d))
        intrinsic_matrix: Camera intrinsic matrix(相机内参矩阵)
        
    Returns:
        plane_values: Distance of each point to the plane(每个点到平面的距离)
    """
    a, b, c, d = plane
    # Extract camera intrinsics(提取相机内参)
    fx = intrinsic_matrix[0]
    fy = intrinsic_matrix[4]
    cx = intrinsic_matrix[2]
    cy = intrinsic_matrix[5]
    
    # Image size(图像尺寸)
    H, W = depth_image.shape
    
    # Generate a pixel coordinate grid(生成像素坐标网格)
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    
    # Calculate camera coordinates(计算相机坐标)
    z = depth_image / 1000.0  # Convert to meters(转换为米)
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy

    # Calculate the plane equation value(计算平面方程值)
    plane_values = a * x + b * y + c * z + d
    
    return plane_values

def create_roi_mask(
    depth_image: np.ndarray,
    bgr_image: np.ndarray,
    corners: np.ndarray,
    camera_info: object,
    extrinsic: np.ndarray,
    max_height: float,
    max_obj_height: float,
) -> np.ndarray:
    """
    Create a region of interest (ROI) mask(创建感兴趣区域(ROI)的遮罩)
    Args:
        depth_image: Depth image(深度图像)
        bgr_image: BGR image(BGR图像)
        corners: Corner coordinates(角点坐标)
        camera_info: Camera parameters(相机参数)
        extrinsic: Extrinsic matrix(外参矩阵)
        max_height: Maximum height(最大高度)
        max_obj_height: Maximum object height(物体最大高度)
    Returns:
        mask: ROI mask(ROI遮罩)
    """
    # image_height, image_width = depth_image.shape[:2]
    
    # # Decompose the extrinsic matrix(分解外参矩阵)
    # translation_vec = extrinsic[:1]
    # rotation_mat = extrinsic[1:]
    # corners_copy = copy.deepcopy(corners)
    
    # # Project the center point(投影中心点)
    # center_points, _ = cv2.projectPoints(
    #     corners_copy[-1:],
    #     np.array(rotation_mat),
    #     np.array(translation_vec),
    #     np.matrix(camera_info.k).reshape(1, -1, 3),
    #     np.array(camera_info.d)
    # )
    # center_points = np.int32(center_points).reshape(2)

    # # Calculate the extrinsics after plane shift(计算平面偏移后的外参)
    # shifted_tvec, shifted_rmat = common.extristric_plane_shift(
    #     np.array(translation_vec).reshape((3, 1)),
    #     np.array(rotation_mat),
    #     max_obj_height
    # )
    
    # # Project other corner points(投影其他角点)
    # projected_points, _ = cv2.projectPoints(
    #     corners_copy[:-1],
    #     np.array(shifted_rmat),
    #     np.array(shifted_tvec),
    #     np.matrix(camera_info.k).reshape(1, -1, 3),
    #     np.array(camera_info.d)
    # )
    # projected_points = np.int32(projected_points).reshape(-1, 2)
    
    # # Calculate ROI boundaries(计算ROI边界)
    # x_min = max(0, min(projected_points[:, 0]))
    # x_max = min(image_width, max(projected_points[:, 0]))
    # y_min = max(0, min(projected_points[:, 1]))
    # y_max = min(image_height, max(projected_points[:, 1]))
   
    # # Draw the ROI bounding box on the BGR image(在BGR图像上绘制ROI框)
    # # cv2.rectangle(bgr_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

    # # Create ROI region(创建ROI区域)
    # x, y = x_min + 10, y_min - 40
    # w, h = x_max - x_min, y_max - y_min
    
    # # Create mask(创建遮罩)
    # mask = np.zeros_like(depth_image)
    # x2 = min(x + w, image_width)
    # y2 = max(y, 0)
    # mask[y2:y+h, x:x2] = depth_image[y2:y+h, x:x2]

    # # Set the area outside the corresponding region in the depth image to 0(将深度图像中对应的区域外设置为0)
    # depth_image[mask == 0] = max_height

    # return depth_image

    # Deep copy to avoid accidental modification of the original data by the upper layer(深拷贝避免上层意外修改原始数据)
    full_depth = depth_image.copy()

    # Uniformly fill invalid points as max_height to prevent subsequent algorithms from treating them as the closest points(把无效点统一填成 max_height，防止后续算法将其当作最近点)
    full_depth[full_depth == 0] = max_height

    return full_depth

def find_depth_range(depth_image: np.ndarray, max_distance: float) -> Tuple[float, float]:
    """
    Find the minimum in the depth image(查找深度图像中的最小)
    Args:
        depth_image: Depth image(深度图像)
    Returns:
        min_distance: Minimum distance (mm)(最小距离(mm))
    """
    height, width = depth_image.shape[:2]
    
    # Process depth data(处理深度数据)
    depth = np.copy(depth_image).reshape(-1)
    depth[depth <= 0] = max_distance  # Set invalid values to max_distance(将无效值设为max_distance)
    
    # Find the closest point(找到最近点)
    min_idx = np.argmin(depth)
    min_y, min_x = min_idx // width, min_idx % width
    min_distance = depth_image[min_y, min_x] 
    
    return min_distance

def extract_contours(
    plane_values: np.ndarray,
    filter_height: float
) -> List[np.ndarray]:
    """
    Extract contours from the depth image(提取深度图像中的轮廓)
    Args:
        plane_values: Plane values(平面值)
        filter_height: Filter height(过滤高度)
    Returns:
        contours: List of contours(轮廓列表)
    """
    # Filter height(过滤高度)
    filtered_image = np.where(plane_values <= filter_height, 0, 255).astype(np.uint8)
    
    # Binarization and contour extraction(二值化和轮廓提取)
    _, binary = cv2.threshold(filtered_image, 1, 255, cv2.THRESH_BINARY)
    # cv2.imshow(color, binary)
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    return contours

def convert_depth_to_camera_coords(
    pixel_coords: Tuple[float, float, float],
    intrinsic_matrix: np.ndarray
) -> np.ndarray:
    """
    Convert depth pixel coordinates to the camera coordinate system(将深度像素坐标转换为相机坐标系)
    Args:
        pixel_coords: Pixel coordinates (x, y, z)(像素坐标 (x, y, z))
        intrinsic_matrix: Camera intrinsic matrix(相机内参矩阵)
    Returns:
        camera_coords: Coordinates in the camera coordinate system(相机坐标系下的坐标)
    """
    fx, fy = intrinsic_matrix[0], intrinsic_matrix[4]
    cx, cy = intrinsic_matrix[2], intrinsic_matrix[5]
    px, py, pz = pixel_coords
    
    x = (px - cx) * pz / fx
    y = (py - cy) * pz / fy
    z = pz
    
    return np.array([x, y, z])

def calculate_world_position(
    pixel_x: float,
    pixel_y: float,
    depth: float,
    plane: Tuple[float, float, float, float],
    endpoint: np.ndarray,
    hand2cam_tf_matrix: np.ndarray,
    intrinsic_matrix: np.ndarray,
    camera_roll_offset: float = 6.0,  #0.96  0.0   Front and rear error modification(前后误差修改)
    camera_pitch_offset: float = 0.0, #-0.73
    camera_yaw_offset: float = 0.0
) -> np.ndarray:
    """
    Calculate the position in the world coordinate system (eliminate camera Roll, Pitch, Yaw tilt)(计算世界坐标系中的位置 (消除摄像头 Roll, Pitch, Yaw 倾斜))

    Args:
        pixel_x: Pixel x coordinate(像素x坐标)
        pixel_y: Pixel y coordinate(像素y坐标)
        depth: Depth value (mm)(深度值 (mm))
        plane: Plane parameters (a, b, c, d)(平面参数 (a, b, c, d))
        endpoint: Pose matrix of the end effector in the world coordinate system (4x4)(末端执行器在世界坐标系中的位姿矩阵 (4x4))
        hand2cam_tf_matrix: Hand-eye calibration matrix, converts the camera coordinate system to the end effector coordinate system (4x4)(手眼标定矩阵，将相机坐标系转换到末端执行器坐标系 (4x4))
        intrinsic_matrix: Camera intrinsic matrix (3x3)(相机内参矩阵 (3x3))
        camera_roll_offset: Tilt angle of the camera Roll axis (degrees)(相机 Roll 轴的倾斜角度 (度))
        camera_pitch_offset: Tilt angle of the camera Pitch axis (degrees)(相机 Pitch 轴的倾斜角度 (度))
        camera_yaw_offset: Tilt angle of the camera Yaw axis (degrees)(相机 Yaw 轴的倾斜角度 (度))

    Returns:
        world_position: Position in the world coordinate system(世界坐标系中的位置)
    """

    # 1. Eliminate camera tilt: Construct rotation matrix(1. 消除相机倾斜：构建旋转矩阵)
    roll_offset_rad = math.radians(camera_roll_offset)
    pitch_offset_rad = math.radians(camera_pitch_offset)
    yaw_offset_rad = math.radians(camera_yaw_offset)

    # Construct the matrix for rotation around the X axis (Roll)(构建绕 X 轴旋转的矩阵 (Roll))
    rotation_x = np.array([
        [1, 0, 0],
        [0, math.cos(roll_offset_rad), -math.sin(roll_offset_rad)],
        [0, math.sin(roll_offset_rad), math.cos(roll_offset_rad)]
    ])

    # Construct the matrix for rotation around the Y axis (Pitch)(构建绕 Y 轴旋转的矩阵 (Pitch))
    rotation_y = np.array([
        [math.cos(pitch_offset_rad), 0, math.sin(pitch_offset_rad)],
        [0, 1, 0],
        [-math.sin(pitch_offset_rad), 0, math.cos(pitch_offset_rad)]
    ])

    # Construct the matrix for rotation around the Z axis (Yaw)(构建绕 Z 轴旋转的矩阵 (Yaw))
    rotation_z = np.array([
        [math.cos(yaw_offset_rad), -math.sin(yaw_offset_rad), 0],
        [math.sin(yaw_offset_rad), math.cos(yaw_offset_rad), 0],
        [0, 0, 1]
    ])

    #  Combine rotation matrices (pay attention to the order of rotation)(组合旋转矩阵 (注意旋转顺序))
    # ZYX (Yaw, Pitch, Roll)
    camera_rotation_correction_matrix = np.eye(4)  # Initialize as an identity matrix(初始化为单位矩阵)
    camera_rotation_correction_matrix[:3, :3] = np.dot(rotation_z, np.dot(rotation_y, rotation_x))

    # 2. Convert pixel coordinates to the camera coordinate system(2. 将像素坐标转换为相机坐标系)
    camera_z = depth / 1000.0  # Convert to meters(转换为米)
    camera_x = (pixel_x - intrinsic_matrix[2]) * camera_z / intrinsic_matrix[0]
    camera_y = (pixel_y - intrinsic_matrix[5]) * camera_z / intrinsic_matrix[4]
    camera_position = np.array([camera_x, camera_y, camera_z, 1])  # Homogeneous coordinates(齐次坐标)

    # 3. Eliminate camera tilt: Convert camera coordinates to the un-tilted camera coordinate system(3. 消除相机倾斜：将相机坐标转换到未倾斜的相机坐标系)
    # Note: hand2cam_tf_matrix has already converted the coordinates to the *tilted* camera coordinate system(注意：hand2cam_tf_matrix 已经将坐标转换到了 *倾斜的* 相机坐标系)
    #   First convert the coordinates back to the *un-tilted* camera coordinate system(我们需要先将坐标转换回 *未倾斜的* 相机坐标系)
    camera_position_corrected = np.dot(camera_rotation_correction_matrix, camera_position)

    # 4. Convert the (corrected) camera coordinate system to the end effector coordinate system(4. 将（校正后的）相机坐标系转换到末端执行器坐标系)
    hand_position = np.dot(hand2cam_tf_matrix, camera_position_corrected)

    # 5. Convert the end effector coordinate system to the world coordinate system(5. 将末端执行器坐标系转换到世界坐标系)
    world_position_homo = np.dot(endpoint, hand_position)
    world_position = world_position_homo[:3] / world_position_homo[3] 

    return world_position

