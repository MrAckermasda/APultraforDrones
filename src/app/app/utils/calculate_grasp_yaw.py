#!/usr/bin/python3
# coding=utf8
# @Author: Aiden
# @Date: 2024/12/31
"""
Gripper yaw angle calculation module for robotic arm grasping (机械臂抓取物体时的夹爪旋转角度计算模块)

Angle selection strategy: (角度选择策略)
1. First, create two perpendicular rectangles at the center of the object representing possible gripper directions; both rectangles are parallel to the object edges (首先在物体的中心创建两个互相垂直的矩形代表夹爪可能的抓取方向，两个矩形和物体的两边是平行的)
2. Check whether these two rectangles intersect with other objects; intersection indicates a collision (检测这两个矩形是否与其他物体有相交，有相交即发生碰撞)
3. If neither collides, choose the direction with smaller rotation angle (如果都没有碰撞，选择旋转角度较小的方向)
4. If one direction collides, choose the other direction (如果有一个方向发生碰撞，选择另一个方向)
5. If both collide, grasping is impossible (如果都发生碰撞，则无法抓取)
"""

import cv2
import math
import numpy as np

def calculate_distance(point1, point2):
    """Calculate Euclidean distance between two points (计算两点间的欧氏距离)

    Args:
        point1: Coordinates of the first point [x, y] (第一个点的坐标 [x, y])
        point2: Coordinates of the second point [x, y] (第二个点的坐标 [x, y])

    Returns:
        float: Distance between the points (两点间的距离)
    """
    return round(math.sqrt(pow(point1[0] - point2[0], 2) + pow(point1[1] - point2[1], 2)), 5)


def calculate_intersection_area(rect1, rect2):
    """Calculate intersection area of two rotated rectangles for collision checking (计算两个旋转矩形的相交面积，用来判断是否有碰撞)

    Args:
        rect1: First rotated rectangle ((x,y), (w,h), angle) (第一个旋转矩形 ((x,y), (w,h), angle))
        rect2: Second rotated rectangle ((x,y), (w,h), angle) (第二个旋转矩形 ((x,y), (w,h), angle))

    Returns:
        float: Intersection area; returns None if no intersection (相交面积，如果不相交返回None)
    """
    intersection = cv2.rotatedRectangleIntersection(rect1, rect2)[1]
    return cv2.contourArea(intersection) if intersection is not None else None


def detect_collision(target_point, points, rect1, rect2, collision_radius):
    """Detect collisions between the target rectangle and surrounding objects by calculating intersection areas (检测目标矩形与周围物体是否存在碰撞，通过计算两个矩形的相交面积来判断)

    Args:
        target_point: Pixel position of the target object (目标物体的像素位置信息)
        points: Positions of all objects (所有物体的位置信息)
        rect1: First rectangle for checking (检测用的第一个矩形)
        rect2: Second rectangle (perpendicular to rect1) for checking (检测用的第二个矩形(与rect1垂直))
        collision_radius: Collision detection radius (碰撞检测半径)

    Returns:
        list: List of rectangles that would collide; returns None if no collision (发生碰撞的矩形列表，无碰撞返回None)
    """

    collision_rects = []  # Rectangles that would collide (会发生碰撞的矩形夹取矩形)
    
    if len(points) <= 1:
        return collision_rects

    for i in points:  # Iterate over all objects (遍历所有物体)
        if i[0] == target_point[0] and i[1] == target_point[1]:
            continue

        other_point = i[2]  # Coordinates of object for collision check with target (和目标物体进行碰撞检测的物体坐标)
        if calculate_distance(target_point[2], other_point) >= collision_radius:  # Skip if distance exceeds collision radius (如果他们间的距离超过了碰撞检测半径)
            continue

        check_rect = (i[2], i[3], i[-1])
        # logger.info(f'check_rect: {check_rect} rect1: {rect1} rect2: {rect2}') 
        # Simplified collision detection: use bounding rectangle of other object to check intersection with target gripper rectangles 
        # (简化碰撞检测，用其他物体的最小外接矩形来和目标物体的夹取矩形进行相交面积判断)
        if calculate_intersection_area(rect1, check_rect) is not None:
            collision_rects.append(1)
        if calculate_intersection_area(rect2, check_rect) is not None:
            collision_rects.append(2)

    return collision_rects


def get_parallel_line(rect):
    """
    Line passing through rectangle center and parallel to its long side (经过矩形中心且与长边平行的线)
    
    Args:
        rect: Rectangle parameters ((cx, cy), (width, height), angle) (矩形参数 ((cx, cy), (width, height), angle))
    """
    # Extract rectangle parameters (提取矩形参数)
    (cx, cy), (width, height), angle = rect
    
    # Convert angle to radians (将角度转换为弧度)
    theta = np.radians(angle)
    
    # Compute direction vector along the long side (计算方向向量（沿长边方向）)
    dx = np.cos(theta) * max(width, height)  # X component along long side (长边的 x 分量)
    dy = np.sin(theta) * max(width, height)  # Y component along long side (长边的 y 分量)

    # Compute two endpoints of the line (计算线的两个端点)
    point1 = (int(cx - dx / 2), int(cy - dy / 2))
    point2 = (int(cx + dx / 2), int(cy + dy / 2))
    
    return point1, point2 

def calculate_gripper_yaw_angle(target_points, points, gripper_size, yaw):
    """Calculate optimal gripper rotation angle (计算最优的夹爪旋转角度)

    Args:
        target_points: Pixel position of the target object (目标物体的像素位置信息)
        points: Pixel positions of all objects [label, index, (center_x, center_y), [width, height], angle] (所有物体的像素位置信息,[label, index, (center_x, center_y), [width, height], angle])
        gripper_size: Maximum gripper size when grasping the object [width, height] (夹爪抓取物体时张的最大时的尺寸, [width, height])
        yaw: Initial yaw (偏航角)
    
    Returns:
        tuple: (Optimal rotation angle, gripper contour points) (最优旋转角度, 夹爪轮廓点)
    """
    center_x, center_y = target_points[2][0], target_points[2][1]  # 目标物体的像素位置
    angle = target_points[-1] # 目标物体的旋转角度，是通过cv2.minAreaRect()返回的角度
    
    # Get gripper dimensions (获取夹爪尺寸)
    gripper_width, gripper_height = gripper_size
    
    # Objects beyond this radius from the center won't collide (以物体中心为圆心，和物体超过这个半径距离的物体是不会碰撞的)
    half_width = gripper_width // 2
    half_height = gripper_height // 2
    collision_radius = math.sqrt(half_width ** 2 + half_height ** 2)

    angle1 = angle
    angle2 = angle - 90

    # Create two perpendicular rectangles based on gripper size (根据夹爪的尺寸创建两个互相垂直的矩形)
    rect1 = ((center_x, center_y), (gripper_width, gripper_height), angle1)
    rect2 = ((center_x, center_y), (gripper_width, gripper_height), angle2)
    
    # Detect gripper rectangles that would collide (检测会发生碰撞的夹取矩形)
    collisions = detect_collision(target_points, points, rect1, rect2, collision_radius)

    line1 = get_parallel_line(rect1)
    line2 = get_parallel_line(rect2)

    yaw1 = yaw + angle1
    if yaw1 < 0:
        yaw2 = yaw1 + 90
    else:
        yaw2 = yaw1 - 90 

    if len(collisions) == 0:
        if abs(yaw1) < abs(yaw2):
            return yaw1, line1, rect1
        else:
            return yaw2, line2, rect2
    elif len(collisions) == 1:
        if collisions[0] == 1:
            return yaw2, line2, rect2
        else:
            return yaw1, line1, rect1
    else:
        return None
