#!/usr/bin/python3
# coding=utf8
# Map undistorted image pixel coordinates back to original (distorted) image pixels 
# (将校正后图像的像素点反推回原始（带畸变）图像中的像素点)

import numpy as np

def distort_point(x, y, k1, k2, p1, p2, k3=0):
    """
    Compute distorted normalized coordinates based on forward distortion model (根据正向畸变模型计算带畸变归一化坐标)
    Inputs:
      x, y: Normalized coordinates (归一化坐标（原始/待求变量）)
      k1, k2, k3: Radial distortion coefficients (径向畸变参数)
      p1, p2: Tangential distortion coefficients (切向畸变参数)
    Outputs:
      x_dist, y_dist: Distorted normalized coordinates (经过畸变映射后的归一化坐标)
    """
    r2 = x * x + y * y
    radial = 1 + k1 * r2 + k2 * (r2 ** 2) + k3 * (r2 ** 3)
    x_dist = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
    y_dist = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
    return x_dist, y_dist

def invert_undistort_point(xu, yu, k1, k2, p1, p2, k3=0, max_iter=100, tol=1e-10):
    """
    Solve for original normalized coordinates (x, y) by iteration so that after forward distortion mapping it equals the known undistorted coordinates (xu, yu) (通过迭代方法求解原始归一化坐标 (x, y)，使得经过正向畸变映射后等于已知校正后的归一化坐标 (xu, yu))
    Initial guess: (x, y) = (xu, yu) assuming small distortion (初始值取 (x, y) = (xu, yu)（假设畸变较小时差异不大）)
    """
    x = xu
    y = yu
    for i in range(max_iter):
        # Compute distorted coordinates for current estimate (计算当前估计下的畸变映射结果)
        x_dist, y_dist = distort_point(x, y, k1, k2, p1, p2, k3)
        # Compute error (计算误差)
        dx = xu - x_dist
        dy = yu - y_dist
        if dx * dx + dy * dy < tol:
            # Stop if converged (收敛后退出)
            break
        # Simple fixed-step update, damping factor can be added to improve convergence (这里采用简单的固定步长更新，可根据实际情况加入阻尼因子改善收敛性)
        x += dx
        y += dy
    return x, y

def undistorted_to_distorted_pixel(u_undist, v_undist, camera_matrix, dist_coeffs):
    """
    Map undistorted image pixel coordinates back to original (distorted) image pixels using camera intrinsics and distortion coefficients (根据相机内参与畸变系数，将校正后图像的像素点反推回原始（带畸变）图像中的像素点)
    
    Inputs:
      u_undist, v_undist: Pixel coordinates in undistorted image (校正后图像的像素坐标)
      camera_matrix: 3x3 camera intrinsic matrix, e.g. [[fx, 0, cx],
                                                        [0, fy, cy],
                                                        [0,  0,  1]] (3x3 相机内参矩阵)
      dist_coeffs: Distortion coefficients array [k1, k2, p1, p2, (k3)] (畸变系数数组 [k1, k2, p1, p2, (k3)])
                  If k3 is not available, only pass the first 4 coefficients (若没有 k3 可只传前 4 个参数)
      
    Outputs:
      u_dist, v_dist: Pixel coordinates in original (distorted) image (原始图像（带畸变）的像素坐标)
    """
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]
    
    # Convert undistorted pixel coordinates to normalized coordinates (将校正后的像素坐标转换为归一化坐标)
    xu = (u_undist - cx) / fx
    yu = (v_undist - cy) / fy

    # Extract distortion parameters (提取畸变参数)
    k1 = dist_coeffs[0]
    k2 = dist_coeffs[1]
    p1 = dist_coeffs[2]
    p2 = dist_coeffs[3]
    k3 = 0.0
    if len(dist_coeffs) >= 5:
        k3 = dist_coeffs[4]
        
    # Solve original normalized coordinates using iteration (利用迭代法反求原始归一化坐标)
    xd, yd = invert_undistort_point(xu, yu, k1, k2, p1, p2, k3)
    
    # Convert original normalized coordinates to pixel coordinates (将原始归一化坐标转换为像素坐标)
    u_dist = xd * fx + cx
    v_dist = yd * fy + cy
    return int(u_dist), int(v_dist)


