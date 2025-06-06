#!/usr/bin/env python

import rospy
import json
import os
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import struct

def create_point_cloud(poses):
    """从姿态列表创建点云数据"""
    points = []
    for pose in poses:
        # 获取位置信息
        x = pose['pose']['position']['x']
        y = pose['pose']['position']['y']
        z = pose['pose']['position']['z']
        
        # 获取方向信息（四元数）
        qx = pose['pose']['orientation']['x']
        qy = pose['pose']['orientation']['y']
        qz = pose['pose']['orientation']['z']
        qw = pose['pose']['orientation']['w']
        
        # 将夹爪状态转换为强度值
        intensity = 255 if pose['gripper_open'] else 0
        
        # 添加点
        points.append([x, y, z, intensity, qx, qy, qz, qw])
    
    return np.array(points, dtype=np.float32)

def write_pcd_file(points, filename):
    """将点云数据写入PCD文件"""
    with open(filename, 'w') as f:
        # 写入PCD文件头
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z intensity qx qy qz qw\n")
        f.write("SIZES 4 4 4 4 4 4 4 4\n")
        f.write("TYPES F F F F F F F F\n")
        f.write("COUNTS 1 1 1 1 1 1 1 1\n")
        f.write(f"WIDTH {len(points)}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(points)}\n")
        f.write("DATA ascii\n")
        
        # 写入点云数据
        for point in points:
            f.write(f"{point[0]} {point[1]} {point[2]} {point[3]} {point[4]} {point[5]} {point[6]} {point[7]}\n")

def convert_json_to_pcd(json_file):
    """将JSON文件转换为PCD文件"""
    # 读取JSON文件
    with open(json_file, 'r') as f:
        data = json.load(f)
    
    # 创建点云数据
    points = create_point_cloud(data['poses'])
    
    # 生成输出文件名
    pcd_file = json_file.replace('.json', '.pcd')
    
    # 写入PCD文件
    write_pcd_file(points, pcd_file)
    print(f"Converted {json_file} to {pcd_file}")

def main():
    # 获取actions目录
    actions_dir = os.path.join(os.path.dirname(__file__), '../actions')
    
    # 转换所有JSON文件
    for filename in os.listdir(actions_dir):
        if filename.endswith('.json'):
            json_file = os.path.join(actions_dir, filename)
            convert_json_to_pcd(json_file)

if __name__ == '__main__':
    main() 