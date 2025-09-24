#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CILQR数据可视化工具
读取C++程序生成的JSON数据文件并生成可视化图像
特性：
- 批量处理所有数据文件
- 生成单帧可视化图像
- 创建动画GIF
- 二值地图显示：可行驶路径为白色，不可行驶路径为黑色
"""

import json
import numpy as np
import matplotlib.pyplot as plt
import os
import glob
from matplotlib.patches import Rectangle
import math

def load_data(filename):
    """加载JSON数据文件"""
    try:
        with open(filename, 'r') as f:
            return json.load(f)
    except Exception as e:
        print(f"Error loading {filename}: {e}")
        return None

def load_map_data():
    """从maps目录加载地图数据"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    map_file = os.path.join(script_dir, '..', 'maps', 'map_data.json')
    
    try:
        with open(map_file, 'r') as f:
            return json.load(f)
    except Exception as e:
        print(f"Warning: Could not load map data from {map_file}: {e}")
        return None

def draw_vehicle(ax, x, y, theta, gamma, color='blue', alpha=1.0, vehicle_type='ego', vehicle_model=None, obstacle_data=None):
    """绘制车辆
    对于自车(ego): x, y是前轴中心位置，需要根据铰接角绘制前后两个车厢
    对于障碍物车辆: x, y是车辆中心位置，绘制单个矩形
    """
    # 使用车辆模型参数，如果没有提供则使用默认值
    if vehicle_type == 'ego' and vehicle_model:
        half_len = vehicle_model['box_length'] / 2.0
        half_wid = vehicle_model['width'] / 2.0
        lf = vehicle_model['lf']
        lr = vehicle_model['lr']
    elif vehicle_type == 'obstacle' and obstacle_data:
        # 使用障碍物数据中的尺寸信息
        half_len = obstacle_data.get('length', 2.7) / 2.0
        half_wid = obstacle_data.get('width', 2.0) / 2.0
        lf = 1.6  # 障碍物不需要这些参数，但保留以防万一
        lr = 1.13
    else:
        # 默认值
        if vehicle_type == 'ego':
            half_len = 1.04
            half_wid = 1.0
            lf = 1.6
            lr = 1.13
        else:  # obstacle
            half_len = 1.35
            half_wid = 1.0
            lf = 1.6
            lr = 1.13
    
    c = math.cos(theta)
    s = math.sin(theta)
    
    if vehicle_type == 'ego':
        # 自车是铰接车，需要绘制前后两个车厢
        # 前轴中心位置（输入参数x,y就是前轴中心）
        front_axle_x = x
        front_axle_y = y
        
        # 根据前轴中心和lf计算铰接点位置
        hitch_x = front_axle_x - lf * c
        hitch_y = front_axle_y - lf * s
        
        # 根据铰接点、铰接角gamma和lr计算后轴中心位置
        rear_theta = theta + gamma
        rear_c = math.cos(rear_theta)
        rear_s = math.sin(rear_theta)
        rear_axle_x = hitch_x - lr * rear_c
        rear_axle_y = hitch_y - lr * rear_s
        
        # 绘制前车厢（以前轴中心为基准）
        front_corners = np.array([
            [front_axle_x + half_len * c - half_wid * s, front_axle_y + half_len * s + half_wid * c],
            [front_axle_x + half_len * c + half_wid * s, front_axle_y + half_len * s - half_wid * c],
            [front_axle_x - half_len * c + half_wid * s, front_axle_y - half_len * s - half_wid * c],
            [front_axle_x - half_len * c - half_wid * s, front_axle_y - half_len * s + half_wid * c],
            [front_axle_x + half_len * c - half_wid * s, front_axle_y + half_len * s + half_wid * c]  # 闭合
        ])
        ax.plot(front_corners[:, 0], front_corners[:, 1], color=color, alpha=alpha, linewidth=1)
        ax.fill(front_corners[:, 0], front_corners[:, 1], color=color, alpha=0.85)
        
        # 绘制后车厢（以后轴中心为基准）
        rear_corners = np.array([
            [rear_axle_x + half_len * rear_c - half_wid * rear_s, rear_axle_y + half_len * rear_s + half_wid * rear_c],
            [rear_axle_x + half_len * rear_c + half_wid * rear_s, rear_axle_y + half_len * rear_s - half_wid * rear_c],
            [rear_axle_x - half_len * rear_c + half_wid * rear_s, rear_axle_y - half_len * rear_s - half_wid * rear_c],
            [rear_axle_x - half_len * rear_c - half_wid * rear_s, rear_axle_y - half_len * rear_s + half_wid * rear_c],
            [rear_axle_x + half_len * rear_c - half_wid * rear_s, rear_axle_y + half_len * rear_s + half_wid * rear_c]  # 闭合
        ])
        ax.plot(rear_corners[:, 0], rear_corners[:, 1], color=color, alpha=alpha, linewidth=1)
        ax.fill(rear_corners[:, 0], rear_corners[:, 1], color=color, alpha=0.85)
        
        # 绘制从前轴中心到铰接点的连接线
        ax.plot([front_axle_x, hitch_x], [front_axle_y, hitch_y], 
                color='black', linewidth=2, alpha=0.8)
        
        # 绘制从后轴中心到铰接点的连接线
        ax.plot([rear_axle_x, hitch_x], [rear_axle_y, hitch_y], 
                color='black', linewidth=2, alpha=0.8)
        
        # 绘制铰接点
        ax.plot(hitch_x, hitch_y, 'ko', markersize=1.67, alpha=0.8)
    else:
        # 障碍物车辆是普通轮式车，绘制单个矩形
        # x, y是车辆中心位置
        corners = np.array([
            [x + half_len * c - half_wid * s, y + half_len * s + half_wid * c],
            [x + half_len * c + half_wid * s, y + half_len * s - half_wid * c],
            [x - half_len * c + half_wid * s, y - half_len * s - half_wid * c],
            [x - half_len * c - half_wid * s, y - half_len * s + half_wid * c],
            [x + half_len * c - half_wid * s, y + half_len * s + half_wid * c]  # 闭合
        ])
        ax.plot(corners[:, 0], corners[:, 1], color=color, alpha=alpha, linewidth=1)
        ax.fill(corners[:, 0], corners[:, 1], color=color, alpha=0.85)



def visualize_frame(data, frame_num, map_data=None, save_path=None):
    """可视化单帧数据"""
    fig, ax = plt.subplots(figsize=(12, 10))
    
    # 绘制地图背景（从单独的地图数据文件加载）
    if map_data is not None:
        width = map_data['width']
        height = map_data['height']
        resolution = map_data['resolution']
        origin = map_data['origin']
        map_array = np.array(map_data['data'])
        
        # 计算地图范围
        x_min = origin[0]
        y_min = origin[1]
        x_max = x_min + width * resolution
        y_max = y_min + height * resolution
        
        # 显示地图：-1为障碍物（黑色），正数为可行驶区域（白色）
        # 处理地图数据：将-1设为0（黑色），正数设为1（白色）
        processed_map = np.where(map_array == -1, 0, 1)
        
        ax.imshow(processed_map, extent=[x_min, x_max, y_min, y_max], 
                 cmap='gray', vmin=0, vmax=1, alpha=0.9, origin='lower')
        
        # 设置坐标轴范围
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
    
    # # 绘制全局路径
    # global_plan = data.get('global_plan', {})
    # if global_plan.get('x') and global_plan.get('y'):
    #     ax.plot(global_plan['x'], global_plan['y'], 'k-', linewidth=1, label='Global Plan')
    
    # 绘制规划轨迹
    planned_traj = data.get('planned_trajectory', {})
    if planned_traj.get('x') and planned_traj.get('y'):
        ax.plot(planned_traj['x'], planned_traj['y'], 'g-', linewidth=2, label='Planned Trajectory')
    
    # 地图背景已在函数开头绘制，这里移除重复代码
    
    # 绘制自车
    ego = data.get('ego_vehicle', {})
    vehicle_model = data.get('vehicle_model', None)
    if ego:
        ego_x = ego.get('x', 0)
        ego_y = ego.get('y', 0)
        ego_theta = ego.get('theta', 0)
        ego_gamma = ego.get('gamma', 0)
        
        draw_vehicle(ax, ego_x, ego_y, ego_theta, ego_gamma, color='blue', vehicle_type='ego', vehicle_model=vehicle_model)
    
    # 绘制障碍物
    obstacle = data.get('obstacle', {})
    if obstacle:
        obs_x = obstacle.get('x', 0)
        obs_y = obstacle.get('y', 0)
        obs_theta = obstacle.get('theta', 0)
        
        draw_vehicle(ax, obs_x, obs_y, obs_theta, 0, color='#FFCCE5', vehicle_type='obstacle', vehicle_model=vehicle_model, obstacle_data=obstacle)
    
    # 设置图形属性
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_title(f'CILQR Planning - Frame {frame_num}', fontsize=14)
    ax.legend()
    ax.set_aspect('equal')
    
    # 保存或显示
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Frame {frame_num} saved to: {save_path}")
    else:
        plt.show()
    
    plt.close()

def process_all_frames(data_dir='../data'):
    """处理所有数据帧"""
    # 获取脚本所在目录的绝对路径
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # 构建数据目录的绝对路径
    if not os.path.isabs(data_dir):
        data_dir = os.path.join(script_dir, data_dir)
    
    # 加载地图数据（只加载一次）
    map_data = load_map_data()
    if map_data is None:
        print("Warning: No map data loaded. Visualization will not include map background.")
    else:
        print(f"Map data loaded: {map_data['width']}x{map_data['height']} pixels")
    
    # 查找所有数据文件
    data_files = glob.glob(os.path.join(data_dir, 'cilqr_data_*.json'))
    data_files.sort(key=lambda x: int(x.split('_')[-1].split('.')[0]))
    
    print(f"Found {len(data_files)} data files")
    
    # 创建输出目录
    output_dir = '../images/cilqr_visualizations'
    if not os.path.isabs(output_dir):
        output_dir = os.path.join(script_dir, output_dir)
    os.makedirs(output_dir, exist_ok=True)
    
    # 清空之前的图片文件
    existing_images = glob.glob(os.path.join(output_dir, 'cilqr_frame_*.png'))
    if existing_images:
        print(f"Clearing {len(existing_images)} existing images...")
        for img_file in existing_images:
            try:
                os.remove(img_file)
            except Exception as e:
                print(f"Warning: Could not remove {img_file}: {e}")
        print("Previous images cleared.")
    else:
        print("No previous images to clear.")
    
    # 处理每个数据文件
    for i, data_file in enumerate(data_files):
        data = load_data(data_file)
        if data is not None:
            frame_num = int(os.path.basename(data_file).split('_')[-1].split('.')[0])
            output_path = os.path.join(output_dir, f'cilqr_frame_{frame_num:04d}.png')
            visualize_frame(data, frame_num, map_data, output_path)
        
        # 每处理10帧显示一次进度
        if (i + 1) % 10 == 0:
            print(f"Processed {i + 1}/{len(data_files)} frames")
    
    print(f"All frames processed! Images saved in '{output_dir}' directory")

def create_animation(image_dir='../images/cilqr_visualizations', output_name='../images/cilqr_animation.gif'):
    """创建动画GIF（需要安装pillow）"""
    try:
        from PIL import Image
        import glob
        
        # 获取脚本所在目录的绝对路径
        script_dir = os.path.dirname(os.path.abspath(__file__))
        # 构建图像目录和输出文件的绝对路径
        if not os.path.isabs(image_dir):
            image_dir = os.path.join(script_dir, image_dir)
        if not os.path.isabs(output_name):
            output_name = os.path.join(script_dir, output_name)
        
        # 获取所有图像文件
        image_files = glob.glob(os.path.join(image_dir, 'cilqr_frame_*.png'))
        image_files.sort()
        
        if not image_files:
            print("No image files found for animation")
            return
        
        # 创建GIF动画
        images = []
        for img_file in image_files:
            img = Image.open(img_file)
            images.append(img)
        
        # 保存为GIF
        images[0].save(output_name, save_all=True, append_images=images[1:], 
                      duration=100, loop=0)
        print(f"Animation saved as: {output_name}")
        
    except ImportError:
        print("PIL/Pillow not installed. Cannot create animation.")
        print("Install with: pip install pillow")

if __name__ == '__main__':
    print("CILQR数据可视化工具")
    print("==================")
    print("功能：批量处理数据文件，生成可视化图像和动画")
    print("地图显示：可行驶路径(白色) | 不可行驶路径(黑色)")
    print()
    
    # 处理所有帧
    process_all_frames()
    
    # 创建动画
    create_animation()
    
    print("\n可视化完成！")
    print("图像保存位置: ../images/cilqr_visualizations/")
    print("动画保存位置: ../images/cilqr_animation.gif")