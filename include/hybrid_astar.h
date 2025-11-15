#pragma once

#include "utils.h"
#include <vector>
#include <Eigen/Eigen>

// 混合A*参数
struct HybridAStarParams {
    // 栅格与朝向离散
    double grid_resolution = 0.1;      // 栅格分辨率（米）
    double heading_resolution = 10.0;  // 朝向离散（度）

    // 车辆运动与代价
    double turning_radius = 4.0;       // 最小转弯半径（米）
    double move_step = 1;            // 前进步长（米）
    double move_step_backwards = 0.3;  // 倒退步长（米）
    bool   allow_reverse = false;       // 允许倒车
    int    num_gamma_angles = 5;    // 铰接角离散数
    double steering_penalty = 0.2;     // 铰接角代价系数
    double direction_change_penalty = 1.5; // 换向惩罚
    double backwards_penalty = 1.2;    // 倒车惩罚
    double heuristic_weight = 1.0;     // 启发式权重

    // 终止条件与裁剪
    int    max_iterations = 60000;     // 最大迭代
    int    num_nodes_to_keep = 30000;  // 开放表保留上限（裁剪）

    // 膨胀与目标容忍
    double inflation_radius = 2;     // 膨胀半径（米）
    double goal_tolerance_xy = 0.8;    // 目标xy容差（米）
    double goal_tolerance_heading = 15.0; // 目标朝向容差（度）

    // 清距偏好（越靠近障碍物代价越高）
    double clearance_weight = 12;     // 清距代价权重（越大越偏好远离障碍）
    double desired_clearance = 3;    // 期望与障碍的最小距离（米），小于该值增加代价
    double max_clearance = 12.0;       // 清距归一化的上限（米），避免过大距离影响代价
    double min_rs_clearance = 1.5;     // RS解析路径的最小允许清距（米），低于则拒绝解析连接
};

// Dijkstra预计算距离图
struct DijkstraDistanceMap {
    std::vector<std::vector<double>> distances; // 距离图：distances[y][x]
    int width;                                   // 栅格宽度
    int height;                                  // 栅格高度
    double resolution;                           // 栅格分辨率
    double origin_x;                            // 栅格原点x坐标
    double origin_y;                            // 栅格原点y坐标
    bool is_valid;                              // 是否有效
    
    DijkstraDistanceMap() : width(0), height(0), resolution(0.1), origin_x(0), origin_y(0), is_valid(false) {}
    
    // 获取世界坐标对应的距离
    double getDistance(double world_x, double world_y) const;
};

// Dijkstra预计算函数：从目标点计算到所有栅格点的最短距离
DijkstraDistanceMap computeDijkstraDistances(const OccupancyGrid& grid, 
                                           double goal_x, double goal_y);

// 混合A*规划接口：输出包含 heading 的点集
bool hybrid_astar_plan(const MapData& map,
                       const Eigen::Vector3d& start,
                       const Eigen::Vector3d& goal,
                       std::vector<Point>& out_points,
                       const HybridAStarParams& params);