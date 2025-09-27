#include "hybrid_astar.h"
#include <iostream>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

// Hybrid A* node structure
struct HybridAStarNode {
    double x;           // x coordinate (meters)
    double y;           // y coordinate (meters)
    double theta;       // heading angle (radians)
    double g;           // actual cost
    double f;           // total cost (g + h)
    int grid_x;         // grid x coordinate
    int grid_y;         // grid y coordinate
    int grid_theta;     // grid heading index
    bool is_forward;    // whether moving forward
    std::shared_ptr<HybridAStarNode> parent; // parent node pointer
};

// 节点比较函数（用于优先队列）
struct CompareNodes {
    bool operator()(const std::shared_ptr<HybridAStarNode>& a, const std::shared_ptr<HybridAStarNode>& b) const {
        return a->f > b->f; // 小的f值优先
    }
};

// 节点哈希函数（用于栅格索引）
struct NodeKey {
    int x;
    int y;
    int theta;

    bool operator==(const NodeKey& other) const {
        return x == other.x && y == other.y && theta == other.theta;
    }
};

// 自定义哈希函数
namespace std {
    template<>
    struct hash<NodeKey> {
        size_t operator()(const NodeKey& k) const {
            return ((hash<int>()(k.x) ^ (hash<int>()(k.y) << 1)) >> 1) ^ (hash<int>()(k.theta) << 1);
        }
    };
}

// 角度标准化到[-π, π]
static double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// 角度差的绝对值（弧度）
static double angleDiff(double a, double b) {
    return std::abs(normalizeAngle(a - b));
}

// 度转弧度
static double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

// 弧度转度
static double rad2deg(double rad) {
    return rad * 180.0 / M_PI;
}

// 启发式函数：非完整约束下的距离估计
static double heuristic(double x, double y, double theta, double goal_x, double goal_y, double goal_theta, double turning_radius) {
    // 欧几里得距离
    double euclidean_dist = std::hypot(goal_x - x, goal_y - y);
    
    // Reed-Shepp距离估计（简化版）
    double angle_diff = angleDiff(theta, std::atan2(goal_y - y, goal_x - x));
    double rs_dist = euclidean_dist + turning_radius * angle_diff;
    
    // 目标朝向差异惩罚
    double heading_diff = angleDiff(theta, goal_theta);
    double heading_penalty = turning_radius * 0.5 * heading_diff;
    
    return rs_dist + heading_penalty;
}

// 生成运动原语
static std::vector<std::shared_ptr<HybridAStarNode>> getNeighbors(
    const std::shared_ptr<HybridAStarNode>& current,
    const HybridAStarParams& params,
    const OccupancyGrid& grid,
    double goal_x, double goal_y, double goal_theta) {
    
    std::vector<std::shared_ptr<HybridAStarNode>> neighbors;
    
    // 转向角度集合
    std::vector<double> steering_angles;
    double max_steer = std::atan(params.move_step / params.turning_radius);
    double steer_step = 2.0 * max_steer / (params.num_steering_angles - 1);
    
    for (int i = 0; i < params.num_steering_angles; ++i) {
        steering_angles.push_back(-max_steer + i * steer_step);
    }
    
    // 前进方向的运动原语
    for (double steer : steering_angles) {
        double next_theta = normalizeAngle(current->theta + steer);
        double dx = params.move_step * std::cos(next_theta);
        double dy = params.move_step * std::sin(next_theta);
        double next_x = current->x + dx;
        double next_y = current->y + dy;
        
        // 调试信息
        // if (neighbors.empty()) {
            // std::cout << "Trying neighbor: (" << next_x << ", " << next_y << ", " << next_theta << ")" << std::endl;
            // std::cout << "Current: (" << current->x << ", " << current->y << ", " << current->theta << ")" << std::endl;
            // std::cout << "Move step: " << params.move_step << ", steer: " << steer << std::endl;
            
            // bool collision_free = is_collision_free_segment(grid, Eigen::Vector2d(current->x, current->y), Eigen::Vector2d(next_x, next_y));
            // std::cout << "Collision free: " << (collision_free ? "yes" : "no") << std::endl;
        // }
        
        // 检查碰撞
        if (!is_collision_free_segment(grid, Eigen::Vector2d(current->x, current->y), Eigen::Vector2d(next_x, next_y))) {
            continue;
        }
        
        // 计算栅格索引
        int grid_x = static_cast<int>(std::floor((next_x - grid.origin_x) / grid.resolution));
        int grid_y = static_cast<int>(std::floor((next_y - grid.origin_y) / grid.resolution));
        int grid_theta = static_cast<int>(std::floor(rad2deg(next_theta + M_PI) / params.heading_resolution)) % 
                         static_cast<int>(360.0 / params.heading_resolution);
        
        // 计算代价
        double steering_cost = params.steering_penalty * std::abs(steer);
        double direction_change_cost = current->parent && current->is_forward != true ? params.direction_change_penalty : 0.0;
        // 清距代价：若距离障碍小于期望清距，则按比例惩罚（平方增强）
        double c_next = nearest_obstacle_distance_world(grid, Eigen::Vector2d(next_x, next_y));
        double c_clamp = std::min(c_next, params.max_clearance);
        double lack = std::max(0.0, params.desired_clearance - c_clamp);
        double clearance_cost = params.clearance_weight * std::pow(lack / std::max(1e-3, params.desired_clearance), 2.0);
        double g = current->g + params.move_step + steering_cost + direction_change_cost + clearance_cost;
        
        // 计算启发式
        double h = params.heuristic_weight * heuristic(next_x, next_y, next_theta, goal_x, goal_y, next_theta, params.turning_radius);
        
        // 创建新节点
        auto node = std::make_shared<HybridAStarNode>();
        node->x = next_x;
        node->y = next_y;
        node->theta = next_theta;
        node->g = g;
        node->f = g + h;
        node->grid_x = grid_x;
        node->grid_y = grid_y;
        node->grid_theta = grid_theta;
        node->is_forward = true;
        node->parent = current;
        
        neighbors.push_back(node);
    }
    
    // 如果允许倒车，添加后退的运动原语
    if (params.allow_reverse) {
        for (double steer : steering_angles) {
            double next_theta = normalizeAngle(current->theta - steer); // 注意倒车时转向相反
            double dx = -params.move_step_backwards * std::cos(next_theta);
            double dy = -params.move_step_backwards * std::sin(next_theta);
            double next_x = current->x + dx;
            double next_y = current->y + dy;
            
            // 检查碰撞
            if (!is_collision_free_segment(grid, Eigen::Vector2d(current->x, current->y), Eigen::Vector2d(next_x, next_y))) {
                continue;
            }
            
            // 计算栅格索引
            int grid_x = static_cast<int>(std::floor((next_x - grid.origin_x) / grid.resolution));
            int grid_y = static_cast<int>(std::floor((next_y - grid.origin_y) / grid.resolution));
            int grid_theta = static_cast<int>(std::floor(rad2deg(next_theta + M_PI) / params.heading_resolution)) % 
                             static_cast<int>(360.0 / params.heading_resolution);
            
            // 计算代价（倒车有额外惩罚）
            double steering_cost = params.steering_penalty * std::abs(steer);
            double direction_change_cost = current->parent && current->is_forward != false ? params.direction_change_penalty : 0.0;
            // 清距代价
            double c_next = nearest_obstacle_distance_world(grid, Eigen::Vector2d(next_x, next_y));
            double c_clamp = std::min(c_next, params.max_clearance);
            double lack = std::max(0.0, params.desired_clearance - c_clamp);
            double clearance_cost = params.clearance_weight * std::pow(lack / std::max(1e-3, params.desired_clearance), 2.0);
            double g = current->g + params.move_step_backwards * params.backwards_penalty + steering_cost + direction_change_cost + clearance_cost;
            
            // 计算启发式
            double h = params.heuristic_weight * heuristic(next_x, next_y, next_theta, goal_x, goal_y, next_theta, params.turning_radius);
            
            // 创建新节点
            auto node = std::make_shared<HybridAStarNode>();
            node->x = next_x;
            node->y = next_y;
            node->theta = next_theta;
            node->g = g;
            node->f = g + h;
            node->grid_x = grid_x;
            node->grid_y = grid_y;
            node->grid_theta = grid_theta;
            node->is_forward = false;
            node->parent = current;
            
            neighbors.push_back(node);
        }
    }
    
    return neighbors;
}

// 检查是否达到目标
static bool isGoalReached(const HybridAStarNode& node, double goal_x, double goal_y, double goal_theta, const HybridAStarParams& params) {
    double dist = std::hypot(node.x - goal_x, node.y - goal_y);
    double angle_diff = angleDiff(node.theta, goal_theta);
    return dist <= params.goal_tolerance_xy && angle_diff <= deg2rad(params.goal_tolerance_heading);
}

// 路径平滑和后处理
static std::vector<Eigen::Vector2d> smoothPath(const std::vector<Eigen::Vector2d>& path, const OccupancyGrid& grid) {
    // 先进行路径简化（去除冗余点）
    std::vector<Eigen::Vector2d> simplified;
    if (path.size() <= 2) return path;
    
    simplified.push_back(path.front());
    for (size_t i = 1; i < path.size() - 1; ++i) {
        if (!is_collision_free_segment(grid, simplified.back(), path[i+1])) {
            simplified.push_back(path[i]);
        }
    }
    simplified.push_back(path.back());
    
    return simplified;
}

// === RS Analytic Connection (simplified C-S-C) ===
static bool tryRSConnect(const HybridAStarNode& current,
                         double goal_x, double goal_y, double goal_theta,
                         const HybridAStarParams& params,
                         const OccupancyGrid& grid,
                         std::vector<Eigen::Vector2d>& out_path) {
    const double R = std::max(1e-3, params.turning_radius);
    const double step = std::max(0.05, params.move_step);
    const double dtheta_step = step / R; // curvature integration
    const double tol_xy = params.goal_tolerance_xy;
    const double tol_th = deg2rad(params.goal_tolerance_heading);

    // Start state
    double x = current.x;
    double y = current.y;
    double th = current.theta;

    auto angle_diff = [](double a, double b){
        double d = a - b; while (d > M_PI) d -= 2*M_PI; while (d < -M_PI) d += 2*M_PI; return d;
    };

    // Bearing to goal
    double bx = goal_x - x;
    double by = goal_y - y;
    double bearing = std::atan2(by, bx);

    // 1) First arc: turn towards bearing
    double d1 = angle_diff(bearing, th);
    int s1 = (d1 >= 0.0) ? +1 : -1; // left/right
    double rem1 = std::abs(d1);
    std::vector<Eigen::Vector2d> path;
    path.emplace_back(x, y);
    while (rem1 > 1e-6) {
        double dth = std::min(rem1, dtheta_step);
        th += s1 * dth;
        double nx = x + step * std::cos(th);
        double ny = y + step * std::sin(th);
        if (!is_collision_free_segment(grid, Eigen::Vector2d(x,y), Eigen::Vector2d(nx,ny))) {
            return false;
        }
        x = nx; y = ny;
        path.emplace_back(x, y);
        rem1 -= dth;
    }

    // 2) Straight segment towards goal
    double dist = std::hypot(goal_x - x, goal_y - y);
    int max_straight_steps = (int)std::ceil(dist / step);
    for (int k = 0; k < max_straight_steps; ++k) {
        if (std::hypot(goal_x - x, goal_y - y) <= std::max(step, tol_xy)) break;
        double nx = x + step * std::cos(th);
        double ny = y + step * std::sin(th);
        if (!is_collision_free_segment(grid, Eigen::Vector2d(x,y), Eigen::Vector2d(nx,ny))) {
            return false;
        }
        x = nx; y = ny;
        path.emplace_back(x, y);
    }

    // 3) Final arc: align to goal heading while approaching goal
    double d2 = angle_diff(goal_theta, th);
    int s2 = (d2 >= 0.0) ? +1 : -1;
    double rem2 = std::abs(d2);
    int safety_steps = 0;
    while (rem2 > tol_th && safety_steps < 400) {
        double dth = std::min(rem2, dtheta_step);
        th += s2 * dth;
        double nx = x + step * std::cos(th);
        double ny = y + step * std::sin(th);
        if (!is_collision_free_segment(grid, Eigen::Vector2d(x,y), Eigen::Vector2d(nx,ny))) {
            return false;
        }
        x = nx; y = ny;
        path.emplace_back(x, y);
        rem2 -= dth;
        ++safety_steps;
        // small pull towards goal
        double dir_goal = std::atan2(goal_y - y, goal_x - x);
        double align = angle_diff(dir_goal, th);
        if (std::hypot(goal_x - x, goal_y - y) <= tol_xy && std::abs(align) <= tol_th) break;
    }

    // Final check
    if (std::hypot(goal_x - x, goal_y - y) <= tol_xy && std::abs(angle_diff(goal_theta, th)) <= tol_th) {
        out_path = path;
        return true;
    }
    return false;
}

// 混合A*主算法
bool hybrid_astar_plan(const MapData& map,
                       const Eigen::Vector3d& start,
                       const Eigen::Vector3d& goal,
                       std::vector<Point>& out_points,
                       const HybridAStarParams& params) {
    std::cout << "Hybrid A* planning started with params: grid_res=" << params.grid_resolution 
              << ", heading_res=" << params.heading_resolution 
              << ", turning_radius=" << params.turning_radius << std::endl;
    
    // 1) 构建占用栅格
    OccupancyGrid grid = make_occupancy_grid(map, 0.1, params.inflation_radius);
    
    // 2) 初始化起点和终点
    double start_x = start[0];
    double start_y = start[1];
    double start_theta = start[2];
    
    double goal_x = goal[0];
    double goal_y = goal[1];
    double goal_theta = goal[2];
    
    // 检查起点和终点是否在地图范围内且无碰撞
    int start_grid_x = static_cast<int>(std::floor((start_x - grid.origin_x) / grid.resolution));
    int start_grid_y = static_cast<int>(std::floor((start_y - grid.origin_y) / grid.resolution));
    int goal_grid_x = static_cast<int>(std::floor((goal_x - grid.origin_x) / grid.resolution));
    int goal_grid_y = static_cast<int>(std::floor((goal_y - grid.origin_y) / grid.resolution));
    
    // 检查起点和终点是否在地图范围内
    if (start_grid_x < 0 || start_grid_x >= grid.width || start_grid_y < 0 || start_grid_y >= grid.height ||
        goal_grid_x < 0 || goal_grid_x >= grid.width || goal_grid_y < 0 || goal_grid_y >= grid.height) {
        std::cerr << "Start or goal position is outside the map!" << std::endl;
        std::cerr << "Start grid: (" << start_grid_x << ", " << start_grid_y << "), Goal grid: (" 
                  << goal_grid_x << ", " << goal_grid_y << ")" << std::endl;
        std::cerr << "Grid dimensions: " << grid.width << " x " << grid.height << std::endl;
        std::cerr << "Grid resolution: " << params.grid_resolution << std::endl;
        return false;
    }
    
    // 检查起点和终点是否无碰撞
    Eigen::Vector2d start_pos(start_x, start_y);
    Eigen::Vector2d goal_pos(goal_x, goal_y);
    double start_dist = nearest_obstacle_distance_world(grid, start_pos);
    double goal_dist = nearest_obstacle_distance_world(grid, goal_pos);
    
    std::cout << "Start distance to obstacle: " << start_dist << std::endl;
    std::cout << "Goal distance to obstacle: " << goal_dist << std::endl;
    std::cout << "Inflation radius: " << params.inflation_radius << std::endl;
    
    if (start_dist <= params.inflation_radius || goal_dist <= params.inflation_radius) {
        std::cerr << "Start or goal position is in collision!" << std::endl;
        return false;
    }
    
    // 3) 初始化起始节点
    auto start_node = std::make_shared<HybridAStarNode>();
    start_node->x = start_x;
    start_node->y = start_y;
    start_node->theta = start_theta;
    start_node->g = 0.0;
    start_node->f = heuristic(start_x, start_y, start_theta, goal_x, goal_y, goal_theta, params.turning_radius);
    start_node->grid_x = start_grid_x;
    start_node->grid_y = start_grid_y;
    start_node->grid_theta = static_cast<int>(std::floor(rad2deg(start_theta + M_PI) / params.heading_resolution)) % 
                            static_cast<int>(360.0 / params.heading_resolution);
    start_node->is_forward = true;
    start_node->parent = nullptr;
    
    // 4) 初始化开放列表和关闭列表
    std::priority_queue<std::shared_ptr<HybridAStarNode>, std::vector<std::shared_ptr<HybridAStarNode>>, CompareNodes> open_list;
    std::unordered_map<NodeKey, std::shared_ptr<HybridAStarNode>> open_set;
    std::unordered_set<NodeKey> closed_set;
    
    open_list.push(start_node);
    open_set[{start_node->grid_x, start_node->grid_y, start_node->grid_theta}] = start_node;
    
    // 5) 主循环
    int iterations = 0;
    std::shared_ptr<HybridAStarNode> goal_node = nullptr;
    bool analytic_success = false;
    std::vector<Eigen::Vector2d> analytic_path;
    std::shared_ptr<HybridAStarNode> analytic_start_node = nullptr;

    while (!open_list.empty() && iterations < params.max_iterations) {
        std::cout << "Hybrid_a_star Iteration: " << iterations << std::endl;
        // 获取f值最小的节点
        auto current = open_list.top();
        open_list.pop();
        // 从open_set中移除
        NodeKey current_key = {current->grid_x, current->grid_y, current->grid_theta};
        open_set.erase(current_key);
        // 调试：当前节点信息和到目标的距离
        if (iterations % 100 == 0) {
            double d_goal_dbg = std::hypot(current->x - goal_x, current->y - goal_y);
            std::cout << "Current node: x=" << current->x << ", y=" << current->y << ", th=" << current->theta
                      << ", d_goal=" << d_goal_dbg << std::endl;
        }
        // 尝试解析RS连接（仅在接近终点时）
        double d_goal = std::hypot(current->x - goal_x, current->y - goal_y);
        if (d_goal < 10.0 * params.turning_radius) {
            std::vector<Eigen::Vector2d> rs_path;
            if (tryRSConnect(*current, goal_x, goal_y, goal_theta, params, grid, rs_path)) {
                // 解析路径的二次碰撞校验（保守）：逐段采样 + 走廊宽度自适应
                if (!is_collision_free_polyline(grid, rs_path)) {
                    std::cout << "Analytic RS path rejected due to collision along segments, size=" << rs_path.size() << std::endl;
                } else {
                    analytic_success = true;
                    analytic_path = rs_path;
                    analytic_start_node = current; // 记录解析起点以便回溯前缀
                    // 清距检查：若解析路径任一点清距低于阈值，则拒绝
                    bool rs_clearance_ok = true;
                    for (const auto& p : rs_path) {
                        double c = nearest_obstacle_distance_world(grid, p);
                        if (c < params.min_rs_clearance) { rs_clearance_ok = false; break; }
                    }
                    if (!rs_clearance_ok) {
                        analytic_success = false;
                        std::cout << "Analytic RS path rejected due to insufficient clearance (<" << params.min_rs_clearance << ")" << std::endl;
                    } else {
                        std::cout << "Analytic RS connect succeeded at iter " << iterations << ", path points: " << rs_path.size() << std::endl;
                        break;
                    }
                }
            }
        }

        // 检查是否达到目标
        if (isGoalReached(*current, goal_x, goal_y, goal_theta, params)) {
            goal_node = current;
            std::cout << "Goal reached after " << iterations << " iterations!" << std::endl;
            break;
        }

        // 添加到关闭列表
        closed_set.insert(current_key);

        // 生成邻居节点
        auto neighbors = getNeighbors(current, params, grid, goal_x, goal_y, goal_theta);
        // 调试：邻居统计
        if (iterations % 100 == 0) {
            int attempted = params.num_steering_angles * (params.allow_reverse ? 2 : 1);
            std::cout << "Neighbors attempted=" << attempted << ", accepted=" << neighbors.size() << std::endl;
        }
        // 处理每个邻居
        for (const auto& neighbor : neighbors) {
            NodeKey neighbor_key = {neighbor->grid_x, neighbor->grid_y, neighbor->grid_theta};
            
            // 如果在关闭列表中，跳过
            if (closed_set.find(neighbor_key) != closed_set.end()) {
                continue;
            }
            
            // 如果不在开放列表中，或者找到了更好的路径
            auto it = open_set.find(neighbor_key);
            if (it == open_set.end() || neighbor->g < it->second->g) {
                open_set[neighbor_key] = neighbor;
                open_list.push(neighbor);
            }
        }
        
        iterations++;
        if (iterations % 1000 == 0) {
            std::cout << "Hybrid A* iteration " << iterations << ", open list size: " << open_list.size() << std::endl;
        }
        
        // 如果开放列表太大，保留最好的节点
            if (open_list.size() > static_cast<size_t>(params.num_nodes_to_keep)) {
                std::vector<std::shared_ptr<HybridAStarNode>> temp_nodes;
                while (!open_list.empty()) {
                    temp_nodes.push_back(open_list.top());
                    open_list.pop();
                }
                
                // 只保留最好的节点
                size_t keep = std::min(static_cast<size_t>(params.num_nodes_to_keep), temp_nodes.size());
                for (size_t i = 0; i < keep; ++i) {
                    open_list.push(temp_nodes[i]);
                }
            
            // 更新open_set
            open_set.clear();
            for (size_t i = 0; i < keep; ++i) {
                NodeKey key = {temp_nodes[i]->grid_x, temp_nodes[i]->grid_y, temp_nodes[i]->grid_theta};
                open_set[key] = temp_nodes[i];
            }
        }
    }
    
    // 6) 检查是否找到路径
    if (!goal_node && !analytic_success) {
        std::cerr << "No path found after " << iterations << " iterations!" << std::endl;
        return false;
    }

    // 7) 回溯或使用解析路径
    std::vector<Eigen::Vector2d> path;
    if (analytic_success) {
        // 先回溯起点->解析起点的前缀路径
        std::vector<Eigen::Vector2d> prefix;
        auto node = analytic_start_node;
        while (node) {
            prefix.emplace_back(node->x, node->y);
            node = node->parent;
        }
        std::reverse(prefix.begin(), prefix.end());

        // 拼接解析路径，避免重复连接点
        path = prefix;
        if (!analytic_path.empty()) {
            if (!path.empty() && (std::hypot(path.back().x() - analytic_path.front().x(), path.back().y() - analytic_path.front().y()) <= 1e-3)) {
                // 前缀末尾与解析路径起点重复，跳过第一个解析点
                path.insert(path.end(), analytic_path.begin() + 1, analytic_path.end());
            } else {
                path.insert(path.end(), analytic_path.begin(), analytic_path.end());
            }
        }
        // 再次保守检查
        if (!is_collision_free_polyline(grid, path)) {
            std::cerr << "Warning: analytic+prefix path segments not collision-free after verification, applying simplify and filtering." << std::endl;
        }
        std::cout << "Raw analytic path (with prefix) found with " << path.size() << " points" << std::endl;
    } else {
        std::shared_ptr<HybridAStarNode> node = goal_node;
        while (node) {
            path.push_back(Eigen::Vector2d(node->x, node->y));
            node = node->parent;
        }
        std::reverse(path.begin(), path.end());
        std::cout << "Raw path found with " << path.size() << " points" << std::endl;
    }

    // 8) 路径平滑和后处理
    // std::vector<Eigen::Vector2d> smoothed_path = smoothPath(path, grid);
    // std::cout << "Path simplified to " << smoothed_path.size() << " points" << std::endl;

    // 直接使用简化后的路径，不进行QP优化
    std::vector<Eigen::Vector2d> fitted_path = path;

    // 10) 输出路径点（补 heading）
    out_points.clear();
    out_points.reserve(fitted_path.size());

    for (size_t i = 0; i < fitted_path.size(); ++i) {
        double heading = 0.0;
        if (i + 1 < fitted_path.size()) {
            Eigen::Vector2d d = fitted_path[i+1] - fitted_path[i];
            heading = std::atan2(d.y(), d.x());
        } else if (i > 0) {
            Eigen::Vector2d d = fitted_path[i] - fitted_path[i-1];
            heading = std::atan2(d.y(), d.x());
        }
        out_points.emplace_back(fitted_path[i].x(), fitted_path[i].y(), heading);
    }

    // 追加目标点确保接口一致
    out_points.emplace_back(goal[0], goal[1], goal[2]);

    std::cout << "Hybrid A* planning succeeded with " << out_points.size() << " points" << std::endl;
    return true;
}