#pragma once
// #include <mat.h>
#include <vector>
#include <cstdint>
// #include "mclmcrrt.h"
#include "matplotlibcpp.h"
#include "Eigen/Eigen"
#include "ilqr.h"
#include <memory>
#include <string>
#include <sstream>
#include <fstream>

// 地图数据结构
struct MapData {
    int width;
    int height;
    double resolution;
    std::vector<double> origin;
    double max_elevation;
    std::vector<std::vector<double>> data;
};

// 语义地图点结构
struct SemanticMapPoint {
    double x;
    double y;
    int type;
};

// 语义地图数据结构
struct SemanticMapData {
    std::vector<SemanticMapPoint> points;
};

// Point和GlobalPlan结构已在ilqr.h中定义

// 参数结构
struct Params {
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::MatrixXd Qf;
    double dt;
    int N;
    int max_iter;
    double tol;
};

// 车辆模型结构
struct VehicleModel {
    double L1;  // 前车厢长度
    double L2;  // 后车厢长度
    double width; // 车宽
};

// 铰接车曲率与转弯半径上限推导结果
struct ArticulatedLimits {
    double kappa_max; // 最大曲率
    double R_min;     // 最小转弯半径
};

void my_plot(const std::vector<std::vector<double>>& global_plan_log,
    const std::vector<std::vector<double>>& ego_log,
    const Trajectory& obs_traj,
    const Solution& solution,
    const MapData* map_data = nullptr);

void dynamic_plot(const std::vector<std::vector<double>>& global_plan_log,
    const std::vector<std::vector<double>>& ego_log,
    const std::vector<Trajectory>& obs_trajectories,
    const Solution& solution,
    const MapData* map_data,
    const GlobalPlan& global_plan,
    const SystemModel& vehicle_model,
    const Arg& arg);

// Debug-only: draw bitmap map alone to verify matplotlib-cpp rendering; if output_path is non-empty, save to that file
void draw_bitmap_debug(const MapData& map_data, const std::string& output_path = "");

// 保存地图数据到单独的文件
void save_map_data(const MapData* map_data);
std::string resolve_resource_path(const std::string& relative_path);
// 新增：导出 m_map_info 到 outputs/data/m_map_info.json
void save_m_map_info(const std::vector<std::vector<double>>& m_map_info);
void ensure_directory_exists(const std::string& path);
// 地图和路径相关函数
std::vector<std::vector<double>> load_map(double startx=0, double starty=0, double theta=0);
MapData load_bitmap_map(const std::string& file_path);
SemanticMapData load_semantic_map(const std::string& file_path);
void fill_global_path_points(std::vector<std::vector<double>>& global_plan_log);
void set_global_path(GlobalPlan& global_plan, const std::vector<std::vector<double>>& global_plan_log);

// 规划包装器：默认启用RRT*，失败时快速回退到 load_map 生成的路径
bool plan_global_path(const MapData& bitmap_map,
                      const Eigen::Vector3d& start,
                      const Eigen::Vector3d& goal,
                      GlobalPlan& out_plan,
                      bool enable_rrt = true);

// 基于 SystemModel 推导保守的最大曲率与最小转弯半径（给定机械关节角上限）
ArticulatedLimits compute_articulated_limits(const SystemModel& model, double gamma_max_mech);

// 初始化函数
void init_params(Params& params);
void init_vehicle_model(VehicleModel& vehicle_model);
void init_obstacle_trajectory(Trajectory& obs_traj);

// 障碍物预测函数
Trajectory predict_obstacle_trajectory(const State& initial_state, double dt, int N);





// 占用栅格结构（用于快速碰撞检测）
struct OccupancyGrid {
    int width;
    int height;
    double resolution;
    double origin_x; // 地图原点x（米）
    double origin_y; // 地图原点y（米）
    std::vector<uint8_t> cells; // 0=free, 1=occupied
};

// 从 MapData 生成占用栅格，并按给定半径进行膨胀（米）
OccupancyGrid make_occupancy_grid(const MapData& map, double elevation_threshold_ratio = 0.1, double inflate_radius_m = 0.5);

// 线段碰撞检测：检查从点a到点b的直线是否无碰撞
bool is_collision_free_segment(const OccupancyGrid& grid, const Eigen::Vector2d& a, const Eigen::Vector2d& b);
// 折线整体碰撞检测（逐段采样）
bool is_collision_free_polyline(const OccupancyGrid& grid, const std::vector<Eigen::Vector2d>& pts);
// 查询世界坐标点到最近障碍栅格中心的距离（米）；若附近无障碍则返回一个大数
    double nearest_obstacle_distance_world(const OccupancyGrid& grid, const Eigen::Vector2d& p);

// 自动生成障碍物
std::vector<State> generate_obstacles(const GlobalPlan& global_plan, const OccupancyGrid& grid, int num_obstacles, double distance_from_path);

void fill_global_path_points(std::vector<std::vector<double>>& global_plan_log);
void set_global_path(GlobalPlan& global_plan, const std::vector<std::vector<double>>& global_plan_log);




// RRT* 参数配置
struct RRTStarParams {
    int max_iters = 20000;          // 最大迭代次数
    double step_size = 0.5;         // 每步扩展的步长
    double goal_tolerance = 1.0;    // 目标点容忍距离
    double rewire_radius_factor = 1.5; // 重布线半径因子
    double inflation_radius = 0.5;  // 障碍物膨胀半径
    double goal_sample_rate = 0.1;  // 目标点采样概率
    double corridor_sample_rate = 0.2; // 走廊采样概率
    double corridor_width = 2.0;    // 走廊宽度
    int connect_attempt_interval = 100; // 每多少次迭代触发一次 RRT-Connect 尝试
    int max_connect_steps = 200;        // 每次 RRT-Connect 的最大扩展步数
    double cost_obstacle_weight = 0.1; // 障碍物代价权重
    double max_clearance = 10.0;      // 用于归一化的最大安全距离
};

// RRT* 规划接口（返回通过 out_points 输出的路径点，包含 heading）
bool rrt_star_plan(const MapData& map,
                   const Eigen::Vector3d& start,
                   const Eigen::Vector3d& goal,
                   std::vector<Point>& out_points,
                   const RRTStarParams& params);



// 使用QP对折线路径进行二次平滑（带自适应走廊约束），返回优化后的路径点集
std::vector<Eigen::Vector2d> optimize_path_qp(const std::vector<Eigen::Vector2d>& path,
                                              const OccupancyGrid& grid,
                                              int iterations = 35,
                                              double curvature_weight = 1200.0);



double compute_max_violation(const Solution& solution, Vehicle& ego, const std::vector<Trajectory>& obs_list, const Arg& arg);



