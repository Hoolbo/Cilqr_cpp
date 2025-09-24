// #include <mat.h>
#include <vector>
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

void my_plot(const std::vector<std::vector<double>>& global_plan_log,
    const std::vector<std::vector<double>>& ego_log,
    const Trajectory& obs_traj,
    const Solution& solution,
    const MapData* map_data = nullptr);

void dynamic_plot(const std::vector<std::vector<double>>& global_plan_log,
    const std::vector<std::vector<double>>& ego_log,
    const Trajectory& obs_traj,
    const Solution& solution,
    const MapData* map_data,
    const GlobalPlan& global_plan,
    const SystemModel& vehicle_model);

// Debug-only: draw bitmap map alone to verify matplotlib-cpp rendering; if output_path is non-empty, save to that file
void draw_bitmap_debug(const MapData& map_data, const std::string& output_path = "");

// 保存地图数据到单独的文件
void save_map_data(const MapData* map_data);

// 地图和路径相关函数
std::vector<std::vector<double>> load_map();
MapData load_bitmap_map(const std::string& file_path);
SemanticMapData load_semantic_map(const std::string& file_path);
void fill_global_path_points(std::vector<std::vector<double>>& global_plan_log);
void set_global_path(GlobalPlan& global_plan, const std::vector<std::vector<double>>& global_plan_log);

// 初始化函数
void init_params(Params& params);
void init_vehicle_model(VehicleModel& vehicle_model);
void init_obstacle_trajectory(Trajectory& obs_traj);



