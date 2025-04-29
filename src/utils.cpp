#include "utils.h"

double ploynomial(double x){
    return 1*pow(x,5) + 5*pow(x,4)+10*pow(x,3);
}

std::vector<std::vector<double>> load_map(){

    std::vector<std::vector<double>> map_info(3);
    //本来是用matlab导入地图，现在改成自己用正弦曲线拟合地图
    double map_resolution = 0.1;
    for(int i=0;i<3000;i++){
        map_info[0].push_back(i*map_resolution);
        map_info[1].push_back(sin(i*map_resolution*0.0628));
        map_info[2].push_back(atan2(sin(i*map_resolution*0.0628)-sin((i-1)*map_resolution*0.0628),map_resolution));
        // map_info[0].push_back(i*map_resolution);
        // map_info[1].push_back(sin(i*map_resolution*0.000628));
        // map_info[2].push_back(atan2(sin(i*map_resolution*0.000628)-sin((i-1)*map_resolution*0.000628),map_resolution));
        // map_info[0].push_back(i*map_resolution);
        // map_info[1].push_back(ploynomial(i*map_resolution*0.0628));
        // map_info[2].push_back(atan2(ploynomial(i*map_resolution*0.0628)-ploynomial((i-1)*map_resolution*0.0628),map_resolution));
    } 

    return map_info;
}

void my_plot(const std::vector<std::vector<double>>& global_plan_log,
             const std::vector<std::vector<double>>& ego_log,
             const std::vector<Trajectory>& total_obs_traj,
             const Solution& solution,
             const Arg& arg) 
{       
    namespace plt = matplotlibcpp;
    // 静态绘图对象
    static std::unique_ptr<matplotlibcpp::Plot> global_plot, ego_plot, trajectory_plot, vehicle_rect_plot;
    static std::vector<std::unique_ptr<matplotlibcpp::Plot>> obs_traj_plots; // 障碍物矩形
    static bool figure_initialized = false;
    // 车辆和障碍物参数
    double VEHICLE_LENGTH = arg.len;  // 车长（单位：米）
    double VEHICLE_WIDTH = arg.width;     // 车宽
    double OBS_LENGTH = arg.obs_length;      // 障碍物长度
    double OBS_WIDTH = arg.obs_width;         // 障碍物宽度
    // 动态视图参数
    constexpr double FOLLOW_FACTOR = 0.7;
    constexpr double BASE_MARGIN = 20.0;
    static std::pair<double, double> view_center = {0, 0};

    // 检查输入
    if (total_obs_traj.empty()) {
        std::cerr << "Warning: No obstacle trajectories to plot." << std::endl;
    }

    // 初始化图形窗口
    if (!figure_initialized) {
        figure_initialized = true;
        plt::ion();
        plt::figure();
        plt::title("CILQR PLANNING");
        plt::xlabel("X (m)");
        plt::ylabel("Y (m)");
        plt::grid(true);
        
        global_plot.reset(new matplotlibcpp::Plot(
            "global_plot",
            global_plan_log[0], 
            global_plan_log[1],
            "k-."
        ));
        plt::axis("equal");
    }
    global_plot->update(global_plan_log[0], global_plan_log[1]);

    // 动态调整视图中心
    double target_x = ego_log[0].back();
    double target_y = ego_log[1].back();
    view_center.first += FOLLOW_FACTOR * (target_x - view_center.first);
    view_center.second += FOLLOW_FACTOR * (target_y - view_center.second);
    double speed = ego_log[3].back();
    double margin = BASE_MARGIN + speed * 0;

    // 更新规划轨迹
    std::vector<std::vector<double>> trajectory(4);
    for(size_t i = 0; i < solution.ego_trj.states.size(); ++i) {
        trajectory[0].push_back(solution.ego_trj.states[i][0]);
        trajectory[1].push_back(solution.ego_trj.states[i][1]);
    }
    if (!trajectory_plot) {
        trajectory_plot.reset(new matplotlibcpp::Plot(
            "trajectory_plot",
            trajectory[0],
            trajectory[1],
            "g-"
        ));
    }
    trajectory_plot->update(trajectory[0], trajectory[1]);

    // 调整障碍物矩形绘图对象数量
    if (obs_traj_plots.size() != total_obs_traj.size()) {
        obs_traj_plots.clear();
        obs_traj_plots.resize(total_obs_traj.size());
    }

    // 绘制障碍物矩形
    for (size_t i = 0; i < total_obs_traj.size(); ++i) {
        if (total_obs_traj[i].states.empty() || total_obs_traj[i].states[0].size() < 3) {
            std::cerr << "Warning: Invalid obstacle trajectory " << i << ", skipping." << std::endl;
            continue;
        }

        // 检查障碍物是否在视图范围内
        double obs_x = total_obs_traj[i].states[0][0];
        double obs_y = total_obs_traj[i].states[0][1];
        if (abs(obs_x - view_center.first) > margin + 20 || abs(obs_y - view_center.second) > margin + 20) {
            continue; // 跳过视图外的障碍物
        }

        // 计算障碍物矩形坐标
        double obs_theta = total_obs_traj[i].states[0][2];
        std::vector<double> obs_rect_x, obs_rect_y;
        double obs_cos_theta = cos(obs_theta);
        double obs_sin_theta = sin(obs_theta);
        double obs_half_length = OBS_LENGTH / 2;
        double obs_half_width = OBS_WIDTH / 2;
        std::array<std::pair<double, double>, 4> obs_local_points = {
            std::make_pair( obs_half_length,  obs_half_width),  // 前右
            std::make_pair( obs_half_length, -obs_half_width),  // 后右
            std::make_pair(-obs_half_length, -obs_half_width),  // 后左
            std::make_pair(-obs_half_length,  obs_half_width)    // 前左
        };
        for (const auto& pt : obs_local_points) {
            double global_x = obs_x + pt.first * obs_cos_theta - pt.second * obs_sin_theta;
            double global_y = obs_y + pt.first * obs_sin_theta + pt.second * obs_cos_theta;
            obs_rect_x.push_back(global_x);
            obs_rect_y.push_back(global_y);
        }
        obs_rect_x.push_back(obs_rect_x.front());
        obs_rect_y.push_back(obs_rect_y.front());

        // 初始化或更新障碍物矩形绘图对象
        if (!obs_traj_plots[i]) {
            obs_traj_plots[i].reset(new matplotlibcpp::Plot(
                "obs_rect_" + std::to_string(i),
                obs_rect_x, 
                obs_rect_y, 
                "c-"
            ));
        } else {
            obs_traj_plots[i]->update(obs_rect_x, obs_rect_y);
        }
    }

    // 更新自车矩形
    double x = ego_log[0].back();
    double y = ego_log[1].back();
    double theta = ego_log[2].back();
    const double half_len = VEHICLE_LENGTH / 2;
    const double half_wid = VEHICLE_WIDTH / 2;
    std::array<std::pair<double, double>, 4> local_points = {
        std::make_pair( half_len,  half_wid),  // 前右
        std::make_pair( half_len, -half_wid),  // 后右
        std::make_pair(-half_len, -half_wid),  // 后左
        std::make_pair(-half_len,  half_wid)   // 前左
    };
    std::vector<double> rect_x, rect_y;
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);
    for (const auto& pt : local_points) {
        double global_x = x + pt.first * cos_theta - pt.second * sin_theta;
        double global_y = y + pt.first * sin_theta + pt.second * cos_theta;
        rect_x.push_back(global_x);
        rect_y.push_back(global_y);
    }
    rect_x.push_back(rect_x.front());
    rect_y.push_back(rect_y.front()); // 修复之前的非法字符 rect_yउम्स
    if (!vehicle_rect_plot) {
        vehicle_rect_plot.reset(new matplotlibcpp::Plot(
            "vehicle_rect",
            rect_x, 
            rect_y, 
            "b-"
        ));
    }
    vehicle_rect_plot->update(rect_x, rect_y);

    // 更新自车历史轨迹
    if (!ego_plot) {
        ego_plot.reset(new matplotlibcpp::Plot(
            "ego_plot",
            ego_log[0],
            ego_log[1],
            "r-"
        ));
    } 
    ego_plot->update(ego_log[0], ego_log[1]);

    // 设置视图范围并刷新
    // plt::xlim(view_center.first - margin, view_center.first + margin);
    // plt::ylim(view_center.second - margin, view_center.second + margin);
    static std::pair<double, double> last_xlim, last_ylim;
    double new_xlim_min = view_center.first - margin;
    double new_xlim_max = view_center.first + margin;
    double new_ylim_min = view_center.second - margin;
    double new_ylim_max = view_center.second + margin;
    if (abs(new_xlim_min - last_xlim.first) > 0.1 || abs(new_xlim_max - last_xlim.second) > 0.1 ||
        abs(new_ylim_min - last_ylim.first) > 0.1 || abs(new_ylim_max - last_ylim.second) > 0.1) {
        plt::xlim(new_xlim_min, new_xlim_max);
        plt::ylim(new_ylim_min, new_ylim_max);
        last_xlim = {new_xlim_min, new_xlim_max};
        last_ylim = {new_ylim_min, new_ylim_max};
    }
    plt::grid(true);
    plt::pause(0.001); 
   plt::draw();

}