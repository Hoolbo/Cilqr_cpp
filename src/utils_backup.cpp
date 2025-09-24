#include "utils.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <regex>
// #include "nlohmann/json.hpp"


std::vector<std::vector<double>> load_map(){

    std::vector<std::vector<double>> map_info(3);

    //本来是用matlab导入地图，现在改成自己用正弦曲线拟合地图
    for(int i=0;i<1000;i++){
        map_info[0].push_back(i*1.0);
        map_info[1].push_back(sin(i*1.0*0.0628));
        map_info[2].push_back(atan2(sin(i*1.0*0.0628)-sin((i-1)*1.0*0.0628),1.0));
    } 

    return map_info;
}

MapData load_bitmap_map(const std::string& json_file_path) {
    MapData map_data;
    std::cout << "Loading map file: " << json_file_path << std::endl;
    // Attempt to open file to check existence; parsing is skipped in this stub
    std::ifstream file(json_file_path);
    if (!file.is_open()) {
        std::cout << "Warning: Cannot open map file, falling back to synthetic test map." << std::endl;
    } else {
        std::cout << "Map file opened (parsing stub); using synthetic test map for rendering check." << std::endl;
        file.close();
    }

    // Build synthetic map: gradient with a rectangular obstacle (-1.0)
    map_data.width = 200;
    map_data.height = 150;
    map_data.resolution = 0.1; // meters per pixel
    map_data.origin = {0.0, 0.0};
    map_data.max_elevation = 100.0;

    map_data.data.resize(map_data.height, std::vector<double>(map_data.width, 0.0));
    for (int i = 0; i < map_data.height; ++i) {
        for (int j = 0; j < map_data.width; ++j) {
            double v = (static_cast<double>(i) / map_data.height) * map_data.max_elevation;
            map_data.data[i][j] = v;
        }
    }
    // Add a rectangular obstacle region marked as -1.0
    for (int i = map_data.height/3; i < map_data.height/3 + 30; ++i) {
        for (int j = map_data.width/2; j < map_data.width/2 + 40; ++j) {
            if (i >= 0 && i < map_data.height && j >= 0 && j < map_data.width) {
                map_data.data[i][j] = -1.0;
            }
        }
    }

    std::cout << "Synthetic map prepared: " << map_data.width << "x" << map_data.height << ", res=" << map_data.resolution << std::endl;
    return map_data;
}

void draw_bitmap_debug(const MapData& map_data, const std::string& output_path) {
    namespace plt = matplotlibcpp;
    if (map_data.width <= 0 || map_data.height <= 0 || map_data.data.empty()) {
        throw std::runtime_error("draw_bitmap_debug: invalid map_data");
    }
    plt::figure();
    int height = map_data.height;
    int width = map_data.width;
    std::vector<float> map_for_display(height * width);
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            double v = (i < (int)map_data.data.size() && j < (int)map_data.data[i].size())
                        ? map_data.data[i][j] : 0.0;
            map_for_display[i * width + j] = (v == -1.0) ? 0.0f 
                : static_cast<float>((map_data.max_elevation > 0.0 ? v / map_data.max_elevation : 0.0));
        }
    }
    double x_min = map_data.origin.size() > 0 ? map_data.origin[0] : 0.0;
    double y_min = map_data.origin.size() > 1 ? map_data.origin[1] : 0.0;
    double x_max = x_min + width * map_data.resolution;
    double y_max = y_min + height * map_data.resolution;

    std::map<std::string, std::string> keywords;
    keywords["cmap"] = "gray";
    keywords["extent"] = std::to_string(x_min) + "," + std::to_string(x_max) + "," +
                         std::to_string(y_min) + "," + std::to_string(y_max);

    PyObject* im = plt::imshow(map_for_display.data(), height, width, 1, keywords);
    if (im) {
        plt::colorbar(im);
    } else {
        plt::colorbar();
    }
    plt::title("Bitmap Debug");
    plt::xlabel("X (m)");
    plt::ylabel("Y (m)");
    if (!output_path.empty()) {
        plt::save(output_path);
        std::cout << "Bitmap saved to: " << output_path << std::endl;
    } else {
        plt::show();
    }
}


void my_plot(const std::vector<std::vector<double>>& global_plan_log,
    const std::vector<std::vector<double>>& ego_log,
    const Trajectory& obs_traj,
    const Solution& solution,
    const MapData* map_data) 
{       
        namespace plt = matplotlibcpp;
        // 使用智能指针避免静态变量初始化问题
        static std::unique_ptr<matplotlibcpp::Plot> global_plot, ego_plot,obs_traj_plot, 
                                        trajectory_plot,vehicle_rect_plot,vehicle_rear_rect_plot;
        
        static bool figure_initialized = false;
        static bool map_background_set = false;
        constexpr double VEHICLE_LENGTH = 2.7;  // 车长（单位：米）
        constexpr double VEHICLE_FRONT_LENGTH = 2.7;  // 车长（单位：米）
        constexpr double VEHICLE_REAR_LENGTH = 2.7;  // 车长（单位：米）
        constexpr double VEHICLE_WIDTH = 2;   // 车宽
        // 动态视图参数
        constexpr double FOLLOW_FACTOR = 0.7;
        constexpr double BASE_MARGIN = 200.0;
        static std::pair<double, double> view_center = {0, 0};

        // 首次初始化图形窗口
        if (!figure_initialized) {
            figure_initialized = true;
            plt::figure();
            plt::title("CILQR PLANNING");
            plt::xlabel("X (m)");
            plt::ylabel("Y (m)");
            plt::grid(true);
            
            // 如果提供了地图数据，显示地图背景
            std::cout << "Checking map data: map_data=" << (map_data ? "valid" : "null") << std::endl;
            if (map_data) {
                std::cout << "Map size: " << map_data->width << "x" << map_data->height << std::endl;
            }
            if (map_data && map_data->width > 0 && map_data->height > 0 && !map_background_set) {
                std::cout << "Setting map background..." << std::endl;
                // 准备地图数据用于imshow - 转换为一维数组
                int height = map_data->height;
                int width = map_data->width;
                std::vector<float> map_for_display(height * width);
                
                // 转换地图数据，处理-1值（不可行区域），同时转换为一维数组
                for (int i = 0; i < height; ++i) {
                    for (int j = 0; j < width; ++j) {
                        if (i < map_data->data.size() && j < map_data->data[i].size()) {
                            double value = map_data->data[i][j];
                            if (value == -1.0) {
                                map_for_display[i * width + j] = 0.0f;  // 黑色表示不可行区域
                            } else {
                                // 将高程值归一化到0-1范围用于显示
                                map_for_display[i * width + j] = static_cast<float>(value / map_data->max_elevation);
                            }
                        } else {
                            map_for_display[i * width + j] = 0.0f;
                        }
                    }
                }
                
                // 计算地图在世界坐标系中的范围
                double map_width_m = width * map_data->resolution;
                double map_height_m = height * map_data->resolution;
                double x_min = map_data->origin[0];
                double x_max = x_min + map_width_m;
                double y_min = map_data->origin[1];
                double y_max = y_min + map_height_m;
                
                // 使用imshow显示地图背景
                std::map<std::string, std::string> keywords;
                keywords["cmap"] = "gray";
                keywords["extent"] = std::to_string(x_min) + "," + std::to_string(x_max) + "," + 
                                    std::to_string(y_min) + "," + std::to_string(y_max);
                plt::imshow(map_for_display.data(), height, width, 1, keywords);
                map_background_set = true;
                
                std::cout << "Map background set, range: X[" << x_min << ", " << x_max << "], Y[" << y_min << ", " << y_max << "]" << std::endl;
            }
            
            if (global_plan_log[0].empty()) {
                global_plot.reset(new matplotlibcpp::Plot(
                    "global_plot",
                    global_plan_log[0],
                    global_plan_log[1],
                    "k-."
                ));
            }
        }

        // Update planned trajectory points from solution
        std::vector<std::vector<double>> trajectory(2);
        trajectory[0].reserve(solution.ego_trj.states.size());
        trajectory[1].reserve(solution.ego_trj.states.size());
        for (size_t i = 0; i < solution.ego_trj.states.size(); ++i) {
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

        // Draw obstacle rectangle from first state
        if (!obs_traj.states.empty()) {
            double obs_x = obs_traj.states[0][0];
            double obs_y = obs_traj.states[0][1];
            double obs_theta = obs_traj.states[0][2];
            double c = std::cos(obs_theta), s = std::sin(obs_theta);
            double hl = 2.7 / 2.0, hw = 2.0 / 2.0;
            std::array<std::pair<double,double>,4> corners = {
                std::make_pair( hl,  hw),
                std::make_pair( hl, -hw),
                std::make_pair(-hl, -hw),
                std::make_pair(-hl,  hw)
            };
            std::vector<double> ox, oy;
            for (auto& p : corners) {
                ox.push_back(obs_x + p.first * c - p.second * s);
                oy.push_back(obs_y + p.first * s + p.second * c);
            }
            ox.push_back(ox.front());
            oy.push_back(oy.front());
            if (!obs_traj_plot) {
                obs_traj_plot.reset(new matplotlibcpp::Plot(" ", ox, oy, "c-"));
            }
            obs_traj_plot->update(ox, oy);
        }

        // Vehicle body rectangle (front)
        double x = ego_log[0].back();
        double y = ego_log[1].back();
        double theta = ego_log[2].back();
        double gamma = ego_log[3].back();
        double c = std::cos(theta), s = std::sin(theta);
        double half_len = VEHICLE_FRONT_LENGTH / 1.3;
        double half_wid = VEHICLE_WIDTH / 2.0;
        std::array<std::pair<double,double>,4> corners = {
            std::make_pair( half_len,  half_wid),
            std::make_pair( half_len, -half_wid),
            std::make_pair(-half_len, -half_wid),
            std::make_pair(-half_len,  half_wid)
        };
        std::vector<double> vx, vy;
        for (auto& p : corners) {
            vx.push_back(x + p.first * c - p.second * s);
            vy.push_back(y + p.first * s + p.second * c);
        }
        vx.push_back(vx.front());
        vy.push_back(vy.front());
        if (!vehicle_rect_plot) {
            vehicle_rect_plot.reset(new matplotlibcpp::Plot(" ", vx, vy, "b-"));
        }
        vehicle_rect_plot->update(vx, vy);

        // Articulated rear body rectangle
        double theta_r = theta + gamma;
        double xr = x - VEHICLE_FRONT_LENGTH * std::cos(theta) - VEHICLE_REAR_LENGTH * std::cos(theta_r);
        double yr = y - VEHICLE_FRONT_LENGTH * std::sin(theta) - VEHICLE_REAR_LENGTH * std::sin(theta_r);
        double cr = std::cos(theta_r), sr = std::sin(theta_r);
        double half_len_r = VEHICLE_REAR_LENGTH / 1.1;
        double half_wid_r = VEHICLE_WIDTH / 2.0;
        std::array<std::pair<double,double>,4> rcorners = {
            std::make_pair( half_len_r,  half_wid_r),
            std::make_pair( half_len_r, -half_wid_r),
            std::make_pair(-half_len_r, -half_wid_r),
            std::make_pair(-half_len_r,  half_wid_r)
        };
        std::vector<double> rx, ry;
        for (auto& p : rcorners) {
            rx.push_back(xr + p.first * cr - p.second * sr);
            ry.push_back(yr + p.first * sr + p.second * cr);
        }
        rx.push_back(rx.front());
        ry.push_back(ry.front());
        if (!vehicle_rear_rect_plot) {
            vehicle_rear_rect_plot.reset(new matplotlibcpp::Plot(" ", rx, ry, "b-"));
        }
        vehicle_rear_rect_plot->update(rx, ry);
}


// Duplicate function retained temporarily for reference; not used
#if 0
static void my_plot_duplicate_removed(const std::vector<std::vector<double>>& global_plan_log,
    const std::vector<std::vector<double>>& ego_log,
    const Trajectory& obs_traj,
    const Solution& solution,
    const MapData* map_data) 
{       
        namespace plt = matplotlibcpp;
        // 使用智能指针避免静态变量初始化问题
        static std::unique_ptr<matplotlibcpp::Plot> global_plot, ego_plot,obs_traj_plot, 
                                        trajectory_plot,vehicle_rect_plot,vehicle_rear_rect_plot;
        
        static bool figure_initialized = false;
        static bool map_background_set = false;
        constexpr double VEHICLE_LENGTH = 2.7;  // 车长（单位：米）
        constexpr double VEHICLE_FRONT_LENGTH = 2.7;  // 车长（单位：米）
        constexpr double VEHICLE_REAR_LENGTH = 2.7;  // 车长（单位：米）
        constexpr double VEHICLE_WIDTH = 2;   // 车宽
        // 动态视图参数
        constexpr double FOLLOW_FACTOR = 0.7;
        constexpr double BASE_MARGIN = 200.0;
        static std::pair<double, double> view_center = {0, 0};

        // 首次初始化图形窗口
        if (!figure_initialized) {
            figure_initialized = true;
            plt::figure();
            plt::title("CILQR PLANNING");
            plt::xlabel("X (m)");
            plt::ylabel("Y (m)");
            plt::grid(true);
            
            // 如果提供了地图数据，显示地图背景
            std::cout << "Checking map data: map_data=" << (map_data ? "valid" : "null") << std::endl;
            if (map_data) {
                std::cout << "Map size: " << map_data->width << "x" << map_data->height << std::endl;
            }
            if (map_data && map_data->width > 0 && map_data->height > 0 && !map_background_set) {
                std::cout << "Setting map background..." << std::endl;
                // 准备地图数据用于imshow - 转换为一维数组
                int height = map_data->height;
                int width = map_data->width;
                std::vector<float> map_for_display(height * width);
                
                // 转换地图数据，处理-1值（不可行区域），同时转换为一维数组
                for (int i = 0; i < height; ++i) {
                    for (int j = 0; j < width; ++j) {
                        if (i < map_data->data.size() && j < map_data->data[i].size()) {
                            double value = map_data->data[i][j];
                            if (value == -1.0) {
                                map_for_display[i * width + j] = 0.0f;  // 黑色表示不可行区域
                            } else {
                                // 将高程值归一化到0-1范围用于显示
                                map_for_display[i * width + j] = static_cast<float>(value / map_data->max_elevation);
                            }
                        } else {
                            map_for_display[i * width + j] = 0.0f;
                        }
                    }
                }
                
                // 计算地图在世界坐标系中的范围
                double map_width_m = width * map_data->resolution;
                double map_height_m = height * map_data->resolution;
                double x_min = map_data->origin[0];
                double x_max = x_min + map_width_m;
                double y_min = map_data->origin[1];
                double y_max = y_min + map_height_m;
                
                // 使用imshow显示地图背景
                std::map<std::string, std::string> keywords;
                keywords["cmap"] = "gray";
                keywords["extent"] = std::to_string(x_min) + "," + std::to_string(x_max) + "," + 
                                    std::to_string(y_min) + "," + std::to_string(y_max);
                plt::imshow(map_for_display.data(), height, width, 1, keywords);
                map_background_set = true;
                
                std::cout << "Map background set, range: X[" << x_min << ", " << x_max << "], Y[" << y_min << ", " << y_max << "]" << std::endl;
            }
            
            global_plot.reset(new matplotlibcpp::Plot(
                "global_plot",
                global_plan_log[0], 
                global_plan_log[1],
                "k-."
            ));
        }

        
        double target_x = ego_log[0].back();
        double target_y = ego_log[1].back();
        view_center.first += FOLLOW_FACTOR * (target_x - view_center.first);
        view_center.second += FOLLOW_FACTOR * (target_y - view_center.second);
        double speed = ego_log[3].back();
        double margin = BASE_MARGIN + speed * 3;



#if 0
        std::vector<std::vector<double>> trajectory(4);
        for(int i=0;i<solution.ego_trj.states.size();i++){
            trajectory[0].push_back(solution.ego_trj.states[i][0]);
            trajectory[1].push_back(solution.ego_trj.states[i][1]);
        }

        // 初始化并更新轨迹
        if(!trajectory_plot){
            trajectory_plot.reset(new matplotlibcpp::Plot(
                "trajectory_plot",
                trajectory[0],
                trajectory[1],
                "g-"
            ));
        }
        trajectory_plot->update(trajectory[0],trajectory[1]);
        
        //绘制障碍物
        double obs_x = obs_traj.states[0][0];
        double obs_y = obs_traj.states[0][1];
        double obs_theta = obs_traj.states[0][2]; 
        std::vector<double> obs_rect_x, obs_rect_y;
        double obs_cos_theta = cos(obs_theta);
        double obs_sin_theta = sin(obs_theta);
        double obs_half_length = 2.7 / 2;
        double obs_half_width = 2 / 2;
        std::array<std::pair<double, double>, 4> obs_local_points = {
            std::make_pair( obs_half_length,  obs_half_width),  // 前右
            std::make_pair( obs_half_length, -obs_half_width),  // 后右
            std::make_pair(-obs_half_length, -obs_half_width),  // 后左
            std::make_pair(-obs_half_length,  obs_half_width)   // 前左
        };
        for (const auto& pt : obs_local_points) {
            // 旋转和平移变换
            double global_x = obs_x + pt.first * obs_cos_theta - pt.second *  obs_sin_theta;
            double global_y = obs_y + pt.first *  obs_sin_theta + pt.second * obs_cos_theta;
            obs_rect_x.push_back(global_x);
            obs_rect_y.push_back(global_y);
        }
        obs_rect_x.push_back(obs_rect_x.front());
        obs_rect_y.push_back(obs_rect_y.front());
        // 更新或创建绘图对象
        if (!obs_traj_plot) {
            obs_traj_plot.reset(new matplotlibcpp::Plot(
                " ",
                obs_rect_x, 
                obs_rect_y, 
                "c-"
            ));
        }
        obs_traj_plot->update(obs_rect_x, obs_rect_y);


        // 更新车辆矩形
        double x = ego_log[0].back();
        double y = ego_log[1].back();
        double theta = ego_log[2].back(); 
        double gamma = ego_log[3].back();
        // 计算矩形四角相对坐标
        const double half_len = VEHICLE_FRONT_LENGTH /1.3;
        const double half_wid = VEHICLE_WIDTH / 2 ;
        std::array<std::pair<double, double>, 4> local_points = {
            std::make_pair( half_len,  half_wid),  // 前右
            std::make_pair( half_len, -half_wid),  // 后右
            std::make_pair(-half_len, -half_wid),  // 后左
            std::make_pair(-half_len,  half_wid)   // 前左
        };
        // 坐标系变换
        std::vector<double> rect_x, rect_y;
        double cos_theta = cos(theta);
        double sin_theta = sin(theta);
        for (const auto& pt : local_points) {
            // 旋转和平移变换
            double global_x = x + pt.first * cos_theta - pt.second * sin_theta;
            double global_y = y + pt.first * sin_theta + pt.second * cos_theta;
            rect_x.push_back(global_x);
            rect_y.push_back(global_y);
        }
        // 闭合矩形
        rect_x.push_back(rect_x.front());
        rect_y.push_back(rect_y.front());
        // 更新或创建绘图对象
        if (!vehicle_rect_plot) {
            vehicle_rect_plot.reset(new matplotlibcpp::Plot(
                " ",
                rect_x, 
                rect_y, 
                "b-"
            ));
        }
        vehicle_rect_plot->update(rect_x, rect_y);

    //绘制铰接车后部车框
    double theta_r  = theta + gamma ;
    double x_r = x - VEHICLE_FRONT_LENGTH * cos(theta) - VEHICLE_REAR_LENGTH * cos(theta_r);
    double y_r = y - VEHICLE_FRONT_LENGTH * sin(theta) - VEHICLE_REAR_LENGTH * sin(theta_r);
    // 计算矩形四角相对坐标
    const double half_len_r = VEHICLE_REAR_LENGTH / 1.1 ;
    const double half_wid_r = VEHICLE_WIDTH / 2;
    std::array<std::pair<double, double>, 4> local_points_r = {
        std::make_pair( half_len_r,  half_wid_r),  // 前右
        std::make_pair( half_len_r, -half_wid_r),  // 后右
        std::make_pair(-half_len_r, -half_wid_r),  // 后左
        std::make_pair(-half_len_r,  half_wid_r)   // 前左
    };
    // 坐标系变换
    std::vector<double> rect_x_r, rect_y_r;
    double cos_theta_r = cos(theta_r);
    double sin_theta_r = sin(theta_r);
    for (const auto& pt_r : local_points_r) {
        // 旋转和平移变换
        double global_x_r = x_r + pt_r.first * cos_theta_r - pt_r.second * sin_theta_r;
        double global_y_r = y_r + pt_r.first * sin_theta_r + pt_r.second * cos_theta_r;
        rect_x_r.push_back(global_x_r);
        rect_y_r.push_back(global_y_r);
    }
    // 闭合矩形
    rect_x_r.push_back(rect_x_r.front());
    rect_y_r.push_back(rect_y_r.front());
    // 更新或创建绘图对象
    if (!vehicle_rear_rect_plot) {
        vehicle_rear_rect_plot.reset(new matplotlibcpp::Plot(
            " ",
            rect_x_r, 
            rect_y_r, 
            "b-"
        ));
    }
    vehicle_rear_rect_plot->update(rect_x_r, rect_y_r);
#endif




        std::vector<std::vector<double>> trajectory(4);
        for(int i=0;i<solution.ego_trj.states.size();i++){
            trajectory[0].push_back(solution.ego_trj.states[i][0]);
            trajectory[1].push_back(solution.ego_trj.states[i][1]);
        }

        // 初始化并更新轨迹
        if(!trajectory_plot){
            trajectory_plot.reset(new matplotlibcpp::Plot(
                "trajectory_plot",
                trajectory[0],
                trajectory[1],
                "g-"
            ));
        }
        trajectory_plot->update(trajectory[0],trajectory[1]);
        
        //绘制障碍物
        double obs_x = obs_traj.states[0][0];
        double obs_y = obs_traj.states[0][1];
        double obs_theta = obs_traj.states[0][2]; 
        std::vector<double> obs_rect_x, obs_rect_y;
        double obs_cos_theta = cos(obs_theta);
        double obs_sin_theta = sin(obs_theta);
        double obs_half_length = 2.7 / 2;
        double obs_half_width = 2 / 2;
        std::array<std::pair<double, double>, 4> obs_local_points = {
            std::make_pair( obs_half_length,  obs_half_width),  // 前右
            std::make_pair( obs_half_length, -obs_half_width),  // 后右
            std::make_pair(-obs_half_length, -obs_half_width),  // 后左
            std::make_pair(-obs_half_length,  obs_half_width)   // 前左
        };
        for (const auto& pt : obs_local_points) {
            // 旋转和平移变换
            double global_x = obs_x + pt.first * obs_cos_theta - pt.second *  obs_sin_theta;
            double global_y = obs_y + pt.first *  obs_sin_theta + pt.second * obs_cos_theta;
            obs_rect_x.push_back(global_x);
            obs_rect_y.push_back(global_y);
        }
        obs_rect_x.push_back(obs_rect_x.front());
        obs_rect_y.push_back(obs_rect_y.front());
        // 更新或创建绘图对象
        if (!obs_traj_plot) {
            obs_traj_plot.reset(new matplotlibcpp::Plot(
                " ",
                obs_rect_x, 
                obs_rect_y, 
                "c-"
            ));
        }
        obs_traj_plot->update(obs_rect_x, obs_rect_y);


        // 更新车辆矩形
        double x = ego_log[0].back();
        double y = ego_log[1].back();
        double theta = ego_log[2].back(); 
        double gamma = ego_log[3].back();
        // 计算矩形四角相对坐标
        const double half_len = VEHICLE_FRONT_LENGTH /1.3;
        const double half_wid = VEHICLE_WIDTH / 2 ;
        std::array<std::pair<double, double>, 4> local_points = {
            std::make_pair( half_len,  half_wid),  // 前右
            std::make_pair( half_len, -half_wid),  // 后右
            std::make_pair(-half_len, -half_wid),  // 后左
            std::make_pair(-half_len,  half_wid)   // 前左
        };
        // 坐标系变换
        std::vector<double> rect_x, rect_y;
        double cos_theta = cos(theta);
        double sin_theta = sin(theta);
        for (const auto& pt : local_points) {
            // 旋转和平移变换
            double global_x = x + pt.first * cos_theta - pt.second * sin_theta;
            double global_y = y + pt.first * sin_theta + pt.second * cos_theta;
            rect_x.push_back(global_x);
            rect_y.push_back(global_y);
        }
        // 闭合矩形
        rect_x.push_back(rect_x.front());
        rect_y.push_back(rect_y.front());
        // 更新或创建绘图对象
        if (!vehicle_rect_plot) {
            vehicle_rect_plot.reset(new matplotlibcpp::Plot(
                " ",
                rect_x, 
                rect_y, 
                "b-"
            ));
        }
        vehicle_rect_plot->update(rect_x, rect_y);

    //绘制铰接车后部车框
    double theta_r  = theta + gamma ;
    double x_r = x - VEHICLE_FRONT_LENGTH * cos(theta) - VEHICLE_REAR_LENGTH * cos(theta_r);
    double y_r = y - VEHICLE_FRONT_LENGTH * sin(theta) - VEHICLE_REAR_LENGTH * sin(theta_r);
    // 计算矩形四角相对坐标
    const double half_len_r = VEHICLE_REAR_LENGTH / 1.1 ;
    const double half_wid_r = VEHICLE_WIDTH / 2;
    std::array<std::pair<double, double>, 4> local_points_r = {
        std::make_pair( half_len_r,  half_wid_r),  // 前右
        std::make_pair( half_len_r, -half_wid_r),  // 后右
        std::make_pair(-half_len_r, -half_wid_r),  // 后左
        std::make_pair(-half_len_r,  half_wid_r)   // 前左
    };
    // 坐标系变换
    std::vector<double> rect_x_r, rect_y_r;
    double cos_theta_r = cos(theta_r);
    double sin_theta_r = sin(theta_r);
    for (const auto& pt_r : local_points_r) {
        // 旋转和平移变换
        double global_x_r = x_r + pt_r.first * cos_theta_r - pt_r.second * sin_theta_r;
        double global_y_r = y_r + pt_r.first * sin_theta_r + pt_r.second * cos_theta_r;
        rect_x_r.push_back(global_x_r);
        rect_y_r.push_back(global_y_r);
    }
    // 闭合矩形
    rect_x_r.push_back(rect_x_r.front());
    rect_y_r.push_back(rect_y_r.front());
    // 更新或创建绘图对象
    if (!vehicle_rear_rect_plot) {
        vehicle_rear_rect_plot.reset(new matplotlibcpp::Plot(
            " ",
            rect_x_r, 
            rect_y_r, 
            "b-"
        ));
    }
    vehicle_rear_rect_plot->update(rect_x_r, rect_y_r);




        std::vector<std::vector<double>> trajectory(4);
        for(int i=0;i<solution.ego_trj.states.size();i++){
            trajectory[0].push_back(solution.ego_trj.states[i][0]);
            trajectory[1].push_back(solution.ego_trj.states[i][1]);
        }

        // 初始化并更新轨迹
        if(!trajectory_plot){
            trajectory_plot.reset(new matplotlibcpp::Plot(
                "trajectory_plot",
                trajectory[0],
                trajectory[1],
                "g-"
            ));
        }
        trajectory_plot->update(trajectory[0],trajectory[1]);
        
        //绘制障碍物
        double obs_x = obs_traj.states[0][0];
        double obs_y = obs_traj.states[0][1];
        double obs_theta = obs_traj.states[0][2]; 
        std::vector<double> obs_rect_x, obs_rect_y;
        double obs_cos_theta = cos(obs_theta);
        double obs_sin_theta = sin(obs_theta);
        double obs_half_length = 2.7 / 2;
        double obs_half_width = 2 / 2;
        std::array<std::pair<double, double>, 4> obs_local_points = {
            std::make_pair( obs_half_length,  obs_half_width),  // 前右
            std::make_pair( obs_half_length, -obs_half_width),  // 后右
            std::make_pair(-obs_half_length, -obs_half_width),  // 后左
            std::make_pair(-obs_half_length,  obs_half_width)   // 前左
        };
        for (const auto& pt : obs_local_points) {
            // 旋转和平移变换
            double global_x = obs_x + pt.first * obs_cos_theta - pt.second *  obs_sin_theta;
            double global_y = obs_y + pt.first *  obs_sin_theta + pt.second * obs_cos_theta;
            obs_rect_x.push_back(global_x);
            obs_rect_y.push_back(global_y);
        }
        obs_rect_x.push_back(obs_rect_x.front());
        obs_rect_y.push_back(obs_rect_y.front());
        // 更新或创建绘图对象
        if (!obs_traj_plot) {
            obs_traj_plot.reset(new matplotlibcpp::Plot(
                " ",
                obs_rect_x, 
                obs_rect_y, 
                "c-"
            ));
        }
        obs_traj_plot->update(obs_rect_x, obs_rect_y);


        // 更新车辆矩形
        double x = ego_log[0].back();
        double y = ego_log[1].back();
        double theta = ego_log[2].back(); 
        double gamma = ego_log[3].back();
        // 计算矩形四角相对坐标
        const double half_len = VEHICLE_FRONT_LENGTH /1.3;
        const double half_wid = VEHICLE_WIDTH / 2 ;
        std::array<std::pair<double, double>, 4> local_points = {
            std::make_pair( half_len,  half_wid),  // 前右
            std::make_pair( half_len, -half_wid),  // 后右
            std::make_pair(-half_len, -half_wid),  // 后左
            std::make_pair(-half_len,  half_wid)   // 前左
        };
        // 坐标系变换
        std::vector<double> rect_x, rect_y;
        double cos_theta = cos(theta);
        double sin_theta = sin(theta);
        for (const auto& pt : local_points) {
            // 旋转和平移变换
            double global_x = x + pt.first * cos_theta - pt.second * sin_theta;
            double global_y = y + pt.first * sin_theta + pt.second * cos_theta;
            rect_x.push_back(global_x);
            rect_y.push_back(global_y);
        }
        // 闭合矩形
        rect_x.push_back(rect_x.front());
        rect_y.push_back(rect_y.front());
        // 更新或创建绘图对象
        if (!vehicle_rect_plot) {
            vehicle_rect_plot.reset(new matplotlibcpp::Plot(
                " ",
                rect_x, 
                rect_y, 
                "b-"
            ));
        }
        vehicle_rect_plot->update(rect_x, rect_y);

    //绘制铰接车后部车框
    double theta_r  = theta + gamma ;
    double x_r = x - VEHICLE_FRONT_LENGTH * cos(theta) - VEHICLE_REAR_LENGTH * cos(theta_r);
    double y_r = y - VEHICLE_FRONT_LENGTH * sin(theta) - VEHICLE_REAR_LENGTH * sin(theta_r);
    // 计算矩形四角相对坐标
    const double half_len_r = VEHICLE_REAR_LENGTH / 1.1 ;
    const double half_wid_r = VEHICLE_WIDTH / 2;
    std::array<std::pair<double, double>, 4> local_points_r = {
        std::make_pair( half_len_r,  half_wid_r),  // 前右
        std::make_pair( half_len_r, -half_wid_r),  // 后右
        std::make_pair(-half_len_r, -half_wid_r),  // 后左
        std::make_pair(-half_len_r,  half_wid_r)   // 前左
    };
    // 坐标系变换
    std::vector<double> rect_x_r, rect_y_r;
    double cos_theta_r = cos(theta_r);
    double sin_theta_r = sin(theta_r);
    for (const auto& pt_r : local_points_r) {
        // 旋转和平移变换
        double global_x_r = x_r + pt_r.first * cos_theta_r - pt_r.second * sin_theta_r;
        double global_y_r = y_r + pt_r.first * sin_theta_r + pt_r.second * cos_theta_r;
        rect_x_r.push_back(global_x_r);
        rect_y_r.push_back(global_y_r);
    }
    // 闭合矩形
    rect_x_r.push_back(rect_x_r.front());
    rect_y_r.push_back(rect_y_r.front());
    // 更新或创建绘图对象
    if (!vehicle_rear_rect_plot) {
        vehicle_rear_rect_plot.reset(new matplotlibcpp::Plot(
            " ",
            rect_x_r, 
            rect_y_r, 
            "b-"
        ));
    }
    vehicle_rear_rect_plot->update(rect_x_r, rect_y_r);




        std::vector<std::vector<double>> trajectory(4);
        for(int i=0;i<solution.ego_trj.states.size();i++){
            trajectory[0].push_back(solution.ego_trj.states[i][0]);
            trajectory[1].push_back(solution.ego_trj.states[i][1]);
        }

        // 初始化并更新轨迹
        if(!trajectory_plot){
            trajectory_plot.reset(new matplotlibcpp::Plot(
                "trajectory_plot",
                trajectory[0],
                trajectory[1],
                "g-"
            ));
        }
        trajectory_plot->update(trajectory[0],trajectory[1]);
        
        //绘制障碍物
        double obs_x = obs_traj.states[0][0];
        double obs_y = obs_traj.states[0][1];
        double obs_theta = obs_traj.states[0][2]; 
        std::vector<double> obs_rect_x, obs_rect_y;
        double obs_cos_theta = cos(obs_theta);
        double obs_sin_theta = sin(obs_theta);
        double obs_half_length = 2.7 / 2;
        double obs_half_width = 2 / 2;
        std::array<std::pair<double, double>, 4> obs_local_points = {
            std::make_pair( obs_half_length,  obs_half_width),  // 前右
            std::make_pair( obs_half_length, -obs_half_width),  // 后右
            std::make_pair(-obs_half_length, -obs_half_width),  // 后左
            std::make_pair(-obs_half_length,  obs_half_width)   // 前左
        };
        for (const auto& pt : obs_local_points) {
            // 旋转和平移变换
            double global_x = obs_x + pt.first * obs_cos_theta - pt.second *  obs_sin_theta;
            double global_y = obs_y + pt.first *  obs_sin_theta + pt.second * obs_cos_theta;
            obs_rect_x.push_back(global_x);
            obs_rect_y.push_back(global_y);
        }
        obs_rect_x.push_back(obs_rect_x.front());
        obs_rect_y.push_back(obs_rect_y.front());
        // 更新或创建绘图对象
        if (!obs_traj_plot) {
            obs_traj_plot.reset(new matplotlibcpp::Plot(
                " ",
                obs_rect_x, 
                obs_rect_y, 
                "c-"
            ));
        }
        obs_traj_plot->update(obs_rect_x, obs_rect_y);


        // 更新车辆矩形
        double x = ego_log[0].back();
        double y = ego_log[1].back();
        double theta = ego_log[2].back(); 
        double gamma = ego_log[3].back();
        // 计算矩形四角相对坐标
        const double half_len = VEHICLE_FRONT_LENGTH /1.3;
        const double half_wid = VEHICLE_WIDTH / 2 ;
        std::array<std::pair<double, double>, 4> local_points = {
            std::make_pair( half_len,  half_wid),  // 前右
            std::make_pair( half_len, -half_wid),  // 后右
            std::make_pair(-half_len, -half_wid),  // 后左
            std::make_pair(-half_len,  half_wid)   // 前左
        };
        // 坐标系变换
        std::vector<double> rect_x, rect_y;
        double cos_theta = cos(theta);
        double sin_theta = sin(theta);
        for (const auto& pt : local_points) {
            // 旋转和平移变换
            double global_x = x + pt.first * cos_theta - pt.second * sin_theta;
            double global_y = y + pt.first * sin_theta + pt.second * cos_theta;
            rect_x.push_back(global_x);
            rect_y.push_back(global_y);
        }
        // 闭合矩形
        rect_x.push_back(rect_x.front());
        rect_y.push_back(rect_y.front());
        // 更新或创建绘图对象
        if (!vehicle_rect_plot) {
            vehicle_rect_plot.reset(new matplotlibcpp::Plot(
                " ",
                rect_x, 
                rect_y, 
                "b-"
            ));
        }
        vehicle_rect_plot->update(rect_x, rect_y);

    //绘制铰接车后部车框
    double theta_r  = theta + gamma ;
    double x_r = x - VEHICLE_FRONT_LENGTH * cos(theta) - VEHICLE_REAR_LENGTH * cos(theta_r);
    double y_r = y - VEHICLE_FRONT_LENGTH * sin(theta) - VEHICLE_REAR_LENGTH * sin(theta_r);
    // 计算矩形四角相对坐标
    const double half_len_r = VEHICLE_REAR_LENGTH / 1.1 ;
    const double half_wid_r = VEHICLE_WIDTH / 2;
    std::array<std::pair<double, double>, 4> local_points_r = {
        std::make_pair( half_len_r,  half_wid_r),  // 前右
        std::make_pair( half_len_r, -half_wid_r),  // 后右
        std::make_pair(-half_len_r, -half_wid_r),  // 后左
        std::make_pair(-half_len_r,  half_wid_r)   // 前左
    };
    // 坐标系变换
    std::vector<double> rect_x_r, rect_y_r;
    double cos_theta_r = cos(theta_r);
    double sin_theta_r = sin(theta_r);
    for (const auto& pt_r : local_points_r) {
        // 旋转和平移变换
        double global_x_r = x_r + pt_r.first * cos_theta_r - pt_r.second * sin_theta_r;
        double global_y_r = y_r + pt_r.first * sin_theta_r + pt_r.second * cos_theta_r;
        rect_x_r.push_back(global_x_r);
        rect_y_r.push_back(global_y_r);
    }
    // 闭合矩形
    rect_x_r.push_back(rect_x_r.front());
    rect_y_r.push_back(rect_y_r.front());
    // 更新或创建绘图对象
    if (!vehicle_rear_rect_plot) {
        vehicle_rear_rect_plot.reset(new matplotlibcpp::Plot(
            " ",
            rect_x_r, 
            rect_y_r, 
            "b-"
        ));
    }
    vehicle_rear_rect_plot->update(rect_x_r, rect_y_r);




        std::vector<std::vector<double>> trajectory(4);
        for(int i=0;i<solution.ego_trj.states.size();i++){
            trajectory[0].push_back(solution.ego_trj.states[i][0]);
            trajectory[1].push_back(solution.ego_trj.states[i][1]);
        }

        // 初始化并更新轨迹
        if(!trajectory_plot){
            trajectory_plot.reset(new matplotlibcpp::Plot(
                "trajectory_plot",
                trajectory[0],
                trajectory[1],
                "g-"
            ));
        }
        trajectory_plot->update(trajectory[0],trajectory[1]);
        
        //绘制障碍物
        double obs_x = obs_traj.states[0][0];
        double obs_y = obs_traj.states[0][1];
        double obs_theta = obs_traj.states[0][2]; 
        std::vector<double> obs_rect_x, obs_rect_y;
        double obs_cos_theta = cos(obs_theta);
        double obs_sin_theta = sin(obs_theta);
        double obs_half_length = 2.7 / 2;
        double obs_half_width = 2 / 2;
        std::array<std::pair<double, double>, 4> obs_local_points = {
            std::make_pair( obs_half_length,  obs_half_width),  // 前右
            std::make_pair( obs_half_length, -obs_half_width),  // 后右
            std::make_pair(-obs_half_length, -obs_half_width),  // 后左
            std::make_pair(-obs_half_length,  obs_half_width)   // 前左
        };
        for (const auto& pt : obs_local_points) {
            // 旋转和平移变换
            double global_x = obs_x + pt.first * obs_cos_theta - pt.second *  obs_sin_theta;
            double global_y = obs_y + pt.first *  obs_sin_theta + pt.second * obs_cos_theta;
            obs_rect_x.push_back(global_x);
            obs_rect_y.push_back(global_y);
        }
        obs_rect_x.push_back(obs_rect_x.front());
        obs_rect_y.push_back(obs_rect_y.front());
        // 更新或创建绘图对象
        if (!obs_traj_plot) {
            obs_traj_plot.reset(new matplotlibcpp::Plot(
                " ",
                obs_rect_x, 
                obs_rect_y, 
                "c-"
            ));
        }
        obs_traj_plot->update(obs_rect_x, obs_rect_y);


        // 更新车辆矩形
        double x = ego_log[0].back();
        double y = ego_log[1].back();
        double theta = ego_log[2].back(); 
        double gamma = ego_log[3].back();
        // 计算矩形四角相对坐标
        const double half_len = VEHICLE_FRONT_LENGTH /1.3;
        const double half_wid = VEHICLE_WIDTH / 2 ;
        std::array<std::pair<double, double>, 4> local_points = {
            std::make_pair( half_len,  half_wid),  // 前右
            std::make_pair( half_len, -half_wid),  // 后右
            std::make_pair(-half_len, -half_wid),  // 后左
            std::make_pair(-half_len,  half_wid)   // 前左
        };
        // 坐标系变换
        std::vector<double> rect_x, rect_y;
        double cos_theta = cos(theta);
        double sin_theta = sin(theta);
        for (const auto& pt : local_points) {
            // 旋转和平移变换
            double global_x = x + pt.first * cos_theta - pt.second * sin_theta;
            double global_y = y + pt.first * sin_theta + pt.second * cos_theta;
            rect_x.push_back(global_x);
            rect_y.push_back(global_y);
        }
        // 闭合矩形
        rect_x.push_back(rect_x.front());
        rect_y.push_back(rect_y.front());
        // 更新或创建绘图对象
        if (!vehicle_rect_plot) {
            vehicle_rect_plot.reset(new matplotlibcpp::Plot(
                " ",
                rect_x, 
                rect_y, 
                "b-"
            ));
        }
        vehicle_rect_plot->update(rect_x, rect_y);

    //绘制铰接车后部车框
    double theta_r  = theta + gamma ;
    double x_r = x - VEHICLE_FRONT_LENGTH * cos(theta) - VEHICLE_REAR_LENGTH * cos(theta_r);
    double y_r = y - VEHICLE_FRONT_LENGTH * sin(theta) - VEHICLE_REAR_LENGTH * sin(theta_r);
    // 计算矩形四角相对坐标
    const double half_len_r = VEHICLE_REAR_LENGTH / 1.1 ;
    const double half_wid_r = VEHICLE_WIDTH / 2;
    std::array<std::pair<double, double>, 4> local_points_r = {
        std::make_pair( half_len_r,  half_wid_r),  // 前右
        std::make_pair( half_len_r, -half_wid_r),  // 后右
        std::make_pair(-half_len_r, -half_wid_r),  // 后左
        std::make_pair(-half_len_r,  half_wid_r)   // 前左
    };
    // 坐标系变换
    std::vector<double> rect_x_r, rect_y_r;
    double cos_theta_r = cos(theta_r);
    double sin_theta_r = sin(theta_r);
    for (const auto& pt_r : local_points_r) {
        // 旋转和平移变换
        double global_x_r = x_r + pt_r.first * cos_theta_r - pt_r.second * sin_theta_r;
        double global_y_r = y_r + pt_r.first * sin_theta_r + pt_r.second * cos_theta_r;
        rect_x_r.push_back(global_x_r);
        rect_y_r.push_back(global_y_r);
    }
    // 闭合矩形
    rect_x_r.push_back(rect_x_r.front());
    rect_y_r.push_back(rect_y_r.front());
    // 更新或创建绘图对象
    if (!vehicle_rear_rect_plot) {
        vehicle_rear_rect_plot.reset(new matplotlibcpp::Plot(
            " ",
            rect_x_r, 
            rect_y_r, 
            "b-"
        ));
    }
    vehicle_rear_rect_plot->update(rect_x_r, rect_y_r);




        std::vector<std::vector<double>> trajectory(4);
        for(int i=0;i<solution.ego_trj.states.size();i++){
            trajectory[0].push_back(solution.ego_trj.states[i][0]);
            trajectory[1].push_back(solution.ego_trj.states[i][1]);
        }

        // 初始化并更新轨迹
        if(!trajectory_plot){
            trajectory_plot.reset(new matplotlibcpp::Plot(
                "trajectory_plot",
                trajectory[0],
                trajectory[1],
                "g-"
            ));
        }
        trajectory_plot->update(trajectory[0],trajectory[1]);
        
        //绘制障碍物
        double obs_x = obs_traj.states[0][0];
        double obs_y = obs_traj.states[0][1];
        double obs_theta = obs_traj.states[0][2]; 
        std::vector<double> obs_rect_x, obs_rect_y;
        double obs_cos_theta = cos(obs_theta);
        double obs_sin_theta = sin(obs_theta);
        double obs_half_length = 2.7 / 2;
        double obs_half_width = 2 / 2;
        std::array<std::pair<double, double>, 4> obs_local_points = {
            std::make_pair( obs_half_length,  obs_half_width),  // 前右
            std::make_pair( obs_half_length, -obs_half_width),  // 后右
            std::make_pair(-obs_half_length, -obs_half_width),  // 后左
            std::make_pair(-obs_half_length,  obs_half_width)   // 前左
        };
        for (const auto& pt : obs_local_points) {
            // 旋转和平移变换
            double global_x = obs_x + pt.first * obs_cos_theta - pt.second *  obs_sin_theta;
            double global_y = obs_y + pt.first *  obs_sin_theta + pt.second * obs_cos_theta;
            obs_rect_x.push_back(global_x);
            obs_rect_y.push_back(global_y);
        }
        obs_rect_x.push_back(obs_rect_x.front());
        obs_rect_y.push_back(obs_rect_y.front());
        // 更新或创建绘图对象
        if (!obs_traj_plot) {
            obs_traj_plot.reset(new matplotlibcpp::Plot(
                " ",
                obs_rect_x, 
                obs_rect_y, 
                "c-"
            ));
        }
        obs_traj_plot->update(obs_rect_x, obs_rect_y);


        // 更新车辆矩形
        double x = ego_log[0].back();
        double y = ego_log[1].back();
        double theta = ego_log[2].back(); 
        double gamma = ego_log[3].back();
        // 计算矩形四角相对坐标
        const double half_len = VEHICLE_FRONT_LENGTH /1.3;
        const double half_wid = VEHICLE_WIDTH / 2 ;
        std::array<std::pair<double, double>, 4> local_points = {
            std::make_pair( half_len,  half_wid),  // 前右
            std::make_pair( half_len, -half_wid),  // 后右
            std::make_pair(-half_len, -half_wid),  // 后左
            std::make_pair(-half_len,  half_wid)   // 前左
        };
        // 坐标系变换
        std::vector<double> rect_x, rect_y;
        double cos_theta = cos(theta);
        double sin_theta = sin(theta);
        for (const auto& pt : local_points) {
            // 旋转和平移变换
            double global_x = x + pt.first * cos_theta - pt.second * sin_theta;
            double global_y = y + pt.first * sin_theta + pt.second * cos_theta;
            rect_x.push_back(global_x);
            rect_y.push_back(global_y);
        }
        // 闭合矩形
        rect_x.push_back(rect_x.front());
        rect_y.push_back(rect_y.front());
        // 更新或创建绘图对象
        if (!vehicle_rect_plot) {
            vehicle_rect_plot.reset(new matplotlibcpp::Plot(
                " ",
                rect_x, 
                rect_y, 
                "b-"
            ));
        }
        vehicle_rect_plot->update(rect_x, rect_y);

    //绘制铰接车后部车框
    double theta_r  = theta + gamma ;
    double x_r = x - VEHICLE_FRONT_LENGTH * cos(theta) - VEHICLE_REAR_LENGTH * cos(theta_r);
    double y_r = y - VEHICLE_FRONT_LENGTH * sin(theta) - VEHICLE_REAR_LENGTH * sin(theta_r);
    // 计算矩形四角相对坐标
    const double half_len_r = VEHICLE_REAR_LENGTH / 1.1 ;
    const double half_wid_r = VEHICLE_WIDTH / 2;
    std::array<std::pair<double, double>, 4> local_points_r = {
        std::make_pair( half_len_r,  half_wid_r),  // 前右
        std::make_pair( half_len_r, -half_wid_r),  // 后右
        std::make_pair(-half_len_r, -half_wid_r),  // 后左
        std::make_pair(-half_len_r,  half_wid_r)   // 前左
    };
    // 坐标系变换
    std::vector<double> rect_x_r, rect_y_r;
    double cos_theta_r = cos(theta_r);
    double sin_theta_r = sin(theta_r);
    for (const auto& pt_r : local_points_r) {
        // 旋转和平移变换
        double global_x_r = x_r + pt_r.first * cos_theta_r - pt_r.second * sin_theta_r;
        double global_y_r = y_r + pt_r.first * sin_theta_r + pt_r.second * cos_theta_r;
        rect_x_r.push_back(global_x_r);
        rect_y_r.push_back(global_y_r);
    }
    // 闭合矩形
    rect_x_r.push_back(rect_x_r.front());
    rect_y_r.push_back(rect_y_r.front());
    // 更新或创建绘图对象
    if (!vehicle_rear_rect_plot) {
        vehicle_rear_rect_plot.reset(new matplotlibcpp::Plot(
            " ",
            rect_x_r, 
            rect_y_r, 
            "b-"
        ));
    }
    vehicle_rear_rect_plot->update(rect_x_r, rect_y_r);




        std::vector<std::vector<double>> trajectory(4);
        for(int i=0;i<solution.ego_trj.states.size();i++){
            trajectory[0].push_back(solution.ego_trj.states[i][0]);
            trajectory[1].push_back(solution.ego_trj.states[i][1]);
        }

        // 初始化并更新轨迹
        if(!trajectory_plot){
            trajectory_plot.reset(new matplotlibcpp::Plot(
                "trajectory_plot",
                trajectory[0],
                trajectory[1],
                "g-"
            ));
        }
        trajectory_plot->update(trajectory[0],trajectory[1]);
        
        //绘制障碍物
        double obs_x = obs_traj.states[0][0];
        double obs_y = obs_traj.states[0][1];
        double obs_theta = obs_traj.states[0][2]; 
        std::vector<double> obs_rect_x, obs_rect_y;
        double obs_cos_theta = cos(obs_theta);
        double obs_sin_theta = sin(obs_theta);
        double obs_half_length = 2.7 / 2;
        double obs_half_width = 2 / 2;
        std::array<std::pair<double, double>, 4> obs_local_points = {
            std::make_pair( obs_half_length,  obs_half_width),  // 前右
            std::make_pair( obs_half_length, -obs_half_width),  // 后右
            std::make_pair(-obs_half_length, -obs_half_width),  // 后左
            std::make_pair(-obs_half_length,  obs_half_width)   // 前左
        };
        for (const auto& pt : obs_local_points) {
            // 旋转和平移变换
            double global_x = obs_x + pt.first * obs_cos_theta - pt.second *  obs_sin_theta;
            double global_y = obs_y + pt.first *  obs_sin_theta + pt.second * obs_cos_theta;
            obs_rect_x.push_back(global_x);
            obs_rect_y.push_back(global_y);
        }
        obs_rect_x.push_back(obs_rect_x.front());
        obs_rect_y.push_back(obs_rect_y.front());
        // 更新或创建绘图对象
        if (!obs_traj_plot) {
            obs_traj_plot.reset(new matplotlibcpp::Plot(
                " ",
                obs_rect_x, 
                obs_rect_y, 
                "c-"
            ));
        }
        obs_traj_plot->update(obs_rect_x, obs_rect_y);


        // 更新车辆矩形
        double x = ego_log[0].back();
        double y = ego_log[1].back();
        double theta = ego_log[2].back(); 
        double gamma = ego_log[3].back();
        // 计算矩形四角相对坐标
        const double half_len = VEHICLE_FRONT_LENGTH /1.3;
        const double half_wid = VEHICLE_WIDTH / 2 ;
        std::array<std::pair<double, double>, 4> local_points = {
            std::make_pair( half_len,  half_wid),  // 前右
            std::make_pair( half_len, -half_wid),  // 后右
            std::make_pair(-half_len, -half_wid),  // 后左
            std::make_pair(-half_len,  half_wid)   // 前左
        };
        // 坐标系变换
        std::vector<double> rect_x, rect_y;
        double cos_theta = cos(theta);
        double sin_theta = sin(theta);
        for (const auto& pt : local_points) {
            // 旋转和平移变换
            double global_x = x + pt.first * cos_theta - pt.second * sin_theta;
            double global_y = y + pt.first * sin_theta + pt.second * cos_theta;
            rect_x.push_back(global_x);
            rect_y.push_back(global_y);
        }
        // 闭合矩形
        rect_x.push_back(rect_x.front());
        rect_y.push_back(rect_y.front());
        // 更新或创建绘图对象
        if (!vehicle_rect_plot) {
            vehicle_rect_plot.reset(new matplotlibcpp::Plot(
                " ",
                rect_x, 
                rect_y, 
                "b-"
            ));
        }
        vehicle_rect_plot->update(rect_x, rect_y);

    //绘制铰接车后部车框
    double theta_r  = theta + gamma ;
    double x_r = x - VEHICLE_FRONT_LENGTH * cos(theta) - VEHICLE_REAR_LENGTH * cos(theta_r);
    double y_r = y - VEHICLE_FRONT_LENGTH * sin(theta) - VEHICLE_REAR_LENGTH * sin(theta_r);
    // 计算矩形四角相对坐标
    const double half_len_r = VEHICLE_REAR_LENGTH / 1.1 ;
    const double half_wid_r = VEHICLE_WIDTH / 2;
    std::array<std::pair<double, double>, 4> local_points_r = {
        std::make_pair( half_len_r,  half_wid_r),  // 前右
        std::make_pair( half_len_r, -half_wid_r),  // 后右
        std::make_pair(-half_len_r, -half_wid_r),  // 后左
        std::make_pair(-half_len_r,  half_wid_r)   // 前左
    };
    // 坐标系变换
    std::vector<double> rect_x_r, rect_y_r;
    double cos_theta_r = cos(theta_r);
    double sin_theta_r = sin(theta_r);
    for (const auto& pt_r : local_points_r) {
        // 旋转和平移变换
        double global_x_r = x_r + pt_r.first * cos_theta_r - pt_r.second * sin_theta_r;
        double global_y_r = y_r + pt_r.first * sin_theta_r + pt_r.second * cos_theta_r;
        rect_x_r.push_back(global_x_r);
        rect_y_r.push_back(global_y_r);
    }
    // 闭合矩形
    rect_x_r.push_back(rect_x_r.front());
    rect_y_r.push_back(rect_y_r.front());
    // 更新或创建绘图对象
    if (!vehicle_rear_rect_plot) {
        vehicle_rear_rect_plot.reset(new matplotlibcpp::Plot(
            " ",
            rect_x_r, 
            rect_y_r, 
            "b-"
        ));
    }
    vehicle_rear_rect_plot->update(rect_x_r, rect_y_r);




        std::vector<std::vector<double>> trajectory(4);
        for(int i=0;i<solution.ego_trj.states.size();i++){
            trajectory[0].push_back(solution.ego_trj.states[i][0]);
            trajectory[1].push_back(solution.ego_trj.states[i][1]);
        }

        // 初始化并更新轨迹
        if(!trajectory_plot){
            trajectory_plot.reset(new matplotlibcpp::Plot(
                "trajectory_plot",
                trajectory[0],
                trajectory[1],
                "g-"
            ));
        }
        trajectory_plot->update(trajectory[0],trajectory[1]);
        
        //绘制障碍物
        double obs_x = obs_traj.states[0][0];
        double obs_y = obs_traj.states[0][1];
        double obs_theta = obs_traj.states[0][2]; 
        std::vector<double> obs_rect_x, obs_rect_y;
        double obs_cos_theta = cos(obs_theta);
        double obs_sin_theta = sin(obs_theta);
        double obs_half_length = 2.7 / 2;
        double obs_half_width = 2 / 2;
        std::array<std::pair<double, double>, 4
