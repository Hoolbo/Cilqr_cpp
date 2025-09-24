#include "utils.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <algorithm>
#include <cmath>
#include <array>
#include <memory>
#include "matplotlibcpp.h"
#ifdef _WIN32
#include <windows.h>
#endif

std::vector<std::vector<double>> load_map(double startx, double starty, double theta) {
    // 返回一个示例地图数据，实际应该从文件加载
    std::vector<std::vector<double>> map_data(3);
    double costheta = cos(theta);
    double sintheta = sin(theta); 
    // 生成一条简单的直线路径作为示例
    for (int i = 0; i < 1000; ++i) {
        map_data[0].push_back(startx + i*0.1*costheta);  // x坐标
        map_data[1].push_back(starty + i*0.1*sintheta);      // y坐标
        map_data[2].push_back(theta);      // heading
    }
    
    return map_data;
}

MapData load_bitmap_map(const std::string& file_path) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open bitmap map file: " << file_path << std::endl;
        return MapData{};
    }

    MapData map_data;
    std::string json_content((std::istreambuf_iterator<char>(file)),
                             std::istreambuf_iterator<char>());
    
    // 简单的JSON解析 - 查找metadata部分
    size_t metadata_pos = json_content.find("\"metadata\"");
    if (metadata_pos == std::string::npos) {
        std::cerr << "Error: Could not find metadata in JSON file" << std::endl;
        return MapData{};
    }
    
    // 解析width
    size_t width_pos = json_content.find("\"width\"", metadata_pos);
    if (width_pos != std::string::npos) {
        size_t colon_pos = json_content.find(":", width_pos);
        size_t comma_pos = json_content.find(",", colon_pos);
        std::string width_str = json_content.substr(colon_pos + 1, comma_pos - colon_pos - 1);
        map_data.width = std::stoi(width_str);
    }
    
    // 解析height
    size_t height_pos = json_content.find("\"height\"", metadata_pos);   
    if (height_pos != std::string::npos) {
        size_t colon_pos = json_content.find(":", height_pos);
        size_t comma_pos = json_content.find(",", colon_pos);
        std::string height_str = json_content.substr(colon_pos + 1, comma_pos - colon_pos - 1);
        map_data.height = std::stoi(height_str);
    }
    
    // 解析resolution
    size_t res_pos = json_content.find("\"resolution\"", metadata_pos);
    if (res_pos != std::string::npos) {
        size_t colon_pos = json_content.find(":", res_pos);
        size_t end_pos = json_content.find("}", colon_pos);
        std::string res_str = json_content.substr(colon_pos + 1, end_pos - colon_pos - 1);
        map_data.resolution = std::stod(res_str);
    }
    
    // 解析origin
    size_t origin_pos = json_content.find("\"origin\"", metadata_pos);
    if (origin_pos != std::string::npos) {
        size_t bracket_start = json_content.find("[", origin_pos);
        size_t bracket_end = json_content.find("]", bracket_start);
        std::string origin_str = json_content.substr(bracket_start + 1, bracket_end - bracket_start - 1);
        size_t comma_pos = origin_str.find(",");
        double origin_x = std::stod(origin_str.substr(0, comma_pos));
        double origin_y = std::stod(origin_str.substr(comma_pos + 1));
        map_data.origin = {origin_x, origin_y};
    }
    
    // 解析max_elevation
    size_t max_elev_pos = json_content.find("\"max_elevation\"", metadata_pos);
    if (max_elev_pos != std::string::npos) {
        size_t colon_pos = json_content.find(":", max_elev_pos);
        size_t end_pos = json_content.find(",", colon_pos);
        if (end_pos == std::string::npos) end_pos = json_content.find("}", colon_pos);
        std::string max_elev_str = json_content.substr(colon_pos + 1, end_pos - colon_pos - 1);
        map_data.max_elevation = std::stod(max_elev_str);
    }
    
    // 解析data数组
    size_t data_pos = json_content.find("\"data\"");
    if (data_pos != std::string::npos) {
        size_t data_start = json_content.find("[", data_pos);
        size_t data_end = json_content.rfind("]");
        std::string data_str = json_content.substr(data_start + 1, data_end - data_start - 1);
        
        map_data.data.resize(map_data.height);
        
        // 解析每一行
        size_t row_start = 0;
        for (int i = 0; i < map_data.height && row_start < data_str.length(); ++i) {
            map_data.data[i].resize(map_data.width);
            
            // 找到这一行的开始和结束
            size_t row_bracket_start = data_str.find("[", row_start);
            size_t row_bracket_end = data_str.find("]", row_bracket_start);
            
            if (row_bracket_start != std::string::npos && row_bracket_end != std::string::npos) {
                std::string row_str = data_str.substr(row_bracket_start + 1, row_bracket_end - row_bracket_start - 1);
                
                // 解析这一行的数据
                std::istringstream row_stream(row_str);
                std::string value;
                int j = 0;
                while (std::getline(row_stream, value, ',') && j < map_data.width) {
                    // 去除空格和引号
                    value.erase(std::remove_if(value.begin(), value.end(), ::isspace), value.end());
                    if (value.front() == '"') value = value.substr(1, value.length() - 2);
                    
                    if (value == "-1") {
                        map_data.data[i][j] = -1.0;
                    } else {
                        map_data.data[i][j] = std::stod(value);
                    }
                    j++;
                }
                
                row_start = row_bracket_end + 1;
            } else {
                break;
            }
        }
    }
    
    std::cout << "Bitmap map loaded: " << map_data.width << " x " << map_data.height 
              << ", resolution: " << map_data.resolution << ", max_elevation: " << map_data.max_elevation << std::endl;
    return map_data;
}

SemanticMapData load_semantic_map(const std::string& file_path) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open map file" << std::endl;
        return SemanticMapData{};
    }

    SemanticMapData map_data;
    std::string line;
    
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        SemanticMapPoint point;
        if (iss >> point.x >> point.y >> point.type) {
            map_data.points.push_back(point);
        }
    }
    
    std::cout << "Semantic map loaded. Size: " << map_data.points.size() << std::endl;
    return map_data;
}

void fill_global_path_points(std::vector<std::vector<double>>& global_plan_log) {
    global_plan_log.clear();
    global_plan_log.resize(2);
    
    double start_x = 450.0;
    double start_y = 30.0;
    double end_x = 550.0;
    double end_y = 50.0;
    
    int num_points = 100;
    for (int i = 0; i < num_points; ++i) {
        double t = static_cast<double>(i) / (num_points - 1);
        double x = start_x + t * (end_x - start_x);
        double y = start_y + t * (end_y - start_y);
        
        global_plan_log[0].push_back(x);
        global_plan_log[1].push_back(y);
    }
    
    std::cout << "Global path points filled: " << global_plan_log[0].size() << " points" << std::endl;
}

void set_global_path(GlobalPlan& global_plan, const std::vector<std::vector<double>>& global_plan_log) {
    // Clear existing path by setting empty vector
    global_plan.set_plan(std::vector<Point>());
    
    if (global_plan_log.size() >= 2 && !global_plan_log[0].empty()) {
        std::vector<Point> points;
        for (size_t i = 0; i < global_plan_log[0].size(); ++i) {
            Point point(global_plan_log[0][i], global_plan_log[1][i], 0.0);
            points.push_back(point);
        }
        global_plan.set_plan(points);
    }
    
    std::cout << "Global path set with " << global_plan.get_points().size() << " points" << std::endl;
}

void init_params(Params& params) {
    params.Q = Eigen::MatrixXd::Identity(4, 4);
    params.Q(0, 0) = 10.0;
    params.Q(1, 1) = 10.0;
    params.Q(2, 2) = 1.0;
    params.Q(3, 3) = 1.0;
    
    params.R = Eigen::MatrixXd::Identity(2, 2);
    params.R(0, 0) = 0.1;
    params.R(1, 1) = 0.1;
    
    params.Qf = 100.0 * Eigen::MatrixXd::Identity(4, 4);
    
    params.dt = 0.1;
    params.N = 50;
    params.max_iter = 20;
    params.tol = 1e-4;
    
    std::cout << "Parameters initialized" << std::endl;
}

void init_vehicle_model(VehicleModel& vehicle_model) {
    vehicle_model.L1 = 2.7;
    vehicle_model.L2 = 2.7;
    vehicle_model.width = 2.0;
    
    std::cout << "Vehicle model initialized: L1=" << vehicle_model.L1 
              << ", L2=" << vehicle_model.L2 << ", width=" << vehicle_model.width << std::endl;
}

void init_obstacle_trajectory(Trajectory& obs_traj) {
    obs_traj.states.clear();
    
    State obs_state;
    obs_state << 480.0, 35.0, 0.0, 0.0;
    
    int N = 50;
    for (int i = 0; i < N; ++i) {
        obs_traj.states.push_back(obs_state);
    }
    
    std::cout << "Obstacle trajectory initialized with " << obs_traj.states.size() << " states" << std::endl;
}

void draw_bitmap_debug(const MapData& map_data, const std::string& output_path) {
    if (map_data.width <= 0 || map_data.height <= 0 || map_data.data.empty()) {
        throw std::runtime_error("draw_bitmap_debug: invalid map_data");
    }
    
    std::cout << "Drawing bitmap map: " << map_data.width << "x" << map_data.height << std::endl;
    
    // 设置matplotlib后端为Agg（非交互式）
    matplotlibcpp::backend("Agg");
    
    int height = map_data.height;
    int width = map_data.width;
    std::vector<float> map_for_display(height * width);
    
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            double v = (i < (int)map_data.data.size() && j < (int)map_data.data[i].size())
                        ? map_data.data[i][j] : 0.0;
            
            if (v == -1.0) {
                map_for_display[i * width + j] = 0.0f;
            } else {
                map_for_display[i * width + j] = static_cast<float>(
                    map_data.max_elevation > 0.0 ? v / map_data.max_elevation : 0.0);
            }
        }
    }
    
    double x_min = map_data.origin.size() > 0 ? map_data.origin[0] : 0.0;
    double y_min = map_data.origin.size() > 1 ? map_data.origin[1] : 0.0;
    double x_max = x_min + width * map_data.resolution;
    double y_max = y_min + height * map_data.resolution;
    
    std::cout << "Map coordinates: x=[" << x_min << ", " << x_max << "], y=[" << y_min << ", " << y_max << "]" << std::endl;
    std::cout << "Map origin size: " << map_data.origin.size() << std::endl;
    if (map_data.origin.size() >= 2) {
        std::cout << "Origin: [" << map_data.origin[0] << ", " << map_data.origin[1] << "]" << std::endl;
    }

    std::map<std::string, std::string> keywords;
    keywords["cmap"] = "gray";
    keywords["extent"] = std::to_string(x_min) + "," + std::to_string(x_max) + "," +
                         std::to_string(y_min) + "," + std::to_string(y_max);

    matplotlibcpp::imshow(map_for_display.data(), height, width, 1, keywords);
    matplotlibcpp::colorbar();
    matplotlibcpp::title("Bitmap Debug");
    matplotlibcpp::xlabel("X (m)");
    matplotlibcpp::ylabel("Y (m)");
    
    std::string save_path = output_path.empty() ? "bitmap_debug.png" : output_path;
    matplotlibcpp::save(save_path);
    std::cout << "Map saved to: " << save_path << std::endl;
}

void my_plot(const std::vector<std::vector<double>>& global_plan_log,
             const std::vector<std::vector<double>>& ego_log,
             const Trajectory& obs_traj,
             const Solution& solution,
             const MapData* map_data) {
    std::cout << "\n============================" << std::endl;
    std::cout << "my_plot called - generating visualization" << std::endl;
    
    if (!ego_log.empty() && !ego_log[0].empty()) {
        std::cout << "Current vehicle state:" << std::endl;
        std::cout << "  Position: (" << ego_log[0].back() << ", " << ego_log[1].back() << ")" << std::endl;
        if (ego_log.size() > 2) {
            std::cout << "  Heading: " << ego_log[2].back() << " rad" << std::endl;
        }
        if (ego_log.size() > 3) {
            std::cout << "  Articulation angle: " << ego_log[3].back() << " rad" << std::endl;
        }
    }
    
    if (!solution.ego_trj.states.empty()) {
        std::cout << "Planned trajectory: " << solution.ego_trj.states.size() << " points" << std::endl;
        std::cout << "  Start: (" << solution.ego_trj.states[0][0] << ", " << solution.ego_trj.states[0][1] << ")" << std::endl;
        std::cout << "  End: (" << solution.ego_trj.states.back()[0] << ", " << solution.ego_trj.states.back()[1] << ")" << std::endl;
    }
    
    if (!obs_traj.states.empty()) {
        std::cout << "Obstacle position: (" << obs_traj.states[0][0] << ", " << obs_traj.states[0][1] << ")" << std::endl;
    }
    
    if (map_data && map_data->width > 0 && map_data->height > 0) {
        std::cout << "Drawing bitmap map..." << std::endl;
        draw_bitmap_debug(*map_data, "");
    } else {
        std::cout << "No valid map data available for visualization" << std::endl;
    }
    
    std::cout << "============================\n" << std::endl;
}

void save_map_data(const MapData* map_data) {
    if (!map_data || map_data->width <= 0 || map_data->height <= 0 || map_data->data.empty()) {
        std::cout << "No valid map data to save" << std::endl;
        return;
    }
    
    // 获取当前工作目录并构建地图文件路径
    std::string map_filename;
    #ifdef _WIN32
        char buffer[MAX_PATH];
        GetModuleFileNameA(NULL, buffer, MAX_PATH);
        std::string exe_path(buffer);
        std::string exe_dir = exe_path.substr(0, exe_path.find_last_of("\\"));
        // 从可执行文件目录向上找到项目根目录
        size_t build_pos = exe_dir.find("\\build");
        if (build_pos != std::string::npos) {
            std::string project_root = exe_dir.substr(0, build_pos);
            map_filename = project_root + "\\outputs\\maps\\map_data.json";
        } else {
            map_filename = "outputs/maps/map_data.json";
        }
    #else
        map_filename = "../outputs/maps/map_data.json";
    #endif
    
    std::ofstream map_file(map_filename);
    if (map_file.is_open()) {
        map_file << "{\n";
        map_file << "  \"width\": " << map_data->width << ",\n";
        map_file << "  \"height\": " << map_data->height << ",\n";
        map_file << "  \"resolution\": " << map_data->resolution << ",\n";
        map_file << "  \"max_elevation\": " << map_data->max_elevation << ",\n";
        map_file << "  \"origin\": [";
        if (map_data->origin.size() >= 2) {
            map_file << map_data->origin[0] << ", " << map_data->origin[1];
        } else {
            map_file << "0, 0";
        }
        map_file << "],\n";
        map_file << "  \"data\": [";
        for (int i = 0; i < map_data->height; ++i) {
            map_file << "[";
            for (int j = 0; j < map_data->width; ++j) {
                double v = (i < (int)map_data->data.size() && j < (int)map_data->data[i].size())
                            ? map_data->data[i][j] : 0.0;
                map_file << v;
                if (j < map_data->width - 1) map_file << ", ";
            }
            map_file << "]";
            if (i < map_data->height - 1) map_file << ", ";
        }
        map_file << "]\n";
        map_file << "}\n";
        map_file.close();
        
        std::cout << "Map data saved to: " << map_filename << std::endl;
    } else {
        std::cout << "Error: Could not open file " << map_filename << " for writing" << std::endl;
    }
}

void dynamic_plot(const std::vector<std::vector<double>>& global_plan_log,
                  const std::vector<std::vector<double>>& ego_log,
                  const Trajectory& obs_traj,
                  const Solution& solution,
                  const MapData* map_data,
                  const GlobalPlan& global_plan,
                  const SystemModel& vehicle_model,
                  const Arg& arg) {
    
    // 保存数据到文件供Python脚本使用
    static int frame_count = 0;
    
    // 获取当前工作目录并构建数据文件路径
    std::string data_filename;
    #ifdef _WIN32
        char buffer[MAX_PATH];
        GetModuleFileNameA(NULL, buffer, MAX_PATH);
        std::string exe_path(buffer);
        std::string exe_dir = exe_path.substr(0, exe_path.find_last_of("\\"));
        // 从可执行文件目录向上找到项目根目录
        size_t build_pos = exe_dir.find("\\build");
        if (build_pos != std::string::npos) {
            std::string project_root = exe_dir.substr(0, build_pos);
            data_filename = project_root + "\\outputs\\data\\cilqr_data_" + std::to_string(frame_count++) + ".json";
        } else {
            data_filename = "outputs/data/cilqr_data_" + std::to_string(frame_count++) + ".json";
        }
    #else
        data_filename = "../outputs/data/cilqr_data_" + std::to_string(frame_count++) + ".json";
    #endif
    
    std::ofstream data_file(data_filename);
    if (data_file.is_open()) {
        data_file << "{\n";
        
        // 保存算法收敛信息（放在开头）
        data_file << "  \"convergence_info\": {\n";
        data_file << "    \"converged\": " << (solution.converged ? "true" : "false") << ",\n";
        data_file << "    \"iterations\": " << solution.iterations << ",\n";
        data_file << "    \"final_cost\": " << solution.final_cost << ",\n";
        data_file << "    \"solve_time_ms\": " << solution.solve_time_ms << "\n";
        data_file << "  },\n";
        
        // 保存控制序列数据
        data_file << "  \"control_sequence\": {\n";
        if (!solution.control_sequence.controls.empty()) {
            data_file << "    \"velocity\": [";
            for (size_t i = 0; i < solution.control_sequence.controls.size(); ++i) {
                data_file << solution.control_sequence.controls[i][0];
                if (i < solution.control_sequence.controls.size() - 1) data_file << ", ";
            }
            data_file << "],\n";
            
            data_file << "    \"steering\": [";
            for (size_t i = 0; i < solution.control_sequence.controls.size(); ++i) {
                data_file << solution.control_sequence.controls[i][1];
                if (i < solution.control_sequence.controls.size() - 1) data_file << ", ";
            }
            data_file << "]\n";
        } else {
            data_file << "    \"velocity\": [],\n";
            data_file << "    \"steering\": []\n";
        }
        data_file << "  },\n";
        
        // 保存全局路径数据
        data_file << "  \"global_plan\": {\n";
        std::vector<Point> global_points = global_plan.get_points();
        if (!global_points.empty()) {
            data_file << "    \"x\": [";
            for (size_t i = 0; i < global_points.size(); ++i) {
                data_file << global_points[i].x;
                if (i < global_points.size() - 1) data_file << ", ";
            }
            data_file << "],\n";
            
            data_file << "    \"y\": [";
            for (size_t i = 0; i < global_points.size(); ++i) {
                data_file << global_points[i].y;
                if (i < global_points.size() - 1) data_file << ", ";
            }
            data_file << "]\n";
        } else {
            data_file << "    \"x\": [],\n";
            data_file << "    \"y\": []\n";
        }
        data_file << "  },\n";
        
        // 保存全局路径日志数据
        data_file << "  \"global_plan_log\": {\n";
        if (!global_plan_log.empty() && !global_plan_log[0].empty()) {
            data_file << "    \"x\": [";
            for (size_t i = 0; i < global_plan_log[0].size(); ++i) {
                data_file << global_plan_log[0][i];
                if (i < global_plan_log[0].size() - 1) data_file << ", ";
            }
            data_file << "],\n";
            
            data_file << "    \"y\": [";
            for (size_t i = 0; i < global_plan_log[1].size(); ++i) {
                data_file << global_plan_log[1][i];
                if (i < global_plan_log[1].size() - 1) data_file << ", ";
            }
            data_file << "]\n";
        } else {
            data_file << "    \"x\": [],\n";
            data_file << "    \"y\": []\n";
        }
        data_file << "  },\n";
        
        // 保存规划轨迹数据
        data_file << "  \"planned_trajectory\": {\n";
        if (!solution.ego_trj.states.empty()) {
            data_file << "    \"x\": [";
            for (size_t i = 0; i < solution.ego_trj.states.size(); ++i) {
                data_file << solution.ego_trj.states[i][0];
                if (i < solution.ego_trj.states.size() - 1) data_file << ", ";
            }
            data_file << "],\n";
            
            data_file << "    \"y\": [";
            for (size_t i = 0; i < solution.ego_trj.states.size(); ++i) {
                data_file << solution.ego_trj.states[i][1];
                if (i < solution.ego_trj.states.size() - 1) data_file << ", ";
            }
            data_file << "]\n";
        } else {
            data_file << "    \"x\": [],\n";
            data_file << "    \"y\": []\n";
        }
        data_file << "  },\n";
        
        // 保存车辆状态数据
        data_file << "  \"ego_vehicle\": {\n";
        if (!ego_log.empty() && !ego_log[0].empty()) {
            double x = ego_log[0].back();
            double y = ego_log[1].back();
            double theta = ego_log[2].back();
            double gamma = ego_log[3].back();
            
            data_file << "    \"x\": " << x << ",\n";
            data_file << "    \"y\": " << y << ",\n";
            data_file << "    \"theta\": " << theta << ",\n";
            data_file << "    \"gamma\": " << gamma << "\n";
        } else {
            data_file << "    \"x\": 0,\n";
            data_file << "    \"y\": 0,\n";
            data_file << "    \"theta\": 0,\n";
            data_file << "    \"gamma\": 0\n";
        }
        data_file << "  },\n";
        
        // 保存障碍物数据
        data_file << "  \"obstacle\": {\n";
        if (!obs_traj.states.empty()) {
            data_file << "    \"x\": " << obs_traj.states[0][0] << ",\n";
            data_file << "    \"y\": " << obs_traj.states[0][1] << ",\n";
            data_file << "    \"theta\": " << obs_traj.states[0][2] << ",\n";
            data_file << "    \"length\": " << arg.obs_length << ",\n";
            data_file << "    \"width\": " << arg.obs_width << "\n";
        } else {
            data_file << "    \"x\": 0,\n";
            data_file << "    \"y\": 0,\n";
            data_file << "    \"theta\": 0,\n";
            data_file << "    \"length\": " << arg.obs_length << ",\n";
            data_file << "    \"width\": " << arg.obs_width << "\n";
        }
        data_file << "  },\n";
        
        // 保存车辆模型参数
        data_file << "  \"vehicle_model\": {\n";
        data_file << "    \"lf\": " << vehicle_model.lf << ",\n";
        data_file << "    \"lr\": " << vehicle_model.lr << ",\n";
        data_file << "    \"len\": " << vehicle_model.len << ",\n";
        data_file << "    \"width\": " << vehicle_model.width << ",\n";
        data_file << "    \"box_length\": " << vehicle_model.box_length << "\n";
        data_file << "  }\n";
        
        // 移除地图数据保存逻辑，地图数据现在单独保存到maps目录
        
        data_file << "}\n";
        data_file.close();
        
        std::cout << "Data saved to: " << data_filename << std::endl;
    } else {
        std::cout << "Error: Could not open file " << data_filename << " for writing" << std::endl;
    }
}