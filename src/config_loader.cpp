#include "config_loader.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>

// Helper functions for manual JSON parsing
namespace {

std::string remove_comments_and_whitespace(const std::string& json) {
    std::string result;
    bool in_string = false;
    for (size_t i = 0; i < json.length(); ++i) {
        char c = json[i];
        if (c == '"') in_string = !in_string;
        if (!in_string && (c == ' ' || c == '\n' || c == '\r' || c == '\t')) continue;
        result += c;
    }
    return result;
}

// Find the content of a section: "key": { ... }
std::string get_section(const std::string& json, const std::string& key) {
    std::string search_key = "\"" + key + "\":{";
    size_t start_pos = json.find(search_key);
    if (start_pos == std::string::npos) return "";
    
    start_pos += search_key.length();
    int brace_count = 1;
    size_t end_pos = start_pos;
    while (end_pos < json.length() && brace_count > 0) {
        if (json[end_pos] == '{') brace_count++;
        else if (json[end_pos] == '}') brace_count--;
        end_pos++;
    }
    
    if (brace_count == 0) {
        return json.substr(start_pos, end_pos - start_pos - 1);
    }
    return "";
}

// Get a value string: "key": value
std::string get_value_str(const std::string& json, const std::string& key) {
    std::string search_key = "\"" + key + "\":";
    size_t start_pos = json.find(search_key);
    if (start_pos == std::string::npos) return "";
    
    start_pos += search_key.length();
    size_t end_pos = start_pos;
    while (end_pos < json.length() && json[end_pos] != ',' && json[end_pos] != '}') {
        end_pos++;
    }
    return json.substr(start_pos, end_pos - start_pos);
}

double get_double(const std::string& json, const std::string& key, double default_val) {
    std::string val_str = get_value_str(json, key);
    if (val_str.empty()) return default_val;
    try {
        return std::stod(val_str);
    } catch (...) {
        return default_val;
    }
}

int get_int(const std::string& json, const std::string& key, int default_val) {
    std::string val_str = get_value_str(json, key);
    if (val_str.empty()) return default_val;
    try {
        return std::stoi(val_str);
    } catch (...) {
        return default_val;
    }
}

bool get_bool(const std::string& json, const std::string& key, bool default_val) {
    std::string val_str = get_value_str(json, key);
    if (val_str.empty()) return default_val;
    return (val_str == "true");
}

std::vector<double> get_vector(const std::string& json, const std::string& key) {
    std::vector<double> result;
    std::string search_key = "\"" + key + "\":[";
    size_t start_pos = json.find(search_key);
    if (start_pos == std::string::npos) return result;
    
    start_pos += search_key.length();
    size_t end_pos = json.find("]", start_pos);
    if (end_pos == std::string::npos) return result;
    
    std::string content = json.substr(start_pos, end_pos - start_pos);
    std::stringstream ss(content);
    std::string item;
    while (std::getline(ss, item, ',')) {
        try {
            result.push_back(std::stod(item));
        } catch (...) {}
    }
    return result;
}

} // namespace

// Helper to load a single file content
std::string load_file_content(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Warning: Could not open config file " << filename << ". Using default parameters." << std::endl;
        return "";
    }
    return std::string((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
}

void load_config(const std::string& main_config_file, 
                 const std::string& ilqr_config_file,
                 const std::string& ha_config_file,
                 Arg& arg, 
                 SystemModel& system_model, 
                 HybridAStarParams& ha_params,
                 RunConfig& run_config) {
    
    // 1. Load Main Config
    std::string main_json = remove_comments_and_whitespace(load_file_content(main_config_file));
    if (!main_json.empty()) {
        // Read selected_map first
        std::string sm = get_value_str(main_json, "selected_map");
        if (!sm.empty()) {
             if (sm.front() == '"') sm = sm.substr(1, sm.length() - 2);
             run_config.selected_map = sm;
        }

        // RunConfigs
        std::string rcs = get_section(main_json, "run_configs");
        if (!rcs.empty()) {
            // Try to get config for selected_map
            std::string rc = get_section(rcs, run_config.selected_map);
            if (rc.empty()) {
                std::cerr << "Warning: Config for map " << run_config.selected_map << " not found. Using default." << std::endl;
                rc = get_section(rcs, "default");
            }

            if (!rc.empty()) {
                run_config.start_x = get_double(rc, "start_x", run_config.start_x);
                run_config.start_y = get_double(rc, "start_y", run_config.start_y);
                run_config.start_theta = get_double(rc, "start_theta", run_config.start_theta);
                run_config.goal_x = get_double(rc, "goal_x", run_config.goal_x);
                run_config.goal_y = get_double(rc, "goal_y", run_config.goal_y);
                run_config.goal_theta = get_double(rc, "goal_theta", run_config.goal_theta);
                run_config.ITER = get_double(rc, "ITER", run_config.ITER);
                run_config.obstacle_speed = get_double(rc, "obstacle_speed", run_config.obstacle_speed);
                run_config.obstacle_count = get_int(rc, "obstacle_count", run_config.obstacle_count);
                run_config.obstacle_distance = get_double(rc, "obstacle_distance", run_config.obstacle_distance);
                
                std::string st = get_value_str(rc, "solver_type");
                if (!st.empty()) {
                     if (st.front() == '"') st = st.substr(1, st.length() - 2);
                     run_config.solver_type = st;
                }
            }
        }

        // Simulation
        std::string sim = get_section(main_json, "simulation");
        if (!sim.empty()) {
            arg.tf = get_double(sim, "tf", arg.tf);
            double dt = get_double(sim, "dt", arg.dt);
            arg.dt = dt;
            system_model.dt = dt;
        }

        // Vehicle
        std::string v = get_section(main_json, "vehicle");
        if (!v.empty()) {
            system_model.ego_rad = get_double(v, "ego_rad", system_model.ego_rad);
            system_model.lf = get_double(v, "lf", system_model.lf);
            system_model.lr = get_double(v, "lr", system_model.lr);
            system_model.width = get_double(v, "width", system_model.width);
            system_model.box_length = get_double(v, "box_length", system_model.box_length);
        }
    }

    // 2. Load ILQR Config
    std::string ilqr_json = remove_comments_and_whitespace(load_file_content(ilqr_config_file));
    if (!ilqr_json.empty()) {
        // CILQR
        std::string cilqr = get_section(ilqr_json, "cilqr");
        if (!cilqr.empty()) {
            int N = get_int(cilqr, "N", arg.N);
            arg.N = N;
            system_model.N = N;
            arg.tol = get_double(cilqr, "tol", arg.tol);
            arg.rel_tol = get_double(cilqr, "rel_tol", arg.rel_tol);
            arg.max_iter = get_int(cilqr, "max_iter", arg.max_iter);
            arg.lamb_init = get_double(cilqr, "lamb_init", arg.lamb_init);
            arg.lamb_factor = get_double(cilqr, "lamb_factor", arg.lamb_factor);
            arg.lamb_max = get_double(cilqr, "lamb_max", arg.lamb_max);
        }

        // Pure Pursuit
        std::string pp = get_section(ilqr_json, "pure_pursuit");
        if (!pp.empty()) {
            arg.kv = get_double(pp, "kv", arg.kv);
            arg.kp = get_double(pp, "kp", arg.kp);
            arg.ld0 = get_double(pp, "ld0", arg.ld0);
            arg.ld_min = get_double(pp, "ld_min", arg.ld_min);
            arg.ld_max = get_double(pp, "ld_max", arg.ld_max);
        }

        // Cost Weights
        std::string cw = get_section(ilqr_json, "cost_weights");
        if (!cw.empty()) {
            arg.desire_speed = get_double(cw, "desire_speed", arg.desire_speed);
            arg.desire_heading = get_double(cw, "desire_heading", arg.desire_heading);
            arg.v_rate_weight = get_double(cw, "v_rate_weight", arg.v_rate_weight);
            arg.ref_weight = get_double(cw, "ref_weight", arg.ref_weight);
            
            std::vector<double> Q_vec = get_vector(cw, "Q");
            if (Q_vec.size() == 4) {
                arg.Q.setZero();
                arg.Q(0,0) = Q_vec[0];
                arg.Q(1,1) = Q_vec[1];
                arg.Q(2,2) = Q_vec[2];
                arg.Q(3,3) = Q_vec[3];
            }
            std::vector<double> R_vec = get_vector(cw, "R");
            if (R_vec.size() == 2) {
                arg.R.setZero();
                arg.R(0,0) = R_vec[0];
                arg.R(1,1) = R_vec[1];
            }
        }

        // Constraints
        std::string c = get_section(ilqr_json, "constraints");
        if (!c.empty()) {
            arg.if_cal_obs_cost = get_bool(c, "if_cal_obs_cost", arg.if_cal_obs_cost);
            arg.if_cal_lane_cost = get_bool(c, "if_cal_lane_cost", arg.if_cal_lane_cost);
            arg.if_cal_speed_rate_cost = get_bool(c, "if_cal_speed_rate_cost", arg.if_cal_speed_rate_cost);
            
            arg.if_cal_gamma_barrier = get_bool(c, "if_cal_gamma_barrier", arg.if_cal_gamma_barrier);
            arg.gamma_max = get_double(c, "gamma_max", arg.gamma_max);
            arg.gamma_max_q1 = get_double(c, "gamma_max_q1", arg.gamma_max_q1);
            arg.gamma_max_q2 = get_double(c, "gamma_max_q2", arg.gamma_max_q2);
            arg.gamma_min = get_double(c, "gamma_min", arg.gamma_min);
            arg.gamma_min_q1 = get_double(c, "gamma_min_q1", arg.gamma_min_q1);
            arg.gamma_min_q2 = get_double(c, "gamma_min_q2", arg.gamma_min_q2);
            
            arg.if_cal_gamma_dot_barrier = get_bool(c, "if_cal_gamma_dot_barrier", arg.if_cal_gamma_dot_barrier);
            arg.gamma_dot_max = get_double(c, "gamma_dot_max", arg.gamma_dot_max);
            arg.gamma_dot_max_q1 = get_double(c, "gamma_dot_max_q1", arg.gamma_dot_max_q1);
            arg.gamma_dot_max_q2 = get_double(c, "gamma_dot_max_q2", arg.gamma_dot_max_q2);
            arg.gamma_dot_min = get_double(c, "gamma_dot_min", arg.gamma_dot_min);
            arg.gamma_dot_min_q1 = get_double(c, "gamma_dot_min_q1", arg.gamma_dot_min_q1);
            arg.gamma_dot_min_q2 = get_double(c, "gamma_dot_min_q2", arg.gamma_dot_min_q2);
            
            arg.acc_max = get_double(c, "acc_max", arg.acc_max);
            arg.acc_min = get_double(c, "acc_min", arg.acc_min);
            arg.acc_q1 = get_double(c, "acc_q1", arg.acc_q1);
            arg.acc_q2 = get_double(c, "acc_q2", arg.acc_q2);
            
            arg.trace_safe_width_left = get_double(c, "trace_safe_width_left", arg.trace_safe_width_left);
            arg.trace_safe_width_right = get_double(c, "trace_safe_width_right", arg.trace_safe_width_right);
            arg.lane_q1 = get_double(c, "lane_q1", arg.lane_q1);
            arg.lane_q2 = get_double(c, "lane_q2", arg.lane_q2);
            
            arg.obs_q1 = get_double(c, "obs_q1", arg.obs_q1);
            arg.obs_q2 = get_double(c, "obs_q2", arg.obs_q2);
            arg.obs_length = get_double(c, "obs_length", arg.obs_length);
            arg.obs_width = get_double(c, "obs_width", arg.obs_width);
            arg.safe_a_buffer = get_double(c, "safe_a_buffer", arg.safe_a_buffer);
            arg.safe_b_buffer = get_double(c, "safe_b_buffer", arg.safe_b_buffer);
        }
    }

    // 3. Load Hybrid A* Config
    std::string ha_json = remove_comments_and_whitespace(load_file_content(ha_config_file));
    if (!ha_json.empty()) {
        std::string ha = get_section(ha_json, "hybrid_astar");
        if (!ha.empty()) {
            ha_params.grid_resolution = get_double(ha, "grid_resolution", ha_params.grid_resolution);
            ha_params.heading_resolution = get_double(ha, "heading_resolution", ha_params.heading_resolution);
            ha_params.turning_radius = get_double(ha, "turning_radius", ha_params.turning_radius);
            ha_params.move_step = get_double(ha, "move_step", ha_params.move_step);
            ha_params.move_step_backwards = get_double(ha, "move_step_backwards", ha_params.move_step_backwards);
            ha_params.allow_reverse = get_bool(ha, "allow_reverse", ha_params.allow_reverse);
            ha_params.num_gamma_angles = get_int(ha, "num_gamma_angles", ha_params.num_gamma_angles);
            ha_params.steering_penalty = get_double(ha, "steering_penalty", ha_params.steering_penalty);
            ha_params.direction_change_penalty = get_double(ha, "direction_change_penalty", ha_params.direction_change_penalty);
            ha_params.backwards_penalty = get_double(ha, "backwards_penalty", ha_params.backwards_penalty);
            ha_params.heuristic_weight = get_double(ha, "heuristic_weight", ha_params.heuristic_weight);
            ha_params.max_iterations = get_int(ha, "max_iterations", ha_params.max_iterations);
            ha_params.num_nodes_to_keep = get_int(ha, "num_nodes_to_keep", ha_params.num_nodes_to_keep);
            ha_params.inflation_radius = get_double(ha, "inflation_radius", ha_params.inflation_radius);
            ha_params.goal_tolerance_xy = get_double(ha, "goal_tolerance_xy", ha_params.goal_tolerance_xy);
            ha_params.goal_tolerance_heading = get_double(ha, "goal_tolerance_heading", ha_params.goal_tolerance_heading);
            ha_params.clearance_weight = get_double(ha, "clearance_weight", ha_params.clearance_weight);
            ha_params.desired_clearance = get_double(ha, "desired_clearance", ha_params.desired_clearance);
            ha_params.max_clearance = get_double(ha, "max_clearance", ha_params.max_clearance);
            ha_params.min_rs_clearance = get_double(ha, "min_rs_clearance", ha_params.min_rs_clearance);
        }
    }
    
    std::cout << "Configuration loaded from config/main.json, config/ilqr.json, config/hybrid_astar.json" << std::endl;
}
