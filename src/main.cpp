#include <iostream>
#include "ilqr.h"
#include "utils.h"
#include "config_loader.h"
#include <ctime>

#ifdef _WIN32
#include <windows.h>
#endif

// Configuration structure to hold runtime parameters
// RunConfig is now in common_types.h

// Structure to hold obstacle data
struct ObstacleData {
    std::vector<State> current_states;
    std::vector<Trajectory> trajectories;
};

// Function to parse command line arguments
// Function to parse command line arguments
void parse_arguments(int argc, char** argv, RunConfig& config) {
    for(int ai=1; ai<argc; ++ai){
        std::string a = argv[ai];
        if(a == "--solver" && ai+1 < argc){
            config.solver_type = argv[ai+1];
            ++ai;
        }
    }
}

// Function to load and process the map
MapData load_and_process_map(const std::string& selected_map, const std::string& solver_type) {
    std::string map_file = resolve_resource_path("Maps/bitmap/" + selected_map + "_global_map.json");
    std::cout << "Loading map file: " << map_file << " (Selected: " << selected_map << ")" << std::endl;
    
    MapData bitmap_map = load_bitmap_map(map_file);
    std::cout << "Bitmap map loaded: " << bitmap_map.width << " x " << bitmap_map.height << std::endl;
    std::cout << "Map data size: " << bitmap_map.data.size() << std::endl;
    std::cout << "Map resolution: " << bitmap_map.resolution << std::endl;
    std::cout << "Map max_elevation: " << bitmap_map.max_elevation << std::endl;
    if (!bitmap_map.data.empty()) {
        std::cout << "First row size: " << bitmap_map.data[0].size() << std::endl;
    }
    
    std::cout << "About to save map data..." << std::endl;
    std::cout << "About to save map data..." << std::endl;
    save_map_data(&bitmap_map, solver_type);
    std::cout << "Map data save operation completed." << std::endl;
    
    return bitmap_map;
}

// Function to perform global planning
GlobalPlan perform_global_planning(const MapData& bitmap_map, const RunConfig& config, std::vector<std::vector<double>>& m_map_info, const HybridAStarParams& ha_params) {
    GlobalPlan global_plan;
    bool enable_planning = true;
    Eigen::Vector3d start(config.start_x, config.start_y, config.start_theta);
    Eigen::Vector3d goal(config.goal_x, config.goal_y, config.goal_theta);
    
    if (!plan_global_path(bitmap_map, start, goal, global_plan, ha_params, enable_planning)) {
        std::cerr << "plan_global_path failed, fallback to load_map" << std::endl;
        auto fallback_info = load_map(config.start_x, config.start_y, config.start_theta);
        std::vector<Point> fallback_points;
        fallback_points.reserve(fallback_info[0].size());
        for (size_t i = 0; i < fallback_info[0].size(); ++i) {
            fallback_points.emplace_back(fallback_info[0][i], fallback_info[1][i], fallback_info[2][i]);
        }
        global_plan.set_plan(fallback_points);
    }

    // Update m_map_info interface
    const auto& planned_points = global_plan.get_points();
    m_map_info[0].clear(); m_map_info[1].clear(); m_map_info[2].clear();
    m_map_info[0].reserve(planned_points.size());
    m_map_info[1].reserve(planned_points.size());
    m_map_info[2].reserve(planned_points.size());
    for (const auto& p : planned_points) {
        m_map_info[0].push_back(p.x);
        m_map_info[1].push_back(p.y);
        m_map_info[2].push_back(p.heading);
    }
    
    std::cout << "Global path planned. m_map_info points: " << planned_points.size() << std::endl;
    std::cout << "Global path planned. m_map_info points: " << planned_points.size() << std::endl;
    save_m_map_info(m_map_info, config.solver_type);
    
    return global_plan;
}

// Function to finalize solver parameters (derived values)
void finalize_parameters(Arg& arg) {
    SystemModel tmp_model;
    ArticulatedLimits lims = compute_articulated_limits(tmp_model, 0.35);
    
    // Derived parameters that depend on calculations
    double omega_bound = std::max(0.1, 0.5 * arg.desire_speed * lims.kappa_max);
    omega_bound = std::min(omega_bound, 0.4);
    
    // Restore derived values as they depend on system model limits
    arg.gamma_dot_max = omega_bound;
    arg.gamma_dot_min = -omega_bound;
    
    std::cout << "Computed articulated limits: kappa_max=" << lims.kappa_max << ", R_min=" << lims.R_min << std::endl;
    std::cout << "Derived gamma_dot limits: [" << arg.gamma_dot_min << ", " << arg.gamma_dot_max << "]" << std::endl;
}

// Function to initialize obstacles
ObstacleData initialize_obstacles(const GlobalPlan& global_plan, const MapData& bitmap_map, const Arg& arg, const RunConfig& run_config) {
    ObstacleData obs_data;
    OccupancyGrid grid = make_occupancy_grid(bitmap_map, 0.1, 0.5);
    std::vector<State> obs_initial_states = generate_obstacles(global_plan, grid, run_config.obstacle_count, run_config.obstacle_distance, run_config.obstacle_speed);
    
    for(const auto& obs_state : obs_initial_states) {
        Trajectory obs_trj = predict_obstacle_trajectory(obs_state, arg.dt, arg.N);
        obs_data.trajectories.push_back(obs_trj);
        obs_data.current_states.push_back(obs_state);
    }
    return obs_data;
}

// Function to update obstacle states
void update_obstacle_states(ObstacleData& obs_data, const Arg& arg) {
    for(size_t obs_idx = 0; obs_idx < obs_data.current_states.size(); obs_idx++) {
        State& current_obs_state = obs_data.current_states[obs_idx];
        double obs_x = current_obs_state[0] + current_obs_state[3] * cos(current_obs_state[2]) * arg.dt;
        double obs_y = current_obs_state[1] + current_obs_state[3] * sin(current_obs_state[2]) * arg.dt;
        double obs_theta = current_obs_state[2];
        double obs_v = current_obs_state[3];
        current_obs_state = State(obs_x, obs_y, obs_theta, obs_v);
        
        obs_data.trajectories[obs_idx] = predict_obstacle_trajectory(current_obs_state, arg.dt, arg.N);
    }
}

int main(int argc, char** argv){
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif
    std::cout << "=== CILQR Program Starting ===" << std::endl;
    std::cout << "Initializing variables..." << std::endl;
    
    // 0. Load Configuration
    Arg arg;
    SystemModel system_model;
    HybridAStarParams ha_params;
    RunConfig run_config;
    // Load parameters from config files
    load_config("config/main.json", "config/ilqr.json", "config/hybrid_astar.json", arg, system_model, ha_params, run_config);
    finalize_parameters(arg); // Optional: calculate derived values

    // 1. Parse Arguments (Override config)
    parse_arguments(argc, argv, run_config);
    
    std::cout << "======================== solver_type: " << run_config.solver_type << " ========================" <<std::endl;
    std::cout << "Start: (" << run_config.start_x << ", " << run_config.start_y << ", " << run_config.start_theta << ") | "
              << "Goal: (" << run_config.goal_x << ", " << run_config.goal_y << ", " << run_config.goal_theta << ")" << std::endl;

    // 2. Load Map
    // 2. Load Map
    MapData bitmap_map = load_and_process_map(run_config.selected_map, run_config.solver_type);
    
    // 3. Global Planning
    std::vector<std::vector<double>> m_map_info(3);
    std::vector<std::vector<double>> global_plan_log(3);
    GlobalPlan global_plan = perform_global_planning(bitmap_map, run_config, m_map_info, ha_params);
    
    // Sync global plan log
    global_plan_log[0] = m_map_info[0];
    global_plan_log[1] = m_map_info[1];
    global_plan_log[2] = m_map_info[2];

    // 4. Initialize Parameters (Already done via load_config)
    // Arg arg = initialize_parameters(); // Removed

    // 5. Initialize Vehicle
    Vehicle ego;
    ego.set_state(run_config.start_x, run_config.start_y, run_config.start_theta, 0);
    ego.set_global_plan(global_plan);
    ego.set_model(system_model); // Use loaded system model
    
    std::vector<std::vector<double>> ego_log(4);
    for(int i=0;i<4;i++){
        ego_log[i].push_back(ego.get_state()[i]);
    }

    // 6. Initialize Obstacles
    ObstacleData obs_data = initialize_obstacles(global_plan, bitmap_map, arg, run_config);

    // 7. Initialize Solvers
    CILQRSolver cilqr_solver(ego, obs_data.trajectories, arg, "cilqr");
    ALILQRSolver alilqr_solver(ego, obs_data.trajectories, arg);
    Solution solution;
    Control cur_ctrl;
    State cur_state = ego.get_state();

    std::cout << "Variables initialized successfully." << std::endl;

    // 8. Main Loop
    for(int i = 0; i < run_config.ITER; i++){
        std::cout<<"***** Iter ***** " << i <<std::endl;
        
        // Solve
        clock_t start = clock();
        if(run_config.solver_type == "cilqr"){
            solution = cilqr_solver.solve(cur_state, obs_data.trajectories);
        } else if(run_config.solver_type == "alilqr"){
            solution = alilqr_solver.solve(cur_state, obs_data.trajectories);
        } else {
            std::cerr << "Wrong Solver Type :" << run_config.solver_type << std::endl; 
        }
        clock_t end = clock();
        double cpu_time_used = static_cast<double>(end - start) / CLOCKS_PER_SEC;
        std::cout << "CPU time used: " << cpu_time_used * 1000 << " ms\n";
        
        // Validation
        ego.set_state(cur_state);
        ego.set_local_plan();
        double cmax = compute_max_violation(solution, ego, obs_data.trajectories, arg);
        std::cout << "CMax: " << cmax << "\n";

        // Update Vehicle State
        cur_ctrl = solution.control_sequence.controls[0];
        cur_state = ego.get_model().dynamics(cur_state, cur_ctrl);
        
        // Log History
        for(int j=0;j<4;j++){
            ego_log[j].push_back(cur_state[j]);
        }
        
        // Update Obstacles
        update_obstacle_states(obs_data, arg);

        // Visualization
        dynamic_plot(global_plan_log, ego_log, obs_data.trajectories, solution, &bitmap_map, global_plan, ego.get_model(), arg, run_config.solver_type);
    }

    return 0;
}
