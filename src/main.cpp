#include <iostream>
#include "ilqr.h"
#include "utils.h"
#include <ctime>

#ifdef _WIN32
#include <windows.h>
#endif

int main(int argc, char** argv){
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif
    std::cout << "=== CILQR Program Starting ===" << std::endl;
    std::cout << "Initializing variables..." << std::endl;
    
    // Initialize variables
    std::vector<Point> way_points;
    std::vector<std::vector<double>> global_plan_log(3), ego_log(4);
    
    std::cout << "Variables initialized successfully." << std::endl;

    // 起点/终点变量（支持从命令行参数获取：start_x start_y start_theta goal_x goal_y goal_theta）
    double start_x = 70, start_y = 60, start_theta = 1.2;
    double goal_x = 180, goal_y = 127, goal_theta = 1.8;
    double ITER = 280;
    std::string solver_type = "cilqr";
    // 地图选择 - 修改这里来切换不同的地图
    // B201 -- B210
    // B301 -- B310
    std::string selected_map = "B301";  // 修改这里来选择不同的地图
    if (argc >= 7) {
        try {
            start_x = std::stod(argv[1]);
            start_y = std::stod(argv[2]);
            start_theta = std::stod(argv[3]);
            goal_x = std::stod(argv[4]);
            goal_y = std::stod(argv[5]);
            goal_theta = std::stod(argv[6]);
        } catch (...) {
            std::cerr << "Invalid start/goal input; using defaults." << std::endl;
        }
    }
    for(int ai=7; ai<argc; ++ai){
        std::string a = argv[ai];
        if(a == "--solver" && ai+1 < argc){
            solver_type = argv[ai+1];
            ++ai;
        }
    }
    std::cout << "Start: (" << start_x << ", " << start_y << ", " << start_theta << ") | "
              << "Goal: (" << goal_x << ", " << goal_y << ", " << goal_theta << ")" << std::endl;
    

    std::string map_file = resolve_resource_path("Maps/bitmap/" + selected_map + "_global_map.json");
    std::cout << "Loading map file: " << map_file << " (Selected: " << selected_map << ")" << std::endl;
    
    // Load bitmap map data
    MapData bitmap_map = load_bitmap_map(map_file);
    std::cout << "Bitmap map loaded: " << bitmap_map.width << " x " << bitmap_map.height << std::endl;
    std::cout << "Map data size: " << bitmap_map.data.size() << std::endl;
    std::cout << "Map resolution: " << bitmap_map.resolution << std::endl;
    std::cout << "Map max_elevation: " << bitmap_map.max_elevation << std::endl;
    if (!bitmap_map.data.empty()) {
        std::cout << "First row size: " << bitmap_map.data[0].size() << std::endl;
        // 暂时注释掉静态地图生成，专注于动态显示
        // draw_bitmap_debug(bitmap_map, "test_map_output.png");
    }
    
    // 保存地图数据到单独的文件（只在程序开始时保存一次）
    std::cout << "About to save map data..." << std::endl;
    save_map_data(&bitmap_map);
    std::cout << "Map data save operation completed." << std::endl;
    
    // Load semantic map data（保留原有函数，使用起点变量）
    // std::vector<std::vector<double>> m_map_info = load_map(start_x, start_y, start_theta);
    // std::cout << "Semantic map loaded. Size: " << m_map_info.size() << std::endl;
    // 改为在规划完成后再填充 m_map_info 接口
    std::vector<std::vector<double>> m_map_info(3);
    std::cout << "Initialized m_map_info interface with 3 channels (x,y,heading)." << std::endl;

    //填充路点（移至规划完成后，从 global_plan 写入）
    // for(int i=0;i<1000;i++){
    //     Point point(m_map_info[0][i],m_map_info[1][i],m_map_info[2][i]);
    //     way_points.push_back(point);
    //     global_plan_log[0].push_back(m_map_info[0][i]);
    //     global_plan_log[1].push_back(m_map_info[1][i]);
    //     global_plan_log[2].push_back(m_map_info[2][i]);
    // }

    // 设置全局路径（通过规划包装器，默认启用RRT*，失败回退到直线路径）
    GlobalPlan global_plan;
    bool enable_planning = true; // 规划开关默认开启
    Eigen::Vector3d start(start_x, start_y, start_theta);
    Eigen::Vector3d goal(goal_x, goal_y, goal_theta);
    if (!plan_global_path(bitmap_map, start, goal, global_plan, enable_planning)) {
        std::cerr << "plan_global_path failed, fallback to load_map" << std::endl;
        auto fallback_info = load_map(start_x, start_y, start_theta);
        std::vector<Point> fallback_points;
        fallback_points.reserve(fallback_info[0].size());
        for (size_t i = 0; i < fallback_info[0].size(); ++i) {
            fallback_points.emplace_back(fallback_info[0][i], fallback_info[1][i], fallback_info[2][i]);
        }
        global_plan.set_plan(fallback_points);
    }

    // 将全局路径结果通过接口传递给 m_map_info，并同步日志
    {
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
        // 同步到可视化日志
        global_plan_log[0] = m_map_info[0];
        global_plan_log[1] = m_map_info[1];
        global_plan_log[2] = m_map_info[2];
        std::cout << "Global path planned. m_map_info points: " << planned_points.size() << std::endl;
        // 导出 m_map_info 供外部可视化使用
        save_m_map_info(m_map_info);
    }

    //参数初始化
    Arg arg;
    // 结合铰接车模型推导保守曲率/半径限值，并映射到铰接角约束（占位映射）
    SystemModel tmp_model; // 用于尺寸参数
    ArticulatedLimits lims = compute_articulated_limits(tmp_model, 0.35); // 保守机械上限 0.35rad
    
    // 设置铰接角gamma的barrier约束
    arg.if_cal_gamma_barrier = true;
    arg.gamma_max = 1.0;  // 铰接角上限
    arg.gamma_min = -1.0; // 铰接角下限
    
    // 设置铰接角速度gamma_dot的barrier约束
    arg.if_cal_gamma_dot_barrier = true;
    // 将曲率限值粗略映射到铰接角速度上下限：omega ≈ v * kappa_max，取期望速度的 0.5 倍作为保守因子
    double omega_bound = std::max(0.1, 0.5 * arg.desire_speed * lims.kappa_max);
    omega_bound = std::min(omega_bound, 0.4); // 上限再加一道保守夹紧
    arg.gamma_dot_max = omega_bound;
    arg.gamma_dot_min = -omega_bound;

    //车辆模型初始化
    Vehicle ego;
    ego.set_state(start_x, start_y, start_theta, 0);
    ego.set_global_plan(global_plan);
    ego.set_model(SystemModel(arg.dt,arg.N));
    for(int i=0;i<4;i++){
        ego_log[i].push_back(ego.get_state()[i]);
    }
    
    OccupancyGrid grid = make_occupancy_grid(bitmap_map, 0.1, 0.5);
    const auto& planned_points = global_plan.get_points();
    size_t M = planned_points.size();
    auto P = [&](size_t i){ return Eigen::Vector2d(planned_points[i].x, planned_points[i].y); };
    auto Th = [&](size_t i){ return planned_points[i].heading; };
    size_t idx_turn = 0; double best_curv = 0.0;
    for(size_t i=1;i+1<M;i++){
        Eigen::Vector2d t0 = P(i) - P(i-1);
        Eigen::Vector2d t1 = P(i+1) - P(i);
        double a0 = std::atan2(t0.y(), t0.x());
        double a1 = std::atan2(t1.y(), t1.x());
        double d = std::atan2(std::sin(a1 - a0), std::cos(a1 - a0));
        double c = std::abs(d);
        if(c > best_curv){ best_curv = c; idx_turn = i; }
    }
    double th_turn = Th(idx_turn);
    Eigen::Vector2d n_turn(-std::sin(th_turn), std::cos(th_turn));
    Eigen::Vector2d p_turn = P(idx_turn);
    double dpt = nearest_obstacle_distance_world(grid, p_turn + 0.5 * n_turn);
    double dmt = nearest_obstacle_distance_world(grid, p_turn - 0.5 * n_turn);
    Eigen::Vector2d n_turn_side = (dpt < dmt) ? n_turn : -n_turn;
    double offset_turn = 3.0;
    State obs1(p_turn.x() + offset_turn * n_turn_side.x(), p_turn.y() + offset_turn * n_turn_side.y(), th_turn, 0.0);
    size_t idx_bottle = 0; double best_clear = 1e9;
    for(size_t i=0;i<M;i++){
        double clr = nearest_obstacle_distance_world(grid, P(i));
        if(clr < best_clear){ best_clear = clr; idx_bottle = i; }
    }
    double th_b = Th(idx_bottle);
    Eigen::Vector2d n_b(-std::sin(th_b), std::cos(th_b));
    Eigen::Vector2d p_b = P(idx_bottle);
    double dpb = nearest_obstacle_distance_world(grid, p_b + 0.5 * n_b);
    double dmb = nearest_obstacle_distance_world(grid, p_b - 0.5 * n_b);
    Eigen::Vector2d n_b_side = (dpb < dmb) ? n_b : -n_b;
    double offset_b = std::max(1.0, best_clear * 0.8);
    State obs2(p_b.x() + offset_b * n_b_side.x(), p_b.y() + offset_b * n_b_side.y(), th_b, 0.0);
    std::vector<State> obs_initial_states = { obs1, obs2 };
    
    std::vector<Trajectory> obs_trajectories;
    std::vector<State> current_obs_states;
    
    // 为每个障碍物预测轨迹
    for(const auto& obs_state : obs_initial_states) {
        Trajectory obs_trj = predict_obstacle_trajectory(obs_state, arg.dt, arg.N);
        // 快速验证：打印前5步预测
        // std::cout << "Predicted obstacle (init) first 5 states:" << std::endl;
        // for (int k = 0; k < std::min(5, (int)obs_trj.states.size()); ++k) {
        //     std::cout << "  k=" << k
        //               << " x=" << obs_trj.states[k][0]
        //               << " y=" << obs_trj.states[k][1]
        //               << " theta=" << obs_trj.states[k][2]
        //               << " v=" << obs_trj.states[k][3] << std::endl;
        // }
        obs_trajectories.push_back(obs_trj);
        current_obs_states.push_back(obs_state);
    }

    //求解器初始化
    CILQRSolver cilqr_solver(ego, obs_trajectories, arg);
    ALILQRSolver alilqr_solver(ego, obs_trajectories, arg);
    Solution solution;
    Control cur_ctrl;
    State cur_state = ego.get_state();


    //主循环
    // for(int i = 0;i<arg.tf/arg.dt;i++){
    for(int i = 0;i<ITER;i++){
        std::cout<<"***** Iter ***** " << i <<std::endl;
        
        // 问题求解
        clock_t start = clock();
        if(solver_type == "al"){
            solution = alilqr_solver.solve(cur_state, obs_trajectories);
        } else {
            solution = cilqr_solver.solve(cur_state, obs_trajectories);
        }
        clock_t end = clock();
        double cpu_time_used = static_cast<double>(end - start) / CLOCKS_PER_SEC;
        std::cout << "CPU time used: " << cpu_time_used * 1000 << " ms\n";
        double cmax = compute_max_violation(solution, ego, obs_trajectories, arg);
        std::cout << "CMax: " << cmax << "\n";

        //更新车辆状态以及控制
        cur_ctrl = solution.control_sequence.controls[0];
        cur_state = ego.get_model().dynamics(cur_state,cur_ctrl);
        //记录车辆历史轨迹
        for(int j=0;j<4;j++){
            ego_log[j].push_back(cur_state[j]);
        }
        
        // 更新所有障碍物状态（使用匀速模型）
        for(size_t obs_idx = 0; obs_idx < current_obs_states.size(); obs_idx++) {
            State& current_obs_state = current_obs_states[obs_idx];
            double obs_x = current_obs_state[0] + current_obs_state[3] * cos(current_obs_state[2]) * arg.dt;
            double obs_y = current_obs_state[1] + current_obs_state[3] * sin(current_obs_state[2]) * arg.dt;
            double obs_theta = current_obs_state[2];  // 朝向保持不变
            double obs_v = current_obs_state[3];      // 速度保持不变
            current_obs_state = State(obs_x, obs_y, obs_theta, obs_v);
            // std::cout << "Obstacle " << obs_idx + 1 << " updated state: x=" << obs_x << ", y=" << obs_y << ", theta=" << obs_theta << ", v=" << obs_v << std::endl;
            
            // // 重新预测该障碍物的轨迹
            obs_trajectories[obs_idx] = predict_obstacle_trajectory(current_obs_state, arg.dt, arg.N);
            // // 快速验证：每帧打印第一个障碍物前3步预测
            // if (obs_idx == 0) {
            //     std::cout << "Predicted obstacle (frame) first 3 states:" << std::endl;
            //     for (int k = 0; k < std::min(3, (int)obs_trajectories[obs_idx].states.size()); ++k) {
            //         std::cout << "  k=" << k
            //                   << " x=" << obs_trajectories[obs_idx].states[k][0]
            //                   << " y=" << obs_trajectories[obs_idx].states[k][1]
            //                   << " theta=" << obs_trajectories[obs_idx].states[k][2]
            //                   << " v=" << obs_trajectories[obs_idx].states[k][3] << std::endl;
            //     }
            // }
        }

        // std::cout<<"Vehicle state:"<<std::endl;
        // std::cout<<"x   :  "<<cur_state[0]<<std::endl;
        // std::cout<<"y   :   "<<cur_state[1]<<std::endl;
        // std::cout<<"theta   :   "<<cur_state[2]<<std::endl;
        // std::cout<<"gamma   :   "<<cur_state[3]<<std::endl;
        //  std::cout<<"Control:"<<std::endl;
        //  std::cout<<"v  :   "<< cur_ctrl[0]<<std::endl;
        //  std::cout<<"omega  :   "<< cur_ctrl[1]<<std::endl;
        // if(i%3==0){
            dynamic_plot(global_plan_log,ego_log,obs_trajectories,solution,&bitmap_map,global_plan,ego.get_model(),arg);
        // }
 
    }

    // mclTerminateApplication();
    return 0;
}
