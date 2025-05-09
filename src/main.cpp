#include <iostream>
#include <vector>
#include <ctime>
#include "ilqr.h"
#include "utils.h"

enum Scenario {
// 在ilqr.h中自定义场景
    CUSTOM,            
// 动态避障         
    DYNAMIC_COLLISION_AVOID,  
// 密集静态障碍物避障 
    DENSE_STATIC_OBSTACLE, 
// 窄通道通行    
    NARROW_CORRIDOR,      
// 跟车行驶     
    FOLLOWING                   
};

// 设置默认车辆和仿真参数
void set_default_params(Arg& arg) {
    arg.is_following = false;
    arg.following_distance = 20;
    // 车辆参数
    arg.ego_circle_rad = 1.46;
    arg.front_circle_center_to_ego_center = 1.46;
    arg.rear_circle_center_to_ego_center = -1.46;
    arg.lf = 1.45;
    arg.lr = 1.45;
    arg.len = 4.8;
    arg.width = 2.25;

    // 仿真参数
    arg.tf = 1000;
    arg.dt = 0.1;

    // CILQR参数
    arg.N = 80;
    arg.tol = 1e-3;
    arg.max_iter = 5000;
    arg.lamb_init = 1;
    arg.lamb_factor = 2;
    arg.lamb_max = 1000;

    // 纯跟踪参数
    arg.kv = 0.3;
    arg.kp = 0.8;
    arg.ld0 = 3;
    arg.ld_min = 3;
    arg.ld_max = 20;

    // 代价参数
    arg.desire_speed = 5;
    arg.desire_heading = 0;
    arg.if_cal_obs_cost = true;
    arg.if_cal_lane_cost = false;
    arg.if_cal_steer_cost = true;

    // 转向约束
    arg.steer_angle_max = 0.698; // 最大转向角：40°
    arg.steer_max_q1 = 3.87298334620741657730036422435659915208816528320312;
    arg.steer_max_q2 = 775.7992361635523366203415207564830780029296875;
    arg.steer_angle_min = -0.698; // 最小转向角：-40°
    arg.steer_min_q1 = 3.87298334620741657730036422435659915208816528320312;
    arg.steer_min_q2 = 775.7992361635523366203415207564830780029296875;

    // 道路约束
    arg.trace_safe_width_left = 4;
    arg.trace_safe_width_right = 4;
    arg.lane_q1 = 100;
    arg.lane_q2 = 14;

    // 障碍约束
    arg.obs_q1 = 25.290355;
    arg.obs_q2 = 10.447458;
    arg.obs_length = 2;
    arg.obs_width = 1;
    arg.safe_a_buffer = 0.5;
    arg.safe_b_buffer = 0.25;

    // 横向偏移代价       
    arg.ref_weight = 3.0;
    arg.Q << 0, 0, 0, 0,
             0, 0, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
    arg.R << 1, 0,
             0, 200;
}

// 场景配置结构体
struct ScenarioConfig {
    std::vector<State> init_states;
    Arg arg;
};

// 配置场景
ScenarioConfig configure_scenario(Scenario type) {
    ScenarioConfig config;
    set_default_params(config.arg);

    switch (type) {
        case DYNAMIC_COLLISION_AVOID:
            config.init_states = {
                {80, 0.5, 0, 2}, {50, 2, 0, 1}, {70, -2, 0, 1},
                {50, 2, 0, 0}, {70, -2, 0, 0}, {90, 2, 0, 5},
                {110, -2, 0, 2}, {30, 2, -M_PI/2, 1}, {70, -2, M_PI/2, 0.5}
            };
            config.arg.desire_speed = 10;
            break;

        case DENSE_STATIC_OBSTACLE:
            config.init_states = {
                {30, 1, 0, 0}, {45, 2, 0, 0}, {60, -2, 0, 0},
                {70, 2, 0, 0}, {90, -2, 0, 0}, {95, 3, 0, 0},
                {100, 2, -M_PI/4, 0}, {110, -2, M_PI/4, 0},
                {125,2,0,0},{130, 2, 0, 0}, {135,2,0,0},{140, 2, 0, 0}, {145,2,0,0},
                {125,-6,0,0},{130, -6, 0, 0}, {135,-6,0,0},{140, -6, 0, 0}, {145,-6,0,0}
            };
            config.arg.N = 80;
            config.arg.desire_speed = 10;
            break;

        case NARROW_CORRIDOR:
            config.init_states = {
                {50, 3.5, 0, 0}, {50, -3.5, 0, 0}, {70, 2, -M_PI/32, 0},
                {70, -4, -M_PI/32, 0}, {90, 2.5, M_PI/32, 0}, {90, -4, M_PI/32, 0},
                {110, 4, M_PI/64, 0}, {110, -2.5, M_PI/50, 0}
            };
            config.arg.obs_length = 20;
            config.arg.obs_width = 2.5;
            config.arg.safe_b_buffer = 0.0;
            config.arg.ref_weight = 0.0;
            break;

        case FOLLOWING:
            config.init_states = {{30, 0, 0, 5}};
            config.arg.is_following = true;
            config.arg.following_distance = 10;
            config.arg.Q << 1, 0, 0, 0,
                                            0, 1, 0, 0,
                                            0, 0, 1, 0,
                                            0, 0, 0, 1;
            break;
	case CUSTOM:
	    config.arg.obs_length = 2;
	    config.arg.obs_width = 100;
	    config.init_states = {{50,0, 0, 0}};

	    break;

        default:
            config.init_states = {{50, 0.5, 0, 1}};
            break;
    }
    return config;
}

// 初始化障碍物轨迹
void init_obstacle_trajectories(std::vector<Trajectory>& trajectories, int horizon, const std::vector<State>& init_states) {
    trajectories.resize(init_states.size());
    for (size_t i = 0; i < init_states.size(); ++i) {
        trajectories[i].states.push_back(init_states[i]);
    }
}

// 障碍物动态模型
State obstacle_dynamics(const State& state, const Control& control) {
    const double lr = 1.13, lf = 1.6, len = lr + lf, dt = 0.1;
    double beta = atan((lr / len) * tan(control[1]));
    State next_state;
    next_state << 
        state[0] + state[3] * cos(state[2] + beta) * dt,
        state[1] + state[3] * sin(state[2] + beta) * dt,
        state[2] + (state[3] / len) * tan(control[1]) * cos(beta) * dt,
        state[3] + control[0] * dt;
    next_state(2) = angle_wrap(next_state(2));
    return next_state;
}

// 更新障碍物轨迹
void update_obstacle_trajectories(std::vector<Trajectory>& trajectories, int horizon) {
    if (trajectories.empty()) return;
    Control control = {0, 0};
    for (auto& traj : trajectories) {
        State state = traj.get_states()[0];
        traj.states.clear();
        for (int j = 0; j <= horizon; ++j) {
            state = obstacle_dynamics(state, control);
            traj.push_back(state);
        }
    }
}

// 主函数
int main() {
    Scenario scenario_type = Scenario::DYNAMIC_COLLISION_AVOID; // 可选场景

    // 初始化参数和障碍物
    ScenarioConfig config = configure_scenario(scenario_type);
    std::vector<Trajectory> obstacle_trajectories;
    init_obstacle_trajectories(obstacle_trajectories, config.arg.N, config.init_states);

    // 加载地图并设置全局路径
    std::vector<Point> waypoints;
    std::vector<std::vector<double>> global_plan_log(3), ego_history_trajectory_log(4);
    auto map_info = load_map();
    for (size_t i = 0; i < map_info[0].size(); ++i) {
        double x = map_info[0][i];
        double y = scenario_type == FOLLOWING ? 0 : map_info[1][i];
        double heading = scenario_type == FOLLOWING ? 0 : map_info[2][i];
        // y = 0;
        // heading = 0;
        waypoints.emplace_back(x, y, heading);
        global_plan_log[0].push_back(x);
        global_plan_log[1].push_back(y);
        global_plan_log[2].push_back(heading);
    }
    GlobalPlan global_plan;
    global_plan.set_plan(waypoints);

    // 初始化车辆模型
    Vehicle ego;
    ego.set_state(map_info[0][0], map_info[1][0], map_info[2][0], 1);
    ego.set_global_plan(global_plan);
    ego.set_model(SystemModel(config.arg));
    for (int i = 0; i < 4; ++i) {
        ego_history_trajectory_log[i].push_back(ego.get_state()[i]);
    }

    // 初始化求解器
    CILQRSolver cilqr_solver(ego, config.arg);
    State current_state = ego.get_state();

    // 主循环
    for (int i = 0; i < 2000; ++i) {
        std::cout << "***** Iter " << i << " *****\n";
	
        update_obstacle_trajectories(obstacle_trajectories, config.arg.N);
	
	//对外接口
		// //当前帧障碍物轨迹（可能需要进行预测后处理）
		// obstacle_trajectories = obstacle_trajectories;
		// //当前帧自车状态
		// current_state = current_state;
		// //当前帧全局参考路径
		// cilqr_solver.set_global_plan(waypoints);
	//

        // 求解优化问题
        clock_t start = clock();
        Solution solution = cilqr_solver.solve(current_state, obstacle_trajectories, config.arg.is_following);
        clock_t end = clock();
        std::cout << "CPU time used: " << (static_cast<double>(end - start) / CLOCKS_PER_SEC) * 1000 << " ms\n";

        // 更新车辆状态
        Control current_control = solution.control_sequence.controls[0];
        current_state = ego.get_model().dynamics(current_state, current_control);
        for (int j = 0; j < 4; ++j) {
            ego_history_trajectory_log[j].push_back(current_state[j]);
        }

        // 输出状态并绘图
        std::cout << current_state << "\n";
        my_plot(global_plan_log, ego_history_trajectory_log, obstacle_trajectories, solution, config.arg);
    }

    return 0;
}