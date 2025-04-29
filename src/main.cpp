#include <iostream>
#include "ilqr.h"
#include "utils.h"
#include <ctime>

enum scenario{
    custom,                                         //在ilqr.h里面自定义场景
    dynamic_collision_avoid,    //动态避障
    dense_static_obstacle,         //密集静态障碍物避障
    narror_corridor,                       //窄通道同行
    following                                     //跟车行驶
};

void obs_traj_init(std::vector<Trajectory>& total_obs_traj,int N,std::vector<State> init_states){
    int obs_num = init_states.size();
    total_obs_traj.resize(obs_num);
    for(int i=0;i<obs_num;i++){
        total_obs_traj[i].states.push_back(init_states[i]);
    }
}

State obs_dynamic(const State& X,const Control& U){
        double lr = 1.13;
        double lf = 1.6;
        double len = lr + lf;
        double beta = atan((lr / (lr + lf)) * tan(U[1]));
        double dt = 0.1;
        State X_next;
        X_next << 
        X[0] + X[3] * cos(X[2] + beta) * dt,
        X[1] + X[3] * sin(X[2] + beta) * dt,
        X[2] + (X[3] / len) * tan(U[1]) * cos(beta) * dt,
        X[3] + U[0] * dt;
    X_next(2) = angle_wrap(X_next(2));
    return X_next;
}

void update_obs_traj(std::vector<Trajectory>& total_obs_traj,int N){
    if(total_obs_traj.size()==0) return;
    Control U;
    U<<0,0;
    for(int i=0;i<total_obs_traj.size();i++){
        State X = total_obs_traj[i].get_states()[0];
        total_obs_traj[i].states.clear();
        for(int j=0;j<N+1;j++){
            State Xnew = obs_dynamic(X,U);
            total_obs_traj[i].push_back(Xnew);
            X = Xnew;
        }
    }
}

void set_scenario(scenario& scenario_type, std::vector<State>& init_states, Arg& arg,bool& is_following){
        switch (scenario_type)
    {
        case dynamic_collision_avoid:
                init_states.push_back({80,0.5,0,2});
                init_states.push_back({50,2,0,1});
                init_states.push_back({70,-2,0,1});
                init_states.push_back({50,2,0,0});
                init_states.push_back({70,-2,0,0});
                init_states.push_back({90,2,0,5});
                init_states.push_back({110,-2,0,2});
                init_states.push_back({30,2,-M_PI/2,1});
                init_states.push_back({70,-2,M_PI/2,0.5});
                //车辆参数
                arg.ego_rad = 3;
                arg.lf      = 1.6;
                arg.lr      =  1.13;
                arg.len       =  2.73;
                arg.width   =  2;
                // 仿真参数
                arg.tf = 1000;
                arg.dt = 0.1;
                //CILQR参数
                arg.N = 20; //Horizen
                arg.tol = 1e-3;
                arg.rel_tol = 1e-5;
                arg.max_iter = 50;
                arg.lamb_init = 1;
                arg.lamb_factor = 2;
                arg.lamb_max = 100;
                //纯跟踪参数
                arg.kv = 0.3; //前视距离系数
                arg.kp = 0.8; //速度P控制器系数
                arg.ld0 = 3;  //基础前瞻距离
                arg.ld_min = 3;
                arg.ld_max = 20;
                //代价参数
                arg.desire_speed = 10;
                arg.desire_heading = 0;
                arg.if_cal_obs_cost = true;
                arg.if_cal_lane_cost = true;
                arg.if_cal_steer_cost = true;
                //最大转向约束
                arg.steer_angle_max = 1;
                arg.steer_max_q1 = 1;
                arg.steer_max_q2 = 1;
                //最小转向约束
                arg.steer_angle_min = -1;
                arg.steer_min_q1 = 1;
                arg.steer_min_q2 = 1;
                //道路约束
                arg.trace_safe_width_left = 4;
                arg.trace_safe_width_right = 4;
                arg.lane_q1 = 5;
                arg.lane_q2 = 5;
                //障碍约束
                arg.obs_q1 = 5;
                arg.obs_q2 = 5;
                arg.obs_length = 2;
                arg.obs_width = 1;
                arg.safe_a_buffer = 2;
                arg.safe_b_buffer = 0.5;
                //横向偏移代价
                arg.ref_weight = 3;
                arg.Q << 0, 0, 0, 0, 
                            0, 0, 0, 0,
                            0, 0, 1, 0,
                            0, 0, 0, 1;
                arg.R <<    1,    0,
                                0,      100;
            break;

            case dense_static_obstacle:
                init_states.push_back({30,1,0,0});
                init_states.push_back({40,2,0,0});
                init_states.push_back({50,-2,0,0});
                init_states.push_back({60,2,0,0});
                init_states.push_back({70,-2,0,0});
                init_states.push_back({80,3,0,0});
                init_states.push_back({90,-2,0,0});
                init_states.push_back({100,2,-M_PI/4,0});
                init_states.push_back({105, -2,M_PI/4,0});
                //车辆参数
                arg.ego_rad = 3;
                arg.lf      = 1.6;
                arg.lr      =  1.13;
                arg.len       =  2.73;
                arg.width   =  2;
                // 仿真参数
                arg.tf = 1000;
                arg.dt = 0.1;
                //CILQR参数
                arg.N = 10; //Horizen
                arg.tol = 1e-3;
                arg.rel_tol = 1e-5;
                arg.max_iter = 50;
                arg.lamb_init = 1;
                arg.lamb_factor = 2;
                arg.lamb_max = 100;
                //纯跟踪参数
                arg.kv = 0.3; //前视距离系数
                arg.kp = 0.8; //速度P控制器系数
                arg.ld0 = 3;  //基础前瞻距离
                arg.ld_min = 3;
                arg.ld_max = 20;
                //代价参数
                arg.desire_speed = 10;
                arg.desire_heading = 0;
                arg.if_cal_obs_cost = true;
                arg.if_cal_lane_cost = true;
                arg.if_cal_steer_cost = true;
                //最大转向约束
                arg.steer_angle_max = 1;
                arg.steer_max_q1 = 1;
                arg.steer_max_q2 = 1;
                //最小转向约束
                arg.steer_angle_min = -1;
                arg.steer_min_q1 = 1;
                arg.steer_min_q2 = 1;
                //道路约束
                arg.trace_safe_width_left = 4;
                arg.trace_safe_width_right = 4;
                arg.lane_q1 = 5;
                arg.lane_q2 = 5;
                //障碍约束
                arg.obs_q1 = 5;
                arg.obs_q2 = 5;
                arg.obs_length = 2;
                arg.obs_width = 1;
                arg.safe_a_buffer = 2;
                arg.safe_b_buffer = 0.5;
                //横向偏移代价
                arg.ref_weight = 3;
                arg.Q << 0, 0, 0, 0, 
                            0, 0, 0, 0,
                            0, 0, 1, 0,
                            0, 0, 0, 1;
                arg.R <<    1,    0,
                                0,      100;
            break;

            case narror_corridor:
                init_states.push_back({50,   3,0,0});
                init_states.push_back({50, -3,0,0});
                init_states.push_back({70,   2, -M_PI/32,0});
                init_states.push_back({70, -4,  -M_PI/32,0});
                init_states.push_back({90,   2.5, M_PI/32,0});
                init_states.push_back({90, -4,  M_PI/32,0});
                init_states.push_back({110,   4, M_PI/64,0});
                init_states.push_back({110, -2.5,  M_PI/50,0});
                //车辆参数
                arg.ego_rad = 3;
                arg.lf      = 1.6;
                arg.lr      =  1.13;
                arg.len       =  2.73;
                arg.width   =  2;
                // 仿真参数
                arg.tf = 1000;
                arg.dt = 0.1;
                //CILQR参数
                arg.N = 50; //Horizen
                arg.tol = 1e-3;
                arg.rel_tol = 1e-5;
                arg.max_iter = 50;
                arg.lamb_init = 1;
                arg.lamb_factor = 2;
                arg.lamb_max = 100;
                //纯跟踪参数
                arg.kv = 0.3; //前视距离系数
                arg.kp = 0.8; //速度P控制器系数
                arg.ld0 = 3;  //基础前瞻距离
                arg.ld_min = 3;
                arg.ld_max = 20;
                //代价参数
                arg.desire_speed = 10;
                arg.desire_heading = 0;
                arg.if_cal_obs_cost = true;
                arg.if_cal_lane_cost = true;
                arg.if_cal_steer_cost = true;
                //最大转向约束
                arg.steer_angle_max = 1;
                arg.steer_max_q1 = 1;
                arg.steer_max_q2 = 1;
                //最小转向约束
                arg.steer_angle_min = -1;
                arg.steer_min_q1 = 1;
                arg.steer_min_q2 = 1;
                //道路约束
                arg.trace_safe_width_left = 4;
                arg.trace_safe_width_right = 4;
                arg.lane_q1 = 5;
                arg.lane_q2 = 5;
                //障碍约束
                arg.obs_q1 = 5;
                arg.obs_q2 = 5;
                arg.obs_length = 20;
                arg.obs_width = 2;
                arg.safe_a_buffer = 0;
                arg.safe_b_buffer = 1;
                //横向偏移代价
                arg.ref_weight = 3;
                arg.Q << 0, 0, 0, 0, 
                            0, 0, 0, 0,
                            0, 0, 1, 0,
                            0, 0, 0, 1;
                arg.R <<    1,    0,
                                0,      100;
            break;

            case following:
                is_following = true;
                arg.N = 30;
                arg.desire_speed = 10;
                arg.following_distance =10;
                init_states.push_back({30,0,0,5});
        arg.Q <<  1, 0, 0, 0, 
                            0, 1, 0, 0,
                            0, 0, 1, 0,
                            0, 0, 0, 1;
                break;

            default:
                init_states.push_back({50,2,0,1});
                break;
    }
}

int main(){
     scenario scenario_type;
    //----------------- 可选择的scenario类型-----------------------
    // scenario_type = custom;                                      //在ilqr.h里面自定义场景
    scenario_type = dynamic_collision_avoid;      //动态避障
    // scenario_type = dense_static_obstacle;       //密集静态障碍物避障
    // scenario_type = narror_corridor;                     //窄通道同行
    // scenario_type = following;                                  //跟车行驶


    //参数初始化
    Arg arg;
    bool is_following = false;
    //障碍物初始化
    std::vector<Trajectory> total_obs_traj;
    std::vector<State> init_states;
    set_scenario(scenario_type,init_states,arg,is_following);
    obs_traj_init(total_obs_traj,arg.N,init_states);

    std::vector<Point> way_points;
    std::vector<std::vector<double>> global_plan_log(3),ego_log(4);
    //获取地图信息
    std::vector<std::vector<double>> m_map_info = load_map();

    //填充路点
    for(int i=0;i<m_map_info[0].size();i++){
        double x,y,heading;
        if(scenario_type == following){
            x = m_map_info[0][i];
            y = 0;
            heading = 0;
        }else{
            x = m_map_info[0][i];
            y = m_map_info[1][i];
            heading = m_map_info[2][i];
        }

        Point point(x,y,heading);
        way_points.push_back(point);
        global_plan_log[0].push_back(x);
        global_plan_log[1].push_back(y);
        global_plan_log[2].push_back(heading);
    }
    // 设置全局路径
    GlobalPlan global_plan;
    global_plan.set_plan(way_points);



    //车辆模型初始化
    Vehicle ego;
    ego.set_state(m_map_info[0][0],m_map_info[1][0],m_map_info[2][0],1);
    ego.set_global_plan(global_plan);
    ego.set_model(SystemModel(arg.dt,arg.N));
    for(int i=0;i<4;i++){
        ego_log[i].push_back(ego.get_state()[i]);
    }

    //求解器初始化
    CILQRSolver cilqr_solver(ego,arg);
    Solution solution;
    Control cur_ctrl;
    State cur_state = ego.get_state();

    //主循环
    for(int i = 0;i<2000;i++){
        std::cout<<"***** Iter ***** " << i <<std::endl;
        //更新障碍物轨迹
        update_obs_traj(total_obs_traj,arg.N);

        // 问题求解
        clock_t start = clock();
        solution = cilqr_solver.solve(cur_state,total_obs_traj,is_following); 
        clock_t end = clock();
        double cpu_time_used = static_cast<double>(end - start) / CLOCKS_PER_SEC;
        std::cout << "CPU time used: " << cpu_time_used * 1000 << " ms\n";

        //更新车辆状态以及控制
        cur_ctrl = solution.control_sequence.controls[0];
        cur_state = ego.get_model().dynamics(cur_state,cur_ctrl);
        //记录车辆历史轨迹
        for(int j=0;j<4;j++){
            ego_log[j].push_back(cur_state[j]);
        }
        //打印车辆状态，并绘制相应图形
        std::cout<<cur_state<<std::endl;
        my_plot(global_plan_log,ego_log,total_obs_traj,solution,arg);
    }
    return 0;
}

