#include <iostream>
#include "ilqr.h"
#include "utils.h"
#include <ctime>

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

int main(){
    std::vector<Point> way_points;
    std::vector<std::vector<double>> global_plan_log(3),ego_log(4);

    //获取地图信息
    std::vector<std::vector<double>> m_map_info = load_map();

    ///////////////////换道测试
    std::vector<Point> way_points_lanechange;
    std::vector<std::vector<double>> global_plan_lanechange_log(3);
    GlobalPlan global_plan_lanechange;
    
    for(int i=0;i<1000;i++){
        Point point(m_map_info[0][i],m_map_info[1][i]+2,m_map_info[2][i]);
        way_points_lanechange.push_back(point);
        global_plan_lanechange_log[0].push_back(m_map_info[0][i]);
        global_plan_lanechange_log[1].push_back(m_map_info[1][i]+2);
        global_plan_lanechange_log[2].push_back(m_map_info[2][i]);
    }
    global_plan_lanechange.set_plan(way_points_lanechange);
    //////////////////换道测试

    //填充路点
    for(int i=0;i<1000;i++){
        Point point(m_map_info[0][i],m_map_info[1][i],m_map_info[2][i]);
        way_points.push_back(point);
        global_plan_log[0].push_back(m_map_info[0][i]);
        global_plan_log[1].push_back(m_map_info[1][i]);
        global_plan_log[2].push_back(m_map_info[2][i]);
    }

    // 设置全局路径
    GlobalPlan global_plan;
    global_plan.set_plan(way_points);

    //参数初始化
    Arg arg;

    //车辆模型初始化
    Vehicle ego;
    ego.set_state(m_map_info[0][0],m_map_info[1][0],m_map_info[2][0],0);
    ego.set_global_plan(global_plan);
    ego.set_model(SystemModel(arg.dt,arg.N));
    for(int i=0;i<4;i++){
        ego_log[i].push_back(ego.get_state()[i]);
    }
    
    //障碍物初始化
    std::vector<Trajectory> total_obs_traj;
    std::vector<State> init_states;
    //跟车
    // init_states.push_back({70,0.5,0,2});
    
    //避障
    // init_states.push_back({50,1,0,-2});
    // init_states.push_back({80,-1,0,-1});
    
    //人行道让行
    init_states.push_back({20,2,-M_PI/2,0.25});

    obs_traj_init(total_obs_traj,arg.N,init_states);

    //求解器初始化
    CILQRSolver cilqr_solver(ego,arg);
    Solution solution;
    Control cur_ctrl;
    State cur_state = ego.get_state();

    //主循环
    // for(int i = 0;i<arg.tf/arg.dt;i++){
    for(int i = 0;i<2000;i++){
        std::cout<<"***** Iter ***** " << i <<std::endl;
        //换道测试
        // if(i==100){
        //     cilqr_solver.set_global_plan(global_plan_lanechange);
        //     global_plan_log = global_plan_lanechange_log;
        // }

        //更新障碍物轨迹
        update_obs_traj(total_obs_traj,arg.N);

        // 问题求解
        clock_t start = clock();
        solution = cilqr_solver.solve(cur_state,total_obs_traj); 
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
        std::cout<<cur_state<<std::endl;
        my_plot(global_plan_log,ego_log,total_obs_traj,solution);

    }
    return 0;
}

