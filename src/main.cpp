#include <iostream>
#include "ilqr.h"
#include "utils.h"


int main(){
    std::vector<Point> way_points;
    std::vector<double> global_x_log,global_y_log,x_log,y_log;
    std::vector<double> steer_log;

    //获取地图信息
    std::vector<std::vector<double>> m_map_info = load_map();

    //填充路点
    for(int i=0;i<2000;i++){
        Point point(m_map_info[0][i],m_map_info[1][i],m_map_info[2][i]);
        way_points.push_back(point);
        global_x_log.push_back(point.x);
        global_y_log.push_back(point.y);
    }

    // 设置全局路径
    GlobalPlan global_plan;
    global_plan.set_plan(way_points);

    //参数初始化
    Arg arg;

    //车辆模型初始化
    Vehicle ego;
    ego.set_state(m_map_info[0][1],m_map_info[1][1],m_map_info[2][1],arg.desire_speed);
    ego.set_global_plan(global_plan);
    ego.set_model(SystemModel(arg.dt,arg.N));
    x_log.push_back(ego.get_state()[0]);
    y_log.push_back(ego.get_state()[1]);
    
    //障碍物初始化
    Trajectory obs_trj;
    for(int i=0;i<arg.N+1;i++){
        obs_trj.push_back(State(300,2,0,0));
    }

    //求解器初始化
    CILQRSolver cilqr_solver(ego,obs_trj,arg);
    Solution solution;
    Control cur_ctrl;
    State cur_state = ego.get_state();


    //主循环
    // for(int i = 0;i<arg.tf/arg.dt;i++){
    for(int i = 0;i<2000;i++){
        std::cout<<"第 " << i <<" 次迭代: "<<std::endl;
        // 问题求解
        solution = cilqr_solver.solve(cur_state,obs_trj); 
        
        cur_ctrl = solution.control_sequence.controls[0];
        cur_state = ego.get_model().dynamics(cur_state,cur_ctrl);
        x_log.push_back(cur_state[0]);
        y_log.push_back(cur_state[1]);
        steer_log.push_back(cur_ctrl[1]);
        std::cout<<cur_state<<std::endl;
        if(i%200 == 0){
            my_plot(global_x_log,global_y_log,x_log,y_log);
        }

    }

    

    // 保持窗口打开
    matplot::show();
    mclTerminateApplication();
    return 0;
}
