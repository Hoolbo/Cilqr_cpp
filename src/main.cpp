#include <iostream>
#include "ilqr.h"
#include "utils.h"
#include <ctime>

int main(){
    std::vector<Point> way_points;
    std::vector<std::vector<double>> global_plan_log(3),ego_log(4);

    //获取地图信息
    std::vector<std::vector<double>> m_map_info = load_map();

    //填充路点
    for(int i=300;i<1000;i++){
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
    ego.set_state(m_map_info[0][300],m_map_info[1][300],m_map_info[2][300],0);
    ego.set_global_plan(global_plan);
    ego.set_model(SystemModel(arg.dt,arg.N));
    for(int i=0;i<4;i++){
        ego_log[i].push_back(ego.get_state()[i]);
    }
    
    //障碍物初始化
    Trajectory obs_trj;
    for(int i=0;i<arg.N+1;i++){
        obs_trj.push_back(State(380,1,0,0));
    }

    //求解器初始化
    CILQRSolver cilqr_solver(ego,obs_trj,arg);
    Solution solution;
    Control cur_ctrl;
    State cur_state = ego.get_state();


    //主循环
    // for(int i = 0;i<arg.tf/arg.dt;i++){
    for(int i = 0;i<2000;i++){
        std::cout<<"***** Iter ***** " << i <<std::endl;
        // 问题求解
        clock_t start = clock();
        solution = cilqr_solver.solve(cur_state,obs_trj); 
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

        std::cout<<"状态如下"<<std::endl;
        std::cout<<"x   :  "<<cur_state[0]<<std::endl;
        std::cout<<"y   :   "<<cur_state[1]<<std::endl;
        std::cout<<"theta   :   "<<cur_state[2]<<std::endl;
        std::cout<<"gamma   :   "<<cur_state[3]<<std::endl;
         std::cout<<"控制如下"<<std::endl;
         std::cout<<"v  :   "<< cur_ctrl[0]<<std::endl;
         std::cout<<"omega  :   "<< cur_ctrl[1]<<std::endl;
        // if(i%3==0){
            my_plot(global_plan_log,ego_log,obs_trj,solution);
        // }
 
    }

    // mclTerminateApplication();
    return 0;
}
