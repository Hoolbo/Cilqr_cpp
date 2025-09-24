#include <iostream>
#include "ilqr.h"
#include "utils.h"
#include <ctime>

int main(){
    std::cout << "=== CILQR Program Starting ===" << std::endl;
    std::cout << "Initializing variables..." << std::endl;
    
    // Initialize variables
    std::vector<Point> way_points;
    std::vector<std::vector<double>> global_plan_log(3), ego_log(4);
    
    std::cout << "Variables initialized successfully." << std::endl;
    
    // Load bitmap map data
    MapData bitmap_map = load_bitmap_map("../Maps/bitmap/B206_global_map.json");
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
    save_map_data(&bitmap_map);
    
    // Load semantic map data
    std::vector<std::vector<double>> m_map_info = load_map();
    std::cout << "Semantic map loaded. Size: " << m_map_info.size() << std::endl;

    //填充路点
    for(int i=0;i<300;i++){
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
    ego.set_state(m_map_info[0][50],m_map_info[1][50],m_map_info[2][50],0);
    ego.set_global_plan(global_plan);
    ego.set_model(SystemModel(arg.dt,arg.N));
    for(int i=0;i<4;i++){
        ego_log[i].push_back(ego.get_state()[i]);
    }
    
    //障碍物初始化
    Trajectory obs_trj;
    for(int i=0;i<arg.N+1;i++){
        obs_trj.push_back(State(40,22,0,0));
    }

    //求解器初始化
    CILQRSolver cilqr_solver(ego,obs_trj,arg);
    Solution solution;
    Control cur_ctrl;
    State cur_state = ego.get_state();


    //主循环
    // for(int i = 0;i<arg.tf/arg.dt;i++){
    for(int i = 0;i<200;i++){
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
            dynamic_plot(global_plan_log,ego_log,obs_trj,solution,&bitmap_map,global_plan,ego.get_model());
        // }
 
    }

    // mclTerminateApplication();
    return 0;
}
