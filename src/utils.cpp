#include "utils.h"


std::vector<std::vector<double>> load_map(){

    std::vector<std::vector<double>> map_info(3);

    // if (!mclInitializeApplication(nullptr, 0)) { // 必须最先调用
    //     std::cerr << "Could not initialize MATLAB Runtime" << std::endl;
    // }

    // MATFile *pmatFile = matOpen("C:/Users/Hoolbo/Code/CILQR/Cilqr_cpp/map.mat", "r");
    // if (!pmatFile) {
    //     std::cerr << "Error: 无法打开map.mat文件" << std::endl;
    // }

    // mxArray *pMxArray = matGetVariable(pmatFile, "xcoord");
    // if (!pMxArray) {
    //     std::cerr << "Error: 文件内找不到变量xcoord" << std::endl;
    //     matClose(pmatFile);
    // }

    // size_t M = mxGetM(pMxArray); // 行数[1](@ref)
    // size_t N = mxGetN(pMxArray); // 列数
    // double *data = (double*)mxGetData(pMxArray); // 数据指针[1,6](@ref)

    // for(int i=0;i<M;i++){
    //     for(int j=0;j<N;j++){
    //         map_info[0].push_back(data[i*N+j]);
    //     }
    // }

    // pMxArray = matGetVariable(pmatFile, "ycoord");
    // if (!pMxArray) {
    //     std::cerr << "Error: 文件内找不到变量ycoord" << std::endl;
    //     matClose(pmatFile);
    // }

    // M = mxGetM(pMxArray); // 行数[1](@ref)
    // N = mxGetN(pMxArray); // 列数
    // data = (double*)mxGetData(pMxArray); // 数据指针[1,6](@ref)

    // for(int i=0;i<M;i++){
    //     for(int j=0;j<N;j++){
    //         map_info[1].push_back(data[i*N+j]);
    //     }
    // }

    // pMxArray = matGetVariable(pmatFile, "theta");
    // if (!pMxArray) {
    //     std::cerr << "Error: 文件内找不到变量theta" << std::endl;
    //     matClose(pmatFile);
    // }

    // M = mxGetM(pMxArray); // 行数[1](@ref)
    // N = mxGetN(pMxArray); // 列数
    // data = (double*)mxGetData(pMxArray); // 数据指针[1,6](@ref)

    // for(int i=0;i<M;i++){
    //     for(int j=0;j<N;j++){
    //         map_info[2].push_back(data[i*N+j]);
    //     }
    // }

    //     mxDestroyArray(pMxArray);
    //     matClose(pmatFile);

    //本来是用matlab导入地图，现在改成自己用正弦曲线拟合地图
    for(int i=0;i<1000;i++){
        map_info[0].push_back(i*1.0);
        map_info[1].push_back(sin(i*1.0*0.0628));
        map_info[2].push_back(atan2(sin(i*1.0*0.0628)-sin((i-1)*1.0*0.0628),1.0));
    } 

    return map_info;
}


void my_plot(const std::vector<std::vector<double>>& global_plan_log,
    const std::vector<std::vector<double>>& ego_log,
    const Trajectory& obs_traj,
    const Solution& solution) 
{       
        namespace plt = matplotlibcpp;
        // 使用智能指针避免静态变量初始化问题
        static std::unique_ptr<matplotlibcpp::Plot> global_plot, ego_plot,obs_traj_plot, 
                                        trajectory_plot,vehicle_rect_plot;
        
        static bool figure_initialized = false;
        constexpr double VEHICLE_LENGTH = 2.7;  // 车长（单位：米）
        constexpr double VEHICLE_WIDTH = 2;   // 车宽
        // 动态视图参数
        constexpr double FOLLOW_FACTOR = 0.7;
        constexpr double BASE_MARGIN = 20.0;
        static std::pair<double, double> view_center = {0, 0};

        // 首次初始化图形窗口
        if (!figure_initialized) {
            figure_initialized = true;
            plt::figure();
            plt::title("CILQR PLANNING");
            plt::xlabel("X (m)");
            plt::ylabel("Y (m)");
            plt::grid(true);
            
            global_plot.reset(new matplotlibcpp::Plot(
                "global_plot",
                global_plan_log[0], 
                global_plan_log[1],
                "k-."
            ));
        }
        global_plot->update(global_plan_log[0],global_plan_log[1]);

        
        double target_x = ego_log[0].back();
        double target_y = ego_log[1].back();
        view_center.first += FOLLOW_FACTOR * (target_x - view_center.first);
        view_center.second += FOLLOW_FACTOR * (target_y - view_center.second);
        double speed = ego_log[3].back();
        double margin = BASE_MARGIN + speed * 3;



        std::vector<std::vector<double>> trajectory(4);
        for(int i=0;i<solution.ego_trj.states.size();i++){
            trajectory[0].push_back(solution.ego_trj.states[i][0]);
            trajectory[1].push_back(solution.ego_trj.states[i][1]);
        }

        // 初始化并更新轨迹
        if(!trajectory_plot){
            trajectory_plot.reset(new matplotlibcpp::Plot(
                "trajectory_plot",
                trajectory[0],
                trajectory[1],
                "g-"
            ));
        }
        trajectory_plot->update(trajectory[0],trajectory[1]);
        
        //绘制障碍物
        double obs_x = obs_traj.states[0][0];
        double obs_y = obs_traj.states[0][1];
        double obs_theta = obs_traj.states[0][2]; 
        std::vector<double> obs_rect_x, obs_rect_y;
        double obs_cos_theta = cos(obs_theta);
        double obs_sin_theta = sin(obs_theta);
        double obs_half_length = 2.7 / 2;
        double obs_half_width = 2 / 2;
        std::array<std::pair<double, double>, 4> obs_local_points = {
            std::make_pair( obs_half_length,  obs_half_width),  // 前右
            std::make_pair( obs_half_length, -obs_half_width),  // 后右
            std::make_pair(-obs_half_length, -obs_half_width),  // 后左
            std::make_pair(-obs_half_length,  obs_half_width)   // 前左
        };
        for (const auto& pt : obs_local_points) {
            // 旋转和平移变换
            double global_x = obs_x + pt.first * obs_cos_theta - pt.second *  obs_sin_theta;
            double global_y = obs_y + pt.first *  obs_sin_theta + pt.second * obs_cos_theta;
            obs_rect_x.push_back(global_x);
            obs_rect_y.push_back(global_y);
        }
        obs_rect_x.push_back(obs_rect_x.front());
        obs_rect_y.push_back(obs_rect_y.front());
        // 更新或创建绘图对象
        if (!obs_traj_plot) {
            obs_traj_plot.reset(new matplotlibcpp::Plot(
                " ",
                obs_rect_x, 
                obs_rect_y, 
                "c-"
            ));
        }
        obs_traj_plot->update(obs_rect_x, obs_rect_y);


        // 更新车辆矩形
        double x = ego_log[0].back();
        double y = ego_log[1].back();
        double theta = ego_log[2].back(); 
        // 计算矩形四角相对坐标
        const double half_len = VEHICLE_LENGTH / 2;
        const double half_wid = VEHICLE_WIDTH / 2;
        std::array<std::pair<double, double>, 4> local_points = {
            std::make_pair( half_len,  half_wid),  // 前右
            std::make_pair( half_len, -half_wid),  // 后右
            std::make_pair(-half_len, -half_wid),  // 后左
            std::make_pair(-half_len,  half_wid)   // 前左
        };
        // 坐标系变换
        std::vector<double> rect_x, rect_y;
        double cos_theta = cos(theta);
        double sin_theta = sin(theta);
        for (const auto& pt : local_points) {
            // 旋转和平移变换
            double global_x = x + pt.first * cos_theta - pt.second * sin_theta;
            double global_y = y + pt.first * sin_theta + pt.second * cos_theta;
            rect_x.push_back(global_x);
            rect_y.push_back(global_y);
        }
        // 闭合矩形
        rect_x.push_back(rect_x.front());
        rect_y.push_back(rect_y.front());
        // 更新或创建绘图对象
        if (!vehicle_rect_plot) {
            vehicle_rect_plot.reset(new matplotlibcpp::Plot(
                " ",
                rect_x, 
                rect_y, 
                "b-"
            ));
        }
        vehicle_rect_plot->update(rect_x, rect_y);
        
        // 历史轨迹
        if (!ego_plot){
            ego_plot.reset(new matplotlibcpp::Plot(
                "ego_plot",
                ego_log[0],
                ego_log[1],
                "r-"
            ));
        } 
        ego_plot->update(ego_log[0], ego_log[1]);

        // 非阻塞式刷新（关键参数）
        // plt::backend("TkAgg");  // 使用更快的后端
        // plt::ion();             // 启用交互模式
        plt::xlim(view_center.first - margin, view_center.first + margin);
        plt::ylim(view_center.second - margin, view_center.second + margin);
        plt::grid(true);
        matplotlibcpp::pause(0.02);  // 控制刷新频率
        matplotlibcpp::draw();        // 强制立即绘制
            
}
