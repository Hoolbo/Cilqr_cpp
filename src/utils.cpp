#include "utils.h"
#include "Eigen/Eigen"
std::vector<std::vector<double>> load_map(){

    std::vector<std::vector<double>> map_info(3);

    if (!mclInitializeApplication(nullptr, 0)) { // 必须最先调用
        std::cerr << "Could not initialize MATLAB Runtime" << std::endl;
    }

    MATFile *pmatFile = matOpen("C:/Users/Hoolbo/Code/CILQR/Cilqr_cpp/map.mat", "r");
    if (!pmatFile) {
        std::cerr << "Error: 无法打开map.mat文件" << std::endl;
    }

    mxArray *pMxArray = matGetVariable(pmatFile, "xcoord");
    if (!pMxArray) {
        std::cerr << "Error: 文件内找不到变量xcoord" << std::endl;
        matClose(pmatFile);
    }

    size_t M = mxGetM(pMxArray); // 行数[1](@ref)
    size_t N = mxGetN(pMxArray); // 列数
    double *data = (double*)mxGetData(pMxArray); // 数据指针[1,6](@ref)

    for(int i=0;i<M;i++){
        for(int j=0;j<N;j++){
            map_info[0].push_back(data[i*N+j]);
        }
    }

    pMxArray = matGetVariable(pmatFile, "ycoord");
    if (!pMxArray) {
        std::cerr << "Error: 文件内找不到变量ycoord" << std::endl;
        matClose(pmatFile);
    }

    M = mxGetM(pMxArray); // 行数[1](@ref)
    N = mxGetN(pMxArray); // 列数
    data = (double*)mxGetData(pMxArray); // 数据指针[1,6](@ref)

    for(int i=0;i<M;i++){
        for(int j=0;j<N;j++){
            map_info[1].push_back(data[i*N+j]);
        }
    }

    pMxArray = matGetVariable(pmatFile, "theta");
    if (!pMxArray) {
        std::cerr << "Error: 文件内找不到变量theta" << std::endl;
        matClose(pmatFile);
    }

    M = mxGetM(pMxArray); // 行数[1](@ref)
    N = mxGetN(pMxArray); // 列数
    data = (double*)mxGetData(pMxArray); // 数据指针[1,6](@ref)

    for(int i=0;i<M;i++){
        for(int j=0;j<N;j++){
            map_info[2].push_back(data[i*N+j]);
        }
    }

        mxDestroyArray(pMxArray);
        matClose(pmatFile);
        return map_info;
}

void my_plot(std::vector<double> global_x,std::vector<double> global_y,
     std::vector<double> ego_x,std::vector<double> ego_y){
        using namespace std;
        using namespace matplot;
        // backend()->interactive(true); // 启用交互模式
        static figure_handle fig;
        static line_handle global_path;
        static line_handle ego_path;
        if (!fig) {
            fig = figure();
            fig->ion();
            // fig->backend()->is_interactive();
            auto ax = fig->current_axes();
            hold(on);
            ax->xlim({-10, 900}); // 设置合理范围
            ax->ylim({-50, 100});
            global_path = plot(ax,global_x,global_y,"--b");
            ego_path = plot(ax,ego_x,ego_y,"r");
            // matplot::show();
        }
        else{
            global_path->x_data(global_x);
            global_path->y_data(global_y);
            ego_path->x_data(ego_x);
            ego_path->y_data(ego_y);
            // fig->draw();
        }
}

// #include <matplot/matplot.h>

// void my_plot(std::vector<double> global_x, std::vector<double> global_y,
//              std::vector<double> ego_x, std::vector<double> ego_y) {
//     using namespace matplot;
    
//     // 静态变量保存figure和线条句柄
//     static figure_handle fig;
//     static matplot::line_handle lines;

//     // 第一次调用初始化
//         fig = figure();                 // 创建figure对象
//         hold(on);                       // 启用图形保持
//         auto ax = fig->current_axes();  // 获取当前坐标系
        
//         lines->x_data(global_x);
//         lines->y_data(global_y);
//         show();  // 首次显示窗口（非阻塞模式）

// }

