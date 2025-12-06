#include <cmath>
#include <ctime>
#include <iostream>
#include "ilqr.h"
#include "utils.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <chrono>
#include <ctime>
using namespace Eigen;
#include <algorithm>
#include <future>
//计算两点之间的距离
double distance(const Point& p1, const Point& p2) {
    return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
};
size_t find_closest_point(const std::vector<Point>& path,const State& state){
    size_t nearest_index = 0;
    double min_distance = std::numeric_limits<double>::max();
    Point point = {state(0), state(1),0};
    for (size_t i = 0; i < path.size(); ++i) {

        double dist = distance(path[i], point);
        if (dist < min_distance) {
            min_distance = dist;
            nearest_index = i;
        }
    }
    return nearest_index;
}
void LocalPlan::set_plan(const GlobalPlan& global_plan,const State& vehicle_state,size_t num_points_to_extract){

    const std::vector<Point>& global_points = global_plan.get_points();
    if (global_points.empty()) {
        this->points.clear();
        return;
    }

    // 找到最近的点
    size_t nearest_index = find_closest_point(global_points,vehicle_state);

    // 提取后续的点(数量根据速度动态调整,传进来的num_points_to_extract已经是调整过的了)
    size_t end_index = std::min(nearest_index + num_points_to_extract, global_points.size());
    this->points.assign(global_points.begin() + nearest_index, global_points.begin() + end_index);
    while(this->points.size() < num_points_to_extract){
         this->points.push_back(this->points.back());
    }
    // std::cout << "LocalPlan points.size():" << this->points.size() << std::endl;
}

BarrieInfo barrierFunction(double q1,double q2,double c,VectorXd dc){
    BarrieInfo info;
    
    // 数值保护：限制指数函数的输入范围，防止数值爆炸
    const double MAX_EXP_INPUT = 20.0;  // exp(20) ≈ 4.85e8，仍在double范围内
    const double MAX_BARRIER_COST = 1e6; // 最大barrier代价上限
    const double SMOOTH_TRANSITION = 0.1; // 平滑过渡区间
    
    // 改进的barrier函数：使用平滑过渡避免硬截断
    double exp_input = std::min(q2 * c, MAX_EXP_INPUT);
    double exp_val = std::exp(exp_input);
    
    // 使用tanh函数实现平滑过渡，避免硬截断导致的梯度不连续
    double raw_cost = q1 * exp_val;
    double transition_factor = 0.5 * (1.0 + std::tanh((MAX_BARRIER_COST - raw_cost) / (SMOOTH_TRANSITION * MAX_BARRIER_COST)));
    info.b = raw_cost * transition_factor + MAX_BARRIER_COST * (1.0 - transition_factor);
    
    // 梯度计算：考虑平滑过渡的影响
    double gradient_scale = transition_factor;
    if (raw_cost > MAX_BARRIER_COST * 0.9) {
        // 在接近上限时进一步减小梯度
        gradient_scale *= 0.1;
    }
    
    info.d_b = gradient_scale * q1 * q2 * exp_val * dc;
    info.dd_b = gradient_scale * q1 * (q2 * q2) * exp_val * (dc * dc.transpose());
    
    // 确保所有输出都是有限值
    if (!std::isfinite(info.b)) info.b = MAX_BARRIER_COST;
    for (int i = 0; i < info.d_b.size(); ++i) {
        if (!std::isfinite(info.d_b[i])) info.d_b[i] = 0.0;
    }
    for (int i = 0; i < info.dd_b.rows(); ++i) {
        for (int j = 0; j < info.dd_b.cols(); ++j) {
            if (!std::isfinite(info.dd_b(i,j))) info.dd_b(i,j) = 0.0;
        }
    }
    
    return info;
}
// 系统模型
State SystemModel::dynamics(const State& X, const Control& U) {
    double x = X[0];
    double y = X[1];
    double theta = X[2];
    double gamma = X[3];
    double v = U[0];
    double gamma_dot = U[1];
    double len = lr + lf * cos(gamma); // L = l_2 + l_1 * cos(gamma)
    
    State X_next;
    X_next << 
        x + dt * v * cos(theta),
        y + dt * v * sin(theta),
        theta - dt * (v * sin(gamma) + lr * gamma_dot) / len,
        gamma + dt * gamma_dot;
    
    X_next(2) = angle_wrap(X_next(2)); // 保留角度规范化
    return X_next;
}

Matrix4d SystemModel::get_jacobian_state(const Vector4d& X, const Vector2d& U) {
    double theta = X[2];
    double gamma = X[3];
    double v = U[0];
    double gamma_dot = U[1];
    double len = lr + lf * cos(gamma); // L = l_2 + l_1 * cos(gamma)
    
    Matrix4d df_dx;
    // 计算 theta_1,k+1 对 gamma_k 的偏导数
    double num = v * cos(gamma) * (lr + lf * cos(gamma)) + (v * sin(gamma) + lr * gamma_dot) * (lf * sin(gamma));
    double denom = len * len;
    double dtheta_dgamma = -dt * num / denom;
    
    df_dx << 
        1.0, 0.0, -dt * v * sin(theta), 0.0,
        0.0, 1.0, dt * v * cos(theta), 0.0,
        0.0, 0.0, 1.0, dtheta_dgamma,
        0.0, 0.0, 0.0, 1.0;
    
    return df_dx;
}

Matrix<double, 4, 2> SystemModel::get_jacobian_control(const Vector4d& X, const Vector2d& U) {
    double theta = X[2];
    double gamma = X[3];
    double v = U[0];
    double gamma_dot = U[1];
    double len = lr + lf * cos(gamma); // L = l_2 + l_1 * cos(gamma)
    
    Matrix<double, 4, 2> df_du;
    df_du << 
        dt * cos(theta), 0.0,
        dt * sin(theta), 0.0,
        -dt * sin(gamma) / len, -dt * lr / len,
        0.0, dt;
    
    return df_du;
}

//Vehicle类方法
Vehicle::Vehicle(){
    this->state << 0,0,0,0;
    this->global_plan = GlobalPlan();
    this->local_plan = LocalPlan();
    this->model = SystemModel();
}

//CILQRSolver类方法
Solution CILQRSolver::solve(const State& init_state,const std::vector<Trajectory>& obs_list) {
    
    ego.set_state(init_state);
    // 使用传入的obs_list参数更新成员变量
    this->obs_list = obs_list;
    // 初始化局部路径和初始解
    ego.set_local_plan();

    //如果接近终点就把期望速度置0
    // Point local_middle_point = this->ego.get_local_plan().get_points()[ego.get_local_plan().get_points().size()/3];
    // Point global_last_point = this->ego.get_global_plan().get_points()[ego.get_global_plan().get_points().size()-1];
    // 使用距离阈值判断接近终点，避免直接把期望速度置为0
    // double dist_to_goal = distance(local_middle_point, global_last_point);
    // if(dist_to_goal < 5) {
    //     // 平滑降低期望速度而不是直接置0（示例：减半）
    //     arg.desire_speed = std::max(0.0, arg.desire_speed * dist_to_goal / 5);
    //     arg.desire_heading = 0;
    // }
    // 改为使用车辆当前位置与全局路径终点的距离判断，避免局部中点导致过早降速
    {
        const auto& global_points = this->ego.get_global_plan().get_points();
        if (!global_points.empty()) {
            const Point& goal_pt = global_points.back();
            const Vector4d cur_state = this->ego.get_state();
            double dx = cur_state[0] - goal_pt.x;
            double dy = cur_state[1] - goal_pt.y;
            double dist_to_goal = std::sqrt(dx*dx + dy*dy);
            if (dist_to_goal < 10.0) {
                // Allow speed to decay to 0 to ensure stopping at the goal
                arg.desire_speed = arg.desire_speed * dist_to_goal / 10.0;
                if (arg.desire_speed < 0.1) arg.desire_speed = 0;
            }
            // 终点附近保持期望航向为 0（如不需要可后续移除）
            arg.desire_heading = 0;
        }
    }
    Solution nominal_solution = get_nominal_solution(init_state);
    Solution current_solution = nominal_solution;
    Solution new_solution;
    double J_old = cal_cost_with_logging(current_solution, 0);
    lamb = arg.lamb_init;

    converged = false;
    
    // 记录收敛信息
    clock_t start_time = clock();
    int iterations_count = 0;
    
    for (int iter = 0; iter < arg.max_iter; ++iter) {
        iterations_count = iter + 1;
        // std::cout << "Iteration: " << iter + 1 << ", Cost: " << J_old << ", Lambda: " << lamb << std::endl;
        
        // // 备份当前解
        // Solution old_solution = current_solution;

        // 反向传播计算控制修正量
        // clock_t start = clock();
        compute_df(current_solution);
        // clock_t end = clock();
        // double cpu_time_used = static_cast<double>(end - start) / CLOCKS_PER_SEC;
        // std::cout << "compute_df time used: " << cpu_time_used * 1000 << " ms\n";
        // start = clock();

        compute_cost_derivatives(current_solution);
        // end = clock();
        // cpu_time_used = static_cast<double>(end - start) / CLOCKS_PER_SEC;
        // std::cout << "compute_cost_derivatives time used: " << cpu_time_used * 1000 << " ms\n";
        // start = clock();

        backward();
        // end = clock();
        // cpu_time_used = static_cast<double>(end - start) / CLOCKS_PER_SEC;
        // std::cout << "backward time used: " << cpu_time_used * 1000  << " ms\n";
        // start = clock();

        // 正向传播尝试新解并计算新代价
        new_solution = forward(current_solution);
        // end = clock();
        // cpu_time_used = static_cast<double>(end - start) / CLOCKS_PER_SEC;
        // std::cout << "forward time used: " << cpu_time_used * 1000 << " ms\n";
        // start = clock();

        double J_new = cal_cost_with_logging(new_solution, iter + 1);
        // end = clock();
        // cpu_time_used = static_cast<double>(end - start) / CLOCKS_PER_SEC;
        // std::cout << "cal_cost time used: " << cpu_time_used * 1000 << " ms\n";
        // start = clock();

        // 计算相对改进量
        double rel_improve = (J_old - J_new) / J_old;
        if(abs(average_gradient)<1e-3){
            // J_total = J_new;
            pre_solution = current_solution;
            std::cout << "Converged | iteration: " << iter + 1  <<" J_total :" << J_new << std::endl;
            converged = true;
            break;
        }
        if (J_new < J_old  ) {
            // 动态调整正则化系数
            if (rel_improve > 0.1) { // 显著改进时降低λ
                lamb = std::max(lamb * 0.7 ,1e-3);
            } else {                 // 轻微改进时保守调整
                lamb = std::max(lamb * 0.7 ,1e-3);
            }
            // 更新当前解
            current_solution = new_solution;
            // 收敛终止条件判断
            if (J_old - J_new < arg.tol) {
                //记录上一次解
                // J_total = J_new;
                pre_solution = current_solution;
                std::cout << "Converged | iteration: " << iter + 1  <<" J_total :" << J_new << std::endl;
                converged = true;
                break;
            }
            J_old = J_new;
        } else {
            //绝对值之差小于一定值判断收敛
            // if(abs(J_old - J_new)<arg.tol){
            //     pre_solution = current_solution;
            //     std::cout << "Converged | iteration: " << iter + 1  << std::endl;
            //     converged = true;
            //     break;
            // } 
            // 代价未下降时增加正则化
            lamb = lamb * 2;
            // 失败终止条件判断
             if (lamb > arg.lamb_max) {
                current_solution = nominal_solution;
                pre_solution = Solution();
                std::cerr << "Unconverged | Maxmum lamb | iteration: "<< iter + 1 << std::endl;
                converged = false;
                break;
            }
        }
    }
    
    if (!converged && lamb <= arg.lamb_max) {
        current_solution = nominal_solution;
        pre_solution = Solution();
        std::cerr << "Unconverged::Maxmum iteration" << std::endl;
        converged = false;
    }
    
    // 记录求解时间
    clock_t end_time = clock();
    double solve_time = static_cast<double>(end_time - start_time) / CLOCKS_PER_SEC;
    
    // 填入收敛信息到Solution中
    current_solution.converged = converged;
    current_solution.iterations = iterations_count;
    current_solution.final_cost = J_old;
    current_solution.solve_time_ms = solve_time * 1000;
    
    // 输出收敛信息
    // std::cout << "CILQR Solver Summary:" << std::endl;
    // std::cout << "  Converged: " << (converged ? "Yes" : "No") << std::endl;
    // std::cout << "  Iterations: " << iterations_count << "/" << arg.max_iter << std::endl;
    // std::cout << "  Final Cost: " << J_old << std::endl;
    // std::cout << "  Final Lambda: " << lamb << std::endl;
    // std::cout << "  Solve Time: " << solve_time * 1000 << " ms" << std::endl;
    
    // pre_solution = current_solution;
    return current_solution;
}
//获取标称轨迹
Solution CILQRSolver::get_nominal_solution(const State& init_state){
    // 添加长度预检查
    if (this->arg.N <= 0) 
    throw std::invalid_argument("N has to be greater than 0");

    Trajectory nominal_trj;
    ControlSequence nominal_ctrl_sequence;
   
    // 初始化时直接预留空间
    nominal_trj.states.reserve(arg.N + 1);
    nominal_ctrl_sequence.controls.reserve(arg.N); 
    State Xout;
    Control U;
    //把初始状态X0放入轨迹
    State X0 = init_state;
    nominal_trj.push_back(X0);
    if(this->pre_solution.control_sequence.size()!=0){
        //如果有上一帧的控制序列，则拼接控制序列
        for(int i=0;i<(this->arg.N - 1);i++){
            Control U = pre_solution.control_sequence.get_control_sequence()[i+1];
            nominal_ctrl_sequence.push_back(U);
        };
        nominal_ctrl_sequence.push_back(pre_solution.control_sequence.get_control_sequence()[arg.N-1]);
        // nominal_ctrl_sequence.push_back(Control(0,0));
        //更新轨迹
        for(int i=0; i < this->arg.N; i++){
            U = nominal_ctrl_sequence.get_control_sequence()[i];
            Xout = this->ego.get_model().dynamics(X0,U); 
            X0 = Xout;
            nominal_trj.push_back(Xout);
        }
    }
    else{
        //上一帧轨迹无效则用纯跟踪获取新粗解
        for (int i = 0; i < arg.N; ++i) {
            // 获取当前状态
            State X_cur = nominal_trj.back();
            // 生成控制指令 - 启用修正后的纯跟踪
            Control U = pure_pursuit(X_cur);
            
            // 约束预处理：确保初始控制序列满足约束条件
            // 限制速度在合理范围内
            U[0] = std::clamp(U[0], 0.0, 15.0);
            
            // 限制铰接角速度在安全范围内，使用更保守的初始值
            double safe_gamma_dot_max = std::min(arg.gamma_dot_max * 0.8, 0.8);
            U[1] = std::clamp(U[1], -safe_gamma_dot_max, safe_gamma_dot_max);
            
            State X_next = ego.get_model().dynamics(X_cur, U);
            
            // 状态约束预处理：确保铰接角在合理范围内
            X_next[3] = std::clamp(X_next[3], 
                                  arg.gamma_min * 0.9,  // 使用更保守的范围
                                  arg.gamma_max * 0.9);
            
            nominal_ctrl_sequence.push_back(U);
            nominal_trj.push_back(X_next);
        }
    }


    Solution solution(nominal_trj,nominal_ctrl_sequence);

    // 添加长度验证
    if (nominal_trj.states.size() != arg.N + 1 || nominal_ctrl_sequence.size() != arg.N) {
        throw std::length_error("Nominal trajectory length mismatch");
    }
    return solution;
}

double CILQRSolver::cal_cost_with_logging(const Solution& solution, int iteration) {
    // 检查轨迹是否包含NaN值
    for (const auto& state : solution.ego_trj.get_states()) {
        if (std::isnan(state[0]) || std::isnan(state[1]) || std::isnan(state[2]) || std::isnan(state[3])) {
            std::cerr << "NaN detected in trajectory!" << std::endl;
            return std::numeric_limits<double>::infinity();
        }
    }
    for (const auto& control : solution.control_sequence.get_control_sequence()) {
        if (std::isnan(control[0]) || std::isnan(control[1])) {
            std::cerr << "NaN detected in control sequence!" << std::endl;
            return std::numeric_limits<double>::infinity();
        }
    }
    if (solution.ego_trj.get_states().empty()) {
        throw std::runtime_error("trajectory contains NaN values");
    }
    Vector2d P2; P2 << 0, 1;
    const auto& control_sequence = solution.control_sequence.get_control_sequence();
    const auto& trj = solution.ego_trj.get_states();

    // 总代价分类
    double J_state_total = 0;
    double J_ctrl_total = 0;
    double J_obs_total = 0;
    double J_lane_total = 0;
    double J_speed_rate_total = 0;
    double J_gamma_barrier_total = 0;
    double J_gamma_dot_barrier_total = 0;
    
    // 详细分解的代价
    double J_position_total = 0;      // 位置代价 (x,y)
    double J_heading_total = 0;       // 航向代价 (theta)
    double J_gamma_state_total = 0;   // 铰接角状态代价
    double J_lateral_ref_total = 0;   // 横向偏移参考代价
    double J_velocity_total = 0;      // 速度控制代价
    double J_gamma_ctrl_total = 0;    // 铰接角控制代价

    // 状态相关代价
    for (int i = 0; i < arg.N + 1; ++i) {
        // 每个时间步重置临时代价
        double cost_state = 0;
        double cost_state_ref = 0;
        double cost_lane = 0;
        double cost_obs = 0;
        double cost_gamma_barrier = 0;
        
        // 详细分解的临时代价
        double cost_position = 0;
        double cost_heading = 0;
        double cost_gamma_state = 0;

        const State& X = trj[i];
        size_t index = find_closest_point(ego.get_local_plan().get_points(), X);
        size_t match_index = index == ego.get_local_plan().get_points().size() - 1 ? index : index + 1;
        const Point& X_r_point = ego.get_local_plan().get_points()[match_index];
        State X_r = {X_r_point.x, X_r_point.y, X_r_point.heading, 0};
        State X_e = X - X_r;
        
        // 分解状态代价到各个分量
        cost_position = (X_e[0] * X_e[0] * arg.Q(0,0) + X_e[1] * X_e[1] * arg.Q(1,1));
        cost_heading = X_e[2] * X_e[2] * arg.Q(2,2);
        cost_gamma_state = X_e[3] * X_e[3] * arg.Q(3,3);
        cost_state = cost_position + cost_heading + cost_gamma_state;

        // 横向偏移参考代价
        Vector2d dX, nor_r; dX << X_e[0], X_e[1];
        nor_r << -sin(X_r_point.heading), cos(X_r_point.heading);
        cost_state_ref = pow(dX.dot(nor_r), 2) * arg.ref_weight;

        // 车道边界代价
        if (arg.if_cal_lane_cost) {
            double l = dX.transpose() * nor_r;
            double c_left = l - arg.trace_safe_width_left;
            double c_right = -l - arg.trace_safe_width_right;
            double cost_lane_left = arg.lane_q1 * exp(arg.lane_q2 * c_left);
            double cost_lane_right = arg.lane_q1 * exp(arg.lane_q2 * c_right);
            cost_lane = cost_lane_left + cost_lane_right;
        }

        // 障碍物代价（多障碍物累加）
        if (arg.if_cal_obs_cost) {
            for (size_t obs_idx = 0; obs_idx < obs_list.size(); ++obs_idx) {
                const Trajectory& obs = obs_list[obs_idx];
                if (i >= obs.get_states().size()) continue; // 长度保护
                const State& obs_state = obs.get_states()[i];
                double dx = X[0] - obs_state[0];
                double dy = X[1] - obs_state[1];
                double a = arg.obs_length/2 + ego.get_model().ego_rad/2 + arg.safe_a_buffer;
                double b = arg.obs_width/2 + ego.get_model().ego_rad/2 + arg.safe_b_buffer;
                Vector2d dX_obs(dx, dy);
                Matrix2d R; R << cos(obs_state[2]), sin(obs_state[2]), -sin(obs_state[2]), cos(obs_state[2]);
                Vector2d dX_obs_cord = R * dX_obs;
                double c = 1 - (pow(dX_obs_cord[0],2)/pow(a,2) + pow(dX_obs_cord[1],2)/pow(b,2));
                cost_obs += arg.obs_q1 * exp(arg.obs_q2 * c);
            }
        }

        // 状态gamma的barrier代价（上下限）
        if (arg.if_cal_gamma_barrier) {
            double gamma = X[3];
            double c_max = gamma - arg.gamma_max;
            double c_min = arg.gamma_min - gamma;
            Vector4d e4; e4 << 0, 0, 0, 1;
            auto [b_max, db_max_unused, ddb_max_unused] = barrierFunction(arg.gamma_max_q1, arg.gamma_max_q2, c_max, e4);
            auto [b_min, db_min_unused, ddb_min_unused] = barrierFunction(arg.gamma_min_q1, arg.gamma_min_q2, c_min, -e4);
            cost_gamma_barrier = b_max + b_min;
        }

        // 累加到总代价
        J_state_total += cost_state + cost_state_ref;
        J_position_total += cost_position;
        J_heading_total += cost_heading;
        J_gamma_state_total += cost_gamma_state;
        J_lateral_ref_total += cost_state_ref;
        J_lane_total += cost_lane;
        J_obs_total += cost_obs;
        J_gamma_barrier_total += cost_gamma_barrier;
    }

    // 控制相关代价
    for (int i = 0; i < arg.N; ++i) {
        const Control& U = control_sequence[i];
        Control U_ref = {arg.desire_speed, 0};
        Control U_e = U - U_ref;
        
        // 分解控制代价
        double cost_velocity = U_e[0] * U_e[0] * arg.R(0,0);
        double cost_gamma_ctrl = U_e[1] * U_e[1] * arg.R(1,1);
        double cost_ctrl = cost_velocity + cost_gamma_ctrl;

        // 控制gamma_dot的barrier代价（上下限）
        double cost_gamma_dot_barrier = 0.0;
        if (arg.if_cal_gamma_dot_barrier) {
            double gamma_dot = U[1];
            double c_max = gamma_dot - arg.gamma_dot_max;
            double c_min = arg.gamma_dot_min - gamma_dot;
            auto [b_max, db_max_unused, ddb_max_unused] = barrierFunction(arg.gamma_dot_max_q1, arg.gamma_dot_max_q2, c_max, P2);
            auto [b_min, db_min_unused, ddb_min_unused] = barrierFunction(arg.gamma_dot_min_q1, arg.gamma_dot_min_q2, c_min, -P2);
            cost_gamma_dot_barrier = b_max + b_min;
        }

        J_ctrl_total += cost_ctrl;
        J_velocity_total += cost_velocity;
        J_gamma_ctrl_total += cost_gamma_ctrl;
        J_gamma_dot_barrier_total += cost_gamma_dot_barrier;
    }

    // 速度变化率代价
    if (arg.if_cal_speed_rate_cost) {
        for (int i = 0; i < arg.N - 1; ++i) {
            const Control& U_cur = control_sequence[i];
            const Control& U_next = control_sequence[i + 1];
            double v_diff = U_next[0] - U_cur[0]; // 速度差
            J_speed_rate_total += arg.v_rate_weight * v_diff * v_diff;
        }
    }

    double J_constraint_total = J_obs_total + J_lane_total + J_gamma_barrier_total + J_gamma_dot_barrier_total;
    double J_total = J_state_total + J_ctrl_total + J_constraint_total + J_speed_rate_total;
    
    // 详细的代价分解输出
    // std::cout << "\n=== 迭代 " << iteration << " 详细代价分解 ===" << std::endl;
    // std::cout << "状态代价分解:" << std::endl;
    // std::cout << "  位置代价 (x,y)     = " << std::fixed << std::setprecision(6) << J_position_total << std::endl;
    // std::cout << "  航向代价 (theta)   = " << std::fixed << std::setprecision(6) << J_heading_total << std::endl;
    // std::cout << "  铰接角状态代价     = " << std::fixed << std::setprecision(6) << J_gamma_state_total << std::endl;
    // std::cout << "  横向偏移参考代价   = " << std::fixed << std::setprecision(6) << J_lateral_ref_total << std::endl;
    // std::cout << "  状态代价小计       = " << std::fixed << std::setprecision(6) << J_state_total << std::endl;
    
    // std::cout << "\n控制代价分解:" << std::endl;
    // std::cout << "  速度控制代价       = " << std::fixed << std::setprecision(6) << J_velocity_total << std::endl;
    // std::cout << "  铰接角控制代价     = " << std::fixed << std::setprecision(6) << J_gamma_ctrl_total << std::endl;
    // std::cout << "  控制代价小计       = " << std::fixed << std::setprecision(6) << J_ctrl_total << std::endl;
    
    // std::cout << "\n约束代价分解:" << std::endl;
    // std::cout << "  障碍物代价         = " << std::fixed << std::setprecision(6) << J_obs_total << std::endl;
    // std::cout << "  车道边界代价       = " << std::fixed << std::setprecision(6) << J_lane_total << std::endl;
    // std::cout << "  铰接角约束代价     = " << std::fixed << std::setprecision(6) << J_gamma_barrier_total << std::endl;
    // std::cout << "  铰接角速度约束代价 = " << std::fixed << std::setprecision(6) << J_gamma_dot_barrier_total << std::endl;
    // std::cout << "  约束代价小计       = " << std::fixed << std::setprecision(6) << J_constraint_total << std::endl;
    
    // std::cout << "\n其他代价:" << std::endl;
    // std::cout << "  速度变化率代价     = " << std::fixed << std::setprecision(6) << J_speed_rate_total << std::endl;
    
    // std::cout << "\n总代价汇总:" << std::endl;
    // std::cout << "  总代价             = " << std::fixed << std::setprecision(6) << J_total << std::endl;
    // std::cout << "  Lambda值           = " << std::fixed << std::setprecision(6) << lamb << std::endl;
    // std::cout << "===================" << std::endl;
    
    // 记录到日志文件
    log_cost_breakdown(iteration, J_total, 
                      J_position_total, J_heading_total, J_gamma_state_total, J_lateral_ref_total,
                      J_velocity_total, J_gamma_ctrl_total,
                      J_obs_total, J_lane_total, J_gamma_barrier_total, J_gamma_dot_barrier_total,
                      J_speed_rate_total, lamb);
    
    return J_total;
}

void CILQRSolver::compute_df(const Solution& solution) {
    const auto& X_traj = solution.ego_trj.get_states();
    const auto& U_seq = solution.control_sequence.get_control_sequence();
    const int N = arg.N;

    // 确保容器大小正确
    if (static_cast<int>(df_dx.size()) != N) df_dx.assign(N, Matrix4d::Zero());
    if (static_cast<int>(df_du.size()) != N) df_du.assign(N, Matrix<double,4,2>::Zero());

    auto model = ego.get_model();
    for (int i = 0; i < N; ++i) {
        const Vector4d& X = X_traj[i];
        const Vector2d& U = U_seq[i];
        df_dx[i] = model.get_jacobian_state(X, U);
        df_du[i] = model.get_jacobian_control(X, U);
    }
}
void CILQRSolver::compute_cost_derivatives(const Solution& solution) {
    const auto& X_traj = solution.ego_trj.get_states();
    const auto& U = solution.control_sequence.get_control_sequence();
    const auto& local_plan = ego.get_local_plan().get_points();
    const int N = arg.N;

    Vector2d P2(0, 1);

    // 状态相关导数
    for (int i = 0; i <= N; ++i) {
        const State& X = X_traj[i];
        size_t index = find_closest_point(local_plan, X);
        size_t match_index = index == ego.get_local_plan().get_points().size() - 1 ? index : index + 1;
        const Point& X_r_point = local_plan[match_index];
        State X_r; X_r << X_r_point.x, X_r_point.y, X_r_point.heading, 0;
        State X_e = X - X_r;

        Vector4d l_dx = 2 * arg.Q * X_e;
        Matrix4d l_ddx = 2 * arg.Q;

        Vector2d dX(X_e[0], X_e[1]);
        Vector2d nor_r(-std::sin(X_r_point.heading), std::cos(X_r_point.heading));
        Vector4d l_dx_ref(-2*dX.dot(nor_r)*std::sin(X_r_point.heading),
                           2*dX.dot(nor_r)*std::cos(X_r_point.heading), 0, 0);
        Matrix4d l_ddx_ref;
        l_ddx_ref << 2*pow(sin(X_r_point.heading),2), -sin(2*X_r_point.heading), 0, 0,
                    -sin(2*X_r_point.heading), 2*pow(cos(X_r_point.heading),2), 0, 0,
                    0, 0, 0, 0,
                    0, 0, 0, 0;
        l_dx_ref *= arg.ref_weight;
        l_ddx_ref *= arg.ref_weight;

        // 每步重置障碍物导数
        Vector4d db_obs = Vector4d::Zero();
        Matrix4d ddb_obs = Matrix4d::Zero();

        if (arg.if_cal_obs_cost) {
            for (size_t obs_idx = 0; obs_idx < obs_list.size(); ++obs_idx) {
                const Trajectory& obs = obs_list[obs_idx];
                if (i >= obs.get_states().size()) continue;
                const State& obs_state = obs.get_states()[i];
                double dx = X[0] - obs_state[0];
                double dy = X[1] - obs_state[1];
                double a = arg.obs_length/2 + ego.get_model().ego_rad/2 + arg.safe_a_buffer;
                double b = arg.obs_width/2 + ego.get_model().ego_rad/2 + arg.safe_b_buffer;
                Vector2d dX_obs(dx, dy);
                Matrix2d R; R << cos(obs_state[2]), sin(obs_state[2]), -sin(obs_state[2]), cos(obs_state[2]);
                Vector2d dX_obs_cord = R * dX_obs;
                double c = 1 - (pow(dX_obs_cord[0],2)/pow(a,2) + pow(dX_obs_cord[1],2)/pow(b,2));
                Vector2d grad_local(-2 * dX_obs_cord[0] / (a*a), -2 * dX_obs_cord[1] / (b*b));
                Vector2d grad_global = R.transpose() * grad_local;
                Vector4d c_dot; c_dot << grad_global[0], grad_global[1], 0.0, 0.0;
                double exp_term = arg.obs_q1 * arg.obs_q2 * std::exp(arg.obs_q2 * c);
                db_obs += exp_term * c_dot;
                ddb_obs += exp_term * arg.obs_q2 * c_dot * c_dot.transpose();
            }
        }

        // 车道保持代价导数（左右边界）
        Vector4d db_lane_total = Vector4d::Zero();
        Matrix4d ddb_lane_total = Matrix4d::Zero();
        if (arg.if_cal_lane_cost) {
            double l = dX.dot(nor_r);
            double c_left = l - arg.trace_safe_width_left;
            Vector4d dc_left; dc_left << -std::sin(X_r_point.heading), std::cos(X_r_point.heading), 0, 0;
            auto [b_left, db_left, ddb_left] = barrierFunction(arg.lane_q1, arg.lane_q2, c_left, dc_left);

            double c_right = -l - arg.trace_safe_width_right;
            Vector4d dc_right = -dc_left;
            auto [b_right, db_right, ddb_right] = barrierFunction(arg.lane_q1, arg.lane_q2, c_right, dc_right);

            db_lane_total = db_left + db_right;
            ddb_lane_total = ddb_left + ddb_right;
        }

        // 新增：状态gamma的barrier导数
        Vector4d db_gamma_total = Vector4d::Zero();
        Matrix4d ddb_gamma_total = Matrix4d::Zero();
        if (arg.if_cal_gamma_barrier) {
            double gamma = X[3];
            double c_max = gamma - arg.gamma_max;
            double c_min = arg.gamma_min - gamma;
            Vector4d e4; e4 << 0, 0, 0, 1;
            auto [b_max, db_max, ddb_max] = barrierFunction(arg.gamma_max_q1, arg.gamma_max_q2, c_max, e4);
            auto [b_min, db_min, ddb_min] = barrierFunction(arg.gamma_min_q1, arg.gamma_min_q2, c_min, -e4);
            db_gamma_total = db_max + db_min;
            ddb_gamma_total = ddb_max + ddb_min;
        }

        this->lx[i] = l_dx + l_dx_ref + db_obs + db_lane_total + db_gamma_total;
        this->lxx[i] = l_ddx + l_ddx_ref + ddb_obs + ddb_lane_total + ddb_gamma_total;
        // 注意：终端时刻 i==N 没有控制量，避免对 lux[N] 赋值越界

    }

    // 控制相关导数
    for (int i = 0; i < N; ++i) {
        const Vector2d u = U[i];
        const Vector2d u_r(arg.desire_speed, 0);
        const Vector2d u_e = u - u_r;

        Vector2d lu_base = 2 * arg.R * u_e;
        Matrix2d luu_base = 2 * arg.R;

        // gamma_dot上下限barrier导数
        Vector2d db_gamma_dot = Vector2d::Zero();
        Matrix2d ddb_gamma_dot = Matrix2d::Zero();

        if (arg.if_cal_gamma_dot_barrier) {
            double c_max = u.dot(P2) - arg.gamma_dot_max;
            auto [b_max, db_max, ddb_max] = barrierFunction(arg.gamma_dot_max_q1, arg.gamma_dot_max_q2, c_max, P2);
            double c_min = arg.gamma_dot_min - u.dot(P2);
            auto [b_min, db_min, ddb_min] = barrierFunction(arg.gamma_dot_min_q1, arg.gamma_dot_min_q2, c_min, -P2);
            db_gamma_dot = db_max + db_min;
            ddb_gamma_dot = ddb_max + ddb_min;
        }

        this->lu[i] = lu_base + db_gamma_dot;
        this->luu[i] = luu_base + ddb_gamma_dot;
        // 控制-状态交叉项（当前实现为零）
        this->lux[i] = Matrix<double,2,4>::Zero();
    }

    // 速度变化率代价导数
    if (arg.if_cal_speed_rate_cost) {
        for (int i = 0; i < N; ++i) {
            Vector2d lu_speed_rate = Vector2d::Zero();
            Matrix2d luu_speed_rate = Matrix2d::Zero();

            // 当前时刻作为 U_cur 的贡献
            if (i < N - 1) {
                const Vector2d& U_cur = U[i];
                const Vector2d& U_next = U[i + 1];
                double v_diff = U_next[0] - U_cur[0];
                lu_speed_rate[0] += -2 * arg.v_rate_weight * v_diff;
                luu_speed_rate(0, 0) += 2 * arg.v_rate_weight;
            }

            // 当前时刻作为 U_next 的贡献
            if (i > 0) {
                const Vector2d& U_prev = U[i - 1];
                const Vector2d& U_cur = U[i];
                double v_diff = U_cur[0] - U_prev[0];
                lu_speed_rate[0] += 2 * arg.v_rate_weight * v_diff;
                luu_speed_rate(0, 0) += 2 * arg.v_rate_weight;
            }

            this->lu[i] += lu_speed_rate;
            this->luu[i] += luu_speed_rate;
        }
    }
}

void CILQRSolver::backward() {
    const int N = arg.N;
    const double lambda = this->lamb;

    // 初始化价值函数的导数
    Vector4d V_x = lx[N];          // 最终状态梯度
    Matrix4d V_xx = lxx[N];        // 最终状态Hessian
    
    // 清空控制修正量
    this->k = std::vector<MatrixXd>(N, Vector2d::Zero());
    this->K = std::vector<MatrixXd>(N, MatrixXd::Zero(2,4));

    Matrix4d df_dx;
    Matrix<double,4,2> df_du;
    Vector4d Qx;
    Matrix4d Qxx;
    Matrix<double,2,4> Qux;
    Vector2d singular_values;
    Matrix2d U;
    Matrix2d V;
    Matrix2d Quu_inv;
    
    // 调试统计变量
    int cholesky_failures = 0;
    int svd_uses = 0;
    double max_singular_ratio = 0.0;
    double min_singular_value = 1e10;
    double max_k_norm = 0.0;
    double max_K_norm = 0.0;
    
    // 反向迭代
    for (int i = N-1; i >= 0; --i) { // 注意从N-1开始
        // 获取当前时刻的雅可比矩阵
        df_dx = this->df_dx[i];
        df_du = this->df_du[i];
        
        // 计算Q函数相关量
        Qx = lx[i] + df_dx.transpose() * V_x;
        Qu[i] = lu[i] + df_du.transpose() * V_x;
        
        Qxx = lxx[i] + df_dx.transpose() * V_xx * df_dx;
        Qux = lux[i] + df_du.transpose() * V_xx * df_dx;
        Quu[i] = luu[i] + df_du.transpose() * V_xx * df_du;

        // 计算正则化的 Quu
        Matrix2d Quu_reg = Quu[i] + lambda * Matrix2d::Identity();
        
        // 检查Quu矩阵条件数
        double quu_det = Quu_reg.determinant();
        double quu_trace = Quu_reg.trace();

        // 使用 Cholesky 分解求逆（若正定）
        Eigen::LLT<Matrix2d> llt(Quu_reg);
        if (llt.info() == Eigen::Success) {
            Quu_inv = llt.solve(Matrix2d::Identity());
        } else {
            cholesky_failures++;
            // 退化到 SVD 分解
            JacobiSVD<Matrix2d> svd(Quu_reg, ComputeFullU | ComputeFullV);
            singular_values = svd.singularValues();
            U = svd.matrixU();
            V = svd.matrixV();
            
            // 记录奇异值统计
            double max_sv = singular_values.maxCoeff();
            double min_sv = singular_values.minCoeff();
            min_singular_value = std::min(min_singular_value, min_sv);
            if (max_sv > 1e-12) {
                max_singular_ratio = std::max(max_singular_ratio, max_sv / min_sv);
            }
            svd_uses++;
            
            singular_values = singular_values.cwiseMax(1e-8);  // 下限截断
            Quu_inv = V * singular_values.cwiseInverse().asDiagonal() * U.transpose();
        }

        // 计算控制修正量
        this->k[i] = -Quu_inv * Qu[i];
        this->K[i] = -Quu_inv * Qux;
        
        // 记录控制修正量统计
        double k_norm = k[i].norm();
        double K_norm = K[i].norm();
        max_k_norm = std::max(max_k_norm, k_norm);
        max_K_norm = std::max(max_K_norm, K_norm);
        
        // 检查异常值
        // if (k_norm > 100 || K_norm > 100) {
        //     std::cout << "WARNING: 步骤 " << i << " 控制修正量异常大: k_norm=" << k_norm 
        //               << ", K_norm=" << K_norm << ", lambda=" << lambda 
        //               << ", Quu_det=" << quu_det << std::endl;
        // }

        // 更新价值函数导数
        V_x = Qx - K[i].transpose() * Quu[i] * k[i];
        V_xx = Qxx - K[i].transpose() * Quu[i] * K[i];
        
        // 检查价值函数导数的数值稳定性
        // if (V_x.hasNaN() || V_xx.hasNaN()) {
        //     std::cout << "ERROR: 步骤 " << i << " 价值函数导数包含NaN!" << std::endl;
        // }
    }
    
    // 输出backward阶段统计信息
    // std::cout << "\n=== Backward阶段诊断 ===" << std::endl;
    // std::cout << "Lambda值: " << lambda << std::endl;
    // std::cout << "Cholesky失败次数: " << cholesky_failures << "/" << N << std::endl;
    // std::cout << "SVD使用次数: " << svd_uses << "/" << N << std::endl;
    // std::cout << "最小奇异值: " << min_singular_value << std::endl;
    // std::cout << "最大条件数: " << max_singular_ratio << std::endl;
    // std::cout << "最大k范数: " << max_k_norm << std::endl;
    // std::cout << "最大K范数: " << max_K_norm << std::endl;
    // std::cout << "========================" << std::endl;
}

Solution CILQRSolver::forward(const Solution& cur_solution){
        
    // 初始化线搜索参数
    const int max_iterations = 10;
    double alpha = 0.5;  // 优化：从1.0改为0.5，更保守的初始步长
    bool found = false;
    double J_old = cal_cost_with_logging(cur_solution, -1);  // -1 表示line search阶段
    double J_new = 0;
    double delta_cost = 0;
    double delta_V = 0;
    auto  U = cur_solution.control_sequence.get_control_sequence();
    auto  X = cur_solution.ego_trj.get_states();
    static Solution new_solution;
    std::vector<Control> U_tmp;
    std::vector<State> X_tmp;
    Vector4d delta_x;
    average_gradient = 0;
    
    // 调试统计变量
    double initial_alpha = alpha;
    double final_alpha = alpha;
    double best_cost_reduction = 0.0;
    int successful_iter = -1;
    std::vector<double> alpha_history;
    std::vector<double> cost_history;
    std::vector<double> delta_V_history;
    
    // std::cout << "\n=== Forward线搜索开始 ===" << std::endl;
    // std::cout << "初始代价: " << J_old << std::endl;
    
    for (int iter = 0; iter < max_iterations; ++iter) {
        // 临时存储新控制序列
        U_tmp = U;
        X_tmp = X;
        delta_V = 0;  // 重置预期代价变化

        double crash_penalty = 0.0;

        // 应用控制修正
        for (int i = 0; i < arg.N; ++i) {
            // 计算状态偏差
            delta_x = X_tmp[i] - cur_solution.ego_trj.get_states()[i];
            // 应用控制修正：u_new = u_old + alpha*k + K*delta_x
            U_tmp[i] += alpha * k[i]+ K[i] * delta_x;
            
            // Clamp controls to physical limits first
            U_tmp[i][0] = std::clamp(U_tmp[i][0], -5.0, 30.0);  // Velocity limit
            U_tmp[i][1] = std::clamp(U_tmp[i][1], -1.5, 1.5);   // Steering rate limit
            
            // Clamp acceleration (rate of change of velocity)
            // Ensure prev_v is within limits to avoid "sticking" to bad values
            double raw_prev_v = (i == 0) ? cur_solution.control_sequence.get_control_sequence()[0][0] : U_tmp[i-1][0];
            double prev_v = std::clamp(raw_prev_v, -5.0, 30.0);
            
            double v_curr = U_tmp[i][0];
            double max_v = prev_v + arg.acc_max * arg.dt;
            double min_v = prev_v + arg.acc_min * arg.dt;
            
            // Intersect acceleration limits with absolute limits
            max_v = std::min(max_v, 30.0);
            min_v = std::max(min_v, -5.0);
            
            U_tmp[i][0] = std::clamp(v_curr, min_v, max_v);

            // 前向模拟
            X_tmp[i+1] = ego.get_model().dynamics(X_tmp[i], U_tmp[i]);
            
            // Sanity check: if state explodes, add huge penalty but don't abort immediately
            // This allows the solver to find a gradient to recover from the crash
            if (std::abs(X_tmp[i+1][3]) > 2.0) {
                 crash_penalty += 1e6 * (std::abs(X_tmp[i+1][3]) - 2.0);
            }
            if (std::abs(U_tmp[i][0]) > 40.0) {
                 crash_penalty += 1e6 * (std::abs(U_tmp[i][0]) - 40.0);
            }
            if (std::abs(X_tmp[i+1][0]) > 20000.0) {
                 crash_penalty += 1e6;
            }

            //累计deltaV
            delta_V += alpha * (k[i].transpose() * Qu[i]).value() + alpha * alpha * 0.5 * (k[i].transpose() * Quu[i] * k[i]).value(); 
            average_gradient += k[i].maxCoeff() / (U_tmp[i].norm() + 1);
        }

        // 计算新代价
        new_solution.ego_trj.states.swap(X_tmp);
        new_solution.control_sequence.controls.swap(U_tmp);
        J_new = cal_cost_with_logging(new_solution, -2);  // -2 表示line search内部迭代
        J_new += crash_penalty;

        
        delta_cost = J_new - J_old;
        
        // 记录历史数据
        alpha_history.push_back(alpha);
        cost_history.push_back(J_new);
        delta_V_history.push_back(delta_V);
        
        // 计算实际vs预期的代价变化比率
        double cost_ratio = (std::abs(delta_V) > 1e-12) ? delta_cost / delta_V : 1e10;
        
        std::cout << "  迭代 " << iter << ": alpha=" << std::fixed << std::setprecision(6) << alpha 
                  << ", J_new=" << J_new << ", delta_cost=" << delta_cost 
                  << ", delta_V=" << delta_V << ", ratio=" << cost_ratio << std::endl;
        
        // 接受条件判断
        // 优化：改进接受条件，使用Armijo条件和更严格的数值稳定性检查
        double armijo_c1 = 0.1;  // Armijo常数
        double expected_reduction = armijo_c1 * alpha * delta_V;
        
        if (delta_cost < expected_reduction && delta_cost < 0 && 
            std::abs(delta_cost) > 1e-12 && std::isfinite(J_new)) {
            found = true;
            successful_iter = iter;
            final_alpha = alpha;
            best_cost_reduction = -delta_cost;
            // std::cout << "  ✓ 接受步长 alpha=" << alpha << ", 代价降低=" << best_cost_reduction << std::endl;
            break;
        } else {
            // std::cout << "  ✗ 拒绝步长，原因: ";
            // if (!std::isfinite(J_new)) {
            //     std::cout << "数值不稳定 (J_new=" << J_new << ")";
            // } else if (delta_cost >= 0) {
            //     std::cout << "代价增加 (delta_cost=" << delta_cost << ")";
            // } else if (delta_cost >= expected_reduction) {
            //     std::cout << "不满足Armijo条件 (delta_cost=" << delta_cost << ", expected=" << expected_reduction << ")";
            // } else if (std::abs(delta_cost) <= 1e-12) {
            //     std::cout << "改进太小 (|delta_cost|=" << std::abs(delta_cost) << ")";
            // }
            // std::cout << std::endl;
            
            // 优化：改进步长衰减策略
            if (iter < 3) {
                alpha *= 0.5;  // 前3次使用0.5衰减
            } else {
                alpha *= 0.25; // 后续使用更激进的0.25衰减
            }
            
            // 添加最小步长检查
            if (alpha < 1e-8) {
                // std::cout << "  步长过小，终止搜索" << std::endl;
                break;
            }
        }
    }

    average_gradient = average_gradient / (arg.N - 1);
    
    // 输出forward阶段统计信息
    // std::cout << "\n=== Forward阶段诊断 ===" << std::endl;
    // std::cout << "线搜索结果: " << (found ? "成功" : "失败") << std::endl;
    // if (found) {
    //     std::cout << "成功迭代: " << successful_iter << std::endl;
    //     std::cout << "最终步长: " << final_alpha << std::endl;
    //     std::cout << "代价降低: " << best_cost_reduction << std::endl;
    // } else {
    //     std::cout << "所有步长均被拒绝" << std::endl;
    //     std::cout << "最小尝试步长: " << alpha << std::endl;
    // }
    // std::cout << "平均梯度: " << average_gradient << std::endl;
    // std::cout << "初始步长: " << initial_alpha << std::endl;
    // std::cout << "尝试次数: " << alpha_history.size() << "/" << max_iterations << std::endl;
    
    // 显示步长历史
    // if (alpha_history.size() > 1) {
        // std::cout << "步长历史: ";
        // for (size_t i = 0; i < std::min(size_t(5), alpha_history.size()); ++i) {
            // std::cout << alpha_history[i/] << " ";
        // }
        // if (alpha_history.size() > 5) std::cout << "...";
        // std::cout << std::endl;
    // }
    std::cout << "========================" << std::endl;
    
    if (!found) {
        return cur_solution;
    }
    else{
        return new_solution;
    }
}


// 铰接车辆纯跟踪方法
Control CILQRSolver::pure_pursuit(const State& X_cur) {
    const auto& local_plan = ego.get_local_plan().get_points();
    if (local_plan.empty()) {
        throw std::runtime_error("Local plan is empty!");
    }

    // 1. 查找最近点
    size_t indexNow = find_closest_point(local_plan, X_cur);

    // 2. 计算前瞻距离（基于当前速度，而不是铰接角）
    const double Kv = arg.kv;    
    const double Ld0 = arg.ld0;  
    const double Ld_min = arg.ld_min;
    const double Ld_max = arg.ld_max;
    
    // 使用期望速度计算前瞻距离，避免使用状态中的铰接角
    double current_speed = std::max(1.0, arg.desire_speed); 
    double Ld = std::clamp(Kv * current_speed + Ld0, Ld_min, Ld_max);

    // 3. 查找目标点
    size_t indexTarget = indexNow;
    double accumulated_dist = 0.0;
    for (size_t i = indexNow; i < local_plan.size(); ++i) {
        if (i > indexNow) {
            accumulated_dist += distance(local_plan[i], local_plan[i-1]);
        }
        if (accumulated_dist >= Ld) {
            indexTarget = i;
            break;
        }
    }
    if (indexTarget >= local_plan.size()) {
        indexTarget = local_plan.size() - 1;
    }

    // 4. 计算期望的航向角变化率
    const Point& target = local_plan[indexTarget];
    double dx = target.x - X_cur[0];
    double dy = target.y - X_cur[1];
    double desired_theta = std::atan2(dy, dx);
    double theta_error = angle_wrap(desired_theta - X_cur[2]);
    
    // 期望的航向角变化率（简单比例控制）
    double desired_theta_dot = arg.kp * theta_error;
    
    // 5. 根据铰接车辆动力学反推所需的铰接角速度
    // 从动力学方程: theta_dot = -(v * sin(gamma) + lr * gamma_dot) / len
    // 其中 len = lr + lf * cos(gamma)
    
    double current_gamma = X_cur[3];  // 当前铰接角
    double v = arg.desire_speed;      // 期望速度
    double lf = ego.get_model().lf;
    double lr = ego.get_model().lr;
    
    double len = lr + lf * std::cos(current_gamma);
    
    // 避免除零
    if (std::abs(len) < 0.1) {
        len = 0.1;
    }
    
    // 反推所需的铰接角速度
    // theta_dot = -(v * sin(gamma) + lr * gamma_dot) / len
    // => gamma_dot = -(theta_dot * len + v * sin(gamma)) / lr
    double required_gamma_dot = -(desired_theta_dot * len + v * std::sin(current_gamma)) / lr;
    
    // 限制铰接角速度在合理范围内
    required_gamma_dot = std::clamp(required_gamma_dot, 
                                   -arg.gamma_dot_max * 0.8, 
                                    arg.gamma_dot_max * 0.8);

    // 生成控制指令：期望速度和计算出的铰接角速度
    return Control(v, required_gamma_dot);
}


// 日志记录方法实现
void CILQRSolver::init_cost_logging() {
    if (!enable_logging) return;
    
    // 生成带时间戳的日志文件名
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto tm = *std::localtime(&time_t);
    
    std::ostringstream oss;
    oss << solver_name << "_cost_analysis_" << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".csv";
    std::string filename = oss.str();
    
    // Ensure output directory exists
    std::string relative_dir = "outputs/" + solver_name + "/logs";
    std::string full_dir_path = resolve_resource_path(relative_dir);
    
    ensure_directory_exists(full_dir_path);
    log_filename = full_dir_path + "/" + filename;
    
    cost_log_file.open(log_filename);
    if (cost_log_file.is_open()) {
        // 写入CSV头部
        cost_log_file << "Iteration,Total_Cost,Position_Cost,Heading_Cost,Gamma_State_Cost,Lateral_Ref_Cost,"
                     << "Velocity_Cost,Gamma_Ctrl_Cost,Obstacle_Cost,Lane_Cost,Gamma_Barrier_Cost,"
                     << "Gamma_Dot_Barrier_Cost,Speed_Rate_Cost,Lambda_Value" << std::endl;
        std::cout << "代价分析日志文件已创建: " << log_filename << std::endl;
    } else {
        std::cerr << "警告: 无法创建日志文件 " << log_filename << std::endl;
        enable_logging = false;
    }
}

void CILQRSolver::log_cost_breakdown(int iteration, double total_cost, 
                                   double J_position, double J_heading, double J_gamma_state, double J_lateral_ref,
                                   double J_velocity, double J_gamma_ctrl, 
                                   double J_obs, double J_lane, double J_gamma_barrier, double J_gamma_dot_barrier,
                                   double J_speed_rate, double lambda_value) {
    if (!enable_logging || !cost_log_file.is_open()) return;
    
    cost_log_file << iteration << "," << total_cost << "," << J_position << "," << J_heading << ","
                 << J_gamma_state << "," << J_lateral_ref << "," << J_velocity << "," << J_gamma_ctrl << ","
                 << J_obs << "," << J_lane << "," << J_gamma_barrier << "," << J_gamma_dot_barrier << ","
                 << J_speed_rate << "," << lambda_value << std::endl;
    
    // 强制刷新缓冲区，确保数据及时写入
    cost_log_file.flush();
}

void CILQRSolver::close_cost_logging() {
    if (cost_log_file.is_open()) {
        cost_log_file.close();
        if (enable_logging) {
            std::cout << "代价分析日志已保存到: " << log_filename << std::endl;
        }
    }
}

void ALILQRSolver::initialize_penalties() {
    // Initialize penalties with default values
    std::fill(mu_gamma_max.begin(), mu_gamma_max.end(), 10.0);
    std::fill(mu_gamma_min.begin(), mu_gamma_min.end(), 10.0);
    std::fill(mu_obs.begin(), mu_obs.end(), 10.0);
    std::fill(mu_lane.begin(), mu_lane.end(), 10.0);
    std::fill(mu_gdot_max.begin(), mu_gdot_max.end(), 10.0);
    std::fill(mu_gdot_min.begin(), mu_gdot_min.end(), 10.0);
    
    // Initialize multipliers to zero
    std::fill(lambda_gamma_max.begin(), lambda_gamma_max.end(), 0.0);
    std::fill(lambda_gamma_min.begin(), lambda_gamma_min.end(), 0.0);
    std::fill(lambda_obs.begin(), lambda_obs.end(), 0.0);
    std::fill(lambda_lane.begin(), lambda_lane.end(), 0.0);
    std::fill(lambda_gdot_max.begin(), lambda_gdot_max.end(), 0.0);
    std::fill(lambda_gdot_min.begin(), lambda_gdot_min.end(), 0.0);
}

void ALILQRSolver::shift_penalties() {
    // Shift state constraint multipliers (size N+1)
    // lambda[i] = lambda[i+1] for i=0..N-1
    // lambda[N] = lambda[N] (duplicate last)
    for (int i = 0; i < arg.N; ++i) {
        lambda_gamma_max[i] = lambda_gamma_max[i+1];
        mu_gamma_max[i] = mu_gamma_max[i+1];
        
        lambda_gamma_min[i] = lambda_gamma_min[i+1];
        mu_gamma_min[i] = mu_gamma_min[i+1];
        
        lambda_obs[i] = lambda_obs[i+1];
        mu_obs[i] = mu_obs[i+1];
        
        lambda_lane[i] = lambda_lane[i+1];
        mu_lane[i] = mu_lane[i+1];
    }
    // Duplicate last element for state constraints
    // (Already holds value from previous step's N, which is fine as a guess)
    
    // Shift control constraint multipliers (size N)
    // lambda[i] = lambda[i+1] for i=0..N-2
    // lambda[N-1] = lambda[N-1] (duplicate last)
    for (int i = 0; i < arg.N - 1; ++i) {
        lambda_gdot_max[i] = lambda_gdot_max[i+1];
        mu_gdot_max[i] = mu_gdot_max[i+1];
        
        lambda_gdot_min[i] = lambda_gdot_min[i+1];
        mu_gdot_min[i] = mu_gdot_min[i+1];
    }
}

Solution ALILQRSolver::solve(const State& init_state,const std::vector<Trajectory>& obs_list) {
    ego.set_state(init_state);
    this->obs_list = obs_list;
    ego.set_local_plan();
    
    // Initialize solution
    Solution nominal_solution = get_nominal_solution(init_state);
    Solution current_solution = nominal_solution;
    Solution new_solution;
    
    if (this->pre_solution.control_sequence.size() > 0) {
        shift_penalties();
    } else {
        initialize_penalties();
    }
    converged = false;
    
    for(int outer = 0; outer < max_outer_iters; ++outer){
        // Inner iLQR Loop
        double J_old = cal_cost(current_solution); // Augmented Lagrangian Cost
        
        for (int inner = 0; inner < max_inner_iters; ++inner) {
            compute_df(current_solution);
            compute_al_derivatives(current_solution);
            backward();
            new_solution = forward(current_solution);
            
            double J_new = cal_cost(new_solution);
            
            if (std::abs(J_old - J_new) < arg.tol) {
                current_solution = new_solution;
                break;
            }
            current_solution = new_solution;
            J_old = J_new;
        }
        
        // Check constraint violation
        double max_violation = max_constraint_violation(current_solution);
        std::cout << "Outer Iter " << outer << ": Max Violation = " << max_violation << std::endl;
        
        if (max_violation < constraint_tol) {
            converged = true;
            break;
        }
        
        // Update AL parameters
        update_constraints(current_solution);
    }
    
    this->pre_solution = current_solution;
    return current_solution;
}

void ALILQRSolver::update_constraints(const Solution& solution) {
    const auto& X_traj = solution.ego_trj.get_states();
    const auto& U_seq = solution.control_sequence.get_control_sequence();
    
    for (int i = 0; i <= arg.N; ++i) {
        // State Constraints
        const State& X = X_traj[i];
        
        // Gamma Max: gamma <= gamma_max  =>  gamma - gamma_max <= 0
        double c_gmax = X[3] - arg.gamma_max;
        lambda_gamma_max[i] = std::max(0.0, lambda_gamma_max[i] + mu_gamma_max[i] * c_gmax);
        if (c_gmax > constraint_tol) mu_gamma_max[i] = std::min(penalty_max, mu_gamma_max[i] * penalty_scaling);
        
        // Gamma Min: gamma >= gamma_min  =>  gamma_min - gamma <= 0
        double c_gmin = arg.gamma_min - X[3];
        lambda_gamma_min[i] = std::max(0.0, lambda_gamma_min[i] + mu_gamma_min[i] * c_gmin);
        if (c_gmin > constraint_tol) mu_gamma_min[i] = std::min(penalty_max, mu_gamma_min[i] * penalty_scaling);
        
        // Obstacles
        if (arg.if_cal_obs_cost) {
            double max_c_obs = -1e9;
            for (const auto& obs : obs_list) {
                if (i >= obs.get_states().size()) continue;
                const State& obs_state = obs.get_states()[i];
                double dx = X[0] - obs_state[0];
                double dy = X[1] - obs_state[1];
                double a = arg.obs_length/2 + ego.get_model().ego_rad/2 + arg.safe_a_buffer;
                double b = arg.obs_width/2 + ego.get_model().ego_rad/2 + arg.safe_b_buffer;
                Vector2d dX_obs(dx, dy);
                Matrix2d R; R << cos(obs_state[2]), sin(obs_state[2]), -sin(obs_state[2]), cos(obs_state[2]);
                Vector2d dX_obs_cord = R * dX_obs;
                // Constraint: 1 - (x/a)^2 - (y/b)^2 <= 0 (inside ellipse is violation)
                // Wait, standard form is c(x) <= 0.
                // Inside ellipse: (x/a)^2 + (y/b)^2 <= 1.
                // We want OUTSIDE: (x/a)^2 + (y/b)^2 >= 1  =>  1 - (x/a)^2 - (y/b)^2 <= 0.
                double c = 1.0 - (pow(dX_obs_cord[0],2)/pow(a,2) + pow(dX_obs_cord[1],2)/pow(b,2));
                max_c_obs = std::max(max_c_obs, c);
            }
            // Only update based on the worst violation (Max-Constraint approach)
            if (max_c_obs > -1e8) {
                lambda_obs[i] = std::max(0.0, lambda_obs[i] + mu_obs[i] * max_c_obs);
                if (max_c_obs > constraint_tol) mu_obs[i] = std::min(penalty_max, mu_obs[i] * penalty_scaling);
            }
        }
        
        // Lane
        if (arg.if_cal_lane_cost) {
            const auto& local_plan = ego.get_local_plan().get_points();
            size_t index = find_closest_point(local_plan, X);
            size_t match_index = index == local_plan.size() - 1 ? index : index + 1;
            const Point& X_r_point = local_plan[match_index];
            Vector2d dX(X[0] - X_r_point.x, X[1] - X_r_point.y);
            Vector2d nor_r(-std::sin(X_r_point.heading), std::cos(X_r_point.heading));
            double l = dX.dot(nor_r);
            
            double c_left = l - arg.trace_safe_width_left;
            double c_right = -l - arg.trace_safe_width_right;
            double max_c_lane = std::max(c_left, c_right);
            
            lambda_lane[i] = std::max(0.0, lambda_lane[i] + mu_lane[i] * max_c_lane);
            if (max_c_lane > constraint_tol) mu_lane[i] = std::min(penalty_max, mu_lane[i] * penalty_scaling);
        }
    }
    
    for (int i = 0; i < arg.N; ++i) {
        const Control& U = U_seq[i];
        
        // Gamma Dot Max
        double c_umax = U[1] - arg.gamma_dot_max;
        lambda_gdot_max[i] = std::max(0.0, lambda_gdot_max[i] + mu_gdot_max[i] * c_umax);
        if (c_umax > constraint_tol) mu_gdot_max[i] = std::min(penalty_max, mu_gdot_max[i] * penalty_scaling);
        
        // Gamma Dot Min
        double c_umin = arg.gamma_dot_min - U[1];
        lambda_gdot_min[i] = std::max(0.0, lambda_gdot_min[i] + mu_gdot_min[i] * c_umin);
        if (c_umin > constraint_tol) mu_gdot_min[i] = std::min(penalty_max, mu_gdot_min[i] * penalty_scaling);
    }
}

double ALILQRSolver::max_constraint_violation(const Solution& solution) {
    const auto& X_traj = solution.ego_trj.get_states();
    const auto& U_seq = solution.control_sequence.get_control_sequence();
    double max_viol = 0.0;
    
    for (int i = 0; i <= arg.N; ++i) {
        const State& X = X_traj[i];
        max_viol = std::max(max_viol, X[3] - arg.gamma_max);
        max_viol = std::max(max_viol, arg.gamma_min - X[3]);
        
        if (arg.if_cal_obs_cost) {
            for (const auto& obs : obs_list) {
                if (i >= obs.get_states().size()) continue;
                const State& obs_state = obs.get_states()[i];
                double dx = X[0] - obs_state[0];
                double dy = X[1] - obs_state[1];
                double a = arg.obs_length/2 + ego.get_model().ego_rad/2 + arg.safe_a_buffer;
                double b = arg.obs_width/2 + ego.get_model().ego_rad/2 + arg.safe_b_buffer;
                Vector2d dX_obs(dx, dy);
                Matrix2d R; R << cos(obs_state[2]), sin(obs_state[2]), -sin(obs_state[2]), cos(obs_state[2]);
                Vector2d dX_obs_cord = R * dX_obs;
                double c = 1.0 - (pow(dX_obs_cord[0],2)/pow(a,2) + pow(dX_obs_cord[1],2)/pow(b,2));
                max_viol = std::max(max_viol, c);
            }
        }
        
        if (arg.if_cal_lane_cost) {
            const auto& local_plan = ego.get_local_plan().get_points();
            size_t index = find_closest_point(local_plan, X);
            size_t match_index = index == local_plan.size() - 1 ? index : index + 1;
            const Point& X_r_point = local_plan[match_index];
            Vector2d dX(X[0] - X_r_point.x, X[1] - X_r_point.y);
            Vector2d nor_r(-std::sin(X_r_point.heading), std::cos(X_r_point.heading));
            double l = dX.dot(nor_r);
            max_viol = std::max(max_viol, l - arg.trace_safe_width_left);
            max_viol = std::max(max_viol, -l - arg.trace_safe_width_right);
        }
    }
    
    for (int i = 0; i < arg.N; ++i) {
        const Control& U = U_seq[i];
        max_viol = std::max(max_viol, U[1] - arg.gamma_dot_max);
        max_viol = std::max(max_viol, arg.gamma_dot_min - U[1]);
    }
    
    return max_viol;
}

void ALILQRSolver::compute_al_derivatives(const Solution& solution) {
    const auto& X_traj = solution.ego_trj.get_states();
    const auto& U = solution.control_sequence.get_control_sequence();
    const auto& local_plan = ego.get_local_plan().get_points();
    const int N = arg.N;
    Vector2d P2(0, 1);
    
    int chunks = 4;
    int chunk_size = (N + 1 + chunks - 1) / chunks;
    std::vector<std::future<void>> tasks;
    
    for(int c=0;c<chunks;c++){
        int start = c * chunk_size;
        int end = std::min(N + 1, (c + 1) * chunk_size);
        tasks.push_back(std::async(std::launch::async, [&, start, end](){
            for (int i = start; i < end; ++i) {
                const State& X = X_traj[i];
                size_t index = find_closest_point(local_plan, X);
                size_t match_index = index == local_plan.size() - 1 ? index : index + 1;
                const Point& X_r_point = local_plan[match_index];
                State X_r; X_r << X_r_point.x, X_r_point.y, X_r_point.heading, 0;
                State X_e = X - X_r;
                
                // Original Cost Derivatives
                Vector4d l_dx = 2 * arg.Q * X_e;
                Matrix4d l_ddx = 2 * arg.Q;
                
                Vector2d dX(X_e[0], X_e[1]);
                Vector2d nor_r(-std::sin(X_r_point.heading), std::cos(X_r_point.heading));
                Vector4d l_dx_ref(-2*dX.dot(nor_r)*std::sin(X_r_point.heading), 2*dX.dot(nor_r)*std::cos(X_r_point.heading), 0, 0);
                Matrix4d l_ddx_ref;
                l_ddx_ref << 2*pow(sin(X_r_point.heading),2), -sin(2*X_r_point.heading), 0, 0,
                            -sin(2*X_r_point.heading), 2*pow(cos(X_r_point.heading),2), 0, 0,
                            0, 0, 0, 0,
                            0, 0, 0, 0;
                l_dx_ref *= arg.ref_weight;
                l_ddx_ref *= arg.ref_weight;
                
                // Augmented Lagrangian Derivatives
                Vector4d grad_obs = Vector4d::Zero();
                Matrix4d hess_obs = Matrix4d::Zero();
                
                if (arg.if_cal_obs_cost) {
                    // Find max violation obstacle (Max-Constraint)
                    // Or sum them up? ALTRO paper suggests handling constraints individually or max.
                    // For now, let's sum them up as separate constraints, but we only have one lambda/mu per step for "obs".
                    // Wait, if we have one lambda/mu per step, we should treat "obstacle collision" as a single constraint c(x) <= 0?
                    // Usually c(x) = max(c_i(x)).
                    // Let's use the max violation obstacle for the gradient/hessian at this step.
                    
                    double max_c = -1e9;
                    int max_idx = -1;
                    Vector4d max_c_dot = Vector4d::Zero();
                    Matrix4d max_c_ddot = Matrix4d::Zero();
                    
                    for (size_t obs_idx = 0; obs_idx < obs_list.size(); ++obs_idx) {
                        const Trajectory& obs = obs_list[obs_idx];
                        if (i >= obs.get_states().size()) continue;
                        const State& obs_state = obs.get_states()[i];
                        double dx = X[0] - obs_state[0];
                        double dy = X[1] - obs_state[1];
                        double a = arg.obs_length/2 + ego.get_model().ego_rad/2 + arg.safe_a_buffer;
                        double b = arg.obs_width/2 + ego.get_model().ego_rad/2 + arg.safe_b_buffer;
                        Vector2d dX_obs(dx, dy);
                        Matrix2d R; R << cos(obs_state[2]), sin(obs_state[2]), -sin(obs_state[2]), cos(obs_state[2]);
                        Vector2d dX_obs_cord = R * dX_obs;
                        double c = 1.0 - (pow(dX_obs_cord[0],2)/pow(a,2) + pow(dX_obs_cord[1],2)/pow(b,2));
                        
                        if (c > max_c) {
                            max_c = c;
                            max_idx = obs_idx;
                            Vector2d grad_local(-2 * dX_obs_cord[0] / (a*a), -2 * dX_obs_cord[1] / (b*b));
                            Vector2d grad_global = R.transpose() * grad_local;
                            max_c_dot << grad_global[0], grad_global[1], 0.0, 0.0;
                            // Hessian approximation (ignoring curvature of c for now or simple approx)
                            // c = 1 - x'Mx. grad = -2Mx. hess = -2M.
                            Matrix2d M; M << 1.0/(a*a), 0, 0, 1.0/(b*b);
                            Matrix2d hess_global = R.transpose() * (-2.0 * M) * R;
                            max_c_ddot.block<2,2>(0,0) = hess_global;
                        }
                    }
                    
                    if (max_idx != -1) {
                        // PHR Penalty: P(c) = lambda*c + 0.5*mu*c^2 if lambda + mu*c > 0
                        // Gradient: (lambda + mu*c) * grad_c
                        // Hessian: (lambda + mu*c) * hess_c + mu * grad_c * grad_c'
                        double val = lambda_obs[i] + mu_obs[i] * max_c;
                        if (val > 0) {
                            grad_obs += val * max_c_dot;
                            hess_obs += val * max_c_ddot + mu_obs[i] * max_c_dot * max_c_dot.transpose();
                        }
                    }
                }
                
                Vector4d grad_lane = Vector4d::Zero();
                Matrix4d hess_lane = Matrix4d::Zero();
                if (arg.if_cal_lane_cost) {
                    double l = dX.dot(nor_r);
                    Vector4d dc; dc << -std::sin(X_r_point.heading), std::cos(X_r_point.heading), 0, 0;
                    
                    // Max constraint between left and right
                    double c_left = l - arg.trace_safe_width_left;
                    double c_right = -l - arg.trace_safe_width_right;
                    
                    double c_lane = (c_left > c_right) ? c_left : c_right;
                    Vector4d dc_lane = (c_left > c_right) ? dc : -dc;
                    
                    double val = lambda_lane[i] + mu_lane[i] * c_lane;
                    if (val > 0) {
                        grad_lane += val * dc_lane;
                        hess_lane += mu_lane[i] * dc_lane * dc_lane.transpose();
                    }
                }
                
                double gamma = X[3];
                Vector4d e4; e4 << 0,0,0,1;
                
                // Gamma Max
                double c_gmax = gamma - arg.gamma_max;
                double val_gmax = lambda_gamma_max[i] + mu_gamma_max[i] * c_gmax;
                Vector4d grad_gamma = Vector4d::Zero();
                Matrix4d hess_gamma = Matrix4d::Zero();
                if (val_gmax > 0) {
                    grad_gamma += val_gmax * e4;
                    hess_gamma += mu_gamma_max[i] * e4 * e4.transpose();
                }
                
                // Gamma Min
                double c_gmin = arg.gamma_min - gamma;
                double val_gmin = lambda_gamma_min[i] + mu_gamma_min[i] * c_gmin;
                if (val_gmin > 0) {
                    grad_gamma += val_gmin * (-e4);
                    hess_gamma += mu_gamma_min[i] * (-e4) * (-e4).transpose();
                }
                
                lx[i] = l_dx + l_dx_ref + grad_obs + grad_lane + grad_gamma;
                lxx[i] = l_ddx + l_ddx_ref + hess_obs + hess_lane + hess_gamma;
            }
        }));
    }
    for(auto& t:tasks) t.get();
    
    for (int i = 0; i < N; ++i) {
        const Vector2d u = U[i];
        const Vector2d u_r(arg.desire_speed, 0);
        const Vector2d u_e = u - u_r;
        Vector2d lu_base = 2 * arg.R * u_e;
        Matrix2d luu_base = 2 * arg.R;
        
        Vector2d db = Vector2d::Zero();
        Matrix2d ddb = Matrix2d::Zero();
        
        // Gamma Dot Max
        double c_umax = u.dot(P2) - arg.gamma_dot_max;
        double val_umax = lambda_gdot_max[i] + mu_gdot_max[i] * c_umax;
        if (val_umax > 0) {
            db += val_umax * P2;
            ddb += mu_gdot_max[i] * P2 * P2.transpose();
        }
        
        // Gamma Dot Min
        double c_umin = arg.gamma_dot_min - u.dot(P2);
        double val_umin = lambda_gdot_min[i] + mu_gdot_min[i] * c_umin;
        if (val_umin > 0) {
            db += val_umin * (-P2);
            ddb += mu_gdot_min[i] * (-P2) * (-P2).transpose();
        }
        
        lu[i] = lu_base + db;
        luu[i] = luu_base + ddb;
        lux[i] = Matrix<double,2,4>::Zero();
    }
    
    if (arg.if_cal_speed_rate_cost) {
        for (int i = 0; i < N; ++i) {
            Vector2d lu_speed_rate = Vector2d::Zero();
            Matrix2d luu_speed_rate = Matrix2d::Zero();
            if (i < N - 1) {
                const Vector2d& U_cur = U[i];
                const Vector2d& U_next = U[i + 1];
                double v_diff = U_next[0] - U_cur[0];
                lu_speed_rate[0] += -2 * arg.v_rate_weight * v_diff;
                luu_speed_rate(0, 0) += 2 * arg.v_rate_weight;
            }
            if (i > 0) {
                const Vector2d& U_prev = U[i - 1];
                const Vector2d& U_cur = U[i];
                double v_diff = U_cur[0] - U_prev[0];
                lu_speed_rate[0] += 2 * arg.v_rate_weight * v_diff;
                luu_speed_rate(0, 0) += 2 * arg.v_rate_weight;
            }
            lu[i] += lu_speed_rate;
            luu[i] += luu_speed_rate;
        }
    }
}



Solution ALILQRSolver::get_nominal_solution(const State& init_state){
    Trajectory nominal_trj;
    ControlSequence nominal_ctrl_sequence;
    nominal_trj.states.reserve(arg.N + 1);
    nominal_ctrl_sequence.controls.reserve(arg.N);
    State X0 = init_state;
    nominal_trj.push_back(X0);

    if(this->pre_solution.control_sequence.size()!=0){
        // Warm start: use previous solution's control sequence
        for(int i=0;i<(this->arg.N - 1);i++){
            Control U = pre_solution.control_sequence.get_control_sequence()[i+1];
            nominal_ctrl_sequence.push_back(U);
        };
        // Duplicate last control
        nominal_ctrl_sequence.push_back(pre_solution.control_sequence.get_control_sequence()[arg.N-1]);
        
        // Generate trajectory from controls
        State Xout;
        Control U;
        for(int i=0; i < this->arg.N; i++){
            U = nominal_ctrl_sequence.get_control_sequence()[i];
            Xout = this->ego.get_model().dynamics(X0,U); 
            X0 = Xout;
            nominal_trj.push_back(Xout);
        }
    }
    else{
        // Cold start: Pure Pursuit
        for (int i=0;i<arg.N;i++){
            Control U = pure_pursuit(nominal_trj.back());
            U[0] = std::clamp(U[0], 0.1, 15.0);
            double safe_gamma_dot_max = std::min(arg.gamma_dot_max * 0.8, 0.8);
            U[1] = std::clamp(U[1], -safe_gamma_dot_max, safe_gamma_dot_max);
            State X_next = ego.get_model().dynamics(nominal_trj.back(), U);
            X_next[3] = std::clamp(X_next[3], arg.gamma_min * 0.9, arg.gamma_max * 0.9);
            nominal_ctrl_sequence.push_back(U);
            nominal_trj.push_back(X_next);
        }
    }
    return Solution(nominal_trj, nominal_ctrl_sequence);
}

Control ALILQRSolver::pure_pursuit(const State& X_cur){
    const auto& local_plan = ego.get_local_plan().get_points();
    size_t indexNow = find_closest_point(local_plan, X_cur);
    double Kv = arg.kv;
    double Ld0 = arg.ld0;
    double Ld_min = arg.ld_min;
    double Ld_max = arg.ld_max;
    double current_speed = std::max(1.0, arg.desire_speed);
    double Ld = std::clamp(Kv * current_speed + Ld0, Ld_min, Ld_max);
    size_t indexTarget = indexNow;
    double accumulated_dist = 0.0;
    for (size_t i = indexNow; i < local_plan.size(); ++i) {
        if (i > indexNow) accumulated_dist += distance(local_plan[i], local_plan[i-1]);
        if (accumulated_dist >= Ld) { indexTarget = i; break; }
    }
    if (indexTarget >= local_plan.size()) indexTarget = local_plan.size() - 1;
    const Point& target = local_plan[indexTarget];
    double dx = target.x - X_cur[0];
    double dy = target.y - X_cur[1];
    double desired_theta = std::atan2(dy, dx);
    double theta_error = angle_wrap(desired_theta - X_cur[2]);
    double desired_theta_dot = arg.kp * theta_error;
    double current_gamma = X_cur[3];
    double v = arg.desire_speed;
    double lf = ego.get_model().lf;
    double lr = ego.get_model().lr;
    double len = lr + lf * std::cos(current_gamma);
    if (std::abs(len) < 0.1) len = 0.1;
    double required_gamma_dot = -(desired_theta_dot * len + v * std::sin(current_gamma)) / lr;
    required_gamma_dot = std::clamp(required_gamma_dot, -arg.gamma_dot_max * 0.8, arg.gamma_dot_max * 0.8);
    return Control(v, required_gamma_dot);
}

void ALILQRSolver::compute_df(const Solution& solution) {
    const auto& X_traj = solution.ego_trj.get_states();
    const auto& U_seq = solution.control_sequence.get_control_sequence();
    const int N = arg.N;
    if (static_cast<int>(df_dx.size()) != N) df_dx.assign(N, Matrix4d::Zero());
    if (static_cast<int>(df_du.size()) != N) df_du.assign(N, Matrix<double,4,2>::Zero());
    auto model = ego.get_model();
    for (int i = 0; i < N; ++i) {
        const Vector4d& X = X_traj[i];
        const Vector2d& U = U_seq[i];
        df_dx[i] = model.get_jacobian_state(X, U);
        df_du[i] = model.get_jacobian_control(X, U);
    }
}

void ALILQRSolver::backward() {
    const int N = arg.N;
    const double lambda = this->lamb;
    Vector4d V_x = lx[N];
    Matrix4d V_xx = lxx[N];
    this->k = std::vector<MatrixXd>(N, Vector2d::Zero());
    this->K = std::vector<MatrixXd>(N, MatrixXd::Zero(2,4));
    Matrix4d df_dx_m;
    Matrix<double,4,2> df_du_m;
    Vector4d Qx;
    Matrix4d Qxx;
    Matrix<double,2,4> Qux;
    Matrix2d Quu_inv;
    for (int i = N-1; i >= 0; --i) {
        df_dx_m = this->df_dx[i];
        df_du_m = this->df_du[i];
        Qx = lx[i] + df_dx_m.transpose() * V_x;
        Qu[i] = lu[i] + df_du_m.transpose() * V_x;
        Qxx = lxx[i] + df_dx_m.transpose() * V_xx * df_dx_m;
        Qux = lux[i] + df_du_m.transpose() * V_xx * df_dx_m;
        Quu[i] = luu[i] + df_du_m.transpose() * V_xx * df_du_m;
        Matrix2d Quu_reg = Quu[i] + lambda * Matrix2d::Identity();
        Eigen::LLT<Matrix2d> llt(Quu_reg);
        if (llt.info() == Eigen::Success) {
            Quu_inv = llt.solve(Matrix2d::Identity());
        } else {
            JacobiSVD<Matrix2d> svd(Quu_reg, ComputeFullU | ComputeFullV);
            Matrix2d U = svd.matrixU();
            Matrix2d V = svd.matrixV();
            Vector2d s = svd.singularValues().cwiseMax(1e-8);
            Quu_inv = V * s.cwiseInverse().asDiagonal() * U.transpose();
        }
        this->k[i] = -Quu_inv * Qu[i];
        this->K[i] = -Quu_inv * Qux;
        V_x = Qx - K[i].transpose() * Quu[i] * k[i];
        V_xx = Qxx - K[i].transpose() * Quu[i] * K[i];
    }
}

Solution ALILQRSolver::forward(const Solution& cur_solution){
    double J_old = cal_cost_with_logging(cur_solution, -101);
    auto U = cur_solution.control_sequence.get_control_sequence();
    auto X = cur_solution.ego_trj.get_states();
    std::vector<double> candidates = {1.0, 0.5, 0.25, 0.125};
    struct Result { double alpha; double J; double delta_V; Solution sol; bool ok; };
    std::vector<std::future<Result>> tasks;
    average_gradient = 0;
    for(double alpha : candidates){
        tasks.push_back(std::async(std::launch::async, [&, alpha](){
            std::vector<Control> U_tmp = U;
            std::vector<State> X_tmp = X;
            double delta_V = 0;
            for (int i = 0; i < arg.N; ++i) {
                Vector4d delta_x = X_tmp[i] - cur_solution.ego_trj.get_states()[i];
                U_tmp[i] += alpha * k[i] + K[i] * delta_x;
                
                // Clamp controls
                U_tmp[i][0] = std::clamp(U_tmp[i][0], 0.0, 25.0); // Velocity limit (approx)
                U_tmp[i][1] = std::clamp(U_tmp[i][1], arg.gamma_dot_min, arg.gamma_dot_max);
                
                X_tmp[i+1] = ego.get_model().dynamics(X_tmp[i], U_tmp[i]);
                
                // Clamp state gamma for stability
                X_tmp[i+1][3] = std::clamp(X_tmp[i+1][3], arg.gamma_min - 0.1, arg.gamma_max + 0.1);

                delta_V += alpha * (k[i].transpose() * Qu[i]).value() + alpha * alpha * 0.5 * (k[i].transpose() * Quu[i] * k[i]).value();
            }
            Solution s;
            s.ego_trj.states.swap(X_tmp);
            s.control_sequence.controls.swap(U_tmp);
            double J_new = cal_cost_with_logging(s, -102);
            double armijo_c1 = 0.1;
            double expected_reduction = armijo_c1 * alpha * delta_V;
            bool ok = (J_new - J_old) < expected_reduction && (J_new - J_old) < 0 && std::abs(J_new - J_old) > 1e-12 && std::isfinite(J_new);
            return Result{alpha, J_new, delta_V, s, ok};
        }));
    }
    Result best{0, std::numeric_limits<double>::infinity(), 0, cur_solution, false};
    for(auto& f : tasks){
        auto r = f.get();
        if(r.ok && r.J < best.J){ best = r; }
    }
    if(best.ok){ return best.sol; }
    double alpha = 0.5;
    for (int iter = 0; iter < 6; ++iter) {
        std::vector<Control> U_tmp = U;
        std::vector<State> X_tmp = X;
        double delta_V = 0;
        for (int i = 0; i < arg.N; ++i) {
            Vector4d delta_x = X_tmp[i] - cur_solution.ego_trj.get_states()[i];
            U_tmp[i] += alpha * k[i] + K[i] * delta_x;
            
            // Clamp controls
            U_tmp[i][0] = std::clamp(U_tmp[i][0], 0.0, 25.0);
            U_tmp[i][1] = std::clamp(U_tmp[i][1], arg.gamma_dot_min, arg.gamma_dot_max);

            X_tmp[i+1] = ego.get_model().dynamics(X_tmp[i], U_tmp[i]);
            
            // Clamp state gamma
            X_tmp[i+1][3] = std::clamp(X_tmp[i+1][3], arg.gamma_min - 0.1, arg.gamma_max + 0.1);

            delta_V += alpha * (k[i].transpose() * Qu[i]).value() + alpha * alpha * 0.5 * (k[i].transpose() * Quu[i] * k[i]).value();
        }
        Solution s;
        s.ego_trj.states.swap(X_tmp);
        s.control_sequence.controls.swap(U_tmp);
        double J_new = cal_cost_with_logging(s, -103);
        double armijo_c1 = 0.1;
        double expected_reduction = armijo_c1 * alpha * delta_V;
        if ((J_new - J_old) < expected_reduction && (J_new - J_old) < 0 && std::abs(J_new - J_old) > 1e-12 && std::isfinite(J_new)) {
            return s;
        }
        alpha *= 0.5;
        if(alpha < 1e-8) break;
    }
    return cur_solution;
}

double ALILQRSolver::cal_cost(const Solution& solution) {
    const auto& X_traj = solution.ego_trj.get_states();
    const auto& U_seq = solution.control_sequence.get_control_sequence();
    double J = 0.0;
    
    // State Costs
    for (int i = 0; i < arg.N + 1; ++i) {
        const State& X = X_traj[i];
        size_t index = find_closest_point(ego.get_local_plan().get_points(), X);
        size_t match_index = index == ego.get_local_plan().get_points().size() - 1 ? index : index + 1;
        const Point& X_r_point = ego.get_local_plan().get_points()[match_index];
        State X_r; X_r << X_r_point.x, X_r_point.y, X_r_point.heading, 0;
        State X_e = X - X_r;
        
        J += X_e.transpose() * arg.Q * X_e;
        
        Vector2d dX(X_e[0], X_e[1]);
        Vector2d nor_r(-std::sin(X_r_point.heading), std::cos(X_r_point.heading));
        J += pow(dX.dot(nor_r), 2) * arg.ref_weight;
        
        // Constraints (Augmented Lagrangian Terms)
        // Gamma Max
        double c_gmax = X[3] - arg.gamma_max;
        double val_gmax = lambda_gamma_max[i] + mu_gamma_max[i] * c_gmax;
        if (val_gmax > 0) J += lambda_gamma_max[i] * c_gmax + 0.5 * mu_gamma_max[i] * c_gmax * c_gmax;
        
        // Gamma Min
        double c_gmin = arg.gamma_min - X[3];
        double val_gmin = lambda_gamma_min[i] + mu_gamma_min[i] * c_gmin;
        if (val_gmin > 0) J += lambda_gamma_min[i] * c_gmin + 0.5 * mu_gamma_min[i] * c_gmin * c_gmin;
        
        // Obstacles
        if (arg.if_cal_obs_cost) {
            double max_c_obs = -1e9;
            for (const auto& obs : obs_list) {
                if (i >= obs.get_states().size()) continue;
                const State& obs_state = obs.get_states()[i];
                double dx = X[0] - obs_state[0];
                double dy = X[1] - obs_state[1];
                double a = arg.obs_length/2 + ego.get_model().ego_rad/2 + arg.safe_a_buffer;
                double b = arg.obs_width/2 + ego.get_model().ego_rad/2 + arg.safe_b_buffer;
                Vector2d dX_obs(dx, dy);
                Matrix2d R; R << cos(obs_state[2]), sin(obs_state[2]), -sin(obs_state[2]), cos(obs_state[2]);
                Vector2d dX_obs_cord = R * dX_obs;
                double c = 1.0 - (pow(dX_obs_cord[0],2)/pow(a,2) + pow(dX_obs_cord[1],2)/pow(b,2));
                max_c_obs = std::max(max_c_obs, c);
            }
            if (max_c_obs > -1e8) {
                double val = lambda_obs[i] + mu_obs[i] * max_c_obs;
                if (val > 0) J += lambda_obs[i] * max_c_obs + 0.5 * mu_obs[i] * max_c_obs * max_c_obs;
            }
        }
        
        // Lane
        if (arg.if_cal_lane_cost) {
            double l = dX.dot(nor_r);
            double c_left = l - arg.trace_safe_width_left;
            double c_right = -l - arg.trace_safe_width_right;
            double max_c_lane = std::max(c_left, c_right);
            
            double val = lambda_lane[i] + mu_lane[i] * max_c_lane;
            if (val > 0) J += lambda_lane[i] * max_c_lane + 0.5 * mu_lane[i] * max_c_lane * max_c_lane;
        }
    }
    
    // Control Costs
    for (int i = 0; i < arg.N; ++i) {
        const Control& U = U_seq[i];
        Control U_ref = {arg.desire_speed, 0};
        Control U_e = U - U_ref;
        J += U_e.transpose() * arg.R * U_e;
        
        // Gamma Dot Max
        double c_umax = U[1] - arg.gamma_dot_max;
        double val_umax = lambda_gdot_max[i] + mu_gdot_max[i] * c_umax;
        if (val_umax > 0) J += lambda_gdot_max[i] * c_umax + 0.5 * mu_gdot_max[i] * c_umax * c_umax;
        
        // Gamma Dot Min
        double c_umin = arg.gamma_dot_min - U[1];
        double val_umin = lambda_gdot_min[i] + mu_gdot_min[i] * c_umin;
        if (val_umin > 0) J += lambda_gdot_min[i] * c_umin + 0.5 * mu_gdot_min[i] * c_umin * c_umin;
    }
    
    // Speed Rate Cost
    if (arg.if_cal_speed_rate_cost) {
        for (int i = 0; i < arg.N - 1; ++i) {
            const Control& U_cur = U_seq[i];
            const Control& U_next = U_seq[i + 1];
            double v_diff = U_next[0] - U_cur[0];
            J += arg.v_rate_weight * v_diff * v_diff;
        }
    }
    
    return J;
}

double ALILQRSolver::cal_cost_with_logging(const Solution& solution, int iteration) {
    // Just return the AL cost for now, logging can be added if needed but let's keep it simple
    // The original cal_cost_with_logging was quite verbose.
    // We can use cal_cost() here.
    return cal_cost(solution);
}

