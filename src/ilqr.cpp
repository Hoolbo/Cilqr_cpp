#include <cmath>
#include <iostream>
#include "ilqr.h"
using namespace Eigen;
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
}

BarrieInfo barrierFunction(double q1,double q2,double c,VectorXd dc){
    BarrieInfo info;
    info.b = q1 * exp(q2 * c);
    info.d_b = q1*q2*exp(q2*c)*dc;
    info.dd_b = q1*(q2*q2)*exp(q2*c)*(dc * dc.transpose());
    return info;
}
//系统模型
State SystemModel::dynamics(const State& X, const Control& U){
    double beta = atan((lr / (lr + lf)) * tan(U[1]));
    State X_next;
    X_next << 
        X[0] + X[3] * cos(X[2] + beta) * dt,
        X[1] + X[3] * sin(X[2] + beta) * dt,
        X[2] + (X[3] / len) * tan(U[1]) * cos(beta) * dt,
        X[3] + U[0] * dt;
    X_next(2) = angle_wrap(X_next(2));
    return X_next;
}
Matrix4d SystemModel::get_jacobian_state(const Vector4d& X, const Vector2d& U){
    double phi = X(2);
    double v = X(3);
    double delta = U(1);
    double beta = atan((lr / (lr + lf)) * tan(delta));
    Matrix4d df_dx;
    df_dx << 1, 0, -v*sin(phi+beta)*dt,  cos(phi+beta)*dt,
             0, 1,  v*cos(phi+beta)*dt,  sin(phi+beta)*dt,
             0, 0,    1,  tan(delta)*cos(beta) * dt / len, 
             0, 0,    0,                                1;
    return df_dx;
}
Matrix<double,4,2> SystemModel::get_jacobian_control(const Vector4d& X, const Vector2d& U){
    double phi = X(2);
    double v = X(3);
    double delta = U(1);
    double beta = atan((lr / (lr + lf)) * tan(delta));
    double k = lr / (lr + lf); 
    auto sec = [](double x) -> double { return 1 / cos(x); };
    double dbeta_ddelta = k * sec(delta) * sec(delta) / (1 + k*k*tan(delta)*tan(delta));
    Matrix<double,4,2> df_du;
    df_du <<   0  ,                         -v*sin(phi+beta)*dbeta_ddelta * dt                           ,
               0  ,                          v*cos(phi+beta)*dbeta_ddelta * dt                           ,
               0  ,  (v/len) * dt * (sec(delta)*sec(delta)*cos(beta) - tan(delta)*sin(beta)*dbeta_ddelta),
               dt ,                                             0                                        ;
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
Solution CILQRSolver::solve(const State& init_state,const Trajectory& obs) {
    
    ego.set_state(init_state);
    this->obs = obs;
    // 初始化局部路径和初始解
    ego.set_local_plan();

    //如果接近终点就把期望速度置0
    Point local_last_point = this->ego.get_local_plan().get_points()[ego.get_local_plan().get_points().size()-1];
    Point global_last_point = this->ego.get_global_plan().get_points()[ego.get_global_plan().get_points().size()-1];
    if(local_last_point == global_last_point){
        arg.desire_speed = 0;
        arg.desire_heading = 0;
        arg.Q(2,2) = 0;
    }

    Solution current_solution = get_nominal_solution();
    Solution new_solution;
    double J_old = cal_cost(current_solution);
  
    lamb = arg.lamb_init; // 如果上次求解未收敛则初始化正则化系数
    converged = false;
    
    for (int iter = 0; iter < arg.max_iter; ++iter) {
        // // 备份当前解
        // Solution old_solution = current_solution;

        // 反向传播计算控制修正量
        compute_df(current_solution);
        compute_cost_derivatives(current_solution);
        backward();

        // 正向传播尝试新解并计算新代价
        new_solution = forward(current_solution);
        double J_new = cal_cost(new_solution);

        // 计算相对改进量
        double rel_improve = (J_old - J_new) / J_old;
        if (J_new < J_old ) {
            // 动态调整正则化系数
            if (rel_improve > 0.1) { // 显著改进时降低λ
                lamb = lamb / 1.5;
            } else {                 // 轻微改进时保守调整
                lamb = lamb * 2;
            }
            // 更新当前解
            current_solution = new_solution;
            J_old = J_new;
            // 收敛终止条件判断
            if (rel_improve < arg.rel_tol) {
                //记录上一次解
                pre_solution = current_solution;
                std::cout << "Converged | iteration: " << iter + 1  << std::endl;
                converged = true;
                break;
            }
        } else { // 代价未下降时增加正则化
            lamb = lamb * 2;
            // 失败终止条件判断
             if (lamb > arg.lamb_max) {
                pre_solution = Solution();
                std::cerr << "Unconverged | Maxmum lamb | iteration: "<< iter + 1 << std::endl;
                break;
            }
        }
    }
    
    if (!converged && lamb <= arg.lamb_max) {
        pre_solution = Solution();
        std::cerr << "Unconverged::Maxmum iteration" << std::endl;
    }
    pre_solution = current_solution;
    return current_solution;
}
//获取标称轨迹
Solution CILQRSolver::get_nominal_solution(){
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
    State X0 = this->ego.get_state();
    nominal_trj.push_back(X0);
    if(this->pre_solution.control_sequence.size()!=0){
        //如果有上一帧的控制序列，则拼接控制序列
        for(int i=0;i<(this->arg.N - 1);i++){
            Control U = pre_solution.control_sequence.get_control_sequence()[i+1];
            nominal_ctrl_sequence.push_back(U);
        };
        nominal_ctrl_sequence.push_back(pre_solution.control_sequence.get_control_sequence()[arg.N-1]);
        //更新轨迹
        for(int i=0; i < this->arg.N; i++){
            U = nominal_ctrl_sequence.get_control_sequence()[0];
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
            // 生成控制指令
            Control U = pure_pursuit(X_cur);
            // 添加安全限制
            U[1] = std::clamp(U[1], 
                -arg.steer_angle_max, 
                arg.steer_angle_max);
            // 前向模拟
            // U << 0,0.001;
            State X_next = ego.get_model().dynamics(X_cur, U);
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

double CILQRSolver::cal_cost(const Solution& solution){
    if (std::isnan(solution.ego_trj.get_states()[0][0])) {
        throw std::runtime_error("trajectory contains NaN values");
    }
    Vector2d P2;
    P2<<0,1;
    auto control_sequence = solution.control_sequence.get_control_sequence();
    auto trj              = solution.ego_trj.get_states();
    double cost_state = 0;
    double cost_state_ref = 0;
    double cost_ctrl = 0;; 
    double cost_lane = 0;
    double cost_obs = 0;
    double cost_steer = 0;
    double J_state_total= 0;
    double J_ctrl_total = 0;
    double J_constraint_total = 0;
    //计算状态代价
    for(int i=0;i<arg.N+1;i++){

        State X = trj[i];
        size_t index =  find_closest_point(ego.get_local_plan().get_points(),X);
        size_t match_index = index == ego.get_local_plan().get_points().size()-1?index:index+1;
        // match_index = index;
        Point X_r_point = ego.get_local_plan().get_points()[match_index];
        State X_r = {X_r_point.x, X_r_point.y, arg.desire_heading, arg.desire_speed};
        State X_e = X - X_r;
        cost_state = X_e.transpose() * arg.Q * X_e;
        //计算横向偏移代价
        Vector2d dX,nor_r;
        dX<< X_e[0],X_e[1];
        nor_r<< -sin(X_r_point.heading),cos(X_r_point.heading);
        cost_state_ref = pow(dX.dot(nor_r),2) * arg.ref_weight;
        //计算超越车道边界代价  
        if(arg.if_cal_lane_cost){
            //左侧超越车道边界代价
            double l = dX.transpose() * nor_r;
            double c_left = l - arg.trace_safe_width_left;
            double cost_lane_left = arg.lane_q1*exp(arg.lane_q2*c_left);
            //右侧超越车道边界代价
            double c_right = -l - arg.trace_safe_width_right;
            double cost_lane_right = arg.lane_q1*exp(arg.lane_q2*c_right);
            cost_lane = cost_lane_left + cost_lane_right;
        }
        //计算障碍物代价
        if(arg.if_cal_obs_cost){
            //计算与障碍物的距离
            if(i >= obs.get_states().size()){ // 添加长度检查
                std::cerr << "Obs trajectory error" << std::endl;
                break;
            }
            State obs_state = obs.get_states()[i];
            double dx = X[0] - obs_state[0];  // Ego车与障碍物的x坐标差
            double dy = X[1] - obs_state[1];  // Ego车与障碍物的y坐标差
            double dist = sqrt(dx * dx + dy * dy);       // 计算距离
            
            //障碍物代价计算：若距离小于安全阈值，产生代价
            double safe_distance = arg.obs_rad + ego.get_model().ego_rad; // 安全距离
            //c小于0满足约束  c大于0违反约束
            double c = safe_distance - dist;   
            //返回代价
            cost_obs = arg.obs_q1*exp(arg.obs_q2*c);
        } 
        J_state_total += cost_state + cost_state_ref;
        J_constraint_total += cost_lane + cost_obs;
    }

    //计算控制代价
    for(int i=0;i<this->arg.N;i++){
        Control U = control_sequence[i];
        double cost_ctrl = U.transpose() * arg.R * U;
        //计算前轮转角约束代价
        if(arg.if_cal_steer_cost){
            double c = U.transpose() * P2 - arg.steer_angle_max;
            double cost_max_steer = arg.steer_max_q1*exp(arg.steer_max_q2*c);
            c = arg.steer_angle_min -  U.transpose() * P2 ;
            double cost_min_steer = arg.steer_min_q1*exp(arg.steer_min_q2*c);
            cost_steer = cost_max_steer + cost_min_steer;
        }else{
            cost_steer = 0;
        }
        J_constraint_total += cost_steer;
        J_ctrl_total += cost_ctrl;
    }
    // std::cout<<"state cost:"<<J_state_total<<" ctrl cost:"<<J_ctrl_total<<" constraint cost:"<<J_constraint_total<<std::endl;
     return  J_state_total + J_ctrl_total + J_constraint_total;
}

void CILQRSolver::compute_df(const Solution& solution){
    for(int i=0;i<this->arg.N;i++){
        State X = solution.ego_trj.get_states()[i];
        Control U = solution.control_sequence.get_control_sequence()[i]; 
        this->df_dx[i] = this->ego.get_model().get_jacobian_state(X,U);
        this->df_du[i] = this->ego.get_model().get_jacobian_control(X,U);
    }
}

void CILQRSolver::compute_cost_derivatives(const Solution& solution) {
    const auto& X_traj = solution.ego_trj.get_states();
    const auto& U = solution.control_sequence.get_control_sequence();
    const auto& local_plan = ego.get_local_plan().get_points();
    const auto& obs_traj = obs.get_states();
    const int N = arg.N;

    // 初始化导数矩阵

    Vector2d P2(0, 1); // 转向控制投影向量

    // 第一部分：状态相关导数
    for (int i = 0; i <= N; ++i) {
        const State& X = X_traj[i];
        
        // 找到最近参考点
        size_t index = find_closest_point(local_plan, X) ;
        size_t match_index = index == ego.get_local_plan().get_points().size()-1?index:index+1;
        // match_index = index;
        const Point& X_r_point = local_plan[match_index];
        State X_r;
        X_r << X_r_point.x, X_r_point.y, arg.desire_heading, arg.desire_speed;

        // 状态误差
        State X_e = X - X_r;
        // 基本状态代价导数
        Vector4d l_dx = 2 * arg.Q * X_e;
        Matrix4d l_ddx = 2 * arg.Q;

        //计算横向偏移代价导数
        Vector4d l_dx_ref;
        Matrix4d l_ddx_ref;
        Vector2d dX(X_e[0], X_e[1]);
        Vector2d nor_r(-std::sin(X_r_point.heading), std::cos(X_r_point.heading));
        l_dx_ref << -2*dX.dot(nor_r)*sin(X_r_point.heading),
                     2*dX.dot(nor_r)*cos(X_r_point.heading),
                     0, 
                     0;
        l_ddx_ref << 2*pow(sin(X_r_point.heading),2),       -sin(2*X_r_point.heading), 0, 0,
                           -sin(2*X_r_point.heading), 2*pow(cos(X_r_point.heading),2), 0, 0,
                                                   0,                               0, 0, 0,
                                                   0,                               0, 0, 0;
        l_dx_ref = l_dx_ref * arg.ref_weight;
        l_ddx_ref = l_ddx_ref * arg.ref_weight;

        // 障碍物代价导数
        Vector4d db_obs = Vector4d::Zero();
        Matrix4d ddb_obs = Matrix4d::Zero();
        if (arg.if_cal_obs_cost && i < obs_traj.size()) {
            const State& obs_state = obs_traj[i];
            double dx = X[0] - obs_state[0];
            double dy = X[1] - obs_state[1];
            double dist = std::hypot(dx, dy);
            double safe_dist = arg.obs_rad + ego.get_model().ego_rad;
            
            if (dist < 1e-3) dist = 1e-3; // 避免除以零
            Vector4d c_dot;
            c_dot << -dx/dist, -dy/dist, 0, 0;
            
            double c = safe_dist - dist;
            double exp_term = arg.obs_q1 * arg.obs_q2 * std::exp(arg.obs_q2 * c);
            db_obs = exp_term * c_dot;
            Matrix4d ddc;
            ddc << -1/dist,       0, 0, 0,
                         0, -1/dist, 0, 0,
                         0,       0, 0, 0,
                         0,       0, 0, 0;
            ddb_obs = exp_term * (arg.obs_q2 * c_dot * c_dot.transpose() + ddc);
        }

        // 车道保持代价导数
        Vector4d db_lane_total = Vector4d::Zero();
        Matrix4d ddb_lane_total = Matrix4d::Zero();
        if (arg.if_cal_lane_cost) {

            // 左侧约束
            double l = dX.dot(nor_r);
            double c_left = l - arg.trace_safe_width_left;
            Vector4d dc_left;
            dc_left << -std::sin(X_r_point.heading), std::cos(X_r_point.heading), 0, 0;
            
            auto [b_left, db_left, ddb_left] = barrierFunction(
                arg.lane_q1, arg.lane_q2, c_left, dc_left);

            // 右侧约束
            double c_right = -l - arg.trace_safe_width_right;
            Vector4d dc_right = -dc_left;
            
            auto [b_right, db_right, ddb_right] = barrierFunction(
                arg.lane_q1, arg.lane_q2, c_right, dc_right);

            db_lane_total = db_left + db_right;
            ddb_lane_total = ddb_left + ddb_right;
        }

        this->lx[i] = l_dx + l_dx_ref + db_obs + db_lane_total;
        this->lxx[i] = l_ddx + l_ddx_ref + ddb_obs + ddb_lane_total;
    }

    // 第二部分：控制相关导数
    for (int i = 0; i < N; ++i) {
        const Vector2d& u = U[i];
        
        // 基本控制代价导数
        Vector2d lu_base = 2 * arg.R * u;
        Matrix2d luu_base = 2 * arg.R;

        // 转向约束导数
        Vector2d db_steer = Vector2d::Zero();
        Matrix2d ddb_steer = Matrix2d::Zero();
        if (arg.if_cal_steer_cost) {
            // 最大转向约束
            double c_max = u.dot(P2) - arg.steer_angle_max;
            auto [b_max, db_max, ddb_max] = barrierFunction(
                arg.steer_max_q1, arg.steer_max_q2, c_max, P2);
            
            // 最小转向约束
            double c_min = arg.steer_angle_min - u.dot(P2);
            auto [b_min, db_min, ddb_min] = barrierFunction(
                arg.steer_min_q1, arg.steer_min_q2, c_min, -P2);
            
            db_steer = db_max + db_min;
            ddb_steer = ddb_max + ddb_min;
        }

        // 合并控制导数
        this->lu[i] = lu_base + db_steer;
        this->luu[i] = luu_base + ddb_steer;
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

    // 反向迭代
    for (int i = N-1; i >= 0; --i) { // 注意从N-1开始
        // 获取当前时刻的雅可比矩阵
        const Matrix4d& df_dx = this->df_dx[i];
        const Matrix<double,4,2>& df_du = this->df_du[i];
        
        // 计算Q函数相关量
        Vector4d Qx = lx[i] + df_dx.transpose() * V_x;
        Qu[i] = lu[i] + df_du.transpose() * V_x;
        
        Matrix4d Qxx = lxx[i] + df_dx.transpose() * V_xx * df_dx;
        Matrix<double,2,4> Qux = lux[i] + df_du.transpose() * V_xx * df_dx;
        Quu[i] = luu[i] + df_du.transpose() * V_xx * df_du;



        // SVD分解求逆
        JacobiSVD<Matrix2d> svd(Quu[i], ComputeFullU | ComputeFullV);
        Vector2d singular_values = svd.singularValues();
        const Matrix2d& U = svd.matrixU();
        const Matrix2d& V = svd.matrixV();
        Matrix2d Quu_inv;
        // Quu_inv = (Quu[i] + lambda * Matrix2d::Identity()).inverse();
        // 正则化奇异值
        for(int j=0; j<2; ++j){
            // singular_values[j] = std::max(singular_values[j],1.0);
            // singular_values[j] = singular_values[j] * singular_values[j]/(singular_values[j] + lambda);
            singular_values[j] = std::max(singular_values[j],1e-6) + lamb;
        }
        Quu_inv = V * singular_values.cwiseInverse().asDiagonal() * U.transpose();

        // 计算控制修正量
        this->k[i] = -Quu_inv * Qu[i];
        this->K[i] = -Quu_inv * Qux;

        // 更新价值函数导数
        V_x = Qx - K[i].transpose() * Quu[i] * k[i];
        V_xx = Qxx - K[i].transpose() * Quu[i] * K[i];
    }
}

Solution CILQRSolver::forward(const Solution& cur_solution){
        
    // 初始化线搜索参数
    const int max_iterations = 8;
    double alpha = 1;
    bool found = false;
    double J_old = cal_cost(cur_solution);
    double J_new = 0;
    double delta_cost = 0;
    double delta_V = 0;
    auto& U = cur_solution.control_sequence.get_control_sequence();
    auto& X = cur_solution.ego_trj.get_states();
    static Solution new_solution;
    std::vector<Control> U_tmp;
    std::vector<State> X_tmp;
    Vector4d delta_x;
    for (int iter = 0; iter < max_iterations; ++iter) {
        // 临时存储新控制序列
        U_tmp = U;
        X_tmp = X;

        // 应用控制修正
        for (int i = 0; i < arg.N; ++i) {
            // 计算状态偏差
            delta_x = X_tmp[i] - cur_solution.ego_trj.get_states()[i];
            // 应用控制修正：u_new = u_old + alpha*k + K*delta_x
            U_tmp[i] += alpha * k[i]+ K[i] * delta_x;
            // 前向模拟
            X_tmp[i+1] = ego.get_model().dynamics(X_tmp[i], U_tmp[i]);
            //累计deltaV
            delta_V += alpha * (k[i].transpose() * Qu[i]).value() + alpha * alpha * 0.5 * (k[i].transpose() * Quu[i] * k[i]).value(); 
        }

        // 计算新代价
        new_solution.ego_trj.states.swap(X_tmp);
        new_solution.control_sequence.controls.swap(U_tmp);
        J_new = cal_cost(new_solution);
        
        delta_cost = J_new - J_old;
        
        // 接受条件判断
        if (delta_cost/(delta_V) <10 && delta_cost/(delta_V)>1e-4) {
             found = true;
            break;
        } else {
            alpha *= 0.5;
            // if (alpha < min_alpha) break;
        }
    }
    if (!found) {
        return cur_solution;
    }
    else{
        return new_solution;
    }
}


// 纯跟踪方法
Control CILQRSolver::pure_pursuit(const State& X_cur) {
    const auto& local_plan = ego.get_local_plan().get_points();
    if (local_plan.empty()) {
        throw std::runtime_error("Local plan is empty!");
    }

    // 1. 查找最近点（复用现有函数）
    size_t indexNow = find_closest_point(local_plan, X_cur);

    // 2. 计算前瞻距离
    const double Kv = arg.kv;    // 速度增益系数
    const double Ld0 = arg.ld0;  // 基础前瞻距离
    const double Ld_min = arg.ld_min;
    const double Ld_max = arg.ld_max;
    
    double Ld = std::clamp(Kv * X_cur[3] + Ld0, Ld_min, Ld_max);

    // 3. 查找目标点（需预计算路径累计距离）
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

    // 4. 计算转向角
    const Point& target = local_plan[indexTarget];
    double dx = target.x - X_cur[0];
    double dy = target.y - X_cur[1];
    double alpha = std::atan2(dy, dx) - X_cur[2];
    alpha = angle_wrap(alpha);  

    // 5. 计算前轮转角（使用车辆模型参数）
    double steer = std::atan2(2.0 * ego.get_model().len * std::sin(alpha), Ld);

    // 生成控制指令：加速度保持为0，仅转向控制
    return Control(0.01, steer);
}

