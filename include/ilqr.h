#ifndef ILQR_H
#define ILQR_H

#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <algorithm>

using namespace Eigen;
// #define M_PI 3.1415
typedef Vector4d State;
typedef Vector2d Control;

struct Arg{
    // 仿真参数
    double tf = 1000;
    double dt = 0.1;
    //CILQR参数
    int N = 100; //Horizen
    double tol = 1;
    double rel_tol = 1e-5;
    int max_iter = 50;
    double lamb_init = 7.0;               // 优化：从10降到7，减少lambda调节次数
    double lamb_factor = 2;
    double lamb_max = 6000;
    //纯跟踪参数
    double kv = 0.3; //前视距离系数
    double kp = 0.8; //速度P控制器系数
    double ld0 = 3;  //基础前瞻距离
    double ld_min = 3;
    double ld_max = 20;
    //代价参数
    double desire_speed = 15;
    double desire_heading = 0;
    bool if_cal_obs_cost = true;
    bool if_cal_lane_cost = false;
    bool if_cal_speed_rate_cost = true;
    double v_rate_weight = 0.8;           // 优化：从1.0降到0.8，改善速度平滑性
    
    // 铰接角gamma约束参数
    bool if_cal_gamma_barrier = true;     // 对状态gamma施加上下限barrier
    double gamma_max = 1.0;               // 铰接角上限
    double gamma_max_q1 = 1.0;            // 铰接角上限barrier权重
    double gamma_max_q2 = 3.0;            // 铰接角上限barrier曲率
    double gamma_min = -1.0;              // 铰接角下限
    double gamma_min_q1 = 1.0;            // 铰接角下限barrier权重
    double gamma_min_q2 = 3.0;            // 铰接角下限barrier曲率
    
    // 铰接角速度gamma_dot约束参数
    bool if_cal_gamma_dot_barrier = true; // 对控制gamma_dot施加上下限barrier
    double gamma_dot_max = 0.8;           // 优化：从0.6增加到0.8，进一步放宽约束33%
    double gamma_dot_max_q1 = 0.5;        // 优化：从1.0降到0.5，降低barrier权重50%
    double gamma_dot_max_q2 = 2.0;        // 优化：从3.0降到2.0，降低barrier曲率33%
    double gamma_dot_min = -0.8;          // 优化：从-0.6调整到-0.8，保持对称
    double gamma_dot_min_q1 = 0.5;        // 优化：从1.0降到0.5，降低barrier权重50%
    double gamma_dot_min_q2 = 2.0;        // 优化：从3.0降到2.0，降低barrier曲率33%
    
    //道路约束
    double trace_safe_width_left = 5;
    double trace_safe_width_right = 5;
    double lane_q1 = 5;
    double lane_q2 = 3;
    //障碍约束
    double obs_q1 = 20;                 // 优化：从5降到4.25，降低15%
    double obs_q2 = 3.4;                  // 优化：从4降到3.4，降低15%
    // double obs_length = 2.7;
    // double obs_width = 2;
    double obs_length = 5;
    double obs_width = 3;
    double safe_a_buffer = 0.5;
    double safe_b_buffer = 0.5;
    // double buff = 0;
    // double obs_rad = 1 + buff;
    //QR矩阵
    Matrix4d Q;
    Matrix2d R;
    //横向偏移代价
    double ref_weight = 8.0;              // 优化：从10降到8，降低20%
    Arg() { // 在构造函数中初始化矩阵
        Q << 0.2, 0, 0, 0,                // 优化：位置权重从1增加到1.2，增加20%
             0, 0.2, 0, 0,                // 优化：位置权重从1增加到1.2，增加20%
             0, 0, 0.1, 0,                // 航向权重保持不变
             0, 0, 0, 1.5;                // 优化：铰接角权重从1增加到1.5，增加50%

        R <<    0.1,    0,                  // 速度控制权重保持不变
                0,    8;                  // 优化：铰接角控制权重从10降到8，降低20%
    }
};
//路点结构体
struct Point{
    Point(double X,double Y,double Heading){
        x = X;
        y = Y;
        heading = Heading;
    }
    bool operator==(const Point& other) const {
        return (x == other.x && y == other.y && heading == other.heading);
    }
    double x;
    double y;
    double heading;
};

struct BarrieInfo{
    double b;
    VectorXd d_b;
    MatrixXd dd_b;
};


//计算两点之间距离
inline double distance(const Point& p1, const Point& p2);
//找到路径中离自车最短距离的点
inline size_t find_closest_point(const std::vector<Point>& path,const State& state);
//角度归一化到[-π, π]
inline double angle_wrap(double theta) {
    theta = fmod(theta + M_PI, 2.0*M_PI); 
    if (theta < 0.0){
        theta += 2.0*M_PI;
    }
    return theta - M_PI;
}

BarrieInfo barrierFunction(double q1, double q2, double c, VectorXd dc);
//全局路径
class GlobalPlan{
    private:
        std::vector<Point> points;
    public:
        std::vector<Point> get_points() const{
            return this->points;
        };
        void set_plan(const std::vector<Point>& points){
            this->points = points;
        };
};
//局部路径
class LocalPlan{
    private:
        std::vector<Point> points;
    public:
        std::vector<Point> get_points() const{
            return this->points;
        };
        void set_plan(const GlobalPlan& global_plan,const State& vehicle_state,size_t num_points_to_extract);
};

//系统模型
class SystemModel{
    public:
        double ego_rad = 7;
        // double lf      = 1.6;
        // double lr      =  1.13;
        // double len       =  2.73;
        double lf      = 3;
        double lr      =  3;
        double len       =  3;
        double width   =  3;
        double box_length = 5;
        double dt = 0.1;
        size_t N = 50;
        SystemModel() = default;
        SystemModel(double dt,size_t N):dt(dt),N(N){};
        State dynamics(const State& X, const Control& U);
        Matrix4d get_jacobian_state(const Vector4d& X, const Vector2d& U);
        Matrix<double,4,2> get_jacobian_control(const Vector4d& X, const Vector2d& U);
};
//车辆类
class Vehicle{
    private:
        //车辆状态
        State state;
        GlobalPlan global_plan;
        LocalPlan local_plan;
        SystemModel model;
    public:
        Vehicle();

        //设置or获取车辆状态
        void set_state(double x,double y,double heading,double v){
            this->state << x,y,heading,v;
        };
        void set_state(const State& X){
            this->state << X;
        };
        Vector4d get_state() const{
            return this->state;
        };

        //设置全局路径
        void set_global_plan(const GlobalPlan& global_plan){
            this->global_plan = global_plan;
        };
        //获取全局路径
        GlobalPlan get_global_plan(){
            return this->global_plan;
        };
        //设置or获取局部路径
        void set_local_plan(){
            size_t num_points_to_extract = static_cast<size_t>(std::max<double>((state[3] * model.dt * model.N),100) + 300);
            this->local_plan.set_plan(this->global_plan,this->state,num_points_to_extract);
        };
        LocalPlan get_local_plan(){
            return this->local_plan;
        };
        //设置车辆模型
        void set_model(const SystemModel& model){
            this->model = model;
        };
        SystemModel get_model(){
            return this->model;
        };

};

class Trajectory{
    public:
        //状态点集合
        std::vector<State> states;

        Trajectory() = default;
        Trajectory(const std::vector<State>& states) : states(states) {} // 
        //获取轨迹状态集合
        std::vector<State> get_states() const{
            return this->states;
        };
        void set_states(const std::vector<State>& states){
            this->states = states;
        };
        //添加轨迹尾的状态
        void push_back(const State& state){
            this->states.push_back(state);
        };
        //获取轨迹末端状态
        State back(){
            return this->states.back();
        };
};

class ControlSequence {
        
    public:
        std::vector<Control> controls; 
        ControlSequence() = default;
        explicit ControlSequence(const std::vector<Control>& ctrls) : controls(ctrls) {}
        
        std::vector<Control> get_control_sequence() const { 
            return controls; 
        }
        void push_back(const Control& control) { 
            controls.push_back(control); 
        }
        // 添加边界检查
        Control& operator[](size_t i) { 
            if (i >= controls.size()) 
                throw std::out_of_range("ControlSquence index out of range");
            return controls[i]; 
        }
        size_t size() const { return controls.size(); }
    };
struct Solution {
    Solution(){}
    Solution(Trajectory ego_trj,ControlSequence control_sequence){
        this->ego_trj = ego_trj;
        this->control_sequence = control_sequence;
    }
    Solution(const Solution& solution){
        this->ego_trj = solution.ego_trj;
        this->control_sequence = solution.control_sequence;
        this->converged = solution.converged;
        this->iterations = solution.iterations;
        this->final_cost = solution.final_cost;
        this->solve_time_ms = solution.solve_time_ms;
    }
    Trajectory ego_trj;
    ControlSequence control_sequence;
    
    // 收敛信息
    bool converged = false;
    int iterations = 0;
    double final_cost = 0.0;
    double solve_time_ms = 0.0;
};



class CILQRSolver{
    private:
        // double J_total = 0;
        double lamb ;
        double average_gradient = 0.0;
        bool converged = false;
        Solution pre_solution;
        Vehicle ego;
        std::vector<Trajectory> obs_list;
        Arg arg;
        
        // 日志记录相关
        std::string log_filename;
        std::ofstream cost_log_file;
        bool enable_logging = true;
        
        std::vector<MatrixXd> k;
        std::vector<MatrixXd> K;
        std::vector<MatrixXd> df_dx;
        std::vector<MatrixXd> df_du;
        std::vector<MatrixXd> lx;
        std::vector<MatrixXd> lu;
        std::vector<MatrixXd> lxx;
        std::vector<MatrixXd> luu;
        std::vector<MatrixXd> lux;
        std::vector<MatrixXd> Qu;
        std::vector<MatrixXd> Quu;
        Control pure_pursuit(const State& X_cur);
        Solution get_nominal_solution(const State& init_state);
        double cal_cost(const Solution& solution);
        double cal_cost_with_logging(const Solution& solution, int iteration);
        void compute_df(const Solution& solution);
        void compute_cost_derivatives(const Solution& solution);
        void backward();
        Solution forward(const Solution& solution);
        
        // 日志记录方法
        void init_cost_logging();
        void log_cost_breakdown(int iteration, double total_cost, 
                                  double J_position, double J_heading, double J_gamma_state, double J_lateral_ref,
                                  double J_velocity, double J_gamma_ctrl, 
                                  double J_obs, double J_lane, double J_gamma_barrier, double J_gamma_dot_barrier,
                                  double J_speed_rate, double lambda_value);
        void close_cost_logging();

    public:
        //构造函数
        CILQRSolver(const Vehicle& ego, const std::vector<Trajectory>& obs_list, const Arg& arg) 
        : ego(ego), obs_list(obs_list), arg(arg), lamb(arg.lamb_init),
        k(arg.N),
        K(arg.N),
        df_dx(arg.N),
        df_du(arg.N),
        lx(arg.N+1),
        lu(arg.N),
        lxx(arg.N+1),
        luu(arg.N),
        lux(arg.N),
        Qu(arg.N),
        Quu(arg.N){
            // 初始化矩阵向量
            for(int i = 0; i < arg.N; ++i) {
                k[i] = MatrixXd::Zero(2,1);
                K[i] = MatrixXd::Zero(2,4);
                df_dx[i] = MatrixXd::Zero(4,4);
                df_du[i] = MatrixXd::Zero(4,2);
                lu[i] = MatrixXd::Zero(2,1);
                luu[i] = MatrixXd::Zero(2,2);
                lux[i] = MatrixXd::Zero(2,4);
                Qu[i] = MatrixXd::Zero(2,1);
                Quu[i] = MatrixXd::Zero(2,2);
            }
            for(int i = 0; i < arg.N+1; ++i) {
                lx[i] = MatrixXd::Zero(4,1);
                lxx[i] = MatrixXd::Zero(4,4);
            }
            
            // 初始化日志记录
            init_cost_logging();
        }
        
        // 析构函数
        ~CILQRSolver() {
            close_cost_logging();
        }
        
        //接口
        Solution solve(const State& init_state,const std::vector<Trajectory>& obs_list);

};

class ALILQRSolver{
    private:
        double lamb;
        double average_gradient = 0.0;
        bool converged = false;
        Solution pre_solution;
        Vehicle ego;
        std::vector<Trajectory> obs_list;
        Arg arg;
        std::vector<MatrixXd> k;
        std::vector<MatrixXd> K;
        std::vector<MatrixXd> df_dx;
        std::vector<MatrixXd> df_du;
        std::vector<MatrixXd> lx;
        std::vector<MatrixXd> lu;
        std::vector<MatrixXd> lxx;
        std::vector<MatrixXd> luu;
        std::vector<MatrixXd> lux;
        std::vector<MatrixXd> Qu;
        std::vector<MatrixXd> Quu;
        double rho = 50.0;
        double rho_max = 1e8;
        int max_outer = 20;
        double constraint_tol = 1e-4;
        std::vector<double> lambda_obs;
        std::vector<double> lambda_lane;
        std::vector<double> lambda_gamma_max;
        std::vector<double> lambda_gamma_min;
        std::vector<double> lambda_gdot_max;
        std::vector<double> lambda_gdot_min;
        Control pure_pursuit(const State& X_cur);
        Solution get_nominal_solution(const State& init_state);
        double cal_cost(const Solution& solution);
        double cal_cost_with_logging(const Solution& solution, int iteration);
        void compute_df(const Solution& solution);
        void compute_al_derivatives(const Solution& solution);
        void backward();
        Solution forward(const Solution& solution);
        double compute_max_constraint_violation(const Solution& solution);
    public:
        ALILQRSolver(const Vehicle& ego, const std::vector<Trajectory>& obs_list, const Arg& arg)
        : ego(ego), obs_list(obs_list), arg(arg), lamb(arg.lamb_init),
        k(arg.N), K(arg.N), df_dx(arg.N), df_du(arg.N), lx(arg.N+1), lu(arg.N), lxx(arg.N+1), luu(arg.N), lux(arg.N), Qu(arg.N), Quu(arg.N),
        lambda_obs(arg.N+1, 0.0), lambda_lane(arg.N+1, 0.0), lambda_gamma_max(arg.N+1, 0.0), lambda_gamma_min(arg.N+1, 0.0),
        lambda_gdot_max(arg.N, 0.0), lambda_gdot_min(arg.N, 0.0)
        {
            for(int i = 0; i < arg.N; ++i) {
                k[i] = MatrixXd::Zero(2,1);
                K[i] = MatrixXd::Zero(2,4);
                df_dx[i] = MatrixXd::Zero(4,4);
                df_du[i] = MatrixXd::Zero(4,2);
                lu[i] = MatrixXd::Zero(2,1);
                luu[i] = MatrixXd::Zero(2,2);
                lux[i] = MatrixXd::Zero(2,4);
                Qu[i] = MatrixXd::Zero(2,1);
                Quu[i] = MatrixXd::Zero(2,2);
            }
            for(int i = 0; i < arg.N+1; ++i) {
                lx[i] = MatrixXd::Zero(4,1);
                lxx[i] = MatrixXd::Zero(4,4);
            }
        }
        Solution solve(const State& init_state,const std::vector<Trajectory>& obs_list);
};
#endif
