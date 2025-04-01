#ifndef ILQR_H
    #define ILQR_H 
    #include <Eigen/Eigen>
    #include <algorithm>
    using namespace Eigen;
    #define M_PI 3.1415
    typedef Vector4d State;
    typedef Vector2d Control;

    struct Arg{
        // 仿真参数
        double tf = 1000;
        double dt = 0.1;
        //CILQR参数
        int N = 80; //Horizen
        double tol = 1e-3;
        double rel_tol = 1e-5;
        int max_iter = 50;
        double lamb_init = 1e-2;
        double lamb_factor = 2;
        double lamb_max = 100;
        //纯跟踪参数
        double kv = 0.3; //前视距离系数
        double kp = 0.8; //速度P控制器系数
        double ld0 = 3;  //基础前瞻距离
        double ld_min = 3;
        double ld_max = 20;
        //代价参数
        double desire_speed = 10;
        double desire_heading = 0;
        bool if_cal_obs_cost = true;
        bool if_cal_lane_cost = true;
        bool if_cal_steer_cost = true;
        //最大转向约束
        double steer_angle_max = 1;
        double steer_max_q1 = 1;
        double steer_max_q2 = 1;
        //最小转向约束
        double steer_angle_min = -1;
        double steer_min_q1 = 1;
        double steer_min_q2 = 1;
        //道路约束
        double trace_safe_width_left = 4;
        double trace_safe_width_right = 4;
        double lane_q1 = 5;
        double lane_q2 = 3;
        //障碍约束
        double obs_q1 = 5;
        double obs_q2 = 3;
        double obs_length = 2.7;
        double obs_width = 2;
        double safe_a_buffer = 5;
        double safe_b_buffer = 1;
        // double buff = 0;
        // double obs_rad = 1 + buff;
        //QR矩阵
        Matrix4d Q;
        Matrix2d R;
        //横向偏移代价
        double ref_weight = 3;
        Arg() { // 在构造函数中初始化矩阵
            Q << 0.01, 0, 0, 0, 
                 0, 0.01, 0, 0,
                 0, 0, 0, 0,
                 0, 0, 0, 1;

            R <<    0.1,    0,
                    0,    100;
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
            double ego_rad = 2;
            double lf      = 1.6;
            double lr      =  1.13;
            double len       =  2.73;
            double width   =  2;
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
                size_t num_points_to_extract = static_cast<size_t>(std::max<double>((state[3] * model.dt * model.N),0.0) + 20);
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
        }
        Trajectory ego_trj;
        ControlSequence control_sequence;
    };



    class CILQRSolver{
        private:
            // double J_total = 0;
            double lamb ;
            double average_gradient = 0.0;
            bool converged = false;
            Solution pre_solution;
            Vehicle ego;
            Trajectory obs;
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
            Control pure_pursuit(const State& X_cur);
            Solution get_nominal_solution(const State& init_state);
            double cal_cost(const Solution& solution);
            void compute_df(const Solution& solution);
            void compute_cost_derivatives(const Solution& solution);
            void backward();
            Solution forward(const Solution& solution);

        public:
            //构造函数
            CILQRSolver(const Vehicle& ego, const Trajectory& obs, const Arg& arg) 
            : ego(ego), obs(obs), arg(arg), 
            k(arg.N, Vector2d::Zero()),
            K(arg.N,MatrixXd::Zero(2,4)),
            df_dx(arg.N,MatrixXd::Zero(4,4)),
            df_du(arg.N,MatrixXd::Zero(4,2)),
            lx(arg.N+1, Vector4d::Zero()),
            lu(arg.N, Vector2d::Zero()),
            lxx(arg.N+1, Matrix4d::Zero()),
            luu(arg.N, Matrix2d::Zero()),
            lux(arg.N, MatrixXd::Zero(2,4)),
            Qu(arg.N, Vector2d::Zero()),
            Quu(arg.N, MatrixXd::Zero(2,2)){}
            
            //接口

            Solution solve(const State& init_state,const Trajectory& obs);

    };
#endif
