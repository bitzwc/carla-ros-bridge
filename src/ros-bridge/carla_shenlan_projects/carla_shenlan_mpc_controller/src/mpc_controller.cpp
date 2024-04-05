#include "carla_shenlan_mpc_controller/mpc_controller.h"

#include <algorithm>
#include <iomanip>
#include <utility>
#include <vector>

#include "Eigen/LU"
#include "math.h"

//Forward Gradient求解
FG_eval::FG_eval(const Eigen::VectorXd &state,
                 VectorXd coeffs,
                 const double &target_v,
                 const int &cte_weight,
                 const int &epsi_weight,
                 const int &v_weight,
                 const int &steer_actuator_cost_weight,
                 const int &acc_actuator_cost_weight,
                 const int &change_steer_cost_weight,
                 const int &change_accel_cost_weight,
                 const int &mpc_prediction_horizon_length,
                 const int &mpc_control_horizon_length,
                 const double &mpc_control_step_length,
                 const double &kinamatic_para_Lf,
                 const double &a_lateral,
                 const double &old_steer_value,
                 const double &old_throttle_value,
                 const double &steer_ratio)
{
    this->steer_ratio = steer_ratio;
    this->coeffs = coeffs;
    this->ref_v = target_v;
    this->cte_weight = cte_weight;
    this->epsi_weight = epsi_weight;
    this->v_weight = v_weight;
    this->steer_actuator_cost_weight_fg = steer_actuator_cost_weight;
    this->acc_actuator_cost_weight_fg = acc_actuator_cost_weight;
    this->change_steer_cost_weight = change_steer_cost_weight;
    this->change_accel_cost_weight = change_accel_cost_weight;
    this->Np = mpc_prediction_horizon_length;
    this->Nc = mpc_control_horizon_length;
    this->dt = mpc_control_step_length;
    this->Lf = kinamatic_para_Lf;
    this->a_lateral = a_lateral;
    /*Indexes on the 1-D vector for readability， These indexes are used for "vars"*/
    this->x_start = 0;
    this->y_start = x_start + Np;
    this->psi_start = y_start + Np;
    this->v_longitudinal_start = psi_start + Np;
    this->v_lateral_start = v_longitudinal_start + Np;
    this->yaw_rate_start = v_lateral_start + Np;
    this->cte_start = yaw_rate_start + Np;
    this->epsi_start = cte_start + Np;
    this->front_wheel_angle_start = epsi_start + Np;
    this->longitudinal_acceleration_start = front_wheel_angle_start + Nc; 
    //控制时域长度为25的时候，控制增量一共有24个,第一个时刻点控制量与控制增量相同
    // this->front_wheel_angle_increment_start = epsi_start + Np; 
    // this->longitudinal_acceleration_increment_start = front_wheel_angle_increment_start + Nc - 1; 

    this->old_steer_value = old_steer_value;
    this->old_throttle_value = old_throttle_value;

    /* Explicitly gather state values for readability */
    this->x = state[0];
    this->y = state[1];
    this->psi = state[2];
    this->v_longitudinal = state[3];
    this->v_lateral = state[4];
    this->yaw_rate = state[5];
    this->cte = state[6];
    this->epsi = state[7];
}
/******************************************************************************
Name     :
Type     :
Function :
Input(s) :
Retrun(s):
Comments :
*******************************************************************************/
FG_eval::~FG_eval() {}
/******************************************************************************
Name     :
Type     :
Function :
Input(s) :1. fg: 定义目标函数f(x)和约束g(x)
          2. vars: this vector contains all variables used by the cost function 
                   and model: [x,y,psi,v,cte,e_psi][front_wheel_angle,a](state & actuators) 存放的是变量
Retrun(s):
Comments : yaw_rate == yaw_rate
*******************************************************************************/
// 重载函数调用运算符，创建一个可以传递任意数目参数的运算符函数
    // 函数对象,可以将像函数调用一样使用函数对象
void FG_eval::operator()(ADvector &fg, ADvector &vars)
{
    // 部分代码已经给出，请同学们补全，fg[0]为目标函数f(x)
    fg[0] = 0; // 0 is the index at which Ipopt expects fg to store the cost value,
    /* TODO: Objective term 1: Keep close to reference values.*/ 
    //跟踪误差，和参考线的误差要小
    for (size_t t = 0; t < Np; t++)
    {
        fg[0] += cte_weight * CppAD::pow(vars[cte_start + t], 2); //横向跟踪误差(cross track error, 简称CTE) 
        fg[0] += epsi_weight * CppAD::pow(vars[epsi_start + t], 2); //夹角误差
        fg[0] += v_weight * CppAD::pow(vars[v_longitudinal_start + t] - ref_v, 2); 
    }
    /* TODO: Objective term 2: Avoid to actuate as much as possible, minimize the use of actuators.*/
    //控制误差，控制量要小
    for (size_t t = 0; t < Nc; t++)
    {
        fg[0] += steer_actuator_cost_weight_fg * CppAD::pow(vars[front_wheel_angle_start + t], 2);
        fg[0] += acc_actuator_cost_weight_fg * CppAD::pow(vars[longitudinal_acceleration_start + t], 2);
    }
    /* TODO: Objective term 3: Enforce actuators smoothness in change, minimize the value gap between sequential actuation.*/
    //平滑误差，控制量的变化要小
    for (size_t t = 0; t < Nc - 1; t++)
    {
        fg[0] += change_steer_cost_weight * CppAD::pow(vars[front_wheel_angle_start + t + 1] - vars[front_wheel_angle_start + t], 2);
        fg[0] += change_accel_cost_weight * CppAD::pow(vars[longitudinal_acceleration_start + t + 1] - vars[longitudinal_acceleration_start + t], 2);
    }

    /* Initial constraints, initialize the model to the initial state*/
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_longitudinal_start] = vars[v_longitudinal_start];
    fg[1 + v_lateral_start] = vars[v_lateral_start];
    fg[1 + yaw_rate_start] = vars[yaw_rate_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    for (size_t t = 1; t < Np; t++) // 预测模型
    {
        // Values at time (t),第一次进入该循环的时候,x_0...传入的是给MPC的系统状态的真实值,作为MPC求解的初始条件.
        AD<double> x_0 = vars[x_start + t - 1];
        AD<double> y_0 = vars[y_start + t - 1];
        AD<double> psi_0 = vars[psi_start + t - 1];
        AD<double> v_longitudinal_0 = vars[v_longitudinal_start + t - 1] + 0.00001; //防止速度为0
        AD<double> v_lateral_0 = vars[v_lateral_start + t - 1];
        AD<double> yaw_rate_0 = vars[yaw_rate_start + t - 1];
        AD<double> cte_0 = vars[cte_start + t - 1];
        AD<double> epsi_0 = vars[epsi_start + t - 1];

        // Values at time (t+1)
        AD<double> x_1 = vars[x_start + t];
        AD<double> y_1 = vars[y_start + t];
        AD<double> psi_1 = vars[psi_start + t];
        AD<double> v_longitudinal_1 = vars[v_longitudinal_start + t] + 0.00001;
        AD<double> v_lateral_1 = vars[v_lateral_start + t];
        AD<double> yaw_rate_1 = vars[yaw_rate_start + t];
        AD<double> cte_1 = vars[cte_start + t];
        AD<double> epsi_1 = vars[epsi_start + t];

        // Only consider the actuation at time t.
        AD<double> front_wheel_angle_0;
        AD<double> longitudinal_acceleration_0;

        front_wheel_angle_0 = vars[front_wheel_angle_start + t - 1];
        longitudinal_acceleration_0 = vars[longitudinal_acceleration_start + t - 1];

        // reference path
        AD<double> f_0 = coeffs[0] +
                         coeffs[1] * x_0 +
                         coeffs[2] * CppAD::pow(x_0, 2) +
                         coeffs[3] * CppAD::pow(x_0, 3) +
                         coeffs[4] * CppAD::pow(x_0, 4) +
                         coeffs[5] * CppAD::pow(x_0, 5); // + coeffs[6] * CppAD::pow(x_0, 6);

        // reference psi: can be calculated as the tangential angle of the polynomial f evaluated at x_0
        //f_0对x_0求导后的结果
        AD<double> psi_des_0 = CppAD::atan(1 * coeffs[1] +
                                           2 * coeffs[2] * x_0 +
                                           3 * coeffs[3] * CppAD::pow(x_0, 2) +
                                           4 * coeffs[4] * CppAD::pow(x_0, 3) +
                                           5 * coeffs[5] * CppAD::pow(x_0, 4)); // + 6 * coeffs[6] * CppAD::pow(x_0, 5));

        // TODO: 补全车辆动力学模型，作为MPC的等式约束条件，车辆动力学模型需要注意参数的正方向的定义
        /*The idea here is to constraint this value to be 0.*/
        //以下都是约束，下一时刻 = 上一时刻 + 导数 * dt
        /* 全局坐标系 */ 
        fg[1 + x_start + t] = x_1 - (x_0 + (v_longitudinal_0 * CppAD::cos(psi_0) - v_lateral_0 * CppAD::sin(psi_0))*dt);
        fg[1 + y_start + t] = y_1 - (y_0 + (v_longitudinal_0 * CppAD::sin(psi_0) + v_lateral_0 * CppAD::cos(psi_0))*dt);

        /* 航向角变化*/
        //航向角速度,TODO: 这里为什么不是用yaw_rate_0？
        AD<double> a_psi = -v_longitudinal_0 * (front_wheel_angle_0 / 1) / lf;
        fg[1 + psi_start + t] = psi_1 - (psi_0 + a_psi * dt);
        // fg[1 + psi_start + t] = psi_1 - (psi_0 + yaw_rate_0 * dt);

        /* 车辆纵向速度 */
        fg[1 + v_longitudinal_start + t] = v_longitudinal_1 - (v_longitudinal_0 + longitudinal_acceleration_0 * dt);

        /* 车辆横向速度 */
        //横向加速度,TODO: 这里为什么要减去v_longitudinal_0 * yaw_rate_0？为什么正负数不一样？
        AD<double> a_lateral = -v_longitudinal_0 * yaw_rate_0 +
            2/m*(Cf * (-(yaw_rate_0 * lf+ v_lateral_0)/v_longitudinal_0 - front_wheel_angle_0/1)
            + Cr * (yaw_rate_0 * lr - v_lateral_0) / v_longitudinal_0);
        fg[1 + v_lateral_start + t] = v_lateral_1 - (v_lateral_0 + a_lateral * dt);
        
        /* 车辆横摆角速度 */
        //车辆横摆角加速度,TODO: 这里为什么正负号不一样？
        AD<double> yaw_accel = 2/I*(lf*Cf*(-(yaw_rate_0 * lf+ v_lateral_0)/v_longitudinal_0 - front_wheel_angle_0/1)
            - lr*Cr*(yaw_rate_0 * lr - v_lateral_0) / v_longitudinal_0);
        fg[1 + yaw_rate_start + t] = yaw_rate_1 - (yaw_rate_0 + yaw_accel * dt);

        /* 横向位置跟踪误差 */
        fg[1 + cte_start + t] = cte_1 - (f_0 - y_0 + v_longitudinal_0 * CppAD::tan(epsi_0) * dt);
        /* 航向跟踪误差 */
        fg[1 + epsi_start + t] = epsi_1 - (psi_0 - psi_des_0 - v_longitudinal_0 * (front_wheel_angle_0 / 1) / Lf * dt);

    }
}

mpc_controller::mpc_controller() {}

mpc_controller::~mpc_controller() {}

/******************************************************************************
Name     :
Type     :
Function :
Input(s) :
Retrun(s):
Comments : Solve the model given an initial state and polynomial coefficients, return the first actuation
*******************************************************************************/
std::vector<double> mpc_controller::Solve(const Eigen::VectorXd &state,
                                          const Eigen::VectorXd &coeffs,
                                          const double &target_v,
                                          const int &cte_weight,
                                          const int &epsi_weight,
                                          const int &v_weight,
                                          const int &steer_actuator_cost_weight,
                                          const int &acc_actuator_cost_weight,
                                          const int &change_steer_cost_weight,
                                          const int &change_accel_cost_weight,
                                        //   const int &mpc_prediction_horizon_length,
                                          const int &mpc_control_horizon_length,
                                          const double &mpc_control_step_length,
                                          const double &kinamatic_para_Lf,
                                          const double &a_lateral,
                                          const double &old_steer_value,
                                          const double &old_throttle_value,
                                          const double &steer_ratio)
{
    //控制时域长度
    this->Nc = mpc_control_horizon_length;

    //预测时域长度
    this->Np = Nc + 1;

    //每个变量分配对应的位置
    this->x_start = 0;
    this->y_start = x_start + Np;
    this->psi_start = y_start + Np;
    this->v_longitudinal_start = psi_start + Np;
    this->v_lateral_start = v_longitudinal_start + Np;
    this->yaw_rate_start = v_lateral_start + Np;
    this->cte_start = yaw_rate_start + Np;
    this->epsi_start = cte_start + Np;
    this->front_wheel_angle_start = epsi_start + Np;
    this->longitudinal_acceleration_start = front_wheel_angle_start + Nc; //控制时域长度为25的时候，控制量一共有24个。
    // this->front_wheel_angle_increment_start = epsi_start + Np;
    // this->longitudinal_acceleration_increment_start = front_wheel_angle_increment_start + Nc - 1;

    this->a_lateral = a_lateral;
    bool ok = true;

    typedef CPPAD_TESTVECTOR(double) Dvector;

    /* Explicitly gather state values for readability */
    double x = state[0];
    double y = state[1];
    double psi = state[2];
    double v_longitudinal = state[3];
    double v_lateral = state[4];
    double yaw_rate = state[5];
    double cte = state[6];
    double epsi = state[7];

    /* Set the number of model variables (includes both states and inputs). */
    /* 系统状态量 + 系统控制增量 */
    size_t n_vars = Np * 8 + Nc * 2;

    /* Set the number of contraints */
    size_t n_constraints = Np * 8;

    /* Initialize all of independent variables to zero. */
    Dvector vars(n_vars);
    for (size_t i = 0; i < n_vars; i++)
    {
        vars[i] = 0.0;
    }
    // Set the initial variable values**初始化变量及变量上下限**
    vars[x_start] = x;
    vars[y_start] = y;
    vars[psi_start] = psi;
    vars[v_longitudinal_start] = v_longitudinal;
    vars[v_lateral_start] = v_lateral;
    vars[yaw_rate_start] = yaw_rate;
    vars[cte_start] = cte;
    vars[epsi_start] = epsi;

    Dvector vars_lower_bounds(n_vars);
    Dvector vars_upper_bounds(n_vars);

    /* Set limits for non-actuators (avoid numerical issues during optimization). */
    for (size_t i = 0; i < front_wheel_angle_start; i++)
    {
        vars_lower_bounds[i] = -std::numeric_limits<float>::max();
        vars_upper_bounds[i] = +std::numeric_limits<float>::max();
    }
    /* Set upper and lower constraints for steering. */
    double max_degree = 24;
    double max_radians = max_degree * M_PI / 180;
    for (size_t i = front_wheel_angle_start; i < longitudinal_acceleration_start; i++)
    {
        vars_lower_bounds[i] = -max_radians;
        vars_upper_bounds[i] = +max_radians;
    }

    /* Set uppper and lower constraints for acceleration. */
    double max_acceleration_value = 1.0;
    for (size_t i = longitudinal_acceleration_start; i < n_vars; i++)
    {
        vars_lower_bounds[i] = -max_acceleration_value;
        vars_upper_bounds[i] = +max_acceleration_value;
    }
    
    /* Initialize to zero lower and upper limits for the constraints*/
    Dvector constraints_lower_bounds(n_constraints);
    Dvector constraints_upper_bounds(n_constraints);
    for (size_t i = 0; i < n_constraints; i++)
    {
        constraints_lower_bounds[i] = 0;
        constraints_upper_bounds[i] = 0;
    }
    /* Force the solver to start from current state in optimization space. */
    /* 约束条件的形式为 [x(0); x(1) = f(x(0)); ... x(N) = f(x(N-1)); , 每一个约束的第一项其实就是系统当前状态，所以直接约束该量的第一项。
       不能直接给状态 vars 的第一项赋值来限制求解起点，因为状态和控制量理论上都是自由的，所以这里通过设置约束条件第一项的方式来指定优化求解起点，从而加速求解。*/
    constraints_lower_bounds[x_start] = x;
    constraints_lower_bounds[y_start] = y;
    constraints_lower_bounds[psi_start] = psi;
    constraints_lower_bounds[v_longitudinal_start] = v_longitudinal;
    constraints_lower_bounds[v_lateral_start] = v_lateral;
    constraints_lower_bounds[yaw_rate_start] = yaw_rate;
    constraints_lower_bounds[cte_start] = cte;
    constraints_lower_bounds[epsi_start] = epsi;

    constraints_upper_bounds[x_start] = x;
    constraints_upper_bounds[y_start] = y;
    constraints_upper_bounds[psi_start] = psi;
    constraints_upper_bounds[v_longitudinal_start] = v_longitudinal;
    constraints_upper_bounds[v_lateral_start] = v_lateral;
    constraints_upper_bounds[yaw_rate_start] = yaw_rate;
    constraints_upper_bounds[cte_start] = cte;
    constraints_upper_bounds[epsi_start] = epsi;
    /* Object that computes objective and constraints. */
    FG_eval fg_eval(state,
                    coeffs,
                    target_v,
                    cte_weight,
                    epsi_weight,
                    v_weight,
                    steer_actuator_cost_weight,
                    acc_actuator_cost_weight,
                    change_steer_cost_weight,
                    change_accel_cost_weight,
                    Np,
                    Nc,
                    mpc_control_step_length,
                    kinamatic_para_Lf,
                    a_lateral,
                    old_steer_value,
                    old_throttle_value,
                    steer_ratio);
    /* NOTE: You don't have to worry about these options. options for IPOPT solver. */
    std::string options;
    /* Uncomment this if you'd like more print information. */
    //关闭任何优化器的输出
    options += "Integer print_level  0\n";
    /* NOTE: Setting sparse to true allows the solver to take advantage
       of sparse routines, this makes the computation MUCH FASTER. If you can
       uncomment 1 of these and see if it makes a difference or not but if you
       uncomment both the computation time should go up in orders of magnitude. */
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    /* NOTE: Currently the solver has a maximum time limit of 0.5 seconds. */
    /* Change this as you see fit. */
    options += "Numeric max_cpu_time          0.1\n";

    /* Place to return solution. */
    CppAD::ipopt::solve_result<Dvector> solution;

    /* Solve the problem. */
    // cout << "Solve the problem......" << endl;
    //operator重载的函数在这里被调用
    //https://blog.csdn.net/BigDavid123/article/details/136310264
    // option：求解器的配置项
    // vars：待求解的优化变量(包含初值)
    // vars_lower_bounds：待优化变量的下限
    // vars_upper_bounds：待优化变量的上限
    // constraints_lower_bounds：约束表达式的下限
    // constraints_upper_bounds：约束表达式的上限
    // fg_eval：存储目标函数和约束变量的表达式
    // solution：优化后的结果保存在这里
    CppAD::ipopt::solve<Dvector, FG_eval>(
        options,
        vars,
        vars_lower_bounds, 
        vars_upper_bounds,
        constraints_lower_bounds, 
        constraints_upper_bounds,
        fg_eval,
        solution
    );

    /* Check some of the solution values. */
    //判断求解是否成功
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    /* Cost! */
    // auto cost = solution.obj_value;
    // std::cout << "Cost: " << cost << std::endl;


    /* Return the first actuator values. The variables can be accessed with 'solution.x[i]'. */
    std::vector<double> result;
    result.clear();
    // std::cout << solution.x << std::endl;
    std::cout << "转向角：" << solution.x[front_wheel_angle_start];    
    std::cout << "纵向加速度：" << solution.x[longitudinal_acceleration_start] << std::endl;

    //solution.x是求解的结果，包含8*Np + 2*Nc个变量，通过前轮转角索引和纵向加速度索引来获取2个控制量
    result.push_back(solution.x[front_wheel_angle_start]);
    result.push_back(solution.x[longitudinal_acceleration_start]);

    /* Add 'future' solutions (where MPC is going). */
    // 将Np个位置x和Np个位置y写入result，作为未来Np个时域的车辆位置预测
    for (size_t i = 0; i < Np - 1; i++)
    {
        result.push_back(solution.x[x_start + i]);
        result.push_back(solution.x[y_start + i]);
    }
    // std::cout << "end now" << std::endl;
    return result;
}
