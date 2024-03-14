#include "carla_shenlan_lqr_pid_controller/lqr_controller.h"

#include <algorithm>
#include <iomanip>
#include <utility>
#include <vector>

#include "Eigen/LU"
#include "math.h"

using namespace std;

namespace shenlan {
    namespace control {

        LqrController::LqrController() {}

        LqrController::~LqrController() {}

// lqr的配置
        void LqrController::LoadControlConf() {
            ts_ = 0.01;    // 每隔0.01s进行一次控制

            cf_ = 155494.663;                              // 前轮侧偏刚度,左右轮之和
            cr_ = 155494.663;                              // 后轮侧偏刚度,左右轮之和
            wheelbase_ = 2.852;                            // 前后轮轴距
            steer_ratio_ = 16;                             // 方向盘的转角到轮胎转动角度之间的比值系数
            steer_single_direction_max_degree_ = 470.0;    // 最大方向转角

            const double mass_fl = 1845.0/4;                     // 左前悬的质量
            const double mass_fr = 1845.0/4;                     // 右前悬的质量
            const double mass_rl = 1845.0/4;                     // 左后悬的质量
            const double mass_rr = 1845.0/4;                     // 右后悬的质量
            const double mass_front = mass_fl + mass_fr;    // 前悬质量
            const double mass_rear = mass_rl + mass_rr;     // 后悬质量
            mass_ = mass_front + mass_rear;

            lf_ = wheelbase_ * (1.0 - mass_front / mass_);    // 汽车前轮到中心点的距离
            lr_ = wheelbase_ * (1.0 - mass_rear / mass_);     // 汽车后轮到中心点的距离

            // moment of inertia
            iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;    // 汽车的转动惯量

            lqr_eps_ = 0.01;              // LQR 迭代求解精度
            lqr_max_iteration_ = 1500;    // LQR的迭代次数

            return;
        }

// 初始化控制器的参数
        void LqrController::Init() {
            /*
            A matrix (Gear Drive)
            [0.0,                             1.0,                           0.0,                                            0.0;
             0.0,          (-(c_f + c_r) / m) / v,               (c_f + c_r) / m,                (l_r * c_r - l_f * c_f) / m / v;
             0.0,                             0.0,                           0.0,                                            1.0;
             0.0, ((lr * cr - lf * cf) / i_z) / v, (l_f * c_f - l_r * c_r) / i_z, (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
            */
            // 初始化A矩阵的常数项，matrix_a_为常数项矩阵
            matrix_a_ = Matrix::Zero(basic_state_size_, basic_state_size_);
            matrix_a_(0, 1) = 1.0;
            matrix_a_(1, 2) = (cf_ + cr_) / mass_;
            matrix_a_(2, 3) = 1.0;
            matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;

            // 初始化A矩阵的非常数项系数，还需要除以速度v，v不能是0
            matrix_a_coeff_ = Matrix::Zero(basic_state_size_, basic_state_size_);
            matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
            matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
            matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
            matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;

            /*
            b = [0.0, c_f / m, 0.0, l_f * c_f / i_z]^T
            */
            // 初始化B矩阵
            matrix_b_ = Matrix::Zero(basic_state_size_, 1);
            matrix_b_(1, 0) = cf_ / mass_;
            matrix_b_(3, 0) = lf_ * cf_ / iz_;

            //A的离散化矩阵
            matrix_ad_ = Matrix::Zero(basic_state_size_, basic_state_size_);

            // B的离散化矩阵
            matrix_bd_ = Matrix::Zero(basic_state_size_, 1);

            // 状态向量x
            matrix_state_ = Matrix::Zero(basic_state_size_, 1);

            // 反馈矩阵K
            matrix_k_ = Matrix::Zero(1, basic_state_size_);

            // lqr cost function中 状态向量x的权重，对角矩阵Q，具体的系数可以调整
            /*
                Q = [
                    2, 0, 0,   0
                    0, 1, 0,   0
                    0, 0, 0.1, 0
                    0, 0, 0,   0.1]
            */
            matrix_q_ = Matrix::Zero(basic_state_size_, basic_state_size_);

            // int q_param_size = 4;
            matrix_q_(0, 0) = 4;    // TODO: lateral_error
            matrix_q_(1, 1) = 1;    // TODO: lateral_error_rate
            matrix_q_(2, 2) = 0.1;    // TODO: heading_error
            matrix_q_(3, 3) = 0.1;    // TODO: heading__error_rate

            //更新Q矩阵，这个是做什么的？
            matrix_q_updated_ = matrix_q_;

            // lqr cost function中 输入控制量u的权重R = [10]
            matrix_r_ = Matrix::Identity(1, 1);
            matrix_r_(0, 0) = 10;

            return;
        }

// 两点之间的距离平方
        double PointDistanceSquare(const TrajectoryPoint &point, const double x, const double y) {
            double dx = point.x - x;
            double dy = point.y - y;
            return dx * dx + dy * dy;
        }

// 将角度(弧度制)归化到[-M_PI, M_PI]之间
        double NormalizeAngle(const double angle) {
            //浮点数取模，fmod(4.3, 2.1) = 0.1，fmod(-4.3, 2.1) = -0.1
            double a = std::fmod(angle + M_PI, 2.0 * M_PI);
            if (a < 0.0) {
                a += (2.0 * M_PI);
            }
            return a - M_PI;
        }
        double atan2_to_PI(const double atan2) { return atan2 * M_PI / 180; }

// **TODO **计算控制命令 该函数定频调用 控制大小赋值给cmd.steer_target
        bool LqrController::ComputeControlCommand(const VehicleState &localization,
                                                  const TrajectoryData &planning_published_trajectory, ControlCmd &cmd) {
            // 规划轨迹
            trajectory_points_ = planning_published_trajectory.trajectory_points;

            /*
            A matrix (Gear Drive)
            [0.0,                               1.0,                            0.0,                                               0.0;
             0.0,            (-(c_f + c_r) / m) / v,                (c_f + c_r) / m,                   (l_r * c_r - l_f * c_f) / m / v;
             0.0,                               0.0,                            0.0,                                               1.0;
             0.0,   ((lr * cr - lf * cf) / i_z) / v,   (l_f * c_f - l_r * c_r) / i_z,   (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
            */
            // TODO 01 配置状态矩阵A，这里为了简化，在对x和u做离散化时，用的都是向前欧拉法 此时A离散化矩阵Ad = A * ts + I
            //速度避免为0
            matrix_ad_ = (matrix_a_ + matrix_a_coeff_ / max(localization.vx, 0.01)) * ts_
                         + Matrix::Identity(basic_state_size_, basic_state_size_);

            /*
            b = [0.0, c_f / m, 0.0, l_f * c_f / i_z]^T
            */
            // TODO 02 动力矩阵B，离散化B矩阵，Bd = B * ts
            matrix_bd_ = matrix_b_ * ts_;
            // cout << "matrix_bd_.row(): " << matrix_bd_.rows() << endl;
            // cout << "matrix_bd_.col(): " << matrix_bd_.cols() << endl;
            // Update state = [Lateral Error, Lateral Error Rate, Heading Error, Heading Error Rate]

            // TODO 03 计算横向误差并且更新状态向量x，matrix_state_
            UpdateState(localization);

            // TODO 04 更新状态矩阵A并将状态矩阵A离散化，因为车辆速度发生了变化
//    UpdateMatrix(localization);

            // cout << "matrix_bd_.row(): " << matrix_bd_.rows() << endl;
            // cout << "matrix_bd_.col(): " << matrix_bd_.cols() << endl;

            // TODO 05 Solve Lqr Problem
            SolveLQRProblem(matrix_ad_, matrix_bd_, matrix_q_, matrix_r_, lqr_eps_, lqr_max_iteration_, &matrix_k_);

            // TODO 06 计算feedback, 根据反馈对计算状态变量（误差状态）的时候的符号的理解：K里面的值实际运算中全部为正值，steer = -K * x
            // state，按照代码中采用的横向误差的计算方式，横向误差为正值的时候（state中的第一项为正），参考点位于车辆左侧，车辆应该向左转以减小误差
            // 而根据试验，仿真器中，给正值的时候，车辆向右转，给负值的时候，车辆向左转。
            //   Convert vehicle steer angle from rad to degree and then to steer degrees
            //   then to 100% ratio
            std::cout << "matrix_k_: " << matrix_k_ << std::endl;
            Matrix matrix_u = - matrix_k_ * matrix_state_;
            double steer_angle_feedback = 0;
            steer_angle_feedback = matrix_u(0, 0);

            // TODO 07 计算前馈控制，计算横向转角的反馈量
            double steer_angle_feedforward = 0.0;
            steer_angle_feedforward = ComputeFeedForward(localization, ref_curv_);
            double steer_angle = steer_angle_feedback + steer_angle_feedforward;

            // 限制前轮最大转角，这里定义前轮最大转角位于 [-20度～20度]
            if (steer_angle >= atan2_to_PI(20.0)) {
                steer_angle = atan2_to_PI(20.0);
            } else if (steer_angle <= -atan2_to_PI(20.0)) {
                steer_angle = -atan2_to_PI(20.0);
            }
            // Set the steer commands
            std::cout << "反馈转向角: " << steer_angle_feedback * 180 / M_PI;
            std::cout << "，前馈转向角: " << steer_angle_feedforward * 180 / M_PI;
            std::cout << "，转向角: " << steer_angle * 180 / M_PI << std::endl;
            cmd.steer_target = steer_angle;

            return true;
        }

// 计算横向误差并且更新状态向量x
        void LqrController::UpdateState(const VehicleState &vehicle_state) {
            // LateralControlError lat_con_err;  // 将其更改为智能指针
            std::shared_ptr<LateralControlError> lat_con_err = std::make_shared<LateralControlError>();

            // 计算横向误差
            ComputeLateralErrors(vehicle_state.x,
                                 vehicle_state.y,
                                 vehicle_state.heading, //航向角
                                 vehicle_state.velocity, //速度
                                 vehicle_state.angular_velocity, //平面角速度(绕z轴转动的角速度)
//                         vehicle_state.acceleration, //加速度
                                 lat_con_err);

            // State matrix update;
            // 更新状态x
            matrix_state_(0, 0) = lat_con_err->lateral_error;
            matrix_state_(1, 0) = lat_con_err->lateral_error_rate;
            matrix_state_(2, 0) = lat_con_err->heading_error;
            matrix_state_(3, 0) = lat_con_err->heading_error_rate;

            cout << "状态误差: " << matrix_state_.transpose() << endl;
        }

// TODO 04 更新状态矩阵A并将状态矩阵A离散化
//void LqrController::UpdateMatrix(const VehicleState &vehicle_state) {}

// TODO 07 前馈控制，计算横向转角的反馈量
        double LqrController::ComputeFeedForward(const VehicleState &localization, double ref_curvature) {
            if(isnan(ref_curvature)){
                ref_curvature = 0;
            }
            const double v = localization.velocity; //假设车辆无侧滑，vy约等于0，v约等于vx
            //这里前馈控制量要乘以-1，因为carla左转是负数控制
            double feed_forward_control = -1 * (ref_curvature * (wheelbase_ - lr_ * matrix_k_(0, 2) -
                                                                 mass_ * v * v * (- lr_ / cf_ + lf_ / cr_ - lf_ * matrix_k_(0, 2) / cr_ )/ wheelbase_));

            //乌宁视频的公式，第二项正负号相反，分母乘了2倍（但其实侧偏刚度已经算了左右两轮之和）
//    const double kv = lr_ * mass_ / (2 * cf_ * wheelbase_) - lf_ * mass_ / (2 * cr_ * wheelbase_);
//    double feed_forward_control = wheelbase_ * ref_curvature + kv * v * v * ref_curvature
//            - matrix_k_(0, 2) * (lr_ * ref_curvature - lf_ * mass_ * v * v * ref_curvature / (2 * cr_ * wheelbase_));
//    std::cout << "feed_forward_control: " << feed_forward_control << std::endl;

            return feed_forward_control;
        }

// TODO 03 计算误差，赋值给lat_con_err，有4个变量表示横向误差和转角误差，lat_con_err = [ecg, ecg', etheta, etheta']
        void LqrController::ComputeLateralErrors(const double x,
                                                 const double y,
                                                 const double theta,
                                                 const double linear_v,
                                                 const double angular_v,
//                                         const double linear_a,
                                                 LateralControlErrorPtr &lat_con_err) {
            //获取最近的轨迹点
            TrajectoryPoint trajeoryPoint = QueryNearestPointByPosition(x, y);

            /*
             * 正负号的说明：在carla里，steel为负数时，车辆向左转，此时控制量为负数，误差为正数
             */
            //横向距离误差，车辆方向向量a = (cos(theta), sin(theta))，车辆到轨迹点向量b = (xr-x, yr-y)
            //此时车辆到轨迹点的距离为ecg = a x b（计算按照行列式，方向按照右手定则，车辆方向在轨迹右侧则为正，此时车辆应该向左转向）
            lat_con_err->lateral_error = cos(theta) * (trajeoryPoint.y - y) - sin(theta) * (trajeoryPoint.x - x);

            //横摆角误差，假如x,y和xr,yr重合，θr大于θ，则车辆应该向左转向，此时误差为正，归一化的原因是两个角度相减范围是-2pai～2pai
            double e_theta = NormalizeAngle(trajeoryPoint.heading - theta);
            lat_con_err->heading_error = e_theta;

            //横向距离误差变化率 = vy + vx*tan(eθ)，在车辆无侧滑的假设下，vy = 0， vx = v
            // 这里为什么是sin(e_theta)，这个是横摆角误差，而不是航向角误差？
            lat_con_err->lateral_error_rate = linear_v * sin(e_theta);

            //横摆角误差变化率 = 车辆质心的转动角速度 - 轨迹点的转动角速度？（速度/半径 = 速度 * 曲率）
            lat_con_err->heading_error_rate = trajeoryPoint.v * trajeoryPoint.kappa - angular_v;
        }

// 查询距离当前位置最近的轨迹点
        TrajectoryPoint LqrController::QueryNearestPointByPosition(const double x, const double y) {
            double d_min = PointDistanceSquare(trajectory_points_.front(), x, y);
            size_t index_min = 0;

            for (size_t i = 1; i < trajectory_points_.size(); ++i) {
                double d_temp = PointDistanceSquare(trajectory_points_[i], x, y);
                if (d_temp < d_min) {
                    d_min = d_temp;
                    index_min = i;
                }
            }
            ref_curv_ = trajectory_points_[index_min].kappa;    // 对应的最近的轨迹点上的曲率

            double front_index = index_min + 5;
            if (front_index >= trajectory_points_.size()) {
                ref_curv_front_ = trajectory_points_[index_min].kappa;
            } else {
                ref_curv_front_ = trajectory_points_[front_index].kappa;
            }

            return trajectory_points_[index_min];
        }

// TODO 05:求解LQR方程
        void LqrController::SolveLQRProblem(
                const Matrix &A,
                const Matrix &B,
                const Matrix &Q,
                const Matrix &R,
                const double tolerance,
                const uint max_num_iteration,
                Matrix *ptr_K) {
            // 防止矩阵的维数出错导致后续的运算失败
            if (A.rows() != A.cols() || B.rows() != A.rows() || Q.rows() != Q.cols() || Q.rows() != A.rows() ||
                R.rows() != R.cols() || R.rows() != B.cols()) {
                std::cout << "LQR solver: one or more matrices have incompatible dimensions." << std::endl;
                return;
            }
            //迭代次数不超过给定值，或者最终控制的误差不超过给定阈值，则停止迭代求解
            Matrix P = Q; //P初始化为Q
            uint i = 0;
            for(i = 0; i <= max_num_iteration; i++){
                //迭代求解矩阵P，将Riccati方程右侧结果代入P_next，迭代求解P，为什么这里会慢慢收敛至一个稳定值？
                Matrix P_next = Q + A.transpose() * P * A -
                                A.transpose() * P * B * (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;
                double diff = fabs((P_next - P).maxCoeff());
                P = P_next;
                if(diff < tolerance){
                    //满足条件，求得K
                    cout << "lqr求解P矩阵diff = " << diff << "，迭代次数：" << i << endl;
                    *ptr_K = (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;
                    break;
                }
            }
            if(i > max_num_iteration){
                cout << "fail to solve lqr, after " << max_num_iteration << " times" << endl;
            }
        }

    }    // namespace control
}    // namespace shenlan
