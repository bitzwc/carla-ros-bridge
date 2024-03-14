#include "carla_shenlan_stanley_pid_controller/stanely_controller.h"
#include "carla_shenlan_stanley_pid_controller/pid_controller.h"

#include <algorithm>
#include <iomanip>
#include <utility>
#include <vector>

#include "Eigen/LU"
#include <math.h>

using namespace std;

//stanley算法的实现
namespace shenlan {
    namespace control {

        //PID控制器，用于导航角误差的控制
        shenlan::control::PIDController e_theta_pid_controller(1.0, 0.0, 0.4); // PID控制器中的微分环节相当于阻尼，加在航向误差引起的前轮转角上

        //角度转弧度
        double atan2_to_PI(const double atan2) 
        {
            return atan2 * M_PI / 180;
        }

        double PointDistanceSquare(const TrajectoryPoint &point, const double x, const double y) 
        {
            const double dx = point.x - x;
            const double dy = point.y - y;
            return dx * dx + dy * dy;
        }

        void StanleyController::LoadControlConf() 
        {
            //增益系数
            k_y_ = 0.5;

            //低速平滑系数，避免低速变化对角度产生太大影响
            k_v_ = 5;
        }

        // /** to-do **/ 计算需要的控制命令, 实现对应的stanley模型,并将获得的控制命令传递给汽车
        // 提示，在该函数中你需要调用计算误差
        // 控制器中，前轮转角的命令是弧度单位，发送给carla的横向控制指令，范围是 -1~1
        void StanleyController::ComputeControlCmd(const VehicleState &vehicle_state, const TrajectoryData &planning_published_trajectory, ControlCmd &cmd) 
        {  
            //第一次执行时，没有加这句，没有把启动入口读取到的轨迹赋值给stanley对象，导致直接退出
            this->trajectory_points_ = planning_published_trajectory.trajectory_points;
            double x = vehicle_state.x;
            double y = vehicle_state.y;
            double v = vehicle_state.v;
            double theta = vehicle_state.heading; //车辆的方向和X轴夹角
            double e_y;
            double e_theta;
            //调用误差计算函数
            ComputeLateralErrors(x, y, theta, e_y, e_theta);
            //δ(t) = PD[θe(t)] + δe(t) = θe(t) + tan−1(k * e(t)/(v(t) + ks)) + δff(k)
            //导航角误差和横向误差，转化成控制量
            //1、航向角误差的PID控制，针对高速时控制量的大幅度变化做平滑
            double e_theta_control = e_theta_pid_controller.Control(e_theta, 0.01);
            
            //由于减完后的角度误差在(-2PI, 2PI)，这里需要做一下角度转换(-PI, PI)，认为向右转3/2PI等于向左转1/2PI，即-1/2PI
            if(e_theta_control > M_PI){
                e_theta_control = e_theta_control - 2*M_PI;
            }else if(e_theta_control < - M_PI){
                e_theta_control = e_theta_control + 2*M_PI;
            }

            //2、横向误差的控制，atan2在分母为正数时范围在(-1/2PI,1/2PI)
            double e_y_control = std::atan2(k_y_ * e_y, v + k_v_);

            //3、曲率的前置反馈，大小等于曲率，方向取决车辆方向和法线方向的叉乘
            TrajectoryPoint path_point = QueryNearestPointByPosition(x, y);
            double k = path_point.kappa;
            double e_k_control = -2*k; //这个参数根据实际情况调节

            double lateral_control = e_theta_control + e_y_control + e_k_control;
            cout << "车辆方向:" << 180*theta/M_PI << "°,最近轨迹点切线方向:" << 180*theta_ref_/M_PI << "°,曲率：" << k << ",车辆速度:" << v;

            //输出转向控制量，限定范围
            if(lateral_control <= -1){
                lateral_control = -1;
            }
            if(lateral_control >= 1){
                lateral_control = 1;
            }
            cout << ",总横向控制:" << lateral_control << ",航向角误差控制:" << e_theta_control << ",横向误差控制:" << e_y_control << endl;

            cmd.steer_target = lateral_control;
        }

        // /** to-do **/ 计算需要的误差，包括横向误差，朝向误差，误差计算函数没有传入主车速度，因此返回的位置误差就是误差，不是根据误差计算得到的前轮转角
        void StanleyController::ComputeLateralErrors(const double x, const double y, const double theta, double &e_y, double &e_theta) 
        {
            //找到当前车辆位置和轨迹最近的点，到这个点的距离就是横向误差，这里可以直接用轨迹点的切线单位向量和轨迹点到车辆的向量的叉乘，因为这两个向量是垂直的，大小就等于轨迹点到车辆的欧式距离，且方向符合右手定则
            //当车辆向左打方向时，认为角度是负的
            TrajectoryPoint path_point = QueryNearestPointByPosition(x, y);

            //这里需要有正负的区分，用的向量叉乘计算 z = x1y2-x2y1，叉乘为正时，表示要向左转弯，这里增加负号
            e_y = -(std::cos(theta) * (path_point.y - y) - std::sin(theta) * (path_point.x - x));

            //找到当前车辆位置和轨迹最近的点，获取切线方向的角度，减去车辆的方向角度，就是导航角误差
            //第一次执行时，这里方向反了，当轨迹切线角度比车辆角度大时，应该左转，此时角度误差是负的
            //减完后的误差项范围在(-2PI, 2PI)
            e_theta = theta - theta_ref_;
        }

        // 返回参考路径上和车辆当前位置距离最近的点，返回的是点结构体
        TrajectoryPoint StanleyController::QueryNearestPointByPosition(const double x, const double y) 
        {
            double d_min = PointDistanceSquare(trajectory_points_.front(), x, y); // 得到当前位置距离参考路径的第一个点的距离
            size_t index_min = 0;

            for (size_t i = 1; i < trajectory_points_.size(); ++i) 
            {
                double d_temp = PointDistanceSquare(trajectory_points_[i], x, y);
                if (d_temp < d_min) 
                {
                    d_min = d_temp;
                    index_min = i;
                }
            }
            // cout << " index_min: " << index_min << endl;
            //cout << "tarjectory.heading: " << trajectory_points_[index_min].heading << endl;
            theta_ref_ = trajectory_points_[index_min].heading; // 获得距离车辆当前位置最近的路径点的角度

            return trajectory_points_[index_min];
        }
    }  // namespace control
}  // namespace shenlan