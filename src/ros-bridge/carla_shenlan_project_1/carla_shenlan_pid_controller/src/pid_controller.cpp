#include "carla_shenlan_pid_controller/pid_controller.h" //在include文件夹里

#include <assert.h>
#include <iostream>
namespace shenlan {
    namespace control {

        //继承父类PIDController，构造函数
        PIDController::PIDController(const double kp, const double ki, const double kd) {
            kp_ = kp;
            ki_ = ki;
            kd_ = kd;
            previous_error_ = 0.0;
            previous_output_ = 0.0;
            integral_ = 0.0;
            first_hit_ = true; //首次处理
        }

        // /**to-do**/ 实现PID控制
        // 输入是误差反馈量error和控制周期dt，输出是当前的控制量
        double PIDController::Control(const double error, const double dt) {
            //时间增量dt非法时，直接返回前一个输出结果
            if(dt <= 0){
                assert(dt > 0 && "dt must be positive!!!");
                return previous_output_;
                // ROS_ERROR("");
            }
            integral_ += error;
            current_output = kp_ * error + ki_ * integral_;

            // 注意微分项的计算需要分情况，i=0时，可以使用“first_hit_”做判断
            if(!first_hit_){
                current_output += kd_ * (error - previous_error_) / dt;
            }
            first_hit_ = false;
            previous_error_ = error;
            previous_output_ = current_output;
            if(integral_ >= 200 || integral_ <= -200){
                Reset();
            }
            return current_output;
        }

        // /**to-do**/ 重置PID参数
        void PIDController::Reset() {
            // previous_error_ = 0.0;
            // previous_output_ = 0.0;
            integral_ = 0.0;
            // first_hit_ = true;
        }
    }    // namespace control
}    // namespace shenlan
