#include <Eigen/Dense>
#include <iostream>
/*
如果是通过位置控制来实现导纳控制，需要先通过如下公式计算合力:
F = Mx¨+ Dx˙+ Kx => Mx¨ = F - (Dx˙+ Kx)
合力主要由三部分组成: 外力、阻力、弹力，阻力由速度决定，弹力由误差决定
主要问题: 在高频控制的情况下，每个控制周期机械臂的增量都很小，此时机械臂的响应误差和获取的位置误差与增量相比就会很大。

1.如果最终位置控制指令Xout, 由当前位置Xnow加上规划增量Xincre加上导纳控制增量Xadmi得到:
Xout = Xnow + Xincre + Xadmi
不考虑导纳控制部分的增量，由于当前位置Xnow有滞后会小于实际值，得到的Xout会小于规划位置，机械臂的实际轨迹落后于规划轨迹。
2.从整体上看机械臂实际轨迹落后于规划轨迹一定程度，可以由导纳控制增量保证落后不会太大。
假设力矩 F=2Nm，达到速度x˙=1rad/s，需要0.5s，由于D为2此时阻力完全抵消力矩，只有回到规划位置的弹力，弹力为5*Xerr
然后根据加速度计算出下一控制周期需要达到的位置:x˙*t + 0.5*X¨*t^2
*/
class hybridPositionForceControl
{
public:
    //质量、阻尼和刚度参数
    Eigen::Matrix<float, 6, 6> M_, D_, K_;
    Eigen::Matrix<float, 6, 6> inverse_of_M_;
    Eigen::Matrix<float, 6, 1> error_;
    //机械臂当前位置、姿态、速度和受力(N/Nm)
    Eigen::Matrix<float, 3, 1> arm_position_;
    Eigen::Quaternionf arm_orientation_;
    Eigen::Matrix<float, 3, 1> arm_twist_;
    Eigen::Quaternionf arm_orient_twist_;
    Eigen::Matrix<float, 6, 1> force_external_;
    //根据当前状态计算出的加速度、速度和下一步期望的位置
    Eigen::Matrix<float, 6, 1> force_control_desired_acc_;
    Eigen::Matrix<float, 6, 1> force_control_desired_twist_;
    Eigen::Matrix<float, 6, 1> force_control_desired_position_;
    //目标位置、姿态和增量
    Eigen::Matrix<float, 3, 1> cmd_position_;
    Eigen::Quaternionf cmd_orientation_;

public:
    hybridPositionForceControl()
    {
        M_ << 1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1;
        std::cout << "力控模块: M: " << std::endl
                  << M_ << std::endl;
        inverse_of_M_ = M_.inverse();
        //速度为0.5m/s时，造成的阻力为5N; 速度为0.5rad/s（约30度每秒）时，造成1Nm的力
        std::cout << "力控模块: inverse_of_M: " << std::endl
                  << inverse_of_M_ << std::endl;
        D_ << 15, 0, 0, 0, 0, 0,
            0, 15, 0, 0, 0, 0,
            0, 0, 15, 0, 0, 0,
            0, 0, 0, 2, 0, 0,
            0, 0, 0, 0, 2, 0,
            0, 0, 0, 0, 0, 2;
        std::cout << "力控模块: D: " << std::endl
                  << D_ << std::endl;
        //误差为0.1m时，造成10N的力; 误差为0.5rad（约30度）时造成2Nm的力
        K_ << 100, 0, 0, 0, 0, 0,
            0, 100, 0, 0, 0, 0,
            0, 0, 100, 0, 0, 0,
            0, 0, 0, 4, 0, 0,
            0, 0, 0, 0, 4, 0,
            0, 0, 0, 0, 0, 4;
        std::cout << "力控模块: K: " << std::endl
                  << K_ << std::endl;
        arm_twist_ << 0, 0, 0;
        arm_orient_twist_ = Eigen::Quaternionf::Identity();
        force_control_desired_twist_ << 0, 0, 0, 0, 0, 0;
        std::cout << "力控模块: force_control_desired_twist_: " << std::endl
                  << force_control_desired_twist_ << std::endl;
    }
    void computeAdmittance(float& time_interval)
    {
        //计算位置误差
        error_.topRows(3) = force_control_desired_position_.topRows(3) - cmd_position_;
        //计算姿态误差，先计算四元数，再转化为轴角，最后把轴角的角度乘到方向向量
        Eigen::Quaternionf current_orientation;
        current_orientation = Eigen::AngleAxisf(force_control_desired_position_[5], Eigen::Vector3f::UnitZ()) *
                              Eigen::AngleAxisf(force_control_desired_position_[4], Eigen::Vector3f::UnitY()) *
                              Eigen::AngleAxisf(force_control_desired_position_[3], Eigen::Vector3f::UnitX());
        Eigen::Quaternionf quat_arm_orientation(current_orientation * cmd_orientation_.inverse());
        Eigen::AngleAxisf arm_orientation(quat_arm_orientation);
        error_.bottomRows(3) << (arm_orientation.axis() * arm_orientation.angle());
        // std::cout << "力控模块: 误差: " << std::endl
        //           << error_ << std::endl;
        //由动力学方程 F = Mx¨+ Dx˙+ Kx 计算出目标加速度
        force_control_desired_acc_ = inverse_of_M_ * (force_external_ - (D_ * (force_control_desired_twist_) + K_ * error_));
        // std::cout << "力控模块: force_control_desired_twist_: \n"
        //           << force_control_desired_twist_ << std::endl;
        // std::cout << "力控模块: force_control_desired_acc_为: \n"
        //           << force_control_desired_acc_ << std::endl;
        //计算速度和位置控制量
        force_control_desired_twist_ += 0.5 * force_control_desired_acc_ * time_interval;
        force_control_desired_position_ += force_control_desired_twist_ * time_interval;
        // std::cout << "力控模块: force_control_desired_position_为: \n"
        //           << force_control_desired_position_ << std::endl;
    }
    //设置机械臂初始位姿, 单位m、rad
    void setInitialPose(Eigen::Matrix<float, 3, 1>& initial_position, Eigen::Matrix<float, 3, 1>& initial_xyz_euler)
    {
        force_control_desired_position_.topRows(3) = initial_position;
        force_control_desired_position_.bottomRows(3) = initial_xyz_euler;
        std::cout << "力控模块: 设置初始位姿为: " << std::endl;
        std::cout << force_control_desired_position_ << std::endl;
    }
    //更新实时的位置、力和目标位置, 单位m、rad
    void updateData(Eigen::Matrix<float, 3, 1>& cmd_position, Eigen::Quaternionf& cmd_orientation, Eigen::Matrix<float, 6, 1>& force)
    {
        //更新控制目标位置
        cmd_position_ = cmd_position;
        cmd_orientation_ = cmd_orientation;
        // std::cout << "力控模块: 更新后的目标位姿为: " << std::endl;
        // std::cout << cmd_position_ << std::endl;
        // std::cout << cmd_orientation_.coeffs() << std::endl;
        //更新力
        force_external_ = force;
        // std::cout << "力控模块: 更新后的力为: " << std::endl;
        // std::cout << force_external_ << std::endl;
    }
    //更新实时的位置、力和目标位置
    void updateData(Eigen::Matrix<float, 3, 1>& cmd_position, Eigen::Matrix<float, 3, 1>& cmd_orientation, Eigen::Matrix<float, 6, 1>& force)
    {
        //更新控制目标位置
        cmd_position_ = cmd_position;
        Eigen::Quaternionf orientation;
        orientation = Eigen::AngleAxisf(cmd_orientation[2], Eigen::Vector3f::UnitZ()) *
                      Eigen::AngleAxisf(cmd_orientation[1], Eigen::Vector3f::UnitY()) *
                      Eigen::AngleAxisf(cmd_orientation[0], Eigen::Vector3f::UnitX());
        cmd_orientation_ = orientation;
        // std::cout << "力控模块: 更新后的目标位姿为: " << std::endl;
        std::cout << cmd_position_ << std::endl;
        std::cout << cmd_orientation_.coeffs() << std::endl;
        //更新力
        force_external_ = force;
        // std::cout << "力控模块: 更新后的力为: " << std::endl;
        std::cout << force_external_ << std::endl;
    }
};