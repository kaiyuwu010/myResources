#include <Eigen/Dense>
#include <iostream>
#include "../dependency/rm-arm/include/rm_base.h"
#include "../dependency/yaml/include/yaml.h"
#include <ros/ros.h>
#include <ros/package.h>
#include "forwardKinematics.cpp"
#include <chrono>
#include <thread>
#include <nlohmann/json.hpp>
#include <cstring>       // for memset
#include <cstdlib>       // for exit
#include <unistd.h>      // for close()
#include <arpa/inet.h>
#include <csignal>
#include <memory>

class hybridPositionForceControl
{
public:
    //质量、阻尼和刚度参数
    Eigen::Matrix<float, 3, 3> M_, D_, K_;
    Eigen::Matrix<float, 3, 3> inverse_of_M_;
    Eigen::Matrix<float, 3, 1> error_;
    //机械臂当前位置、速度和受力
    Eigen::Matrix<float, 3, 1> arm_position_;
    Eigen::Matrix<float, 3, 1> arm_twist_;           
    Eigen::Matrix<float, 3, 1> force_external_;  // N/Nm
    //根据当前状态计算出的加速度、速度和下一步期望的位置          
    Eigen::Matrix<float, 3, 1> force_control_desired_acc_;    
    Eigen::Matrix<float, 3, 1> force_control_desired_twist_;     
    Eigen::Matrix<float, 3, 1> force_control_desired_position_;             
    //期望的位置和相对上次的增量
    Eigen::Matrix<float, 3, 1> cmd_pose_position_;
    Eigen::Matrix<float, 3, 1> cmd_pose_increment_;
    //目标位置相对当前位置的向量
    Eigen::Matrix<float, 3, 1> diff_vector_;
    float diff_length_;
    //频率
    const float rate_ = 100;  // Hz
    //阈值, 每秒钟限定最大位移为1000mm
    const float max_displacement_ = 1000/rate_;  
    //首次更新位置标志位
    bool first_update_flag_ = true;
public:
    hybridPositionForceControl()
    {
        std::cout<<"力控模块: rate_: "<<rate_<<" max_displacement_: "<<max_displacement_<<std::endl;
        // M_<< 1,0,0,
        //      0,1,0,
        //      0,0,1;
        M_<< 0.3,0,0,
             0,0.3,0,
             0,0,0.3;
        std::cout<<"力控模块: M: "<<std::endl<<M_<<std::endl;
        inverse_of_M_ = M_.inverse();
        //速度为0.5m/s时，造成的阻力为5N
        std::cout<<"力控模块: inverse_of_M: "<<std::endl<<inverse_of_M_<<std::endl;
        D_<< 5,0,0,
             0,5,0,
             0,0,5;
        std::cout<<"力控模块: D: "<<std::endl<<D_<<std::endl;
        //误差为0.01m时，乘上K造成的加速度为100m/s^2，一个周期0.01s可以额外增加的位移为1/2*a*t^2, 也就是0.005m
        //误差为0.01m时，造成10N的力
        K_<< 200,0,0,
             0,200,0,
             0,0,200;
        std::cout<<"力控模块: K: "<<std::endl<<K_<<std::endl;
        force_control_desired_twist_<< 0, 0, 0;
        std::cout<<"力控模块: force_control_desired_twist_: "<<std::endl<<force_control_desired_twist_<<std::endl;
    }
    void computeAdmittance() 
    {
        //计算位置误差
        error_ = arm_position_ - cmd_pose_position_;
        //由动力学方程 F = Mx¨+ Dx˙+ Kx 计算出目标加速度
        force_control_desired_acc_ =  inverse_of_M_ * (force_external_ - (D_ * (force_control_desired_twist_) + K_*error_));
        std::cout<<"力控模块: force_control_desired_twist_: "<<std::endl<<force_control_desired_twist_<<std::endl;
        std::cout<<"力控模块: arm_twist_: "<<std::endl<<arm_twist_<<std::endl;
        std::cout<<"力控模块: force_control_desired_acc_为: "<<std::endl;
        std::cout<<force_control_desired_acc_<<std::endl;
        //判断加速度是否超过阈值
        // float a_acc_norm = force_control_desired_acc_.norm();
        // if (a_acc_norm > arm_max_acc_) 
        // {
        //     std::cout << "力控模块: 加速度: "<< a_acc_norm<<" 超过阈值!" << std::endl;
        //     force_control_desired_acc_ *= (arm_max_acc_ / a_acc_norm);
        // }
        //计算力控部分位置控制目标
        force_control_desired_twist_ += force_control_desired_acc_ / rate_;
        // force_control_desired_twist_ = arm_twist_ + force_control_desired_acc_ / rate_;
        std::cout<<"力控模块: force_control_desired_twist_为: \n"<<force_control_desired_twist_<<std::endl;
        // //更新目标位置与当前位置差值
        // diff_vector_ = arm_position_ - cmd_pose_position_;
        // diff_length_ = diff_vector_.norm();
        // if(diff_length_ < max_displacement_)
        // {
        //     force_control_desired_position_ = cmd_pose_position_;
        //     std::cout<<"力控模块: 当前位置加上目标位置的影响后的位置: \n"<<force_control_desired_position_<<std::endl;
        // }
        // else
        // {
        //     force_control_desired_position_ = arm_position_ + diff_vector_ * max_displacement_ / diff_length_;
        //     std::cout<<"力控模块: 当前位置加上目标位置的影响后的位置: \n"<<force_control_desired_position_<<std::endl;
        // }
        // force_control_desired_position_ += force_control_desired_twist_/ rate_;
        force_control_desired_position_ = arm_position_ + force_control_desired_twist_/ rate_;
        std::cout<<"力控模块: force_control_desired_position_为: \n"<<force_control_desired_position_<<std::endl;
    }
    //更新实时的位置、力和目标位置(位置和力实时性越强，控制越精准)
    void updateData(Eigen::Matrix<float, 3, 1>& position,
                    Eigen::Matrix<float, 3, 1>& cmd_position,
                    Eigen::Matrix<float, 3, 1>& force)
    {
        if(first_update_flag_)
        {
            std::cout<<"力控模块: 第一次更新arm_position_和cmd_pose_position_!"<<std::endl;
            arm_position_ = position;
            cmd_pose_position_ = position;
            first_update_flag_ = false;
        }
        //先由位置更新速度
        arm_twist_ = (position - arm_position_)*rate_;
        std::cout<<"力控模块: 更新后的速度为: "<<std::endl;
        std::cout<<arm_twist_<<std::endl;
        //更新位置
        arm_position_ = position;
        std::cout<<"力控模块: 更新后的当前位置为: "<<std::endl;
        std::cout<<arm_position_<<std::endl;
        //更新控制目标位置
        cmd_pose_position_ = cmd_position;
        std::cout<<"力控模块: 更新后的指令位置为: "<<std::endl;
        std::cout<<cmd_pose_position_<<std::endl;
        //更新力
        force_external_ = force;
        std::cout<<"力控模块: 更新后的力为: "<<std::endl;
        std::cout<<force_external_<<std::endl;
    }
};

void MCallback(CallbackData data)
{
    // 判断接口类型
    switch(data.errCode)
    {
        case MOVEJ_CANFD_CB: // 角度透传
            printf("MOVEJ_CANFD 透传结果: %d\r\n", data.errCode);
        break;
        case MOVEP_CANFD_CB: // 位姿透传
            printf("MOVEP_CANFD  透传结果: %d\r\n", data.errCode);
        break;
        case FORCE_POSITION_MOVE_CB: // 力位混合透传
            printf("FORCE_POSITION_MOVE  透传结果: %d\r\n", data.errCode);
        break;
    }
}
class ReceiveDataByUDP 
{
private:
    int server_fd;
    struct sockaddr_in server_addr;
    struct sockaddr_in client_addr;
    socklen_t len = sizeof(client_addr);
    char buffer[1024];
    double interval = 0;
    std::chrono::_V2::system_clock::time_point last_update_time;
public:
    //关节角互斥锁
    std::mutex realtime_arm_data_mutex_;
    nlohmann::json jsonData_;
    //通过UDP获取的主要参数
    float udp_euclid_position_[3];
    float udp_euclid_euler_[3];
    float udp_joint_position_[6];
    float udp_external_force_[6];
public:
    ReceiveDataByUDP(const std::string& address, int port) 
    {
        server_fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (server_fd < 0) {
            std::cerr << "Failed to create socket" << std::endl;
            exit(EXIT_FAILURE);
        }
        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = INADDR_ANY;
        server_addr.sin_port = htons(port);
        if (bind(server_fd, (const struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            std::cerr << "Bind failed" << std::endl;
            close(server_fd);
            exit(EXIT_FAILURE);
        }
        std::thread updateData(&ReceiveDataByUDP::receiveMessages, this);
        updateData.detach();
    }

    ~ReceiveDataByUDP() {
        close(server_fd);
    }
    // void receiveMessages() 
    // {
    //     auto time_start = std::chrono::high_resolution_clock::now();
    //     int n = recvfrom(server_fd, buffer, sizeof(buffer), 0, (struct sockaddr*)&client_addr, &len);
    //     auto time_end = std::chrono::high_resolution_clock::now();
    //     if (n < 0) {
    //         std::cerr << "Receive failed" << std::endl;
    //         exit(EXIT_FAILURE);
    //     }
    //     buffer[n] = '\0';
    //     // std::cout << "!!!!!!!!!!!!!!!Received message: " << buffer << std::endl;
    //     // 使用互斥锁保护全局变量
    //     jsonData_ = nlohmann::json::parse(buffer);
    //     auto joint_position = jsonData_["joint_status"]["joint_position"];
    //     auto external_force = jsonData_["six_force_sensor"]["zero_force"];
    //     auto euclid_euler = jsonData_["waypoint"]["euler"];
    //     auto euclid_position = jsonData_["waypoint"]["position"];
    //     for (size_t i=0; i<6; i++) 
    //     {
    //         udp_joint_position_[i] = static_cast<float>(joint_position[i]) / 1000;
    //         udp_external_force_[i] = static_cast<float>(external_force[i]) / 1000;
    //         // std::cout << udp_joint_position_[i] << "--" <<udp_external_force_[i]<<", ";
    //     }
    //     // std::cout << std::endl;
    //     // std::cout << "ReceiveDataByUDP模块: udp_euclid_position_ , udp_euclid_euler_: \n";
    //     for (size_t i=0; i<3; i++) 
    //     {
    //         udp_euclid_position_[i] = static_cast<float>(euclid_position[i]) / 1000000;
    //         udp_euclid_euler_[i] = static_cast<float>(euclid_euler[i]) / 1000;
    //         // std::cout << udp_euclid_position_[i] << "--" <<udp_euclid_euler_[i]<<", ";
    //     }
    //     // std::cout << std::endl;
    //     auto time_now = std::chrono::high_resolution_clock::now();
    //     double duration = std::chrono::duration_cast<std::chrono::nanoseconds>(time_now - last_update_time).count();
    //     double duration1 = std::chrono::duration_cast<std::chrono::nanoseconds>(time_end - time_start).count();
    //     last_update_time = time_now;
    //     // std::cout<<"数据获取时间间隔: "<<duration * 1e-9<<"   数据获取时间: "<<duration1 * 1e-9<<std::endl;
    // }
    void receiveMessages() 
    {
        while (true) 
        {
            auto time_start = std::chrono::high_resolution_clock::now();
            int n = recvfrom(server_fd, buffer, sizeof(buffer), 0, (struct sockaddr*)&client_addr, &len);
            auto time_end = std::chrono::high_resolution_clock::now();
            if (n < 0) {
                std::cerr << "Receive failed" << std::endl;
                exit(EXIT_FAILURE);
            }
            buffer[n] = '\0';
            // std::cout << "!!!!!!!!!!!!!!!Received message: " << buffer << std::endl;
            // 使用互斥锁保护全局变量
            std::lock_guard<std::mutex> lock(realtime_arm_data_mutex_);
            jsonData_ = nlohmann::json::parse(buffer);
            auto joint_position = jsonData_["joint_status"]["joint_position"];
            auto external_force = jsonData_["six_force_sensor"]["zero_force"];
            auto euclid_euler = jsonData_["waypoint"]["euler"];
            auto euclid_position = jsonData_["waypoint"]["position"];
            for (size_t i=0; i<6; i++) 
            {
                udp_joint_position_[i] = static_cast<float>(joint_position[i]) / 1000;
                udp_external_force_[i] = static_cast<float>(external_force[i]) / 1000;
                // std::cout << udp_joint_position_[i] << "--" <<udp_external_force_[i]<<", ";
            }
            // std::cout << std::endl;
            // std::cout << "ReceiveDataByUDP模块: udp_euclid_position_ , udp_euclid_euler_: \n";
            for (size_t i=0; i<3; i++) 
            {
                udp_euclid_position_[i] = static_cast<float>(euclid_position[i]) / 1000000;
                udp_euclid_euler_[i] = static_cast<float>(euclid_euler[i]) / 1000;
                // std::cout << udp_euclid_position_[i] << "--" <<udp_euclid_euler_[i]<<", ";
            }
            // std::cout << std::endl;
            auto time_now = std::chrono::high_resolution_clock::now();
            double duration = std::chrono::duration_cast<std::chrono::nanoseconds>(time_now - last_update_time).count();
            double duration1 = std::chrono::duration_cast<std::chrono::nanoseconds>(time_end - time_start).count();
            last_update_time = time_now;
            std::cout<<"数据获取时间间隔: "<<duration * 1e-9<<"   数据获取时间: "<<duration1 * 1e-9<<std::endl;
        }
        close(server_fd);
    }
};


int main(int argc, char** argv)
{
    // UDP线程获取机械臂实时状态，该端口号会被Arm_Socket_Start函数占用，故先运行ReceiveDataByUDP
    ReceiveDataByUDP getUDPData("192.168.1.100", 8099);
    // 连接, 初始化API, 注册回调函数
    RM_API_Init(ARM_65, MCallback);
    SOCKHANDLE m_sockhand = -1;
    m_sockhand =  Arm_Socket_Start((char*)"192.168.1.18", 8080, 5000);
    // Set_Realtime_Push(m_sockhand, 1, 8099, true, 1, "192.168.1.100");
    for(size_t j=0; j<6; j++)
    {
        Set_Joint_Err_Clear(m_sockhand, j, 1);
    }
    //初始化力位混合控制
    hybridPositionForceControl hpfc;
    float joint[6];
    float force[6];
    float zero_force[6];
    float work_zero[6];
    float tool_zero[6];
    int ret = -1;
    //控制频率
    int rate = 0;
    //轨迹参数
    size_t times = 400;
    //开始点
    float start_position[3] = {-0.20, 0.20, 0.30};
    float start_joint[6] = {-45, 16.619, 77.012, -0.015, 86.351, -44.989};
    //结束点
    float end_position[3] = {-0.20, -0.20, 0.30};
    float end_joint[6] = {45, 16.641, 77.012, -0.015, 86.391, 45.0};
    //轨迹
    std::vector<float> joint_incre;
    for(size_t i=0; i<6; i++)
    {
        joint_incre.push_back((end_joint[i] - start_joint[i])/times);
    } 
    std::cout<<"位置增量为: ";
    for(size_t i=0; i<6; i++)
    {
        std::cout<<", "<<joint_incre[i];
    }   
    std::cout<<std::endl;
    float joint_trajectory[times+1][6];
    for(size_t i=0; i<times+1; i++)
    {
        for(size_t j=0; j<6; j++)
        {
            joint_trajectory[i][j] = start_joint[j] + joint_incre[j] * i;
        }
    }    
    std::cout<<"轨迹值为: "<<std::endl;
    for(size_t i=0; i<times+1; i++)
    {
        for(size_t j=0; j<6; j++)
        {
            std::cout<<", "<<joint_trajectory[i][j];
        }
        std::cout<<std::endl;
    }
    //计算对应的欧氏空间轨迹
    float euclid_trajectory[times+1][6];
    for(size_t i=0; i<times+1; i++)
    {
        Pose q_pose;
        q_pose = Algo_Forward_Kinematics(joint_trajectory[i]);
        euclid_trajectory[i][0] = q_pose.position.x;
        euclid_trajectory[i][1] = q_pose.position.y;
        euclid_trajectory[i][2] = q_pose.position.z;
        euclid_trajectory[i][3] = q_pose.euler.rx;
        euclid_trajectory[i][4] = q_pose.euler.ry;
        euclid_trajectory[i][5] = q_pose.euler.rz;
    }
    std::cout<<"欧氏空间轨迹值为: "<<std::endl;
    for(size_t i=0; i<times+1; i++)
    {
        for(size_t j=0; j<6; j++)
        {
            std::cout<<", "<<euclid_trajectory[i][j];
        }
        std::cout<<std::endl;
    }
    const std::chrono::milliseconds interval(int(1000/hpfc.rate_));
    ret = Movej_Cmd(m_sockhand, start_joint, 20, 0, 0, 1);
    if(ret != 0)
    {
        std::cout<<"回到初始位置失败"<<std::endl;
        return 0;
    }
    int num = 0;
    bool reverse_flag = false;
    float last_joints[6];
    while(true)
    {
        auto time_start = std::chrono::high_resolution_clock::now();
        auto trajectory_start = std::chrono::steady_clock::now();
        if(reverse_flag)
        {
            num--;
        }
        else
        {
            num++;
        }
        if(num>399)
        {
            reverse_flag = true;
        }
        else if(num<1)
        {
            reverse_flag = false;
        }
        std::this_thread::sleep_until(trajectory_start + interval);
        // getUDPData.receiveMessages();
        // float current_joint[6];
        // ret = Get_Joint_Degree(m_sockhand, current_joint);
        // Pose current_pose;
        // current_pose = Algo_Forward_Kinematics(current_joint);
        // if(ret!=0)
        // {
        //     return 0;
        // }
        // Eigen::Matrix<float, 3, 1> position = {current_pose.position.x, current_pose.position.y, current_pose.position.z};
        Eigen::Matrix<float, 3, 1> position = {getUDPData.udp_euclid_position_[0], getUDPData.udp_euclid_position_[1], getUDPData.udp_euclid_position_[2]};
        //计算力控结果
        Eigen::Matrix<float, 3, 1> cmd_position = {euclid_trajectory[num][0], euclid_trajectory[num][1], euclid_trajectory[num][2]};
        Eigen::Matrix<float, 3, 1> xyz_force = {getUDPData.udp_external_force_[0], getUDPData.udp_external_force_[1], getUDPData.udp_external_force_[2]};
        hpfc.updateData(position, cmd_position, xyz_force);
        // std::cout <<"udp_euclid_position_: "<< std::endl;
        // for (size_t i=0; i<3; i++) 
        // {
        //     std::cout << getUDPData.udp_euclid_position_[i]<<", ";
        // }
        // std::cout << std::endl;
        // std::cout <<"udp_euclid_euler_: "<< std::endl;
        // for (size_t i=0; i<3; i++) 
        // {
        //     std::cout << getUDPData.udp_euclid_euler_[i]<<", ";
        // }
        // std::cout << std::endl;
        // std::cout <<"udp_external_force_: "<< std::endl;
        // for (size_t i=0; i<3; i++) 
        // {
        //     std::cout << getUDPData.udp_external_force_[i]<<", ";
        // }
        // std::cout << std::endl;
        hpfc.computeAdmittance();
        Pose q_pose;
        q_pose.position.x = hpfc.force_control_desired_position_[0];
        q_pose.position.y = hpfc.force_control_desired_position_[1];
        q_pose.position.z = hpfc.force_control_desired_position_[2];
        q_pose.euler.rx = euclid_trajectory[num][3];
        q_pose.euler.ry = euclid_trajectory[num][4];
        q_pose.euler.rz = euclid_trajectory[num][5];
        float q_out[6];
        for(size_t j=0; j<6; j++)
        {
            last_joints[j] = getUDPData.udp_joint_position_[j];
        }
        ret = Algo_Inverse_Kinematics(last_joints, &q_pose, q_out, 1);
        if(ret != 0)
        {
            std::cout<<"关节逆运动学解算失败, xyz为"<<q_pose.position.x<<" "<<q_pose.position.y<<" "<<q_pose.position.z<<std::endl;
            std::cout<<"关节逆运动学解算失败, rxryrz为"<<q_pose.euler.rx<<" "<<q_pose.euler.ry<<" "<<q_pose.euler.rz<<std::endl;
            return 0;
        }
        std::cout<<"关节控制指令: "<<std::endl;
        for(size_t j=0; j<6; j++)
        {
            std::cout<<", "<<q_out[j];
        }
        std::cout<<std::endl;
        std::cout<<"当前关节位置: "<<std::endl;
        for(size_t j=0; j<6; j++)
        {
            std::cout<<", "<<getUDPData.udp_joint_position_[j];
        }
        std::cout<<std::endl;
        //控制机械臂运动
        ret = Movej_CANFD(m_sockhand, q_out, false, 1);
        if(ret != 0)
        {
            std::cout<<"透传控制机械部失败"<<std::endl;
            return 0;
        }
        std::cout<<"上一帧joint_trajectory数据: "<<std::endl;
        for(size_t j=0; j<6; j++)
        {
            std::cout<<", "<<joint_trajectory[num-1][j];
        }
        std::cout<<std::endl;
        std::cout<<"当前帧joint_trajectory数据: "<<std::endl;
        for(size_t j=0; j<6; j++)
        {
            std::cout<<", "<<joint_trajectory[num][j];
        }
        std::cout<<std::endl;
        std::cout<<"上一帧euclid_trajectory数据: "<<std::endl;
        for(size_t j=0; j<6; j++)
        {
            std::cout<<", "<<euclid_trajectory[num-1][j];
        }
        std::cout<<std::endl;
        std::cout<<"当前帧euclid_trajectory数据: "<<std::endl;
        for(size_t j=0; j<6; j++)
        {
            std::cout<<", "<<euclid_trajectory[num][j];
        }
        std::cout<<std::endl;
        auto time_end = std::chrono::high_resolution_clock::now();
        double duration = std::chrono::duration_cast<std::chrono::nanoseconds>(time_end - time_start).count();
        std::cout<<"控制周期时间: "<<duration * 1e-9<<"第"<<num<<"个控制周期"<<std::endl;
    }

    // auto end = std::chrono::high_resolution_clock::now();
    // double duration1 = std::chrono::duration_cast<std::chrono::nanoseconds>(time_end - time_start).count();
    // double duration2 = std::chrono::duration_cast<std::chrono::nanoseconds>(end - mid).count();
    // std::cout<<"运行次数: "<<rate<<std::endl;
    // std::cout<<"运行时间duration1: "<<duration1 * 1e-9<<std::endl;
    // std::cout<<"运行时间duration2: "<<duration2 * 1e-9<<std::endl;
}