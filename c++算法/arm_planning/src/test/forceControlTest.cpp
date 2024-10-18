#include <arpa/inet.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <unistd.h>  // for close()

#include <Eigen/Dense>
#include <chrono>
#include <csignal>
#include <cstdlib>  // for exit
#include <cstring>  // for memset
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>
#include <thread>

#include "../../include/arm_planning/hybridPositionForceControl.hpp"
#include "../dependency/rm-arm/include/rm_base.h"
#include "../dependency/yaml/include/yaml.h"
#include "../include/arm_planning/getArmUdpData.hpp"

int main(int argc, char** argv)
{
    // UDP线程获取机械臂实时状态，该端口号会被Arm_Socket_Start函数占用，故先运行ReceiveDataByUDP
    ReceiveDataByUDP getUDPData("192.168.1.100", 8099);
    // 连接, 初始化API, 注册回调函数
    RM_API_Init(ARM_65, NULL);
    SOCKHANDLE m_sockhand = -1;
    m_sockhand = Arm_Socket_Start((char*)"192.168.1.18", 8080, 5000);
    // Set_Realtime_Push(m_sockhand, 1, 8099, true, 1, "192.168.1.100");
    for (size_t j = 0; j < 6; j++)
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
    for (size_t i = 0; i < 6; i++)
    {
        joint_incre.push_back((end_joint[i] - start_joint[i]) / times);
    }
    std::cout << "位置增量为: ";
    for (size_t i = 0; i < 6; i++)
    {
        std::cout << ", " << joint_incre[i];
    }
    std::cout << std::endl;
    float joint_trajectory[times + 1][6];
    for (size_t i = 0; i < times + 1; i++)
    {
        for (size_t j = 0; j < 6; j++)
        {
            joint_trajectory[i][j] = start_joint[j] + joint_incre[j] * i;
        }
    }
    std::cout << "轨迹值为: " << std::endl;
    for (size_t i = 0; i < times + 1; i++)
    {
        for (size_t j = 0; j < 6; j++)
        {
            std::cout << ", " << joint_trajectory[i][j];
        }
        std::cout << std::endl;
    }
    //计算对应的欧氏空间轨迹
    float euclid_trajectory[times + 1][6];
    for (size_t i = 0; i < times + 1; i++)
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
    std::cout << "欧氏空间轨迹值为: " << std::endl;
    for (size_t i = 0; i < times + 1; i++)
    {
        for (size_t j = 0; j < 6; j++)
        {
            std::cout << ", " << euclid_trajectory[i][j];
        }
        std::cout << std::endl;
    }
    const std::chrono::milliseconds interval(int(10));
    ret = Movej_Cmd(m_sockhand, start_joint, 20, 0, 0, 1);
    if (ret != 0)
    {
        std::cout << "回到初始位置失败" << std::endl;
        return 0;
    }
    int num = 0;
    bool reverse_flag = false;
    float last_joints[6];
    Eigen::Matrix<float, 3, 1> initial_position;
    initial_position << getUDPData.udp_euclid_position_[0], getUDPData.udp_euclid_position_[1], getUDPData.udp_euclid_position_[2];
    Eigen::Matrix<float, 3, 1> initial_euler;
    initial_euler << getUDPData.udp_euclid_euler_[0], getUDPData.udp_euclid_euler_[1], getUDPData.udp_euclid_euler_[2];
    hpfc.setInitialPose(initial_position, initial_euler);
    while (true)
    {
        auto time_start = std::chrono::high_resolution_clock::now();
        auto trajectory_start = std::chrono::steady_clock::now();
        if (reverse_flag)
        {
            num--;
        }
        else
        {
            num++;
        }
        if (num > 399)
        {
            reverse_flag = true;
        }
        else if (num < 1)
        {
            reverse_flag = false;
        }
        std::this_thread::sleep_until(trajectory_start + interval);
        Eigen::Matrix<float, 3, 1> position = {euclid_trajectory[num][0], euclid_trajectory[num][1], euclid_trajectory[num][2]};
        //计算力控结果
        Eigen::Matrix<float, 3, 1> cmd_orientation = {euclid_trajectory[num][3], euclid_trajectory[num][4], euclid_trajectory[num][5]};
        Eigen::Matrix<float, 6, 1> xyz_force(getUDPData.udp_external_force_);
        hpfc.updateData(position, cmd_orientation, xyz_force);

        float interval = 0.01;
        hpfc.computeAdmittance(interval);
        Pose q_pose;
        q_pose.position.x = hpfc.force_control_desired_position_[0];
        q_pose.position.y = hpfc.force_control_desired_position_[1];
        q_pose.position.z = hpfc.force_control_desired_position_[2];
        // q_pose.euler.rx = hpfc.force_control_desired_position_[3];
        // q_pose.euler.ry = hpfc.force_control_desired_position_[4];
        // q_pose.euler.rz = hpfc.force_control_desired_position_[5];
        q_pose.euler.rx = euclid_trajectory[num][3];
        q_pose.euler.ry = euclid_trajectory[num][4];
        q_pose.euler.rz = euclid_trajectory[num][5];
        float q_out[6];
        for (size_t j = 0; j < 6; j++)
        {
            last_joints[j] = getUDPData.udp_joint_position_[j];
        }
        ret = Algo_Inverse_Kinematics(last_joints, &q_pose, q_out, 1);
        if (ret != 0)
        {
            std::cout << "armPlanning模块: 关节逆运动学解算失败！ xyz为"
                      << q_pose.position.x << " " << q_pose.position.y << " " << q_pose.position.z << " "
                      << q_pose.quaternion.w << " " << q_pose.quaternion.x << " " << q_pose.quaternion.y << " " << q_pose.quaternion.z << std::endl;
            // std::cout << "关节逆运动学解算失败, xyz为" << q_pose.position.x << " " << q_pose.position.y << " " << q_pose.position.z << " ret: " << ret << std::endl;
            // std::cout << "关节逆运动学解算失败, rxryrz为" << q_pose.euler.rx << " " << q_pose.euler.ry << " " << q_pose.euler.rz << std::endl;
            return 0;
        }
        std::cout << "关节控制指令: " << std::endl;
        for (size_t j = 0; j < 6; j++)
        {
            std::cout << ", " << q_out[j];
        }
        std::cout << std::endl;
        std::cout << "当前关节位置: " << std::endl;
        for (size_t j = 0; j < 6; j++)
        {
            std::cout << ", " << getUDPData.udp_joint_position_[j];
        }
        std::cout << std::endl;
        // 控制机械臂运动
        ret = Movej_CANFD(m_sockhand, q_out, false, 1);
        // ret = Movej_CANFD(m_sockhand, joint_trajectory[num], false, 1);
        if (ret != 0)
        {
            std::cout << "透传控制机械部失败" << std::endl;
            return 0;
        }
        std::cout << "上一帧joint_trajectory数据: " << std::endl;
        for (size_t j = 0; j < 6; j++)
        {
            std::cout << ", " << joint_trajectory[num - 1][j];
        }
        std::cout << std::endl;
        std::cout << "当前帧joint_trajectory数据: " << std::endl;
        for (size_t j = 0; j < 6; j++)
        {
            std::cout << ", " << joint_trajectory[num][j];
        }
        std::cout << std::endl;
        std::cout << "上一帧euclid_trajectory数据: " << std::endl;
        for (size_t j = 0; j < 6; j++)
        {
            std::cout << ", " << euclid_trajectory[num - 1][j];
        }
        std::cout << std::endl;
        std::cout << "当前帧euclid_trajectory数据: " << std::endl;
        for (size_t j = 0; j < 6; j++)
        {
            std::cout << ", " << euclid_trajectory[num][j];
        }
        std::cout << std::endl;
        auto time_end = std::chrono::high_resolution_clock::now();
        double duration = std::chrono::duration_cast<std::chrono::nanoseconds>(time_end - time_start).count();
        std::cout << "控制周期时间: " << duration * 1e-9 << "第" << num << "个控制周期" << std::endl;
    }

    // auto end = std::chrono::high_resolution_clock::now();
    // double duration1 = std::chrono::duration_cast<std::chrono::nanoseconds>(time_end - time_start).count();
    // double duration2 = std::chrono::duration_cast<std::chrono::nanoseconds>(end - mid).count();
    // std::cout<<"运行次数: "<<rate<<std::endl;
    // std::cout<<"运行时间duration1: "<<duration1 * 1e-9<<std::endl;
    // std::cout<<"运行时间duration2: "<<duration2 * 1e-9<<std::endl;
}