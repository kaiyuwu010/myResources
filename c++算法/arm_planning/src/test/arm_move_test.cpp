#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include "../dependency/rm-arm/include/rm_base.h"
#include <nlohmann/json.hpp>
#include <mutex>

class UDPServer {
public:
    int num=0;
    //关节角互斥锁
    std::mutex realtime_arm_data_mutex_;
    nlohmann::json jsonData_;
    //通过UDP获取的主要参数
    float udp_external_force_[6];
    UDPServer(const std::string& address, int port) {
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
    }

    ~UDPServer() {
        close(server_fd);
    }

    void receiveMessages() {
        char buffer[1024];
        struct sockaddr_in client_addr;
        socklen_t len = sizeof(client_addr);

        while (true) {
            num++;
            int n = recvfrom(server_fd, buffer, sizeof(buffer), 0, (struct sockaddr*)&client_addr, &len);
            if (n < 0) {
                std::cerr << "Receive failed" << std::endl;
                break;
            }
            buffer[n] = '\0';
            std::cout << "!!!!!!!!!!!!!!!Received message: " << num << "个message"<<buffer<<std::endl;
            std::lock_guard<std::mutex> lock(realtime_arm_data_mutex_);
            jsonData_ = nlohmann::json::parse(buffer);
            auto external_force = jsonData_["six_force_sensor"]["zero_force"];
            for (size_t i=0; i<6; i++) 
            {
                udp_external_force_[i] = static_cast<float>(external_force[i]) / 1000;
            }
        }
    }

private:
    int server_fd;
    struct sockaddr_in server_addr;
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
int main() 
{
    UDPServer server("192.168.1.100", 8099);
    std::thread updateArmData(&UDPServer::receiveMessages, &server);
    updateArmData.detach();
    // 连接, 初始化API, 注册回调函数
    RM_API_Init(ARM_65, MCallback);
    SOCKHANDLE m_sockhand = -1;
    m_sockhand =  Arm_Socket_Start((char*)"192.168.1.18", 8080, 5000);
    int ret = -1;
    //关节空间多项式规划
    Pose pose0;
    pose0.position.x = -0.2;
    pose0.position.y = -0.2;
    pose0.position.z = 0.2;
    pose0.euler.rx = 3.14159;
    pose0.euler.ry = 0.0;
    pose0.euler.rz = 0.0;
    Pose pose1 = pose0;
    pose1.position.y = -0.1;
    Pose pose2 = pose0;
    pose2.position.y = 0.0;
    Pose pose3 = pose0;
    pose3.position.y = 0.1;
    Pose pose4 = pose0;
    pose4.position.y = 0.2;
    // for(size_t i=0; i<100; i++)
    // {
    ret = Moves_Cmd(m_sockhand, pose0, 20, 0, 1, 1);;
    if(ret != 0 )
    {
        return 0;
    }
    ret = Moves_Cmd(m_sockhand, pose2, 20, 0, 1, 1);
    if(ret != 0 )
    {
        return 0;
    }
    ret = Moves_Cmd(m_sockhand, pose4, 20, 0, 0, 1);
    if(ret != 0 )
    {
        return 0;
    }
    // }
    
    // /*
    // 梯形速度规划，加速段2s，匀速段2s，减速段2s，令t0 = 2s
    // 加速段距离S = 1/2 * a * t0^2，减速段距离S1 = S，匀速段距离S2 = 2S
    // 总距离 4S = 0.6m，S = 0.15m，a = 0.075m/s^2 ，匀速段速度为 v = 0.15m/s
    // */
    // const double t = 0.01;
    // double a = 0.075;
    // double v = 0.15;
    // float q_pose_joints[6] = {-56.304, 36.39, 60.192, -0.015, 83.395, -56.303};
    // ret = Movej_Cmd(m_sockhand, q_pose_joints, 20, 0, 0, 1);
    // if(ret != 0)
    // {
    //     std::cout<<"回到初始位置失败"<<std::endl;
    //     return 0;
    // }
    // float q_data[6] = {-56.304, 36.39, 60.192, -0.015, 83.395, -56.303};
    // Pose q_pose;
    // q_pose.position.x = -0.2;
    // q_pose.position.y = 0.3;
    // q_pose.position.z = 0.25;
    // q_pose.euler.rx = 3.14159;
    // q_pose.euler.ry = 0.0;
    // q_pose.euler.rz = 0.0;
    // //三个行程，每个行程2s，每0.01s为一个控制周期
    // const std::chrono::milliseconds interval(10);
    // for(int cycle=1; cycle<201; cycle++)
    // {
    //     auto stage1_start = std::chrono::steady_clock::now();
    //     for (size_t i=0; i<6; i++) 
    //     {
    //         std::cout << server.udp_external_force_[i] << ", ";
    //     }
    //     double time_point = cycle*t;
    //     float length_placement = float(0.5 * a * time_point * time_point);
    //     // std::cout<<"运动学逆解, length_placement为"<<length_placement<<std::endl;
    //     q_pose.position.y = 0.3 - length_placement;
    //     ret = Algo_Inverse_Kinematics(q_data, &q_pose, q_data, 1);
    //     if(ret != 0)
    //     {
    //         std::cout<<"运动学逆解失败"<<std::endl;
    //         return 0;
    //     }
    //     std::cout<<"运动学逆解: "<<cycle<<" , q_posexyz为"<<q_pose.position.x<<" "<<q_pose.position.y<<" "<<q_pose.position.z<<std::endl;
    //     ret = Movej_CANFD(m_sockhand, q_data, false, 1);
    //     if(ret != 0)
    //     {
    //         std::cout<<"关节透传控制失败"<<std::endl;
    //         return 0;
    //     }
    //     std::this_thread::sleep_until(stage1_start + interval);
    // }
    // for(int cycle=1; cycle<201; cycle++)
    // {
    //     auto stage2_start = std::chrono::steady_clock::now();
    //     for (size_t i=0; i<6; i++) 
    //     {
    //         std::cout << server.udp_external_force_[i] << ", ";
    //     }
    //     double time_point = cycle*t;
    //     float length_placement = float(0.15 + v * time_point);
    //     q_pose.position.y = 0.3 - length_placement;
    //     ret = Algo_Inverse_Kinematics(q_data, &q_pose, q_data, 1);
    //     if(ret != 0)
    //     {
    //         std::cout<<"运动学逆解失败"<<std::endl;
    //         return 0;
    //     }
    //     std::cout<<"运动学逆解: "<<cycle<<" , q_posexyz为"<<q_pose.position.x<<" "<<q_pose.position.y<<" "<<q_pose.position.z<<std::endl;
    //     ret = Movej_CANFD(m_sockhand, q_data, false, 1);
    //     if(ret != 0)
    //     {
    //         std::cout<<"关节透传控制失败"<<std::endl;
    //         return 0;
    //     }
    //     std::this_thread::sleep_until(stage2_start + interval);
    // }
    // for(int cycle=199; cycle>-1; cycle--)
    // {
    //     auto stage3_start = std::chrono::steady_clock::now();
    //     for (size_t i=0; i<6; i++) 
    //     {
    //         std::cout << server.udp_external_force_[i] << ", ";
    //     }
    //     double time_point = cycle*t;
    //     float length_placement = float(0.5 * a * time_point * time_point);
    //     q_pose.position.y = -0.3 + length_placement;
    //     ret = Algo_Inverse_Kinematics(q_data, &q_pose, q_data, 1);
    //     if(ret != 0)
    //     {
    //         std::cout<<"运动学逆解失败"<<std::endl;
    //         return 0;
    //     }
    //     std::cout<<"运动学逆解: "<<cycle<<" , q_posexyz为"<<q_pose.position.x<<" "<<q_pose.position.y<<" "<<q_pose.position.z<<std::endl;
    //     ret = Movej_CANFD(m_sockhand, q_data, false, 1);
    //     if(ret != 0)
    //     {
    //         std::cout<<"关节透传控制失败"<<std::endl;
    //         return 0;
    //     }
    //     std::this_thread::sleep_until(stage3_start + interval);
    // }
    return 0;
}