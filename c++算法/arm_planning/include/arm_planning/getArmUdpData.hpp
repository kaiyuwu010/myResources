#include <arpa/inet.h>
#include <unistd.h>

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>
#include <thread>

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
        if (server_fd < 0)
        {
            std::cerr << "Failed to create socket" << std::endl;
            exit(EXIT_FAILURE);
        }
        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = INADDR_ANY;
        server_addr.sin_port = htons(port);
        if (bind(server_fd, (const struct sockaddr*)&server_addr, sizeof(server_addr)) < 0)
        {
            std::cerr << "Bind failed" << std::endl;
            close(server_fd);
            exit(EXIT_FAILURE);
        }
        std::thread updateData(&ReceiveDataByUDP::receiveMessages, this);
        updateData.detach();
    }

    ~ReceiveDataByUDP()
    {
        close(server_fd);
    }

    void receiveMessages()
    {
        while (true)
        {
            auto time_start = std::chrono::high_resolution_clock::now();
            int n = recvfrom(server_fd, buffer, sizeof(buffer), 0, (struct sockaddr*)&client_addr, &len);
            auto time_end = std::chrono::high_resolution_clock::now();
            if (n < 0)
            {
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
            for (size_t i = 0; i < 6; i++)
            {
                udp_joint_position_[i] = static_cast<float>(joint_position[i]) / 1000;
                udp_external_force_[i] = static_cast<float>(external_force[i]) / 1000;
                // std::cout << udp_joint_position_[i] << "--" << udp_external_force_[i] << ", ";
            }
            // std::cout << std::endl;
            // std::cout << "ReceiveDataByUDP模块: udp_euclid_position_ , udp_euclid_euler_: \n";
            for (size_t i = 0; i < 3; i++)
            {
                udp_euclid_position_[i] = static_cast<float>(euclid_position[i]) / 1000000;
                udp_euclid_euler_[i] = static_cast<float>(euclid_euler[i]) / 1000;
                // std::cout << udp_euclid_position_[i] << "--" << udp_euclid_euler_[i] << ", ";
            }
            // std::cout << std::endl;
            auto time_now = std::chrono::high_resolution_clock::now();
            double duration = std::chrono::duration_cast<std::chrono::nanoseconds>(time_now - last_update_time).count();
            double duration1 = std::chrono::duration_cast<std::chrono::nanoseconds>(time_end - time_start).count();
            last_update_time = time_now;
            // std::cout << "数据获取时间间隔: " << duration * 1e-9 << "   数据获取时间: " << duration1 * 1e-9 << std::endl;
        }
        close(server_fd);
    }
};