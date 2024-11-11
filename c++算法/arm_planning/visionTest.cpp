#include <functional>

#include "../dependency/rm-arm/include/rm_base.h"
#undef SUCCESS
#include <unistd.h>

#include <Eigen/Dense>
#include <cstdlib>

#include "../include/arm_planning/getArmUdpData.hpp"
#include "../include/arm_planning/gripperControl.hpp"
#include "../include/arm_planning/trajectoryPlanning.h"
#include "rclcpp/rclcpp.hpp"
#include "robot_interface/srv/cmd.hpp"

class visionControlTest : public rclcpp::Node
{
public:
    visionControlTest() :
        Node("vision_test_node")
    {
        // 创建客户端
        client_ = this->create_client<robot_interface::srv::Cmd>("query_poses");
        // 机械臂初始位置四元数表示
        init_end_to_base_ = Eigen::Affine3d::Identity();
        init_end_to_base_.translation() << -0.359,
            0.0,
            0.169;
        init_end_to_base_.rotate(Eigen::AngleAxisd(-0.768, Eigen::Vector3d::UnitZ()) *
                                 Eigen::AngleAxisd(0.78, Eigen::Vector3d::UnitY()) *
                                 Eigen::AngleAxisd(2.544, Eigen::Vector3d::UnitX()));
        std::cout << "init_end_to_base_:\n"
                  << init_end_to_base_.matrix() << std::endl;
        // 相机外参初始化
        cam_to_end_ = Eigen::Affine3d::Identity();
        cam_to_end_.translation() << -0.0658948,
            0.0752519,
            0.029099;
        cam_to_end_.rotate(Eigen::AngleAxisd(-119.084 / 180.0 * M_PI, Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(0.275362 / 180.0 * M_PI, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(0.416655 / 180.0 * M_PI, Eigen::Vector3d::UnitX()));
        std::cout << "cam_to_end_:\n"
                  << cam_to_end_.matrix() << std::endl;
        // 机械臂连接, 初始化API, 注册回调函数
        RM_API_Init(ARM_65, NULL);
        m_sockhand_ = Arm_Socket_Start((char*)"192.168.2.18", 8080, 5000);
        version_ = API_Version();
        std::cout << "armPlanning模块: API_Version : " << version_ << std::endl;
        // 机械臂回到准备位置
        ret_ = Movej_Cmd(m_sockhand_, readyPose_, 30, 0, 0, 1);
        if (ret_ != 0)
        {
            std::cout << "armPlanning模块: 到达初始位姿失败！" << std::endl;
            exit(1);
        }
        // 连接夹爪
        handle_ = connect();
        // 获取夹爪控制指令
        torque_open_data = Gripper_Torque(400);         // 力控模式（数值范围在-2000~2000，对应的相电流-32A~32A）正数是打开夹爪，负数是关闭夹爪
        torque_close_data = Gripper_Torque(-400);       // 力控模式（数值范围在-2000~2000，对应的相电流-32A~32A）正数是打开夹爪，负数是关闭夹爪
        position_open_data = Gripper_Pos(20000, 1000);  // 位置模式（位置数值范围在0~36000，速度范围0~65535）  10000对应58mm 20000对应94mm 25000对应最大106mm
        position_close_data = Gripper_Pos(0, 1000);     // 位置模式（位置数值范围在0~36000，速度范围0~65535）  10000对应58mm 20000对应94mm 25000对应最大106mm
    }
    ~visionControlTest()
    {
        // 释放内存
        delete[] torque_open_data;
        delete[] torque_close_data;
        delete[] position_open_data;
        delete[] position_close_data;
        // 关闭设备
        Close_Device(handle_, 0);
    }
    void grasp_object()
    {
        // 从服务端获取位姿
        auto request = std::make_shared<robot_interface::srv::Cmd::Request>();
        auto result_future = client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto result = result_future.get();
            RCLCPP_INFO(this->get_logger(), "get pose!");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service");
        }
        // 打开夹爪
        Send_Data(handle_, channel, ID, 8, torque_open_data);
        sleep(0.5);
        // 计算目标到相机位姿
        obj_to_cam_ = Eigen::Affine3d::Identity();
        obj_to_cam_.translation() << result_future.get()->rst.data.pose.pose.position.x,
            result_future.get()->rst.data.pose.pose.position.y,
            result_future.get()->rst.data.pose.pose.position.z;

        Eigen::Quaterniond quaternion(result_future.get()->rst.data.pose.pose.orientation.w,
                                      result_future.get()->rst.data.pose.pose.orientation.x,
                                      result_future.get()->rst.data.pose.pose.orientation.y,
                                      result_future.get()->rst.data.pose.pose.orientation.z);
        // Eigen::Quaterniond flip_xy(0, 0, 0, 1);
        // quaternion = quaternion * flip_xy;
        Eigen::Quaterniond flip_xy(0.7071, 0, 0, 0.7071);
        std::cout << "-----------------------------------------------------------" << std::endl;
        std::cout << "视觉给的position:" << std::endl;
        std::cout << result_future.get()->rst.data.pose.pose.position.x << " " << result_future.get()->rst.data.pose.pose.position.y << " "
                  << result_future.get()->rst.data.pose.pose.position.z << std::endl;
        std::cout << "视觉给的orientation:" << std::endl;
        std::cout << result_future.get()->rst.data.pose.pose.orientation.w << " " << result_future.get()->rst.data.pose.pose.orientation.x << " "
                  << result_future.get()->rst.data.pose.pose.orientation.y << " " << result_future.get()->rst.data.pose.pose.orientation.z << std::endl;
        obj_to_cam_.rotate(flip_xy);
        std::cout << "视觉给的obj_to_cam_:\n"
                  << obj_to_cam_.matrix() << std::endl;
        std::cout << "-----------------------------------------------------------" << std::endl;
        Eigen::Affine3d final_pose = init_end_to_base_ * cam_to_end_ * obj_to_cam_;
        Eigen::Matrix3d rotation_matrix = final_pose.rotation();
        Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0);
        Eigen::Vector3d position = final_pose.translation();
        // 计算关节表示的目标位置
        Pose q_pose;
        // q_pose.position.x = position[0];
        // q_pose.position.y = position[1];
        // q_pose.position.z = position[2];
        // q_pose.euler.rx = euler_angles[2];
        // q_pose.euler.ry = euler_angles[1];
        // q_pose.euler.rz = euler_angles[0];
        q_pose.position.x = position[0] + 0.05;
        q_pose.position.y = position[1];
        q_pose.position.z = position[2];
        q_pose.euler.rx = 3.141;
        q_pose.euler.ry = 1.57;
        q_pose.euler.rz = 0;
        std::cout << "最终position:" << std::endl;
        std::cout << position << std::endl;
        std::cout << "最终euler_angles:" << std::endl;
        std::cout << euler_angles << std::endl;
        ret_ = Algo_Inverse_Kinematics(readyPose_, &q_pose, targetPose_, 1);
        if (ret_ != 0)
        {
            std::cout << "armPlanning模块: 关节逆运动学解算失败！" << std::endl;
            exit(1);
        }
        std::cout << "targetPose_:" << std::endl;
        for (size_t i = 0; i < 6; i++)
        {
            std::cout << targetPose_[i] << " ";
        }
        // 前往目标位置
        ret_ = Movej_Cmd(m_sockhand_, targetPose_, 30, 0, 0, 1);
        if (ret_ != 0)
        {
            std::cout << "armPlanning模块: 到达目标位姿失败！" << std::endl;
            exit(1);
        }
        // 关闭夹爪
        Send_Data(handle_, channel, ID, 8, torque_close_data);
        sleep(0.5);
        // 到抬起位置
        ret_ = Movej_Cmd(m_sockhand_, pickUpObjectPose_, 30, 0, 0, 1);
        if (ret_ != 0)
        {
            std::cout << "armPlanning模块: 到达初始位姿失败！" << std::endl;
            exit(1);
        }
        // 到初始位置
        ret_ = Movej_Cmd(m_sockhand_, putObjectPose_, 30, 0, 0, 1);
        if (ret_ != 0)
        {
            std::cout << "armPlanning模块: 到达初始位姿失败！" << std::endl;
            exit(1);
        }
        // 打开夹爪
        Send_Data(handle_, channel, ID, 8, torque_open_data);
    }

private:
    rclcpp::Client<robot_interface::srv::Cmd>::SharedPtr client_;
    // 机械臂参数
    char* version_;
    SOCKHANDLE m_sockhand_ = -1;
    int ret_ = -1;
    float readyPose_[6] = {0, 30, 124, 0.0, -28, 30.00};
    float putObjectPose_[6] = {0, 64.6, 118.4, 0.0, -85.134, 0};
    float pickUpObjectPose_[6] = {0, 14.486, 117.527, 0.0, -34, 0};
    float targetPose_[6] = {0, 78.659, 71.658, 0, -57.021, 0};
    // 夹爪参数
    int channel = 0;  // CAN默认通道选择
    int ID = 0x141;   // ID=0x140 + 电机ID（1~32）
    unsigned int handle_;
    uint8_t* torque_open_data;
    uint8_t* torque_close_data;
    uint8_t* position_open_data;
    uint8_t* position_close_data;
    // 位姿变换四元数
    Eigen::Affine3d cam_to_end_;
    Eigen::Affine3d init_end_to_base_;
    Eigen::Affine3d obj_to_cam_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<visionControlTest>();
    node->grasp_object();
    rclcpp::shutdown();
    return 0;
}
