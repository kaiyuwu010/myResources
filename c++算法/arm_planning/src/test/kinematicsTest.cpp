#include <ros/package.h>
#include <ros/ros.h>

#include <chrono>

#include "../../include/arm_planning/forwardKinematics.hpp"
#include "../../include/arm_planning/inverseKinematics.hpp"
#include "../dependency/rm-arm/include/rm_base.h"
#include "../dependency/yaml/include/yaml.h"

void kinematicsTest()
{
    RM_API_Init(ARM_65, NULL);
    SOCKHANDLE m_sockhand_ = Arm_Socket_Start((char*)"192.168.2.18", 8080, 5000);

    // MDH参数
    std::array<double, 7> a;
    std::array<double, 7> alpha;
    std::array<double, 7> d;
    std::array<double, 7> theta;
    // 加载参数
    std::string package_path = ros::package::getPath("arm_planning");
    YAML::Node config = YAML::LoadFile(package_path + "/config/kinematics.yaml");
    YAML::Node yaml_a = config["a"];
    YAML::Node yaml_alpha = config["alpha"];
    YAML::Node yaml_d = config["d"];
    YAML::Node yaml_theta = config["theta"];
    if (yaml_a.size() != 7 || yaml_alpha.size() != 7 || yaml_d.size() != 7 || yaml_theta.size() != 7)
    {
        std::cout << "MDH参数数量不对,请检查参数!" << std::endl;
        return;
    }
    for (std::size_t i = 0; i < yaml_a.size(); ++i)
    {
        a[i] = yaml_a[i].as<double>();
        alpha[i] = yaml_alpha[i].as<double>();
        d[i] = yaml_d[i].as<double>();
        theta[i] = yaml_theta[i].as<double>();
    }
    // std::cout << "a: "<<a[0]<<", "<<a[1]<<", "<<a[2]<<", "<<a[3]<<", "<<a[4]<<", "<<a[5]<<", "<<a[6]<<", "<<std::endl;
    // std::cout << "alpha: "<<alpha[0]<<", "<<alpha[1]<<", "<<alpha[2]<<", "<<alpha[3]<<", "<<alpha[4]<<", "<<alpha[5]<<", "<<alpha[6]<<", "<<std::endl;
    // std::cout << "d: "<<d[0]<<", "<<d[1]<<", "<<d[2]<<", "<<d[3]<<", "<<d[4]<<", "<<d[5]<<", "<<d[6]<<", "<<std::endl;
    // std::cout << "theta: "<<theta[0]<<", "<<theta[1]<<", "<<theta[2]<<", "<<theta[3]<<", "<<theta[4]<<", "<<theta[5]<<", "<<theta[6]<<", "<<std::endl;

    //计算正运动学
    ForwardKinematics fk(a, alpha, d, theta);
    std::vector<double> theta_input{0,
                                    5 / 180.0 * 3.141592, -30 / 180.0 * 3.141592, 60 / 180.0 * 3.141592,
                                    0 / 180.0 * 3.141592, 0 / 180.0 * 3.141592, 0 / 180.0 * 3.141592};
    std::cout << "输入角度为: "
              << theta_input[1] / 3.141592 * 180 << "  " << theta_input[2] / 3.141592 * 180 << "  " << theta_input[3] / 3.141592 * 180 << "  "
              << theta_input[4] / 3.141592 * 180 << "  " << theta_input[5] / 3.141592 * 180 << "  " << theta_input[6] / 3.141592 * 180 << std::endl;
    std::cout << "输入弧度角为: "
              << "  " << theta_input[1] << "  " << theta_input[2] << "  " << theta_input[3] << "  "
              << theta_input[4] << "  " << theta_input[5] << "  " << theta_input[6] << std::endl;
    Eigen::Matrix4d transform_output = fk.forwardKinematics(theta_input, 6);
    Eigen::Matrix4d transform_output1 = fk.forwardKinematics(theta_input, 3);
    std::cout << "正运动学结果为(单位mm):" << std::endl
              << transform_output << std::endl;
    // std::cout << "前三个轴正运动学结果为(单位mm):" << std::endl
    //           << transform_output1 << std::endl;
    Pose q_pose;
    float q_out[6] = {0, 0, 0, 0, 0, 0};
    q_pose.position.x = transform_output(0, 3) / 1000;
    q_pose.position.y = transform_output(1, 3) / 1000;
    q_pose.position.z = transform_output(2, 3) / 1000;
    Eigen::Quaterniond q(transform_output.block<3, 3>(0, 0));
    q_pose.quaternion.w = q.w();
    q_pose.quaternion.x = q.x();
    q_pose.quaternion.y = q.y();
    q_pose.quaternion.z = q.z();
    int ret_ = Algo_Inverse_Kinematics(q_out, &q_pose, q_out, 0);
    std::cout << "睿尔曼api逆运动学结果:" << ret_ << " ,角度： " << q_out[0] << " " << q_out[1] << " " << q_out[2]
              << " " << q_out[3] << " " << q_out[4] << " " << q_out[5] << std::endl;
    Pose cmd_pose;
    float cmd_joints[6] = {0, 0, 0, 0, 0, 0};
    for (int i = 0; i < 6; i++)
    {
        cmd_joints[i] = theta_input[i + 1] / 3.141592 * 180;
    }
    std::cout << "睿尔曼输入弧度角为: "
              << "  " << cmd_joints[0] << " " << cmd_joints[1] << "  " << cmd_joints[2] << "  " << cmd_joints[3] << "  "
              << cmd_joints[4] << "  " << cmd_joints[5] << std::endl;
    cmd_pose = Algo_Forward_Kinematics(cmd_joints);
    Eigen::Quaternionf quat(cmd_pose.quaternion.w, cmd_pose.quaternion.x, cmd_pose.quaternion.y, cmd_pose.quaternion.z);
    Eigen::Matrix3f rotationMatrix;
    rotationMatrix = quat.toRotationMatrix();
    std::cout << "睿尔曼api结果position:" << cmd_pose.position.x << " " << cmd_pose.position.y << " " << cmd_pose.position.z << std::endl;
    std::cout << "睿尔曼api结果为rotation: \n"
              << rotationMatrix << std::endl;
    Eigen::Matrix3d rotation_matrix = transform_output.block(0, 0, 3, 3);
    Eigen::Vector3d eulerAngle1 = rotation_matrix.eulerAngles(2, 1, 0);  // zyx顺序
    std::cout << "rpy欧拉角 rad:" << eulerAngle1[2] << " " << eulerAngle1[1] << " " << eulerAngle1[0] << std::endl;
    std::cout << "rpy欧拉角 deg:" << eulerAngle1[2] / 3.141592 * 180.0 << " " << eulerAngle1[1] / 3.141592 * 180.0 << " " << eulerAngle1[0] / 3.141592 * 180.0 << std::endl;

    //计算逆运动学
    InverseKinematics ik(a, alpha, d, theta);
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    ik.inverseKinematics(transform_output);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "逆运动学计算时间: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << " us" << std::endl;
    std::cout << "逆运动学结果为: " << std::endl;
    std::cout << "atan2(0, 0)" << atan2(0, -1) << std::endl;
    std::cout << "-------------------------弧度--------------------------"
              << std::endl
              << ik.theta[1][1] << ",       " << ik.theta[1][2] << ",       "
              << ik.theta[1][3] << ",       " << ik.theta[1][4] << ",       "
              << ik.theta[1][5] << ",       " << ik.theta[1][6] << std::endl
              << ik.theta[2][1] << ",       " << ik.theta[2][2] << ",       "
              << ik.theta[2][3] << ",       " << ik.theta[2][4] << ",       "
              << ik.theta[2][5] << ",       " << ik.theta[2][6] << std::endl
              << ik.theta[3][1] << ",       " << ik.theta[3][2] << ",       "
              << ik.theta[3][3] << ",       " << ik.theta[3][4] << ",       "
              << ik.theta[3][5] << ",       " << ik.theta[3][6] << std::endl
              << ik.theta[4][1] << ",       " << ik.theta[4][2] << ",       "
              << ik.theta[4][3] << ",       " << ik.theta[4][4] << ",       "
              << ik.theta[4][5] << ",       " << ik.theta[4][6] << std::endl
              << ik.theta[5][1] << ",       " << ik.theta[5][2] << ",       "
              << ik.theta[5][3] << ",       " << ik.theta[5][4] << ",       "
              << ik.theta[5][5] << ",       " << ik.theta[5][6] << std::endl
              << ik.theta[6][1] << ",       " << ik.theta[6][2] << ",       "
              << ik.theta[6][3] << ",       " << ik.theta[6][4] << ",       "
              << ik.theta[6][5] << ",       " << ik.theta[6][6] << std::endl
              << ik.theta[7][1] << ",       " << ik.theta[7][2] << ",       "
              << ik.theta[7][3] << ",       " << ik.theta[7][4] << ",       "
              << ik.theta[7][5] << ",       " << ik.theta[7][6] << std::endl
              << ik.theta[8][1] << ",       " << ik.theta[8][2] << ",       "
              << ik.theta[8][3] << ",       " << ik.theta[8][4] << ",       "
              << ik.theta[8][5] << ",       " << ik.theta[8][6] << std::endl;
    std::cout << "-------------------------角度--------------------------"
              << std::endl
              << ik.theta[1][1] / 3.14159 * 180 << "       "
              << ik.theta[1][2] / 3.14159 * 180 << "       "
              << ik.theta[1][3] / 3.14159 * 180 << "       "
              << ik.theta[1][4] / 3.14159 * 180 << "       "
              << ik.theta[1][5] / 3.14159 * 180 << "       "
              << ik.theta[1][6] / 3.14159 * 180 << std::endl
              << ik.theta[2][1] / 3.14159 * 180 << "       "
              << ik.theta[2][2] / 3.14159 * 180 << "       "
              << ik.theta[2][3] / 3.14159 * 180 << "       "
              << ik.theta[2][4] / 3.14159 * 180 << "       "
              << ik.theta[2][5] / 3.14159 * 180 << "       "
              << ik.theta[2][6] / 3.14159 * 180 << std::endl
              << ik.theta[3][1] / 3.14159 * 180 << "       "
              << ik.theta[3][2] / 3.14159 * 180 << "       "
              << ik.theta[3][3] / 3.14159 * 180 << "       "
              << ik.theta[3][4] / 3.14159 * 180 << "       "
              << ik.theta[3][5] / 3.14159 * 180 << "       "
              << ik.theta[3][6] / 3.14159 * 180 << std::endl
              << ik.theta[4][1] / 3.14159 * 180 << "       "
              << ik.theta[4][2] / 3.14159 * 180 << "       "
              << ik.theta[4][3] / 3.14159 * 180 << "       "
              << ik.theta[4][4] / 3.14159 * 180 << "       "
              << ik.theta[4][5] / 3.14159 * 180 << "       "
              << ik.theta[4][6] / 3.14159 * 180 << std::endl
              << ik.theta[5][1] / 3.14159 * 180 << "       "
              << ik.theta[5][2] / 3.14159 * 180 << "       "
              << ik.theta[5][3] / 3.14159 * 180 << "       "
              << ik.theta[5][4] / 3.14159 * 180 << "       "
              << ik.theta[5][5] / 3.14159 * 180 << "       "
              << ik.theta[5][6] / 3.14159 * 180 << std::endl
              << ik.theta[6][1] / 3.14159 * 180 << "       "
              << ik.theta[6][2] / 3.14159 * 180 << "       "
              << ik.theta[6][3] / 3.14159 * 180 << "       "
              << ik.theta[6][4] / 3.14159 * 180 << "       "
              << ik.theta[6][5] / 3.14159 * 180 << "       "
              << ik.theta[6][6] / 3.14159 * 180 << std::endl
              << ik.theta[7][1] / 3.14159 * 180 << "       "
              << ik.theta[7][2] / 3.14159 * 180 << "       "
              << ik.theta[7][3] / 3.14159 * 180 << "       "
              << ik.theta[7][4] / 3.14159 * 180 << "       "
              << ik.theta[7][5] / 3.14159 * 180 << "       "
              << ik.theta[7][6] / 3.14159 * 180 << std::endl
              << ik.theta[8][1] / 3.14159 * 180 << "       "
              << ik.theta[8][2] / 3.14159 * 180 << "       "
              << ik.theta[8][3] / 3.14159 * 180 << "       "
              << ik.theta[8][4] / 3.14159 * 180 << "       "
              << ik.theta[8][5] / 3.14159 * 180 << "       "
              << ik.theta[8][6] / 3.14159 * 180 << std::endl;
}

int main(int argc, char** argv)
{
    kinematicsTest();
    return 0;
}
