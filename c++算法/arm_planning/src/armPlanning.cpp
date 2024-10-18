#include <ros/package.h>

#include <chrono>
#include <functional>
#include <mutex>
#include <nlohmann/json.hpp>
#include <thread>

#include "../dependency/rm-arm/include/rm_base.h"
#include "../dependency/yaml/include/yaml.h"
#include "../include/arm_planning/forwardKinematics.hpp"
#include "../include/arm_planning/getArmUdpData.hpp"
#include "../include/arm_planning/hybridPositionForceControl.hpp"
#include "../include/arm_planning/inverseKinematics.hpp"
#include "../include/arm_planning/trajectoryPlanning.h"

#define Deg_To_Rad 0.017453289
#define Rad_To_Deg 57.295791433
class ArmPlanning
{
private:
    SOCKHANDLE m_sockhand_ = -1;
    int ret_ = -1;
    ReceiveDataByUDP getUDPData_;
    // 初始化各个模块
    hybridPositionForceControl hpfc_;
    CubicSplinePlanning csp_;
    std::shared_ptr<InverseKinematics> ik_ptr_;
    std::shared_ptr<ForwardKinematics> fk_ptr_;
    // MDH参数
    std::array<double, 7> a;
    std::array<double, 7> alpha;
    std::array<double, 7> d;
    std::array<double, 7> theta;
    // 目标关节位置变量
    bool first_update_cmd_joints_ = true;
    float last_joints_[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    float q_out[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    //初始化点位
    float readyPose_[6] = {-43.401, -24.554, 44.461, 3.184, 97.649, 82.511};
    float fryPoint_[6] = {-36.055, 16.221, 39.515, -3.817, 82.001, 98};
    std::vector<double> fryPoint0_ = {-36.055, 16.221, 39.515, -3.817, 82.001, 98};
    std::vector<double> fryPoint1_ = {-36.055, 20.538, 39.526, -3.758, 82.029, 98};
    std::vector<double> fryPoint2_ = {-36.055, 19.602, 39.567, -3.298, 86.785, 98};
    std::vector<double> fryPoint3_ = {-36.055, 16.557, 39.554, -2.272, 92.964, 98};
    std::vector<double> fryPoint4_ = {-36.055, 12.325, 39.534, -1.1, 100.607, 98};
    std::vector<double> fryPoint5_ = {-36.055, 7.764, 39.519, -1.1, 107.054, 98};
    std::vector<double> fryPoint6_ = {-36.055, 0.148, 39.0, -1.406, 105.132, 98};
    // std::vector<double> fryPathPoint0_ = {6.788, 16.491, 62.225, 28.166, 82.207, 93.140};
    // std::vector<double> fryPathPoint1_ = {-1.303, 30.036, 75.831, 23.215, 67.295, 78.609};
    // std::vector<double> fryPathPoint2_ = {1.415, 32.302, 77.140, 27.652, 63.621, 78.446};
    // std::vector<double> fryPathPoint3_ = {4.437, 32.221, 78.909, 34.079, 61.035, 78.872};
    // std::vector<double> fryPathPoint4_ = {4.546, 31.839, 78.684, 42.330, 61.107, 78.807};
    // std::vector<double> fryPathPoint5_ = {4.023, 16.388, 66.435, 45.708, 78.582, 87.426};

    float testPoint_[6] = {-45.01, 16.641, 77.015, -0.017, 86.350, -44.989};
    std::vector<double> testPoint0_ = {-45.01, 16.641, 77.015, -0.017, 86.350, -44.989};
    std::vector<double> testPoint1_ = {45.010, 16.641, 76.984, -0.017, 86.391, 45.011};

public:
    ArmPlanning(unsigned int dimension) :
        csp_(dimension, 20), getUDPData_("192.168.1.100", 8099)
    {
        // 连接, 初始化API, 注册回调函数
        RM_API_Init(ARM_65, NULL);
        m_sockhand_ = Arm_Socket_Start((char*)"192.168.2.18", 8080, 5000);
        char* version;
        version = API_Version();
        std::cout << "armPlanning模块: API_Version : " << version << std::endl;
        //机械臂回到准备位置
        ret_ = Movej_Cmd(m_sockhand_, readyPose_, 30, 0, 0, 1);
        if (ret_ != 0)
        {
            std::cout << "armPlanning模块: 到达初始位姿失败！" << std::endl;
            exit(1);
        }
        // 加载运动学参数
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
        ik_ptr_ = std::make_shared<InverseKinematics>(a, alpha, d, theta);
        fk_ptr_ = std::make_shared<ForwardKinematics>(a, alpha, d, theta);
    }
    ~ArmPlanning()
    {
        Arm_Socket_Close(m_sockhand_);
        m_sockhand_ = -1;
    }
    int stirFry()
    {
        ret_ = Movej_Cmd(m_sockhand_, fryPoint_, 20, 0, 0, 1);
        if (ret_ != 0)
        {
            std::cout << "armPlanning模块: 回到初始位置失败" << std::endl;
            exit(1);
        }
        std::vector<std::vector<double>> points_for_planning_;
        // points_for_planning_.push_back(testPoint0_);
        // points_for_planning_.push_back(testPoint1_);
        // points_for_planning_.push_back(testPoint0_);
        points_for_planning_.push_back(fryPoint0_);
        points_for_planning_.push_back(fryPoint1_);
        points_for_planning_.push_back(fryPoint2_);
        points_for_planning_.push_back(fryPoint3_);
        points_for_planning_.push_back(fryPoint4_);
        points_for_planning_.push_back(fryPoint5_);
        points_for_planning_.push_back(fryPoint6_);
        points_for_planning_.push_back(fryPoint0_);
        for (size_t i = 0; i < points_for_planning_.size(); i++)
        {
            csp_.addPoint(points_for_planning_[i]);
        }
        csp_.initCubicSplineWithControlPoints();
        csp_.insertAllJointsCubicSpline(0.01);

        for (size_t i = 0; i < csp_.pos_list_.size(); i++)
        {
            std::cout << csp_.pos_list_[i].time;
            for (size_t j = 0; j < 6; j++)
            {
                std::cout << " " << csp_.pos_list_[i].coordinates[j];
            }
            std::cout << std::endl;
        }
        // return 0;
        for (size_t j = 0; j < 100; j++)
        {
            Eigen::Matrix<float, 3, 1> initial_position;
            initial_position << getUDPData_.udp_euclid_position_[0], getUDPData_.udp_euclid_position_[1], getUDPData_.udp_euclid_position_[2];
            Eigen::Matrix<float, 3, 1> initial_euler;
            initial_euler << getUDPData_.udp_euclid_euler_[0], getUDPData_.udp_euclid_euler_[1], getUDPData_.udp_euclid_euler_[2];
            hpfc_.setInitialPose(initial_position, initial_euler);
            std::cout << "initialPosition: " << initial_position[0] << " " << initial_position[1] << " " << initial_position[2] << " " << std::endl;
            for (size_t i = 0; i < csp_.pos_list_.size(); i++)
            {
                auto start = std::chrono::steady_clock::now();
                std::vector<double> cmd_joints = {0,
                                                  double(csp_.pos_list_[i].coordinates[0]) * Deg_To_Rad, double(csp_.pos_list_[i].coordinates[1]) * Deg_To_Rad,
                                                  double(csp_.pos_list_[i].coordinates[2]) * Deg_To_Rad, double(csp_.pos_list_[i].coordinates[3]) * Deg_To_Rad,
                                                  double(csp_.pos_list_[i].coordinates[4]) * Deg_To_Rad, double(csp_.pos_list_[i].coordinates[5]) * Deg_To_Rad};
                std::cout << "joints: " << csp_.pos_list_[i].coordinates[0] << ", " << csp_.pos_list_[i].coordinates[1] << ", " << csp_.pos_list_[i].coordinates[2] << ", "
                          << csp_.pos_list_[i].coordinates[3] << ", " << csp_.pos_list_[i].coordinates[4] << ", " << csp_.pos_list_[i].coordinates[5] << " ,time: " << csp_.pos_list_[i].time << std::endl;
                Pose cmd_pose;
                Eigen::Matrix4d transform_output = fk_ptr_->forwardKinematics(cmd_joints, 6);
                Eigen::Matrix<float, 3, 1> position = transform_output.block<3, 1>(0, 3).cast<float>() / 1000.0;
                Eigen::Quaterniond quat(transform_output.block<3, 3>(0, 0));
                Eigen::Matrix<float, 6, 1> force;
                // force << 0, 0, 0, 0, 0, 0;
                force << getUDPData_.udp_external_force_[0], getUDPData_.udp_external_force_[1], getUDPData_.udp_external_force_[2],
                    getUDPData_.udp_external_force_[3], getUDPData_.udp_external_force_[4], getUDPData_.udp_external_force_[5];
                std::cout << "armPlanning模块: force "
                          << getUDPData_.udp_external_force_[0] << " " << getUDPData_.udp_external_force_[1] << " " << getUDPData_.udp_external_force_[2] << " "
                          << getUDPData_.udp_external_force_[3] << " " << getUDPData_.udp_external_force_[4] << " " << getUDPData_.udp_external_force_[5] << std::endl;
                Eigen::Quaternionf quat_float = quat.cast<float>();
                hpfc_.updateData(position, quat_float, force);
                double second_interval;
                if (i == 0)
                {
                    second_interval = 0;
                }
                else
                {
                    second_interval = csp_.pos_list_[i].time - csp_.pos_list_[i - 1].time;
                }
                float control_interval = float(second_interval);
                hpfc_.computeAdmittance(control_interval);
                transform_output.block<3, 1>(0, 3) = hpfc_.force_control_desired_position_.block<3, 1>(0, 0).cast<double>() * 1000.0;
                float last_joints[6];
                for (size_t k = 0; k < 6; k++)
                {
                    last_joints[k] = getUDPData_.udp_joint_position_[k];
                }

                if (first_update_cmd_joints_ == true)
                {
                    ik_ptr_->inverseKinematics(transform_output, last_joints[0]);
                    first_update_cmd_joints_ = false;
                }
                else
                {
                    ik_ptr_->inverseKinematics(transform_output, last_joints_[0]);
                }
                double min_angel_diff = 10000;
                int index = 0;
                for (size_t k = 1; k < 9; k++)
                {
                    double sum_angel_diff = 0;
                    for (size_t n = 1; n < 7; n++)
                    {
                        sum_angel_diff += std::fabs(last_joints_[n - 1] - ik_ptr_->theta[k][n]);
                    }
                    if (sum_angel_diff < min_angel_diff)
                    {
                        min_angel_diff = sum_angel_diff;
                        index = k;
                    }
                }
                for (size_t k = 1; k < 7; k++)
                {
                    last_joints_[k - 1] = ik_ptr_->theta[index][k];
                }
                float cmd_angel_joints[6];
                for (size_t k = 0; k < 6; k++)
                {
                    cmd_angel_joints[k] = float(last_joints_[k]) * Rad_To_Deg;
                }
                ret_ = Movej_CANFD(m_sockhand_, cmd_angel_joints, false, 1);
                if (ret_ != 0)
                {
                    std::cout << "armPlanning模块: 透传控制机械臂失败！" << std::endl;
                    exit(1);
                }
                // std::cout << "cmd_angel_joints: " << cmd_angel_joints[0] << ", " << cmd_angel_joints[1] << ", " << cmd_angel_joints[2] << ", "
                //           << cmd_angel_joints[3] << ", " << cmd_angel_joints[4] << ", " << cmd_angel_joints[5] << std::endl;
                std::chrono::milliseconds interval(int(second_interval * 1000));
                std::this_thread::sleep_until(start + interval);
                // std::cout << "armPlanning模块: sleep: " << second_interval << std::endl;
            }
        }
        return 0;
    }
};

int main(int argc, char** argv)
{
    // ros::init(argc, argv, "motion_planning");
    // ros::NodeHandle nh;
    ArmPlanning ap(6);
    ap.stirFry();
    // ros::Rate rate(1);
    // while (ros::ok())
    // {
    //     rate.sleep();
    // }
    return 0;
}
