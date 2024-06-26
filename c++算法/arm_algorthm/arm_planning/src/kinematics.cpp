#include "arm_planning/kinematics.h"

Eigen::Matrix4d Kinematics::kinematics(
    const std::vector<double>& theta_input,
    const int& index_joint)  //参数为弧度表示的角度
{
    Eigen::Matrix4d T[6];
    for (int i = 0; i < 6; i++) {
        T[i](0, 0) = cos(theta_input[i]);
        T[i](0, 1) = -sin(theta_input[i]);
        T[i](0, 2) = 0;
        T[i](0, 3) = a[i] / 1000;
        T[i](1, 0) = sin(theta_input[i]) * cos(alpha[i] / 180 * M_PI);
        T[i](1, 1) = cos(theta_input[i]) * cos(alpha[i] / 180 * M_PI);
        T[i](1, 2) = -sin(alpha[i] / 180 * M_PI);
        T[i](1, 3) = -sin(alpha[i] / 180 * M_PI) * d[i] / 1000;
        T[i](2, 0) = sin(theta_input[i]) * sin(alpha[i] / 180 * M_PI);
        T[i](2, 1) = cos(theta_input[i]) * sin(alpha[i] / 180 * M_PI);
        T[i](2, 2) = cos(alpha[i] / 180 * M_PI);
        T[i](2, 3) = cos(alpha[i] / 180 * M_PI) * d[i] / 1000;
        T[i](3, 0) = 0;
        T[i](3, 1) = 0;
        T[i](3, 2) = 0;
        T[i](3, 3) = 1;
    }
    Eigen::Matrix4d transform_output;
    switch (index_joint) {
        case 1:
            transform_output = T[0];
            break;
        case 2:
            transform_output = T[0] * T[1];
            break;
        case 3:
            transform_output = T[0] * T[1] * T[2];
            break;
        case 4:
            transform_output = T[0] * T[1] * T[2] * T[3];
            break;
        case 5:
            transform_output = T[0] * T[1] * T[2] * T[3] * T[4];
            break;
        case 6:
            transform_output = T[0] * T[1] * T[2] * T[3] * T[4] * T[5];
            break;
        default:
            std::cout << "Kinematics模块:输入有误,请输入1到6的整数!"
                      << std::endl;
            break;
    }
    return transform_output;
}
