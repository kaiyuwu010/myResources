/*
本算法采用MDH参数表示机械臂结构，连杆i相对连杆i-1的变换矩阵表示如下：
|    cosθi           -sinθi             0            a(i-1)   |
|sinθicosα(i-1)   cosθicosα(i-1)    -sinα(i-1)    -disinα(i-1)|
|sinθisinα(i-1)   cosθisinα(i-1)     cosα(i-1)     dicosα(i-1)|
|      0                0               0               1     |
θi表示第i个关节的旋转角度，α(i-1)表示第i个关节z轴相对第i-1个关节z轴的扭转角，
a(i-1)表示连杆i-1的z轴到连杆i的z轴的公垂线长度，di表示沿连杆i的z轴从x(i-1)移动到xi的距离。
MDH参数示例，单位(mm、°)：
d[7] = {0,  x,  x,  x,  x,  x,  x}; MDH参数中d[0]一定为0，单位为mm
a[7] = {0,  x,  x,  x,  x,  x,  x}; MDH参数中a[0]一般为0，单位为mm
α[7] = {0,  x,  x,  x,  x,  x,  x}; MDH参数中α[0]一般为0，单位为度
θ[7] = {0,  x,  x,  x,  x,  x,  x}; MDH参数中θ[0]一定为0，单位为度
*/
#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <vector>
#include <array>

class ForwardKinematics
{
  private:
    // MDH参数
    std::array<double, 7> d_;
    std::array<double, 7> a_;
    std::array<double, 7> alpha_;
    std::array<double, 7> theta_;
    const double deg_to_rad = M_PI/180.0;

  public:
    // 参数为度表示的角度，mm表示的长度
    ForwardKinematics(std::array<double, 7> a, std::array<double, 7> alpha, std::array<double, 7> d, std::array<double, 7> theta)
    {
        for(int i=0; i<7; i++)
        {
            a_[i] = a[i];
            d_[i] = d[i];
            alpha_[i] = alpha[i] * deg_to_rad;
            theta_[i] = theta[i] * deg_to_rad;
        }
    }

    // 参数为弧度表示的角度，输出单位为mm（注意theta_input第一个值不用，从第1个值开始使用）
    Eigen::Matrix4d forwardKinematics(const std::vector<double>& theta_input, const int& index_joint)  
    {
        Eigen::Matrix4d T[7];
        for (int i = 1; i < 7; i++) 
        {
            T[i](0, 0) = cos(theta_input[i] + theta_[i]);
            T[i](0, 1) = -sin(theta_input[i] + theta_[i]);
            T[i](0, 2) = 0;
            T[i](0, 3) = a_[i-1];
            T[i](1, 0) = sin(theta_input[i] + theta_[i]) * cos(alpha_[i-1]);
            T[i](1, 1) = cos(theta_input[i] + theta_[i]) * cos(alpha_[i-1]);
            T[i](1, 2) = -sin(alpha_[i-1]);
            T[i](1, 3) = -sin(alpha_[i-1]) * d_[i];
            T[i](2, 0) = sin(theta_input[i] + theta_[i]) * sin(alpha_[i-1]);
            T[i](2, 1) = cos(theta_input[i] + theta_[i]) * sin(alpha_[i-1]);
            T[i](2, 2) = cos(alpha_[i-1]);
            T[i](2, 3) = cos(alpha_[i-1]) * d_[i];
            T[i](3, 0) = 0;
            T[i](3, 1) = 0;
            T[i](3, 2) = 0;
            T[i](3, 3) = 1;
        }
        Eigen::Matrix4d transform_output;
        switch (index_joint) 
        {
            case 1:
                transform_output = T[1];
                break;
            case 2:
                transform_output = T[1] * T[2];
                break;
            case 3:
                transform_output = T[1] * T[2] * T[3];
                break;
            case 4:
                transform_output = T[1] * T[2] * T[3] * T[4];
                break;
            case 5:
                transform_output = T[1] * T[2] * T[3] * T[4] * T[5];
                break;
            case 6:
                transform_output = T[1] * T[2] * T[3] * T[4] * T[5] * T[6];
                break;
            default:
                std::cout << "ForwardKinematics模块:输入有误,请输入1到6的整数!" << std::endl;
                break;
        }
        // std::cout <<"T[1]"<< std::endl<< T[1] << std::endl;
        // std::cout <<"T[2]"<< std::endl<< T[2] << std::endl;
        // std::cout <<"T[3]"<< std::endl<< T[3] << std::endl;
        // std::cout <<"T[4]"<< std::endl<< T[4] << std::endl;
        // std::cout <<"T[5]"<< std::endl<< T[5] << std::endl;
        // std::cout <<"T[6]"<< std::endl<< T[6] << std::endl;
        return transform_output;
    }
};

#endif