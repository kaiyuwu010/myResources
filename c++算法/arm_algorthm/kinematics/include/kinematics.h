#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <ctime>
#include <iostream>

class Kinematics
{
  private:
    // DH参数
    const double d[6] = {243.3, 0, 0, 227.6, 0, 61.5};
    const double a[6] = {0, 0, 200, 87, 0, 0};
    const double alpha[6] = {0, -90, 180, 90, 90, -90};
    // arm参数
    const double lenth_of_link2_ = 200;
    const double lenth_of_link3_ = 87;
    const double lenth_of_link4_ = 227.6;
    const double length_of_link34_ = 243.661158;
    const double square_of_link34_ = 59370.76;
    const double lenth_of_link6_ = 61.5;
    const double height_of_link1_ = 243.3;

  public:
    double theta[8 + 1][6 + 1];  //八组解，每组解六个角，第0个都不用
    Eigen::Matrix3d matrix_rotate_aroundx;
    Eigen::Matrix4d forwardKinematics(const std::vector<double>& theta_input, const int& index_joint);
    Kinematics();
    Eigen::Matrix3d getRotationMatrix(const double joint1, const double joint2, const double joint3);
    void inverseKinematics(const Eigen::Matrix4d& T);
    void caculateLastThreeJoint(const Eigen::Matrix4d& T, int index_Solustion);
};
#endif
