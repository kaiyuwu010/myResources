#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <vector>

class Kinematics
{
  private:
    // DH参数
    const double d[6] = {243.3, 0, 0, 227.6, 0, 61.5};
    const double a[6] = {0, 0, 200, 87, 0, 0};
    const double alpha[6] = {0, -90, 180, 90, 90, -90};

  public:
    Eigen::Matrix4d kinematics(const std::vector<double>& theta_input,
                               const int& index_joint);
};

#endif
