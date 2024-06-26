#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <vector>

#include "arm_planning/inverseKinematics.h"
#include "arm_planning/kinematics.h"

class CubicSplinePlanning
{
  public:
    //多维空间点
    struct multiDimensionPoint
    {
      protected:
        const static unsigned int num_dimension;

      public:
        double* point;
        multiDimensionPoint()  //默认构造
        {
            this->point = new double[num_dimension];
            for (int i = 0; i < num_dimension; i++) {
                this->point[i] = 0;
            }
        }
        multiDimensionPoint(const multiDimensionPoint& pt)  //拷贝构造
        {
            this->point = new double[num_dimension];
            for (int i = 0; i < num_dimension; i++) {
                this->point[i] = pt.point[i];
            }
        }
        multiDimensionPoint(multiDimensionPoint& pt)  //移动构造
        {
            this->point = new double[num_dimension];
            for (int i = 0; i < num_dimension; i++) {
                this->point[i] = pt.point[i];
            }
            pt.point = nullptr;
        }
        multiDimensionPoint(
            const std::initializer_list<double>& list)  //列表初始化
        {
            if (list.size() != num_dimension) {
                std::cout << "轨迹规划模块:输入的多维点参数维度不正确！"
                          << std::endl;
                exit(1);
            }
            this->point = new double[list.size()];
            int i = 0;
            for (auto element = list.begin(); element != list.end();
                 element++) {
                this->point[i] = *element;
                i++;
            }
        }
        ~multiDimensionPoint()
        {
            delete[] point;
            point = nullptr;
        }
        double& operator[](int num) const
        {
            return this->point[num];
        }
        const multiDimensionPoint& operator=(
            const multiDimensionPoint& pt) const
        {
            for (int i = 0; i < sizeof(point); i++) {
                this->point[i] = pt[i];
            }
            return *this;
        }
    };
    //带时间戳的单关节点
    struct pointWithTimeStamp
    {
        double time;
        double value;
    };
    //带时间戳的多关节点
    struct jointAnglesWithTimeStamp
    {
        double time;
        double joints_value[6];
    };
    //带时间戳的多关节速度点
    struct jointVelocitiesWithTimeStamp
    {
        double time;
        double joints_vel_value[6];
    };
    //带时间戳的多关节加速度点
    struct jointAccelerationsWithTimeStamp
    {
        double time;
        double joints_acc_value[6];
    };

  private:
    int input_points_num_ = -1;  //输入点的数量
    std::vector<multiDimensionPoint>
        points_for_planning_;  //要进行轨迹规划的路径点集NX6矩阵
    std::vector<std::vector<std::vector<double>>>
        cubic_spline_parameter_;  // a, b, c, d三次样条曲线参数.
    std::vector<std::vector<double>>
        quartic_spline_parameter_;  // a, b, c, d四次样条曲线参数.
    std::vector<std::vector<double>>
        quintic_spline_parameter0_;  // a, b, c, d五次样条第一条曲线参数.
    std::vector<std::vector<double>>
        quintic_spline_parameter1_;  // a, b, c, d五次样条第二条曲线参数.
    double rad_time_ratio_;          //多少弧度每秒
    std::vector<double> time_interval_;  //弧度时间比率
  public:
    std::vector<jointAnglesWithTimeStamp> pos_list_;  //输出的六轴位置序列
    std::vector<jointVelocitiesWithTimeStamp> vel_list_;  //输出的六轴速度序列
    std::vector<jointAccelerationsWithTimeStamp>
        acc_list_;  //输出的六轴加速度序列

    CubicSplinePlanning();
    CubicSplinePlanning(float rad_time_ratio);
    //计算多项式样条曲线参数
    bool initCubicSplineWithControlPoints(
        const std::vector<multiDimensionPoint>& points_input);
    //多关节采样
    void insertAllJointsCubicSpline(int step);
    //获取三次样条直线规划
    void getLineTrajectoryPlanning(const multiDimensionPoint* point_start,
                                   const multiDimensionPoint* point_end,
                                   int num_insert);
};
const unsigned int CubicSplinePlanning::multiDimensionPoint::num_dimension = 6;
