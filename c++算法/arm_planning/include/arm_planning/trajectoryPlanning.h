#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <vector>

class CubicSplinePlanning
{
private:
    //要进行插值的点的维度
    const unsigned int point_dimension_;

public:
    //多维空间点
    struct Point
    {
        std::vector<double> point_;
        Point(unsigned int point_dimension)  //普通构造函数
        {
            point_.assign(point_dimension, 0);
        }
        Point(const Point& pt)  //拷贝构造
        {
            point_.assign(pt.point_.begin(), pt.point_.end());
        }
        Point(const std::initializer_list<double> list)  //列表初始化
        {
            point_.assign(list);
        }
        Point(const std::vector<double>& vec)  //vector容器初始化
        {
            point_.assign(vec.begin(), vec.end());
        }
        const Point& operator=(const Point& pt)  //赋值运算符重载
        {
            point_.assign(pt.point_.begin(), pt.point_.end());
            return *this;
        }
        double& operator[](unsigned int num)  //小括号重载
        {
            return point_[num];
        }
        bool operator==(const Point& pt) const  //判断是否相等运算符重载
        {
            for (int i = 0; i < point_.size(); i++)
            {
                if (point_[i] != pt.point_[i])
                {
                    return false;
                }
            }
            return true;
        }
    };
    //带时间戳的坐标值
    struct valueWithTimeStamp
    {
        double time;
        double value;
    };
    //带时间戳的多维坐标值
    struct CoordinatesWithTimeStamp
    {
        double time;
        std::vector<double> coordinates;
        CoordinatesWithTimeStamp(unsigned int capacity)
        {
            coordinates.resize(capacity);
        }
    };
    //带时间戳的多维速度点
    struct VelocitiesWithTimeStamp
    {
        double time;
        std::vector<double> velocities;
        VelocitiesWithTimeStamp(unsigned int capacity)
        {
            velocities.resize(capacity);
        }
    };
    //带时间戳的多维加速度点
    struct AccelerationsWithTimeStamp
    {
        double time;
        std::vector<double> accelerations;
        AccelerationsWithTimeStamp(unsigned int capacity)
        {
            accelerations.resize(capacity);
        }
    };

private:
    std::vector<Point> points_;
    int input_points_num_ = -1;                                             //输入点的数量
    std::vector<Point> points_for_planning_;                                //要进行轨迹规划的路径点集NX6矩阵
    std::vector<double> v0_;                                                //初始速度
    std::vector<double> m0_;                                                //初始加速度
    std::vector<double> vn_;                                                //终止速度
    std::vector<double> mn_;                                                //终止加速度
    double rad_time_ratio_;                                                 //多少弧度每秒
    std::vector<double> time_interval_;                                     //时间间隔
    std::vector<std::vector<std::vector<double>>> cubic_spline_parameter_;  // a, b, c, d三次样条曲线参数.
    std::vector<std::vector<double>> quartic_spline_parameter_;             // a, b, c, d四次样条曲线参数.
    std::vector<std::vector<double>> quintic_spline_parameter0_;            // a, b, c, d五次样条第一条曲线参数.
    std::vector<std::vector<double>> quintic_spline_parameter1_;            // a, b, c, d五次样条第二条曲线参数.
public:
    std::vector<CoordinatesWithTimeStamp> pos_list_;    //输出的位置序列
    std::vector<VelocitiesWithTimeStamp> vel_list_;     //输出的速度序列
    std::vector<AccelerationsWithTimeStamp> acc_list_;  //输出的加速度序列
    //构造函数
    CubicSplinePlanning(unsigned int num_dimension);
    CubicSplinePlanning(unsigned int num_dimension, float rad_time_ratio);
    //设置函数
    void setAverageSpeed(float rad_time_ratio);
    void setEndpointsSpeedAndAcceleration(std::vector<double>& initial_speed, std::vector<double>& initial_acceleration, std::vector<double>& final_speed, std::vector<double>& final_acceleration);
    //添加路径点
    void addPoint(std::vector<double>& point);
    void addLinePoint(std::vector<double>& line_point, unsigned int num_insert = 1);
    //计算多项式样条曲线参数
    bool initCubicSplineWithControlPoints();
    //多关节采样
    void quartic_spline_Interpolation(double time_diff);
    void cubic_spline_Interpolation(double time_diff);
    void insertAllJointsCubicSpline(double time_diff);
};
