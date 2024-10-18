#include "arm_planning/trajectoryPlanning.h"

CubicSplinePlanning::CubicSplinePlanning(const unsigned int num_dimension) :
    point_dimension_(num_dimension)
{
    // rad_time_ratio_ = 0.6;
    rad_time_ratio_ = 30;
    v0_.assign(num_dimension, 0);  //初始速度
    m0_.assign(num_dimension, 0);  //初始加速度
    vn_.assign(num_dimension, 0);  //终止速度
    mn_.assign(num_dimension, 0);  //终止加速度
    std::cout << "轨迹规划模块:初始化轨迹规划模块！" << std::endl;
}
// rad_time_ratio表示相邻路径点间关节转过的角度(rad)与时间(s)的比例
CubicSplinePlanning::CubicSplinePlanning(unsigned int num_dimension, float rad_time_ratio) :
    point_dimension_(num_dimension)
{
    rad_time_ratio_ = rad_time_ratio;
    v0_.assign(num_dimension, 0);  //初始速度
    m0_.assign(num_dimension, 0);  //初始加速度
    vn_.assign(num_dimension, 0);  //终止速度
    mn_.assign(num_dimension, 0);  //终止加速度
    std::cout << "轨迹规划模块:初始化轨迹规划模块！" << std::endl;
}
// 单位rad/s
void CubicSplinePlanning::setAverageSpeed(float rad_time_ratio)
{
    rad_time_ratio_ = rad_time_ratio;
}
// 单位rad/s,rad/s^2,rad/s,rad/s^2
void CubicSplinePlanning::setEndpointsSpeedAndAcceleration(std::vector<double>& initial_speed, std::vector<double>& initial_acceleration, std::vector<double>& final_speed, std::vector<double>& final_acceleration)
{
    v0_ = initial_speed;
    m0_ = initial_acceleration;
    vn_ = final_speed;
    mn_ = final_acceleration;
}
void CubicSplinePlanning::addPoint(std::vector<double>& point)
{
    if (point.size() != point_dimension_)
    {
        std::cout << "轨迹规划模块:添加点维度与预定义维度不一致！" << std::endl;
        return;
    }
    points_.emplace_back(point);
}

bool CubicSplinePlanning::initCubicSplineWithControlPoints()
{
    std::cout.precision(4);
    input_points_num_ = points_.size();
    //检测路径中是否有连续相同点
    for (int i = 0; i < input_points_num_ - 1; i++)
    {
        if (points_[i] == points_[i + 1])
        {
            std::cout << "轨迹规划模块:有连续的两个路径点相同，请重新输入路径点！" << std::endl;
            return false;
        }
    }
    //给一个点返回
    if (input_points_num_ < 2)
    {
        std::cout << "轨迹规划模块:用于轨迹规划的点数不能小于2!" << std::endl;
        return false;
    }
    //给两个点用于规划时，采样五次多项式进行规划
    else if (input_points_num_ == 2)
    {
        std::cout << "轨迹规划模块:用于轨迹规划的点数为2, 采样五次多项式进行规划!" << std::endl;
        time_interval_.clear();
        time_interval_.resize(1, 0);
        double interval, x1, x2;
        //计算相邻点间的时间间隔
        for (int i = 0; i < point_dimension_; i++)
        {
            x1 = points_[0][i];
            x2 = points_[1][i];
            interval = abs((x2 - x1) / rad_time_ratio_);
            time_interval_[0] = std::max(time_interval_[0], interval);
        }
        //计算五次多项式参数
        quartic_spline_parameter_.resize(point_dimension_, std::vector<double>(6, 0));
        for (int i = 0; i < point_dimension_; i++)
        {
            double rad_interval = points_[1][i] - points_[0][i];
            quartic_spline_parameter_[i][0] = points_[0][i];                                                                                                                                                     // a0
            quartic_spline_parameter_[i][1] = v0_[i];                                                                                                                                                            // a1
            quartic_spline_parameter_[i][2] = 0.5 * m0_[i];                                                                                                                                                      // a2
            quartic_spline_parameter_[i][3] = 0.5 * (20 * rad_interval - (8 * vn_[i] + 12 * v0_[i]) * time_interval_[0] - (3 * m0_[i] - mn_[i]) * pow(time_interval_[0], 2)) / pow(time_interval_[0], 3);        // a3
            quartic_spline_parameter_[i][4] = 0.5 * (-30 * rad_interval + (14 * vn_[i] + 16 * v0_[i]) * time_interval_[0] + (3 * m0_[i] - 2 * mn_[i]) * pow(time_interval_[0], 2)) / pow(time_interval_[0], 4);  // a4
            quartic_spline_parameter_[i][5] = 0.5 * (12 * rad_interval - 6 * (vn_[i] + v0_[i]) * time_interval_[0] + (mn_[i] - m0_[i]) * pow(time_interval_[0], 2)) / pow(time_interval_[0], 5);                 // a5
        }
        return true;
    }
    //给三个点及以上用于规划时，采用三次多项式进行规划
    else
    {
        std::cout << "轨迹规划模块:用于轨迹规划的点数大于等于3, 采样三次多项式进行规划!" << std::endl;
        int num_of_points = points_.size();
        std::vector<double> vec(point_dimension_, 0);
        CubicSplinePlanning::Point control_point(vec);
        //插入控制点
        for (int i = 0; i < num_of_points; i++)
        {
            if (i == 1 || i == (num_of_points - 1))
            {
                points_for_planning_.push_back(control_point);
            }
            points_for_planning_.push_back(points_[i]);
        }
        num_of_points = points_for_planning_.size();
        // std::cout<<"轨迹规划模块:加上控制点后的点----------数量:
        // "<<num_of_points<<std::endl; for(int i=0;
        // i<points_for_planning_.size(); i++)
        // {
        // 	for(int j=0; j<6; j++)
        // 	{
        // 		std::cout<<points_for_planning_[i].point[j]<<" ";
        // 	}
        // 	std::cout<<std::endl;
        // }
        std::vector<Eigen::MatrixXf> A(point_dimension_, Eigen::MatrixXf::Zero(num_of_points - 2, num_of_points - 2));
        std::vector<Eigen::MatrixXf> B(point_dimension_, Eigen::MatrixXf::Zero(num_of_points - 2, 1));
        std::vector<Eigen::MatrixXf> W(point_dimension_, Eigen::MatrixXf::Zero(num_of_points - 2, 1));  //待求的加速度向量
        time_interval_.clear();
        time_interval_.resize(num_of_points - 1, 0);  //加入控制点后，n个点对应n-1个时间间隔
        //确定两点间的时间间隔
        double interval, x1, x2;
        for (int i = 0; i < num_of_points - 1; i++)
        {
            if (i == 0 || i == 1)  //前两个由控制点分割的时间间隔，设置为相等
            {
                for (int j = 0; j < point_dimension_; j++)
                {
                    interval = abs((points_for_planning_[2][j] - points_for_planning_[0][j]) / rad_time_ratio_);
                    time_interval_[i] = std::max(time_interval_[i], interval / 2);
                }
            }
            else if (i == num_of_points - 3 || i == num_of_points - 2)  //后两个由控制点分割的时间间隔，设置为相等
            {
                for (int j = 0; j < point_dimension_; j++)
                {
                    interval = abs((points_for_planning_[num_of_points - 1][j] - points_for_planning_[num_of_points - 3][j]) / rad_time_ratio_);
                    time_interval_[i] = std::max(time_interval_[i], interval / 2);
                }
            }
            else
            {
                for (int j = 0; j < point_dimension_; j++)
                {
                    x1 = points_for_planning_[i][j];
                    x2 = points_for_planning_[i + 1][j];
                    interval = abs((x2 - x1) / rad_time_ratio_);
                    time_interval_[i] = std::max(time_interval_[i], interval);
                }
            }
        }
        //根据各个路径点的值计算矩阵方程各元素
        for (int i = 0; i < num_of_points - 2; i++)
        {
            if (i == 0)
            {
                for (int j = 0; j < point_dimension_; j++)
                {
                    A[j](0, 0) = 2 * time_interval_[1] + time_interval_[0] * (3 + time_interval_[0] / time_interval_[1]);
                    A[j](0, 1) = time_interval_[1];
                    B[j](i, 0) = 6 * ((points_for_planning_[2][j] - points_for_planning_[0][j]) / time_interval_[1] - (time_interval_[0] / time_interval_[1] + 1) * v0_[j] - (0.5 * time_interval_[0] + 0.3333 * std::pow(time_interval_[0], 2) / time_interval_[1]) * m0_[j]);
                }
            }
            else if (i == 1)
            {
                for (int j = 0; j < point_dimension_; j++)
                {
                    A[j](1, 0) = time_interval_[1] - std::pow(time_interval_[0], 2) / time_interval_[1];
                    A[j](1, 1) = 2 * (time_interval_[1] + time_interval_[2]);
                    A[j](1, 2) = time_interval_[2];
                    B[j](i, 0) = 6 * ((points_for_planning_[3][j] - points_for_planning_[2][j]) / time_interval_[2] - (points_for_planning_[2][j] - points_for_planning_[0][j]) / time_interval_[1] + (time_interval_[0] / time_interval_[1]) * v0_[j] - 0.3333 * (std::pow(time_interval_[0], 2) / time_interval_[1]) * m0_[j]);
                }
            }
            else if (i == num_of_points - 4)
            {
                for (int j = 0; j < point_dimension_; j++)
                {
                    A[j](num_of_points - 4, num_of_points - 5) = time_interval_[num_of_points - 4];
                    A[j](num_of_points - 4, num_of_points - 4) = 2 * (time_interval_[num_of_points - 4] + time_interval_[num_of_points - 3]);
                    A[j](num_of_points - 4, num_of_points - 3) = time_interval_[num_of_points - 3] - std::pow(time_interval_[num_of_points - 2], 2) / time_interval_[num_of_points - 3];
                    B[j](i, 0) = 6 * ((points_for_planning_[num_of_points - 1][j] - points_for_planning_[num_of_points - 3][j]) / time_interval_[num_of_points - 3] -
                                      (points_for_planning_[num_of_points - 3][j] - points_for_planning_[num_of_points - 4][j]) / time_interval_[num_of_points - 4] -
                                      (time_interval_[num_of_points - 2] / time_interval_[num_of_points - 3]) * vn_[j] +
                                      1 / 3 * (std::pow(time_interval_[num_of_points - 2], 2) / time_interval_[num_of_points - 3]) * mn_[j]);
                }
            }
            else if (i == num_of_points - 3)
            {
                for (int j = 0; j < point_dimension_; j++)
                {
                    A[j](num_of_points - 3, num_of_points - 4) = time_interval_[num_of_points - 3];
                    A[j](num_of_points - 3, num_of_points - 3) = 2 * time_interval_[num_of_points - 3] + time_interval_[num_of_points - 2] * (3 + time_interval_[num_of_points - 2] / time_interval_[num_of_points - 3]);
                    B[j](i, 0) = 6 * ((points_for_planning_[num_of_points - 3][j] - points_for_planning_[num_of_points - 1][j]) / time_interval_[num_of_points - 3] +
                                      (1 + time_interval_[num_of_points - 2] / time_interval_[num_of_points - 3]) * vn_[j] -
                                      (0.5 * time_interval_[num_of_points - 2] + 0.3333 * (std::pow(time_interval_[num_of_points - 2], 2) / time_interval_[num_of_points - 3])) * mn_[j]);
                }
            }
            else
            {
                for (int j = 0; j < point_dimension_; j++)
                {
                    A[j](i, i) = 2 * (time_interval_[i] + time_interval_[i + 1]);
                    A[j](i, i - 1) = time_interval_[i];
                    A[j](i, i + 1) = time_interval_[i + 1];
                    B[j](i, 0) = 6 * ((points_for_planning_[i + 2][j] - points_for_planning_[i + 1][j]) / time_interval_[i + 1] - (points_for_planning_[i + 1][j] - points_for_planning_[i][j]) / time_interval_[i]);
                }
            }
        }
        for (int j = 0; j < point_dimension_; j++)
        {
            W[j] = A[j].partialPivLu().solve(B[j]);
        }
        //加速度矩阵m转化为向量
        std::vector<std::vector<double>> vector_acceleration(num_of_points - 2, std::vector<double>(point_dimension_, 0));  // vector_acceleration(n-2)X6
        for (int i = 0; i < num_of_points - 2; i++)
        {
            for (int j = 0; j < point_dimension_; j++)
            {
                vector_acceleration[i][j] = W[j](i, 0);  // vector_acceleration是N X 6
            }
        }
        //把控制点的值插入points_for_planning_
        for (int i = 0; i < point_dimension_; i++)
        {
            points_for_planning_[1][i] = points_for_planning_[0][i] + v0_[i] * time_interval_[0] +
                                         (std::pow(time_interval_[0], 2)) * m0_[i] / 2 +
                                         (std::pow(time_interval_[0], 2)) * (vector_acceleration[0][i] - m0_[i]) / 6;
            points_for_planning_[num_of_points - 2][i] = points_for_planning_[num_of_points - 1][i] -
                                                         time_interval_[num_of_points - 2] * vn_[i] +
                                                         (2 * mn_[i] + vector_acceleration[num_of_points - 3][i]) / 6 * std::pow(time_interval_[num_of_points - 2], 2);
        }
        std::cout << "轨迹规划模块:带控制点的点集合:" << std::endl;
        for (int i = 0; i < points_for_planning_.size(); i++)
        {
            for (size_t j = 0; j < point_dimension_; j++)
            {
                std::cout << points_for_planning_[i][j] << " ";
            }
            std::cout << std::endl;
        }
        std::cout << "---------------------------" << std::endl;
        //加速度向量插入收尾值
        vector_acceleration.insert(vector_acceleration.begin(), m0_);
        vector_acceleration.insert(vector_acceleration.end(), mn_);
        // std::cout << "轨迹规划模块:各个关节的加速度值:" << std::endl;
        // for (int i = 0; i < vector_acceleration.size(); i++)
        // {
        //     std::cout << vector_acceleration[i][0] << ", " << vector_acceleration[i][1] << ", " << vector_acceleration[i][2] << ", "
        //               << vector_acceleration[i][3] << ", " << vector_acceleration[i][4] << ", " << vector_acceleration[i][5] << ", " << std::endl;
        // }
        cubic_spline_parameter_.clear();
        cubic_spline_parameter_.resize(point_dimension_, std::vector<std::vector<double>>(num_of_points - 1, std::vector<double>(4, 0)));  // cubic_spline_parameter_是6 X N X 4矩阵
        double a, b, c, d;
        //计算三次样条曲线参数
        for (int i = 0; i < num_of_points - 1; i++)
        {
            for (int j = 0; j < point_dimension_; j++)
            {
                a = points_for_planning_[i][j];
                b = (points_for_planning_[i + 1][j] - points_for_planning_[i][j]) / time_interval_[i] - (time_interval_[i] / 6) * (vector_acceleration[i + 1][j] + 2 * vector_acceleration[i][j]);
                c = vector_acceleration[i][j] / 2;
                d = (vector_acceleration[i + 1][j] - vector_acceleration[i][j]) / (6 * time_interval_[i]);
                std::vector<double> cubic_spline_parameter_element;
                cubic_spline_parameter_element.push_back(a);
                cubic_spline_parameter_element.push_back(b);
                cubic_spline_parameter_element.push_back(c);
                cubic_spline_parameter_element.push_back(d);
                cubic_spline_parameter_[j][i] = cubic_spline_parameter_element;  // cubic_spline_parameter_是一个 6 X N 矩阵
            }
        }
        // std::cout<<"轨迹规划模块:第一个关节的插入曲线的参数:"<<std::endl;
        // for(int i=0; i<cubic_spline_parameter_[0].size(); i++)
        // {
        // 	std::cout<<cubic_spline_parameter_[0][i][0]<<"
        // "<<cubic_spline_parameter_[0][i][1]<<"
        // "<<cubic_spline_parameter_[0][i][2]<<"
        // "<<cubic_spline_parameter_[0][i][3]<<std::endl;
        // }
        return true;
    }
}
void CubicSplinePlanning::quartic_spline_Interpolation(double time_diff)
{
    CoordinatesWithTimeStamp angels_value_with_time(point_dimension_);
    VelocitiesWithTimeStamp vels_value_with_time(point_dimension_);
    AccelerationsWithTimeStamp accs_value_with_time(point_dimension_);
    double time = 0;
    double remainder = std::fmod(time_interval_[0], time_diff);
    double steps_size = std::floor(time_interval_[0] / time_diff);
    for (int i = 0; i < steps_size + 1; i++)
    {
        angels_value_with_time.time = time;
        vels_value_with_time.time = time;
        accs_value_with_time.time = time;
        for (int j = 0; j < point_dimension_; j++)
        {
            //位置采样
            double pos_each_step = quartic_spline_parameter_[j][0] + quartic_spline_parameter_[j][1] * time + quartic_spline_parameter_[j][2] * pow(time, 2) + quartic_spline_parameter_[j][3] * pow(time, 3) + quartic_spline_parameter_[j][4] * pow(time, 4) + quartic_spline_parameter_[j][5] * pow(time, 5);
            angels_value_with_time.coordinates[j] = pos_each_step;
            //速度采样
            double vel_each_step = quartic_spline_parameter_[j][1] + 2 * quartic_spline_parameter_[j][2] * time + 3 * quartic_spline_parameter_[j][3] * pow((time), 2) + 4 * quartic_spline_parameter_[j][4] * pow((time), 3) + 5 * quartic_spline_parameter_[j][5] * pow((time), 4);
            vels_value_with_time.velocities[j] = vel_each_step;
            //加速度采样
            double acc_each_step = 2 * quartic_spline_parameter_[j][2] + 6 * quartic_spline_parameter_[j][3] * time + 12 * quartic_spline_parameter_[j][4] * pow((time), 2) + 20 * quartic_spline_parameter_[j][5] * pow((time), 3);
            accs_value_with_time.accelerations[j] = acc_each_step;
        }
        pos_list_.push_back(angels_value_with_time);
        vel_list_.push_back(vels_value_with_time);
        acc_list_.push_back(accs_value_with_time);
        time += time_diff;
    }
    time = time_interval_[0];
    angels_value_with_time.time = time;
    vels_value_with_time.time = time;
    accs_value_with_time.time = time;
    for (int j = 0; j < point_dimension_; j++)
    {
        //位置采样
        double pos_each_step = quartic_spline_parameter_[j][0] + quartic_spline_parameter_[j][1] * time + quartic_spline_parameter_[j][2] * pow(time, 2) + quartic_spline_parameter_[j][3] * pow(time, 3) + quartic_spline_parameter_[j][4] * pow(time, 4) + quartic_spline_parameter_[j][5] * pow(time, 5);
        angels_value_with_time.coordinates[j] = pos_each_step;
        //速度采样
        double vel_each_step = quartic_spline_parameter_[j][1] + 2 * quartic_spline_parameter_[j][2] * time + 3 * quartic_spline_parameter_[j][3] * pow((time), 2) + 4 * quartic_spline_parameter_[j][4] * pow((time), 3) + 5 * quartic_spline_parameter_[j][5] * pow((time), 4);
        vels_value_with_time.velocities[j] = vel_each_step;
        //加速度采样
        double acc_each_step = 2 * quartic_spline_parameter_[j][2] + 6 * quartic_spline_parameter_[j][3] * time + 12 * quartic_spline_parameter_[j][4] * pow((time), 2) + 20 * quartic_spline_parameter_[j][5] * pow((time), 3);
        accs_value_with_time.accelerations[j] = acc_each_step;
    }
    if (remainder > 0.5 * time_diff || time_interval_[0] < time_diff)
    {
        pos_list_.push_back(angels_value_with_time);
        vel_list_.push_back(vels_value_with_time);
        acc_list_.push_back(accs_value_with_time);
    }
    else
    {
        pos_list_.pop_back();
        vel_list_.pop_back();
        acc_list_.pop_back();
        pos_list_.push_back(angels_value_with_time);
        vel_list_.push_back(vels_value_with_time);
        acc_list_.push_back(accs_value_with_time);
    }
    quartic_spline_parameter_.clear();
    return;
}

void CubicSplinePlanning::cubic_spline_Interpolation(double time_diff)
{
    CoordinatesWithTimeStamp angels_value_with_time(point_dimension_);
    VelocitiesWithTimeStamp vels_value_with_time(point_dimension_);
    AccelerationsWithTimeStamp accs_value_with_time(point_dimension_);
    //先插入第一个点
    angels_value_with_time.time = 0;
    vels_value_with_time.time = 0;
    accs_value_with_time.time = 0;
    for (int k = 0; k < point_dimension_; k++)
    {
        //位置采样
        double pos_each_step = cubic_spline_parameter_[k][0][0];
        angels_value_with_time.coordinates[k] = pos_each_step;
        //速度采样
        double vel_each_step = cubic_spline_parameter_[k][0][1];
        vels_value_with_time.velocities[k] = vel_each_step;
        //加速度采样
        double acc_each_step = 2 * cubic_spline_parameter_[k][0][2];
        accs_value_with_time.accelerations[k] = acc_each_step;
    }
    pos_list_.push_back(angels_value_with_time);
    vel_list_.push_back(vels_value_with_time);
    acc_list_.push_back(accs_value_with_time);
    double time = time_diff;
    double time_segment = 0;
    for (int i = 0; i < cubic_spline_parameter_[0].size(); i++)
    {
        double time_each_segment = time - time_segment;
        double steps = std::floor((time_segment + time_interval_[i]) / time_diff) - std::floor(time_segment / time_diff);
        std::cout << "time_each_segment: " << time_each_segment << std::endl;
        for (int j = 0; j < steps; j++)
        {
            angels_value_with_time.time = time;
            vels_value_with_time.time = time;
            accs_value_with_time.time = time;
            for (int k = 0; k < point_dimension_; k++)
            {
                //位置采样
                double pos_each_step = cubic_spline_parameter_[k][i][0] + cubic_spline_parameter_[k][i][1] * time_each_segment + cubic_spline_parameter_[k][i][2] * pow(time_each_segment, 2) + cubic_spline_parameter_[k][i][3] * pow(time_each_segment, 3);
                angels_value_with_time.coordinates[k] = pos_each_step;
                //速度采样
                double vel_each_step = cubic_spline_parameter_[k][i][1] + 2 * cubic_spline_parameter_[k][i][2] * time_each_segment + 3 * cubic_spline_parameter_[k][i][3] * pow(time_each_segment, 2);
                vels_value_with_time.velocities[k] = vel_each_step;
                //加速度采样
                double acc_each_step = 2 * cubic_spline_parameter_[k][i][2] + 6 * cubic_spline_parameter_[k][i][3] * time_each_segment;
                accs_value_with_time.accelerations[k] = acc_each_step;
            }
            pos_list_.push_back(angels_value_with_time);
            vel_list_.push_back(vels_value_with_time);
            acc_list_.push_back(accs_value_with_time);
            time_each_segment += time_diff;
            time += time_diff;
        }
        time_segment += time_interval_[i];
    }
    angels_value_with_time.time = time_segment;
    vels_value_with_time.time = time_segment;
    accs_value_with_time.time = time_segment;
    for (int k = 0; k < point_dimension_; k++)
    {
        //位置采样
        double pos_each_step = cubic_spline_parameter_[k][cubic_spline_parameter_[0].size() - 1][0] + cubic_spline_parameter_[k][cubic_spline_parameter_[0].size() - 1][1] * time_interval_.back() + cubic_spline_parameter_[k][cubic_spline_parameter_[0].size() - 1][2] * pow(time_interval_.back(), 2) + cubic_spline_parameter_[k][cubic_spline_parameter_[0].size() - 1][3] * pow(time_interval_.back(), 3);
        angels_value_with_time.coordinates[k] = pos_each_step;
        //速度采样
        double vel_each_step = cubic_spline_parameter_[k][cubic_spline_parameter_[0].size() - 1][1] + 2 * cubic_spline_parameter_[k][cubic_spline_parameter_[0].size() - 1][2] * time_interval_.back() + 3 * cubic_spline_parameter_[k][cubic_spline_parameter_[0].size() - 1][3] * pow(time_interval_.back(), 2);
        vels_value_with_time.velocities[k] = vel_each_step;
        //加速度采样
        double acc_each_step = 2 * cubic_spline_parameter_[k][cubic_spline_parameter_[0].size() - 1][2] + 6 * cubic_spline_parameter_[k][cubic_spline_parameter_[0].size() - 1][3] * time_interval_.back();
        accs_value_with_time.accelerations[k] = acc_each_step;
    }
    double remainder = std::fmod(time_segment, time_diff);
    std::cout << "remainder: " << remainder << std::endl;
    if (remainder > 0.5 * time_diff)
    {
        std::cout << "remainder > 0.5 * time_diff: " << std::endl;
        pos_list_.push_back(angels_value_with_time);
        vel_list_.push_back(vels_value_with_time);
        acc_list_.push_back(accs_value_with_time);
    }
    else
    {
        pos_list_.pop_back();
        vel_list_.pop_back();
        acc_list_.pop_back();
        pos_list_.push_back(angels_value_with_time);
        vel_list_.push_back(vels_value_with_time);
        acc_list_.push_back(accs_value_with_time);
    }
    cubic_spline_parameter_.clear();
    return;
}
//多关节采样
void CubicSplinePlanning::insertAllJointsCubicSpline(double time_diff)  //返回6XN矩阵, time_diff表示采样点的时间间隔
{
    std::vector<std::vector<valueWithTimeStamp>> points_list_all_joints;  // 6XN矩阵
    pos_list_.clear();
    vel_list_.clear();
    acc_list_.clear();
    //输入点为2进行五次多项式采样
    if (input_points_num_ == 2)
    {
        quartic_spline_Interpolation(time_diff);
    }
    //输入3个及以上点进行三次多项式采样
    else
    {
        cubic_spline_Interpolation(time_diff);
    }
    for (int k = 0; k < time_interval_.size(); k++)  //插入最后时刻的目标点
    {
        std::cout << "轨迹规划模块:time_interval_: " << time_interval_[k] << std::endl;
    }
    points_.clear();
    points_for_planning_.clear();
    time_interval_.clear();
    std::cout << "轨迹规划模块:多关节采样完成！" << std::endl;
    return;
}
// 添加直线点
void CubicSplinePlanning::addLinePoint(std::vector<double>& line_point, unsigned int num_insert)
{
    if (line_point.size() != point_dimension_ || num_insert < 1)
    {
        std::cout << "轨迹规划模块:添加点维度与预定义维度不一致,或插入点数小于1!" << std::endl;
        return;
    }
    if (points_.size() < 1)
    {
        std::cout << "轨迹规划模块:第一个路径点不能为直线点!" << std::endl;
        return;
    }
    else
    {
        std::cout << "轨迹规划模块:开始直线规划!" << std::endl;
        std::cout << "轨迹规划模块:直线点为: " << line_point[0] << " " << line_point[1] << " " << line_point[2] << " " << line_point[3] << " " << line_point[4] << " " << line_point[5] << " " << std::endl;
        std::vector<double> point_joints(point_dimension_, 0);
        std::vector<double> direction(point_dimension_, 0);
        for (int i = 0; i < point_dimension_; i++)
        {
            direction[i] = (line_point[i] - points_.back()[i]) / (num_insert + 1);
        }
        for (int i = 0; i < num_insert + 1; i++)
        {
            for (int j = 0; j < point_dimension_; j++)
            {
                point_joints[j] = points_.back()[j] + direction[j];
            }
            points_.emplace_back(point_joints);
        }
        for (int i = 0; i < points_.size(); i++)
        {
            std::cout << "轨迹规划模块:直线点: " << points_[i][0] << " " << points_[i][1] << " " << points_[i][2] << " " << points_[i][3] << " " << points_[i][4] << " " << points_[i][5] << " " << std::endl;
        }
    }
}
