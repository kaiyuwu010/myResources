#include "arm_planning/trajectoryPlanning.h"

CubicSplinePlanning::CubicSplinePlanning()
{
    rad_time_ratio_ = 0.8;
    std::cout << "轨迹规划模块:初始化轨迹规划模块！" << std::endl;
}
CubicSplinePlanning::CubicSplinePlanning(
    float
        rad_time_ratio)  // rad_time_ratio表示相邻路径点间关节转过的弧度与时间的比例
{
    rad_time_ratio_ = rad_time_ratio;
    std::cout << "轨迹规划模块:初始化轨迹规划模块！" << std::endl;
}

bool CubicSplinePlanning::initCubicSplineWithControlPoints(
    const std::vector<CubicSplinePlanning::multiDimensionPoint>&
        points_input)  // points_input为 N X 6 矩阵
{
    std::cout.precision(4);
    input_points_num_ = points_input.size();
    //检测路径中是否有连续相同点
    for (int i = 0; i < input_points_num_ - 1; i++) {
        int num_equal_joint = 0;
        for (int j = 0; j < 6; j++) {
            if (points_input[i][j] == points_input[i + 1][j]) {
                num_equal_joint++;
            }
        }
        if (num_equal_joint == 6) {
            std::cout
                << "轨迹规划模块:有连续的两个路径点相同，请重新输入路径点！"
                << std::endl;
            return false;
        }
    }
    //给一个点返回
    if (input_points_num_ < 2) {
        std::cout << "轨迹规划模块:用于轨迹规划的点数不能小于2!" << std::endl;
        return false;
    }
    //给两个点用于规划时，采样五次多项式进行规划
    else if (input_points_num_ == 2) {
        std::cout
            << "轨迹规划模块:用于轨迹规划的点数为2, 采样五次多项式进行规划!"
            << std::endl;
        time_interval_.clear();
        time_interval_.resize(input_points_num_ - 1, 0);
        double interval, x1, x2;
        for (int i = 0; i < 6; i++) {
            x1 = points_input[0][i];
            x2 = points_input[1][i];
            interval = abs((x2 - x1) / rad_time_ratio_);
            time_interval_[0] = std::max(time_interval_[0], interval);
        }
        //计算五次多项式参数
        quartic_spline_parameter_.resize(
            6,
            std::vector<double>(
                4, 0));  // 6 X 4五次多项式参数矩阵，a1、a2为0所以参数只有4个
        for (int i = 0; i < 6; i++) {
            double rad_interval = points_input[1][i] - points_input[0][i];
            quartic_spline_parameter_[i][0] = points_input[0][i];  // a0
            quartic_spline_parameter_[i][1] =
                10 * (rad_interval) / pow(time_interval_[0], 3);  // a3
            quartic_spline_parameter_[i][2] =
                -15 * (rad_interval) / pow(time_interval_[0], 4);  // a4
            quartic_spline_parameter_[i][3] =
                6 * (rad_interval) / pow(time_interval_[0], 5);  // a5
        }
        return true;
    }
    //给三个点用于规划时，采用四次多项式进行规划
    else if (input_points_num_ == 3) {
        std::cout
            << "轨迹规划模块:用于轨迹规划的点数为3, 采样四次多项式进行规划!"
            << std::endl;
        time_interval_.clear();
        time_interval_.resize(input_points_num_ - 1, 0);
        double interval, x1, x2;
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 6; j++) {
                x1 = points_input[i][j];
                x2 = points_input[i + 1][j];
                interval = abs((x2 - x1) / rad_time_ratio_);
                time_interval_[i] = std::max(time_interval_[i], interval);
            }
        }
        //计算四次多项式参数
        quintic_spline_parameter0_.resize(
            6,
            std::vector<double>(5,
                                0));  // 6X5五次多项式参数矩阵，第一条曲线参数
        quintic_spline_parameter1_.resize(
            6,
            std::vector<double>(5,
                                0));  // 6X5五次多项式参数矩阵，第二条曲线参数
        for (int i = 0; i < 6; i++)  //先求第二条曲线参数
        {
            double rad_interval_0 = points_input[1][i] - points_input[0][i];
            double rad_interval_1 = points_input[2][i] - points_input[1][i];
            quintic_spline_parameter1_[i][0] = points_input[1][i];
            quintic_spline_parameter1_[i][1] =
                2 * (rad_interval_1 / time_interval_[1] -
                     rad_interval_1 / (time_interval_[0] + time_interval_[1]) +
                     time_interval_[1] * rad_interval_0 /
                         (time_interval_[0] *
                          (time_interval_[0] + time_interval_[1])));
            quintic_spline_parameter1_[i][2] =
                6 *
                (rad_interval_1 / (time_interval_[1] *
                                   (time_interval_[0] + time_interval_[1])) -
                 rad_interval_0 / (time_interval_[0] *
                                   (time_interval_[0] + time_interval_[1])));
            quintic_spline_parameter1_[i][3] =
                -2 * rad_interval_1 / pow(time_interval_[1], 3) -
                quintic_spline_parameter1_[i][2] / time_interval_[1];
            quintic_spline_parameter1_[i][4] =
                rad_interval_1 / pow(time_interval_[1], 4) +
                quintic_spline_parameter1_[i][2] /
                    (3 * pow(time_interval_[1], 2));
        }
        for (int i = 0; i < 6; i++) {
            double rad_interval_0 = points_input[1][i] - points_input[0][i];
            double rad_interval_1 = points_input[2][i] - points_input[1][i];
            quintic_spline_parameter0_[i][0] = points_input[0][i];
            quintic_spline_parameter0_[i][1] = 0;
            quintic_spline_parameter0_[i][2] = 0;
            quintic_spline_parameter0_[i][3] =
                (3 * quintic_spline_parameter1_[i][1] -
                 2 * quintic_spline_parameter1_[i][2] * time_interval_[0]) /
                (3 * pow(time_interval_[0], 2));
            quintic_spline_parameter0_[i][4] =
                (quintic_spline_parameter1_[i][2] * time_interval_[0] -
                 quintic_spline_parameter1_[i][1]) /
                (2 * pow(time_interval_[0], 3));
        }
        return true;
    }
    //给四个点及以上用于规划时，采用三次多项式进行规划
    else {
        std::cout
            << "轨迹规划模块:用于轨迹规划的点数大于3, 采样三次多项式进行规划!"
            << std::endl;
        int num_of_points = points_input.size();
        CubicSplinePlanning::multiDimensionPoint control_point{
            0, 0, 0, 0, 0, 0};
        //插入控制点
        for (int i = 0; i < num_of_points; i++) {
            if (i == 1 || i == (num_of_points - 1)) {
                points_for_planning_.push_back(control_point);
            }
            points_for_planning_.push_back(points_input[i]);
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
        std::vector<Eigen::MatrixXf> A(
            6, Eigen::MatrixXf::Zero(num_of_points - 2, num_of_points - 2));
        std::vector<Eigen::MatrixXf> B(
            6, Eigen::MatrixXf::Zero(num_of_points - 2, 1));
        std::vector<Eigen::MatrixXf> W(
            6, Eigen::MatrixXf::Zero(num_of_points - 2, 1));  //待求的加速度向量
        time_interval_.clear();
        time_interval_.resize(num_of_points - 1,
                              0);  //加入控制点后，n个点对应n-1个时间间隔
        //根据两点角度差值确定规划时间间隔
        double interval, x1, x2;
        for (int i = 0; i < num_of_points - 1; i++) {
            if (i == 0 || i == 1)  //前两个由控制点分割的时间间隔，设置为相等
            {
                for (int j = 0; j < 6; j++) {
                    interval = abs((points_for_planning_[2][j] -
                                    points_for_planning_[0][j]) /
                                   rad_time_ratio_);
                    time_interval_[i] =
                        std::max(time_interval_[i], interval / 2);
                }
            } else if (i == num_of_points - 3 ||
                       i == num_of_points -
                                2)  //后两个由控制点分割的时间间隔，设置为相等
            {
                for (int j = 0; j < 6; j++) {
                    interval =
                        abs((points_for_planning_[num_of_points - 1][j] -
                             points_for_planning_[num_of_points - 3][j]) /
                            rad_time_ratio_);
                    time_interval_[i] =
                        std::max(time_interval_[i], interval / 2);
                }
            } else {
                for (int j = 0; j < 6; j++) {
                    x1 = points_for_planning_[i][j];
                    x2 = points_for_planning_[i + 1][j];
                    interval = abs((x2 - x1) / rad_time_ratio_);
                    time_interval_[i] = std::max(time_interval_[i], interval);
                }
            }
        }
        //根据各个路径点的值计算方程组右边的列向量
        for (int i = 0; i < num_of_points - 2; i++) {
            if (i == 0) {
                for (int j = 0; j < 6; j++) {
                    A[j](0, 0) =
                        2 * time_interval_[1] +
                        time_interval_[0] *
                            (3 + time_interval_[0] / time_interval_[1]);
                    A[j](0, 1) = time_interval_[1];
                    B[j](0, 0) = 6 * ((points_for_planning_[2][j] -
                                       points_for_planning_[0][j]) /
                                      time_interval_[1]);
                }
            } else if (i == 1) {
                for (int j = 0; j < 6; j++) {
                    A[j](1, 0) =
                        time_interval_[1] -
                        std::pow(time_interval_[0], 2) / time_interval_[1];
                    A[j](1, 1) = 2 * (time_interval_[1] + time_interval_[2]);
                    A[j](1, 2) = time_interval_[2];
                    B[j](i, 0) = 6 * ((points_for_planning_[3][j] -
                                       points_for_planning_[2][j]) /
                                          time_interval_[2] -
                                      (points_for_planning_[2][j] -
                                       points_for_planning_[0][j]) /
                                          time_interval_[1]);
                }
            } else if (i == num_of_points - 4) {
                for (int j = 0; j < 6; j++) {
                    A[j](num_of_points - 4, num_of_points - 5) =
                        time_interval_[num_of_points - 4];
                    A[j](num_of_points - 4, num_of_points - 4) =
                        2 * (time_interval_[num_of_points - 4] +
                             time_interval_[num_of_points - 3]);
                    A[j](num_of_points - 4, num_of_points - 3) =
                        time_interval_[num_of_points - 3] -
                        std::pow(time_interval_[num_of_points - 2], 2) /
                            time_interval_[num_of_points - 3];
                    B[j](i, 0) =
                        6 * ((points_for_planning_[num_of_points - 1][j] -
                              points_for_planning_[num_of_points - 3][j]) /
                                 time_interval_[num_of_points - 3] -
                             (points_for_planning_[num_of_points - 3][j] -
                              points_for_planning_[num_of_points - 4][j]) /
                                 time_interval_[num_of_points - 4]);
                }
            } else if (i == num_of_points - 3) {
                for (int j = 0; j < 6; j++) {
                    A[j](num_of_points - 3, num_of_points - 4) =
                        time_interval_[num_of_points - 3];
                    A[j](num_of_points - 3, num_of_points - 3) =
                        2 * time_interval_[num_of_points - 3] +
                        time_interval_[num_of_points - 2] *
                            (3 + time_interval_[num_of_points - 2] /
                                     time_interval_[num_of_points - 3]);
                    B[j](i, 0) =
                        6 * ((points_for_planning_[num_of_points - 3][j] -
                              points_for_planning_[num_of_points - 1][j]) /
                             time_interval_[num_of_points - 3]);
                }
            } else {
                for (int j = 0; j < 6; j++) {
                    A[j](i, i) =
                        2 * (time_interval_[i] + time_interval_[i + 1]);
                    A[j](i, i - 1) = time_interval_[i];
                    A[j](i, i + 1) = time_interval_[i + 1];
                    B[j](i, 0) = 6 * ((points_for_planning_[i + 2][j] -
                                       points_for_planning_[i + 1][j]) /
                                          time_interval_[i + 1] -
                                      (points_for_planning_[i + 1][j] -
                                       points_for_planning_[i][j]) /
                                          time_interval_[i]);
                }
            }
        }
        for (int j = 0; j < 6; j++) {
            W[j] = A[j].partialPivLu().solve(B[j]);
        }
        //加速度矩阵m转化为向量
        std::vector<std::vector<double>> vector_acceleration(
            num_of_points - 2,
            std::vector<double>(6, 0));  // vector_acceleration(n-2)X6
        for (int i = 0; i < num_of_points - 2; i++) {
            for (int j = 0; j < 6; j++) {
                vector_acceleration[i][j] =
                    W[j](i, 0);  // vector_acceleration是N X 6
            }
        }
        //把控制点的值插入points_for_planning_
        for (int i = 0; i < 6; i++) {
            points_for_planning_[1][i] =
                points_for_planning_[0][i] + (std::pow(time_interval_[0], 2)) /
                                                 6 * vector_acceleration[0][i];
            points_for_planning_[num_of_points - 2][i] =
                points_for_planning_[num_of_points - 1][i] +
                std::pow(time_interval_[num_of_points - 2], 2) / 6 *
                    vector_acceleration[num_of_points - 3][i];
        }
        std::cout << "轨迹规划模块:带控制点的点集合:" << std::endl;
        for (int i = 0; i < points_for_planning_.size(); i++) {
            std::cout << points_for_planning_[i][0] << " "
                      << points_for_planning_[i][1] << " "
                      << points_for_planning_[i][2] << " "
                      << points_for_planning_[i][3] << " "
                      << points_for_planning_[i][4] << " "
                      << points_for_planning_[i][5] << " " << std::endl;
        }
        std::cout << "---------------------------" << std::endl;
        //加速度向量插入收尾值
        std::vector<double> zero{0, 0, 0, 0, 0, 0};
        vector_acceleration.insert(vector_acceleration.begin(), zero);
        vector_acceleration.insert(vector_acceleration.end(), zero);
        // std::cout<<"轨迹规划模块:各个关节的加速度值:"<<std::endl;
        // for(int i=0;i<vector_acceleration.size();i++)
        // {
        // 	std::cout<<vector_acceleration[i][0]<<"
        // "<<vector_acceleration[i][1]<<" "<<vector_acceleration[i][2]<<"
        // "<<vector_acceleration[i][3]<<" "<<vector_acceleration[i][4]<<"
        // "<<vector_acceleration[i][5]<<" "<<std::endl;
        // }
        cubic_spline_parameter_.clear();
        cubic_spline_parameter_.resize(
            6,
            std::vector<std::vector<double>>(
                num_of_points - 1,
                std::vector<double>(
                    4, 0)));  // cubic_spline_parameter_是6 X N X 4矩阵
        double a, b, c, d;
        //计算三次样条曲线参数
        for (int i = 0; i < num_of_points - 1; i++) {
            for (int j = 0; j < 6; j++) {
                a = points_for_planning_[i][j];
                b = (points_for_planning_[i + 1][j] -
                     points_for_planning_[i][j]) /
                        time_interval_[i] -
                    (time_interval_[i] / 6) * (vector_acceleration[i + 1][j] +
                                               2 * vector_acceleration[i][j]);
                c = vector_acceleration[i][j] / 2;
                d = (vector_acceleration[i + 1][j] -
                     vector_acceleration[i][j]) /
                    (6 * time_interval_[i]);
                std::vector<double> cubic_spline_parameter_element;
                cubic_spline_parameter_element.push_back(a);
                cubic_spline_parameter_element.push_back(b);
                cubic_spline_parameter_element.push_back(c);
                cubic_spline_parameter_element.push_back(d);
                cubic_spline_parameter_[j][i] =
                    cubic_spline_parameter_element;  // cubic_spline_parameter_是一个
                                                     // 6 X N 矩阵
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

//多关节采样
void CubicSplinePlanning::insertAllJointsCubicSpline(
    int step)  //返回6XN矩阵, step表示相邻两个路径点间采样的点数
{
    std::vector<std::vector<pointWithTimeStamp>>
        points_list_all_joints;  // 6XN矩阵
    pos_list_.clear();
    vel_list_.clear();
    acc_list_.clear();
    //输入点为2进行五次多项式采样
    if (input_points_num_ == 2) {
        jointAnglesWithTimeStamp angels_value_with_time;
        jointVelocitiesWithTimeStamp vels_value_with_time;
        jointAccelerationsWithTimeStamp accs_value_with_time;
        double time = 0;
        double time_step_size = time_interval_[0] / step;
        for (int i = 0; i < step + 1; i++) {
            angels_value_with_time.time = time;
            vels_value_with_time.time = time;
            accs_value_with_time.time = time;
            for (int j = 0; j < 6; j++) {
                //位置采样
                double pos_each_step =
                    quartic_spline_parameter_[j][0] +
                    quartic_spline_parameter_[j][1] * pow(time, 3) +
                    quartic_spline_parameter_[j][2] * pow(time, 4) +
                    quartic_spline_parameter_[j][3] * pow(time, 5);
                angels_value_with_time.joints_value[j] = pos_each_step;
                //速度采样
                double vel_each_step =
                    3 * quartic_spline_parameter_[j][1] * pow((time), 2) +
                    4 * quartic_spline_parameter_[j][2] * pow((time), 3) +
                    5 * quartic_spline_parameter_[j][3] * pow((time), 4);
                vels_value_with_time.joints_vel_value[j] = vel_each_step;
                //加速度采样
                double acc_each_step =
                    6 * quartic_spline_parameter_[j][1] * (time) +
                    12 * quartic_spline_parameter_[j][2] * pow((time), 2) +
                    20 * quartic_spline_parameter_[j][3] * pow((time), 3);
                accs_value_with_time.joints_acc_value[j] = acc_each_step;
            }
            pos_list_.push_back(angels_value_with_time);
            vel_list_.push_back(vels_value_with_time);
            acc_list_.push_back(accs_value_with_time);
            time += time_step_size;
        }
        quartic_spline_parameter_.clear();
        return;
    }
    //输入点为3进行四次多项式采样
    else if (input_points_num_ == 3) {
        //对四次多项式进行采样
        jointAnglesWithTimeStamp angels_value_with_time;
        jointVelocitiesWithTimeStamp vels_value_with_time;
        jointAccelerationsWithTimeStamp accs_value_with_time;
        double time = 0;
        double time_step_size = time_interval_[0] / step;
        for (int i = 0; i < step + 1; i++)  //第一条曲线采样
        {
            angels_value_with_time.time = time;
            vels_value_with_time.time = time;
            accs_value_with_time.time = time;
            for (int j = 0; j < 6; j++) {
                //位置采样
                double value_each_step =
                    quintic_spline_parameter0_[j][0] +
                    quintic_spline_parameter0_[j][3] * pow(time, 3) +
                    quintic_spline_parameter0_[j][4] * pow(time, 4);
                angels_value_with_time.joints_value[j] = value_each_step;
                //速度采样
                double vel_each_step =
                    3 * quintic_spline_parameter0_[j][3] * pow(time, 2) +
                    4 * quintic_spline_parameter0_[j][4] * pow(time, 3);
                vels_value_with_time.joints_vel_value[j] = vel_each_step;
                //加速度采样
                double acc_each_step =
                    6 * quintic_spline_parameter0_[j][3] * time +
                    12 * quintic_spline_parameter0_[j][4] * pow(time, 2);
                accs_value_with_time.joints_acc_value[j] = acc_each_step;
            }
            pos_list_.push_back(angels_value_with_time);
            vel_list_.push_back(vels_value_with_time);
            acc_list_.push_back(accs_value_with_time);
            time += time_step_size;
        }
        time = time - time_step_size;
        time_step_size = time_interval_[1] / step;
        double time_spline2 = time_step_size;
        time = time + time_step_size;
        for (int i = 0; i < step; i++)  //第二条曲线采样
        {
            angels_value_with_time.time = time;
            vels_value_with_time.time = time;
            accs_value_with_time.time = time;
            for (int j = 0; j < 6; j++) {
                //位置采样
                double pos_each_step =
                    quintic_spline_parameter1_[j][0] +
                    quintic_spline_parameter1_[j][1] * time_spline2 +
                    quintic_spline_parameter1_[j][2] * pow(time_spline2, 2) +
                    quintic_spline_parameter1_[j][3] * pow(time_spline2, 3) +
                    quintic_spline_parameter1_[j][4] * pow(time_spline2, 4);
                angels_value_with_time.joints_value[j] = pos_each_step;
                //速度采样
                double vel_each_step =
                    quintic_spline_parameter1_[j][1] +
                    2 * quintic_spline_parameter1_[j][2] * time_spline2 +
                    3 * quintic_spline_parameter1_[j][3] *
                        pow(time_spline2, 2) +
                    4 * quintic_spline_parameter1_[j][4] * pow(time_spline2, 3);
                vels_value_with_time.joints_vel_value[j] = vel_each_step;
                //加速度采样
                double acc_each_step =
                    2 * quintic_spline_parameter1_[j][2] +
                    6 * quintic_spline_parameter1_[j][3] * time_spline2 +
                    12 * quintic_spline_parameter1_[j][4] *
                        pow(time_spline2, 2);
                accs_value_with_time.joints_acc_value[j] = acc_each_step;
            }
            pos_list_.push_back(angels_value_with_time);
            vel_list_.push_back(vels_value_with_time);
            acc_list_.push_back(accs_value_with_time);
            time += time_step_size;
            time_spline2 += time_step_size;
        }
        quintic_spline_parameter0_.clear();
        quintic_spline_parameter1_.clear();
        return;
    }
    //输入4个及以上点进行三次多项式采样
    else {
        jointAnglesWithTimeStamp angels_value_with_time;
        jointVelocitiesWithTimeStamp vels_value_with_time;
        jointAccelerationsWithTimeStamp accs_value_with_time;
        double time_whole_curve = 0;
        for (int i = 0; i < cubic_spline_parameter_[0].size(); i++) {
            double time_each_segment = 0;
            double time_step_size = time_interval_[i] / step;
            for (int j = 0; j < step; j++) {
                angels_value_with_time.time = time_whole_curve;
                vels_value_with_time.time = time_whole_curve;
                accs_value_with_time.time = time_whole_curve;
                for (int k = 0; k < 6; k++) {
                    //位置采样
                    double pos_each_step =
                        cubic_spline_parameter_[k][i][0] +
                        cubic_spline_parameter_[k][i][1] * time_each_segment +
                        cubic_spline_parameter_[k][i][2] *
                            pow(time_each_segment, 2) +
                        cubic_spline_parameter_[k][i][3] *
                            pow(time_each_segment, 3);
                    angels_value_with_time.joints_value[k] = pos_each_step;
                    //速度采样
                    double vel_each_step =
                        cubic_spline_parameter_[k][i][1] +
                        2 * cubic_spline_parameter_[k][i][2] *
                            time_each_segment +
                        3 * cubic_spline_parameter_[k][i][3] *
                            pow(time_each_segment, 2);
                    vels_value_with_time.joints_vel_value[k] = vel_each_step;
                    //加速度采样
                    double acc_each_step =
                        2 * cubic_spline_parameter_[k][i][2] +
                        6 * cubic_spline_parameter_[k][i][3] *
                            time_each_segment;
                    accs_value_with_time.joints_acc_value[k] = acc_each_step;
                }
                pos_list_.push_back(angels_value_with_time);
                vel_list_.push_back(vels_value_with_time);
                acc_list_.push_back(accs_value_with_time);
                time_each_segment += time_step_size;
                time_whole_curve += time_step_size;
            }
        }
        angels_value_with_time.time = time_whole_curve;
        vels_value_with_time.time = time_whole_curve;
        accs_value_with_time.time = time_whole_curve;
        for (int k = 0; k < 6; k++)  //插入最后时刻的目标点
        {
            angels_value_with_time.joints_value[k] =
                points_for_planning_.back()[k];
            vels_value_with_time.joints_vel_value[k] = 0;
            accs_value_with_time.joints_acc_value[k] = 0;
        }
        pos_list_.push_back(angels_value_with_time);
        vel_list_.push_back(vels_value_with_time);
        acc_list_.push_back(accs_value_with_time);
    }
    // for (int k = 0; k < time_interval_.size(); k++)//插入最后时刻的目标点
    // {
    // 	std::cout<<"轨迹规划模块:time_interval_:
    // "<<time_interval_[k]<<std::endl;
    // }
    points_for_planning_.clear();
    time_interval_.clear();
    cubic_spline_parameter_.clear();
    std::cout << "轨迹规划模块:多关节采样完成！" << std::endl;
    return;
}

void CubicSplinePlanning::getLineTrajectoryPlanning(
    const CubicSplinePlanning::multiDimensionPoint* point_start,
    const CubicSplinePlanning::multiDimensionPoint* point_end,
    int num_insert)  //直线轨迹规划
{
    if (num_insert < 2) {
        std::cout << "轨迹规划模块:插入点不能少于2个!" << std::endl;
    } else {
        std::cout << "轨迹规划模块:开始直线规划!" << std::endl;
        std::cout << "轨迹规划模块:start: " << (*point_start)[0] << " "
                  << (*point_start)[1] << " " << (*point_start)[2] << " "
                  << (*point_start)[3] << " " << (*point_start)[4] << " "
                  << (*point_start)[5] << " " << std::endl;
        std::cout << "轨迹规划模块:end: " << (*point_end)[0] << " "
                  << (*point_end)[1] << " " << (*point_end)[2] << " "
                  << (*point_end)[3] << " " << (*point_end)[4] << " "
                  << (*point_end)[5] << " " << std::endl;
        CubicSplinePlanning::multiDimensionPoint point_joints{0, 0, 0, 0, 0, 0};
        CubicSplinePlanning::multiDimensionPoint vec{0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 6; i++) {
            vec[i] = ((*point_end)[i] - (*point_start)[i]) / (num_insert + 1);
        }
        std::vector<multiDimensionPoint> points_input;  // points_input为NX6矩阵
        points_input.emplace_back(*point_start);
        for (int i = 0; i < num_insert + 1; i++) {
            for (int j = 0; j < 6; j++) {
                point_joints[j] = points_input.back()[j] + vec[j];
            }
            points_input.push_back(point_joints);
        }
        for (int i = 0; i < points_input.size(); i++) {
            std::cout << "轨迹规划模块:直线点: " << points_input[i][0] << " "
                      << points_input[i][1] << " " << points_input[i][2] << " "
                      << points_input[i][3] << " " << points_input[i][4] << " "
                      << points_input[i][5] << " " << std::endl;
        }
        initCubicSplineWithControlPoints(points_input);
        insertAllJointsCubicSpline(200);
        return;
    }
}
