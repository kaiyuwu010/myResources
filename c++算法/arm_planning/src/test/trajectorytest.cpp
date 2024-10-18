#include "../../include/arm_planning/trajectoryPlanning.h"

void trajectoryPlanningTest()
{
    // std::cout.precision(6);
    std::vector<double> point0 = {0, 0, 0, 0, 0, 0, 0};
    std::vector<double> point1 = {60, 30, 90, 60, 40, 45, 70};
    std::vector<double> point2 = {10, 55, 30, 70, 90, 30, 45};
    std::vector<double> point3 = {5, 30, 10, 15, 30, 50, 80};
    std::vector<double> point4 = {15, 25, 40, 20, 50, 45, 60};
    std::vector<std::vector<double>> points_for_planning_;
    points_for_planning_.push_back(point0);
    points_for_planning_.push_back(point1);
    points_for_planning_.push_back(point2);
    points_for_planning_.push_back(point3);
    points_for_planning_.push_back(point4);
    CubicSplinePlanning csp(7);
    for (size_t i = 0; i < points_for_planning_.size(); i++)
    {
        csp.addPoint(points_for_planning_[i]);
    }
    csp.setAverageSpeed(30);
    // std::vector<double> v0(7, 5);
    // std::vector<double> m0(7, 5);
    // std::vector<double> vn(7, 5);
    // std::vector<double> mn(7, 5);
    std::vector<double> v0(7, 10);
    std::vector<double> m0(7, 10);
    std::vector<double> vn(7, 10);
    std::vector<double> mn(7, 0);
    csp.setEndpointsSpeedAndAcceleration(v0, m0, vn, mn);
    csp.initCubicSplineWithControlPoints();
    csp.insertAllJointsCubicSpline(0.01);
    // for(int i=0; i<csp.pos_list_.size(); i++)
    // {
    // 	std::cout<<"时间:"<<csp.pos_list_[i].time<<" 关节1:
    // "<<csp.pos_list_[i].joints_value[0]<<" 关节2:
    // "<<csp.pos_list_[i].joints_value[1]<<" 关节3:
    // "<<csp.pos_list_[i].joints_value[2]<<" 关节4: "
    //     <<csp.pos_list_[i].joints_value[3]<<" 关节5:
    //     "<<csp.pos_list_[i].joints_value[4]<<" 关节6:
    //     "<<csp.pos_list_[i].joints_value[5]<<std::endl;
    // }
    // 用于gnuplot工具显示的打印数据，输出数据的指令是: rosrun arm_planning
    // trajectorytest > test.dat, 打开gnuplot指令是: gnuplot ,显示数据的指令是: plot
    // "test.dat" w l
    // // 位置曲线
    // for (size_t j = 0; j < point0.size(); j++)
    // {
    //     for (size_t i = 0; i < csp.pos_list_.size(); i++)
    //     {
    //         std::cout << csp.pos_list_[i].time << " " << csp.pos_list_[i].coordinates[j] << std::endl;
    //     }
    // }
    // // 速度曲线
    // for (size_t j = 0; j < point0.size(); j++)
    // {
    //     for (size_t i = 0; i < csp.pos_list_.size(); i++)
    //     {
    //         std::cout << csp.vel_list_[i].time << " " << csp.vel_list_[i].velocities[j] << std::endl;
    //     }
    // }
    //加速度曲线
    for (size_t j = 0; j < point0.size(); j++)
    {
        for (size_t i = 0; i < csp.pos_list_.size(); i++)
        {
            std::cout << csp.acc_list_[i].time << " " << csp.acc_list_[i].accelerations[j] << std::endl;
        }
    }
}

int main(int argc, char** argv)
{
    trajectoryPlanningTest();
    return 0;
}
