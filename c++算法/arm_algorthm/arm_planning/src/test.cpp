#include "arm_planning/RRTConnect.h"
#include "arm_planning/collisionDetect.h"
#include "arm_planning/inverseKinematics.h"
#include "arm_planning/kinematics.h"
#include "arm_planning/trajectoryPlanning.h"

double joint1;
double joint2;
double joint3;
double joint4;
double joint5;
double joint6;
bool get_joint_state = false;
CollisionDetection* cd = new CollisionDetection();

bool isStateValid(const std::vector<double>& node)
{
    return cd->isStateValid(node);
}

void rrtTest(int argc, char** argv)
{
    RRTConnect::VectorSixDimension lower_vector(
        -3.1415926, -3.1415926, -1.570796, -3.1415926, -1.570796, -3.1415926);
    RRTConnect::VectorSixDimension upper_vector(
        3.1415926, 3.1415926, 4.71238898, 3.1415926, 1.570796, 3.1415926);
    RRTConnect::SampleRange sample_range(upper_vector, lower_vector);
    double dis_expand = 0.05;
    double dis_optimize = 0.6;
    double goal_sampleRate = 10;
    int max_iteration = 1000000;
    RRTConnect rrt(std::bind(&isStateValid, std::placeholders::_1),
                   sample_range,
                   dis_expand,
                   dis_optimize,
                   goal_sampleRate,
                   max_iteration);
    std::vector<RRTConnect::VectorSixDimension> path;
    std::vector<RRTConnect::RRTNode*> points_sequences;
    std::vector<double> vecPoint1 = {0, 0, 0, 0, 0, 0};
    std::vector<double> vecPoint2 = {30, 0, 0, 0, 0, 0};
    std::shared_ptr<RRTConnect::VectorSixDimension> Point1 =
        std::make_shared<RRTConnect::VectorSixDimension>(vecPoint1[0] / 180.0 * 3.141592,
                                           (vecPoint1[1] / 180.0 * 3.141592),
                                           (vecPoint1[2] / 180.0 * 3.141592),
                                           vecPoint1[3] / 180.0 * 3.141592,
                                           vecPoint1[4] / 180.0 * 3.141592,
                                           vecPoint1[5] / 180.0 * 3.141592);
    std::shared_ptr<RRTConnect::VectorSixDimension> Point2 =
        std::make_shared<RRTConnect::VectorSixDimension>(vecPoint2[0] / 180.0 * 3.141592,
                                           (vecPoint2[1] / 180.0 * 3.141592),
                                           (vecPoint2[2] / 180.0 * 3.141592),
                                           vecPoint2[3] / 180.0 * 3.141592,
                                           vecPoint2[4] / 180.0 * 3.141592,
                                           vecPoint2[5] / 180.0 * 3.141592);
    // rrt.setBegin(Point1);
    // rrt.setEnd(Point2);
    // path = rrt.planning();
    std::vector<std::shared_ptr<RRTConnect::VectorSixDimension>> way_points;
    way_points.push_back(Point1);
    way_points.push_back(Point2);
    way_points.push_back(Point1);
    rrt.multiWayPointsPlanning(way_points, path);
    std::cout << "输入角度Point1为: " << Point1->element1_ << "  "
              << Point1->element2_ << "  " << Point1->element3_ << "  "
              << Point1->element4_ << "  " << Point1->element5_ << "  "
              << Point1->element6_ << std::endl;
    std::cout << "输入角度Point2为: " << Point2->element1_ << "  "
              << Point2->element2_ << "  " << Point2->element3_ << "  "
              << Point2->element4_ << "  " << Point2->element5_ << "  "
              << Point2->element6_ << std::endl;
}
void collisionDetectTest()
{
    clock_t start, end;
    CollisionDetection cd;  // 45度0.785398163；90度1.570796327；
    std::vector<double> theta_input{
        0, -1.570796327, -1.570796327, 0, 0, 0};  //机械臂初始位姿
    bool a;
    start = clock();
    a = cd.isStateValid(theta_input);
    end = clock();
    double elapsedTime = static_cast<double>(end - start) / 1000;
    std::cout << "运行时间为:" << elapsedTime << "ms" << std::endl;
    std::cout << "第一段碰撞检测结果(1不碰撞;0碰撞):" << a << std::endl;
}
void trajectoryPlanningTest()
{
    std::cout.precision(4);
    CubicSplinePlanning::multiDimensionPoint point0 = {0, 0, 0, 0, 0, 0};
    CubicSplinePlanning::multiDimensionPoint point1 = {0.20373566448688507,
                                                       0.5496061444282532,
                                                       1.307684063911438,
                                                       -0.3112063407897949,
                                                       0.5709284543991089,
                                                       0.39078161120414734};
    CubicSplinePlanning::multiDimensionPoint point2 = {-0.3661976456642151,
                                                       0.6344352960586548,
                                                       1.203168272972107,
                                                       0.09644903987646103,
                                                       0.48711878061294556,
                                                       0.2667209208011627};
    CubicSplinePlanning::multiDimensionPoint point3 = {0, 0, 0, 0, 0, 0};
    std::vector<CubicSplinePlanning::multiDimensionPoint> points_for_planning_;
    points_for_planning_.push_back(point0);
    points_for_planning_.push_back(point1);
    points_for_planning_.push_back(point2);
    // points_for_planning_.push_back(point3);
    CubicSplinePlanning csp;
    csp.initCubicSplineWithControlPoints(points_for_planning_);
    csp.insertAllJointsCubicSpline(200);
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
    // moduleTest > test.dat, 打开gnuplot指令是: gnuplot ,显示数据的指令是: plot
    // "test.dat" w l
    for (int i = 0; i < csp.pos_list_.size(); i++) {
        std::cout << csp.pos_list_[i].time << " "
                  << csp.pos_list_[i].joints_value[0] << std::endl;
    }
    for (int i = 0; i < csp.pos_list_.size(); i++) {
        std::cout << csp.pos_list_[i].time << " "
                  << csp.pos_list_[i].joints_value[1] << std::endl;
    }
    for (int i = 0; i < csp.pos_list_.size(); i++) {
        std::cout << csp.pos_list_[i].time << " "
                  << csp.pos_list_[i].joints_value[2] << std::endl;
    }
    for (int i = 0; i < csp.pos_list_.size(); i++) {
        std::cout << csp.pos_list_[i].time << " "
                  << csp.pos_list_[i].joints_value[3] << std::endl;
    }
    for (int i = 0; i < csp.pos_list_.size(); i++) {
        std::cout << csp.pos_list_[i].time << " "
                  << csp.pos_list_[i].joints_value[4] << std::endl;
    }
    for (int i = 0; i < csp.pos_list_.size(); i++) {
        std::cout << csp.pos_list_[i].time << " "
                  << csp.pos_list_[i].joints_value[5] << std::endl;
    }
}
void kinematicsTest()
{
    Kinematics kin;
    // std::vector<double> theta_input{0, 9.9, 31.8, 0, 21.9, 0};
    std::vector<double> theta_input{0 / 180.0 * 3.141592,
                                    0 / 180.0 * 3.141592,
                                    0 / 180.0 * 3.141592,
                                    0 / 180.0 * 3.141592,
                                    -27.7 / 180.0 * 3.141592,
                                    0 / 180.0 * 3.141592};
    std::cout << "输入角度为: " << theta_input[0] / 3.141592 * 180 << "  "
              << theta_input[1] / 3.141592 * 180 << "  "
              << theta_input[2] / 3.141592 * 180 << "  "
              << theta_input[3] / 3.141592 * 180 << "  "
              << theta_input[4] / 3.141592 * 180 << "  "
              << theta_input[5] / 3.141592 * 180 << std::endl;
    std::cout << "输入弧度角为: " << theta_input[0] << "  " << theta_input[1]
              << "  " << theta_input[2] << "  " << theta_input[3] << "  "
              << theta_input[4] << "  " << theta_input[5] << std::endl;
    theta_input[1] -= 1.570796327;
    theta_input[2] -= 1.570796327;
    Eigen::Matrix4d transform_output = kin.kinematics(theta_input, 6);
    std::cout << "正运动学结果为(单位mm):" << std::endl
              << transform_output(0, 0) << ",       " << transform_output(0, 1)
              << ",       " << transform_output(0, 2) << ",       "
              << transform_output(0, 3) * 1000 << ",       " << std::endl
              << transform_output(1, 0) << ",       " << transform_output(1, 1)
              << ",       " << transform_output(1, 2) << ",       "
              << transform_output(1, 3) * 1000 << ",       " << std::endl
              << transform_output(2, 0) << ",       " << transform_output(2, 1)
              << ",       " << transform_output(2, 2) << ",       "
              << transform_output(2, 3) * 1000 << ",       " << std::endl
              << transform_output(3, 0) << ",       " << transform_output(3, 1)
              << ",       " << transform_output(3, 2) << ",       "
              << transform_output(3, 3) << ";       " << std::endl;
    Eigen::Matrix3d rotation_matrix = transform_output.block(0, 0, 3, 3);
    std::cout << "旋转矩阵为:" << std::endl << rotation_matrix << std::endl;
    Eigen::Vector3d eulerAngle1 =
        rotation_matrix.eulerAngles(2, 1, 0);  // zyx顺序
    std::cout << "rpy欧拉角:" << eulerAngle1[2] / 3.141592 * 180.0 << " "
              << eulerAngle1[1] / 3.141592 * 180.0 << " "
              << eulerAngle1[0] / 3.141592 * 180.0 << std::endl;
}
void invKinematicsTest()
{
    Eigen::Matrix4d matrix1;
    matrix1 << 0.999949, 0.00179935, 0.00993482, 268.881, 0.00226969, -0.998867,
        -0.0475355, 5.76791, 0.00983803, 0.0475556, -0.99882, 87.028, 0, 0, 0,
        1;
    clock_t start, end;
    start = clock();
    InverseKinematics ik;
    ik.inverseKinematics(matrix1);
    end = clock();
    double elapsedTime =
        static_cast<double>(end - start) / CLOCKS_PER_SEC * 1000;
    std::cout << "运行时间为:" << elapsedTime << std::endl;
    std::cout << "逆运动学结果为:" << std::endl;
    std::cout << "-------------------------弧度--------------------------"
              << std::endl
              << ik.theta[1][1] << ",       " << ik.theta[1][2] << ",       "
              << ik.theta[1][3] << ",       " << ik.theta[1][4] << ",       "
              << ik.theta[1][5] << ",       " << ik.theta[1][6] << std::endl
              << ik.theta[2][1] << ",       " << ik.theta[2][2] << ",       "
              << ik.theta[2][3] << ",       " << ik.theta[2][4] << ",       "
              << ik.theta[2][5] << ",       " << ik.theta[2][6] << std::endl
              << ik.theta[3][1] << ",       " << ik.theta[3][2] << ",       "
              << ik.theta[3][3] << ",       " << ik.theta[3][4] << ",       "
              << ik.theta[3][5] << ",       " << ik.theta[3][6] << std::endl
              << ik.theta[4][1] << ",       " << ik.theta[4][2] << ",       "
              << ik.theta[4][3] << ",       " << ik.theta[4][4] << ",       "
              << ik.theta[4][5] << ",       " << ik.theta[4][6] << std::endl
              << ik.theta[5][1] << ",       " << ik.theta[5][2] << ",       "
              << ik.theta[5][3] << ",       " << ik.theta[5][4] << ",       "
              << ik.theta[5][5] << ",       " << ik.theta[5][6] << std::endl
              << ik.theta[6][1] << ",       " << ik.theta[6][2] << ",       "
              << ik.theta[6][3] << ",       " << ik.theta[6][4] << ",       "
              << ik.theta[6][5] << ",       " << ik.theta[6][6] << std::endl
              << ik.theta[7][1] << ",       " << ik.theta[7][2] << ",       "
              << ik.theta[7][3] << ",       " << ik.theta[7][4] << ",       "
              << ik.theta[7][5] << ",       " << ik.theta[7][6] << std::endl
              << ik.theta[8][1] << ",       " << ik.theta[8][2] << ",       "
              << ik.theta[8][3] << ",       " << ik.theta[8][4] << ",       "
              << ik.theta[8][5] << ",       " << ik.theta[8][6] << std::endl;
    std::cout << "-------------------------角度--------------------------"
              << std::endl
              << ik.theta[1][1] / 3.14159 * 180 << "       "
              << ik.theta[1][2] / 3.14159 * 180 << "       "
              << ik.theta[1][3] / 3.14159 * 180 << "       "
              << ik.theta[1][4] / 3.14159 * 180 << "       "
              << ik.theta[1][5] / 3.14159 * 180 << "       "
              << ik.theta[1][6] / 3.14159 * 180 << std::endl
              << ik.theta[2][1] / 3.14159 * 180 << "       "
              << ik.theta[2][2] / 3.14159 * 180 << "       "
              << ik.theta[2][3] / 3.14159 * 180 << "       "
              << ik.theta[2][4] / 3.14159 * 180 << "       "
              << ik.theta[2][5] / 3.14159 * 180 << "       "
              << ik.theta[2][6] / 3.14159 * 180 << std::endl
              << ik.theta[3][1] / 3.14159 * 180 << "       "
              << ik.theta[3][2] / 3.14159 * 180 << "       "
              << ik.theta[3][3] / 3.14159 * 180 << "       "
              << ik.theta[3][4] / 3.14159 * 180 << "       "
              << ik.theta[3][5] / 3.14159 * 180 << "       "
              << ik.theta[3][6] / 3.14159 * 180 << std::endl
              << ik.theta[4][1] / 3.14159 * 180 << "       "
              << ik.theta[4][2] / 3.14159 * 180 << "       "
              << ik.theta[4][3] / 3.14159 * 180 << "       "
              << ik.theta[4][4] / 3.14159 * 180 << "       "
              << ik.theta[4][5] / 3.14159 * 180 << "       "
              << ik.theta[4][6] / 3.14159 * 180 << std::endl
              << ik.theta[5][1] / 3.14159 * 180 << "       "
              << ik.theta[5][2] / 3.14159 * 180 << "       "
              << ik.theta[5][3] / 3.14159 * 180 << "       "
              << ik.theta[5][4] / 3.14159 * 180 << "       "
              << ik.theta[5][5] / 3.14159 * 180 << "       "
              << ik.theta[5][6] / 3.14159 * 180 << std::endl
              << ik.theta[6][1] / 3.14159 * 180 << "       "
              << ik.theta[6][2] / 3.14159 * 180 << "       "
              << ik.theta[6][3] / 3.14159 * 180 << "       "
              << ik.theta[6][4] / 3.14159 * 180 << "       "
              << ik.theta[6][5] / 3.14159 * 180 << "       "
              << ik.theta[6][6] / 3.14159 * 180 << std::endl
              << ik.theta[7][1] / 3.14159 * 180 << "       "
              << ik.theta[7][2] / 3.14159 * 180 << "       "
              << ik.theta[7][3] / 3.14159 * 180 << "       "
              << ik.theta[7][4] / 3.14159 * 180 << "       "
              << ik.theta[7][5] / 3.14159 * 180 << "       "
              << ik.theta[7][6] / 3.14159 * 180 << std::endl
              << ik.theta[8][1] / 3.14159 * 180 << "       "
              << ik.theta[8][2] / 3.14159 * 180 << "       "
              << ik.theta[8][3] / 3.14159 * 180 << "       "
              << ik.theta[8][4] / 3.14159 * 180 << "       "
              << ik.theta[8][5] / 3.14159 * 180 << "       "
              << ik.theta[8][6] / 3.14159 * 180 << std::endl;
}
int main(int argc, char** argv)
{
    rrtTest(argc, argv);
    // collisionDetectTest();
    // trajectoryPlanningTest();
    // kinematicsTest();
    return 0;
}
