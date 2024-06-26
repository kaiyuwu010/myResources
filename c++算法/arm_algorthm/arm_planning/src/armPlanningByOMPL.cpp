#include "arm_planning/armPlanningByOMPL.h"

/*
通过调用pathPlanning文件，进行路径规划。pathPlaning文件通过ompl库进行路径规划。
*/
ArmPlanning::~ArmPlanning()
{
    delete cd, csp;
    cd = NULL;
    csp = NULL;
}
bool ArmPlanning::isStateValid(const ob::State* state)
{
    const ob::RealVectorStateSpace::StateType* state6 =
        state->as<ob::RealVectorStateSpace::StateType>();
    double theta_input[7] = {0,
                             (*state6)[0],
                             (*state6)[1] - 1.570796,
                             (*state6)[2] - 1.570796,
                             (*state6)[3],
                             (*state6)[4],
                             (*state6)[5]};
    return cd->isStateValid(theta_input);
    // return 1;
}
void ArmPlanning::plan(const Eigen::Matrix4d& transStart,
                       const Eigen::Matrix4d& transGoal)
{
    ik->inverseKinematics(transStart);
    std::vector<double> start = {ik->theta[1][1],
                                 ik->theta[1][2],
                                 ik->theta[1][3],
                                 ik->theta[1][4],
                                 ik->theta[1][5],
                                 ik->theta[1][6]};
    std::cout << "起始点关节角(弧度):" << ik->theta[1][1] / 3.14159 * 180 << " "
              << ik->theta[1][2] / 3.14159 * 180 << " "
              << ik->theta[1][3] / 3.14159 * 180 << " "
              << ik->theta[1][4] / 3.14159 * 180 << " "
              << ik->theta[1][5] / 3.14159 * 180 << " "
              << ik->theta[1][6] / 3.14159 * 180 << std::endl;
    ik->inverseKinematics(transGoal);
    std::vector<double> goal = {ik->theta[3][1],
                                ik->theta[3][2],
                                ik->theta[3][3],
                                ik->theta[3][4],
                                ik->theta[3][5],
                                ik->theta[3][6]};
    std::cout << "目标点关节角(°):" << ik->theta[3][1] / 3.14159 * 180 << " "
              << ik->theta[3][2] / 3.14159 * 180 << " "
              << ik->theta[3][3] / 3.14159 * 180 << " "
              << ik->theta[3][4] / 3.14159 * 180 << " "
              << ik->theta[3][5] / 3.14159 * 180 << " "
              << ik->theta[3][6] / 3.14159 * 180 << std::endl;
    std::cout << "目标点关节角(弧度):" << ik->theta[3][1] << " "
              << ik->theta[3][2] << " " << ik->theta[3][3] << " "
              << ik->theta[3][4] << " " << ik->theta[3][5] << " "
              << ik->theta[3][6] << std::endl;
    PathPlanning pp;
    pp.plan(std::bind(&ArmPlanning::isStateValid, this, std::placeholders::_1),
            start,
            goal);  //使用std::bind绑定本地类成员函数也要加类作用域
    // ob::RealVectorStateSpace::StateType* state;
    std::cout << "paths:" << std::endl;
    for (int i = 0; i < pp.paths.size(); i++) {
        // state = (pp.paths[i])->as<ob::RealVectorStateSpace::StateType>();
        for (int j = 0; j < 6; j++) {
            std::cout << pp.paths[i][j] << " ";
        }
        std::cout << std::endl;
    }
    csp->initCubicSplineWithControlPoints(pp.paths);
    csp->insertAllJointsCubicSpline(50);
}

void unitTest()
{
    ros::init(argc, argv, "trajectory_planning");
    ros::NodeHandle nh;
    ros::ServiceClient client =
        nh.serviceClient<xarm_msgs::Move>("/ufactory/move_servoj");

    Eigen::Matrix4d matrix1;
    matrix1 << 0.868316, 0.495977, -0.00591813, 400.671, 0.495425, -0.866646,
        0.0589712, 0.826934, 0.0241194, -0.0541376, -0.998242, 59.8589, 0, 0, 0,
        1;
    Eigen::Matrix4d matrix0;
    matrix0 << 1, 0, 0, 287, 0, -1, 0, 0, 0, 0, -1, -45.8, 0, 0, 0, 1;
    ArmPlanning ap;
    ap.plan(matrix0, matrix1);

    xarm_msgs::Move planning_result;
    planning_result.request.pose = {0, 0, 0, 0, 0, 0};
    planning_result.request.mvvelo = 0.0;
    planning_result.request.mvacc = 0.0;
    planning_result.request.mvtime = 0.0;
    planning_result.request.mvradii = 0.0;
    double time = 0;
    for (int i = 0; i < ap.csp->pos_list_.size(); i++) {
        planning_result.request.pose[0] =
            float(ap.csp->pos_list_[i].joints_value[0]);
        planning_result.request.pose[1] =
            float(ap.csp->pos_list_[i].joints_value[1]);
        planning_result.request.pose[2] =
            float(ap.csp->pos_list_[i].joints_value[2]);
        planning_result.request.pose[3] =
            float(ap.csp->pos_list_[i].joints_value[3]);
        planning_result.request.pose[4] =
            float(ap.csp->pos_list_[i].joints_value[4]);
        planning_result.request.pose[5] =
            float(ap.csp->pos_list_[i].joints_value[5]);
        ros::Duration(ap.csp->pos_list_[i].time - time).sleep();
        time = ap.csp->pos_list_[i].time;
        client.call(planning_result);
    }
}
