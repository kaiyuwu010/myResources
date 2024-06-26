#include "arm_planning/armPlanning.h"
// XArmAPI的参数定义
// server_ip,
// true,  is_radian
// true,  do_not_open
// true,  check_tcp_limit
// true,  check_joint_limit
// true,  check_cmdnum_limit
// false, check_robot_sn
// true,  check_is_ready
// true,  check_is_pause
// 0,     max_callback_thread_count
// 512,   max_cmdnum
// dof_,  init_axis
// DEBUG_MODE,   debug
// report_type_  report_type
ArmPlanning::ArmPlanning(ros::NodeHandle nh)
    : nh_(nh)
{
    //初始化机械臂程序控制接口，以及设置机械臂初始状态
    std::string ip_="192.168.1.206";
    arm = std::make_shared<XArmAPI>(ip_,
                                    true,
                                    true,
                                    true,
                                    true,
                                    true,
                                    false,
                                    true,
                                    true,
                                    0,
                                    512,
                                    6,
                                    0,
                                    "normal");
    while (ros::ok()) {
        arm->connect();  //与机械臂建立连接
        if (arm->is_connected()) {
            break;
        }
        std::cout << "armPlanning模块:机械臂网络连接失败！" << std::endl;
    }
    //清除机械臂报错和警告
    if (arm->error_code != 0)
        arm->clean_error();
    if (arm->warn_code != 0)
        arm->clean_warn();
    //设置机械臂与夹爪的通信波特率
    clamp_ret = arm->set_tgpio_modbus_baudrate(921600);
    //通过机械臂向夹爪发送关闭指令
    clamp_ret = arm->getset_tgpio_modbus_data(
        send_close_data, 9, ret_data, 22, 9, true, false);
    //使能机械臂，设置机械臂模式和状态为0
    // Mode 0 : 基于xArm controller规划的位置模式；
    // Mode 1 : 基于外部轨迹规划器的位置模式；
    // Mode 2 : 自由拖动(零重力)模式；
    // Mode 3 : 保留；
    // Mode 4 : 关节速度控制模式；
    // Mode 5 : 笛卡尔速度控制模式；
    // Mode 6 : 关节在线规划模式；
    // state 为0（Ready状态）
    // state 为4（error状态）
    arm->motion_enable(true);
    arm->set_mode(0);
    arm->set_state(0);
    //初始化ros通信类
    setupComm();
    std::cout << "armPlanning模块:当前机械臂关节角:" << arm->angles[0] << " "
              << arm->angles[1] << " " << arm->angles[2] << " "
              << arm->angles[3] << " " << arm->angles[4] << " "
              << arm->angles[5] << " " << std::endl;
    //初始化rrt路径规划类
    RRTConnect::VectorSixDimension lower_vector(
        -1.570796, -1.570796, -1.570796, -3.1415926, -1.570796, -3.1415926);
    RRTConnect::VectorSixDimension upper_vector(
        1.570796, 1.570796, 4.71238898, 3.1415926, 1.570796, 3.1415926);
    RRTConnect::SampleRange sample_range(upper_vector, lower_vector);
    const double dis_expand = 0.05;  //单步拓展距离，弧度表示的关节角
    double disOptimize = 0.6;
    double goalSampleRate = 10;  //采样到目标点的概率
    int maxIter = 10000000;      //最大迭代次数
    rrt = std::make_shared<RRTConnect>(
        std::bind(&ArmPlanning::isStateValid, this, std::placeholders::_1),
        sample_range,
        dis_expand,
        disOptimize,
        goalSampleRate,
        maxIter);  //碰撞检测函数指针，采样空间等参数传递给rrt模块进行实例化
}
void ArmPlanning::setupComm()
{
    joints_states_pub =
        nh_.advertise<sensor_msgs::JointState>("lite6_joints_states", 1);
    server_move_to_goal = nh_.advertiseService(
        "move_to_goal", &ArmPlanning::moveToGoalCallback, this);
}
ArmPlanning::~ArmPlanning()
{}

bool ArmPlanning::isStateValid(
    const std::vector<double>&
        node)  //碰撞检测函数，调用的collisionDetect里的碰撞检测函数
{
    return cd->isStateValid(node);
}

bool ArmPlanning::moveToGoalCallback(
    arm_planning::arm_interactive::Request& req,
    arm_planning::arm_interactive::Response& res)
{
    //根据请求消息中的open_clamp变量的值确定夹爪操作
    if (ClampOperation(req.open_clamp) == Open) {
        //打开夹爪
        clamp_ret = arm->getset_tgpio_modbus_data(
            clear_error_data, 8, ret_data, 22, 9, true, false);
        clamp_ret = arm->getset_tgpio_modbus_data(
            send_open_data, 9, ret_data, 22, 9, true, false);
        std::cout << "armPlanning模块:打开夹爪中, ret=" << clamp_ret
                  << ", ret_data= " << char(ret_data[0]) << " "
                  << char(ret_data[1]) << " " << char(ret_data[2]) << " "
                  << char(ret_data[3]) << " " << char(ret_data[4]) << " "
                  << char(ret_data[5]) << " " << std::endl;
        res.succese = true;
    } else if (ClampOperation(req.open_clamp) == Close) {
        //关闭夹爪
        clamp_ret = arm->getset_tgpio_modbus_data(
            clear_error_data, 8, ret_data, 22, 9, true, false);
        clamp_ret = arm->getset_tgpio_modbus_data(
            send_close_data, 9, ret_data, 22, 9, true, false);
        std::cout << "armPlanning模块:关闭夹爪中, ret=" << clamp_ret
                  << ", ret_data= " << char(ret_data[0]) << " "
                  << char(ret_data[1]) << " " << char(ret_data[2]) << " "
                  << char(ret_data[3]) << " " << char(ret_data[4]) << " "
                  << char(ret_data[5]) << " " << std::endl;
        res.succese = true;
    } else {
        res.succese = true;
        std::cout << "armPlanning模块:夹爪不进行操作!" << std::endl;
    }
    //根据请求消息中的planning_type变量的值确定插值规划还是直线规划
    if (PlanningType(req.planning_type) == InterpolationPlanning) {
        std::cout << "armPlanning模块:进行插值规划!" << std::endl;
        std::shared_ptr<RRTConnect::VectorSixDimension> start_point =
            std::make_shared<RRTConnect::VectorSixDimension>(arm->angles[0],
                                                             arm->angles[1],
                                                             arm->angles[2],
                                                             arm->angles[3],
                                                             arm->angles[4],
                                                             arm->angles[5]);
        std::cout << "armPlanning模块:起始点机械臂坐标为:"
                  << start_point->element1_ << "," << start_point->element2_
                  << "," << start_point->element3_ << ","
                  << start_point->element4_ << "," << start_point->element5_
                  << "," << start_point->element6_ << "," << std::endl;
        //将req请求中的路径点存入way_points，并将机械臂当前位置插入为第一个点
        std::vector<std::shared_ptr<RRTConnect::VectorSixDimension>> way_points;
        way_points.emplace_back(start_point);
        std::vector<RRTConnect::VectorSixDimension> path;
        for (int i = 0; i < req.path_points_number; i++) {
            std::shared_ptr<RRTConnect::VectorSixDimension> way_point =
                std::make_shared<RRTConnect::VectorSixDimension>(
                    0, 0, 0, 0, 0, 0);
            way_point->element1_ = (double) req.planning_points_joint1[i];
            way_point->element2_ = (double) req.planning_points_joint2[i];
            way_point->element3_ = (double) req.planning_points_joint3[i];
            way_point->element4_ = (double) req.planning_points_joint4[i];
            way_point->element5_ = (double) req.planning_points_joint5[i];
            way_point->element6_ = (double) req.planning_points_joint6[i];
            way_points.emplace_back(way_point);
        }
        //检测way_points中相邻路径点的距离是否过小，如果距离过小返回值res.succese置为false
        for (int i = 0; i < (way_points.size() - 1); i++) {
            //当路径点向量中只有两个点时，每个关节差值都小于0.01则返回false
            if (way_points.size() == 2 &&
                abs(way_points[i]->element1_ - way_points[i + 1]->element1_) <
                    0.001 &&
                abs(way_points[i]->element2_ - way_points[i + 1]->element2_) <
                    0.001 &&
                abs(way_points[i]->element3_ - way_points[i + 1]->element3_) <
                    0.001 &&
                abs(way_points[i]->element4_ - way_points[i + 1]->element4_) <
                    0.001 &&
                abs(way_points[i]->element5_ - way_points[i + 1]->element5_) <
                    0.001 &&
                abs(way_points[i]->element6_ - way_points[i + 1]->element6_) <
                    0.001) {
                std::cout << "armPlanning模块:已到达目标点!" << std::endl;
                res.succese = true;
                return true;
            }
            //当路径点向量中大于两个点时，每个关节差值都小于0.05则返回false
            else if (abs(way_points[i]->element1_ -
                         way_points[i + 1]->element1_) < 0.05 &&
                     abs(way_points[i]->element2_ -
                         way_points[i + 1]->element2_) < 0.05 &&
                     abs(way_points[i]->element3_ -
                         way_points[i + 1]->element3_) < 0.05 &&
                     abs(way_points[i]->element4_ -
                         way_points[i + 1]->element4_) < 0.05 &&
                     abs(way_points[i]->element5_ -
                         way_points[i + 1]->element5_) < 0.05 &&
                     abs(way_points[i]->element6_ -
                         way_points[i + 1]->element6_) < 0.05) {
                way_points.erase(way_points.begin() + i);
                std::cout << "armPlanning模块:删除路径点中距离过近的点!"
                          << std::endl;
                if (way_points.size() < 2) {
                    std::cout << "armPlanning模块:有效路径点过少，请重选给定!"
                              << std::endl;
                    res.succese = false;
                    return false;
                }
            }
        }
        //显示路径点信息
        for (int i = 0; i < way_points.size(); i++) {
            std::cout << "armPlanning模块:路径点坐标为:"
                      << way_points[i]->element1_ << ","
                      << way_points[i]->element2_ << ","
                      << way_points[i]->element3_ << ","
                      << way_points[i]->element4_ << ","
                      << way_points[i]->element5_ << ","
                      << way_points[i]->element6_ << "," << std::endl;
        }
        //进行rrt规划，如果rrt规划失败，返回false
        if (!rrt->multiWayPointsPlanning(way_points, path)) {
            res.succese = false;
            std::cout << "armPlanning模块:RRT模块规划失败!" << std::endl;
            return false;
        }
        //将rrt规划的路径转化为为CubicSplinePlanning::multiDimensionPoint形式存入points_input
        std::vector<CubicSplinePlanning::multiDimensionPoint> points_input;
        for (int i = 0; i < path.size(); i++) {
            CubicSplinePlanning::multiDimensionPoint point;
            point[0] = path[i].element1_;
            point[1] = path[i].element2_;
            point[2] = path[i].element3_;
            point[3] = path[i].element4_;
            point[4] = path[i].element5_;
            point[5] = path[i].element6_;
            points_input.emplace_back(point);
        }
        //给路径插入样条曲线，如果插值失败返回false
        if (!csp->initCubicSplineWithControlPoints(points_input)) {
            res.succese = false;
            std::cout << "armPlanning模块:init样条插值模块失败!" << std::endl;
            return false;
        }
        //给定采样数，对插入的样条曲线进行采样
        csp->insertAllJointsCubicSpline(20);
        //设置机械臂模式和状态，模式1表示进行点位伺服控制
        arm->set_mode(1);
        arm->set_state(0);
        double last_time = 0;
        double sleep_time = 0;
        //将csp中采样得到的点位序列存入数组发送给机械臂，根据当前点位对应时间戳确定进程阻塞时间
        for (int i = 0; i < csp->pos_list_.size(); i++) {
            if (arm->is_connected() && arm->state != 4) {
                fp32 angles[7] = {float(csp->pos_list_[i].joints_value[0]),
                                  float(csp->pos_list_[i].joints_value[1]),
                                  float(csp->pos_list_[i].joints_value[2]),
                                  float(csp->pos_list_[i].joints_value[3]),
                                  float(csp->pos_list_[i].joints_value[4]),
                                  float(csp->pos_list_[i].joints_value[5]),
                                  0};
                double sleep_time = csp->pos_list_[i].time - last_time;
                last_time = csp->pos_list_[i].time;
                ros::Duration(sleep_time).sleep();
                int ret = arm->set_servo_angle_j(angles);
            }
        }
        ros::Duration duration(0.01);
        bool flag_arrive_goal = false;
        //循环等待机械臂到达目标点，通过计算最终路径点与机械臂当前关节值的差值是否小于0.01确定是否完成对机械臂的控制，并返回res.succese为true
        while (true) {
            flag_arrive_goal = true;
            for (int i = 0; i < 6; i++) {
                if ((csp->pos_list_.back().joints_value[i] - arm->angles[i]) >
                    0.01) {
                    flag_arrive_goal = false;
                    break;
                }
            }
            if (flag_arrive_goal == true) {
                res.succese = true;
                break;
            }
            duration.sleep();
        }
    }
    //直线插值规划
    else if (PlanningType(req.planning_type) == LinePlanning) {
        std::cout << "armPlanning模块:进行直线规划!" << std::endl;
        CubicSplinePlanning::multiDimensionPoint point_start{arm->angles[0],
                                                             arm->angles[1],
                                                             arm->angles[2],
                                                             arm->angles[3],
                                                             arm->angles[4],
                                                             arm->angles[5]};
        CubicSplinePlanning::multiDimensionPoint point_end{
            (double) req.planning_points_joint1[0],
            (double) req.planning_points_joint2[0],
            (double) req.planning_points_joint3[0],
            (double) req.planning_points_joint4[0],
            (double) req.planning_points_joint5[0],
            (double) req.planning_points_joint6[0]};
        //调用getLineTrajectoryPlanning函数规划直线轨迹
        csp->getLineTrajectoryPlanning(&point_start, &point_end, 10);
        //切换机械臂模式和状态
        arm->set_mode(1);
        arm->set_state(0);
        //将插值的到的点位发送给机械臂进行伺服控制
        for (int i = 0; i < csp->pos_list_.size(); i++) {
            if (arm->is_connected() && arm->state != 4) {
                fp32 angles[7] = {float(csp->pos_list_[i].joints_value[0]),
                                  float(csp->pos_list_[i].joints_value[1]),
                                  float(csp->pos_list_[i].joints_value[2]),
                                  float(csp->pos_list_[i].joints_value[3]),
                                  float(csp->pos_list_[i].joints_value[4]),
                                  float(csp->pos_list_[i].joints_value[5]),
                                  0};
                int ret = arm->set_servo_angle_j(angles);
            }
        }
        ros::Duration duration(0.01);
        //循环等待机械臂到达目标点，通过计算最终路径点与机械臂当前关节值的差值是否小于0.01确定是否完成对机械臂的控制，并返回res.succese为true
        bool flag_arrive_goal = false;
        while (true) {
            flag_arrive_goal = true;
            for (int i = 0; i < 6; i++) {
                if ((csp->pos_list_.back().joints_value[i] - arm->angles[i]) >
                    0.01) {
                    flag_arrive_goal = false;
                    break;
                }
            }
            if (flag_arrive_goal == true) {
                res.succese = true;
                break;
            }
            duration.sleep();
        }
    }
    //未请求对机械臂规划进行
    else {
        res.succese = true;
        std::cout << "armPlanning模块:不进行规划!" << std::endl;
    }
    //返回消息中附带当前机械臂关节位置
    res.current_joint1_angel = arm->angles[0];
    res.current_joint2_angel = arm->angles[1];
    res.current_joint3_angel = arm->angles[2];
    res.current_joint4_angel = arm->angles[3];
    res.current_joint5_angel = arm->angles[4];
    res.current_joint6_angel = arm->angles[5];
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motion_planning");
    ros::NodeHandle nh;
    ArmPlanning ap(nh);
    ros::Rate rate(10);
    while (ros::ok()) {
        sensor_msgs::JointState joints_states_msg;
        joints_states_msg.header.stamp = ros::Time::now();
        joints_states_msg.position.push_back(ap.arm->angles[0]);
        joints_states_msg.position.push_back(ap.arm->angles[1]);
        joints_states_msg.position.push_back(ap.arm->angles[2]);
        joints_states_msg.position.push_back(ap.arm->angles[3]);
        joints_states_msg.position.push_back(ap.arm->angles[4]);
        joints_states_msg.position.push_back(ap.arm->angles[5]);
        ap.joints_states_pub.publish(joints_states_msg);
        ros::spinOnce();
        rate.sleep();
    }
}
