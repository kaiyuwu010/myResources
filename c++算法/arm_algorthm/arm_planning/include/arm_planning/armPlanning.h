#pragma once
#include <sensor_msgs/JointState.h>

#include <functional>

#include "arm_planning/RRTConnect.h"
#include "arm_planning/arm_interactive.h"
#include "arm_planning/collisionDetect.h"
#include "arm_planning/inverseKinematics.h"
#include "arm_planning/trajectoryPlanning.h"
#include "ros/ros.h"
#include "xarm/wrapper/xarm_api.h"

class ArmPlanning
{
  public:
    enum PlanningType
    {
        NoPlanning = 0,
        InterpolationPlanning = 1,
        LinePlanning = 2
    };
    enum ClampOperation
    {
        NoClampOperation = 0,
        Open = 1,
        Close = 2
    };

  private:
    std::shared_ptr<CollisionDetection> cd = std::make_shared<CollisionDetection>();
    std::shared_ptr<InverseKinematics> ik = std::make_shared<InverseKinematics>();
    std::shared_ptr<RRTConnect> rrt;

  public:
    std::shared_ptr<XArmAPI> arm;
    std::shared_ptr<RRTConnect::RRTNode> beginPtr = std::make_shared<RRTConnect::RRTNode>(0, 0, 0, 0, 0, 0);
    std::shared_ptr<RRTConnect::RRTNode> endPtr = std::make_shared<RRTConnect::RRTNode>(0, 0, 0, 0, 0, 0);
    std::vector<std::shared_ptr<RRTConnect::RRTNode>> waypoints;
    unsigned char send_close_data[9] =
        {0x55, 0xAA, 0x04, 0x01, 0x21, 0x37, 0x50, 0x04, 0xB1};  //关闭夹爪
    unsigned char send_open_data[9] =
        {0x55, 0xAA, 0x04, 0x01, 0x21, 0x37, 0x00, 0x00, 0x5D};  //打開夾爪
    unsigned char clear_error_data[9] =
        {0x55, 0xAA, 0x03, 0x01, 0x04, 0x00, 0x1E, 0x26};  //清除错误

    int clamp_ret;
    unsigned char ret_data[22] = {0};
    bool updated_begin_node_ = false;
    ros::NodeHandle nh_;
    ros::ServiceServer server_move_to_goal;
    ros::Publisher joints_states_pub;

    ArmPlanning(ros::NodeHandle nh);
    void setupComm();
    ~ArmPlanning();
    std::shared_ptr<CubicSplinePlanning> csp = std::make_shared<CubicSplinePlanning>(2);
    bool isStateValid(const std::vector<double>& node);
    bool moveToGoalCallback(arm_planning::arm_interactive::Request& req,
                            arm_planning::arm_interactive::Response& res);
};
