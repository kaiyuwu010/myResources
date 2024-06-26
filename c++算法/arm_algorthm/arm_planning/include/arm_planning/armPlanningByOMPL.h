#pragma once
#include <functional>

#include "path_planning_based_ompl/collisionDetect.h"
#include "path_planning_based_ompl/inverseKinematics.h"
#include "path_planning_based_ompl/pathPlanningByOMPL.h"
#include "path_planning_based_ompl/trajectoryPlanning.h"
#include "ros/ros.h"
#include "xarm_msgs/Move.h"
class ArmPlanning
{
  private:
    CollisionDetection* cd = new CollisionDetection();
    InverseKinematics* ik = new InverseKinematics();

  public:
    ~ArmPlanning();
    CubicSplinePlanning* csp = new CubicSplinePlanning();
    void plan(const Eigen::Matrix4d& transStart,
              const Eigen::Matrix4d& transGoal);
    bool isStateValid(const ob::State* state);
};
void unitTest();
