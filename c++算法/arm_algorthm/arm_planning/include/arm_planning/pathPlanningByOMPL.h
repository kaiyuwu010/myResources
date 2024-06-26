#pragma once
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/config.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <functional>
#include <vector>
namespace ob = ompl::base;
namespace og = ompl::geometric;

class PathPlanning
{
  private:
    ob::StateSpacePtr space = std::make_shared<ob::RealVectorStateSpace>(6);
    ob::ScopedState<> start_;
    ob::ScopedState<> goal_;

  public:
    // std::vector<ob::State *> paths;
    std::vector<std::vector<double>> paths;
    PathPlanning();
    bool plan(std::function<bool(const ob::State*)> isStateValid,
              std::vector<double> start,
              std::vector<double> goal);
};
void unitTest();
