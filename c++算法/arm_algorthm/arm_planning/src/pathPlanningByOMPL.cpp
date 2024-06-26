#include "arm_planning/pathPlanningByOMPL.h"

PathPlanning::PathPlanning()
    : start_(space)
    , goal_(space)
{}

bool PathPlanning::plan(std::function<bool(const ob::State*)> isStateValid,
                        std::vector<double> start,
                        std::vector<double> goal)
{
    // 创建一个6D空间
    // ob::StateSpacePtr space(new ob::RealVectorStateSpace(6));
    // 设置6D空间边界
    ob::RealVectorBounds bounds(6);
    bounds.setLow(-3.141592);
    bounds.setHigh(3.141592);
    bounds.setLow(2, 0);
    bounds.setHigh(2, 3.141592);
    bounds.setLow(4, -1.570796);
    bounds.setHigh(4, 1.570796);
    // space->setBounds(bounds);
    // space->as<ob::RealVectorStateSpace>()->setBounds(-3.141592,3.141592);
    space->as<ob::RealVectorStateSpace>()->setBounds(bounds);
    // 通过状态空间实例化SpaceInformation
    ob::SpaceInformationPtr si(std::make_shared<ob::SpaceInformation>(space));
    // 设置状态有效性检查器StateValidityChecker
    si->setStateValidityChecker(isStateValid);
    si->setup();
    start_->as<ob::RealVectorStateSpace::StateType>()->values[0] = start[0];
    start_->as<ob::RealVectorStateSpace::StateType>()->values[1] = start[1];
    start_->as<ob::RealVectorStateSpace::StateType>()->values[2] = start[2];
    start_->as<ob::RealVectorStateSpace::StateType>()->values[3] = start[3];
    start_->as<ob::RealVectorStateSpace::StateType>()->values[4] = start[4];
    start_->as<ob::RealVectorStateSpace::StateType>()->values[5] = start[5];
    goal_->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal[0];
    goal_->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal[1];
    goal_->as<ob::RealVectorStateSpace::StateType>()->values[2] = goal[2];
    goal_->as<ob::RealVectorStateSpace::StateType>()->values[3] = goal[3];
    goal_->as<ob::RealVectorStateSpace::StateType>()->values[4] = goal[4];
    goal_->as<ob::RealVectorStateSpace::StateType>()->values[5] = goal[5];
    // start_->as<ob::RealVectorStateSpace::StateType>()->values[0] = 0 / 180.0
    // * 3.1415926535897932384626433832795;
    // start_->as<ob::RealVectorStateSpace::StateType>()->values[1] = 0 / 180.0
    // * 3.1415926535897932384626433832795;
    // start_->as<ob::RealVectorStateSpace::StateType>()->values[2] = 0 / 180.0
    // * 3.1415926535897932384626433832795;
    // start_->as<ob::RealVectorStateSpace::StateType>()->values[3] = 0 / 180.0
    // * 3.1415926535897932384626433832795;
    // start_->as<ob::RealVectorStateSpace::StateType>()->values[4] = 0 / 180.0
    // * 3.1415926535897932384626433832795;
    // start_->as<ob::RealVectorStateSpace::StateType>()->values[5] = 0 / 180.0
    // * 3.1415926535897932384626433832795;
    // goal_->as<ob::RealVectorStateSpace::StateType>()->values[0] = 0 / 180.0
    // * 3.1415926535897932384626433832795;
    // goal_->as<ob::RealVectorStateSpace::StateType>()->values[1] = -90 / 180.0
    // * 3.1415926535897932384626433832795;
    // goal_->as<ob::RealVectorStateSpace::StateType>()->values[2] = 0 / 180.0
    // * 3.1415926535897932384626433832795;
    // goal_->as<ob::RealVectorStateSpace::StateType>()->values[3] = 0 / 180.0
    // * 3.1415926535897932384626433832795;
    // goal_->as<ob::RealVectorStateSpace::StateType>()->values[4] = 0 / 180.0
    // * 3.1415926535897932384626433832795;
    // goal_->as<ob::RealVectorStateSpace::StateType>()->values[5] = 0 / 180.0
    // * 3.1415926535897932384626433832795; 实例化规划问题
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));
    // 将起始和目标状态定义到问题中
    pdef->setStartAndGoalStates(start_, goal_);
    pdef->setOptimizationObjective(
        std::make_shared<ob::PathLengthOptimizationObjective>(si));
    // 通过状态空间创建规划器
    auto planner(std::make_shared<og::RRTConnect>(si));
    // 在规划器中设置需要解决的问题
    planner->setProblemDefinition(pdef);
    // 执行设置步骤
    planner->setIntermediateStates(true);
    planner->setRange(0.2);
    planner->setup();
    // 打印设置
    si->printSettings(std::cout);
    pdef->print(std::cout);
    // 执行规划器以解决所设置的问题，其中规划时间设置为3.0秒
    ob::PlannerStatus solved = planner->ob::Planner::solve(100.0);

    if (solved) {
        // 获取规划结果
        // ob::PathPtr path = pdef->getSolutionPath();
        og::PathGeometric* path =
            pdef->getSolutionPath()->as<og::PathGeometric>();
        // paths = path->as<og::PathGeometric>()->getStates();
        std::cout << "path point number=" << path->getStateCount() << std::endl;
        for (size_t i = 0; i < path->getStateCount(); i++) {
            const ob::RealVectorStateSpace::StateType* state =
                path->getState(i)->as<ob::RealVectorStateSpace::StateType>();
            std::vector<double> point;
            for (int j = 0; j < 6; j++) {
                point.push_back(state->values[j]);
            }
            paths.push_back(point);
            // std::cout<<state->values[0]<<","<<state->values[1]<<","<<state->values[2]<<","<<state->values[3]<<","<<state->values[4]<<","<<state->values[5]<<std::endl;
        }
        std::cout << "Found solution:" << std::endl;
        // 打印规划结果
        path->print(std::cout);
    } else
        std::cout << "No solution found" << std::endl;
}
void unitTest()
{
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
    PathPlanning pp;
    // pp.plan();
    std::cout << " " << std::endl;
}
