#include "arm_planning/RRTConnect.h"

bool isStateValid(const RRTConnect::MultiDimVector& node)
{
    return true;
}

void rrtTest(int argc, char** argv)
{
    std::vector<double> upper_vector{3.1415926, 3.1415926, 3.1415926, 3.1415926, 3.1415926, 3.1415926};
    std::vector<double> lower_vector{-3.1415926, -3.1415926, -3.1415926, -3.1415926, -3.1415926, -3.1415926};
    double dis_expand = 0.2;
    double dis_optimize = 0.6;
    double goal_sampleRate = 10;
    int max_iteration = 10000;
    std::cout<<"rrtTest"<<std::endl;
    RRTConnect rrt(std::bind(&isStateValid, std::placeholders::_1),
                   upper_vector,
                   lower_vector,
                   dis_expand,
                   dis_optimize,
                   goal_sampleRate,
                   max_iteration);
    std::cout<<"rrtTest"<<std::endl;
    std::vector<RRTConnect::MultiDimVector> path;
    std::vector<RRTConnect::RRTNode*> points_sequences;
    std::vector<double> vecPoint1 = {0, 0, 0, 0, 0, 0};
    std::vector<double> vecPoint2 = {2.0, 2.0, 2.0, 2.0, 2.0, 2.0};

    std::vector<std::shared_ptr<RRTConnect::MultiDimVector>> way_points;
    std::cout<<"rrtTest"<<std::endl;
    way_points.push_back(rrt.generateMultiDimVec(vecPoint1));
    way_points.push_back(rrt.generateMultiDimVec(vecPoint2));
    way_points.push_back(rrt.generateMultiDimVec(vecPoint1));
    std::cout<<"rrtTest"<<std::endl;
    rrt.multiWayPointsPlanning(way_points, path);

}

int main(int argc, char** argv)
{
    rrtTest(argc, argv);
    return 0;
}
