#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <list>
#include <vector>

#include "arm_planning/collisionDetect.h"
// #define PI 3.14159265359

class RRTConnect
{
  public:
    struct VectorSixDimension
    {
        double element1_, element2_, element3_, element4_, element5_, element6_;
        VectorSixDimension();
        VectorSixDimension(double element_1,
                           double element_2,
                           double element_3,
                           double element_4,
                           double element_5,
                           double element_6);
        bool operator==(const VectorSixDimension& vector) const;
        VectorSixDimension& operator=(const VectorSixDimension& vector);
        VectorSixDimension operator+(const VectorSixDimension& vector) const;
        VectorSixDimension operator-(const VectorSixDimension& vector) const;
        double operator*(const VectorSixDimension& vector) const;
        RRTConnect::VectorSixDimension operator*(const double& value) const;
        VectorSixDimension operator/(const double& value) const;
        double calculateEuclideanDis() const;
        double calculateEuclideanDis(const VectorSixDimension& vector) const;
        std::vector<double> transformToVectorAndAdjustAngle();
    };
    struct RRTNode
    {
        VectorSixDimension joint_vector_;
        RRTNode();
        RRTNode(const VectorSixDimension& vector);
        RRTNode(double joint1,
                double joint2,
                double joint3,
                double joint4,
                double joint5,
                double joint6);
        std::shared_ptr<RRTNode> parent_;
        double dist_to_origin_;
        bool operator==(const RRTNode& node) const;
        bool operator!=(const RRTNode& node) const;
    };
    struct SampleRange
    {
        VectorSixDimension upper_limit_;
        VectorSixDimension lower_limit_;
        SampleRange(const VectorSixDimension& upper_limit,
                    const VectorSixDimension& lower_limit)
        {
            upper_limit_ = upper_limit;
            lower_limit_ = lower_limit;
        }
    };
    bool isNodeInSampleRange(const std::shared_ptr<RRTNode> node);

  private:
    SampleRange sample_range_;  //采样区域 x,y ∈ [min,max]
    double dis_expand_;         //扩展步长
    double dis_optimize_;       //优化步长
    double goal_sample_rate_;   //采样目标点的概率
    std::vector<std::shared_ptr<RRTNode>> node_list_1_, node_list_2_;
    std::shared_ptr<RRTNode> begin_;
    std::shared_ptr<RRTNode> end_;
    int max_iter_;
    std::function<bool(const std::vector<double>& node)>
        obstacleFree;  //函数指针判断是否有障碍物

  public:
    RRTConnect(
        std::function<bool(const std::vector<double>& node)> isStateValid,
        const SampleRange& sample_range,
        double expand_dis,
        double dis_optimize,
        double goal_sample_rate,
        int max_iter);
    ~RRTConnect();
    int setBegin(std::shared_ptr<VectorSixDimension> begin);
    int setEnd(std::shared_ptr<VectorSixDimension> End);

    std::shared_ptr<RRTNode> randomSampling(bool sample_object_node,
                                        bool from_start);  //采样生成节点
    int getNearestNodeIndex(const std::vector<std::shared_ptr<RRTNode>> node_list,
                            const std::shared_ptr<RRTNode> rnd_node);  //计算最近的节点
    VectorSixDimension calculateDirectionVector(
        const std::shared_ptr<RRTNode> from_node,
        const std::shared_ptr<RRTNode> to_node,
        double& length);  // 计算两个节点间的距离和方位角
    std::shared_ptr<RRTNode> steer(
        std::shared_ptr<RRTNode> from_node,
        std::shared_ptr<RRTNode> to_node,
        double extend_length =
            std::numeric_limits<double>::max());  //连线方向扩展固定步长生成节点
    double calDistTwoNode(const std::shared_ptr<RRTNode> node1,
                          const std::shared_ptr<RRTNode> node2);  //计算两点的距离

    void getNearestNodesIndex(
        std::vector<std::shared_ptr<RRTNode>> node_list,
        std::shared_ptr<RRTNode> new_node,
        double radius,
        std::vector<std::pair<int, double>>& found_nodes);  //计算半径内的节点
    void caculateDiffVector(VectorSixDimension& A,
                            VectorSixDimension& B,
                            VectorSixDimension& diff_vector,
                            int shrink_times);
    double cosineOfIncludedAngle(VectorSixDimension& A,
                                 VectorSixDimension& B);
    void rewire(std::vector<std::shared_ptr<RRTNode>>& node_list,
                std::shared_ptr<RRTNode> new_node);  //重布线
    void smoothPathNarrowly(std::vector<VectorSixDimension>& path,
                            int times_input);  //细部优化
    void smoothPathBroadly(
        std::vector<VectorSixDimension>& path);  //广域优化
    std::vector<VectorSixDimension> generatePathNodesFromNodesList(
        int nearst_index,
        bool link_to_end_tree);  //生成节点组成的路径

    std::vector<VectorSixDimension> planning();  //进行路径规划
    bool multiWayPointsPlanning(
        const std::vector<std::shared_ptr<VectorSixDimension>>& way_points,
        std::vector<VectorSixDimension>& output_path);  //进行多点路径规划
};
