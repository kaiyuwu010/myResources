#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <list>
#include <vector>
#include <initializer_list>
#include <memory>
// #define PI 3.14159265359

class RRTConnect
{
    public:
        class MultiDimVector
        {
            public:
                std::vector<double> vector_;
                //构造函数设置为私有，实现只有外层类能生成当前类
                MultiDimVector(const std::vector<double>& vec): vector_(vec){}
            public:
                //运算符重载
                bool operator==(const MultiDimVector& vec) const;
                MultiDimVector& operator=(const MultiDimVector& vec);
                MultiDimVector operator+(const MultiDimVector& vec) const;
                MultiDimVector operator-(const MultiDimVector& vec) const;
                double operator*(const MultiDimVector& vec) const;
                MultiDimVector operator*(const double& value) const;
                MultiDimVector operator/(const double& value) const;
                //距离计算
                double calculateEuclideanDis() const
                {
                    double square = 0;
                    for(size_t i=0; i<this->vector_.size(); i++)
                    {
                        square += pow(this->vector_[i], 2);
                    }
                    return sqrt(square);
                }
                double calculateEuclideanDis(const MultiDimVector& vec) const
                {
                    double square = 0;
                    for(size_t i=0; i<this->vector_.size(); i++)
                    {
                        square += pow(this->vector_[i] - vec.vector_[i], 2);
                    }
                    return sqrt(square);            
                }
            //声明外层类为友元
            friend class RRTConnect;
            friend class RRTNode;
            friend class SampleRange;
        };
        class RRTNode
        {
            public:
                MultiDimVector node_vector_;
                double dist_to_origin_;
                std::shared_ptr<RRTNode> parent_node_;
                RRTNode(const MultiDimVector& vector):node_vector_(vector), parent_node_(nullptr), dist_to_origin_(-1.0){}

            public:
                //运算符重载
                bool operator==(const RRTNode& node) const
                {
                    return this->node_vector_ == node.node_vector_;
                }
                bool operator!=(const RRTNode& node) const
                {
                    return !(*this == node);
                }
            //声明外层类为友元
            friend class RRTConnect;
        };
        class SampleRange
        {
            private:
                MultiDimVector upper_limit_;
                MultiDimVector lower_limit_;
            public:
                SampleRange& operator=(const SampleRange& other)
                {
                    if (this != &other)
                    {
                        upper_limit_ = other.upper_limit_;
                        lower_limit_ = other.lower_limit_;
                    }
                    return *this;
                }
                SampleRange(const std::vector<double>& upper_limit, const std::vector<double>& lower_limit):upper_limit_(upper_limit), lower_limit_(lower_limit){}
            //声明外层类为友元
            friend class RRTConnect;
        };
        bool isNodeInSampleRange(const RRTNode& node);

    private:
        size_t dim_;                                                        //维度
        SampleRange sample_range_;                                          //采样范围 x,y ∈ [min,max]
        double dis_expand_;                                                 //扩展步长
        double dis_optimize_;                                               //优化步长
        double goal_sample_rate_;                                           //直接采样目标点的概率
        std::vector<std::shared_ptr<RRTNode>> node_list_1_, node_list_2_;   //正向和反向rrt节点链
        std::shared_ptr<RRTNode> begin_;                          //拓展目标点
        std::shared_ptr<RRTNode> end_;                            //拓展起始点
        int max_iter_;                                                      //最大迭代次数
        std::function<bool(const std::shared_ptr<RRTNode> node)> obstacleFree;     //函数指针判断是否有障碍物

    public:
        RRTConnect(std::function<bool(const std::shared_ptr<RRTConnect::RRTNode> node)> isStateValid,
                   const std::vector<double>& upper_limit,
                   const std::vector<double>& lower_limit,
                   double expand_dis,
                   double dis_optimize,
                   double goal_sample_rate,
                   int max_iter);
        ~RRTConnect();
        //生成多维向量
        std::shared_ptr<MultiDimVector> generateMultiDimVec() const 
        {
            std::vector<double> vec;
            for(size_t i=0; i<dim_; i++)
            {
                vec.push_back(0);
            }
            std::shared_ptr<MultiDimVector> multiDimVec = std::make_shared<MultiDimVector>(vec);
            return multiDimVec;
        }
        std::shared_ptr<MultiDimVector> generateMultiDimVec(std::vector<double>& vec) const 
        {
            if(vec.size() != dim_)
            {
                throw std::invalid_argument("RRTConnect::generateMultiDimVec: 生成的向量维度与rrt不一致!");
            }
            std::shared_ptr<MultiDimVector> multiDimVec = std::make_shared<MultiDimVector>(vec);
            return multiDimVec;
        }
        //生成RRT节点
        std::shared_ptr<RRTNode> generateRRTNode() const 
        {
            std::vector<double> vec;
            for(size_t i=0; i<dim_; i++)
            {
                vec.push_back(0);
            }
            std::shared_ptr<RRTNode> RRTNode = this->generateRRTNode(vec);
            return RRTNode;
        }
        std::shared_ptr<RRTNode> generateRRTNode(const MultiDimVector& vector) const 
        {
            if(vector.vector_.size() != dim_)
            {
                throw std::invalid_argument("RRTConnect::generateMultiDimVec: 生成的向量维度与rrt不一致!");
            }
            MultiDimVector multiDimVector(vector);
            std::shared_ptr<RRTNode> rrtNode = std::make_shared<RRTNode>(multiDimVector);
            return rrtNode;
        }
        //设置起始点和终点
        void setBegin(std::shared_ptr<MultiDimVector> begin)
        {
            begin_->node_vector_ = *begin;
            begin_->dist_to_origin_ = 0;
            begin_->parent_node_ = nullptr;
            if (!isNodeInSampleRange(*begin_)) {
                throw std::invalid_argument("RRT运动规划模块:节点不在采样范围，请重新给起始点！");
            }
        }
        void setEnd(std::shared_ptr<MultiDimVector> end)
        {
            end_->node_vector_ = *end;
            end_->dist_to_origin_ = 0;
            end_->parent_node_ = nullptr;
            if (!isNodeInSampleRange(*end_)) {
                throw std::invalid_argument("RRT运动规划模块:节点不在采样范围，请重新给终点！");
            }
        }
        //采样生成节点
        void randomSampling(std::shared_ptr<RRTNode> &new_node, bool sample_target, bool from_start);  
        //计算最近的节点
        size_t getNearestNodeIndex(const std::vector<std::shared_ptr<RRTNode>> &node_list, const std::shared_ptr<RRTNode> &rnd_node);  
        //计算两个节点间的距离和方位角
        MultiDimVector calculateDirectionVector(const std::shared_ptr<RRTNode> &from_node, const std::shared_ptr<RRTNode> &to_node, double& length);  
        //连线方向扩展固定步长生成节点
        std::shared_ptr<RRTNode> steer(std::shared_ptr<RRTNode> from_node,std::shared_ptr<RRTNode> to_node, double extend_length = std::numeric_limits<double>::max());  
        //计算两点的距离
        double calDistTwoNode(const std::shared_ptr<RRTNode> node1, const std::shared_ptr<RRTNode> node2);  
        //找出邻近节点
        void getNearestNodesIndex(std::vector<std::shared_ptr<RRTNode>> node_list, std::shared_ptr<RRTNode> new_node, double radius, std::vector<std::pair<int, double>>& found_nodes);  
        void caculateDiffVector(MultiDimVector& A, MultiDimVector& B, MultiDimVector& diff_vector, int shrink_times);
        double cosineOfIncludedAngle(MultiDimVector& A, MultiDimVector& B);
        //重布线
        void rewire(std::vector<std::shared_ptr<RRTNode>>& node_list, std::shared_ptr<RRTNode> new_node);  
        //细部顺滑
        void smoothPathNarrowly(std::vector<std::shared_ptr<RRTConnect::RRTNode>>& path, int times_input);  
        //广域优化
        void smoothPathBroadly(std::vector<std::shared_ptr<RRTConnect::RRTNode>>& path);  
        //生成节点组成的路径
        std::vector<std::shared_ptr<RRTConnect::RRTNode>> generatePathNodesFromNodesList(int nearst_index, bool link_to_end_tree);  
        //进行路径规划
        std::vector<std::shared_ptr<RRTConnect::RRTNode>> planning(); 
        //进行多点路径规划 
        bool multiWayPointsPlanning(const std::vector<std::shared_ptr<MultiDimVector>>& way_points, std::vector<std::shared_ptr<RRTConnect::RRTNode>>& output_path);  
};
