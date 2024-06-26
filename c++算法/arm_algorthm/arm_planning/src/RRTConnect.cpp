#include "arm_planning/RRTConnect.h"

// VectorSixDimension类的构造函数及运算符重载函数
RRTConnect::VectorSixDimension::VectorSixDimension(){};
RRTConnect::VectorSixDimension::VectorSixDimension(double element1,
                                                   double element2,
                                                   double element3,
                                                   double element4,
                                                   double element5,
                                                   double element6)
    : element1_(element1)
    , element2_(element2)
    , element3_(element3)
    , element4_(element4)
    , element5_(element5)
    , element6_(element6)
{}
bool RRTConnect::VectorSixDimension::operator==(
    const RRTConnect::VectorSixDimension& vector) const
{
    return this->element1_ == vector.element1_ &&
           this->element2_ == vector.element2_ &&
           this->element3_ == vector.element3_ &&
           this->element4_ == vector.element4_ &&
           this->element5_ == vector.element5_ &&
           this->element6_ == vector.element6_;
}
RRTConnect::VectorSixDimension& RRTConnect::VectorSixDimension::operator=(
    const RRTConnect::VectorSixDimension& vector)
{
    this->element1_ = vector.element1_;
    this->element2_ = vector.element2_;
    this->element3_ = vector.element3_;
    this->element4_ = vector.element4_;
    this->element5_ = vector.element5_;
    this->element6_ = vector.element6_;
    return *this;
}
RRTConnect::VectorSixDimension RRTConnect::VectorSixDimension::operator+(
    const RRTConnect::VectorSixDimension& vector) const
{
    VectorSixDimension new_vector;
    new_vector.element1_ = this->element1_ + vector.element1_;
    new_vector.element2_ = this->element2_ + vector.element2_;
    new_vector.element3_ = this->element3_ + vector.element3_;
    new_vector.element4_ = this->element4_ + vector.element4_;
    new_vector.element5_ = this->element5_ + vector.element5_;
    new_vector.element6_ = this->element6_ + vector.element6_;
    return new_vector;
}
RRTConnect::VectorSixDimension RRTConnect::VectorSixDimension::operator-(
    const RRTConnect::VectorSixDimension& vector) const
{
    VectorSixDimension new_vector;
    new_vector.element1_ = this->element1_ - vector.element1_;
    new_vector.element2_ = this->element2_ - vector.element2_;
    new_vector.element3_ = this->element3_ - vector.element3_;
    new_vector.element4_ = this->element4_ - vector.element4_;
    new_vector.element5_ = this->element5_ - vector.element5_;
    new_vector.element6_ = this->element6_ - vector.element6_;
    return new_vector;
}
double RRTConnect::VectorSixDimension::operator*(
    const RRTConnect::VectorSixDimension& vector) const
{
    double element1 = this->element1_ * vector.element1_;
    double element2 = this->element2_ * vector.element2_;
    double element3 = this->element3_ * vector.element3_;
    double element4 = this->element4_ * vector.element4_;
    double element5 = this->element5_ * vector.element5_;
    double element6 = this->element6_ * vector.element6_;
    return element1 + element2 + element3 + element4 + element5 + element6;
}
RRTConnect::VectorSixDimension RRTConnect::VectorSixDimension::operator*(
    const double& value) const
{
    VectorSixDimension new_vector;
    new_vector.element1_ = this->element1_ * value;
    new_vector.element2_ = this->element2_ * value;
    new_vector.element3_ = this->element3_ * value;
    new_vector.element4_ = this->element4_ * value;
    new_vector.element5_ = this->element5_ * value;
    new_vector.element6_ = this->element6_ * value;
    return new_vector;
}
RRTConnect::VectorSixDimension RRTConnect::VectorSixDimension::operator/(
    const double& value) const
{
    double element1 = this->element1_ / value;
    double element2 = this->element2_ / value;
    double element3 = this->element3_ / value;
    double element4 = this->element4_ / value;
    double element5 = this->element5_ / value;
    double element6 = this->element6_ / value;
    VectorSixDimension new_vector(
        element1, element2, element3, element4, element5, element6);
    return new_vector;
}
// VectorSixDimension计算欧式距离函数
double RRTConnect::VectorSixDimension::calculateEuclideanDis() const
{
    return sqrt(pow(this->element1_, 2) + pow(this->element2_, 2) +
                pow(this->element3_, 2) + pow(this->element4_, 2) +
                pow(this->element5_, 2) + pow(this->element6_, 2));
}
//两个VectorSixDimension对象之间计算欧式距离函数
double RRTConnect::VectorSixDimension::calculateEuclideanDis(
    const RRTConnect::VectorSixDimension& vector) const
{
    return sqrt(pow(this->element1_ - vector.element1_, 2) +
                pow(this->element2_ - vector.element2_, 2) +
                pow(this->element3_ - vector.element3_, 2) +
                pow(this->element4_ - vector.element4_, 2) +
                pow(this->element5_ - vector.element5_, 2) +
                pow(this->element6_ - vector.element6_, 2));
}
// VectorSixDimension类型转化为vector类型，并把角度调整为lite6机械臂算运动学时对应的角度
std::vector<double>
RRTConnect::VectorSixDimension::transformToVectorAndAdjustAngle()
{
    std::vector<double> vec{this->element1_,
                            this->element2_ - 1.570796327,
                            this->element3_ - 1.570796327,
                            this->element4_,
                            this->element5_,
                            this->element6_};
    return vec;
}
// RRTNode类的构造函数和运算符重载函数
RRTConnect::RRTNode::RRTNode()
    : parent_(nullptr)
    , dist_to_origin_(-1.0)
{
    VectorSixDimension vector(0, 0, 0, 0, 0, 0);
    this->joint_vector_ = vector;
};
RRTConnect::RRTNode::RRTNode(const VectorSixDimension& vector)
    : joint_vector_(vector)
    , parent_(nullptr)
    , dist_to_origin_(-1.0)
{}
RRTConnect::RRTNode::RRTNode(double joint1,
                             double joint2,
                             double joint3,
                             double joint4,
                             double joint5,
                             double joint6)
    : parent_(nullptr)
    , dist_to_origin_(-1.0)
{
    this->joint_vector_.element1_ = joint1;
    this->joint_vector_.element2_ = joint2;
    this->joint_vector_.element3_ = joint3;
    this->joint_vector_.element4_ = joint4;
    this->joint_vector_.element5_ = joint5;
    this->joint_vector_.element6_ = joint6;
}
bool RRTConnect::RRTNode::operator==(const RRTConnect::RRTNode& node) const
{
    return this->joint_vector_ == node.joint_vector_;
}
bool RRTConnect::RRTNode::operator!=(const RRTConnect::RRTNode& node) const
{
    return !(*this == node);
}
// RRTConnect类构造函数，参数需要碰撞检测函数、采样范围、拓展步长、优化步长、直接采样目标点概率、最大迭代次数
RRTConnect::RRTConnect(
    std::function<bool(const std::vector<double>& node)> isStateValid,
    const SampleRange& sample_range,
    double expand_dis,
    double dis_optimize,
    double goal_sample_rate,
    int max_iter)
    : obstacleFree(isStateValid)
    , sample_range_(sample_range)
    , dis_expand_(expand_dis)
    , goal_sample_rate_(goal_sample_rate)
    , max_iter_(max_iter)
    , dis_optimize_(dis_optimize)
{
    begin_ = std::make_shared<RRTConnect::RRTNode>(0, 0, 0, 0, 0, 0);
    end_ = std::make_shared<RRTConnect::RRTNode>(0, 0, 0, 0, 0, 0);
}
//随机采样函数，再给点范围进行随机采样
std::shared_ptr<RRTConnect::RRTNode> RRTConnect::randomSampling(
    bool sample_object_node,
    bool from_start)
{
    if (sample_object_node &&
        rand() % (100) <
            goal_sample_rate_)  // srand随机数种子不能重复定义，否则会造成随机数重复，
                                // 这里高频调用sampleFree，应该把随机数种子放在外层
    {
        if (from_start) {
            return end_;
        } else {
            return begin_;
        }
    }
    std::shared_ptr<RRTNode> node = std::make_shared<RRTNode>();
    VectorSixDimension vector =
        sample_range_.upper_limit_ - sample_range_.lower_limit_;
    vector.element1_ =
        rand() / double(RAND_MAX) * vector.element1_ +
        sample_range_.lower_limit_
            .element1_;  // rand()/double(RAND_MAX)产生0~1之间的浮点数
    vector.element2_ = rand() / double(RAND_MAX) * vector.element2_ +
                       sample_range_.lower_limit_.element2_;
    vector.element3_ = rand() / double(RAND_MAX) * vector.element3_ +
                       sample_range_.lower_limit_.element3_;
    vector.element4_ = rand() / double(RAND_MAX) * vector.element4_ +
                       sample_range_.lower_limit_.element4_;
    vector.element5_ = rand() / double(RAND_MAX) * vector.element5_ +
                       sample_range_.lower_limit_.element5_;
    vector.element6_ = rand() / double(RAND_MAX) * vector.element6_ +
                       sample_range_.lower_limit_.element6_;
    node->joint_vector_ = vector;
    // std::cout<<"RRT路径规划模块:randomSampling:采样结果:
    // "<<node->joint_vector_.element1_<<" "<<node->joint_vector_.element2_<<"
    // "<<node->joint_vector_.element3_<<" "<<node->joint_vector_.element4_<<"
    // "<<node->joint_vector_.element5_<<"
    // "<<node->joint_vector_.element6_<<std::endl;
    return node;
}
// RRTConnect类析构函数
RRTConnect::~RRTConnect()
{}

//输入拓展树节点向量和RRTNode节点，找到距离RRTNode节点最近的点
int RRTConnect::getNearestNodeIndex(
    const std::vector<std::shared_ptr<RRTConnect::RRTNode>> node_list,
    const std::shared_ptr<RRTConnect::RRTNode> node)
{
    int min_index = -1;
    double dis_min = std::numeric_limits<double>::max();
    for (int i = 0; i < node_list.size(); i++) {
        VectorSixDimension search_node = node_list[i]->joint_vector_;
        double dist = search_node.calculateEuclideanDis(node->joint_vector_);
        if (dis_min > dist) {
            dis_min = dist;
            min_index = i;
        }
    }
    return min_index;
}
//给定两个节点计算起始节点到目标节点的方向向量和距离
RRTConnect::VectorSixDimension RRTConnect::calculateDirectionVector(
    const std::shared_ptr<RRTConnect::RRTNode> from_node,
    const std::shared_ptr<RRTConnect::RRTNode> to_node,
    double& length)  //两个点的值不能一样
{
    if (from_node == to_node) {
        std::cout << "RRT路径规划模块:"
                     "calculateDirectionVector函数输入了两个相同的node!"
                  << std::endl;
        return from_node->joint_vector_;
    }
    VectorSixDimension diff_joint =
        to_node->joint_vector_ - from_node->joint_vector_;
    length = diff_joint.calculateEuclideanDis();
    VectorSixDimension new_vector = diff_joint / length;
    return new_vector;
}
//给定起始节点和目标节点，往目标节点的方向拓展一定步长的距离生成新节点，指定新节点父节点为from_node并计算出新节点到拓展树根节点的距离
std::shared_ptr<RRTConnect::RRTNode> RRTConnect::steer(
    std::shared_ptr<RRTConnect::RRTNode> from_node,
    std::shared_ptr<RRTConnect::RRTNode> to_node,
    double extend_length)  //生成节点
{
    double length = 0;
    VectorSixDimension direction_element =
        calculateDirectionVector(from_node, to_node, length);
    std::shared_ptr<RRTNode> new_node =
        std::make_shared<RRTNode>(0, 0, 0, 0, 0, 0);
    new_node->parent_ = from_node;
    new_node->dist_to_origin_ = from_node->dist_to_origin_ + extend_length;
    new_node->joint_vector_ =
        from_node->joint_vector_ + (direction_element * extend_length);
    if (!isNodeInSampleRange(new_node))  //沿方向取的点超过范围则直接取to_node点
    {
        std::cout << "RRT路径规划模块:超过采样范围取随机点" << std::endl;
        new_node->parent_ = from_node;
        new_node->dist_to_origin_ = from_node->dist_to_origin_ + length;
        new_node->joint_vector_ = to_node->joint_vector_;
    }
    return new_node;
}
//给定拓展树向量node_list、节点和搜索半径，获取给定节点最近节点集合在向量的序号和到给定节点的距离
void RRTConnect::getNearestNodesIndex(
    std::vector<std::shared_ptr<RRTConnect::RRTNode>> node_list,
    std::shared_ptr<RRTConnect::RRTNode> node,
    double radius,
    std::vector<std::pair<int, double>>&
        found_nodes)  //输出在随机拓展树容器找到的最近点的序号和距离found_nodes
{
    for (int i = 0; i < node_list.size(); i++) {
        std::shared_ptr<RRTNode> node = node_list[i];
        double distance =
            node->joint_vector_.calculateEuclideanDis(node->joint_vector_);
        if (distance <= radius) {
            found_nodes.push_back(std::pair<int, double>(i, distance));
        }
    }
}
//设置rrt拓展起点
int RRTConnect::setBegin(std::shared_ptr<RRTConnect::VectorSixDimension> Begin)
{
    begin_->joint_vector_ = *Begin;
    begin_->dist_to_origin_ = 0;
    begin_->parent_ = nullptr;
    if (!isNodeInSampleRange(begin_)) {
        std::cout << "RRT路径规划模块:节点不在采样范围，请重新给起始点！"
                  << std::endl;
        return 0;
    }
}
//设置rrt拓展终点
int RRTConnect::setEnd(std::shared_ptr<RRTConnect::VectorSixDimension> End)
{
    end_->joint_vector_ = *End;
    end_->parent_ = nullptr;
    if (!isNodeInSampleRange(end_)) {
        std::cout << "RRT路径规划模块:节点不在采样范围，请重新给终点！"
                  << std::endl;
        return 0;
    }
}
//计算两个节点间的拓展距离
double RRTConnect::calDistTwoNode(
    const std::shared_ptr<RRTConnect::RRTNode> node1,
    const std::shared_ptr<RRTConnect::RRTNode> node2)
{
    return node1->joint_vector_.calculateEuclideanDis(node2->joint_vector_);
}
//给定拓展树中连接成功的节点的序号，生成连接成功的路径
std::vector<RRTConnect::VectorSixDimension>
RRTConnect::generatePathNodesFromNodesList(int nearst_index,
                                           bool link_to_end_tree)
{
    std::vector<RRTConnect::VectorSixDimension> nodes_path, temp_path;
    std::shared_ptr<RRTNode> node;
    // link_to_end_tree为true表示拓展树node_list_1_拓展的节点连接到了node_list_2_，此时node_list_1_最后一个节点就是路径上的点
    if (link_to_end_tree == true) {
        node = node_list_1_[node_list_1_.size() - 1];
    }
    //若是node_list_2_生成的节点连接到了node_list_1_，则nearst_index表示路径上的点
    else {
        node = node_list_1_[nearst_index];
    }
    //从node_list_1中找出路径点
    while (node->parent_ != nullptr) {
        temp_path.push_back(node->joint_vector_);
        node = node->parent_;
    }
    temp_path.push_back(node->joint_vector_);
    //路径点放入nodes_path
    for (int i = temp_path.size() - 1; i > -1; i--) {
        nodes_path.push_back(temp_path[i]);
    }
    temp_path.clear();
    //从node_list_2中找出路径点
    if (link_to_end_tree == true) {
        node = node_list_2_[nearst_index];
    } else {
        node = node_list_2_[node_list_2_.size() - 1];
    }
    while (node->parent_ != nullptr) {
        temp_path.push_back(node->joint_vector_);
        node = node->parent_;
    }
    temp_path.push_back(node->joint_vector_);
    // node_list_2_路径点追加到nodes_path
    for (int i = 0; i < temp_path.size(); i++) {
        nodes_path.push_back(node->joint_vector_);
    }
    // for(int i=0; i<nodes_path.size(); i++)
    // {
    //     std::cout<<"最终未优化节点路径:  "<<nodes_path[i].element1_<<"
    //     "<<nodes_path[i].element2_<<"  "<<nodes_path[i].element3_<<"
    //     "<<nodes_path[i].element4_<<"  "<<nodes_path[i].element5_<<"
    //     "<<nodes_path[i].element6_<<std::endl;
    // }
    return nodes_path;
}

void RRTConnect::rewire(
    std::vector<std::shared_ptr<RRTConnect::RRTNode>>& node_list,
    std::shared_ptr<RRTConnect::RRTNode> new_node)
{
    //重选父节点
    std::vector<std::pair<int, double>> found_nodes;
    getNearestNodesIndex(node_list, new_node, 1.5 * dis_expand_, found_nodes);
    double dist_shortest = std::numeric_limits<double>::max();
    int index = -1;
    for (int i = 0; i < found_nodes.size(); i++) {
        double temp = node_list[found_nodes[i].first]->dist_to_origin_ +
                      found_nodes[i].second;
        if (temp < dist_shortest) {
            dist_shortest = temp;
            index = found_nodes[i].first;
        }
    }
    new_node->parent_ = node_list[index];
    new_node->dist_to_origin_ = dist_shortest;
    //重选子节点
    for (int i = 0; i < found_nodes.size(); i++) {
        double temp = dist_shortest + found_nodes[i].second;
        if (node_list[found_nodes[i].first]->dist_to_origin_ > temp) {
            node_list[found_nodes[i].first]->parent_ = new_node;
            node_list[found_nodes[i].first]->dist_to_origin_ = temp;
        }
    }
}
//判断节点是否在给定的采样范围
bool RRTConnect::isNodeInSampleRange(const std::shared_ptr<RRTNode> node)
{
    if (node->joint_vector_.element1_ > sample_range_.upper_limit_.element1_) {
        return false;
    } else if (node->joint_vector_.element2_ >
               sample_range_.upper_limit_.element2_) {
        return false;
    } else if (node->joint_vector_.element3_ >
               sample_range_.upper_limit_.element3_) {
        return false;
    } else if (node->joint_vector_.element4_ >
               sample_range_.upper_limit_.element4_) {
        return false;
    } else if (node->joint_vector_.element5_ >
               sample_range_.upper_limit_.element5_) {
        return false;
    } else if (node->joint_vector_.element6_ >
               sample_range_.upper_limit_.element6_) {
        return false;
    } else if (node->joint_vector_.element1_ <
               sample_range_.lower_limit_.element1_) {
        return false;
    } else if (node->joint_vector_.element2_ <
               sample_range_.lower_limit_.element2_) {
        return false;
    } else if (node->joint_vector_.element3_ <
               sample_range_.lower_limit_.element3_) {
        return false;
    } else if (node->joint_vector_.element4_ <
               sample_range_.lower_limit_.element4_) {
        return false;
    } else if (node->joint_vector_.element5_ <
               sample_range_.lower_limit_.element5_) {
        return false;
    } else if (node->joint_vector_.element6_ <
               sample_range_.lower_limit_.element6_) {
        return false;
    } else {
        return true;
    }
}
//输入路径向量，对路径进行局部顺滑，times_input表示顺滑次数(TODO:检测新顺滑点是否在采样空间)
void RRTConnect::smoothPathNarrowly(
    std::vector<RRTConnect::VectorSixDimension>& path,
    int times_input)  //输入的path点数量大于3，path大小不能变
{
    VectorSixDimension new_node(0, 0, 0, 0, 0, 0);
    int a = path.size() - 1;
    //如果路径点数量小于4个不进行局部顺滑
    if (a < 4) {
        std::cout << "RRT路径规划模块:smoothPathNarrowly函数输入路径点小于3个,"
                     "请重新输入！"
                  << std::endl;
        return;
    }
    // index向量表示不能变动的路径点的序号的几何，后续会对这些固定点之间的路径进行平滑处理。路径首尾点是固定点，所以先存入首尾点的序号
    std::vector<int> index{0, a};
    for (int times_loop = 0; times_loop < times_input; times_loop++) {
        sort(
            index.begin(),
            index
                .end());  //对序号进行排序，因为需要按顺序对每个固定点间的路径进行平滑处理
        int size_index =
            index.size() -
            1;  // size_index表示不能变动的路径点，后续会把接近障碍物的点的序号放入向量
        for (int j = 0; j < size_index; j++)  // j表示第几个固定点
        {
            for (
                int i = index[j] + 1; i < index[j + 1];
                i++)  // i表示第j个和j+1个固定点中间的路径点的序号，所以第一个为index[j]+1最后一个要小于index[j+1]
            {
                new_node =
                    path[i - 1] + (path[i + 1] - path[i - 1]) *
                                      0.5;  //计算第i个路径点两端的路径点的中点
                if (obstacleFree(
                        new_node
                            .transformToVectorAndAdjustAngle()))  //检查新生成的中点是否发生碰撞
                {
                    path[i] = new_node;  //不发生碰撞就用新点替换原来的路径点
                } else {
                    index.push_back(
                        i +
                        1);  //发生碰撞的点的序号放入index容器，表示这个点不能进行平滑处理，当成固定点
                }
            }
        }
    }
}
//对路径进行大范围的顺滑
void RRTConnect::smoothPathBroadly(
    std::vector<RRTConnect::VectorSixDimension>& path)  // path是NX6矩阵
{
    std::vector<int> turning_point_index;  //转折点序号
    turning_point_index.emplace_back(0);
    VectorSixDimension direction_begin;  //存放路径段起始方向
    VectorSixDimension direction_end;    //存放路径段结束处的方向
    bool flag_init_begin = true;         //路径段首次顺滑处理标志
    double last_cosine_value = 1;        //上一次计算出的余弦值
    for (int i = 0; i < path.size() - 1; i++)  //找转折点
    {
        if (!flag_init_begin) {
            direction_end =
                path[i + 1] -
                path[i];  //计算当前路径点的方向（当前点到下一个路径点的方向）
            double temp = cosineOfIncludedAngle(
                direction_begin,
                direction_end);  //计算当前路径点方向与起始方向的夹角余弦
            if (temp <=
                last_cosine_value)  //两方向间的夹角变大也就是余弦值temp变小时，更新last_cosine_value为temp
            {
                last_cosine_value = temp;
            } else if (
                temp > 0.8 ||
                abs(temp - last_cosine_value) <
                    0.01)  //方向变动不大，更新最大夹角(最小cos值)last_cosine_value
            {
                last_cosine_value = temp;
            } else  //找到转折点
            {
                // std::cout<<"RRT路径规划模块:last_cosine_value:
                // "<<last_cosine_value<<std::endl;
                // std::cout<<"RRT路径规划模块:temp: "<<temp<<std::endl;
                // std::cout<<"RRT路径规划模块:index: "<<i<<std::endl;
                // std::cout<<"RRT路径规划模块:end_direction:
                // "<<direction_end.joint1<<" "<<direction_end.joint2<<"
                // "<<direction_end.joint3<<" "<<direction_end.joint4<<"
                // "<<direction_end.joint5<<" "<<direction_end.joint6<<"
                // "<<std::endl;
                turning_point_index.emplace_back(i);
                last_cosine_value = 1;
                flag_init_begin = true;
            }
        } else  //对路径段进行首次顺滑时记录路径方向
        {
            direction_begin = path[i + 1] - path[i];
            flag_init_begin = false;
        }
    }
    turning_point_index.emplace_back(path.size() - 1);  //首尾点都加入转折点向量
    std::cout << "RRT路径规划模块:转折点：" << std::endl;
    for (int j = 0; j < turning_point_index.size(); j++) {
        std::cout << turning_point_index[j] << " ";
    }
    std::cout << std::endl;
    int num_turning_points = turning_point_index.size();
    for (int i = 0; i < num_turning_points - 1; i++)  //回缩处理非转折点
    {
        direction_begin =
            path[turning_point_index[i + 1]] -
            path[turning_point_index
                     [i]];  // direction_begin记录转折点间路径段首点到尾点的方向
        // std::cout<<"RRT路径规划模块:direction_begin:
        // "<<direction_begin.element1_<<" "<<direction_begin.element2_<<"
        // "<<direction_begin.element3_<<" "<<direction_begin.element4_<<"
        // "<<direction_begin.element5_<<" "<<direction_begin.element6_<<" "<<"
        // "<<std::endl;
        if ((turning_point_index[i + 1] - turning_point_index[i]) >
            2)  //转折点之间至少有一个点才进行回缩
        {
            std::vector<VectorSixDimension> shrink_vectors;  // NX6矩阵
            int shrink_times = 4;
            for (int j = turning_point_index[i] + 1;
                 j < turning_point_index[i + 1];
                 j++)  //转折点之间的回缩点计算回缩向量
            {
                direction_end =
                    path[j] -
                    path[turning_point_index
                             [i]];  //计算转折点间路径段起始点到当前点的方向
                // std::cout<<"RRT路径规划模块:direction_end:
                // "<<direction_end.element1_<<" "<<direction_end.element2_<<"
                // "<<direction_end.element3_<<" "<<direction_end.element4_<<"
                // "<<direction_end.element5_<<" "<<direction_end.element6_<<"
                // "<<std::endl;
                VectorSixDimension diff_vector(0, 0, 0, 0, 0, 0);
                caculateDiffVector(
                    direction_end,
                    direction_begin,
                    diff_vector,
                    shrink_times);  //计算当前点到路径段首尾点连线的方向向量，并除以收缩次数得到diff_vector
                shrink_vectors.emplace_back(diff_vector);
            }
            // std::cout<<"RRT路径规划模块:shrink_vectors:
            // "<<shrink_vectors[0].element1_<<"
            // "<<shrink_vectors[0].element2_<<"
            // "<<shrink_vectors[0].element3_<<"
            // "<<shrink_vectors[0].element4_<<"
            // "<<shrink_vectors[0].element5_<<"
            // "<<shrink_vectors[0].element6_<<" "<<std::endl;
            int shrink_layer = -2;
            for (int j = 0; j < shrink_times; j++)  //找出收缩层数
            {
                int shrink_vectors_index =
                    0;  //记录当前路径点对应的收缩向量diff_vector序号
                for (int k = turning_point_index[i] + 1;
                     k < turning_point_index[i + 1];
                     k++)  //回缩点进行碰撞检测
                {
                    VectorSixDimension new_node(0, 0, 0, 0, 0, 0);
                    new_node =
                        path[k] +
                        shrink_vectors
                            [shrink_vectors_index];  //在回缩向量指向的地方生成新点，并进行碰撞检测
                    if (!obstacleFree(
                            new_node.transformToVectorAndAdjustAngle())) {
                        turning_point_index.push_back(k);
                        shrink_layer = j;  //若发生碰撞记录当前收缩层数
                    }
                    shrink_vectors_index++;
                }
                if (shrink_layer > -1)  //回缩中检测到碰撞
                {
                    break;
                }
            }
            if (shrink_layer < 0)  //回缩成直线也没有碰撞
            {
                shrink_layer = shrink_times;
            }
            int shrink_vectors_index = 0;
            for (int j = turning_point_index[i] + 1;
                 j < turning_point_index[i + 1];
                 j++)  //对路径点按照收缩层数进行回缩
            {
                path[j] = path[j] +
                          shrink_vectors[shrink_vectors_index] * shrink_layer;
                shrink_vectors_index++;
            }
        }
    }
}
//计算A向量与(A在B上投影的向量)的向量差,shrink_times表示对这个向量进行放缩处理
void RRTConnect::caculateDiffVector(RRTConnect::VectorSixDimension& A,
                                    RRTConnect::VectorSixDimension& B,
                                    VectorSixDimension& diff_vector,
                                    int shrink_times = 1)
{
    //计算两向量长度
    double lengthA = A.calculateEuclideanDis();
    double lengthB = B.calculateEuclideanDis();
    if (lengthA == 0 || lengthB == 0) {
        std::cout << "RRT路径规划模块:caculateDiffVector函数:投影向量为零！"
                  << std::endl;
        return;
    }
    double projection = (A * B) / std::pow(lengthB, 2);
    diff_vector = (B * projection - A) / shrink_times;
}
double RRTConnect::cosineOfIncludedAngle(
    RRTConnect::VectorSixDimension& A,
    RRTConnect::VectorSixDimension& B)  //计算向量夹角
{
    double lengthA = A.calculateEuclideanDis();
    double lengthB = B.calculateEuclideanDis();
    if (lengthA == 0 || lengthB == 0) {
        return 0;
    }
    return (A * B) / (lengthA * lengthB);
}

std::vector<RRTConnect::VectorSixDimension>
RRTConnect::planning()  //返回NX6矩阵
{
    node_list_1_.push_back(begin_);
    node_list_2_.push_back(end_);
    srand((int) time(0));
    for (int i = 0; i < max_iter_; i++) {
        // std::cout<<"RRT路径规划模块:-----------------"<<"起点树拓展!"<<std::endl;
        std::shared_ptr<RRTNode> rnd_node = randomSampling(true, true);
        // std::cout<<"RRT路径规划模块:随机点rnd_node:"<<rnd_node->joint_vector_.element1_<<"
        // "<<rnd_node->joint_vector_.element2_<<"
        // "<<rnd_node->joint_vector_.element3_
        // <<"  "<<rnd_node->joint_vector_.element4_<<"
        // "<<rnd_node->joint_vector_.element5_<<"
        // "<<rnd_node->joint_vector_.element6_<<std::endl;
        int nearest_ind = getNearestNodeIndex(node_list_1_, rnd_node);
        // std::cout<<"RRT路径规划模块:最近点nearest_index:"<<nearest_ind<<std::endl;
        std::shared_ptr<RRTNode> nearest_node = node_list_1_[nearest_ind];
        std::shared_ptr<RRTNode> new_node =
            steer(nearest_node, rnd_node, dis_expand_);
        if (obstacleFree(
                new_node->joint_vector_.transformToVectorAndAdjustAngle())) {
            // std::cout<<"RRT路径规划模块:碰撞检测通过！起点树增加节点"<<std::endl;
            node_list_1_.push_back(new_node);
            nearest_ind = getNearestNodeIndex(node_list_2_, new_node);
            nearest_node = node_list_2_[nearest_ind];
            if (calDistTwoNode(new_node, nearest_node) < dis_expand_) {
                // std::cout<<"RRT路径规划模块:！！！！！！起点树拓展连接到终点树!"<<std::endl;
                std::vector<RRTConnect::VectorSixDimension> path;
                path = generatePathNodesFromNodesList(nearest_ind, true);
                // std::cout<<"RRT路径规划模块:原始路径:共"<<path.size()<<"个点"<<std::endl;
                for (int i = 0; i < path.size(); i++) {
                    std::cout << path[i].element1_ << " " << path[i].element2_
                              << " " << path[i].element3_ << " "
                              << path[i].element4_ << " " << path[i].element5_
                              << " " << path[i].element6_ << std::endl;
                }
                smoothPathNarrowly(path, 50);
                // std::cout<<"RRT路径规划模块:smoothPathNarrowly函数顺滑后的路径:共"<<path.size()<<"个点"<<std::endl;
                for (int i = 0; i < path.size(); i++) {
                    std::cout << path[i].element1_ << " " << path[i].element2_
                              << " " << path[i].element3_ << " "
                              << path[i].element4_ << " " << path[i].element5_
                              << " " << path[i].element6_ << std::endl;
                }
                smoothPathBroadly(path);
                // std::cout<<"RRT路径规划模块:smoothPathBroadly函数顺滑后的路径:共"<<path.size()<<"个点"<<std::endl;
                for (int i = 0; i < path.size(); i++) {
                    std::cout << path[i].element1_ << " " << path[i].element2_
                              << " " << path[i].element3_ << " "
                              << path[i].element4_ << " " << path[i].element5_
                              << " " << path[i].element6_ << std::endl;
                }
                node_list_1_.clear();
                node_list_2_.clear();
                return path;
            }
        }
        // std::cout<<"RRT路径规划模块:-----------------"<<"终点树拓展!"<<std::endl;
        if (*rnd_node == *end_) {
            // std::cout<<"RRT路径规划模块:采样到了终点，终点树拓展重新采样！"<<std::endl;
            rnd_node = randomSampling(true, false);
        }
        int nearest_ind_2 = getNearestNodeIndex(node_list_2_, rnd_node);
        std::shared_ptr<RRTNode> nearest_node_2 = node_list_2_[nearest_ind_2];
        std::shared_ptr<RRTNode> new_node_2 =
            steer(nearest_node_2, rnd_node, dis_expand_);
        if (obstacleFree(
                new_node_2->joint_vector_.transformToVectorAndAdjustAngle())) 
        {
            // std::cout<<"RRT路径规划模块:碰撞检测通过！终点树增加节点"<<std::endl;
            node_list_2_.push_back(new_node_2);
            nearest_ind_2 = getNearestNodeIndex(node_list_1_, new_node_2);
            nearest_node_2 = node_list_1_[nearest_ind_2];
            if (calDistTwoNode(new_node_2, nearest_node_2) < dis_expand_) {
                // std::cout<<"RRT路径规划模块:！！！！！！终点树拓展连接到起点树!"<<std::endl;
                std::vector<RRTConnect::VectorSixDimension> path;
                path = generatePathNodesFromNodesList(nearest_ind_2, false);
                // std::cout<<"RRT路径规划模块:原始路径:共"<<path.size()<<"个点"<<std::endl;
                for (int i = 0; i < path.size(); i++) {
                    std::cout << path[i].element1_ << " " << path[i].element2_
                              << " " << path[i].element3_ << " "
                              << path[i].element4_ << " " << path[i].element5_
                              << " " << path[i].element6_ << std::endl;
                }
                smoothPathNarrowly(path, 50);
                // std::cout<<"RRT路径规划模块:smoothPathNarrowly函数顺滑后的路径:共"<<path.size()<<"个点"<<std::endl;
                for (int i = 0; i < path.size(); i++) {
                    std::cout << path[i].element1_ << " " << path[i].element2_
                              << " " << path[i].element3_ << " "
                              << path[i].element4_ << " " << path[i].element5_
                              << " " << path[i].element6_ << std::endl;
                }
                smoothPathBroadly(path);
                // std::cout<<"RRT路径规划模块:smoothPathBroadly函数顺滑后的路径:共"<<path.size()<<"个点"<<std::endl;
                for (int i = 0; i < path.size(); i++) {
                    std::cout << path[i].element1_ << " " << path[i].element2_
                              << " " << path[i].element3_ << " "
                              << path[i].element4_ << " " << path[i].element5_
                              << " " << path[i].element6_ << std::endl;
                }
                node_list_1_.clear();
                node_list_2_.clear();
                return path;
            }
        }
    }
    std::cout << "RRT路径规划模块:exceed the max iteration times!" << std::endl;
    return {};
}
//在多个路径点之间进行路径规划，way_points是输入路径点，output_path是输出路径点
bool RRTConnect::multiWayPointsPlanning(
    const std::vector<std::shared_ptr<RRTConnect::VectorSixDimension>>&
        way_points,
    std::vector<RRTConnect::VectorSixDimension>& output_path)
{
    for (int i = 0; i < (way_points.size() - 1); i++) {
        if (abs(way_points[i]->element1_ - way_points[i + 1]->element1_) <
                0.05 &&
            abs(way_points[i]->element2_ - way_points[i + 1]->element2_) <
                0.05 &&
            abs(way_points[i]->element3_ - way_points[i + 1]->element3_) <
                0.05 &&
            abs(way_points[i]->element4_ - way_points[i + 1]->element4_) <
                0.05 &&
            abs(way_points[i]->element5_ - way_points[i + 1]->element5_) <
                0.05 &&
            abs(way_points[i]->element6_ - way_points[i + 1]->element6_) <
                0.05) {
            std::cout << "RRT路径规划模块:存在过近路径点:" << std::endl;
            return false;
        }
    }
    RRTConnect::VectorSixDimension firstPoint(way_points[0]->element1_,
                                              way_points[0]->element2_,
                                              way_points[0]->element3_,
                                              way_points[0]->element4_,
                                              way_points[0]->element5_,
                                              way_points[0]->element6_);
    output_path.emplace_back(firstPoint);
    for (int i = 0; i < (way_points.size() - 1); i++) {
        setBegin(way_points[i]);
        setEnd(way_points[i + 1]);
        std::vector<RRTConnect::VectorSixDimension> temp = planning();
        output_path.insert(output_path.end(), temp.begin() + 1, temp.end());
    }
    std::cout << "RRT路径规划模块:paths:" << std::endl;
    for (int i = 0; i < output_path.size(); i++) {
        std::cout << output_path[i].element1_ << " " << output_path[i].element2_
                  << " " << output_path[i].element3_ << " "
                  << output_path[i].element4_ << " " << output_path[i].element5_
                  << " " << output_path[i].element6_ << " " << std::endl;
    }
    return true;
}
