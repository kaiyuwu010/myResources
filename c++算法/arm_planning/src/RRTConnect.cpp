#include "arm_planning/RRTConnect.h"
/*
ToDo:
超过目标点部分的路径点，经过回缩处理后，还是会超过目标点
*/
//-------------------------------------------------------------------------------------------------------
// MultiDimVector类运算符重载
bool RRTConnect::MultiDimVector::operator==(const RRTConnect::MultiDimVector& vec) const
{
    for(int i=0; i<this->vector_.size(); i++)
    {
        if(this->vector_[i] != vec.vector_[i])
        {
            return false;
        }
    }
    return true;
}
RRTConnect::MultiDimVector& RRTConnect::MultiDimVector::operator=(const RRTConnect::MultiDimVector& other)
{
    for(int i=0; i<other.vector_.size(); i++)
    {
        this->vector_[i] = other.vector_[i];
    }
    return *this;
}
RRTConnect::MultiDimVector RRTConnect::MultiDimVector::operator+(const RRTConnect::MultiDimVector& vec) const
{
    std::vector<double> new_vec;
    for(int i=0; i<this->vector_.size(); i++)
    {
        new_vec.push_back(this->vector_[i] + vec.vector_[i]);
    }
    MultiDimVector new_vector(new_vec);
    return new_vector;
}
RRTConnect::MultiDimVector RRTConnect::MultiDimVector::operator-(const RRTConnect::MultiDimVector& other) const
{
    std::vector<double> new_vec;
    for(int i=0; i<this->vector_.size(); i++)
    {
        new_vec.push_back(this->vector_[i] - other.vector_[i]);
    }
    MultiDimVector new_vector(new_vec);
    return new_vector;
}
double RRTConnect::MultiDimVector::operator*(const RRTConnect::MultiDimVector& vec) const
{
    double dotProduct;
    for(int i=0; i<this->vector_.size(); i++)
    {
        dotProduct += this->vector_[i] * vec.vector_[i];
    }
    return dotProduct;
}
RRTConnect::MultiDimVector RRTConnect::MultiDimVector::operator*(const double& value) const
{
    std::vector<double> new_vec;
    for(int i=0; i<this->vector_.size(); i++)
    {
        new_vec.push_back(this->vector_[i] * value);
    }
    MultiDimVector new_vector(new_vec);
    return new_vector;
}
RRTConnect::MultiDimVector RRTConnect::MultiDimVector::operator/(const double& value) const
{
    std::vector<double> new_vec;
    for(int i=0; i<this->vector_.size(); i++)
    {
        new_vec.push_back(this->vector_[i] / value);
    }
    MultiDimVector new_vector(new_vec);
    return new_vector;
}
//-------------------------------------------------------------------------------------------------------
// RRTConnect类构造函数，参数需要碰撞检测函数、采样范围、拓展步长、优化步长、直接采样目标点概率、最大迭代次数
RRTConnect::RRTConnect(std::function<bool(const std::shared_ptr<RRTConnect::RRTNode> node)> isStateValid,
                       const std::vector<double>& upper_limit,
                       const std::vector<double>& lower_limit,
                       double expand_dis,
                       double dis_optimize,
                       double goal_sample_rate,
                       int max_iter): 
                       sample_range_(upper_limit, lower_limit),
                       obstacleFree(isStateValid), 
                       dis_expand_(expand_dis), 
                       goal_sample_rate_(goal_sample_rate), 
                       max_iter_(max_iter), 
                       dis_optimize_(dis_optimize)
{
    if(upper_limit.size() != lower_limit.size() || upper_limit.size() == 0){
        throw std::invalid_argument("RRT运动规划模块: 给定的采样上下限维度不一致或者维度为0!!!");
    }
    std::cout<<"rrtTest"<<std::endl;
    dim_ = upper_limit.size();
    std::cout<<"RRT运动规划模块: 采样空间上限为: "<<std::endl;
    for(size_t i=0; i<upper_limit.size(); i++)
    {
        std::cout<<sample_range_.upper_limit_.vector_[i]<<"  ";
    }
    std::cout<<std::endl;
    std::cout<<"RRT运动规划模块: 采样空间下限为: "<<std::endl;
    for(size_t i=0; i<lower_limit.size(); i++)
    {
        std::cout<<sample_range_.lower_limit_.vector_[i]<<"  ";
    }
    begin_ = this->generateRRTNode();
    end_ = this->generateRRTNode();
    std::cout<<std::endl;
    std::cout<<"RRT运动规划模块: 由给定采样空间确定RRT规划维度为: "<<dim_<<std::endl;
}
//随机采样函数，在给定采样范围进行随机采样
void RRTConnect::randomSampling(std::shared_ptr<RRTConnect::RRTNode> &new_node, bool sample_target, bool from_start)
{
    // srand随机数种子不能重复定义，否则会造成随机数重复，这里高频调用sampleFree，应该把随机数种子放在外层
    if (sample_target && rand() % (100) < goal_sample_rate_)  
    {
        std::cout<<"RRT运动规划模块: 直接采样目标点!"<<std::endl;
        if (from_start) 
            new_node = end_;
        else 
            new_node = begin_;
    }
    MultiDimVector diff = sample_range_.upper_limit_ - sample_range_.lower_limit_;
    //rand()/double(RAND_MAX)产生0~1之间的浮点数
    for(size_t i=0; i<dim_; i++)
    {
        diff.vector_[i] = rand() / double(RAND_MAX) * diff.vector_[i] + sample_range_.lower_limit_ .vector_[i];  
    }
    new_node = std::make_shared<RRTNode>(diff);
}
// RRTConnect类析构函数
RRTConnect::~RRTConnect(){}

//输入拓展树节点向量和RRTNode节点，找到关节空间欧氏距离，距离RRTNode节点最近的点
size_t RRTConnect::getNearestNodeIndex(const std::vector<std::shared_ptr<RRTConnect::RRTNode>> &node_list, const std::shared_ptr<RRTConnect::RRTNode> &node)
{
    int min_index = -1;
    double dis_min = std::numeric_limits<double>::max();
    // std::cout<<"查找最近点的node值: ";
    // for(size_t i=0; i<dim_; i++)
    // {
    //     std::cout<<node->node_vector_.vector_[i]<<" ";
    // }
    // std::cout<<std::endl;
    for (int i = 0; i < node_list.size(); i++) {
        double dist = node_list[i]->node_vector_.calculateEuclideanDis(node->node_vector_);
        // std::cout<<"欧氏距离"<<dist<<std::endl;
        if (dis_min > dist) {
            dis_min = dist;
            min_index = i;
        }
    }
    std::cout<<"查找最近点的dist值: "<< dis_min <<"  索引为: "<<min_index<<std::endl;
    return min_index;
}
//给定两个节点计算起始节点到目标节点的方向向量和距离
RRTConnect::MultiDimVector RRTConnect::calculateDirectionVector(const std::shared_ptr<RRTConnect::RRTNode> &from_node,
                                                                const std::shared_ptr<RRTConnect::RRTNode> &to_node,
                                                                double& length)  //两个点的值不能一样
{
    if (from_node == to_node) {
        std::cout << "RRT运动规划模块: calculateDirectionVector函数输入了两个相同的node!" << std::endl;
        return from_node->node_vector_;
    }
    MultiDimVector diff_joint = to_node->node_vector_ - from_node->node_vector_;
    length = diff_joint.calculateEuclideanDis();
    diff_joint = diff_joint / length;
    return diff_joint;
}
//给定起始节点和目标节点，往目标节点的方向拓展一定步长的距离生成新节点，指定新节点父节点为from_node并计算出新节点到拓展树根节点的距离
std::shared_ptr<RRTConnect::RRTNode> RRTConnect::steer(std::shared_ptr<RRTConnect::RRTNode> from_node,
                                                       std::shared_ptr<RRTConnect::RRTNode> to_node,
                                                       double extend_length)  
{
    double length = 0;
    MultiDimVector direction_element = calculateDirectionVector(from_node, to_node, length);
    std::cout<<"RRT运动规划模块:steer函数中生成的单位方向向量为: ";
    for(size_t i=0; i<dim_; i++)
    {
        std::cout<<direction_element.vector_[i]<<" ";
    }
    std::cout<<std::endl;
    std::vector<double> vec;
    for(size_t i=0; i<dim_; i++)
    {
        vec.push_back(0);
    }
    std::shared_ptr<RRTNode> new_node = std::make_shared<RRTNode>(vec);
    new_node->parent_node_ = from_node;
    new_node->dist_to_origin_ = from_node->dist_to_origin_ + extend_length;
    new_node->node_vector_ = from_node->node_vector_ + (direction_element * extend_length);
    if (!isNodeInSampleRange(*new_node))  //沿方向取的点超过范围则直接取to_node点
    {
        std::cout << "RRT运动规划模块:超过采样范围取随机点!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
        new_node->parent_node_ = from_node;
        new_node->dist_to_origin_ = from_node->dist_to_origin_ + length;
        new_node->node_vector_ = to_node->node_vector_;
    }
    return new_node;
}
//给定拓展树向量node_list、节点和搜索半径，获取给定节点最近节点集合在向量的序号和到给定节点的距离
void RRTConnect::getNearestNodesIndex(std::vector<std::shared_ptr<RRTConnect::RRTNode>> node_list, 
                                      std::shared_ptr<RRTConnect::RRTNode> node,
                                      double radius, 
                                      std::vector<std::pair<int, double>>& found_nodes)  
{
    for (int i = 0; i < node_list.size(); i++) {
        std::shared_ptr<RRTNode> node = node_list[i];
        double distance = node->node_vector_.calculateEuclideanDis(node->node_vector_);
        if (distance <= radius) {
            found_nodes.push_back(std::pair<int, double>(i, distance));
        }
    }
}

//计算两个节点间的拓展距离
double RRTConnect::calDistTwoNode(const std::shared_ptr<RRTConnect::RRTNode> node1, const std::shared_ptr<RRTConnect::RRTNode> node2)
{
    return node1->node_vector_.calculateEuclideanDis(node2->node_vector_);
}
//给定拓展树中连接成功的节点的序号，生成连接成功的路径
std::vector<std::shared_ptr<RRTConnect::RRTNode>> RRTConnect::generatePathNodesFromNodesList(int nearst_index, bool link_to_end_tree)
{
    std::cout<<"RRT运动规划模块:node_list_1_共"<<node_list_1_.size()<<"个点: "<<std::endl;
    for (size_t i = 0; i < 10; i++) {
        for(size_t j=0; j<dim_; j++){
            std::cout << node_list_1_[i]->node_vector_.vector_[j] << " ";
        }
        std::cout<<std::endl;
    }
    std::cout<<"RRT运动规划模块:node_list_2_共"<<node_list_2_.size()<<"个点: "<<std::endl;
    for (size_t i = 0; i < 10; i++) {
        for(size_t j=0; j<dim_; j++){
            std::cout << node_list_2_[i]->node_vector_.vector_[j] << " ";
        }
        std::cout<<std::endl;
    }
    std::vector<std::shared_ptr<RRTConnect::RRTNode>> nodes_path, list1_path;
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
    while (node->parent_node_ != nullptr) {
        list1_path.push_back(node);
        node = node->parent_node_;
    }
    list1_path.push_back(node);
    std::cout<<"RRT运动规划模块:node_list_1_上反向索引找到的路径点: 共"<<list1_path.size()<<"个点: "<<std::endl;
    for (size_t i = 0; i < list1_path.size(); i++) {
        for(size_t j=0; j<dim_; j++){
            std::cout << list1_path[i]->node_vector_.vector_[j] << " ";
        }
        std::cout<<std::endl;
    }
    //路径点放入nodes_path
    for (size_t i = list1_path.size() - 1; i > -1; i--) {
        nodes_path.push_back(list1_path[i]);
    }
    list1_path.clear();
    //从node_list_2中找出路径点
    if (link_to_end_tree == true) {
        node = node_list_2_[nearst_index];
    } else {
        node = node_list_2_[node_list_2_.size() - 1];
    }
    std::cout<<"RRT运动规划模块:node_list_2_上反向索引找到的路径点: "<<std::endl;
    while (node->parent_node_ != nullptr) {
        nodes_path.push_back(node);
        for(size_t i=0; i<dim_; i++){
            std::cout << node->node_vector_.vector_[i] << " ";
        }
        std::cout<<std::endl;
        node = node->parent_node_;
    }
    nodes_path.push_back(node);
    std::cout<<"RRT运动规划模块:node_list_1_和node_list_2_合并路径点: 共"<<nodes_path.size()<<"个点: "<<std::endl;
    for (size_t i = 0; i < nodes_path.size(); i++) {
        for(size_t j=0; j<dim_; j++){
            std::cout << nodes_path[i]->node_vector_.vector_[j] << " ";
        }
        std::cout<<std::endl;
    }
    return nodes_path;
}

void RRTConnect::rewire(std::vector<std::shared_ptr<RRTConnect::RRTNode>>& node_list, std::shared_ptr<RRTConnect::RRTNode> new_node)
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
    new_node->parent_node_ = node_list[index];
    new_node->dist_to_origin_ = dist_shortest;
    //重选子节点
    for (int i = 0; i < found_nodes.size(); i++) {
        double temp = dist_shortest + found_nodes[i].second;
        if (node_list[found_nodes[i].first]->dist_to_origin_ > temp) {
            node_list[found_nodes[i].first]->parent_node_ = new_node;
            node_list[found_nodes[i].first]->dist_to_origin_ = temp;
        }
    }
}
//判断节点是否在给定的采样范围
bool RRTConnect::isNodeInSampleRange(const RRTConnect::RRTNode& node)
{
    for(size_t i=0; i<dim_; i++)
    {
        double a= node.node_vector_.vector_[i] - sample_range_.upper_limit_.vector_[i];
        if (node.node_vector_.vector_[i] > sample_range_.upper_limit_.vector_[i] || node.node_vector_.vector_[i] < sample_range_.lower_limit_.vector_[i]) 
        {
            return false;
        }
    }
    std::cout << "RRT运动规划模块:该点在采样范围！" << std::endl;
    return true;
}
//输入路径向量，对路径进行局部顺滑，times_input表示顺滑次数(TODO:检测新顺滑点是否在采样空间)
void RRTConnect::smoothPathNarrowly(std::vector<std::shared_ptr<RRTConnect::RRTNode>>& path, int times_input)  //输入的path点数量大于3，path大小不能变
{
    std::shared_ptr<RRTNode> new_node = this->generateRRTNode();
    int n = path.size() - 1;
    //如果路径点数量小于4个不进行局部顺滑
    if (n < 4) {
        throw std::invalid_argument("RRT运动规划模块:smoothPathNarrowly函数输入路径点小于3个,请重新输入！");
    }
    //index向量表示固定点的序号的集合，后面只会对固定点之间的路径点进行平滑处理。路径首尾点也是固定点，所以先存入首尾点的序号
    std::vector<int> index{0, n};
    for (int times_loop = 0; times_loop < times_input; times_loop++) {
        //对序号进行排序，因为需要按顺序对每个固定点间的路径进行平滑处理
        sort(index.begin(), index.end());
        //size_index表示固定点个数减1后的值
        int size_index = index.size() - 1;  
        //遍历固定点
        for (int j = 0; j < size_index; j++)  
        {
            //i表示第j个和j+1个固定点中间的路径点的序号，所以第一个等于index[j]+1，最后一个小于index[j+1]
            for (int i = index[j] + 1; i < index[j + 1]; i++)  
            {
                //计算第i个路径点相邻两路径点的中点
                new_node->node_vector_ = path[i - 1]->node_vector_ + (path[i + 1]->node_vector_ - path[i - 1]->node_vector_) * 0.5;
                //检查新生成的中点是否发生碰撞，不发生碰撞就用新点替换原来的路径点  
                if (obstacleFree(new_node))  
                {
                    path[i] = new_node;
                } 
                //发生碰撞的点不能进行平滑处理，把它的序号放入index容器
                else {
                    index.push_back(i + 1);  
                }
            }
        }
    }
}
//对路径进行大范围的顺滑,参数path是NX6矩阵
void RRTConnect::smoothPathBroadly(std::vector<std::shared_ptr<RRTConnect::RRTNode>>& path)  
{
    std::vector<int> turning_point_index;                                        //转折点序号
    turning_point_index.emplace_back(0);
    MultiDimVector direction_begin = *(this->generateMultiDimVec());             //存放路径段起始方向
    MultiDimVector direction_end = *(this->generateMultiDimVec());               //存放路径段结束处的方向
    bool flag_init_begin = true;                                                 //路径段首次顺滑处理标志
    double last_cosine_value = 1;                                                //上一次计算出的余弦值
    int shrink_times = 4;                                                        //回缩处理次数
    //----------------------------------------------------找出路径中的转折点----------------------------------------------------                                                
    for (int i = 0; i < path.size() - 1; i++)                                    
    {
        if (!flag_init_begin) {
            //计算当前点到下一个路径点的方向
            direction_end = path[i + 1]->node_vector_ - path[i]->node_vector_; 
            //计算当前路径点方向与起始方向的夹角余弦 
            double temp = cosineOfIncludedAngle(direction_begin, direction_end); 
            //两方向间的夹角变大也就是余弦值temp变小时，更新last_cosine_value为temp 
            if (temp <= last_cosine_value)  
            {
                last_cosine_value = temp;
            }
            //方向变动不大，更新最大夹角值(最小cos值)last_cosine_value
            else if(temp > 0.8 || abs(temp - last_cosine_value) < 0.01)  
            {
                last_cosine_value = temp;
            }
            //找到转折点 
            else{
                turning_point_index.emplace_back(i);
                last_cosine_value = 1;
                flag_init_begin = true;
            }
        }
        //对路径段进行首次顺滑时记录路径方向 
        else{
            direction_begin = path[i + 1]->node_vector_ - path[i]->node_vector_;
            flag_init_begin = false;
        }
    }
    //尾点索引加入转折点索引向量
    turning_point_index.emplace_back(path.size() - 1);  
    std::cout << "RRT运动规划模块: 转折点序号:";
    for (int j = 0; j < turning_point_index.size(); j++){
        std::cout << turning_point_index[j] << "、";
    }
    std::cout << std::endl;
    //----------------------------------------------------回缩处理非转折点---------------------------------------------------
    for (int i = 0; i < turning_point_index.size() - 1; i++)  
    {
        //direction_begin记录转折点间路径段首点到尾点的方向
        direction_begin = path[turning_point_index[i + 1]]->node_vector_ - path[turning_point_index[i]]->node_vector_;  
        //转折点之间至少有一个点才进行回缩
        if ((turning_point_index[i + 1] - turning_point_index[i]) > 2)  
        {
            std::vector<MultiDimVector> shrink_vectors;  //NX6矩阵
            //转折点之间的回缩点计算回缩向量
            for (size_t j = turning_point_index[i] + 1; j < turning_point_index[i + 1]; j++)  
            {
                //计算转折点间路径段起始点到当前点的方向
                direction_end = path[j]->node_vector_ - path[turning_point_index[i]]->node_vector_;  
                std::vector<double> vec;
                for(size_t i=0; i<dim_; i++)
                {
                    vec.push_back(0);
                }
                MultiDimVector diff_vector(vec);
                //计算当前点到路径段首尾点连线的方向向量，并除以收缩次数得到diff_vector
                caculateDiffVector(direction_end, direction_begin, diff_vector, shrink_times);  
                shrink_vectors.emplace_back(diff_vector);
            }
            int shrink_layer = -2;
            //找出收缩层数
            for (int j = 0; j < shrink_times; j++)  
            {
                //记录当前路径点对应的收缩向量diff_vector序号
                int shrink_vectors_index = 0;
                //回缩点进行碰撞检测  
                for (int k = turning_point_index[i] + 1; k < turning_point_index[i + 1]; k++)  
                {
                    std::vector<double> vec;
                    for(size_t i=0; i<dim_; i++)
                    {
                        vec.push_back(0);
                    }
                    MultiDimVector new_node(vec);
                    //在回缩向量指向的地方生成新点，并进行碰撞检测
                    new_node = path[k]->node_vector_ + shrink_vectors[shrink_vectors_index];  
                    if (!obstacleFree(new_node)) {
                        turning_point_index.push_back(k);
                        //若发生碰撞记录当前收缩层数
                        shrink_layer = j;  
                    }
                    shrink_vectors_index++;
                }
                //回缩中检测到碰撞
                if (shrink_layer > -1)  
                {
                    break;
                }
            }
            //回缩成直线也没有碰撞
            if (shrink_layer < 0)  
            {
                shrink_layer = shrink_times;
            }
            int shrink_vectors_index = 0;
            //对路径点按照收缩层数进行回缩
            for (int j = turning_point_index[i] + 1; j < turning_point_index[i + 1]; j++)  
            {
                path[j] = path[j] + shrink_vectors[shrink_vectors_index] * shrink_layer;
                shrink_vectors_index++;
            }
        }
    }
}
//计算A向量与(A在B上投影的向量)的向量差,shrink_times表示对这个向量进行放缩处理
void RRTConnect::caculateDiffVector(RRTConnect::MultiDimVector& A,
                                    RRTConnect::MultiDimVector& B,
                                    MultiDimVector& diff_vector,
                                    int shrink_times = 1)
{
    //计算两向量长度
    double lengthA = A.calculateEuclideanDis();
    double lengthB = B.calculateEuclideanDis();
    if (lengthA == 0 || lengthB == 0) {
        std::cout << "RRT运动规划模块:caculateDiffVector函数:投影向量为零！" << std::endl;
        return;
    }
    double projection = (A * B) / std::pow(lengthB, 2);
    diff_vector = (B * projection - A) / shrink_times;
}
//计算向量夹角
double RRTConnect::cosineOfIncludedAngle(RRTConnect::MultiDimVector& A, RRTConnect::MultiDimVector& B)  
{
    double lengthA = A.calculateEuclideanDis();
    double lengthB = B.calculateEuclideanDis();
    if (lengthA == 0 || lengthB == 0) {
        std::cout << "RRT运动规划模块:cosineOfIncludedAngle函数:存在向量长度为零！" << std::endl;
        return 0;
    }
    return (A * B) / (lengthA * lengthB);
}
//返回NX6矩阵
std::vector<std::shared_ptr<RRTConnect::RRTNode>> RRTConnect::planning()  
{
    node_list_1_.push_back(begin_);
    node_list_2_.push_back(end_);
    srand((int) time(0));
    for (size_t i = 0; i < max_iter_; i++) {
        std::cout<<"RRT运动规划模块:-----------------"<<"起点树拓展!"<<std::endl;
        std::shared_ptr<RRTNode> rnd_node;
        randomSampling(rnd_node, true, true);
        std::cout<<"RRT运动规划模块:采样到随机点rnd_node: ";
        for(size_t i=0; i<dim_; i++)
        {
            std::cout<<rnd_node->node_vector_.vector_[i]<<" ";
        }
        std::cout<<std::endl;
        size_t nearest_ind = getNearestNodeIndex(node_list_1_, rnd_node);
        std::shared_ptr<RRTNode> nearest_node = node_list_1_[nearest_ind];
        std::cout<<"RRT运动规划模块:查找到最近点的索引nearest_index为:"<<nearest_ind<<" 值为: ";
        for(size_t i=0; i<dim_; i++)
        {
            std::cout<<nearest_node->node_vector_.vector_[i]<<" ";
        }
        std::cout<<std::endl;
        std::shared_ptr<RRTNode> new_node = steer(nearest_node, rnd_node, dis_expand_);
        std::cout<<"RRT运动规划模块:生成的新node为: ";
        for(size_t i=0; i<dim_; i++)
        {
            std::cout<<new_node->node_vector_.vector_[i]<<" ";
        }
        std::cout<<std::endl;
        if (obstacleFree(new_node)) {
            std::cout<<"RRT运动规划模块:碰撞检测通过！起点树增加节点"<<std::endl;
            node_list_1_.push_back(new_node);
            std::cout<<"RRT运动规划模块:node_list_1_大小为:"<<node_list_1_.size()<<std::endl;
            nearest_ind = getNearestNodeIndex(node_list_2_, new_node);
            nearest_node = node_list_2_[nearest_ind];
            if (calDistTwoNode(new_node, nearest_node) < dis_expand_) {
                std::cout<<"RRT运动规划模块:！！！！！！起点树拓展连接到终点树!"<<std::endl;
                std::vector<std::shared_ptr<RRTConnect::RRTNode>> path;
                path = generatePathNodesFromNodesList(nearest_ind, true);
                std::cout<<"RRT运动规划模块:原始路径:共"<<path.size()<<"个点"<<std::endl;
                for (size_t i = 0; i < path.size(); i++){
                    for(size_t j=0; j<dim_; j++){
                        std::cout << path[i]->node_vector_.vector_[j] << " ";
                    }
                    std::cout<<std::endl;
                }
                smoothPathNarrowly(path, 50);
                // std::cout<<"RRT运动规划模块:smoothPathNarrowly函数顺滑后的路径:共"<<path.size()<<"个点"<<std::endl;
                for (size_t i = 0; i < path.size(); i++) {
                    for(size_t j=0; j<dim_; j++){
                        std::cout << path[i]->node_vector_.vector_[j] << " ";
                    }
                    std::cout<<std::endl;
                }
                smoothPathBroadly(path);
                std::cout<<"RRT运动规划模块:smoothPathBroadly函数顺滑后的路径:共"<<path.size()<<"个点，值为: "<<std::endl;
                for (size_t i = 0; i < path.size(); i++) {
                    for(size_t j=0; j<dim_; j++){
                        std::cout << path[i]->node_vector_.vector_[j] << " ";
                    }
                    std::cout<<std::endl;
                }
                node_list_1_.clear();
                node_list_2_.clear();
                return path;
            }
        }
        std::cout<<"RRT运动规划模块:-----------------"<<"终点树拓展!"<<std::endl;
        if (*rnd_node == *end_) {
            // std::cout<<"RRT运动规划模块:采样到了终点，终点树拓展重新采样！"<<std::endl;
            randomSampling(rnd_node, true, true);
        }
        int nearest_ind_2 = getNearestNodeIndex(node_list_2_, rnd_node);
        std::shared_ptr<RRTNode> nearest_node_2 = node_list_2_[nearest_ind_2];
        std::shared_ptr<RRTNode> new_node_2 = steer(nearest_node_2, rnd_node, dis_expand_);
        if (obstacleFree(new_node_2)) 
        {
            // std::cout<<"RRT运动规划模块:碰撞检测通过！终点树增加节点"<<std::endl;
            node_list_2_.push_back(new_node_2);
            nearest_ind_2 = getNearestNodeIndex(node_list_1_, new_node_2);
            nearest_node_2 = node_list_1_[nearest_ind_2];
            if (calDistTwoNode(new_node_2, nearest_node_2) < dis_expand_) {
                std::cout<<"RRT运动规划模块:！！！！！！终点树拓展连接到起点树!"<<std::endl;
                std::vector<std::shared_ptr<RRTConnect::RRTNode>> path;
                path = generatePathNodesFromNodesList(nearest_ind_2, false);
                std::cout<<"RRT运动规划模块:原始路径:共"<<path.size()<<"个点"<<std::endl;
                for (size_t i = 0; i < path.size(); i++) {
                    for(size_t j=0; j<dim_; j++){
                        std::cout << path[i]->node_vector_.vector_[j] << " ";
                    }
                    std::cout<<std::endl;
                }
                smoothPathNarrowly(path, 50);
                std::cout<<"RRT运动规划模块:smoothPathNarrowly函数顺滑后的路径:共"<<path.size()<<"个点"<<std::endl;
                for (size_t i = 0; i < path.size(); i++) {
                    for(size_t j=0; j<dim_; j++){
                        std::cout << path[i]->node_vector_.vector_[j] << " ";
                    }
                    std::cout<<std::endl;
                }
                smoothPathBroadly(path);
                std::cout<<"RRT运动规划模块:smoothPathBroadly函数顺滑后的路径:共"<<path.size()<<"个点"<<std::endl;
                for (size_t i = 0; i < path.size(); i++) {
                    for(size_t j=0; j<dim_; j++){
                        std::cout << path[i]->node_vector_.vector_[j] << " ";
                    }
                    std::cout<<std::endl;
                }
                node_list_1_.clear();
                node_list_2_.clear();
                return path;
            }
        }
    }
    std::cout << "RRT运动规划模块:exceed the max iteration times!" << std::endl;
    return {};
}
//在多个路径点之间进行路径规划，way_points是输入路径点，output_path是输出路径点
bool RRTConnect::multiWayPointsPlanning(const std::vector<std::shared_ptr<RRTConnect::MultiDimVector>>& way_points,
                                        std::vector<std::shared_ptr<RRTConnect::RRTNode>>& output_path)
{
    for (size_t i = 0; i < (way_points.size() - 1); i++) 
    {
        if ((*(way_points[i])).calculateEuclideanDis(*(way_points[i + 1])) < 0.05)
        {
            throw std::invalid_argument("RRT运动规划模块:存在过近路径点!!!");
        }            
    }
    std::vector<double> vec;
    for(size_t i=0; i<dim_; i++)
    {
        vec.push_back(way_points[0]->vector_[i]);
    }
    RRTConnect::MultiDimVector firstPoint(vec);
    output_path.emplace_back(firstPoint);
    std::cout << "RRT运动规划模块:要进行规划的路径点数量为:" <<way_points.size()<< std::endl;
    for (size_t i = 0; i < (way_points.size() - 1); i++) {
        setBegin(way_points[i]);
        setEnd(way_points[i + 1]);
        std::vector<std::shared_ptr<RRTConnect::RRTNode>> temp = planning();
        output_path.insert(output_path.end(), temp.begin() + 1, temp.end());
    }
    std::cout << "RRT运动规划模块:多点规划最终得到的路径点数量为:" <<output_path.size()<< std::endl;
    for (size_t i = 0; i < output_path.size(); i++) {
        for(size_t j=0; j<dim_; j++){
            std::cout << output_path[i]->node_vector_.vector_[j] << " ";
        }
        std::cout<<std::endl;
    }
    return true;
}
