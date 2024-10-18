#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H
#include <Eigen/Dense>
#include <cmath>
#include <ctime>
#include <iostream>
/*
--------------------------------------------------------------------
算法使用注意事项：
1.用MDH参数表示时，要求基座标系(连杆0)的z轴与连杆1的z轴重合，连杆1和连杆2的坐标系原点重合，且连杆2、连杆3、连杆4、连杆5和连杆6坐标系原点在连杆1坐标系的x、y平面上。
2.当第5个关节为0时，机械臂处于奇异位型，此时本算法将第4个关节的角度规定为0。
3.腕部位置x、y等于0的情况未解决。
--------------------------------------------------------------------
MDH参数
d[7] = {0,  240.5,   0,     0,   210,    0,  172.5}; MDH参数中d[0]一定为0，单位为mm
a[7] = {0,    0,    256,    0,    0,     0,    0  }; MDH参数中a[0]一般为0，单位为mm
α[7] = {0,   90,     0,    90,   -90,   90,    0  }; MDH参数中α[0]一般为0，单位为度
θ[7] = {0,    0,    90,    90,    0,     0,    0  }; MDH参数中θ[0]一定为0，单位为度
--------------------------------------------------------------------
逆运动学
输出的八组解theta[1~8]对应的机械臂位形:
1.轴1朝目标方向, 轴2使肘关节外翻(轴2负方向偏);
轴3使使肘关节外翻;第5个关节值为正(初始状态时，顺时针旋转); 2.轴1使朝目标方向;
轴2使肘关节外翻(轴2负方向偏);
轴3使使肘关节外翻; 第5个关节值为负(初始状态时，逆时针旋转); 3.轴1使朝目标方向;
轴2使肘关节内凹(轴2正方向偏);
轴3使使肘关节内凹;第5个关节值为正(初始状态时，顺时针旋转); 4.轴1使朝目标方向;
轴2使肘关节内凹(轴2正方向偏);
轴3使使肘关节内凹;第5个关节值为负(初始状态时，逆时针旋转);
5.轴1朝目标相反方向, 轴2使肘关节内凹(轴2正方向偏);
轴3使使肘关节外翻;第5个关节值为正(初始状态时，顺时针旋转);
6.轴1使朝目标相反方向; 轴2使肘关节内凹(轴2正方向偏);
轴3使使肘关节外翻;第5个关节值为负(初始状态时，逆时针旋转);
7.轴1使朝目标相反方向; 轴2使肘关节外翻(轴2负方向偏);
轴3使使肘关节内凹;第5个关节值为正(初始状态时，顺时针旋转);
8.轴1使朝目标相反方向; 轴2使肘关节外翻(轴2负方向偏);
轴3使使肘关节内凹;第5个关节值为负(初始状态时，逆时针旋转);
--------------------------------------------------------------------
*/
class InverseKinematics
{
private:
    //MDH参数
    std::array<double, 7> d_;
    std::array<double, 7> a_;
    std::array<double, 7> alpha_;
    std::array<double, 7> theta_;
    const double deg_to_rad = M_PI / 180.0;
    /*
    --------------------------------------------------------------------
    已知机械臂第i个关节相对第i-1个关节坐标系的相对变换为:
    |    cosθi           -sinθi             0            a(i-1)   |
    |sinθicosα(i-1)   cosθicosα(i-1)    -sinα(i-1)    -disinα(i-1)|
    |sinθisinα(i-1)   cosθisinα(i-1)     cosα(i-1)     dicosα(i-1)|
    |      0                0               0               1     |
    可以得到:
    连杆2坐标系原点到连杆3坐标系原点的长度（upperArm大臂长度）为:a2
    连杆3坐标系原点到连杆4坐标系原点的长度（forearm小臂长度）为:d4
    --------------------------------------------------------------------
    */
    double length_of_upperArm_;
    double square_length_of_upperArm_;
    double length_of_forearm_;
    double square_length_of_forearm_;

public:
    //八组解，每组解六个关节角度，数组第0位不使用
    double theta[8 + 1][6 + 1];
    Eigen::Matrix3d matrix_rotate_aroundx;
    //用MDH参数初始化时，MDH参数的单位为mm和deg
    InverseKinematics(std::array<double, 7> a, std::array<double, 7> alpha, std::array<double, 7> d, std::array<double, 7> theta)
    {
        for (int i = 0; i < 7; i++)
        {
            a_[i] = a[i];
            d_[i] = d[i];
            alpha_[i] = alpha[i] * deg_to_rad;
            theta_[i] = theta[i] * deg_to_rad;
            length_of_upperArm_ = a[2];
            square_length_of_upperArm_ = pow(a[2], 2);
            length_of_forearm_ = d[4];
            square_length_of_forearm_ = pow(d[4], 2);
        }
        matrix_rotate_aroundx << 1, 0, 0, 0, 0, -1, 0, 1, 0;
        std::cout << "InverseKinematics模块:初始化逆运动学模块完成！" << std::endl;
    }
    /*
    --------------------------------------------------------------------
    求解末端三轴交于一点构型的机械臂腕部位姿:
    连杆i相对于连杆i-1的坐标系变换的旋转部分为:
    |    cosθi           -sinθi            0     |
    |sinθicosα(i-1)   cosθicosα(i-1)   -sinα(i-1)|
    |sinθisinα(i-1)   cosθisinα(i-1)    cosα(i-1)|
    --------------------------------------------------------------------
    */
    Eigen::Matrix3d getRotationMatrixForWrist(int index)
    {
        Eigen::Matrix3d T[4];
        for (int i = 1; i < 4; i++)
        {
            T[i](0, 0) = cos(theta[index][i] + theta_[i]);
            T[i](0, 1) = -sin(theta[index][i] + theta_[i]);
            T[i](0, 2) = 0;
            T[i](1, 0) = sin(theta[index][i] + theta_[i]) * cos(alpha_[i - 1]);
            T[i](1, 1) = cos(theta[index][i] + theta_[i]) * cos(alpha_[i - 1]);
            T[i](1, 2) = -sin(alpha_[i - 1]);
            T[i](2, 0) = sin(theta[index][i] + theta_[i]) * sin(alpha_[i - 1]);
            T[i](2, 1) = cos(theta[index][i] + theta_[i]) * sin(alpha_[i - 1]);
            T[i](2, 2) = cos(alpha_[i - 1]);
        }
        T[0] = T[1] * T[2] * T[3];
        return T[0];
    }
    /*
    --------------------------------------------------------------------
    欧拉角zyz对应的旋转矩阵:  
    |c4c5c6-s4s6  -c4c5s6-s4c6   c4s5|
    |s4c5c6+c4s6  -s4c5s6+c4c6   s4s5|
    |   -s5c6          s5s6       c5 |
    acos函数接受[-1,1]之间的实数，返回值的范围是[0,π]; asin函数接受[-1,1]之间的实数，返回值的范围是[-π/2,π/2]
    --------------------------------------------------------------------
    */
    void caculateLastThreeJoint(const Eigen::Matrix4d& T, int solution_index)
    {
        //先解算出第3个连杆的坐标系的旋转矩阵(腕部位姿)，在关节角度都置为0的状态下，把第3个连杆的坐标系姿态转变成第6个连杆坐标系的姿态
        Eigen::Matrix3d wrist_pose = getRotationMatrixForWrist(solution_index) * matrix_rotate_aroundx;
        // 求出目标姿态相对腕部位姿的相对变换
        wrist_pose = wrist_pose.inverse() * T.block(0, 0, 3, 3);
        // std::cout << "wrist_pose: " << std::endl
        //           << wrist_pose << std::endl;
        //根据上面的zyz矩阵可知，wrist_pose(2, 2)等于c5，计算出第五个关节角，acos函数接受[-1,1]之间的实数，返回值的范围是[0,π]
        //这里因为有时候计算出的wrist_pose(2, 2)绝对值可能略略微大于1，所以要进行判断
        // std::cout << "wrist_pose(2, 2): " << wrist_pose(2, 2) << std::endl;
        if (std::fabs(wrist_pose(2, 2)) > 1)
        {
            wrist_pose(2, 2) = (wrist_pose(2, 2) > 0) ? 1 : -1;
        }
        theta[solution_index][5] = acos(wrist_pose(2, 2));
        // std::cout << "theta[solution_index][5]: " << theta[solution_index][5] << ", wrist_pose(2, 2): " << wrist_pose(2, 2) << std::endl;
        //第五个关节不为零的情况
        if (std::fabs(theta[solution_index][5]) > 0.00001)
        {
            //根据wrist_pose(2, 0)计算出第6个关节角，取值范围为(0, pi)
            double cosTheta6 = -wrist_pose(2, 0) / sin(theta[solution_index][5]);
            //防止cosTheta6绝对值略微大于1
            if (std::fabs(cosTheta6) > 1)
            {
                cosTheta6 = (cosTheta6 > 0) ? 1 : -1;
            }
            theta[solution_index][6] = acos(cosTheta6);
            //根据关节6的正弦值的正负,判断其余弦值正负
            if (asin(wrist_pose(2, 1) / sin(theta[solution_index][5])) < 0)
            {
                theta[solution_index][6] = -theta[solution_index][6];
            }
            //根据wrist_pose(0, 2)计算出第4个关节角，取值范围为(0, pi)
            double cosTheta4 = wrist_pose(0, 2) / sin(theta[solution_index][5]);
            //防止cosTheta4绝对值略微大于1
            if (std::fabs(cosTheta4) > 1)
            {
                cosTheta4 = (cosTheta4 > 0) ? 1 : -1;
            }
            theta[solution_index][4] = acos(cosTheta4);
            //根据关节4的正弦值的正负,判断其余弦值正负
            if (asin(wrist_pose(1, 2) / sin(theta[solution_index][5])) < 0)
            {
                theta[solution_index][4] = -theta[solution_index][4];
            }
        }
        //第五个关节为零，机械臂处于奇异位形，令第四个关节关节角度为零
        else
        {
            theta[solution_index][4] = 0;
            //防止wrist_pose(0, 0)绝对值略微大于1
            if (std::fabs(wrist_pose(0, 0)) > 1)
            {
                wrist_pose(0, 0) = (wrist_pose(0, 0) > 0) ? 1 : -1;
            }
            theta[solution_index][6] = acos(wrist_pose(0, 0));
            theta[solution_index + 1][5] = 0;
            theta[solution_index + 1][4] = 0;
            theta[solution_index + 1][6] = theta[solution_index][6];
            return;
        }
        //该位姿对应的第2组解计算，第二组解对应关节5朝反方向弯曲时的姿态
        theta[solution_index + 1][5] = -theta[solution_index][5];
        //theta[1][4]取值范围为(0, pi]则
        if (theta[solution_index][4] >= 0)
        {
            //当关节5朝反方向弯曲同样角度时，关节4也应该转到相反的方向，theta[2][4]取值范围为(0, -pi)
            theta[solution_index + 1][4] = theta[solution_index][4] - M_PI;
        }
        //theta[1][4]取值范围为[-pi, 0)则
        else
        {
            theta[solution_index + 1][4] = theta[solution_index][4] + M_PI;
        }
        //theta[1][6]取值范围为(0, pi)则
        if (theta[solution_index][6] >= 0)
        {
            //当关节5朝反方向弯曲同样角度时，关节6应该转到相反的方向抵消关节4旋转的影响，theta[2][6]取值范围为(0, -pi)
            theta[solution_index + 1][6] = theta[solution_index][6] - M_PI;
        }
        //否则theta[2][6]取值范围为(0, pi)
        else
        {
            theta[solution_index + 1][6] = theta[solution_index][6] + M_PI;
        }
    }

    void inverseKinematics(const Eigen::Matrix4d& T)
    {
        //提取目标位姿的位置值
        double x = T(0, 3), y = T(1, 3), z = T(2, 3);
        /*
        计算机械臂腕部的位置，机械臂第i个关节相对第i-1个关节坐标系的相对变换为:
        |    cosθi           -sinθi             0            a(i-1)   |
        |sinθicosα(i-1)   cosθicosα(i-1)    -sinα(i-1)    -disinα(i-1)|
        |sinθisinα(i-1)   cosθisinα(i-1)     cosα(i-1)     dicosα(i-1)|
        |      0                0               0               1     |
        对于4、5、6轴交于一点的机械臂，α5为90或-90度，cosα5=0所以上式可以写为:
        |    cosθi           -sinθi             0               0     |
        |      0                0           -sinα(i-1)    -disinα(i-1)|
        |sinθisinα(i-1)   cosθisinα(i-1)        0               0     |
        |      0                0               0               1     | 
        已知第6个关节的位姿，再乘以第5个关节相对第6个关节的变换矩阵就可以得到第5个关节的位姿，这里我们只关心位置，所以只需要求出上式的逆矩阵的位置部分:
        |x   x   x   0 |
        |x   x   x   0 |
        |x   x   x  -di|
        |0   0   0   1 |  
        所以腕部的坐标可以按如下公式计算:              
        */
        x -= d_[6] * T(0, 2);
        y -= d_[6] * T(1, 2);
        z -= d_[6] * T(2, 2);
        //计算腕部中心到基坐标系原点的连线在基坐标系xy面的投影与基坐标系x轴的夹角
        //******************************************关节1第一个角度计算******************************************
        //atan2取值范围为(-π，π]，x和y同时为零时atan2结果为0，y为0且x小于0时atan2结果为π
        if (std::fabs(y) < 0.00001 && std::fabs(x) < 0.00001)
        {
            theta[4][1] = theta[3][1] = theta[2][1] = theta[1][1] = 0;
        }
        else
        {
            theta[4][1] = theta[3][1] = theta[2][1] = theta[1][1] = atan2(-y, -x);
        }
        // std::cout << "atan2(-y, -x) " << atan2(y, x) << std::endl;
        //******************************************关节1第二个角度计算******************************************
        //关节1可以取与第一个角度方向相反的另外一个角度，这里要根据第一个角度的关节值确定加π还是减π
        if (theta[1][1] > 0 && theta[1][1] <= M_PI)
        {
            theta[8][1] = theta[7][1] = theta[6][1] = theta[5][1] = theta[1][1] - M_PI;
        }
        else if (theta[1][1] <= 0 && theta[1][1] > -M_PI)
        {
            theta[8][1] = theta[7][1] = theta[6][1] = theta[5][1] = theta[1][1] + M_PI;
        }
        //计算腕部中心到连杆1坐标系原点的距离
        //由连杆间的相对变换矩阵，可以求出连杆1的原点位置，再用腕部位置减去连杆1的原点位置就可以求出该距离
        //连杆1的坐标系一般在基座标系的z轴正上方
        double square_radius_of_wrist_point = pow(x, 2) + pow(y, 2);
        double radius_of_wrist_point = sqrt(square_radius_of_wrist_point);
        double square_length_strech = square_radius_of_wrist_point + pow(z - d_[1], 2);
        double length_strech = sqrt(square_length_strech);
        //计算坐标系2原点到坐标系3原点连线与坐标系2原点到腕部中心连线的夹角，acos函数取值范围（0，π）,angle_joint大于0
        double angle_joint = acos((square_length_of_upperArm_ + square_length_strech - square_length_of_forearm_) / (2 * length_of_upperArm_ * length_strech));
        //******************************************关节2第一种角度计算******************************************
        //关节二与关节一对应，关节外翻，加上夹角angle_joint，函数atan2输入(y, x)
        theta[2][2] = theta[1][2] = atan2(radius_of_wrist_point, z - d_[1]) - angle_joint;
        //关节内凹，减去夹角angle_joint
        theta[4][2] = theta[3][2] = theta[1][2] + 2 * angle_joint;
        //******************************************关节2第二种角度计算******************************************
        //关节二与关节一对应，当关节1方向取相反的方向时，关节2也要取相对于基坐标系z轴对称的另外一个角度
        theta[6][2] = theta[5][2] = -theta[1][2];
        theta[8][2] = theta[7][2] = -theta[3][2];
        //******************************************关节3第一种角度计算******************************************
        //肘关节外翻，atan(lenth_of_link4_/87)为初始状态时，坐标系3到腕部中心连线相对x轴的夹角
        theta[8][3] = theta[7][3] = theta[2][3] = theta[1][3] = M_PI - acos((square_length_of_upperArm_ + square_length_of_forearm_ - square_length_strech) / (2 * length_of_upperArm_ * length_of_forearm_));
        //******************************************关节3第二种角度计算******************************************
        //肘关节内缩
        theta[6][3] = theta[5][3] = theta[4][3] = theta[3][3] = -theta[1][3];
        //************************************关节4,5,6对应第一个位姿的角度计算************************************
        caculateLastThreeJoint(T, 1);
        //************************************关节4,5,6对应第二个位姿的角度计算************************************
        caculateLastThreeJoint(T, 3);
        //************************************关节4,5,6对应第三个位姿的角度计算************************************
        caculateLastThreeJoint(T, 5);
        //************************************关节4,5,6对应第四个位姿的角度计算************************************
        caculateLastThreeJoint(T, 7);
    }
    void inverseKinematics(const Eigen::Matrix4d& T, const double& joint_1)
    {
        double x = T(0, 3), y = T(1, 3), z = T(2, 3);
        x -= d_[6] * T(0, 2);
        y -= d_[6] * T(1, 2);
        z -= d_[6] * T(2, 2);
        if (std::fabs(y) < 0.00001 && std::fabs(x) < 0.00001)
        {
            theta[4][1] = theta[3][1] = theta[2][1] = theta[1][1] = joint_1;
        }
        else
        {
            theta[4][1] = theta[3][1] = theta[2][1] = theta[1][1] = atan2(-y, -x);
        }
        if (theta[1][1] > 0 && theta[1][1] <= M_PI)
        {
            theta[8][1] = theta[7][1] = theta[6][1] = theta[5][1] = theta[1][1] - M_PI;
        }
        else if (theta[1][1] <= 0 && theta[1][1] > -M_PI)
        {
            theta[8][1] = theta[7][1] = theta[6][1] = theta[5][1] = theta[1][1] + M_PI;
        }
        //计算腕部中心到连杆1坐标系原点的距离
        //由连杆间的相对变换矩阵，可以求出连杆1的原点位置，再用腕部位置减去连杆1的原点位置就可以求出该距离
        //连杆1的坐标系一般在基座标系的z轴正上方
        double square_radius_of_wrist_point = pow(x, 2) + pow(y, 2);
        double radius_of_wrist_point = sqrt(square_radius_of_wrist_point);
        double square_length_strech = square_radius_of_wrist_point + pow(z - d_[1], 2);
        double length_strech = sqrt(square_length_strech);
        //计算坐标系2原点到坐标系3原点连线与坐标系2原点到腕部中心连线的夹角，acos函数取值范围（0，π）,angle_joint大于0
        double angle_joint = acos((square_length_of_upperArm_ + square_length_strech - square_length_of_forearm_) / (2 * length_of_upperArm_ * length_strech));
        //******************************************关节2第一种角度计算******************************************
        //关节二与关节一对应，关节外翻，加上夹角angle_joint，函数atan2输入(y, x)
        theta[2][2] = theta[1][2] = atan2(radius_of_wrist_point, z - d_[1]) - angle_joint;
        //关节内凹，减去夹角angle_joint
        theta[4][2] = theta[3][2] = theta[1][2] + 2 * angle_joint;
        //******************************************关节2第二种角度计算******************************************
        //关节二与关节一对应，当关节1方向取相反的方向时，关节2也要取相对于基坐标系z轴对称的另外一个角度
        theta[6][2] = theta[5][2] = -theta[1][2];
        theta[8][2] = theta[7][2] = -theta[3][2];
        //******************************************关节3第一种角度计算******************************************
        //肘关节外翻，atan(lenth_of_link4_/87)为初始状态时，坐标系3到腕部中心连线相对x轴的夹角
        theta[8][3] = theta[7][3] = theta[2][3] = theta[1][3] = M_PI - acos((square_length_of_upperArm_ + square_length_of_forearm_ - square_length_strech) / (2 * length_of_upperArm_ * length_of_forearm_));
        //******************************************关节3第二种角度计算******************************************
        //肘关节内缩
        theta[6][3] = theta[5][3] = theta[4][3] = theta[3][3] = -theta[1][3];
        //************************************关节4,5,6对应第一个位姿的角度计算************************************
        caculateLastThreeJoint(T, 1);
        //************************************关节4,5,6对应第二个位姿的角度计算************************************
        caculateLastThreeJoint(T, 3);
        //************************************关节4,5,6对应第三个位姿的角度计算************************************
        caculateLastThreeJoint(T, 5);
        //************************************关节4,5,6对应第四个位姿的角度计算************************************
        caculateLastThreeJoint(T, 7);
    }
};

#endif
