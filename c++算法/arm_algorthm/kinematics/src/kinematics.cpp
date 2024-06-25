#include "arm_planning/inverseKinematics.h"
/*
lite6 DH参数:
const double d[6+1] = { 0, 243.3, 0, 0, 227.6, 0, 61.5};//第0个不用
const double a[6] = {0, 0, 200, 87, 0, 0};//a有0，没有6
const double alpha[6] = { 0, -90, 180, 90, 90, -90};//alpha有0，没有6
沿x轴的位移矩阵trans(a(i-1), 0, 0)=
|1      0      0    a(i-1)|
|0      1      0      0   |
|0      0      1      0   |
|0      0      0      1   |
绕x轴的旋转矩阵rot(x, α(i-1))=
|1        0             0       0|
|0   cos(α(i-1))  -sin(α(i-1))  0|
|0   sin(α(i-1))   cos(α(i-1))  0|
|0        0             0       1|
沿z轴的位移矩阵trans(0, 0, di)=
|1      0      0      0 |
|0      1      0      0 |
|0      0      1      di|
|0      0      0      1 |
绕z轴的旋转矩阵rot(z, θi)=
|cos(θi)  -sin(θi)     0       0|
|sin(θi)   cos(θi)     0       0|
|   0         0        0       0|
|   0         0        0       1|
连杆i相对连杆i-1的变换矩阵trans(a(i-1), 0, 0)rot(x, α(i-1))trans(0, 0, di)rot(z, θi)=
|     cos(θi)             -sin(θi)             0            a(i-1)    |
|sin(θi)cos(α(i-1))   cos(θi)cos(α(i-1))  -sin(α(i-1))  -disin(α(i-1))|
|sin(θi)cos(α(i-1))   cos(θi)cos(α(i-1))   cos(α(i-1))   dicos(α(i-1))|
|        0                    0                 0              1      |
*/
Eigen::Matrix4d Kinematics::kinematics(const std::vector<double>& theta_input, const int& index_joint=6)  //参数为弧度表示的角度
{
    Eigen::Matrix4d T[6];
    for (int i = 0; i < 6; i++) 
    {
        T[i](0, 0) = cos(theta_input[i]);
        T[i](0, 1) = -sin(theta_input[i]);
        T[i](0, 2) = 0;
        T[i](0, 3) = a[i] / 1000;
        T[i](1, 0) = sin(theta_input[i]) * cos(alpha[i] / 180 * M_PI);
        T[i](1, 1) = cos(theta_input[i]) * cos(alpha[i] / 180 * M_PI);
        T[i](1, 2) = -sin(alpha[i] / 180 * M_PI);
        T[i](1, 3) = -sin(alpha[i] / 180 * M_PI) * d[i] / 1000;
        T[i](2, 0) = sin(theta_input[i]) * sin(alpha[i] / 180 * M_PI);
        T[i](2, 1) = cos(theta_input[i]) * sin(alpha[i] / 180 * M_PI);
        T[i](2, 2) = cos(alpha[i] / 180 * M_PI);
        T[i](2, 3) = cos(alpha[i] / 180 * M_PI) * d[i] / 1000;
        T[i](3, 0) = 0;
        T[i](3, 1) = 0;
        T[i](3, 2) = 0;
        T[i](3, 3) = 1;
    }
    Eigen::Matrix4d transform_output;
    switch (index_joint) {
        case 1:
            transform_output = T[0];
            break;
        case 2:
            transform_output = T[0] * T[1];
            break;
        case 3:
            transform_output = T[0] * T[1] * T[2];
            break;
        case 4:
            transform_output = T[0] * T[1] * T[2] * T[3];
            break;
        case 5:
            transform_output = T[0] * T[1] * T[2] * T[3] * T[4];
            break;
        case 6:
            transform_output = T[0] * T[1] * T[2] * T[3] * T[4] * T[5];
            break;
        default:
            std::cout << "Kinematics模块:输入有误,请输入1到6的整数!" << std::endl;
            break;
    }
    return transform_output;
}
/*
逆运动学
输出的八组解theta[1~8]对应的机械臂位形:
1.轴1使朝目标方向;    轴2使肘关节外翻(轴2负方向偏)；
轴3使使肘关节外翻；第5个关节值为正(初始状态时，顺时针旋转)； 2.轴1使朝目标方向;
轴2使肘关节外翻(轴2负方向偏)；
轴3使使肘关节外翻；第5个关节值为负(初始状态时，逆时针旋转)； 3.轴1使朝目标方向;
轴2使肘关节内凹(轴2正方向偏)；
轴3使使肘关节内凹；第5个关节值为正(初始状态时，顺时针旋转)； 4.轴1使朝目标方向;
轴2使肘关节内凹(轴2正方向偏)；
轴3使使肘关节内凹；第5个关节值为负(初始状态时，逆时针旋转)；
5.轴1使朝目标相反方向; 轴2使肘关节内凹(轴2正方向偏)；
轴3使使肘关节外翻；第5个关节值为正(初始状态时，顺时针旋转)；
6.轴1使朝目标相反方向; 轴2使肘关节内凹(轴2正方向偏)；
轴3使使肘关节外翻；第5个关节值为负(初始状态时，逆时针旋转)；
7.轴1使朝目标相反方向; 轴2使肘关节外翻(轴2负方向偏)；
轴3使使肘关节内凹；第5个关节值为正(初始状态时，顺时针旋转)；
8.轴1使朝目标相反方向; 轴2使肘关节外翻(轴2负方向偏)；
轴3使使肘关节内凹；第5个关节值为负(初始状态时，逆时针旋转)；
*/
InverseKinematics::InverseKinematics()
{
    matrix_rotate_aroundx << 1, 0, 0, 0, 0, -1, 0, 1, 0;
    std::cout << "InverseKinematics模块:初始化逆运动学模块完成！" << std::endl;
}
Eigen::Matrix3d InverseKinematics::getRotationMatrix(const double joint1,
                                                     const double joint2,
                                                     const double joint3)
{
    Eigen::Matrix3d wrist_pose;
    wrist_pose(0, 0) = cos(joint1) * cos(joint2) * cos(joint3) +
                       cos(joint1) * sin(joint2) * sin(joint3);
    wrist_pose(0, 1) = -cos(joint1) * cos(joint2) * sin(joint3) +
                       cos(joint1) * sin(joint2) * cos(joint3);
    wrist_pose(0, 2) = sin(joint1);
    wrist_pose(1, 0) = sin(joint1) * cos(joint2) * cos(joint3) +
                       sin(joint1) * sin(joint2) * sin(joint3);
    wrist_pose(1, 1) = -sin(joint1) * cos(joint2) * sin(joint3) +
                       sin(joint1) * sin(joint2) * cos(joint3);
    wrist_pose(1, 2) = -cos(joint1);
    wrist_pose(2, 0) = -sin(joint2) * cos(joint3) + cos(joint2) * sin(joint3);
    wrist_pose(2, 1) = sin(joint2) * sin(joint3) + cos(joint2) * cos(joint3);
    wrist_pose(2, 2) = 0;
    return wrist_pose;
}
/*
欧拉角zyz对应的旋转矩阵:  由于DH参数的原因这里的sin5都取负号
|c4c5c6-s4s6  -c6s4-c4c5s6  -c4s5|
|c4s6+c5c6s4   c4c6-c5s4s6  -s4s5|
|c6s5         -s5s6         c5   |
acos取值范围(0, pi);asin取值范围(-pi/2, pi/2)
*/

void InverseKinematics::caculateLastThreeJoint(const Eigen::Matrix4d& T,
                                               int index_Solustion)
{
    //由前三个关节轴的角度解算出机械臂腕部的位姿1
    Eigen::Matrix3d wrist_pose = getRotationMatrix(theta[index_Solustion][1],
                                                   theta[index_Solustion][2],
                                                   theta[index_Solustion][3]) *
                                 matrix_rotate_aroundx;
    wrist_pose = wrist_pose.inverse() * T.block(0, 0, 3, 3);
    theta[index_Solustion][5] =
        acos(wrist_pose(2, 2));  //根据上面的zyz矩阵可知，wrist_pose(2,
                                 // 2)等于c5，计算出第五个关节角
    if (theta[index_Solustion][5] != 0) {
        theta[index_Solustion][6] =
            acos(wrist_pose(2, 0) /
                 sin(theta[index_Solustion]
                          [5]));  //根据wrist_pose(2,
                                  // 0)等于c6s5计算出第6个关节角，取值范围为(0,
                                  // pi)
        if (asin(-wrist_pose(2, 1) / sin(theta[index_Solustion][5])) <
            0)  //根据正弦值判断轴6在坐标系上部还是下部,从而判断余弦正负
        {
            theta[index_Solustion][6] = -theta[index_Solustion][6];
        }
        theta[index_Solustion][4] =
            acos(-wrist_pose(0, 2) / sin(theta[index_Solustion][5]));
        if (asin(-wrist_pose(1, 2) / sin(theta[index_Solustion][5])) <
            0)  //根据正弦值判断轴4在坐标系上部还是下部，从而判断余弦正负
        {
            theta[index_Solustion][4] = -theta[index_Solustion][4];
        }
    } else  //第五个关节为零，机械臂处于奇异位形，令第四个关节关节角度为零
    {
        theta[index_Solustion][4] = 0;
        theta[index_Solustion][6] = acos(wrist_pose(0, 0));
    }
    //该位姿对应的第2组解计算
    theta[index_Solustion + 1][5] =
        -theta[index_Solustion][5];  //第二组解对应关节5朝反方向弯曲时的姿态
    if (theta[index_Solustion][4] >= 0)  // theta[1][4]取值范围为(0, pi)则
    {
        theta[index_Solustion + 1][4] =
            theta[index_Solustion][4] -
            M_PI;  //当关节5朝反方向弯曲同样角度时，关节4也应该转到相反的方向，theta[2][4]取值范围为(0,
                   //-pi)
    } else {
        theta[index_Solustion + 1][4] =
            theta[index_Solustion][4] +
            M_PI;  //否则theta[2][4]取值范围为(0, pi)
    }
    if (theta[index_Solustion][6] >= 0)  // theta[1][6]取值范围为(0, pi)则
    {
        theta[index_Solustion + 1][6] =
            theta[index_Solustion][6] -
            M_PI;  //当关节5朝反方向弯曲同样角度时，关节6应该转到相反的方向抵消关节4旋转的影响，theta[2][6]取值范围为(0,
                   //-pi)
    } else {
        theta[index_Solustion + 1][6] =
            theta[index_Solustion][6] +
            M_PI;  //否则theta[2][6]取值范围为(0, pi)
    }
}

void InverseKinematics::inverseKinematics(const Eigen::Matrix4d& T)
{
    double x = T(0, 3), y = T(1, 3), z = T(2, 3);  //提取目标位姿的位置值
    //计算机械臂腕部的位置，机械臂最后关节的长度乘以最后关节坐标系z轴方向得到腕部位置
    x -= lenth_of_link6_ * T(0, 2);
    y -= lenth_of_link6_ * T(1, 2);
    z -= lenth_of_link6_ * T(2, 2);
    //关节1第一个角度计算,atan2取值范围为(-π，π],x和y同时为零时atan2出错,y为0且x小于0时atan2结果为PI
    theta[1][1] = atan2(
        y,
        x);  //计算腕部中心到基坐标系原点的连线在基坐标系xy面的投影与基坐标系x轴的夹角
    theta[2][1] = theta[1][1];
    theta[3][1] = theta[1][1];
    theta[4][1] = theta[1][1];
    //关节1第二个角度计算，关节1可以取与第一个角度方向相反的另外一个角度，这里要根据第一个角度的关节值确定加π还是减π
    if (theta[1][1] > 0 && theta[1][1] <= M_PI) {
        theta[5][1] = theta[1][1] - M_PI;
        theta[6][1] = theta[1][1] - M_PI;
        theta[7][1] = theta[1][1] - M_PI;
        theta[8][1] = theta[1][1] - M_PI;
    } else if (theta[1][1] <= 0 && theta[1][1] > -M_PI) {
        theta[5][1] = theta[1][1] + M_PI;
        theta[6][1] = theta[1][1] + M_PI;
        theta[7][1] = theta[1][1] + M_PI;
        theta[8][1] = theta[1][1] + M_PI;
    }
    double length_strech =
        pow(x, 2) + pow(y, 2) +
        pow(z - height_of_link1_, 2);  //计算腕部中心到连杆1坐标系原点的距离
    //计算坐标系2原点到坐标系3原点连线与坐标系2原点到腕部中心连线的夹角
    double angle_joint = acos(
        (pow(lenth_of_link2_, 2) + length_strech - square_of_link34_) /
        (2 * lenth_of_link2_ *
         sqrt(length_strech)));  // acos函数取值范围（0，π）,angle_joint大于0
    //关节2第一种角度计算，关节二与关节一对应
    theta[1][2] = -(atan2(z - height_of_link1_, sqrt(pow(x, 2) + pow(y, 2))) +
                    angle_joint);  //关节外翻，加上夹角angle_joint
    theta[2][2] = theta[1][2];
    theta[3][2] = -(atan2(z - height_of_link1_, sqrt(pow(x, 2) + pow(y, 2))) -
                    angle_joint);  //关节内凹，减去夹角angle_joint
    theta[4][2] = theta[3][2];
    //关节2第二种角度计算，关节二与关节一对应。当关节1方向取相反的方向时，关节2也要取相对于基坐标系z轴对称的另外一个角度
    theta[5][2] = -M_PI - theta[1][2];
    theta[6][2] = theta[5][2];
    theta[7][2] = -M_PI - theta[3][2];
    theta[8][2] = theta[7][2];
    //关节3第一种角度计算，肘关节外翻
    theta[1][3] =
        acos((pow(lenth_of_link2_, 2) + square_of_link34_ - length_strech) /
             (2 * lenth_of_link2_ * length_of_link34_)) +
        atan(lenth_of_link4_ / lenth_of_link3_) -
        M_PI;  // atan(lenth_of_link4_/87)为初始状态时，坐标系3到腕部中心连线相对x轴的夹角
    theta[2][3] = theta[1][3];
    theta[7][3] = theta[1][3];
    theta[8][3] = theta[1][3];
    //关节3第二种角度计算，肘关节内缩
    theta[3][3] = 2 * atan(lenth_of_link4_ / lenth_of_link3_) - theta[1][3];
    theta[4][3] = theta[3][3];
    theta[5][3] = theta[3][3];
    theta[6][3] = theta[3][3];
    //关节4,5,6对应第一个位姿的角度计算
    caculateLastThreeJoint(T, 1);
    //关节4,5,6对应第二个位姿的角度计算
    caculateLastThreeJoint(T, 3);
    //关节4,5,6对应第三个位姿的角度计算
    caculateLastThreeJoint(T, 5);
    //关节4,5,6对应第四个位姿的角度计算
    caculateLastThreeJoint(T, 7);
}
