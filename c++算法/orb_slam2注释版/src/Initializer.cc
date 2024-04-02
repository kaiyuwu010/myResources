/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Initializer.h"
#include "Thirdparty/DBoW2/DUtils/Random.h"
#include "Optimizer.h"
#include "ORBmatcher.h"
#include<thread>

namespace ORB_SLAM2
{

//初始化器用第一帧图像进行初始化调用的构造函数
Initializer::Initializer(const Frame &ReferenceFrame, float sigma, int iterations)
{
    mK = ReferenceFrame.mK.clone();
    mvKeys1 = ReferenceFrame.mvKeysUn;
    mSigma = sigma;
    mSigma2 = sigma*sigma;
    mMaxIterations = iterations;
}

bool Initializer::Initialize(const Frame &CurrentFrame, const vector<int> &vMatches12, cv::Mat &R21, cv::Mat &t21,
                             vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated)
{
    // Fill structures with current keypoints and matches with reference frame
    // Reference Frame: 1, Current Frame: 2
    mvKeys2 = CurrentFrame.mvKeysUn;
    mvMatches12.clear();
    mvMatches12.reserve(mvKeys2.size());
    mvbMatched1.resize(mvKeys1.size());
    for(size_t i=0, iend=vMatches12.size();i<iend; i++)
    {
        if(vMatches12[i]>=0)
        {
            mvMatches12.push_back(make_pair(i,vMatches12[i]));
            mvbMatched1[i]=true;
        }
        else
            mvbMatched1[i]=false;
    }
    const int N = mvMatches12.size();

    //生成ransac随机采样集合
    vector<size_t> vAllIndices;//声明存放序号的向量
    vAllIndices.reserve(N);
    vector<size_t> vAvailableIndices;//声明可用的序号向量，存放的是没有被使用的序号
    for(int i=0; i<N; i++)
    {
        vAllIndices.push_back(i);//按顺序给该向量赋序号值
    }
    // 为每次RANSAC迭代生成8组不重复的随机点序号
    mvSets = vector< vector<size_t> >(mMaxIterations,vector<size_t>(8,0));
    DUtils::Random::SeedRandOnce(0);//初始化随机数种子
    for(int it=0; it<mMaxIterations; it++)
    {
        vAvailableIndices = vAllIndices;//每次迭代重新从所有点对里选择，所有这里要刷新vAvailableIndices向量
        // 为每次迭代生成不重复的8对随机匹配点序号，存放进mvSets变量
        for(size_t j=0; j<8; j++)
        {
            int randi = DUtils::Random::RandomInt(0,vAvailableIndices.size()-1);//生成0到最大序号之间的随机数
            int idx = vAvailableIndices[randi];//存放该随机数对应的序号
            mvSets[it][j] = idx;
            vAvailableIndices[randi] = vAvailableIndices.back();//把这个位置的序号替换为向量最后一个序号
            vAvailableIndices.pop_back();//把向量最后一个序号取出来
        }
    }

    //开启两个线程并行计算F矩阵和H矩阵
    vector<bool> vbMatchesInliersH, vbMatchesInliersF;
    float SH, SF;
    cv::Mat H, F;
    thread threadH(&Initializer::FindHomography,this,ref(vbMatchesInliersH), ref(SH), ref(H));
    thread threadF(&Initializer::FindFundamental,this,ref(vbMatchesInliersF), ref(SF), ref(F));
    //等待线程完成工作
    threadH.join();
    threadF.join();
    //计算H矩阵占两种矩阵的得分和的比例
    float RH = SH/(SH+SF);
    //如果H矩阵得分占比大于0.4就使用H矩阵（算法更偏向使用H矩阵）
    if(RH>0.40)
        return ReconstructH(vbMatchesInliersH,H,mK,R21,t21,vP3D,vbTriangulated,1.0,50);
    else
        return ReconstructF(vbMatchesInliersF,F,mK,R21,t21,vP3D,vbTriangulated,1.0,50);
    return false;
}


void Initializer::FindHomography(vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21)
{
    //匹配点对数量
    const int N = mvMatches12.size();
    //归一化坐标
    vector<cv::Point2f> vPn1, vPn2;
    cv::Mat T1, T2;
    Normalize(mvKeys1, vPn1, T1);
    Normalize(mvKeys2, vPn2, T2);
    cv::Mat T2inv = T2.inv();
    //初始化内点和得分变量
    score = 0.0;
    vbMatchesInliers = vector<bool>(N,false);
    //声明存放一组关键点的向量
    vector<cv::Point2f> vPn1i(8);
    vector<cv::Point2f> vPn2i(8);
    cv::Mat H21i, H12i;
    vector<bool> vbCurrentInliers(N,false);
    float currentScore;
    //进行RANSAC迭代，找到得分最高的H矩阵
    for(int it=0; it<mMaxIterations; it++)
    {
        //遍历8个点对
        for(size_t j=0; j<8; j++)
        {
            int idx = mvSets[it][j];

            vPn1i[j] = vPn1[mvMatches12[idx].first];
            vPn2i[j] = vPn2[mvMatches12[idx].second];
        }
        //计算单应矩阵
        cv::Mat Hn = ComputeH21(vPn1i,vPn2i);
        H21i = T2inv*Hn*T1;
        H12i = H21i.inv();
        //计算H矩阵得分
        currentScore = CheckHomography(H21i, H12i, vbCurrentInliers, mSigma);
        //找出最高得分，并记录对应矩阵、得分和内点序号
        if(currentScore>score)
        {
            H21 = H21i.clone();
            vbMatchesInliers = vbCurrentInliers;
            score = currentScore;
        }
    }
}


void Initializer::FindFundamental(vector<bool> &vbMatchesInliers, float &score, cv::Mat &F21)
{
    // Number of putative matches
    const int N = vbMatchesInliers.size();
    vector<cv::Point2f> vPn1, vPn2;//声明存放正则坐标的关键点坐标向量
    cv::Mat T1, T2;
    Normalize(mvKeys1, vPn1, T1);//参考帧关键点向量mvKeys1进行中心化和归一化
    Normalize(mvKeys2, vPn2, T2);//当前帧关键点向量mvKeys2进行中心化和归一化
    cv::Mat T2t = T2.t();
    score = 0.0;//最高得分变量置为0
    vbMatchesInliers = vector<bool>(N,false);//初始化内点标志向量
    //8对匹配点为一组计算F矩阵，声明存放一组关键点的向量
    vector<cv::Point2f> vPn1i(8);
    vector<cv::Point2f> vPn2i(8);
    cv::Mat F21i;
    vector<bool> vbCurrentInliers(N,false);//声明当前内点标志向量
    float currentScore;
    //进行RANSAC迭代，保存得分最高的解
    for(int it=0; it<mMaxIterations; it++)
    {
        // 根据mvSets中选好的随机8对匹配点序号，存放进vPn1i和vPn12变量
        for(int j=0; j<8; j++)
        {
            int idx = mvSets[it][j];

            vPn1i[j] = vPn1[mvMatches12[idx].first];
            vPn2i[j] = vPn2[mvMatches12[idx].second];
        }
        cv::Mat Fn = ComputeF21(vPn1i,vPn2i);//计算F矩阵
        F21i = T2t*Fn*T1;
        currentScore = CheckFundamental(F21i, vbCurrentInliers, mSigma);
        //如果检查得到的F矩阵得分最高，则更新F矩阵F21、内点向量vbMatchesInliers、最高得分score
        if(currentScore>score)
        {
            F21 = F21i.clone();
            vbMatchesInliers = vbCurrentInliers;
            score = currentScore;
        }
    }
}


cv::Mat Initializer::ComputeH21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2)
{
    const int N = vP1.size();//获取匹配点数量
    cv::Mat A(2*N,9,CV_32F);//声明要进行分解的矩阵
    for(int i=0; i<N; i++)
    {
        //取出匹配点对的坐标值
        const float u1 = vP1[i].x;
        const float v1 = vP1[i].y;
        const float u2 = vP2[i].x;
        const float v2 = vP2[i].y;
        //初始化A矩阵，每对匹配点提供两个约束有两个方程，只需要4对点
        A.at<float>(2*i,0) = 0.0;
        A.at<float>(2*i,1) = 0.0;
        A.at<float>(2*i,2) = 0.0;
        A.at<float>(2*i,3) = -u1;
        A.at<float>(2*i,4) = -v1;
        A.at<float>(2*i,5) = -1;
        A.at<float>(2*i,6) = v2*u1;
        A.at<float>(2*i,7) = v2*v1;
        A.at<float>(2*i,8) = v2;

        A.at<float>(2*i+1,0) = u1;
        A.at<float>(2*i+1,1) = v1;
        A.at<float>(2*i+1,2) = 1;
        A.at<float>(2*i+1,3) = 0.0;
        A.at<float>(2*i+1,4) = 0.0;
        A.at<float>(2*i+1,5) = 0.0;
        A.at<float>(2*i+1,6) = -u2*u1;
        A.at<float>(2*i+1,7) = -u2*v1;
        A.at<float>(2*i+1,8) = -u2;
    }
    cv::Mat u,w,vt;
    //求解Ax=0，若存在非零解A矩阵的SVD分解的vt的最后一行就是该方程的解
    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    //把行向量reshape为mat矩阵类型
    return vt.row(8).reshape(0, 3);
}

cv::Mat Initializer::ComputeF21(const vector<cv::Point2f> &vP1,const vector<cv::Point2f> &vP2)
{
    const int N = vP1.size();//读取匹配点的数量，程序中每次计算F矩阵用到了8对点(F矩阵有9个元素，由于F矩阵的可缩放特性，自由度减1，由于该矩阵的秩为2自由度再减1，所以实际7对点就可以)
    cv::Mat A(N,9,CV_32F);//构造N行9列的目标矩阵
    for(int i=0; i<N; i++)
    {
        //取出匹配点对的坐标值
        const float u1 = vP1[i].x;
        const float v1 = vP1[i].y;
        const float u2 = vP2[i].x;
        const float v2 = vP2[i].y;
        //初始化A矩阵
        A.at<float>(i,0) = u2*u1;
        A.at<float>(i,1) = u2*v1;
        A.at<float>(i,2) = u2;
        A.at<float>(i,3) = v2*u1;
        A.at<float>(i,4) = v2*v1;
        A.at<float>(i,5) = v2;
        A.at<float>(i,6) = u1;
        A.at<float>(i,7) = v1;
        A.at<float>(i,8) = 1;
    }
    cv::Mat u,w,vt;
    //u,vt类型cv::OutputArray
    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    //求解Ax=0，若存在非零解A矩阵的SVD分解的vt的最后一行就是该方程的解
    cv::Mat Fpre = vt.row(8).reshape(0, 3);
    //再次进行SVD分解，若Fpre是正确的解，它的秩应该是2(反对称矩阵秩为2)，所以令SVD分解后的第三个奇异值为零可以更加接近真实解
    cv::SVDecomp(Fpre,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    w.at<float>(2)=0;
    //把第三个奇异值置为零后再把它们组合起来更接近真实解
    return  u*cv::Mat::diag(w)*vt;
}

float Initializer::CheckHomography(const cv::Mat &H21, const cv::Mat &H12, vector<bool> &vbMatchesInliers, float sigma)
{   
    const int N = mvMatches12.size();
    //读取H21矩阵的值
    const float h11 = H21.at<float>(0,0);
    const float h12 = H21.at<float>(0,1);
    const float h13 = H21.at<float>(0,2);
    const float h21 = H21.at<float>(1,0);
    const float h22 = H21.at<float>(1,1);
    const float h23 = H21.at<float>(1,2);
    const float h31 = H21.at<float>(2,0);
    const float h32 = H21.at<float>(2,1);
    const float h33 = H21.at<float>(2,2);
    //读取H12矩阵的值，H12是H21的逆矩阵
    const float h11inv = H12.at<float>(0,0);
    const float h12inv = H12.at<float>(0,1);
    const float h13inv = H12.at<float>(0,2);
    const float h21inv = H12.at<float>(1,0);
    const float h22inv = H12.at<float>(1,1);
    const float h23inv = H12.at<float>(1,2);
    const float h31inv = H12.at<float>(2,0);
    const float h32inv = H12.at<float>(2,1);
    const float h33inv = H12.at<float>(2,2);
    vbMatchesInliers.resize(N);
    float score = 0;
    const float th = 5.991; //自由度为2的卡方分布，显著性水平为0.05，对应的临界阈值
    const float invSigmaSquare = 1.0/(sigma*sigma);	//信息矩阵，或协方差矩阵的逆矩阵，消除不同金字塔层级误差大小不同的影响
    //遍历匹配点对
    for(int i=0; i<N; i++)
    {
        bool bIn = true;
        //取出匹配关键点对
        const cv::KeyPoint &kp1 = mvKeys1[mvMatches12[i].first];
        const cv::KeyPoint &kp2 = mvKeys2[mvMatches12[i].second];
        //取出匹配点对像素坐标
        const float u1 = kp1.pt.x;
        const float v1 = kp1.pt.y;
        const float u2 = kp2.pt.x;
        const float v2 = kp2.pt.y;
        //计算在第1幅图片的重投影坐标，x2in1 = H12*x2，这里w2in1inv的值应该为1，之所以把它乘到下面是为了把它的误差也考虑进去
        const float w2in1inv = 1.0/(h31inv*u2+h32inv*v2+h33inv);
        const float u2in1 = (h11inv*u2+h12inv*v2+h13inv)*w2in1inv;
        const float v2in1 = (h21inv*u2+h22inv*v2+h23inv)*w2in1inv;
        //计算投影点与特征点的距离的平方
        const float squareDist1 = (u1-u2in1)*(u1-u2in1)+(v1-v2in1)*(v1-v2in1);
        //乘上权重invSigmaSquare，invSigmaSquare=1
        const float chiSquare1 = squareDist1*invSigmaSquare;
        //若平方距离大于阈值，改匹配点记为外点。否则把得分加上这个值与阈值的差
        if(chiSquare1>th)
            bIn = false;
        else
            score += th - chiSquare1;
        //计算在第2幅图片的重投影坐标，x1in2 = H21*x1
        const float w1in2inv = 1.0/(h31*u1+h32*v1+h33);
        const float u1in2 = (h11*u1+h12*v1+h13)*w1in2inv;
        const float v1in2 = (h21*u1+h22*v1+h23)*w1in2inv;
        //计算投影点与特征点的距离的平方
        const float squareDist2 = (u2-u1in2)*(u2-u1in2)+(v2-v1in2)*(v2-v1in2);
        //乘上权重invSigmaSquare，invSigmaSquare=1
        const float chiSquare2 = squareDist2*invSigmaSquare;
        //若平方距离大于阈值，改匹配点记为外点。否则把得分加上这个值与阈值的差
        if(chiSquare2>th)
            bIn = false;
        else
            score += th - chiSquare2;
        //若两个投影都满足阈值要求，标记为内点
        if(bIn)
            vbMatchesInliers[i]=true;
        else
            vbMatchesInliers[i]=false;
    }
    return score;
}

float Initializer::CheckFundamental(const cv::Mat &F21, vector<bool> &vbMatchesInliers, float sigma)
{
    const int N = mvMatches12.size();
    //读取F矩阵的值
    const float f11 = F21.at<float>(0,0);
    const float f12 = F21.at<float>(0,1);
    const float f13 = F21.at<float>(0,2);
    const float f21 = F21.at<float>(1,0);
    const float f22 = F21.at<float>(1,1);
    const float f23 = F21.at<float>(1,2);
    const float f31 = F21.at<float>(2,0);
    const float f32 = F21.at<float>(2,1);
    const float f33 = F21.at<float>(2,2);
    vbMatchesInliers.resize(N);
    //得分置为0
    float score = 0;
    //初始化阈值
    const float th = 3.841; //自由度为1的卡方分布，显著性水平为0.05，对应的临界阈值
    const float thScore = 5.991; //自由度为2的卡方分布，显著性水平为0.05，对应的临界阈值
    const float invSigmaSquare = 1.0/(sigma*sigma);//信息矩阵或协方差矩阵的逆矩阵，程序中invSigmaSquare的值为1，消除不同金字塔层级误差大小不同的影响
    //针对该F矩阵，把每对匹配点带入检验
    for(int i=0; i<N; i++)
    {
        bool bIn = true;//是否为内点标志位，初始为true
        const cv::KeyPoint &kp1 = mvKeys1[mvMatches12[i].first];//读取匹配关键点1
        const cv::KeyPoint &kp2 = mvKeys2[mvMatches12[i].second];//读取匹配关键点2
        //读取图像坐标
        const float u1 = kp1.pt.x;
        const float v1 = kp1.pt.y;
        const float u2 = kp2.pt.x;
        const float v2 = kp2.pt.y;
        //利用F矩阵进行重投影，把关键点1进行重投影，然后与关键点2进行比较
        //l2=F21x1=(a2,b2,c2)
        const float a2 = f11*u1+f12*v1+f13;
        const float b2 = f21*u1+f22*v1+f23;
        const float c2 = f31*u1+f32*v1+f33;
        //计算误差e=(a*p2.x + b*p2.y + c)/sqrt(a*a + b*b)的平方，误差为点到极线的距离
        const float num2 = a2*u2+b2*v2+c2;
        const float squareDist1 = num2*num2/(a2*a2+b2*b2);
        //带权重误差
        const float chiSquare1 = squareDist1*invSigmaSquare;
        //带权重误差大于阈值则放弃该点对，置为外点
        if(chiSquare1>th)
            bIn = false;
        else
            score += thScore - chiSquare1;
        //利用F矩阵进行重投影，把关键点2进行重投影，然后与关键点1进行比较
        //l1 =x2tF21=(a1,b1,c1)
        const float a1 = f11*u2+f21*v2+f31;
        const float b1 = f12*u2+f22*v2+f32;
        const float c1 = f13*u2+f23*v2+f33;
        //计算误差e=(a*p2.x + b*p2.y + c)/sqrt(a*a + b*b)的平方，误差为点到极线的距离，这里的squareDist2为距离的平方，距离的期望值为0，自由度为1，
        const float num1 = a1*u1+b1*v1+c1;
        const float squareDist2 = num1*num1/(a1*a1+b1*b1);
        //带权重误差
        const float chiSquare2 = squareDist2*invSigmaSquare;
        //带权重误差大于阈值则放弃该点对，置为外点
        if(chiSquare2>th)
            bIn = false;
        else
            score += thScore - chiSquare2;
        //若两个投影都满足阈值要求，标记为内点
        if(bIn)
            vbMatchesInliers[i]=true;
        else
            vbMatchesInliers[i]=false;
    }
    return score;
}

bool Initializer::ReconstructF(vector<bool> &vbMatchesInliers, cv::Mat &F21, cv::Mat &K, cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated)
{
    //计算内点个数
    int N=0;
    for(size_t i=0, iend = vbMatchesInliers.size() ; i<iend; i++)
        if(vbMatchesInliers[i])
            N++;
    cv::Mat E21 = K.t()*F21*K;//计算本质矩阵
    cv::Mat R1, R2, t;
    // 从本质矩阵求解两个R解和两个t解，共四组解
    // 不过由于两个t解互为相反数，因此这里先只获取一个
    // 虽然这个函数对t有归一化，但并没有决定单目整个SLAM过程的尺度，因为CreateInitialMapMonocular函数对3D点深度会缩放，然后反过来对t有改变.
    // 注意下文中的符号“'”表示矩阵的转置
    //                          |0 -1  0|
    // E = U Sigma V'   let W = |1  0  0|
    //                          |0  0  1|
    // 得到4个解 E = [R|t]
    // R1 = UWV' R2 = UW'V' t1 = U3 t2 = -U3
    DecomposeE(E21,R1,R2,t);  
    cv::Mat t1=t;
    cv::Mat t2=-t;

    // Reconstruct with the 4 hyphoteses and check
    vector<cv::Point3f> vP3D1, vP3D2, vP3D3, vP3D4;
    vector<bool> vbTriangulated1,vbTriangulated2,vbTriangulated3, vbTriangulated4;
    float parallax1,parallax2, parallax3, parallax4;
    //计算四种位姿对应匹配点中，匹配比较好的点对的数量
    int nGood1 = CheckRT(R1,t1,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D1, 4.0*mSigma2, vbTriangulated1, parallax1);
    int nGood2 = CheckRT(R2,t1,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D2, 4.0*mSigma2, vbTriangulated2, parallax2);
    int nGood3 = CheckRT(R1,t2,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D3, 4.0*mSigma2, vbTriangulated3, parallax3);
    int nGood4 = CheckRT(R2,t2,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D4, 4.0*mSigma2, vbTriangulated4, parallax4);
    //找出数量最大的值
    int maxGood = max(nGood1,max(nGood2,max(nGood3,nGood4)));

    R21 = cv::Mat();
    t21 = cv::Mat();
    //minTriangulated为50，这里取0.9倍所有匹配点对数量和50中较大的值
    int nMinGood = max(static_cast<int>(0.9*N),minTriangulated);
    int nsimilar = 0;
    //比较每种姿态与最好姿态的匹配比较好的点对的数量是否有0.7倍的差距，没有这么明显的差距则nsimilar++
    if(nGood1>0.7*maxGood)
        nsimilar++;
    if(nGood2>0.7*maxGood)
        nsimilar++;
    if(nGood3>0.7*maxGood)
        nsimilar++;
    if(nGood4>0.7*maxGood)
        nsimilar++;
    //如果最好的位姿没有明显比其他位姿更好，或者最好的位姿的好的匹配点不够多，拒绝这个位姿
    if(maxGood<nMinGood || nsimilar>1)
    {
        return false;
    }

    //找出最好的位姿
    if(maxGood==nGood1)
    {
        if(parallax1>minParallax)//minParallax等于1，如果计算的夹角大于1（弧度表示），把对应的3D坐标、位姿、好点作为输出
        {
            vP3D = vP3D1;
            vbTriangulated = vbTriangulated1;

            R1.copyTo(R21);
            t1.copyTo(t21);
            return true;
        }
    }else if(maxGood==nGood2)
    {
        if(parallax2>minParallax)
        {
            vP3D = vP3D2;
            vbTriangulated = vbTriangulated2;

            R2.copyTo(R21);
            t1.copyTo(t21);
            return true;
        }
    }else if(maxGood==nGood3)
    {
        if(parallax3>minParallax)
        {
            vP3D = vP3D3;
            vbTriangulated = vbTriangulated3;

            R1.copyTo(R21);
            t2.copyTo(t21);
            return true;
        }
    }else if(maxGood==nGood4)
    {
        if(parallax4>minParallax)
        {
            vP3D = vP3D4;
            vbTriangulated = vbTriangulated4;

            R2.copyTo(R21);
            t2.copyTo(t21);
            return true;
        }
    }
    return false;
}

bool Initializer::ReconstructH(vector<bool> &vbMatchesInliers, cv::Mat &H21, cv::Mat &K, cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated)
{
    //统计匹配点对中内点数量
    int N=0;
    for(size_t i=0, iend = vbMatchesInliers.size() ; i<iend; i++)
        if(vbMatchesInliers[i])
            N++;
    //计算与H矩阵像素坐标转换相对应的3D点转换的A矩阵
    cv::Mat invK = K.inv();
    cv::Mat A = invK*H21*K;
    //对A矩阵进行SVD分解
    cv::Mat U,w,Vt,V;
    cv::SVD::compute(A,w,U,Vt,cv::SVD::FULL_UV);
    V=Vt.t();

    float s = cv::determinant(U)*cv::determinant(Vt);

    float d1 = w.at<float>(0);
    float d2 = w.at<float>(1);
    float d3 = w.at<float>(2);

    if(d1/d2<1.00001 || d2/d3<1.00001)
    {
        return false;
    }

    vector<cv::Mat> vR, vt, vn;
    vR.reserve(8);
    vt.reserve(8);
    vn.reserve(8);

    //n'=[x1 0 x3] 4 posibilities e1=e3=1, e1=1 e3=-1, e1=-1 e3=1, e1=e3=-1
    float aux1 = sqrt((d1*d1-d2*d2)/(d1*d1-d3*d3));
    float aux3 = sqrt((d2*d2-d3*d3)/(d1*d1-d3*d3));
    float x1[] = {aux1,aux1,-aux1,-aux1};
    float x3[] = {aux3,-aux3,aux3,-aux3};

    //case d'=d2
    float aux_stheta = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1+d3)*d2);

    float ctheta = (d2*d2+d1*d3)/((d1+d3)*d2);
    float stheta[] = {aux_stheta, -aux_stheta, -aux_stheta, aux_stheta};

    for(int i=0; i<4; i++)
    {
        cv::Mat Rp=cv::Mat::eye(3,3,CV_32F);
        Rp.at<float>(0,0)=ctheta;
        Rp.at<float>(0,2)=-stheta[i];
        Rp.at<float>(2,0)=stheta[i];
        Rp.at<float>(2,2)=ctheta;

        cv::Mat R = s*U*Rp*Vt;
        vR.push_back(R);

        cv::Mat tp(3,1,CV_32F);
        tp.at<float>(0)=x1[i];
        tp.at<float>(1)=0;
        tp.at<float>(2)=-x3[i];
        tp*=d1-d3;

        cv::Mat t = U*tp;
        vt.push_back(t/cv::norm(t));

        cv::Mat np(3,1,CV_32F);
        np.at<float>(0)=x1[i];
        np.at<float>(1)=0;
        np.at<float>(2)=x3[i];

        cv::Mat n = V*np;
        if(n.at<float>(2)<0)
            n=-n;
        vn.push_back(n);
    }

    //case d'=-d2
    float aux_sphi = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1-d3)*d2);

    float cphi = (d1*d3-d2*d2)/((d1-d3)*d2);
    float sphi[] = {aux_sphi, -aux_sphi, -aux_sphi, aux_sphi};

    for(int i=0; i<4; i++)
    {
        cv::Mat Rp=cv::Mat::eye(3,3,CV_32F);
        Rp.at<float>(0,0)=cphi;
        Rp.at<float>(0,2)=sphi[i];
        Rp.at<float>(1,1)=-1;
        Rp.at<float>(2,0)=sphi[i];
        Rp.at<float>(2,2)=-cphi;

        cv::Mat R = s*U*Rp*Vt;
        vR.push_back(R);

        cv::Mat tp(3,1,CV_32F);
        tp.at<float>(0)=x1[i];
        tp.at<float>(1)=0;
        tp.at<float>(2)=x3[i];
        tp*=d1+d3;

        cv::Mat t = U*tp;
        vt.push_back(t/cv::norm(t));

        cv::Mat np(3,1,CV_32F);
        np.at<float>(0)=x1[i];
        np.at<float>(1)=0;
        np.at<float>(2)=x3[i];

        cv::Mat n = V*np;
        if(n.at<float>(2)<0)
            n=-n;
        vn.push_back(n);
    }


    int bestGood = 0;
    int secondBestGood = 0;    
    int bestSolutionIdx = -1;
    float bestParallax = -1;
    vector<cv::Point3f> bestP3D;
    vector<bool> bestTriangulated;

    // Instead of applying the visibility constraints proposed in the Faugeras' paper (which could fail for points seen with low parallax)
    // We reconstruct all hypotheses and check in terms of triangulated points and parallax
    for(size_t i=0; i<8; i++)
    {
        float parallaxi;
        vector<cv::Point3f> vP3Di;
        vector<bool> vbTriangulatedi;
        int nGood = CheckRT(vR[i],vt[i],mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K,vP3Di, 4.0*mSigma2, vbTriangulatedi, parallaxi);

        if(nGood>bestGood)
        {
            secondBestGood = bestGood;
            bestGood = nGood;
            bestSolutionIdx = i;
            bestParallax = parallaxi;
            bestP3D = vP3Di;
            bestTriangulated = vbTriangulatedi;
        }
        else if(nGood>secondBestGood)
        {
            secondBestGood = nGood;
        }
    }


    if(secondBestGood<0.75*bestGood && bestParallax>=minParallax && bestGood>minTriangulated && bestGood>0.9*N)
    {
        vR[bestSolutionIdx].copyTo(R21);
        vt[bestSolutionIdx].copyTo(t21);
        vP3D = bestP3D;
        vbTriangulated = bestTriangulated;

        return true;
    }

    return false;
}
// 三角化原理
// Trianularization: 已知匹配特征点对{x x'} 和 各自相机矩阵{P P'}, 估计三维点 X
// x' = P'X  x = PX
// 它们都属于 x = aPX模型
//                         |X|
// |x|     |p1 p2  p3  p4 ||Y|     |x|    |--p0--||.|
// |y| = a |p5 p6  p7  p8 ||Z| ===>|y| = a|--p1--||X|
// |z|     |p9 p10 p11 p12||1|     |z|    |--p2--||.|
// 采用DLT的方法：x叉乘PX = 0
// |yp2 -  p1|     |0|
// |p0 -  xp2| X = |0|
// |xp1 - yp0|     |0|
// 两个点:
// |yp2   -  p1  |     |0|
// |p0    -  xp2 | X = |0| ===> AX = 0
// |y'p2' -  p1' |     |0|
// |p0'   - x'p2'|     |0|
// 变成程序中的形式：
// |xp2  - p0 |     |0|
// |yp2  - p1 | X = |0| ===> AX = 0
// |x'p2'- p0'|     |0|
// |y'p2'- p1'|     |0|
// 然后就组成了一个四元一次正定方程组，SVD求解，右奇异矩阵的最后一行就是最终的解.
void Initializer::Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D)
{
    cv::Mat A(4,4,CV_32F);

    A.row(0) = kp1.pt.x*P1.row(2)-P1.row(0);
    A.row(1) = kp1.pt.y*P1.row(2)-P1.row(1);
    A.row(2) = kp2.pt.x*P2.row(2)-P2.row(0);
    A.row(3) = kp2.pt.y*P2.row(2)-P2.row(1);
    //SVD分解求AX=0的解X，vt的最后一行就是解
    cv::Mat u,w,vt;
    cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
    x3D = vt.row(3).t();
    x3D = x3D.rowRange(0,3)/x3D.at<float>(3);//最后一个元素化为1
}

//对关键点进行中心化和归一化
void Initializer::Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T)
{
    //初始化均值变量(几何中心坐标),归一化后的点向量
    float meanX = 0;
    float meanY = 0;
    const int N = vKeys.size();
    vNormalizedPoints.resize(N);
    //计算所有点的几何中心
    for(int i=0; i<N; i++)
    {
        meanX += vKeys[i].pt.x;
        meanY += vKeys[i].pt.y;
    }
    meanX = meanX/N;
    meanY = meanY/N;
    //向vNormalizedPoints向量中存放中心化后的点坐标，并计算所有点的x.y的绝对值的和meanDevX.meanDevY
    float meanDevX = 0;
    float meanDevY = 0;
    for(int i=0; i<N; i++)
    {
        vNormalizedPoints[i].x = vKeys[i].pt.x - meanX;
        vNormalizedPoints[i].y = vKeys[i].pt.y - meanY;
        meanDevX += fabs(vNormalizedPoints[i].x);
        meanDevY += fabs(vNormalizedPoints[i].y);
    }
    //计算平均的x和y方向的长度meanDevX,meanDevY(这里是相对于几何中心的平均值与meanX和meanY不一样)
    meanDevX = meanDevX/N;
    meanDevY = meanDevY/N;
    //求倒数
    float sX = 1.0/meanDevX;
    float sY = 1.0/meanDevY;
    //对每个相对几何中心的坐标，进行长度归一化(除以meanDevX,meanDevY)
    for(int i=0; i<N; i++)
    {
        vNormalizedPoints[i].x = vNormalizedPoints[i].x * sX;
        vNormalizedPoints[i].y = vNormalizedPoints[i].y * sY;
    }
    // 计算归一化矩阵：其实就是前面做的操作用矩阵变换来表示而已
    // |sX  0  -meanx*sX|
    // |0   sY -meany*sY|
    // |0   0      1    |
    T = cv::Mat::eye(3,3,CV_32F);
    T.at<float>(0,0) = sX;
    T.at<float>(1,1) = sY;
    T.at<float>(0,2) = -meanX*sX;
    T.at<float>(1,2) = -meanY*sY;
}


int Initializer::CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                       const vector<Match> &vMatches12, vector<bool> &vbMatchesInliers,
                       const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax)
{
    // Calibration parameters
    const float fx = K.at<float>(0,0);
    const float fy = K.at<float>(1,1);
    const float cx = K.at<float>(0,2);
    const float cy = K.at<float>(1,2);

    vbGood = vector<bool>(vKeys1.size(),false);//特征点是否是good点的标记，全部初始化为false
    vP3D.resize(vKeys1.size());	//初始化存储空间坐标点的大小

    vector<float> vCosParallax;
    vCosParallax.reserve(vKeys1.size());//存储计算出来的每对特征点的视差

    // 相机1的投影矩阵[K|0](p1)，相机1坐标系表示的3D点投影到相机1的2D像素坐标
    cv::Mat P1(3,4,CV_32F,cv::Scalar(0));
    K.copyTo(P1.rowRange(0,3).colRange(0,3));//把k复制到P1的前三行和前三列，rowRange(0,3)表示从第0行开始的3行

    cv::Mat O1 = cv::Mat::zeros(3,1,CV_32F);//相机1的光心，置为世界坐标系所以为0

    // 相机2的投影矩阵K[R|t](p2)，相机1坐标系表示的3D点投影到相机2的2D像素坐标
    cv::Mat P2(3,4,CV_32F);
    R.copyTo(P2.rowRange(0,3).colRange(0,3));
    t.copyTo(P2.rowRange(0,3).col(3));
    P2 = K*P2;

    cv::Mat O2 = -R.t()*t;//相机2的光心

    int nGood=0;

    for(size_t i=0, iend=vMatches12.size();i<iend;i++)
    {
        //跳过外点
        if(!vbMatchesInliers[i])
            continue;
        //提取2D坐标
        const cv::KeyPoint &kp1 = vKeys1[vMatches12[i].first];
        const cv::KeyPoint &kp2 = vKeys2[vMatches12[i].second];
        cv::Mat p3dC1;
        //三角化，根据空间点在两相机上的像素坐标和两相机间的坐标变换求空间点坐标
        Triangulate(kp1,kp2,P1,P2,p3dC1);
        //用isfinite()函数判断计算出的空间坐标值是否有限（有限值是既不是无限也不是NAN的值），一个不是有限值vbGood就置为false
        if(!isfinite(p3dC1.at<float>(0)) || !isfinite(p3dC1.at<float>(1)) || !isfinite(p3dC1.at<float>(2)))
        {
            vbGood[vMatches12[i].first]=false;
            continue;
        }

        //计算向量的模长
        cv::Mat normal1 = p3dC1 - O1;
        float dist1 = cv::norm(normal1);
        cv::Mat normal2 = p3dC1 - O2;
        float dist2 = cv::norm(normal2);
        //对3D点到两相机光心的向量求点积
        float cosParallax = normal1.dot(normal2)/(dist1*dist2);
        //检测3D点相对相机1的深度值(深度值为负并且夹角余弦小于0.99998，跳过该匹配点)
        if(p3dC1.at<float>(2)<=0 && cosParallax<0.99998)
            continue;
        //检测3D点相对相机2的深度值(深度值为负并且夹角余弦小于0.99998，跳过该匹配点)
        cv::Mat p3dC2 = R*p3dC1+t;
        if(p3dC2.at<float>(2)<=0 && cosParallax<0.99998)
            continue;
        //检测3D点在第1个相机的投影像素坐标
        float im1x, im1y;
        float invZ1 = 1.0/p3dC1.at<float>(2);
        im1x = fx*p3dC1.at<float>(0)*invZ1+cx;
        im1y = fy*p3dC1.at<float>(1)*invZ1+cy;
        //计算投影点与相机1对应特征点的距离的平方(也就是误差)
        float squareError1 = (im1x-kp1.pt.x)*(im1x-kp1.pt.x)+(im1y-kp1.pt.y)*(im1y-kp1.pt.y);
        //误差大于阈值跳过
        if(squareError1>th2)
            continue;
        //检测3D点在第2个相机的投影像素坐标
        float im2x, im2y;
        float invZ2 = 1.0/p3dC2.at<float>(2);
        im2x = fx*p3dC2.at<float>(0)*invZ2+cx;
        im2y = fy*p3dC2.at<float>(1)*invZ2+cy;
        float squareError2 = (im2x-kp2.pt.x)*(im2x-kp2.pt.x)+(im2y-kp2.pt.y)*(im2y-kp2.pt.y);
        if(squareError2>th2)
            continue;

        //如果匹配点通过了上述检测，把向量夹角和3D点进行存储，并且nGood加1
        vCosParallax.push_back(cosParallax);
        vP3D[vMatches12[i].first] = cv::Point3f(p3dC1.at<float>(0),p3dC1.at<float>(1),p3dC1.at<float>(2));
        nGood++;
        //3D点到两相机光心的向量夹角小于0.99998，才会将vbGood对应值置为true
        if(cosParallax<0.99998)
            vbGood[vMatches12[i].first]=true;
    }
    //当好的匹配点对数量大于0进入该分支
    if(nGood>0)
    {
        sort(vCosParallax.begin(),vCosParallax.end());//从小到大排序，注意vCosParallax值越大，视差角度越小
        size_t idx = min(50,int(vCosParallax.size()-1));//比较夹角数量和50的大小，最大取50
        parallax = acos(vCosParallax[idx])*180/CV_PI;//计算相对来说比较小的夹角对应的角度
    }
    else
        parallax=0;
    return nGood;
}

//分解本质矩阵E，得到四组解
void Initializer::DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t)
{
    //本质矩阵分解公式: E = t^R = (U*dialog(1,1,0)*W*UT)*R = U*dialog(1,1,0)*(W*UT*R)
    cv::Mat u,w,vt;
    cv::SVD::compute(E,w,u,vt);
    //分解公式 E = t^R = U*dialog(1,1,0)*(W*UT*R)中U矩阵的z轴即两相机光心位置的连线，所以求t只需要提取u矩阵的第三列(t取反方向的向量也成立，所以也可以取-t)
    u.col(2).copyTo(t); //提取矩阵的某一行或者某一列可以使用函数cv::row(i)或cv::col(i)
    t=t/cv::norm(t); //cv::norm这个函数是求解范数，默认的是L2范数(对t进行归一化)
    //     | 0  -1   0 |          | 0   1   0 |
    // W = | 1   0   0 |  W.t() = |-1   0   0 | 
    //     | 0   0   1 |          | 0   0   1 |
    cv::Mat W(3,3,CV_32F,cv::Scalar(0));
    W.at<float>(0,1)=-1;
    W.at<float>(1,0)=1;
    W.at<float>(2,2)=1;
    //W取上面的W和W.t()都成立，所以这里有两个解R1和R2
    R1 = u*W*vt;
    if(cv::determinant(R1)<0)
        R1=-R1;

    R2 = u*W.t()*vt;
    if(cv::determinant(R2)<0)
        R2=-R2;
}

} //namespace ORB_SLAM
