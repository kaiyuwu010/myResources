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


#include "Sim3Solver.h"

#include <vector>
#include <cmath>
#include <opencv2/core/core.hpp>

#include "KeyFrame.h"
#include "ORBmatcher.h"

#include "Thirdparty/DBoW2/DUtils/Random.h"

namespace ORB_SLAM2
{

//初始化sim3Solver
Sim3Solver::Sim3Solver(KeyFrame *pKF1, KeyFrame *pKF2, const vector<MapPoint *> &vpMatched12, const bool bFixScale):
    mnIterations(0), mnBestInliers(0), mbFixScale(bFixScale)
{
    //记录进行sim3的关键帧
    mpKF1 = pKF1;
    mpKF2 = pKF2;
    //取出关键帧1地图点
    vector<MapPoint*> vpKeyFrameMP1 = pKF1->GetMapPointMatches();
    mN1 = vpMatched12.size();
    //初始化容器
    mvpMapPoints1.reserve(mN1);
    mvpMapPoints2.reserve(mN1);
    mvpMatches12 = vpMatched12;
    mvnIndices1.reserve(mN1);
    mvX3Dc1.reserve(mN1);
    mvX3Dc2.reserve(mN1);
    //获取两关键帧的位姿
    cv::Mat Rcw1 = pKF1->GetRotation();
    cv::Mat tcw1 = pKF1->GetTranslation();
    cv::Mat Rcw2 = pKF2->GetRotation();
    cv::Mat tcw2 = pKF2->GetTranslation();
    mvAllIndices.reserve(mN1);
    size_t idx=0;
    for(int i1=0; i1<mN1; i1++)
    {
        //如果该索引对应的地图点是匹配的
        if(vpMatched12[i1])
        {
            //取出对应的地图点
            MapPoint* pMP1 = vpKeyFrameMP1[i1];
            MapPoint* pMP2 = vpMatched12[i1];
            //地图点为空或者是坏点，跳过
            if(!pMP1)
                continue;
            if(pMP1->isBad() || pMP2->isBad())
                continue;
            //获取地图点在关键帧中的索引
            int indexKF1 = pMP1->GetIndexInKeyFrame(pKF1);
            int indexKF2 = pMP2->GetIndexInKeyFrame(pKF2);
            //没有索引跳过
            if(indexKF1<0 || indexKF2<0)
                continue;
            //获取对应特征点图像坐标
            const cv::KeyPoint &kp1 = pKF1->mvKeysUn[indexKF1];
            const cv::KeyPoint &kp2 = pKF2->mvKeysUn[indexKF2];
            //获取金字塔层级系数的平方值
            const float sigmaSquare1 = pKF1->mvLevelSigma2[kp1.octave];
            const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
            //根据金字塔层级系数，给地图点最大误差赋值
            mvnMaxError1.push_back(9.210*sigmaSquare1);
            mvnMaxError2.push_back(9.210*sigmaSquare2);
            //把地图点存进容器
            mvpMapPoints1.push_back(pMP1);
            mvpMapPoints2.push_back(pMP2);
            mvnIndices1.push_back(i1);
            //计算地图点1相对相机坐标系的位姿 
            cv::Mat X3D1w = pMP1->GetWorldPos();
            mvX3Dc1.push_back(Rcw1*X3D1w+tcw1);
            //计算地图点2相对相机坐标系的位姿 
            cv::Mat X3D2w = pMP2->GetWorldPos();
            mvX3Dc2.push_back(Rcw2*X3D2w+tcw2);
            //添加索引
            mvAllIndices.push_back(idx);
            idx++;
        }
    }
    //获取相机内参
    mK1 = pKF1->mK;
    mK2 = pKF2->mK;
    //把地图点相对相机的坐标转化为图像坐标
    FromCameraToImage(mvX3Dc1, mvP1im1, mK1);
    FromCameraToImage(mvX3Dc2, mvP2im2, mK2);
    //设置ransac参数
    SetRansacParameters();
}

void Sim3Solver::SetRansacParameters(double probability, int minInliers, int maxIterations)
{
    //设置置信度、最少内点数和最大迭代次数
    mRansacProb = probability;
    mRansacMinInliers = minInliers;
    mRansacMaxIts = maxIterations;    
    //初始化变量
    N = mvpMapPoints1.size(); // number of correspondences
    mvbInliersi.resize(N);
    //根据地图点数量和最少内点数，计算系数
    float epsilon = (float)mRansacMinInliers/N;
    //设置RANSAC最大迭代次数，根据相关参数计算得到
    int nIterations;
    if(mRansacMinInliers==N)
        nIterations=1;
    else
        nIterations = ceil(log(1-mRansacProb)/log(1-pow(epsilon, 3)));
    mRansacMaxIts = max(1, min(nIterations, mRansacMaxIts));
    mnIterations = 0;
}

cv::Mat Sim3Solver::iterate(int nIterations, bool &bNoMore, vector<bool> &vbInliers, int &nInliers)
{
    bNoMore = false;
    //初始化向量，记录内点位置
    vbInliers = vector<bool>(mN1,false);
    nInliers=0;
    //最少内点数量大于地图点数量返回空矩阵
    if(N<mRansacMinInliers)
    {
        bNoMore = true;
        return cv::Mat();
    }
    vector<size_t> vAvailableIndices;
    //初始化地图点在相机坐标系的位姿矩阵
    cv::Mat P3Dc1i(3, 3, CV_32F);
    cv::Mat P3Dc2i(3, 3, CV_32F);
    int nCurrentIterations = 0;
    //当前迭代次数小于最大迭代次数，且小于给点的迭代次数，继续迭代
    while(mnIterations<mRansacMaxIts && nCurrentIterations<nIterations)
    {
        nCurrentIterations++;
        mnIterations++;
        vAvailableIndices = mvAllIndices;
        //获取随机点集合
        for(short i = 0; i < 3; ++i)
        {
            //随机选取索引
            int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size()-1);
            //取出随机到的索引
            int idx = vAvailableIndices[randi];
            //取出在相机坐标系的3D位姿
            mvX3Dc1[idx].copyTo(P3Dc1i.col(i));
            mvX3Dc2[idx].copyTo(P3Dc2i.col(i));
            //把该索引从向量中删除
            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }
        //计算sim3
        ComputeSim3(P3Dc1i, P3Dc2i);
        //检测内点
        CheckInliers();
        //比较内点数，记录最多的内点数
        if(mnInliersi>=mnBestInliers)
        {
            mvbBestInliers = mvbInliersi;
            mnBestInliers = mnInliersi;
            //记录对应的变换、旋转、位移和缩放系数
            mBestT12 = mT12i.clone();
            mBestRotation = mR12i.clone();
            mBestTranslation = mt12i.clone();
            mBestScale = ms12i;
            //如果内点数大于阈值，返回最好的变换
            if(mnInliersi>mRansacMinInliers)
            {
                nInliers = mnInliersi;
                for(int i=0; i<N; i++)
                    if(mvbInliersi[i])
                        vbInliers[mvnIndices1[i]] = true;
                return mBestT12;
            }
        }
    }
    //如果迭代次数用完，bNoMore变量置为true
    if(mnIterations>=mRansacMaxIts)
        bNoMore=true;
    return cv::Mat();
}

cv::Mat Sim3Solver::find(vector<bool> &vbInliers12, int &nInliers)
{
    bool bFlag;
    return iterate(mRansacMaxIts,bFlag,vbInliers12,nInliers);
}

void Sim3Solver::ComputeCentroid(cv::Mat &P, cv::Mat &Pr, cv::Mat &C)
{
    //输出P每行的和记录到矩阵C
    cv::reduce(P, C, 1, CV_REDUCE_SUM);
    //计算坐标的平均值
    C = C/P.cols;
    //计算关于平均值的相对值（相对坐标）
    for(int i=0; i<P.cols; i++)
    {
        //每一列代表一个坐标
        Pr.col(i)=P.col(i)-C;
    }
}

//用类似icp的方式计算相对位姿，P1和P2每一列代表一个坐标
void Sim3Solver::ComputeSim3(cv::Mat &P1, cv::Mat &P2)
{
    //中心化相关坐标
    cv::Mat Pr1(P1.size(), P1.type()); //记录关于中心的相对坐标
    cv::Mat Pr2(P2.size(), P2.type()); //记录关于中心的相对坐标
    cv::Mat O1(3, 1, Pr1.type()); //P1的中心点
    cv::Mat O2(3, 1, Pr2.type()); //P2的中心点
    //计算中心点，及相对坐标
    ComputeCentroid(P1, Pr1, O1);
    ComputeCentroid(P2, Pr2, O2);
    //计算M矩阵，中心化后的对应点矩阵
    cv::Mat M = Pr2*Pr1.t();
    //计算N矩阵
    double N11, N12, N13, N14, N22, N23, N24, N33, N34, N44;
    cv::Mat N(4,4,P1.type());
    N11 = M.at<float>(0,0)+M.at<float>(1,1)+M.at<float>(2,2);
    N12 = M.at<float>(1,2)-M.at<float>(2,1);
    N13 = M.at<float>(2,0)-M.at<float>(0,2);
    N14 = M.at<float>(0,1)-M.at<float>(1,0);
    N22 = M.at<float>(0,0)-M.at<float>(1,1)-M.at<float>(2,2);
    N23 = M.at<float>(0,1)+M.at<float>(1,0);
    N24 = M.at<float>(2,0)+M.at<float>(0,2);
    N33 = -M.at<float>(0,0)+M.at<float>(1,1)-M.at<float>(2,2);
    N34 = M.at<float>(1,2)+M.at<float>(2,1);
    N44 = -M.at<float>(0,0)-M.at<float>(1,1)+M.at<float>(2,2);
    N = (cv::Mat_<float>(4,4) << N11, N12, N13, N14,
                                 N12, N22, N23, N24,
                                 N13, N23, N33, N34,
                                 N14, N24, N34, N44);

    //计算N特征值和对应的特征向量
    cv::Mat eval, evec;
    //evec[0]是理想旋转的四元数表示
    cv::eigen(N, eval, evec); 
    //提取四元数表示的旋转向量，N最大的特征值和对应的特征向量
    cv::Mat vec(1, 3, evec.type());
    (evec.row(0).colRange(1, 4)).copyTo(vec); 
    //计算理想旋转的旋转角
    double ang=atan2(norm(vec), evec.at<float>(0, 0));
    //乘上旋转角把旋转转化为轴角表示
    vec = 2*ang*vec/norm(vec); 
    //初始化矩阵存储旋转矩阵
    mR12i.create(3, 3, P1.type());
    //通过罗德里格斯公式计算旋转矩阵
    cv::Rodrigues(vec, mR12i); 
    //把第二个集合中的点进行旋转
    cv::Mat P3 = mR12i*Pr2;
    //计算缩放值
    if(!mbFixScale)
    {
        //求原始点Pr1和转换后的点P3的点积
        double nom = Pr1.dot(P3);
        cv::Mat aux_P3(P3.size(), P3.type());
        aux_P3 = P3;
        //P3每个元素求2次幂，输出到aux_P3
        cv::pow(P3, 2, aux_P3);
        double den = 0;
        //累加aux_P3每个元素的值
        for(int i=0; i<aux_P3.rows; i++)
        {
            for(int j=0; j<aux_P3.cols; j++)
            {
                den+=aux_P3.at<float>(i,j);
            }
        }
        //Pr1的平均长度比上P3的平均长度（缩放系数）
        ms12i = nom/den;
    }
    else
        ms12i = 1.0f;
    //计算平移，旋转中心点，然后进行缩放，最后求差就是平移
    mt12i.create(1, 3, P1.type());
    mt12i = O1 - ms12i*mR12i*O2;
    //计算变换矩阵mT12i
    mT12i = cv::Mat::eye(4,4,P1.type());
    cv::Mat sR = ms12i*mR12i;  //缩放旋转矩阵部分
    sR.copyTo(mT12i.rowRange(0,3).colRange(0,3));
    mt12i.copyTo(mT12i.rowRange(0,3).col(3));  //添加位移部分
    //计算变换矩阵mT12i
    mT21i = cv::Mat::eye(4,4,P1.type());
    cv::Mat sRinv = (1.0/ms12i)*mR12i.t();  //缩放旋转矩阵部分
    sRinv.copyTo(mT21i.rowRange(0,3).colRange(0,3));
    cv::Mat tinv = -sRinv*mt12i;
    tinv.copyTo(mT21i.rowRange(0,3).col(3));  //添加位移部分
}


void Sim3Solver::CheckInliers()
{
    vector<cv::Mat> vP1im2, vP2im1;
    //把3D点根据变换矩阵进行投影，存放到vP1im2和vP2im1
    Project(mvX3Dc2, vP2im1, mT12i, mK1);
    Project(mvX3Dc1, vP1im2, mT21i, mK2);

    mnInliersi=0;
    //遍历图像坐标并与投影得到的坐标计算误差
    for(size_t i=0; i<mvP1im1.size(); i++)
    {
        cv::Mat dist1 = mvP1im1[i]-vP2im1[i];
        cv::Mat dist2 = vP1im2[i]-mvP2im2[i];
        //点积得到误差
        const float err1 = dist1.dot(dist1);
        const float err2 = dist2.dot(dist2);
        //判断误差是否小于阈值，并记录到mvbInliersi
        if(err1<mvnMaxError1[i] && err2<mvnMaxError2[i])
        {
            mvbInliersi[i]=true;
            mnInliersi++;
        }
        else
            mvbInliersi[i]=false;
    }
}


cv::Mat Sim3Solver::GetEstimatedRotation()
{
    return mBestRotation.clone();
}

cv::Mat Sim3Solver::GetEstimatedTranslation()
{
    return mBestTranslation.clone();
}

float Sim3Solver::GetEstimatedScale()
{
    return mBestScale;
}

//3D坐标投影到图像平面
void Sim3Solver::Project(const vector<cv::Mat> &vP3Dw, vector<cv::Mat> &vP2D, cv::Mat Tcw, cv::Mat K)
{
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    const float &fx = K.at<float>(0,0);
    const float &fy = K.at<float>(1,1);
    const float &cx = K.at<float>(0,2);
    const float &cy = K.at<float>(1,2);
    vP2D.clear();
    vP2D.reserve(vP3Dw.size());
    for(size_t i=0, iend=vP3Dw.size(); i<iend; i++)
    {
        cv::Mat P3Dc = Rcw*vP3Dw[i]+tcw;
        const float invz = 1/(P3Dc.at<float>(2));
        const float x = P3Dc.at<float>(0)*invz;
        const float y = P3Dc.at<float>(1)*invz;
        vP2D.push_back((cv::Mat_<float>(2,1) << fx*x+cx, fy*y+cy));
    }
}

//相机坐标系3D坐标转化为图像坐标
void Sim3Solver::FromCameraToImage(const vector<cv::Mat> &vP3Dc, vector<cv::Mat> &vP2D, cv::Mat K)
{
    const float &fx = K.at<float>(0,0);
    const float &fy = K.at<float>(1,1);
    const float &cx = K.at<float>(0,2);
    const float &cy = K.at<float>(1,2);
    vP2D.clear();
    vP2D.reserve(vP3Dc.size());
    for(size_t i=0, iend=vP3Dc.size(); i<iend; i++)
    {
        const float invz = 1/(vP3Dc[i].at<float>(2));
        const float x = vP3Dc[i].at<float>(0)*invz;
        const float y = vP3Dc[i].at<float>(1)*invz;
        vP2D.push_back((cv::Mat_<float>(2,1) << fx*x+cx, fy*y+cy));
    }
}

} //namespace ORB_SLAM
