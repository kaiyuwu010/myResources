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

#include "ORBmatcher.h"

#include<limits.h>

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include<stdint-gcc.h>

using namespace std;

namespace ORB_SLAM2
{

const int ORBmatcher::TH_HIGH = 100;
const int ORBmatcher::TH_LOW = 50;
const int ORBmatcher::HISTO_LENGTH = 30;

ORBmatcher::ORBmatcher(float nnratio, bool checkOri): mfNNratio(nnratio), mbCheckOrientation(checkOri)
{
}

int ORBmatcher::SearchByProjection(Frame &F, const vector<MapPoint*> &vpMapPoints, const float th)
{
    int nmatches=0;

    const bool bFactor = th!=1.0;

    for(size_t iMP=0; iMP<vpMapPoints.size(); iMP++)
    {
        MapPoint* pMP = vpMapPoints[iMP];
        if(!pMP->mbTrackInView)
            continue;

        if(pMP->isBad())
            continue;

        const int &nPredictedLevel = pMP->mnTrackScaleLevel;

        // The size of the window will depend on the viewing direction
        float r = RadiusByViewingCos(pMP->mTrackViewCos);

        if(bFactor)
            r*=th;

        const vector<size_t> vIndices =
                F.GetFeaturesInArea(pMP->mTrackProjX,pMP->mTrackProjY,r*F.mvScaleFactors[nPredictedLevel],nPredictedLevel-1,nPredictedLevel);

        if(vIndices.empty())
            continue;

        const cv::Mat MPdescriptor = pMP->GetDescriptor();

        int bestDist=256;
        int bestLevel= -1;
        int bestDist2=256;
        int bestLevel2 = -1;
        int bestIdx =-1 ;

        // Get best and second matches with near keypoints
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;

            if(F.mvpMapPoints[idx])
                if(F.mvpMapPoints[idx]->Observations()>0)
                    continue;

            if(F.mvuRight[idx]>0)
            {
                const float er = fabs(pMP->mTrackProjXR-F.mvuRight[idx]);
                if(er>r*F.mvScaleFactors[nPredictedLevel])
                    continue;
            }

            const cv::Mat &d = F.mDescriptors.row(idx);

            const int dist = DescriptorDistance(MPdescriptor,d);

            if(dist<bestDist)
            {
                bestDist2=bestDist;
                bestDist=dist;
                bestLevel2 = bestLevel;
                bestLevel = F.mvKeysUn[idx].octave;
                bestIdx=idx;
            }
            else if(dist<bestDist2)
            {
                bestLevel2 = F.mvKeysUn[idx].octave;
                bestDist2=dist;
            }
        }

        // Apply ratio to second match (only if best and second are in the same scale level)
        if(bestDist<=TH_HIGH)
        {
            if(bestLevel==bestLevel2 && bestDist>mfNNratio*bestDist2)
                continue;

            F.mvpMapPoints[bestIdx]=pMP;
            nmatches++;
        }
    }

    return nmatches;
}

float ORBmatcher::RadiusByViewingCos(const float &viewCos)
{
    if(viewCos>0.998)
        return 2.5;
    else
        return 4.0;
}

//检查极线距离
bool ORBmatcher::CheckDistEpipolarLine(const cv::KeyPoint &kp1,const cv::KeyPoint &kp2,const cv::Mat &F12,const KeyFrame* pKF2)
{
    //极线在第二帧相机的3D坐标：l = x1'F12 = [a b c]
    const float a = kp1.pt.x*F12.at<float>(0,0)+kp1.pt.y*F12.at<float>(1,0)+F12.at<float>(2,0);
    const float b = kp1.pt.x*F12.at<float>(0,1)+kp1.pt.y*F12.at<float>(1,1)+F12.at<float>(2,1);
    const float c = kp1.pt.x*F12.at<float>(0,2)+kp1.pt.y*F12.at<float>(1,2)+F12.at<float>(2,2);
    //第二帧记录的该特征点的图像坐标与基础矩阵投影的坐标求点积（求点到直线的距离，a和b是直线的法线方向）
    const float num = a*kp2.pt.x+b*kp2.pt.y+c;
    const float den = a*a+b*b;
    //如果该点在相机光心线上，返回false
    if(den==0)
        return false;
    //求点到直线的距离
    const float dsqr = num*num/den;
    //根据距离和金字塔层级判断
    return dsqr<3.84*pKF2->mvLevelSigma2[kp2.octave];
}

int ORBmatcher::SearchByBoW(KeyFrame* pKF, Frame &F, vector<MapPoint*> &vpMapPointMatches)
{
    //取出关键帧地图点
    const vector<MapPoint*> vpMapPointsKF = pKF->GetMapPointMatches();
    //按普通帧特征点数量初始化匹配地图点向量
    vpMapPointMatches = vector<MapPoint*>(F.N,static_cast<MapPoint*>(NULL));
    //取出关键帧的词袋特征向量
    const DBoW2::FeatureVector &vFeatVecKF = pKF->mFeatVec;
    int nmatches=0;
    //用直方图统计特征点角度旋转差，HISTO_LENGTH表示直方图x方向长度(角度转化到对应x坐标)，rotHist向量存储直方图y方向(不同角度对应的汉明距离,对应y坐标)
    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;
    //把属于同一节点的orb特征进行匹配
    DBoW2::FeatureVector::const_iterator KFit = vFeatVecKF.begin();
    DBoW2::FeatureVector::const_iterator Fit = F.mFeatVec.begin();
    DBoW2::FeatureVector::const_iterator KFend = vFeatVecKF.end();
    DBoW2::FeatureVector::const_iterator Fend = F.mFeatVec.end();
    //遍历关键帧和普通帧的描述子对应节点node
    while(KFit != KFend && Fit != Fend)
    {
        //如果关键帧和普通帧的描述子属于同一node
        if(KFit->first == Fit->first)
        {
            //分别取出属于同一node的ORB特征点
            const vector<unsigned int> vIndicesKF = KFit->second;
            const vector<unsigned int> vIndicesF = Fit->second;
            //遍历关键帧节点对应的描述子索引
            for(size_t iKF=0; iKF<vIndicesKF.size(); iKF++)
            {
                const unsigned int realIdxKF = vIndicesKF[iKF];
                //从关键帧取出对应地图点
                MapPoint* pMP = vpMapPointsKF[realIdxKF];
                //忽略空的和不好的地图点
                if(!pMP)
                    continue;
                if(pMP->isBad())
                    continue;                
                //取出对应描述子
                const cv::Mat &dKF= pKF->mDescriptors.row(realIdxKF);
                //因为描述子有256位，所以计算汉明距离最好就是256
                int bestDist1=256;//记录最佳汉明距离
                int bestIdxF =-1 ;//记录索引
                int bestDist2=256;//记录次佳汉明距离
                //遍历普通帧节点对应的描述子索引
                for(size_t iF=0; iF<vIndicesF.size(); iF++)
                {
                    const unsigned int realIdxF = vIndicesF[iF];
                    if(vpMapPointMatches[realIdxF])
                        continue;
                    //取出对应描述子
                    const cv::Mat &dF = F.mDescriptors.row(realIdxF);
                    //计算描述子间的汉明距离
                    const int dist =  DescriptorDistance(dKF,dF);
                    //记录最佳距离、次佳距离和对应描述子在普通帧中的索引
                    if(dist<bestDist1)
                    {
                        bestDist2=bestDist1;
                        bestDist1=dist;
                        bestIdxF=realIdxF;
                    }
                    else if(dist<bestDist2)
                    {
                        bestDist2=dist;
                    }
                }
                //第一次筛选：最佳匹配距离必须小于设定阈值
                if(bestDist1<=TH_LOW)
                {
                    //第二次筛选：最佳匹配比次佳匹配明显要好，mfNNratio为事先定义的好的程度
                    if(static_cast<float>(bestDist1)<mfNNratio*static_cast<float>(bestDist2))
                    {
                        //记录成功匹配特征点的对应的地图点(关键帧中的)
                        vpMapPointMatches[bestIdxF]=pMP;
                        //记录成功匹配的特征点(关键帧中的)
                        const cv::KeyPoint &kp = pKF->mvKeysUn[realIdxKF];
                        //计算匹配点旋转角度差
                        if(mbCheckOrientation)
                        {
                            //计算特征点描述子旋转角度差
                            float rot = kp.angle-F.mvKeys[bestIdxF].angle;
                            //角度差统计为正方向
                            if(rot<0.0)
                                rot+=360.0f;
                            //按直方图单位角度，计算角度差对应x坐标
                            int bin = round(rot*factor);
                            if(bin==HISTO_LENGTH)
                                bin=0;
                            assert(bin>=0 && bin<HISTO_LENGTH);
                            //直方图统计x坐标对应的各种最佳距离
                            rotHist[bin].push_back(bestIdxF);
                        }
                        //记录匹配点数量
                        nmatches++;
                    }
                }
            }
            KFit++;
            Fit++;
        }
        //对齐关键帧和普通帧迭代器，使迭代器对应节点id相同
        //如果关键帧节点id小于普通帧的节点id
        else if(KFit->first < Fit->first)
        {
            //vFeatVecKF属于std::map类型容器，函数std::map::lower_bound()返回指向第一个不小于key的元素的迭代器k
            KFit = vFeatVecKF.lower_bound(Fit->first);
        }
        //如果关键帧节点id大于普通帧的节点id
        else
        {
            Fit = F.mFeatVec.lower_bound(KFit->first);
        }
    }
    //根据方向剔除误匹配的点
    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;
        //筛选出在旋转角度差落在在直方图区间内数量最多的前三个bin的索引
        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);
        //遍历直方图中的特征点
        for(int i=0; i<HISTO_LENGTH; i++)
        {
            //如果特征点的旋转角度变化量属于这三个组，则保留
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            //剔除掉不在前三的匹配对，因为不符合“主旋转方向”
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                vpMapPointMatches[rotHist[i][j]]=static_cast<MapPoint*>(NULL);
                nmatches--;
            }
        }
    }
    //返回匹配点数量
    return nmatches;
}

int ORBmatcher::SearchByProjection(KeyFrame* pKF, cv::Mat Scw, const vector<MapPoint*> &vpPoints, vector<MapPoint*> &vpMatched, int th)
{
    //获取相机内参
    const float &fx = pKF->fx;
    const float &fy = pKF->fy;
    const float &cx = pKF->cx;
    const float &cy = pKF->cy;
    //分解位姿矩阵Scw
    cv::Mat sRcw = Scw.rowRange(0,3).colRange(0,3);
    const float scw = sqrt(sRcw.row(0).dot(sRcw.row(0)));
    //计算除去缩放系数的旋转矩阵
    cv::Mat Rcw = sRcw/scw;
    //计算除去缩放系数的位移矩阵
    cv::Mat tcw = Scw.rowRange(0,3).col(3)/scw;
    cv::Mat Ow = -Rcw.t()*tcw;
    //取出关键帧中已经发现的地图点
    set<MapPoint*> spAlreadyFound(vpMatched.begin(), vpMatched.end());
    spAlreadyFound.erase(static_cast<MapPoint*>(NULL));//删除空点
    //记录匹配的地图点
    int nmatches=0;
    //对每个选地图点进行投影匹配
    for(int iMP=0, iendMP=vpPoints.size(); iMP<iendMP; iMP++)
    {
        MapPoint* pMP = vpPoints[iMP];
        //抛弃坏点和已经发现的地图点
        if(pMP->isBad() || spAlreadyFound.count(pMP))
            continue;
        //获取3D坐标
        cv::Mat p3Dw = pMP->GetWorldPos();
        //转换到相机坐标系
        cv::Mat p3Dc = Rcw*p3Dw+tcw;
        //深度必须为正
        if(p3Dc.at<float>(2)<0.0)
            continue;
        //投影到图像平面
        const float invz = 1/p3Dc.at<float>(2);
        const float x = p3Dc.at<float>(0)*invz;
        const float y = p3Dc.at<float>(1)*invz;
        const float u = fx*x+cx;
        const float v = fy*y+cy;
        //图像坐标必须在图像内
        if(!pKF->IsInImage(u,v))
            continue;
        //深度值必须满足深度要求
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        cv::Mat PO = p3Dw-Ow;//计算光心到地图点的向量PO
        const float dist = cv::norm(PO);
        if(dist<minDistance || dist>maxDistance)
            continue;
        //视角与平均视角的角度差要小于60度
        cv::Mat Pn = pMP->GetNormal();
        if(PO.dot(Pn)<0.5*dist)
            continue;
        //预测地图点的金字塔层级
        int nPredictedLevel = pMP->PredictScale(dist, pKF);
        //按半径搜索
        const float radius = th*pKF->mvScaleFactors[nPredictedLevel];
        //找到半径范围的特征点
        const vector<size_t> vIndices = pKF->GetFeaturesInArea(u, v, radius);
        //没有搜索到，跳过
        if(vIndices.empty())
            continue;
        //获取描述子，根据描述子距离找到最佳匹配
        const cv::Mat dMP = pMP->GetDescriptor();
        int bestDist = 256;
        int bestIdx = -1;
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;
            //存在地图点跳过
            if(vpMatched[idx])
                continue;
            const int &kpLevel= pKF->mvKeysUn[idx].octave;
            if(kpLevel<nPredictedLevel-1 || kpLevel>nPredictedLevel)
                continue;
            const cv::Mat &dKF = pKF->mDescriptors.row(idx);
            const int dist = DescriptorDistance(dMP,dKF);
            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = idx;
            }
        }
        //小于阈值，就取该点作为匹配点
        if(bestDist<=TH_LOW)
        {
            vpMatched[bestIdx]=pMP;
            nmatches++;
        }
    }
    return nmatches;
}

int ORBmatcher::SearchForInitialization(Frame &F1, Frame &F2, vector<cv::Point2f> &vbPrevMatched, vector<int> &vnMatches12, int windowSize)
{
    int nmatches=0;
    vnMatches12 = vector<int>(F1.mvKeysUn.size(),-1);

    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    vector<int> vMatchedDistance(F2.mvKeysUn.size(),INT_MAX);
    vector<int> vnMatches21(F2.mvKeysUn.size(),-1);

    for(size_t i1=0, iend1=F1.mvKeysUn.size(); i1<iend1; i1++)
    {
        cv::KeyPoint kp1 = F1.mvKeysUn[i1];
        int level1 = kp1.octave;
        if(level1>0)
            continue;

        vector<size_t> vIndices2 = F2.GetFeaturesInArea(vbPrevMatched[i1].x, vbPrevMatched[i1].y, windowSize, level1, level1);

        if(vIndices2.empty())
            continue;

        cv::Mat d1 = F1.mDescriptors.row(i1);

        int bestDist = INT_MAX;
        int bestDist2 = INT_MAX;
        int bestIdx2 = -1;

        for(vector<size_t>::iterator vit=vIndices2.begin(); vit!=vIndices2.end(); vit++)
        {
            size_t i2 = *vit;

            cv::Mat d2 = F2.mDescriptors.row(i2);

            int dist = DescriptorDistance(d1,d2);

            if(vMatchedDistance[i2]<=dist)
                continue;

            if(dist<bestDist)
            {
                bestDist2=bestDist;
                bestDist=dist;
                bestIdx2=i2;
            }
            else if(dist<bestDist2)
            {
                bestDist2=dist;
            }
        }

        if(bestDist<=TH_LOW)
        {
            if(bestDist<(float)bestDist2*mfNNratio)
            {
                if(vnMatches21[bestIdx2]>=0)
                {
                    vnMatches12[vnMatches21[bestIdx2]]=-1;
                    nmatches--;
                }
                vnMatches12[i1]=bestIdx2;
                vnMatches21[bestIdx2]=i1;
                vMatchedDistance[bestIdx2]=bestDist;
                nmatches++;

                if(mbCheckOrientation)
                {
                    float rot = F1.mvKeysUn[i1].angle-F2.mvKeysUn[bestIdx2].angle;
                    if(rot<0.0)
                        rot+=360.0f;
                    int bin = round(rot*factor);
                    if(bin==HISTO_LENGTH)
                        bin=0;
                    assert(bin>=0 && bin<HISTO_LENGTH);
                    rotHist[bin].push_back(i1);
                }
            }
        }

    }

    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                int idx1 = rotHist[i][j];
                if(vnMatches12[idx1]>=0)
                {
                    vnMatches12[idx1]=-1;
                    nmatches--;
                }
            }
        }

    }

    //Update prev matched
    for(size_t i1=0, iend1=vnMatches12.size(); i1<iend1; i1++)
        if(vnMatches12[i1]>=0)
            vbPrevMatched[i1]=F2.mvKeysUn[vnMatches12[i1]].pt;

    return nmatches;
}

int ORBmatcher::SearchByBoW(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches12)
{
    //获取关键帧1的特征点、特征向量、地图点和描述子
    const vector<cv::KeyPoint> &vKeysUn1 = pKF1->mvKeysUn;
    const DBoW2::FeatureVector &vFeatVec1 = pKF1->mFeatVec;
    const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
    const cv::Mat &Descriptors1 = pKF1->mDescriptors;
    //获取关键帧2的特征点、特征向量、地图点和描述子
    const vector<cv::KeyPoint> &vKeysUn2 = pKF2->mvKeysUn;
    const DBoW2::FeatureVector &vFeatVec2 = pKF2->mFeatVec;
    const vector<MapPoint*> vpMapPoints2 = pKF2->GetMapPointMatches();
    const cv::Mat &Descriptors2 = pKF2->mDescriptors;
    //初始化存放匹配地图点的容器，大小为关键帧1的地图点数量
    vpMatches12 = vector<MapPoint*>(vpMapPoints1.size(), static_cast<MapPoint*>(NULL));
    vector<bool> vbMatched2(vpMapPoints2.size(), false);
    //初始化描述子旋转差直方图向量
    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;
    //初始化匹配点数量变量
    int nmatches = 0;
    //初始化两关键帧的特征向量迭代器
    DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
    DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
    DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
    DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();
    while(f1it != f1end && f2it != f2end)
    {
        if(f1it->first == f2it->first)
        {
            //遍历关键帧1相同节点对应的索引
            for(size_t i1=0, iend1=f1it->second.size(); i1<iend1; i1++)
            {
                const size_t idx1 = f1it->second[i1];
                //判断该特征点在关键帧1中是否存在地图点，是否是坏点
                MapPoint* pMP1 = vpMapPoints1[idx1];
                if(!pMP1)
                    continue;
                if(pMP1->isBad())
                    continue;
                //取出描述子
                const cv::Mat &d1 = Descriptors1.row(idx1);
                int bestDist1=256;
                int bestIdx2 =-1 ;
                int bestDist2=256;
                //遍历关键帧2相同节点对应的索引
                for(size_t i2=0, iend2=f2it->second.size(); i2<iend2; i2++)
                {
                    const size_t idx2 = f2it->second[i2];
                    MapPoint* pMP2 = vpMapPoints2[idx2];
                    //判断该特征点是否已经匹配了，在关键帧2中是否存在地图点，是否是坏点
                    if(vbMatched2[idx2] || !pMP2)
                        continue;
                    if(pMP2->isBad())
                        continue;
                    //取出描述子
                    const cv::Mat &d2 = Descriptors2.row(idx2);
                    //计算描述子距离
                    int dist = DescriptorDistance(d1,d2);
                    //记录最佳距离、次佳距离和对应索引
                    if(dist<bestDist1)
                    {
                        bestDist2 = bestDist1;
                        bestDist1 = dist;
                        bestIdx2 = idx2;
                    }
                    else if(dist<bestDist2)
                    {
                        bestDist2=dist;
                    }
                }
                //选出得分小于阈值的地图点，存入vpMatches12
                if(bestDist1<TH_LOW)
                {
                    if(static_cast<float>(bestDist1)<mfNNratio*static_cast<float>(bestDist2))
                    {
                        vpMatches12[idx1] = vpMapPoints2[bestIdx2];
                        vbMatched2[bestIdx2]=true;
                        //计算旋转角度差
                        if(mbCheckOrientation)
                        {
                            float rot = vKeysUn1[idx1].angle-vKeysUn2[bestIdx2].angle;
                            if(rot<0.0)
                                rot+=360.0f;
                            int bin = round(rot*factor);
                            if(bin==HISTO_LENGTH)
                                bin=0;
                            assert(bin>=0 && bin<HISTO_LENGTH);
                            rotHist[bin].push_back(idx1);
                        }
                        nmatches++;
                    }
                }
            }
            f1it++;
            f2it++;
        }
        else if(f1it->first < f2it->first)
        {
            f1it = vFeatVec1.lower_bound(f2it->first);
        }
        else
        {
            f2it = vFeatVec2.lower_bound(f1it->first);
        }
    }
    //对旋转角度差进行筛选
    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;
        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);
        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                vpMatches12[rotHist[i][j]]=static_cast<MapPoint*>(NULL);
                nmatches--;
            }
        }
    }
    return nmatches;
}

//相比其他投影匹配函数，本函数多了通过基本矩阵进行坐标转换再进行特征点到极线的距离检测的步骤
int ORBmatcher::SearchForTriangulation(KeyFrame *pKF1, KeyFrame *pKF2, cv::Mat F12, vector<pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo)
{    
    const DBoW2::FeatureVector &vFeatVec1 = pKF1->mFeatVec;
    const DBoW2::FeatureVector &vFeatVec2 = pKF2->mFeatVec;
    //计算第一帧照相机光心在第二帧相机坐标系的位置C2
    cv::Mat Cw = pKF1->GetCameraCenter();
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();
    cv::Mat C2 = R2w*Cw+t2w;
    const float invz = 1.0f/C2.at<float>(2);
    //计算第一帧相机光心在第二帧上的图像坐标
    const float ex =pKF2->fx*C2.at<float>(0)*invz+pKF2->cx;
    const float ey =pKF2->fy*C2.at<float>(1)*invz+pKF2->cy;
    //声明变量记录匹配点数量
    int nmatches=0;
    vector<bool> vbMatched2(pKF2->N,false);
    vector<int> vMatches12(pKF1->N,-1);
    //声明向量存放旋转差直方图数据
    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;
    //生成迭代器，遍历第一和第二帧的特征的词袋向量
    DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
    DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
    DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
    DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();
    while(f1it!=f1end && f2it!=f2end)
    {
        //找出第一帧和第二帧相同的节点（单词所属节点）
        if(f1it->first == f2it->first)
        {
            //对节点下属的单词进行遍历
            for(size_t i1=0, iend1=f1it->second.size(); i1<iend1; i1++)
            {
                const size_t idx1 = f1it->second[i1];
                //获取对应的地图点
                MapPoint* pMP1 = pKF1->GetMapPoint(idx1);
                //已经有地图点跳过
                if(pMP1)
                    continue;
                //如果对应的右目坐标存在，则把标志位置为true
                const bool bStereo1 = pKF1->mvuRight[idx1]>=0;
                //如果只考虑双目点，右目不存在则跳过
                if(bOnlyStereo)
                    if(!bStereo1)
                        continue;
                //取出对应的关键点
                const cv::KeyPoint &kp1 = pKF1->mvKeysUn[idx1];
                //取出描述子
                const cv::Mat &d1 = pKF1->mDescriptors.row(idx1);
                
                int bestDist = TH_LOW;
                int bestIdx2 = -1;
                //遍历第二帧该节点的下属单词
                for(size_t i2=0, iend2=f2it->second.size(); i2<iend2; i2++)
                {
                    size_t idx2 = f2it->second[i2];
                    MapPoint* pMP2 = pKF2->GetMapPoint(idx2);                    
                    //如果该点匹配过了或者是地图点跳过
                    if(vbMatched2[idx2] || pMP2)
                        continue;
                    //如果对应的右目坐标存在，则把标志位置为true
                    const bool bStereo2 = pKF2->mvuRight[idx2]>=0;
                    //如果只考虑双目点，右目不存在则跳过
                    if(bOnlyStereo)
                        if(!bStereo2)
                            continue;
                    //取出描述子
                    const cv::Mat &d2 = pKF2->mDescriptors.row(idx2);
                    //计算两帧对应描述子的距离
                    const int dist = DescriptorDistance(d1,d2);
                    //如果距离大于阈值或者大于最好的值，跳过
                    if(dist>TH_LOW || dist>bestDist)
                        continue;
                    //取出图像坐标
                    const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];
                    //如果第一帧和第二帧都不是双目图像
                    if(!bStereo1 && !bStereo2)
                    {
                        //计算第一帧光心在第二帧上的图像坐标与特征点图像坐标的差
                        const float distex = ex - kp2.pt.x;
                        const float distey = ey - kp2.pt.y;
                        //如果两点的图像像素距离的平方小于金字塔层级乘100，跳过
                        if(distex*distex+distey*distey<100*pKF2->mvScaleFactors[kp2.octave])
                            continue;
                    }
                    //检查第二帧特征点到极线的距离
                    if(CheckDistEpipolarLine(kp1, kp2, F12, pKF2))
                    {
                        //重置最短距离和对应特征点索引
                        bestIdx2 = idx2;
                        bestDist = dist;
                    }
                }
                //把匹配信息存到vMatches12向量
                if(bestIdx2>=0)
                {
                    const cv::KeyPoint &kp2 = pKF2->mvKeysUn[bestIdx2];
                    vMatches12[idx1]=bestIdx2;
                    nmatches++;
                    //计算旋转角度差，并把数据存放到rotHist
                    if(mbCheckOrientation)
                    {
                        float rot = kp1.angle-kp2.angle;
                        if(rot<0.0)
                            rot+=360.0f;
                        int bin = round(rot*factor);
                        if(bin==HISTO_LENGTH)
                            bin=0;
                        assert(bin>=0 && bin<HISTO_LENGTH);
                        rotHist[bin].push_back(idx1);
                    }
                }
            }
            f1it++;
            f2it++;
        }
        //把迭代器进行对齐
        else if(f1it->first < f2it->first)
        {
            f1it = vFeatVec1.lower_bound(f2it->first);
        }
        else
        {
            f2it = vFeatVec2.lower_bound(f1it->first);
        }
    }
    //检测角度旋转差，取出数量最多的旋转差对应的点
    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;
        ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);
        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                vMatches12[rotHist[i][j]]=-1;
                nmatches--;
            }
        }
    }
    vMatchedPairs.clear();
    vMatchedPairs.reserve(nmatches);
    //存放两帧匹配的点对应的索引对
    for(size_t i=0, iend=vMatches12.size(); i<iend; i++)
    {
        if(vMatches12[i]<0)
            continue;
        vMatchedPairs.push_back(make_pair(i, vMatches12[i]));
    }
    //返回匹配点的点数量
    return nmatches;
}

int ORBmatcher::Fuse(KeyFrame *pKF, const vector<MapPoint *> &vpMapPoints, const float th)
{
    //取出当前帧位姿、内参、光心在世界坐标系下坐标
    cv::Mat Rcw = pKF->GetRotation();
    cv::Mat tcw = pKF->GetTranslation();
    const float &fx = pKF->fx;
    const float &fy = pKF->fy;
    const float &cx = pKF->cx;
    const float &cy = pKF->cy;
    const float &bf = pKF->mbf;
    cv::Mat Ow = pKF->GetCameraCenter();
    //声明变量记录融合的点数
    int nFused=0;
    const int nMPs = vpMapPoints.size();
    //遍历所有的待投影地图点
    for(int i=0; i<nMPs; i++)
    {
        //取出地图点
        MapPoint* pMP = vpMapPoints[i];
        if(!pMP)
            continue;
        //判断地图点是否是坏点，或者是否在关键帧中
        if(pMP->isBad() || pMP->IsInKeyFrame(pKF))
            continue;
        //取出地图点相对世界坐标系的3D坐标，并计算在关键帧相机坐标系中的3D坐标
        cv::Mat p3Dw = pMP->GetWorldPos();
        cv::Mat p3Dc = Rcw*p3Dw + tcw;
        //判断深度是否为正
        if(p3Dc.at<float>(2)<0.0f)
            continue;
        //将地图点投影到关键帧的图像坐标
        const float invz = 1/p3Dc.at<float>(2);
        const float x = p3Dc.at<float>(0)*invz;
        const float y = p3Dc.at<float>(1)*invz;
        const float u = fx*x+cx;
        const float v = fy*y+cy;
        //判断该地图点是否在关键帧图像范围
        if(!pKF->IsInImage(u,v))
            continue;
        //计算右目对应的u值
        const float ur = u-bf*invz;
        //取出能观测到该地图点的深度范围
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        //计算光心到地图点的向量，并求出距离
        cv::Mat PO = p3Dw-Ow;
        const float dist3D = cv::norm(PO);
        //判断距离是否在地图点有效观测范围内
        if(dist3D<minDistance || dist3D>maxDistance )
            continue;
        //地图点到光心的连线与该地图点的平均观测方向向量之间夹角要小于60°
        cv::Mat Pn = pMP->GetNormal();
        if(PO.dot(Pn)<0.5*dist3D)
            continue;
        //根据地图点到相机光心距离预测关键帧中的匹配点所在的金字塔层数
        int nPredictedLevel = pMP->PredictScale(dist3D, pKF);
        //根据金字塔层级确定搜索半径
        const float radius = th*pKF->mvScaleFactors[nPredictedLevel];
        //在关键在搜索该区域的所有特征点
        const vector<size_t> vIndices = pKF->GetFeaturesInArea(u, v, radius);
        if(vIndices.empty())
            continue;
        //遍历寻找最佳匹配点
        const cv::Mat dMP = pMP->GetDescriptor();
        int bestDist = 256;
        int bestIdx = -1;
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            //取出特征点
            const size_t idx = *vit;
            const cv::KeyPoint &kp = pKF->mvKeysUn[idx];
            //取出金字塔层级
            const int &kpLevel= kp.octave;
            //金字塔层级要接近（同一层或小一层）
            if(kpLevel<nPredictedLevel-1 || kpLevel>nPredictedLevel)
                continue;
            //双目时，计算投影点与候选匹配特征点的距离，如果偏差很大，直接跳过
            if(pKF->mvuRight[idx]>=0)
            {
                //取出特征点图像坐标
                const float &kpx = kp.pt.x;
                const float &kpy = kp.pt.y;
                const float &kpr = pKF->mvuRight[idx];
                //计算误差
                const float ex = u-kpx;
                const float ey = v-kpy;
                const float er = ur-kpr;
                const float e2 = ex*ex+ey*ey+er*er;
                //自由度为3, 误差小于1个像素,这种事情95%发生的概率对应卡方检验阈值为7.82
                if(e2*pKF->mvInvLevelSigma2[kpLevel]>7.8)
                    continue;
            }
            //单目情况
            else
            {
                const float &kpx = kp.pt.x;
                const float &kpy = kp.pt.y;
                const float ex = u-kpx;
                const float ey = v-kpy;
                const float e2 = ex*ex+ey*ey;
                //自由度为2的，卡方检验阈值5.99（假设测量有一个像素的偏差）
                if(e2*pKF->mvInvLevelSigma2[kpLevel]>5.99)
                    continue;
            }

            const cv::Mat &dKF = pKF->mDescriptors.row(idx);
            const int dist = DescriptorDistance(dMP, dKF);
            //记录最小描述子距离和对应索引
            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = idx;
            }
        }
        //如果描述子最小距离小于阈值
        if(bestDist<=TH_LOW)
        {
            //取出最佳匹配特征点对应的地图点
            MapPoint* pMPinKF = pKF->GetMapPoint(bestIdx);
            //如果对应地图点已经存在
            if(pMPinKF)
            {
                //如果该地图点非坏点
                if(!pMPinKF->isBad())
                {
                    //把地图点观测小的替换为观测大的地图点
                    if(pMPinKF->Observations()>pMP->Observations())
                        pMP->Replace(pMPinKF);
                    else
                        pMPinKF->Replace(pMP);
                }
            }
            //当前帧对应地图点不存在，地图点添加对当前关键帧的观测，并且给当前帧添加地图点
            else
            {
                pMP->AddObservation(pKF, bestIdx);
                pKF->AddMapPoint(pMP, bestIdx);
            }
            nFused++;
        }
    }
    //返回融合的点数量
    return nFused;
}

int ORBmatcher::Fuse(KeyFrame *pKF, cv::Mat Scw, const vector<MapPoint *> &vpPoints, float th, vector<MapPoint *> &vpReplacePoint)
{
    // Get Calibration Parameters for later projection
    const float &fx = pKF->fx;
    const float &fy = pKF->fy;
    const float &cx = pKF->cx;
    const float &cy = pKF->cy;

    // Decompose Scw
    cv::Mat sRcw = Scw.rowRange(0,3).colRange(0,3);
    const float scw = sqrt(sRcw.row(0).dot(sRcw.row(0)));
    cv::Mat Rcw = sRcw/scw;
    cv::Mat tcw = Scw.rowRange(0,3).col(3)/scw;
    cv::Mat Ow = -Rcw.t()*tcw;

    // Set of MapPoints already found in the KeyFrame
    const set<MapPoint*> spAlreadyFound = pKF->GetMapPoints();

    int nFused=0;

    const int nPoints = vpPoints.size();

    // For each candidate MapPoint project and match
    for(int iMP=0; iMP<nPoints; iMP++)
    {
        MapPoint* pMP = vpPoints[iMP];

        // Discard Bad MapPoints and already found
        if(pMP->isBad() || spAlreadyFound.count(pMP))
            continue;

        // Get 3D Coords.
        cv::Mat p3Dw = pMP->GetWorldPos();

        // Transform into Camera Coords.
        cv::Mat p3Dc = Rcw*p3Dw+tcw;

        // Depth must be positive
        if(p3Dc.at<float>(2)<0.0f)
            continue;

        // Project into Image
        const float invz = 1.0/p3Dc.at<float>(2);
        const float x = p3Dc.at<float>(0)*invz;
        const float y = p3Dc.at<float>(1)*invz;

        const float u = fx*x+cx;
        const float v = fy*y+cy;

        // Point must be inside the image
        if(!pKF->IsInImage(u,v))
            continue;

        // Depth must be inside the scale pyramid of the image
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        cv::Mat PO = p3Dw-Ow;
        const float dist3D = cv::norm(PO);

        if(dist3D<minDistance || dist3D>maxDistance)
            continue;

        // Viewing angle must be less than 60 deg
        cv::Mat Pn = pMP->GetNormal();

        if(PO.dot(Pn)<0.5*dist3D)
            continue;

        // Compute predicted scale level
        const int nPredictedLevel = pMP->PredictScale(dist3D,pKF);

        // Search in a radius
        const float radius = th*pKF->mvScaleFactors[nPredictedLevel];

        const vector<size_t> vIndices = pKF->GetFeaturesInArea(u,v,radius);

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius

        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = INT_MAX;
        int bestIdx = -1;
        for(vector<size_t>::const_iterator vit=vIndices.begin(); vit!=vIndices.end(); vit++)
        {
            const size_t idx = *vit;
            const int &kpLevel = pKF->mvKeysUn[idx].octave;

            if(kpLevel<nPredictedLevel-1 || kpLevel>nPredictedLevel)
                continue;

            const cv::Mat &dKF = pKF->mDescriptors.row(idx);

            int dist = DescriptorDistance(dMP,dKF);

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        // If there is already a MapPoint replace otherwise add new measurement
        if(bestDist<=TH_LOW)
        {
            MapPoint* pMPinKF = pKF->GetMapPoint(bestIdx);
            if(pMPinKF)
            {
                if(!pMPinKF->isBad())
                    vpReplacePoint[iMP] = pMPinKF;
            }
            else
            {
                pMP->AddObservation(pKF,bestIdx);
                pKF->AddMapPoint(pMP,bestIdx);
            }
            nFused++;
        }
    }

    return nFused;
}

int ORBmatcher::SearchBySim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint*> &vpMatches12,
                             const float &s12, const cv::Mat &R12, const cv::Mat &t12, const float th)
{
    const float &fx = pKF1->fx;
    const float &fy = pKF1->fy;
    const float &cx = pKF1->cx;
    const float &cy = pKF1->cy;

    // Camera 1 from world
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();

    //Camera 2 from world
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    //Transformation between cameras
    cv::Mat sR12 = s12*R12;
    cv::Mat sR21 = (1.0/s12)*R12.t();
    cv::Mat t21 = -sR21*t12;

    const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
    const int N1 = vpMapPoints1.size();

    const vector<MapPoint*> vpMapPoints2 = pKF2->GetMapPointMatches();
    const int N2 = vpMapPoints2.size();

    vector<bool> vbAlreadyMatched1(N1,false);
    vector<bool> vbAlreadyMatched2(N2,false);

    for(int i=0; i<N1; i++)
    {
        MapPoint* pMP = vpMatches12[i];
        if(pMP)
        {
            vbAlreadyMatched1[i]=true;
            int idx2 = pMP->GetIndexInKeyFrame(pKF2);
            if(idx2>=0 && idx2<N2)
                vbAlreadyMatched2[idx2]=true;
        }
    }

    vector<int> vnMatch1(N1,-1);
    vector<int> vnMatch2(N2,-1);

    // Transform from KF1 to KF2 and search
    for(int i1=0; i1<N1; i1++)
    {
        MapPoint* pMP = vpMapPoints1[i1];

        if(!pMP || vbAlreadyMatched1[i1])
            continue;

        if(pMP->isBad())
            continue;

        cv::Mat p3Dw = pMP->GetWorldPos();
        cv::Mat p3Dc1 = R1w*p3Dw + t1w;
        cv::Mat p3Dc2 = sR21*p3Dc1 + t21;

        // Depth must be positive
        if(p3Dc2.at<float>(2)<0.0)
            continue;

        const float invz = 1.0/p3Dc2.at<float>(2);
        const float x = p3Dc2.at<float>(0)*invz;
        const float y = p3Dc2.at<float>(1)*invz;

        const float u = fx*x+cx;
        const float v = fy*y+cy;

        // Point must be inside the image
        if(!pKF2->IsInImage(u,v))
            continue;

        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        const float dist3D = cv::norm(p3Dc2);

        // Depth must be inside the scale invariance region
        if(dist3D<minDistance || dist3D>maxDistance )
            continue;

        // Compute predicted octave
        const int nPredictedLevel = pMP->PredictScale(dist3D,pKF2);

        // Search in a radius
        const float radius = th*pKF2->mvScaleFactors[nPredictedLevel];

        const vector<size_t> vIndices = pKF2->GetFeaturesInArea(u,v,radius);

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = INT_MAX;
        int bestIdx = -1;
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;

            const cv::KeyPoint &kp = pKF2->mvKeysUn[idx];

            if(kp.octave<nPredictedLevel-1 || kp.octave>nPredictedLevel)
                continue;

            const cv::Mat &dKF = pKF2->mDescriptors.row(idx);

            const int dist = DescriptorDistance(dMP,dKF);

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        if(bestDist<=TH_HIGH)
        {
            vnMatch1[i1]=bestIdx;
        }
    }

    // Transform from KF2 to KF2 and search
    for(int i2=0; i2<N2; i2++)
    {
        MapPoint* pMP = vpMapPoints2[i2];

        if(!pMP || vbAlreadyMatched2[i2])
            continue;

        if(pMP->isBad())
            continue;

        cv::Mat p3Dw = pMP->GetWorldPos();
        cv::Mat p3Dc2 = R2w*p3Dw + t2w;
        cv::Mat p3Dc1 = sR12*p3Dc2 + t12;

        // Depth must be positive
        if(p3Dc1.at<float>(2)<0.0)
            continue;

        const float invz = 1.0/p3Dc1.at<float>(2);
        const float x = p3Dc1.at<float>(0)*invz;
        const float y = p3Dc1.at<float>(1)*invz;

        const float u = fx*x+cx;
        const float v = fy*y+cy;

        // Point must be inside the image
        if(!pKF1->IsInImage(u,v))
            continue;

        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        const float dist3D = cv::norm(p3Dc1);

        // Depth must be inside the scale pyramid of the image
        if(dist3D<minDistance || dist3D>maxDistance)
            continue;

        // Compute predicted octave
        const int nPredictedLevel = pMP->PredictScale(dist3D,pKF1);

        // Search in a radius of 2.5*sigma(ScaleLevel)
        const float radius = th*pKF1->mvScaleFactors[nPredictedLevel];

        const vector<size_t> vIndices = pKF1->GetFeaturesInArea(u,v,radius);

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = INT_MAX;
        int bestIdx = -1;
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;

            const cv::KeyPoint &kp = pKF1->mvKeysUn[idx];

            if(kp.octave<nPredictedLevel-1 || kp.octave>nPredictedLevel)
                continue;

            const cv::Mat &dKF = pKF1->mDescriptors.row(idx);

            const int dist = DescriptorDistance(dMP,dKF);

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        if(bestDist<=TH_HIGH)
        {
            vnMatch2[i2]=bestIdx;
        }
    }

    // Check agreement
    int nFound = 0;

    for(int i1=0; i1<N1; i1++)
    {
        int idx2 = vnMatch1[i1];

        if(idx2>=0)
        {
            int idx1 = vnMatch2[idx2];
            if(idx1==i1)
            {
                vpMatches12[i1] = vpMapPoints2[idx2];
                nFound++;
            }
        }
    }

    return nFound;
}

//通过投影地图点到当前帧，找到匹配点
int ORBmatcher::SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono)
{
    int nmatches = 0;
    //初始化描述子旋转角度差直方图
    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;
    //取出当前帧相对世界坐标系的旋转和位移部分
    const cv::Mat Rcw = CurrentFrame.mTcw.rowRange(0,3).colRange(0,3);
    const cv::Mat tcw = CurrentFrame.mTcw.rowRange(0,3).col(3);
    //
    const cv::Mat twc = -Rcw.t()*tcw;
    //取出上一帧相对世界坐标系的旋转和位移部分
    const cv::Mat Rlw = LastFrame.mTcw.rowRange(0,3).colRange(0,3);
    const cv::Mat tlw = LastFrame.mTcw.rowRange(0,3).col(3);

    const cv::Mat tlc = Rlw*twc+tlw;
    //判断前进还是后退，并以此预测特征点在当前帧所在的金字塔层数
    const bool bForward = tlc.at<float>(2)>CurrentFrame.mb && !bMono;//System::MONOCULAR=0, bMono=0
    const bool bBackward = -tlc.at<float>(2)>CurrentFrame.mb && !bMono;

    for(int i=0; i<LastFrame.N; i++)
    {
        //取出地图点
        MapPoint* pMP = LastFrame.mvpMapPoints[i];
        if(pMP)
        {
            if(!LastFrame.mvbOutlier[i])
            {
                //上一帧地图点由世界坐标系转换到当前帧相机坐标系
                cv::Mat x3Dw = pMP->GetWorldPos();
                cv::Mat x3Dc = Rcw*x3Dw+tcw;
                //提取坐标
                const float xc = x3Dc.at<float>(0);
                const float yc = x3Dc.at<float>(1);
                const float invzc = 1.0/x3Dc.at<float>(2);
                //深度值为负跳过
                if(invzc<0)
                    continue;
                //计算成像平面u、v值
                float u = CurrentFrame.fx*xc*invzc+CurrentFrame.cx;
                float v = CurrentFrame.fy*yc*invzc+CurrentFrame.cy;
                //检查坐标是否在当前帧的边界范围内
                if(u<CurrentFrame.mnMinX || u>CurrentFrame.mnMaxX)
                    continue;
                if(v<CurrentFrame.mnMinY || v>CurrentFrame.mnMaxY)
                    continue;
                //取出上一帧的特征点层级
                int nLastOctave = LastFrame.mvKeys[i].octave;
                //根据上一帧的特征点层级得到金字塔缩放系数，计算搜索半径，层级越高搜索半径越大
                float radius = th*CurrentFrame.mvScaleFactors[nLastOctave];
                //声明存放索引的向量
                vector<size_t> vIndices2;
                //搜索匹配点
                if(bForward)
                    vIndices2 = CurrentFrame.GetFeaturesInArea(u, v, radius, nLastOctave);
                else if(bBackward)
                    vIndices2 = CurrentFrame.GetFeaturesInArea(u, v, radius, 0, nLastOctave);
                else
                    vIndices2 = CurrentFrame.GetFeaturesInArea(u, v, radius, nLastOctave-1, nLastOctave+1);
                //如果没有搜索到可能匹配的特征点集合，跳过
                if(vIndices2.empty())
                    continue;
                //取出描述子
                const cv::Mat dMP = pMP->GetDescriptor();
                int bestDist = 256;
                int bestIdx2 = -1;
                //遍历找到的潜在匹配点集合，计算描述子汉明距离
                for(vector<size_t>::const_iterator vit=vIndices2.begin(), vend=vIndices2.end(); vit!=vend; vit++)
                {
                    const size_t i2 = *vit;
                    //判断当前帧地图点观测是否添加过观测，添加过就跳过
                    if(CurrentFrame.mvpMapPoints[i2])
                        if(CurrentFrame.mvpMapPoints[i2]->Observations()>0)
                            continue;
                    //如果当前帧对应索引右目特征点也有值
                    if(CurrentFrame.mvuRight[i2]>0)
                    {
                        const float ur = u - CurrentFrame.mbf*invzc;
                        const float er = fabs(ur - CurrentFrame.mvuRight[i2]);
                        if(er>radius)
                            continue;
                    }
                    //找到当前帧对应描述子
                    const cv::Mat &d = CurrentFrame.mDescriptors.row(i2);
                    //计算汉明距离
                    const int dist = DescriptorDistance(dMP,d);
                    //找到最好的结果，并记录其索引
                    if(dist<bestDist)
                    {
                        bestDist=dist;
                        bestIdx2=i2;
                    }
                }
                //如果匹配最好的特征点对应距离小于阈值检测描述子的旋转差，计算旋转差直方图信息
                if(bestDist<=TH_HIGH)
                {
                    //把地图点指针赋值到匹配最好的特征点索引处
                    CurrentFrame.mvpMapPoints[bestIdx2]=pMP;
                    nmatches++;
                    if(mbCheckOrientation)
                    {
                        float rot = LastFrame.mvKeysUn[i].angle-CurrentFrame.mvKeysUn[bestIdx2].angle;
                        if(rot<0.0)
                            rot+=360.0f;
                        int bin = round(rot*factor);
                        if(bin==HISTO_LENGTH)
                            bin=0;
                        assert(bin>=0 && bin<HISTO_LENGTH);
                        rotHist[bin].push_back(bestIdx2);
                    }
                }
            }
        }
    }
    //通过旋转差直方图，把旋转差不属于前三的地图点置为空
    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;
        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);
        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i!=ind1 && i!=ind2 && i!=ind3)
            {
                for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                {
                    CurrentFrame.mvpMapPoints[rotHist[i][j]]=static_cast<MapPoint*>(NULL);
                    nmatches--;
                }
            }
        }
    }

    return nmatches;
}


int ORBmatcher::SearchByProjection(Frame &CurrentFrame, KeyFrame *pKF, const set<MapPoint*> &sAlreadyFound, const float th , const int ORBdist)
{
    int nmatches = 0;
    //取出当前帧位姿，计算相机光心到世界坐标系原点的向量
    const cv::Mat Rcw = CurrentFrame.mTcw.rowRange(0,3).colRange(0,3);
    const cv::Mat tcw = CurrentFrame.mTcw.rowRange(0,3).col(3);
    const cv::Mat Ow = -Rcw.t()*tcw;
    //初始化旋转差直方图变量
    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;
    //获取关键帧地图点
    const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();
    //遍历地图点
    for(size_t i=0, iend=vpMPs.size(); i<iend; i++)
    {
        MapPoint* pMP = vpMPs[i];
        if(pMP)
        {
            //判断地图点是否是坏点或者是否已经被匹配了
            if(!pMP->isBad() && !sAlreadyFound.count(pMP))
            {
                //地图点重投影到当前帧坐标系
                cv::Mat x3Dw = pMP->GetWorldPos();
                cv::Mat x3Dc = Rcw*x3Dw+tcw;
                //计算图像坐标
                const float xc = x3Dc.at<float>(0);
                const float yc = x3Dc.at<float>(1);
                const float invzc = 1.0/x3Dc.at<float>(2);
                const float u = CurrentFrame.fx*xc*invzc+CurrentFrame.cx;
                const float v = CurrentFrame.fy*yc*invzc+CurrentFrame.cy;
                //判断图坐标是否在图像有效范围
                if(u<CurrentFrame.mnMinX || u>CurrentFrame.mnMaxX)
                    continue;
                if(v<CurrentFrame.mnMinY || v>CurrentFrame.mnMaxY)
                    continue;
                //计算地图点到当前帧相机距离
                cv::Mat PO = x3Dw-Ow;
                float dist3D = cv::norm(PO);
                //获取该地图点的可观测深度范围
                const float maxDistance = pMP->GetMaxDistanceInvariance();
                const float minDistance = pMP->GetMinDistanceInvariance();
                //地图点深度必须在可观测深度范围
                if(dist3D<minDistance || dist3D>maxDistance)
                    continue;
                //根据地图点深度计算当前帧观测到该点的金字塔层级
                int nPredictedLevel = pMP->PredictScale(dist3D, &CurrentFrame);
                //根据金字塔层级计算搜索半径
                const float radius = th*CurrentFrame.mvScaleFactors[nPredictedLevel];
                //按搜索半径搜索特征点
                const vector<size_t> vIndices2 = CurrentFrame.GetFeaturesInArea(u, v, radius, nPredictedLevel-1, nPredictedLevel+1);
                //没搜索到处理下一个
                if(vIndices2.empty())
                    continue;
                //取出描述子
                const cv::Mat dMP = pMP->GetDescriptor();
                int bestDist = 256;
                int bestIdx2 = -1;
                //遍历搜索到的特征点
                for(vector<size_t>::const_iterator vit=vIndices2.begin(); vit!=vIndices2.end(); vit++)
                {
                    const size_t i2 = *vit;
                    if(CurrentFrame.mvpMapPoints[i2])
                        continue;
                    //取出描述子
                    const cv::Mat &d = CurrentFrame.mDescriptors.row(i2);
                    //计算汉明距离
                    const int dist = DescriptorDistance(dMP,d);
                    //找出最好结果和对应特征点索引
                    if(dist<bestDist)
                    {
                        bestDist=dist;
                        bestIdx2=i2;
                    }
                }
                //最好的匹配点的结果要好于阈值
                if(bestDist<=ORBdist)
                {
                    CurrentFrame.mvpMapPoints[bestIdx2]=pMP;   //把匹配的地图点赋值给当前帧对应地图点索引
                    nmatches++;
                    //计算旋转差直方图
                    if(mbCheckOrientation)
                    {
                        float rot = pKF->mvKeysUn[i].angle-CurrentFrame.mvKeysUn[bestIdx2].angle;
                        if(rot<0.0)
                            rot+=360.0f;
                        int bin = round(rot*factor);
                        if(bin==HISTO_LENGTH)
                            bin=0;
                        assert(bin>=0 && bin<HISTO_LENGTH);
                        rotHist[bin].push_back(bestIdx2);
                    }
                }
            }
        }
    }
    //筛选旋转差直方图最好的前三项特征点
    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;
        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);
        //旋转差较差的对应地图点置为空
        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i!=ind1 && i!=ind2 && i!=ind3)
            {
                for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                {
                    CurrentFrame.mvpMapPoints[rotHist[i][j]]=NULL;
                    nmatches--;
                }
            }
        }
    }
    //返回匹配数量
    return nmatches;
}

void ORBmatcher::ComputeThreeMaxima(vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3)
{
    int max1=0;
    int max2=0;
    int max3=0;

    for(int i=0; i<L; i++)
    {
        const int s = histo[i].size();
        if(s>max1)
        {
            max3=max2;
            max2=max1;
            max1=s;
            ind3=ind2;
            ind2=ind1;
            ind1=i;
        }
        else if(s>max2)
        {
            max3=max2;
            max2=s;
            ind3=ind2;
            ind2=i;
        }
        else if(s>max3)
        {
            max3=s;
            ind3=i;
        }
    }

    if(max2<0.1f*(float)max1)
    {
        ind2=-1;
        ind3=-1;
    }
    else if(max3<0.1f*(float)max1)
    {
        ind3=-1;
    }
}


//通过比特位计算描述子距离
//http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
int ORBmatcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
{
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();
    int dist=0;
    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }
    return dist;
}

} //namespace ORB_SLAM
