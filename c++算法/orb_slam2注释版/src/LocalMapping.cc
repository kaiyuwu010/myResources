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

#include "LocalMapping.h"
#include "LoopClosing.h"
#include "ORBmatcher.h"
#include "Optimizer.h"

#include<mutex>

namespace ORB_SLAM2
{

LocalMapping::LocalMapping(Map *pMap, const float bMonocular):
    mbMonocular(bMonocular), mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
    mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true)
{
}

void LocalMapping::SetLoopCloser(LoopClosing* pLoopCloser)
{
    mpLoopCloser = pLoopCloser;
}

void LocalMapping::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}


void LocalMapping::Run()
{
    //标记状态，表示当前run函数正在运行
    mbFinished = false;
    while(1)
    {
        //告诉Tracking，LocalMapping正处于繁忙状态
        SetAcceptKeyFrames(false);
        //等待处理的关键帧列表不为空
        if(CheckNewKeyFrames())
        {
            //处理列表中的关键帧，包括计算BoW、更新观测、描述子、共视图，插入到地图等
            ProcessNewKeyFrame();
            //根据地图点的观测情况剔除质量不好的地图点，主要是针对非单目相机新产生的地图点
            MapPointCulling();
            //当前关键帧与相邻关键帧通过三角化产生新的地图点，使得跟踪更稳
            CreateNewMapPoints();
            //已经处理完队列中的最后的一个关键帧
            if(!CheckNewKeyFrames())
            {
                //检查并融合当前关键帧与相邻关键帧帧（两级相邻）中重复的地图点
                SearchInNeighbors();
            }
            mbAbortBA = false;
            if(!CheckNewKeyFrames() && !stopRequested())
            {
                //当局部地图中的关键帧大于2个的时候进行局部地图的BA
                if(mpMap->KeyFramesInMap()>2)
                    //注意这里的第二个参数是按地址传递的，当mbAbortBA状态发生变化时能够及时执行/停止BA
                    Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame, &mbAbortBA, mpMap);
                //检测并剔除与当前帧相邻的关键帧中冗余的关键帧
                KeyFrameCulling();
            }
            //将当前帧加入到闭环检测队列中
            mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);
        }
        //当要终止当前LocalMapping线程的时候
        else if(Stop())
        {
            //等待线程结束
            while(isStopped() && !CheckFinish())
            {
                usleep(3000);
            }
            //确定终止了就跳出这个线程的主循环
            if(CheckFinish())
                break;
        }
        //查看是否有复位线程的请求
        ResetIfRequested();
        //处理完毕接收新的关键帧
        SetAcceptKeyFrames(true);
        //如果当前线程已经结束了就跳出主循环
        if(CheckFinish())
            break;
        usleep(3000);
    }
    //设置线程已经终止标志位，mbFinished和mbStopped
    SetFinish();
}

void LocalMapping::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.push_back(pKF);
    mbAbortBA=true;
}


bool LocalMapping::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    return(!mlNewKeyFrames.empty());
}

void LocalMapping::ProcessNewKeyFrame()
{
    {
        unique_lock<mutex> lock(mMutexNewKFs);
        //从待处理列表取出要插入的新关键帧
        mpCurrentKeyFrame = mlNewKeyFrames.front();
        mlNewKeyFrames.pop_front();
    }
    //计算mBowVec和mFeatVec
    mpCurrentKeyFrame->ComputeBoW();
    //取出该关键帧的地图点
    const vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    //遍历地图点
    for(size_t i=0; i<vpMapPointMatches.size(); i++)
    {
        MapPoint* pMP = vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                //如果该地图点不在关键帧的观测中
                if(!pMP->IsInKeyFrame(mpCurrentKeyFrame))
                {
                    pMP->AddObservation(mpCurrentKeyFrame, i); //添加当前关键帧的观测
                    pMP->UpdateNormalAndDepth();           //更新平均观测方向和平均观测距离
                    pMP->ComputeDistinctiveDescriptors();  //计算最具代表性的描述子
                }
                //如果这个地图点不是来自当前帧的观测（这些地图点可能来自双目或RGBD跟踪过程中新生成的地图点，或者是CreateNewMapPoints 中通过三角化产生），将上述地图点放入mlpRecentAddedMapPoints，等待后续MapPointCulling函数的检验。
                else 
                {
                    mlpRecentAddedMapPoints.push_back(pMP);
                }
            }
        }
    }    
    //更新共视图连接
    mpCurrentKeyFrame->UpdateConnections();
    //地图中插入关键帧
    mpMap->AddKeyFrame(mpCurrentKeyFrame);
}

//检查新增地图点，根据地图点的观测情况剔除质量不好的新增的地图点。
void LocalMapping::MapPointCulling()
{
    //检测最近新增的地图点
    list<MapPoint*>::iterator lit = mlpRecentAddedMapPoints.begin();
    const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;
    //据相机类型设置不同的观测阈值
    int nThObs;
    if(mbMonocular)
        nThObs = 2;
    else
        nThObs = 3;
    const int cnThObs = nThObs;
    //遍历新添加的地图点
    while(lit!=mlpRecentAddedMapPoints.end())
    {
        MapPoint* pMP = *lit;
        //已经是坏点的地图点仅从队列中删除
        if(pMP->isBad())
        {
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        //跟踪到该地图点的帧数相比预计可观测到该地图点的帧数的比例小于25%，从地图中删除
        else if(pMP->GetFoundRatio()<0.25f )
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        //从该点建立开始，到现在已经过了不小于2个关键帧，但是该点的观测数却不超过阈值cnThObs，从地图中删除
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=2 && pMP->Observations()<=cnThObs)
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        //从建立该点开始，已经过了3个关键帧，仅从队列中删除
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=3)
            lit = mlpRecentAddedMapPoints.erase(lit);
        else
            lit++;
    }
}

void LocalMapping::CreateNewMapPoints()
{
    //nn表示搜索最佳共视关键帧的数目
    int nn = 10;
    //单目需要有更多的具有较好共视关系的关键帧来建立地图
    if(mbMonocular)
        nn=20;
    //获取共视关键帧
    const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
    ORBmatcher matcher(0.6,false);
    //取出当前帧从世界坐标系到相机坐标系的旋转矩阵
    cv::Mat Rcw1 = mpCurrentKeyFrame->GetRotation();
    //计算当前帧相机坐标系到世界坐标系的旋转矩阵
    cv::Mat Rwc1 = Rcw1.t();
    //取出当前帧从世界坐标系到相机坐标系的位移
    cv::Mat tcw1 = mpCurrentKeyFrame->GetTranslation();
    //计算世界坐标系到相机坐标系的变换矩阵
    cv::Mat Tcw1(3, 4, CV_32F);
    Rcw1.copyTo(Tcw1.colRange(0,3));
    tcw1.copyTo(Tcw1.col(3));
    //得到当前关键帧（左目）光心在世界坐标系中的坐标
    cv::Mat Ow1 = mpCurrentKeyFrame->GetCameraCenter();
    //得到当前关键帧的内参
    const float &fx1 = mpCurrentKeyFrame->fx;
    const float &fy1 = mpCurrentKeyFrame->fy;
    const float &cx1 = mpCurrentKeyFrame->cx;
    const float &cy1 = mpCurrentKeyFrame->cy;
    const float &invfx1 = mpCurrentKeyFrame->invfx;
    const float &invfy1 = mpCurrentKeyFrame->invfy;
    //用于后面的点深度的验证
    const float ratioFactor = 1.5f*mpCurrentKeyFrame->mfScaleFactor;
    //记录三角化成功的地图点数目
    int nnew=0;
    //遍历相邻关键帧，搜索匹配并用极线约束剔除误匹配，最终三角化
    for(size_t i=0; i<vpNeighKFs.size(); i++)
    {
        if(i>0 && CheckNewKeyFrames())
            return;

        KeyFrame* pKF2 = vpNeighKFs[i];

        //相邻的关键帧光心在世界坐标系中的坐标
        cv::Mat Ow2 = pKF2->GetCameraCenter();
        //基线向量，两个关键帧间的相机位移
        cv::Mat vBaseline = Ow2-Ow1;
        const float baseline = cv::norm(vBaseline);
        //判断相机运动的基线是不是足够长
        if(!mbMonocular)
        {
            //如果是双目相机，关键帧间距小于本身的基线时不生成3D点
            if(baseline<pKF2->mb)
                continue;
        }
        //单目相机情况
        else
        {
            //计算相邻关键帧的场景深度中值（地图点最大深度的一半）
            const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
            //基线与景深的比例
            const float ratioBaselineDepth = baseline/medianDepthKF2;
            //如果比例特别小，基线太短恢复3D点不准，那么跳过当前邻接关键帧
            if(ratioBaselineDepth<0.01)
                continue;
        }
        //根据关键帧间的位姿计算基本矩阵
        cv::Mat F12 = ComputeF12(mpCurrentKeyFrame, pKF2);
        //通过词袋对两关键帧的未匹配的特征点快速匹配，用极线约束抑制离群点，生成新的匹配点对
        vector<pair<size_t,size_t> > vMatchedIndices;
        matcher.SearchForTriangulation(mpCurrentKeyFrame, pKF2, F12, vMatchedIndices, false);
        //计算世界坐标系到相邻关键帧相机坐标系的变换矩阵
        cv::Mat Rcw2 = pKF2->GetRotation();
        cv::Mat Rwc2 = Rcw2.t();
        cv::Mat tcw2 = pKF2->GetTranslation();
        cv::Mat Tcw2(3,4,CV_32F);
        Rcw2.copyTo(Tcw2.colRange(0,3));
        tcw2.copyTo(Tcw2.col(3));
        //获取相邻关键帧的内参
        const float &fx2 = pKF2->fx;
        const float &fy2 = pKF2->fy;
        const float &cx2 = pKF2->cx;
        const float &cy2 = pKF2->cy;
        const float &invfx2 = pKF2->invfx;
        const float &invfy2 = pKF2->invfy;
        //对每对匹配通过三角化生成3D点，和Triangulate函数差不多
        const int nmatches = vMatchedIndices.size();
        for(int ikp=0; ikp<nmatches; ikp++)
        {
            //取出匹配点的序号
            const int &idx1 = vMatchedIndices[ikp].first;
            const int &idx2 = vMatchedIndices[ikp].second;
            //当前匹配在当前关键帧中的特征点
            const cv::KeyPoint &kp1 = mpCurrentKeyFrame->mvKeysUn[idx1];
            const float kp1_ur=mpCurrentKeyFrame->mvuRight[idx1];
            bool bStereo1 = kp1_ur>=0;
            //当前匹配在邻接关键帧中的特征点
            const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];
            const float kp2_ur = pKF2->mvuRight[idx2];
            bool bStereo2 = kp2_ur>=0;
            //计算在两帧中各自相机对同一地图点的视差
            cv::Mat xn1 = (cv::Mat_<float>(3,1) << (kp1.pt.x-cx1)*invfx1, (kp1.pt.y-cy1)*invfy1, 1.0);
            cv::Mat xn2 = (cv::Mat_<float>(3,1) << (kp2.pt.x-cx2)*invfx2, (kp2.pt.y-cy2)*invfy2, 1.0);
            cv::Mat ray1 = Rwc1*xn1;
            cv::Mat ray2 = Rwc2*xn2;
            //匹配点射线夹角余弦值
            const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));
            //如果非单目，重新计算视差
            float cosParallaxStereo = cosParallaxRays+1;
            float cosParallaxStereo1 = cosParallaxStereo;
            float cosParallaxStereo2 = cosParallaxStereo;
            if(bStereo1)
                //假设是平行的双目相机，计算出双目相机观察这个点的时候的视差角余弦
                cosParallaxStereo1 = cos(2*atan2(mpCurrentKeyFrame->mb/2,mpCurrentKeyFrame->mvDepth[idx1]));
            else if(bStereo2)
                cosParallaxStereo2 = cos(2*atan2(pKF2->mb/2,pKF2->mvDepth[idx2]));
            //视差取较小值
            cosParallaxStereo = min(cosParallaxStereo1,cosParallaxStereo2);
            cv::Mat x3D;
            //匹配点对夹角大，用三角法恢复3D点
            if(cosParallaxRays<cosParallaxStereo && cosParallaxRays>0 && (bStereo1 || bStereo2 || cosParallaxRays<0.9998))
            {
                //线性三角化
                cv::Mat A(4,4,CV_32F);
                A.row(0) = xn1.at<float>(0)*Tcw1.row(2)-Tcw1.row(0);
                A.row(1) = xn1.at<float>(1)*Tcw1.row(2)-Tcw1.row(1);
                A.row(2) = xn2.at<float>(0)*Tcw2.row(2)-Tcw2.row(0);
                A.row(3) = xn2.at<float>(1)*Tcw2.row(2)-Tcw2.row(1);
                cv::Mat w,u,vt;
                cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
                x3D = vt.row(3).t();
                if(x3D.at<float>(3)==0)
                    continue;
                //归一化成为齐次坐标,然后提取前面三个维度作为欧式坐标
                x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
            }
            //匹配点对夹角小，用双目恢复3D点
            else if(bStereo1 && cosParallaxStereo1<cosParallaxStereo2)
            {
                x3D = mpCurrentKeyFrame->UnprojectStereo(idx1);                
            }
            else if(bStereo2 && cosParallaxStereo2<cosParallaxStereo1)
            {
                x3D = pKF2->UnprojectStereo(idx2);
            }
            //单目相机且视差角小，跳过
            else
                continue;
            cv::Mat x3Dt = x3D.t();
            //检测生成的3D点是否在两帧相机前方,不在的话就放弃这个点
            float z1 = Rcw1.row(2).dot(x3Dt)+tcw1.at<float>(2);
            if(z1<=0)
                continue;
            float z2 = Rcw2.row(2).dot(x3Dt)+tcw2.at<float>(2);
            if(z2<=0)
                continue;
            //计算3D点在当前关键帧下的重投影误差
            const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
            //3D点坐标转化到当前帧相机坐标系
            const float x1 = Rcw1.row(0).dot(x3Dt)+tcw1.at<float>(0);
            const float y1 = Rcw1.row(1).dot(x3Dt)+tcw1.at<float>(1);
            const float invz1 = 1.0/z1;
            //单目情况
            if(!bStereo1)
            {
                //3D坐标投影为图像坐标，并计算误差
                float u1 = fx1*x1*invz1+cx1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                //检验误差，根据金字塔层级
                if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1)
                    continue;
            }
            //非单目情况
            else
            {
                float u1 = fx1*x1*invz1+cx1;
                float u1_r = u1 - mpCurrentKeyFrame->mbf*invz1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                float errX1_r = u1_r - kp1_ur;
                if((errX1*errX1+errY1*errY1+errX1_r*errX1_r)>7.8*sigmaSquare1)
                    continue;
            }
            //计算3D点在另一个关键帧下的重投影误差，操作同上
            const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
            const float x2 = Rcw2.row(0).dot(x3Dt)+tcw2.at<float>(0);
            const float y2 = Rcw2.row(1).dot(x3Dt)+tcw2.at<float>(1);
            const float invz2 = 1.0/z2;
            if(!bStereo2)
            {
                float u2 = fx2*x2*invz2+cx2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2)
                    continue;
            }
            else
            {
                float u2 = fx2*x2*invz2+cx2;
                float u2_r = u2 - mpCurrentKeyFrame->mbf*invz2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                float errX2_r = u2_r - kp2_ur;
                if((errX2*errX2+errY2*errY2+errX2_r*errX2_r)>7.8*sigmaSquare2)
                    continue;
            }
            //世界坐标系下，3D点与相机间的向量，方向由相机指向3D点
            cv::Mat normal1 = x3D-Ow1;
            float dist1 = cv::norm(normal1);
            cv::Mat normal2 = x3D-Ow2;
            float dist2 = cv::norm(normal2);
            //距离为0，跳过
            if(dist1==0 || dist2==0)
                continue;
            //ratioDist是不考虑金字塔尺度下的距离比例
            const float ratioDist = dist2/dist1;
            //金字塔层级比例
            const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave]/pKF2->mvScaleFactors[kp2.octave];
            //距离的比例和图像金字塔的比例不应该差太多，否则就跳过
            if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
                continue;
            //三角化成功，生成地图点
            MapPoint* pMP = new MapPoint(x3D, mpCurrentKeyFrame, mpMap);
            //添加观测
            pMP->AddObservation(mpCurrentKeyFrame,idx1);            
            pMP->AddObservation(pKF2,idx2);
            //关键帧添加地图点
            mpCurrentKeyFrame->AddMapPoint(pMP,idx1);
            pKF2->AddMapPoint(pMP,idx2);
            //更新描述子，方向向量和平均深度
            pMP->ComputeDistinctiveDescriptors();
            pMP->UpdateNormalAndDepth();
            //地图添加地图点
            mpMap->AddMapPoint(pMP);
            //最近新增地图点向量存入新点
            mlpRecentAddedMapPoints.push_back(pMP);
            //新地图点计数加1
            nnew++;
        }
    }
}

//检查并融合当前关键帧与相邻帧（两级相邻）重复的地图点
void LocalMapping::SearchInNeighbors()
{
    //取共视关键帧
    int nn = 10;
    if(mbMonocular)
        nn=20;
    const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
    vector<KeyFrame*> vpTargetKFs;
    //遍历共视关键帧
    for(vector<KeyFrame*>::const_iterator vit=vpNeighKFs.begin(), vend=vpNeighKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        //检查是否是坏帧或者是否与当前帧进行过融合的操作
        if(pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)
            continue;
        //存入待融合帧向量
        vpTargetKFs.push_back(pKFi);
        //记录发生融合操作的帧id
        pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;
        //取出二级共视关键帧
        const vector<KeyFrame*> vpSecondNeighKFs = pKFi->GetBestCovisibilityKeyFrames(5);
        //遍历二级关键帧
        for(vector<KeyFrame*>::const_iterator vit2=vpSecondNeighKFs.begin(), vend2=vpSecondNeighKFs.end(); vit2!=vend2; vit2++)
        {
            KeyFrame* pKFi2 = *vit2;
            //检查是否是坏帧，是否与当前帧进行过融合的操作，是否是当前关键帧
            if(pKFi2->isBad() || pKFi2->mnFuseTargetForKF==mpCurrentKeyFrame->mnId || pKFi2->mnId==mpCurrentKeyFrame->mnId)
                continue;
            //存入待融合帧向量
            vpTargetKFs.push_back(pKFi2);
        }
    }
    //将当前帧地图点投影在目标关键帧中搜索匹配并融合，称为正向投影融合
    ORBmatcher matcher;
    vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(vector<KeyFrame*>::iterator vit=vpTargetKFs.begin(), vend=vpTargetKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        //进行地图点融合
        matcher.Fuse(pKFi, vpMapPointMatches);
    }
    //将目标关键帧地图点投影到当前关键帧，寻找匹配点对应的地图点进行融合，称为反向投影融合
    vector<MapPoint*> vpFuseCandidates;
    vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());
    //遍历一二级相邻关键帧，存储这些关键帧的所有MapPoints的集合
    for(vector<KeyFrame*>::iterator vitKF=vpTargetKFs.begin(), vendKF=vpTargetKFs.end(); vitKF!=vendKF; vitKF++)
    {
        //取出地图点
        KeyFrame* pKFi = *vitKF;
        vector<MapPoint*> vpMapPointsKFi = pKFi->GetMapPointMatches();
        //遍历地图点去掉空点、坏点和已经融合过的点
        for(vector<MapPoint*>::iterator vitMP=vpMapPointsKFi.begin(), vendMP=vpMapPointsKFi.end(); vitMP!=vendMP; vitMP++)
        {
            MapPoint* pMP = *vitMP;
            if(!pMP)
                continue;
            if(pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId)
                continue;
            pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
            vpFuseCandidates.push_back(pMP);
        }
    }
    //融合
    matcher.Fuse(mpCurrentKeyFrame, vpFuseCandidates);
    //取出当前关键帧所有地图点
    vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    //遍历地图点，进行描述子、观测方向和观测深度更新
    for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
    {
        MapPoint* pMP=vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                pMP->ComputeDistinctiveDescriptors();
                pMP->UpdateNormalAndDepth();
            }
        }
    }
    //更新共视图连接
    mpCurrentKeyFrame->UpdateConnections();
}

cv::Mat LocalMapping::ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2)
{
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    cv::Mat R12 = R1w*R2w.t();

    cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

    cv::Mat t12x = SkewSymmetricMatrix(t12);

    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;

    return K1.t().inv()*t12x*R12*K2.inv();
}

void LocalMapping::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;
    unique_lock<mutex> lock2(mMutexNewKFs);
    mbAbortBA = true;
}

bool LocalMapping::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(mbStopRequested && !mbNotStop)
    {
        mbStopped = true;
        cout << "Local Mapping STOP" << endl;
        return true;
    }

    return false;
}

bool LocalMapping::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool LocalMapping::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

void LocalMapping::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);
    if(mbFinished)
        return;
    mbStopped = false;
    mbStopRequested = false;
    for(list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
        delete *lit;
    mlNewKeyFrames.clear();

    cout << "Local Mapping RELEASE" << endl;
}

bool LocalMapping::AcceptKeyFrames()
{
    unique_lock<mutex> lock(mMutexAccept);
    return mbAcceptKeyFrames;
}

void LocalMapping::SetAcceptKeyFrames(bool flag)
{
    unique_lock<mutex> lock(mMutexAccept);
    mbAcceptKeyFrames=flag;
}

bool LocalMapping::SetNotStop(bool flag)
{
    unique_lock<mutex> lock(mMutexStop);

    if(flag && mbStopped)
        return false;

    mbNotStop = flag;

    return true;
}

void LocalMapping::InterruptBA()
{
    mbAbortBA = true;
}

//剔除冗余关键帧，冗余关键帧的判定：90%以上的地图点能被其他关键帧（至少3个）观测到
void LocalMapping::KeyFrameCulling()
{
    //根据共视图提取当前关键帧的所有共视关键帧
    vector<KeyFrame*> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();
    //遍历共视关键帧
    for(vector<KeyFrame*>::iterator vit=vpLocalKeyFrames.begin(), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
    {
        KeyFrame* pKF = *vit;
        //第1个关键帧不能删除，跳过
        if(pKF->mnId==0)
            continue;
        //提取每个共视关键帧的地图点
        const vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();
        //记录某个点被观测次数
        int nObs = 3;
        //观测次数阈值，默认为3
        const int thObs=nObs;
        //记录冗余观测点的数目
        int nRedundantObservations=0;
        int nMPs=0;
        //遍历该共视关键帧的所有地图点，其中能被其它至少3个关键帧观测到的地图点为冗余地图点
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    if(!mbMonocular)
                    {
                        //对于双目或RGB-D，仅考虑近处（不超过基线的40倍）的地图点
                        if(pKF->mvDepth[i]>pKF->mThDepth || pKF->mvDepth[i]<0)
                            continue;
                    }
                    nMPs++;
                    //观测到该地图点的关键帧数目大于阈值3
                    if(pMP->Observations()>thObs)
                    {
                        const int &scaleLevel = pKF->mvKeysUn[i].octave;
                        //Observation存储的是可以看到该地图点的所有关键帧的集合
                        const map<KeyFrame*, size_t> observations = pMP->GetObservations();
                        int nObs=0;
                        //遍历观测到该地图点的关键帧
                        for(map<KeyFrame*, size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                        {
                            KeyFrame* pKFi = mit->first;
                            //跳过自己关键帧
                            if(pKFi==pKF)
                                continue;
                            //取出金字塔层级
                            const int &scaleLeveli = pKFi->mvKeysUn[mit->second].octave;
                            // 尺度约束：为什么pKF 尺度+1 要大于等于 pKFi 尺度？
                            // 回答：因为同样或更低金字塔层级的地图点更准确
                            if(scaleLeveli<=scaleLevel+1)
                            {
                                nObs++;
                                //已经找到3个满足条件的关键帧，就停止不找了
                                if(nObs>=thObs)
                                    break;
                            }
                        }
                        //地图点至少被3个关键帧观测到，就记录为冗余点，更新冗余点计数数目
                        if(nObs>=thObs)
                        {
                            nRedundantObservations++;
                        }
                    }
                }
            }
        }  

        if(nRedundantObservations>0.9*nMPs)
            pKF->SetBadFlag();
    }
}

cv::Mat LocalMapping::SkewSymmetricMatrix(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) <<             0, -v.at<float>(2), v.at<float>(1),
            v.at<float>(2),               0,-v.at<float>(0),
            -v.at<float>(1),  v.at<float>(0),              0);
}

void LocalMapping::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequested)
                break;
        }
        usleep(3000);
    }
}

void LocalMapping::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        mlNewKeyFrames.clear();
        mlpRecentAddedMapPoints.clear();
        mbResetRequested=false;
    }
}

void LocalMapping::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LocalMapping::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LocalMapping::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;    
    unique_lock<mutex> lock2(mMutexStop);
    mbStopped = true;
}

bool LocalMapping::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

} //namespace ORB_SLAM
