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

#include "LoopClosing.h"

#include "Sim3Solver.h"

#include "Converter.h"

#include "Optimizer.h"

#include "ORBmatcher.h"

#include<mutex>
#include<thread>


namespace ORB_SLAM2
{

LoopClosing::LoopClosing(Map *pMap, KeyFrameDatabase *pDB, ORBVocabulary *pVoc, const bool bFixScale):
    mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
    mpKeyFrameDB(pDB), mpORBVocabulary(pVoc), mpMatchedKF(NULL), mLastLoopKFid(0), mbRunningGBA(false), mbFinishedGBA(true),
    mbStopGBA(false), mpThreadGBA(NULL), mbFixScale(bFixScale), mnFullBAIdx(0)
{
    mnCovisibilityConsistencyTh = 3;
}

void LoopClosing::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

void LoopClosing::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}


void LoopClosing::Run()
{
    mbFinished =false;

    while(1)
    {
        //查看闭环检测队列mlpLoopKeyFrameQueue中有没有关键帧进来
        if(CheckNewKeyFrames())
        {
            //筛选出当前帧mpCurrentKF的闭环候选关键帧，更新连续组关系mvConsistentGroups
            if(DetectLoop())
            {
               //通过sim3变换剔除一部分不满足要求的闭环候选帧，对于剩下的闭环候选关键帧，我们遍历每一个闭环候选帧进行Sim3Solver求解，得到候选帧pKF到当前帧mpCurrentKF的R
               if(ComputeSim3())
               {
                   //通过求解的Sim3以及相对姿态关系，调整与当前帧相连的关键帧位姿以及这些关键帧观测到的地图点位置
                   CorrectLoop();
               }
            }
        }       
        //查看是否有外部线程请求复位当前线程
        ResetIfRequested();
        //查看外部线程是否有终止当前线程的请求,如果有的话就跳出这个线程的主函数的主循环
        if(CheckFinish())
            break;

        usleep(5000);
    }
    //运行到这里说明有外部线程请求终止当前线程,在这个函数中执行终止当前线程的一些操作
    SetFinish();
}

void LoopClosing::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    if(pKF->mnId!=0)
        mlpLoopKeyFrameQueue.push_back(pKF);
}

bool LoopClosing::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    return(!mlpLoopKeyFrameQueue.empty());
}

//筛选出当前帧mpCurrentKF的闭环候选关键帧，更新连续组关系mvConsistentGroups
bool LoopClosing::DetectLoop()
{
    {
        unique_lock<mutex> lock(mMutexLoopQueue);
        //从队列头开始取先进来的关键帧
        mpCurrentKF = mlpLoopKeyFrameQueue.front();
        mlpLoopKeyFrameQueue.pop_front();
        //设置当前关键帧不要在优化的过程中被删除
        mpCurrentKF->SetNotErase();
    }
    //如果距离上次闭环没多久（小于10帧），或者map中关键帧总共还没有10帧，则不进行闭环检测
    if(mpCurrentKF->mnId<mLastLoopKFid+10)
    {
        //把关键帧添加到关键帧数据库
        mpKeyFrameDB->add(mpCurrentKF);
        mpCurrentKF->SetErase();
        return false;
    }
    //遍历当前回环关键帧所有连接（>15个共视地图点）关键帧，计算当前关键帧与每个共视关键的bow相似度得分，并得到最低得分minScore（找候选闭环关键帧必须大于最低得分）
    const vector<KeyFrame*> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();
    const DBoW2::BowVector &CurrentBowVec = mpCurrentKF->mBowVec;
    float minScore = 1;
    for(size_t i=0; i<vpConnectedKeyFrames.size(); i++)
    {
        KeyFrame* pKF = vpConnectedKeyFrames[i];
        if(pKF->isBad())
            continue;
        const DBoW2::BowVector &BowVec = pKF->mBowVec;
        //计算与当前帧的相似度得分
        float score = mpORBVocabulary->score(CurrentBowVec, BowVec);
        //找出最低得分
        if(score<minScore)
            minScore = score;
    }
    //在所有关键帧（mpKeyFrameDB）中找出闭环候选帧（注意不和当前帧共视）
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectLoopCandidates(mpCurrentKF, minScore);
    //如果没有闭环候选帧，返回false
    if(vpCandidateKFs.empty())
    {
        //把关键帧添加到关键帧数据库
        mpKeyFrameDB->add(mpCurrentKF);
        //清理连续组容器
        mvConsistentGroups.clear();//ConsistentGroup的定义：typedef pair<set<KeyFrame*>, int> ConsistentGroup，其中 ConsistentGroup.first对应每个“连续组”中的关键帧集合，ConsistentGroup.second为每个“连续组”的连续长度
        mpCurrentKF->SetErase();
        return false;
    }
    mvpEnoughConsistentCandidates.clear(); //清理存放达到连续条件的候选关键帧容器
    vector<ConsistentGroup> vCurrentConsistentGroups; //声明当前连续组
    vector<bool> vbConsistentGroup(mvConsistentGroups.size(), false);//记录当前连续组与之前闭环检测连续组的连续关系
    //遍历刚才得到的每一个候选关键帧
    for(size_t i=0, iend=vpCandidateKFs.size(); i<iend; i++)
    {
        KeyFrame* pCandidateKF = vpCandidateKFs[i];
        //将候选关键帧自己以及与自己共视的关键帧构成一个“子候选组”
        set<KeyFrame*> spCandidateGroup = pCandidateKF->GetConnectedKeyFrames();
        spCandidateGroup.insert(pCandidateKF);
        //连续性达标的标志
        bool bEnoughConsistent = false;
        bool bConsistentForSomeGroup = false;
        //遍历之前闭环检测到的连续组的子连续组，第一次闭环检测该容器为空
        for(size_t iG=0, iendG=mvConsistentGroups.size(); iG<iendG; iG++)
        {
            //取出子连续组中的关键帧集合
            set<KeyFrame*> sPreviousGroup = mvConsistentGroups[iG].first;
            //遍历每个“子候选组”每一帧在子连续组中是否存在
            bool bConsistent = false;
            for(set<KeyFrame*>::iterator sit=spCandidateGroup.begin(), send=spCandidateGroup.end(); sit!=send;sit++)
            {
                //如果存在，该“子候选组”与该“子连续组”相连
                if(sPreviousGroup.count(*sit))
                {
                    bConsistent=true;
                    bConsistentForSomeGroup = true;
                    break;
                }
            }
            //如果连续
            if(bConsistent)
            {
                //取出和当前的候选组发生"连续"关系的子连续组的"已连续次数"
                int nPreviousConsistency = mvConsistentGroups[iG].second;
                //连续次数加一
                int nCurrentConsistency = nPreviousConsistency + 1;
                //如果上述连续关系还未记录到 vCurrentConsistentGroups，那么记录一下
                if(!vbConsistentGroup[iG])
                {
                    ConsistentGroup cg = make_pair(spCandidateGroup, nCurrentConsistency);
                    //把候选帧组添加到当前连续组
                    vCurrentConsistentGroups.push_back(cg);
                    //标记一下，防止重复添加到同一个子连续组
                    vbConsistentGroup[iG]=true; 
                }
                //如果连续长度满足要求，那么当前的这个候选关键帧是足够靠谱的
                if(nCurrentConsistency>=mnCovisibilityConsistencyTh && !bEnoughConsistent)
                {
                    //记录达到连续条件的候选关键帧
                    mvpEnoughConsistentCandidates.push_back(pCandidateKF);
                    //标记一下，防止重复添加
                    bEnoughConsistent = true; 
                }
            }
        }
        //如果该“子候选组”的所有关键帧都和上次闭环连续组不连续（不连续），vCurrentConsistentGroups没有新添加连续关系
        if(!bConsistentForSomeGroup)
        {
            ConsistentGroup cg = make_pair(spCandidateGroup, 0);
            //把候选帧组添加到当前连续组
            vCurrentConsistentGroups.push_back(cg);
        }
    }
    //更新闭环检测线程连续组
    mvConsistentGroups = vCurrentConsistentGroups;
    //添加帧到数据库
    mpKeyFrameDB->add(mpCurrentKF);
    //如果没有达到连续条件的候选关键帧，返回false
    if(mvpEnoughConsistentCandidates.empty())
    {
        mpCurrentKF->SetErase();
        return false;
    }
    else
    {
        return true;
    }
    mpCurrentKF->SetErase();
    return false;
}

bool LoopClosing::ComputeSim3()
{
    //对每个具有足够连续关系的闭环候选帧都准备算一个Sim3
    const int nInitialCandidates = mvpEnoughConsistentCandidates.size();
    //为每个候选帧查找候选匹配
    ORBmatcher matcher(0.75,true);
    //存储每一个候选帧的Sim3Solver求解器
    vector<Sim3Solver*> vpSim3Solvers;
    vpSim3Solvers.resize(nInitialCandidates);
    //存储每个候选帧的匹配地图点信息
    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nInitialCandidates);
    //存储每个候选帧应该被放弃(True）或者保留(False)的信息
    vector<bool> vbDiscarded;
    vbDiscarded.resize(nInitialCandidates);
    //通过词袋匹配后，被保留的候选帧数量
    int nCandidates=0; 
    //遍历闭环候选帧集，初步筛选出与当前关键帧的匹配特征点数大于20的候选帧集合，并为每一个候选帧构造一个Sim3Solver
    for(int i=0; i<nInitialCandidates; i++)
    {
        KeyFrame* pKF = mvpEnoughConsistentCandidates[i];
        //避免局部建图线程删除该关键帧
        pKF->SetNotErase();
        //坏帧直接删除
        if(pKF->isBad())
        {
            vbDiscarded[i] = true;
            continue;
        }
        //通过词袋匹配，找出匹配点
        int nmatches = matcher.SearchByBoW(mpCurrentKF, pKF, vvpMapPointMatches[i]);
        //粗筛：匹配的特征点数太少，该候选帧剔除
        if(nmatches<20)
        {
            vbDiscarded[i] = true;
            continue;
        }
        else
        {
            //为保留的候选帧构造Sim3求解器
            Sim3Solver* pSolver = new Sim3Solver(mpCurrentKF, pKF, vvpMapPointMatches[i], mbFixScale);
            //Sim3Solver Ransac过程置信度0.99，至少20个inliers，最多300次迭代
            pSolver->SetRansacParameters(0.99, 20, 300);
            vpSim3Solvers[i] = pSolver;
        }
        //保留的候选帧数量加一
        nCandidates++;
    }
    bool bMatch = false;
    //对每一个候选帧用Sim3Solver 迭代匹配，直到有一个候选帧匹配成功，或者全部失败
    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nInitialCandidates; i++)
        {
            //跳过抛弃的帧
            if(vbDiscarded[i])
                continue;
            KeyFrame* pKF = mvpEnoughConsistentCandidates[i];
            //即标记经过RANSAC、sim3求解后，vvpMapPointMatches中的哪些作为内点
            vector<bool> vbInliers;
            //内点（Inliers）数量
            int nInliers;
            //是否到达了最优解
            bool bNoMore;
            Sim3Solver* pSolver = vpSim3Solvers[i];
            cv::Mat Scm  = pSolver->iterate(5, bNoMore, vbInliers, nInliers);
            //总迭代次数达到最大限制还没有求出合格的Sim3变换，该候选帧剔除
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }
            //如果计算出了Sim3变换，继续匹配出更多点并优化。因为之前SearchByBoW匹配可能会有遗漏
            if(!Scm.empty())
            {
                //取出经过Sim3Solver后匹配点中的内点集合
                vector<MapPoint*> vpMapPointMatches(vvpMapPointMatches[i].size(), static_cast<MapPoint*>(NULL));
                for(size_t j=0, jend=vbInliers.size(); j<jend; j++)
                {
                    //保存内点
                    if(vbInliers[j])
                       vpMapPointMatches[j]=vvpMapPointMatches[i][j];
                }
                //通过上面求取的Sim3变换引导关键帧匹配，弥补词袋匹配中的漏匹配
                cv::Mat R = pSolver->GetEstimatedRotation();
                cv::Mat t = pSolver->GetEstimatedTranslation();
                const float s = pSolver->GetEstimatedScale();
                //通过Sim3变换，投影搜索pKF1的特征点在pKF2中的匹配，同理，投影搜索pKF2的特征点在pKF1中的匹配，只有互相都成功匹配的才认为是可靠的匹配
                matcher.SearchBySim3(mpCurrentKF, pKF, vpMapPointMatches, s, R, t, 7.5);
                //用新的匹配来优化 Sim3，只要有一个候选帧通过Sim3的求解与优化，就跳出停止对其它候选帧的判断
                //gScm：候选关键帧到当前帧的Sim3变换
                g2o::Sim3 gScm(Converter::toMatrix3d(R), Converter::toVector3d(t), s);
                //优化mpCurrentKF与pKF对应的MapPoints间的Sim3，得到优化后的量gScm
                const int nInliers = Optimizer::OptimizeSim3(mpCurrentKF, pKF, vpMapPointMatches, gScm, 10, mbFixScale);
                //如果优化成功，则停止while循环遍历闭环候选
                if(nInliers>=20)
                {
                    //为True时将不再进入while循环
                    bMatch = true;
                    //mpMatchedKF就是最终闭环检测出来与当前帧形成闭环的关键帧
                    mpMatchedKF = pKF;
                    //gSmw：从世界坐标系 w 到该候选帧 m 的Sim3变换，都在一个坐标系下，所以尺度 Scale=1
                    g2o::Sim3 gSmw(Converter::toMatrix3d(pKF->GetRotation()), Converter::toVector3d(pKF->GetTranslation()), 1.0);
                    //得到g2o优化后从世界坐标系到当前帧的Sim3变换
                    mg2oScw = gScm*gSmw;
                    mScw = Converter::toCvMat(mg2oScw);
                    mvpCurrentMatchedPoints = vpMapPointMatches;
                    //只要有一个候选帧通过Sim3的求解与优化，就跳出停止对其它候选帧的判断
                    break;
                }
            }
        }
    }
    //退出上面while循环的原因有两种,一种是求解到了bMatch置位后出的,另外一种是nCandidates耗尽为0
    if(!bMatch)
    {
        // 如果没有一个闭环匹配候选帧通过Sim3的求解与优化，清空mvpEnoughConsistentCandidates，这些候选关键帧以后都不会在再参加回环检测过程了
        for(int i=0; i<nInitialCandidates; i++)
             mvpEnoughConsistentCandidates[i]->SetErase();
        //当前关键帧也将不会再参加回环检测了
        mpCurrentKF->SetErase();
        return false;
    }
    //取出与当前帧闭环匹配上的关键帧及其共视关键帧，以及这些共视关键帧的地图点
    vector<KeyFrame*> vpLoopConnectedKFs = mpMatchedKF->GetVectorCovisibleKeyFrames();
    //包含闭环匹配关键帧本身,形成一个“闭环关键帧小组“
    vpLoopConnectedKFs.push_back(mpMatchedKF);
    mvpLoopMapPoints.clear();
    //遍历这个组中的每一个关键帧，将它们的地图点添加到mvpLoopMapPoints
    for(vector<KeyFrame*>::iterator vit=vpLoopConnectedKFs.begin(); vit!=vpLoopConnectedKFs.end(); vit++)
    {
        KeyFrame* pKF = *vit;
        vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();
        //遍历其中一个关键帧的所有有效地图点
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                //mnLoopPointForKF 用于标记，避免重复添加
                if(!pMP->isBad() && pMP->mnLoopPointForKF!=mpCurrentKF->mnId)
                {
                    mvpLoopMapPoints.push_back(pMP);
                    //标记一下
                    pMP->mnLoopPointForKF=mpCurrentKF->mnId;
                }
            }
        }
    }
    //将闭环关键帧及其连接关键帧的所有地图点投影到当前关键帧进行投影匹配
    matcher.SearchByProjection(mpCurrentKF, mScw, mvpLoopMapPoints, mvpCurrentMatchedPoints, 10);
    int nTotalMatches = 0;
    for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
    {
        if(mvpCurrentMatchedPoints[i])
            nTotalMatches++;
    }
    //统计当前帧与闭环关键帧的匹配地图点数目，超过40个说明成功闭环，否则失败
    if(nTotalMatches>=40)
    {
        //如果当前回环可靠,保留当前待闭环关键帧，其他闭环候选全部删掉以后不用了
        for(int i=0; i<nInitialCandidates; i++)
            if(mvpEnoughConsistentCandidates[i]!=mpMatchedKF)
                mvpEnoughConsistentCandidates[i]->SetErase();
        return true;
    }
    else
    {
        //闭环不可靠，闭环候选及当前待闭环帧全部删除
        for(int i=0; i<nInitialCandidates; i++)
            mvpEnoughConsistentCandidates[i]->SetErase();
        mpCurrentKF->SetErase();
        return false;
    }
}

void LoopClosing::CorrectLoop()
{
    cout << "Loop detected!" << endl;
    //请求局部地图停止，防止在回环矫正时局部地图线程中InsertKeyFrame函数插入新的关键帧
    mpLocalMapper->RequestStop();
    //如果有全局BA在运行，终止掉，迎接新的全局BA
    if(isRunningGBA())
    {
        unique_lock<mutex> lock(mMutexGBA);
        mbStopGBA = true;
        //记录全局BA次数
        mnFullBAIdx++;
        if(mpThreadGBA)
        {
            //停止全局BA线程
            mpThreadGBA->detach();
            delete mpThreadGBA;
        }
    }
    //一直等到局部地图线程结束再继续
    while(!mpLocalMapper->isStopped())
    {
        usleep(1000);
    }
    //根据共视关系更新当前关键帧与其它关键帧之间的连接关系，因为之前闭环检测、计算Sim3中改变了该关键帧的地图点，所以需要更新
    mpCurrentKF->UpdateConnections();
    //取出当前关键帧及其共视关键帧，称为“当前关键帧组”
    mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
    mvpCurrentConnectedKFs.push_back(mpCurrentKF);
    //CorrectedSim3：存放闭环g2o优化后当前关键帧的共视关键帧的世界坐标系下Sim3变换
    //NonCorrectedSim3：存放没有矫正的当前关键帧的共视关键帧的世界坐标系下Sim3变换
    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
    //先将mpCurrentKF的Sim3变换存入，认为是准的，所以固定不动
    CorrectedSim3[mpCurrentKF] = mg2oScw;
    //当前关键帧到世界坐标系下的变换矩阵
    cv::Mat Twc = mpCurrentKF->GetPoseInverse();
    {
        //给地图上锁
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
        //遍历"当前关键帧组"
        for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKFi = *vit;
            //获取世界坐标系相对共视关键帧的位姿
            cv::Mat Tiw = pKFi->GetPose();
            //跳过当前关键帧，以当前关键帧是参考基准，调整共视关键帧的位姿
            if(pKFi!=mpCurrentKF)
            {
                //得到当前关键帧mpCurrentKF到其共视关键帧pKFi的相对变换
                cv::Mat Tic = Tiw*Twc;
                cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);
                cv::Mat tic = Tic.rowRange(0,3).col(3);
                //生成当前关键帧mpCurrentKF到其共视关键帧pKFi的Sim3相对变换
                g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric), Converter::toVector3d(tic), 1.0);
                //当前帧的位姿固定不动，其它的关键帧根据相对关系得到Sim3调整的位姿
                g2o::Sim3 g2oCorrectedSiw = g2oSic*mg2oScw;
                //存放闭环g2o优化后当前关键帧的共视关键帧的Sim3位姿
                CorrectedSim3[pKFi] = g2oCorrectedSiw;
            }
            cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);
            cv::Mat tiw = Tiw.rowRange(0,3).col(3);
            g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw), Converter::toVector3d(tiw), 1.0);
            //存放没有矫正的当前关键帧的共视关键帧的Sim3变换
            NonCorrectedSim3[pKFi] = g2oSiw;
        }
        //得到矫正的当前关键帧的共视关键帧位姿后，修正这些共视关键帧的地图点
        //遍历待矫正的共视关键帧（不包括当前关键帧）
        for(KeyFrameAndPose::iterator mit=CorrectedSim3.begin(), mend=CorrectedSim3.end(); mit!=mend; mit++)
        {
            //取出当前关键帧连接关键帧
            KeyFrame* pKFi = mit->first;
            //取出经过位姿传播后的Sim3变换
            g2o::Sim3 g2oCorrectedSiw = mit->second;
            g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();
            //取出未经过位姿传播的Sim3变换
            g2o::Sim3 g2oSiw = NonCorrectedSim3[pKFi];
            vector<MapPoint*> vpMPsi = pKFi->GetMapPointMatches();
            //遍历待矫正共视关键帧中的每一个地图点
            for(size_t iMP=0, endMPi = vpMPsi.size(); iMP<endMPi; iMP++)
            {
                MapPoint* pMPi = vpMPsi[iMP];
                if(!pMPi)
                    continue;
                if(pMPi->isBad())
                    continue;
                //标记，防止重复矫正
                if(pMPi->mnCorrectedByKF==mpCurrentKF->mnId)
                    continue;
                //将该未校正的eigP3Dw先从世界坐标系映射到未校正的pKFi相机坐标系，然后再反映射到校正后的世界坐标系下
                cv::Mat P3Dw = pMPi->GetWorldPos();
                //计算地图点矫正后在世界坐标系下的坐标cvCorrectedP3Dw
                Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
                Eigen::Matrix<double,3,1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw));
                cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
                pMPi->SetWorldPos(cvCorrectedP3Dw);
                //记录矫正该地图点的关键帧id，防止重复
                pMPi->mnCorrectedByKF = mpCurrentKF->mnId;
                //记录该地图点所在的关键帧id
                pMPi->mnCorrectedReference = pKFi->mnId;
                //因为地图点更新了，需要更新其平均观测方向以及观测距离范围
                pMPi->UpdateNormalAndDepth();
            }
            //将共视关键帧的Sim3转换为SE3，根据更新的Sim3，更新关键帧的位姿
            Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
            Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
            double s = g2oCorrectedSiw.scale();
            //平移向量中包含有尺度信息，还需要用尺度归一化
            eigt *=(1./s);
            cv::Mat correctedTiw = Converter::toCvSE3(eigR, eigt);
            //设置矫正后的新的pose
            pKFi->SetPose(correctedTiw);
            //根据共视关系更新当前帧与其它关键帧之间的连接
            pKFi->UpdateConnections();
        }
        //检查当前帧的地图点与经过闭环匹配后该帧的地图点是否存在冲突，对冲突的进行替换或填补
        for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
        {
            //mvpCurrentMatchedPoints是当前关键帧和闭环关键帧组的所有地图点进行投影得到的匹配点
            if(mvpCurrentMatchedPoints[i])
            {
                //取出同一个索引对应的两种地图点，决定是否要替换
                MapPoint* pLoopMP = mvpCurrentMatchedPoints[i];
                MapPoint* pCurMP = mpCurrentKF->GetMapPoint(i);
                //如果有重复的MapPoint，则用匹配的地图点代替现有的，因为匹配的地图点是经过一系列操作后比较精确的，现有的地图点很可能有累计误差
                if(pCurMP)
                    pCurMP->Replace(pLoopMP);
                else
                {
                    //如果当前帧没有该MapPoint，则直接添加
                    mpCurrentKF->AddMapPoint(pLoopMP,i);
                    pLoopMP->AddObservation(mpCurrentKF,i);
                    pLoopMP->ComputeDistinctiveDescriptors();
                }
            }
        }
    }
    //将闭环相连关键帧组mvpLoopMapPoints投影到当前关键帧组中，进行匹配、融合、新增或替换当前关键帧组中KF的地图点
    SearchAndFuse(CorrectedSim3);
    //更新当前关键帧组之间的两级共视相连关系，得到因闭环时地图点融合而新得到的连接关系
    map<KeyFrame*, set<KeyFrame*> > LoopConnections;
    //遍历当前帧相连关键帧组（一级相连）
    for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        //得到与当前帧相连关键帧的相连关键帧（二级相连）
        vector<KeyFrame*> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();
        //更新一级相连关键帧的连接关系(会把当前关键帧添加进去，因为地图点已经更新和替换了)
        pKFi->UpdateConnections();
        //取出该帧更新后的连接关系
        LoopConnections[pKFi] = pKFi->GetConnectedKeyFrames();
        //从连接关系中去除闭环之前的二级连接关系，剩下的连接就是由闭环得到的连接关系
        for(vector<KeyFrame*>::iterator vit_prev=vpPreviousNeighbors.begin(), vend_prev=vpPreviousNeighbors.end(); vit_prev!=vend_prev; vit_prev++)
        {
            LoopConnections[pKFi].erase(*vit_prev);
        }
        //从连接关系中去除闭环之前的一级连接关系，剩下的连接就是由闭环得到的连接关系
        for(vector<KeyFrame*>::iterator vit2=mvpCurrentConnectedKFs.begin(), vend2=mvpCurrentConnectedKFs.end(); vit2!=vend2; vit2++)
        {
            LoopConnections[pKFi].erase(*vit2);
        }
    }
    //进行本质图优化，优化本质图中所有关键帧的位姿和地图点
    Optimizer::OptimizeEssentialGraph(mpMap, mpMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections, mbFixScale);
    mpMap->InformNewBigChange();
    //添加当前帧与闭环匹配帧之间的边（这个连接关系不优化）
    mpMatchedKF->AddLoopEdge(mpCurrentKF);
    mpCurrentKF->AddLoopEdge(mpMatchedKF);
    //新建一个线程用于全局BA优化
    mbRunningGBA = true;
    mbFinishedGBA = false;
    mbStopGBA = false;
    //OptimizeEssentialGraph只是优化了一些主要关键帧的位姿，这里进行全局BA可以全局优化所有位姿和MapPoints
    mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment, this, mpCurrentKF->mnId);
    mpLocalMapper->Release();    
    mLastLoopKFid = mpCurrentKF->mnId;   
}

//将闭环相连关键帧组mvpLoopMapPoints投影到当前关键帧组中进行匹配，新增或替换当前关键帧组中KF的地图点
void LoopClosing::SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap)
{
    ORBmatcher matcher(0.8);
    for(KeyFrameAndPose::const_iterator mit=CorrectedPosesMap.begin(), mend=CorrectedPosesMap.end(); mit!=mend;mit++)
    {
        //取出对地图点位置和关键帧位姿矫正过的关键帧
        KeyFrame* pKF = mit->first;
        //取出位姿
        g2o::Sim3 g2oScw = mit->second;
        cv::Mat cvScw = Converter::toCvMat(g2oScw);
        //按照闭环相连关键帧组大小初始化容器
        vector<MapPoint*> vpReplacePoints(mvpLoopMapPoints.size(), static_cast<MapPoint*>(NULL));
        //融合
        matcher.Fuse(pKF, cvScw, mvpLoopMapPoints, 4, vpReplacePoints);
        //上锁
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
        const int nLP = mvpLoopMapPoints.size();
        //替换闭环相连关键帧组的地图点
        for(int i=0; i<nLP;i++)
        {
            MapPoint* pRep = vpReplacePoints[i];
            if(pRep)
            {
                pRep->Replace(mvpLoopMapPoints[i]);
            }
        }
    }
}


void LoopClosing::RequestReset()
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
        usleep(5000);
    }
}

void LoopClosing::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        mlpLoopKeyFrameQueue.clear();
        mLastLoopKFid=0;
        mbResetRequested=false;
    }
}

void LoopClosing::RunGlobalBundleAdjustment(unsigned long nLoopKF)
{
    cout << "Starting Global Bundle Adjustment" << endl;
    //记录Global BA（GBA）（全局BA）已经迭代次数,用来检查全局BA过程是否是因为意外结束的
    int idx =  mnFullBAIdx;
    Optimizer::GlobalBundleAdjustemnt(mpMap, 10, &mbStopGBA, nLoopKF, false);
    {
        unique_lock<mutex> lock(mMutexGBA);
        //如果全局BA过程是因为意外结束的,那么直接退出GBA
        if(idx!=mnFullBAIdx)
            return;
        //如果当前GBA没有中断请求，更新位姿和地图点
        if(!mbStopGBA)
        {
            cout << "Global Bundle Adjustment finished" << endl;
            cout << "Updating map ..." << endl;
            mpLocalMapper->RequestStop();
            //等待直到local mapping结束才会继续后续操作
            while(!mpLocalMapper->isStopped() && !mpLocalMapper->isFinished())
            {
                usleep(1000);
            }
            unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
            //从第一个关键帧开始矫正关键帧，没有进行全局优化时mvpKeyFrameOrigins只保存了第一个初始化关键帧
            list<KeyFrame*> lpKFtoCheck(mpMap->mvpKeyFrameOrigins.begin(), mpMap->mvpKeyFrameOrigins.end());
            //遍历并更新全局地图中的所有拓展树中的关键帧
            while(!lpKFtoCheck.empty())
            {
                //从第一帧开始遍历
                KeyFrame* pKF = lpKFtoCheck.front();
                const set<KeyFrame*> sChilds = pKF->GetChilds();
                cv::Mat Twc = pKF->GetPoseInverse();
                //遍历子关键帧
                for(set<KeyFrame*>::const_iterator sit=sChilds.begin();sit!=sChilds.end();sit++)
                {
                    KeyFrame* pChild = *sit;
                    //mnBAGlobalForKF记录是由于哪个闭环匹配关键帧触发的全局BA，并且表示已经经过了GBA的优化
                    //如果没有经过优化，进入分支
                    if(pChild->mnBAGlobalForKF!=nLoopKF)
                    {
                        //计算从父关键帧到当前子关键帧的位姿变换
                        cv::Mat Tchildc = pChild->GetPose()*Twc;
                        //再利用优化后的父关键帧的位姿，把子关键帧位姿转换到世界坐标系下，相当于更新了子关键帧优化后的位姿
                        pChild->mTcwGBA = Tchildc*pKF->mTcwGBA;//*Tcorc*pKF->mTcwGBA;
                        //间接优化子关键帧后，也给子关键帧标记上
                        pChild->mnBAGlobalForKF=nLoopKF;
                    }
                    //子关键帧更新矫正完毕然后存入
                    lpKFtoCheck.push_back(pChild);
                }
                //记录未矫正的关键帧的位姿
                pKF->mTcwBefGBA = pKF->GetPose();
                //把矫正后的关键帧的位姿，赋值给关键帧
                pKF->SetPose(pKF->mTcwGBA);
                //取出当前帧，处理下一帧
                lpKFtoCheck.pop_front();
            }
            // Correct MapPoints
            const vector<MapPoint*> vpMPs = mpMap->GetAllMapPoints();
            //遍历每一个地图点并用更新的关键帧位姿来更新地图点位置
            for(size_t i=0; i<vpMPs.size(); i++)
            {
                MapPoint* pMP = vpMPs[i];
                if(pMP->isBad())
                    continue;
                //如果这个地图点直接参与到了全局BA优化的过程,那么就直接重新设置其位姿即可
                if(pMP->mnBAGlobalForKF==nLoopKF)
                {
                    pMP->SetWorldPos(pMP->mPosGBA);
                }
                else
                {
                    //如这个地图点并没有直接参与到全局BA优化的过程中,那么就使用其参考关键帧的新位姿来优化自己的坐标
                    KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();
                    //如果参考关键帧并没有经过此次全局BA优化，就跳过
                    if(pRefKF->mnBAGlobalForKF!=nLoopKF)
                        continue;
                    //取出参考关键帧未经优化的位姿的旋转和平移部分
                    cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0,3).colRange(0,3);
                    cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0,3).col(3);
                    //把地图点位姿转换到其参考关键帧相机坐标系下
                    cv::Mat Xc = Rcw*pMP->GetWorldPos() + tcw;
                    //然后使用已经纠正过的参考关键帧的位姿，再将该地图点变换到世界坐标系下
                    cv::Mat Twc = pRefKF->GetPoseInverse();
                    cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);
                    cv::Mat twc = Twc.rowRange(0,3).col(3);
                    //赋值给地图点
                    pMP->SetWorldPos(Rwc*Xc+twc);
                }
            }            
            mpMap->InformNewBigChange();
            // 释放
            mpLocalMapper->Release();
            cout << "Map updated!" << endl;
        }
        mbFinishedGBA = true;  //完成BA
        mbRunningGBA = false;  //没有进行BA优化了
    }
}

void LoopClosing::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LoopClosing::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LoopClosing::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool LoopClosing::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}


} //namespace ORB_SLAM
