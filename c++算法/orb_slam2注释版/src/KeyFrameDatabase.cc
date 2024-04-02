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

#include "KeyFrameDatabase.h"

#include "KeyFrame.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"

#include<mutex>

using namespace std;

namespace ORB_SLAM2
{

KeyFrameDatabase::KeyFrameDatabase (const ORBVocabulary &voc):
    mpVoc(&voc)
{
    //根据叶子节点数量，resize逆向索引向量
    mvInvertedFile.resize(voc.size());
}


void KeyFrameDatabase::add(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutex);

    for(DBoW2::BowVector::const_iterator vit= pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit!=vend; vit++)
        mvInvertedFile[vit->first].push_back(pKF);
}

void KeyFrameDatabase::erase(KeyFrame* pKF)
{
    unique_lock<mutex> lock(mMutex);

    // Erase elements in the Inverse File for the entry
    for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit!=vend; vit++)
    {
        // List of keyframes that share the word
        list<KeyFrame*> &lKFs =   mvInvertedFile[vit->first];

        for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
        {
            if(pKF==*lit)
            {
                lKFs.erase(lit);
                break;
            }
        }
    }
}

void KeyFrameDatabase::clear()
{
    mvInvertedFile.clear();
    mvInvertedFile.resize(mpVoc->size());
}

//检测回环候选关键帧
vector<KeyFrame*> KeyFrameDatabase::DetectLoopCandidates(KeyFrame* pKF, float minScore)
{
    //获取发生共视的关键帧
    set<KeyFrame*> spConnectedKeyFrames = pKF->GetConnectedKeyFrames();
    list<KeyFrame*> lKFsSharingWords;
    {
        unique_lock<mutex> lock(mMutex);
        //遍历当前关键帧的mBowVec，包含该帧单词id和对应单词权重
        for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit != vend; vit++)
        {
            //根据单词id找出逆排索引中存放的，包含该单词的所有关键帧
            list<KeyFrame*> &lKFs = mvInvertedFile[vit->first];
            //遍历共享单词的关键帧
            for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                KeyFrame* pKFi=*lit;
                //防止对同一帧重复发生闭环检测
                if(pKFi->mnLoopQuery!=pKF->mnId)
                {
                    pKFi->mnLoopWords=0;
                    //如果该共享单词关键帧与当前帧没有共视关系
                    if(!spConnectedKeyFrames.count(pKFi))
                    {
                        //记录该关键帧与当前关键帧发生过回环检测，不再进行回环检测
                        pKFi->mnLoopQuery=pKF->mnId;
                        lKFsSharingWords.push_back(pKFi);
                    }
                }
                //记录共享单词的关键帧与当前帧共享的单词数
                pKFi->mnLoopWords++;
            }
        }
    }
    //如果没有共视关系但是有共享单词的关键帧不存在，返回空向量
    if(lKFsSharingWords.empty())
        return vector<KeyFrame*>();

    list<pair<float,KeyFrame*> > lScoreAndMatch;
    int maxCommonWords=0;
    //遍历没有发生共视关系但是有共享单词的关键帧
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        //找出最大的共享单词数
        if((*lit)->mnLoopWords>maxCommonWords)
            maxCommonWords=(*lit)->mnLoopWords;
    }
    //初始化最小共享单词数，取最大数的0.8
    int minCommonWords = maxCommonWords*0.8f;
    int nscores=0;
    //再次遍历没有发生共视关系但是有共享单词的关键帧，计算词向量匹配得分
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        //记录大于最小共享单词数的关键帧，并且与当前关键帧的相似度大于阈值的关键帧
        if(pKFi->mnLoopWords>minCommonWords)
        {
            nscores++;
            float si = mpVoc->score(pKF->mBowVec, pKFi->mBowVec);
            pKFi->mLoopScore = si;
            if(si>=minScore)
                lScoreAndMatch.push_back(make_pair(si, pKFi));
        }
    }
    //如果大于最小共享单词数并且相似度大于阈值的关键帧为空，返回空向量
    if(lScoreAndMatch.empty())
        return vector<KeyFrame*>();

    list<pair<float,KeyFrame*> > lAccScoreAndMatch;
    float bestAccScore = minScore;
    //遍历上边筛选出的关键帧
    for(list<pair<float,KeyFrame*> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        KeyFrame* pKFi = it->second;
        //取出前10共视关键帧
        vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first;
        float accScore = it->first;
        KeyFrame* pBestKF = pKFi;
        for(vector<KeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKF2 = *vit;
            //如果通过了上边的检测，并且共享单词大于最小单词数
            if(pKF2->mnLoopQuery==pKF->mnId && pKF2->mnLoopWords>minCommonWords)
            {
                //累加得分
                accScore += pKF2->mLoopScore;
                //记录最高得分和对应关键帧
                if(pKF2->mLoopScore>bestScore)
                {
                    pBestKF=pKF2;
                    bestScore = pKF2->mLoopScore;
                }
            }
        }
        //记录共视帧组的类加得分和最高的分关键帧
        lAccScoreAndMatch.push_back(make_pair(accScore, pBestKF));
        //记录最大累加得分
        if(accScore>bestAccScore)
            bestAccScore=accScore;
    }
    //取出最大累加得分的0.75作为最小分阈值
    float minScoreToRetain = 0.75f*bestAccScore;

    set<KeyFrame*> spAlreadyAddedKF;
    vector<KeyFrame*> vpLoopCandidates;
    vpLoopCandidates.reserve(lAccScoreAndMatch.size());
    //遍历上边筛选出的关键帧组中的最高得分关键帧
    for(list<pair<float,KeyFrame*> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        //如果大于阈值（前百分之75帧组的最高得分关键帧）
        if(it->first>minScoreToRetain)
        {
            KeyFrame* pKFi = it->second;
            //防止重复添加关键帧（关键帧组很有可能重叠，各组的最高分关键帧很有可能是同一帧）
            if(!spAlreadyAddedKF.count(pKFi))
            {
                //添加到候选关键帧容器
                vpLoopCandidates.push_back(pKFi);
                //该容器记录关键帧，快速检测
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }
    return vpLoopCandidates;
}

//用词袋找到与当前帧相似的候选关键帧
vector<KeyFrame*> KeyFrameDatabase::DetectRelocalizationCandidates(Frame *F)
{
    //声明存放共享单词的关键帧的列表
    list<KeyFrame*> lKFsSharingWords;
    //找出和当前帧具有公共单词(叶子节点)的所有关键帧
    {
        unique_lock<mutex> lock(mMutex);
        //mBowVec内部存储的是std::map<WordId, WordValue>，WordId和WordValue表示Word对应叶子节点的id和权重
        for(DBoW2::BowVector::const_iterator vit=F->mBowVec.begin(), vend=F->mBowVec.end(); vit != vend; vit++)
        {
            //根据逆向索引，提取所有包含该wordid的所有KeyFrame
            list<KeyFrame*> &lKFs =   mvInvertedFile[vit->first];
            //遍历关键帧
            for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                KeyFrame* pKFi=*lit;
                //pKFi->mnRelocQuery起标记作用，是为了防止重复选取(其他叶子节点的逆向索引可能也包含同样的关键帧)
                if(pKFi->mnRelocQuery!=F->mnId)
                {
                    pKFi->mnRelocWords=0;
                    pKFi->mnRelocQuery=F->mnId;  //重置关键帧的重定位请求帧id
                    lKFsSharingWords.push_back(pKFi); //放入容器
                }
                pKFi->mnRelocWords++;//记录该关键帧与普通帧F共享的单词数量
            }
        }
    }
    //如果没有共享单词的关键帧，返回空向量
    if(lKFsSharingWords.empty())
        return vector<KeyFrame*>();
    //统计上述关键帧中与当前帧F具有共同单词最多的单词数maxCommonWords
    int maxCommonWords=0;
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        if((*lit)->mnRelocWords>maxCommonWords)
            maxCommonWords=(*lit)->mnRelocWords;
    }
    //最小共享单词数为最大共享单词数目的0.8倍
    int minCommonWords = maxCommonWords*0.8f;
    list<pair<float,KeyFrame*> > lScoreAndMatch;
    int nscores=0;
    //遍历上述关键帧
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        //挑选出共有单词数大于minCommonWords的
        if(pKFi->mnRelocWords>minCommonWords)
        {
            nscores++;
            //计算关键帧和当前帧单词匹配得分存入lScoreAndMatch，把具有相同单词对应的权重相乘再累加
            float si = mpVoc->score(F->mBowVec,pKFi->mBowVec);
            pKFi->mRelocScore=si;  //在关键帧中记录重定位得分
            lScoreAndMatch.push_back(make_pair(si, pKFi));
        }
    }
    //如果得分和匹配向量为空，返回空向量
    if(lScoreAndMatch.empty())
        return vector<KeyFrame*>();
    //共视组累计得分和组内得分最高关键帧
    list<pair<float,KeyFrame*> > lAccScoreAndMatch;
    float bestAccScore = 0;
    //计算lScoreAndMatch中每个关键帧的共视关键帧组的总得分，得到最高组得分bestAccScore，并以此决定阈值2
    for(list<pair<float,KeyFrame*> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        KeyFrame* pKFi = it->second;
        //计算与关键帧共视程度最高的前十个关键帧，存入vpNeighs
        vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);
        //取出关键帧得分
        float bestScore = it->first; //该变量记录共视组单个最高得分
        float accScore = bestScore; //该变量记录共视组累计得分
        KeyFrame* pBestKF = pKFi;
        //遍历共视组关键帧
        for(vector<KeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKF2 = *vit;
            //如果该共视关键帧没有被当前帧通过前面进行词袋匹配匹配到，略过该帧
            if(pKF2->mnRelocQuery!=F->mnId)
                continue;
            //如果该共视帧之前被匹配到过，累加之前的重定位得分
            accScore+=pKF2->mRelocScore;
            //记录单个帧的最高得分，和对应的关键帧
            if(pKF2->mRelocScore>bestScore)
            {
                pBestKF=pKF2;
                bestScore = pKF2->mRelocScore;
            }
        }
        //存放累计得分信息和最佳匹配关键帧
        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
        if(accScore>bestAccScore)
            bestAccScore=accScore;
    }
    //得到所有组中总得分大于阈值的，组内得分最高的关键帧，作为候选关键帧组，阈值最高得分的0.75倍
    float minScoreToRetain = 0.75f*bestAccScore;
    set<KeyFrame*> spAlreadyAddedKF;
    vector<KeyFrame*> vpRelocCandidates;
    vpRelocCandidates.reserve(lAccScoreAndMatch.size());
    //遍历总得分匹配帧
    for(list<pair<float,KeyFrame*> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        const float &si = it->first;
        //只返回组累计得分大于阈值的组中分数最高的关键帧
        if(si>minScoreToRetain)
        {
            KeyFrame* pKFi = it->second;
            //放置插入重复关键帧
            if(!spAlreadyAddedKF.count(pKFi))
            {
                vpRelocCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }
    return vpRelocCandidates;
}

} //namespace ORB_SLAM
