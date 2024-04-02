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


#include "Tracking.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"ORBmatcher.h"
#include"FrameDrawer.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"

#include"Optimizer.h"
#include"PnPsolver.h"

#include<iostream>

#include<mutex>


using namespace std;

namespace ORB_SLAM2
{

Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor):
    mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
    mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys), mpViewer(NULL),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0)
{
    //从配置文件加载相机参数
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];
    //设置线程相机参数
    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);
    //从配置文件加载相机畸变参数
    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    //设置线程畸变参数
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);
    mbf = fSettings["Camera.bf"];
    //读取fps值赋值给mMaxFrames
    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;
    //插入关键帧和重定位判断的最大最小帧数值
    mMinFrames = 0;
    mMaxFrames = fps;
    //输出上述信息
    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if(DistCoef.rows==5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;
    //读取图像rgb顺序
    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;
    //输出图像rgb顺序信息
    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;
    //加载ORB特征点提取器的相关参数
    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];
    //初始化orb特征提取器
    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    //如果是双目相机，初始化右目特征提取器
    if(sensor==System::STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    //如果是单目相机，初始化Ini特征提取器(提取2倍数量特征点)
    if(sensor==System::MONOCULAR)
        mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    //输出特征提取器相关参数
    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;
    //如果是双目或者深度相机，设置mThDepth变量
    if(sensor==System::STEREO || sensor==System::RGBD)
    {
        mThDepth = mbf*(float)fSettings["ThDepth"]/fx;
        cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
    }
    //如果是深度相机，设置mDepthMapFactor变量
    if(sensor==System::RGBD)
    {
        mDepthMapFactor = fSettings["DepthMapFactor"];
        if(fabs(mDepthMapFactor)<1e-5)
            mDepthMapFactor=1;
        else
            mDepthMapFactor = 1.0f/mDepthMapFactor;
    }
}

//初始化其他三个线程
void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}
void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}
void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}

//双目相机track
cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp)
{
    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGB2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGR2GRAY);
        }
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGBA2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGRA2GRAY);
        }
    }
    mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
    Track();
    return mCurrentFrame.mTcw.clone();
}

//深度相机track
cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp)
{
    mImGray = imRGB;
    cv::Mat imDepth = imD;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }
    if((fabs(mDepthMapFactor-1.0f)>1e-5) || imDepth.type()!=CV_32F)
        imDepth.convertTo(imDepth,CV_32F,mDepthMapFactor);
    mCurrentFrame = Frame(mImGray,imDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
    Track();
    return mCurrentFrame.mTcw.clone();
}

//单目相机的跟踪
cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
{
    mImGray = im;
    //判断通道数，转化为灰度图
    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    //如果是4通道图像转化为对应灰度图
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }
    //根据线程状态，给当前帧变量赋值（初始化完成后使用mpORBextractorLeft特征提取器）
    if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
    else
        mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
    //跟踪
    Track();
    //返回相机相对世界坐标系的坐标系 
    return mCurrentFrame.mTcw.clone();
}

//tacking线程主函数
void Tracking::Track()
{
    //tracking线程接收第一帧图像后，状态置为NOT_INITIALIZED
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }
    //上重置变量，记录上一轮track完成后的状态
    mLastProcessedState=mState;
    //地图加锁
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
    //没有初始化过就初始化
    if(mState==NOT_INITIALIZED)
    {
        //根据相机类型选择初始化类型
        if(mSensor==System::STEREO || mSensor==System::RGBD)
            StereoInitialization();
        else
            MonocularInitialization();
        mpFrameDrawer->Update(this);
        //初始化函数会重置mState变量，如果初始化结果不好就返回
        if(mState!=OK)
            return;
    }
    //系统初始化成功跟踪帧
    else
    {
        bool bOK;
        //建图跟踪模式
        if(!mbOnlyTracking)
        {
            if(mState==OK)
            {
                //局部建图线程则可能会对原有的地图点进行替换，在这里进行检查
                CheckReplacedInLastFrame();
                //如果运动模型为空(刚初始化，或者跟踪失败)，或者当前帧紧紧地跟着在重定位的帧的后面，我们将跟踪参考帧来恢复位姿
                if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                {
                    bOK = TrackReferenceKeyFrame();
                }
                //通过运动模型进行跟踪
                else
                {
                    bOK = TrackWithMotionModel();
                    //运动模型跟踪失败，跟踪参考关键帧
                    if(!bOK)
                        bOK = TrackReferenceKeyFrame();
                }
            }
            //如果上一帧未跟踪到，跟踪状态lost，重定位
            else
            {
                bOK = Relocalization();
            }
        }
        //使用定位模式
        else 
        {
            //如果跟踪状态lost，重定位
            if(mState==LOST)
            {
                bOK = Relocalization();
            }
            else
            {
                //mbVO为false表示此帧匹配了很多的MapPoints，跟踪正常
                //mbVO为true表明此帧匹配了很少的MapPoints，少于10个，
                //mbVO初始值为false，刚初始化成功会进入TrackReferenceKeyFrame函数
                if(!mbVO)
                {
                    //如果运动模型输出了结果
                    if(!mVelocity.empty())
                    {
                        bOK = TrackWithMotionModel();
                    }
                    //如果运动模型没输出结果,就通过参考关键帧来定位
                    else
                    {
                        bOK = TrackReferenceKeyFrame();
                    }
                }
                //上一帧匹配了很少地图点，这里既做跟踪又做重定位
                else
                {
                    //通过重定位和运动模型计算两种帧间位姿，如果重定位成功我们选择重定位结果，否则保持视觉里程计结果
                    bool bOKMM = false;
                    bool bOKReloc = false;
                    //运动模型中构造的地图点
                    vector<MapPoint*> vpMPsMM;
                    //运动模型中发现的外点
                    vector<bool> vbOutMM;
                    //运动模型得到的位姿
                    cv::Mat TcwMM;
                    if(!mVelocity.empty())
                    {
                        bOKMM = TrackWithMotionModel();
                        //把运动模型得到的地图点、外点和位姿信息进行保存，因为重定位会改变这些变量
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM = mCurrentFrame.mTcw.clone();
                    }
                    bOKReloc = Relocalization();
                    //运动模型成功、重定位失败，使用运动模型结果
                    if(bOKMM && !bOKReloc)
                    {
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;
                        //如果当前帧匹配的3D点很少，增加当前可视地图点的被观测次数
                        if(mbVO)
                        {
                            for(int i =0; i<mCurrentFrame.N; i++)
                            {
                                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                {
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                }
                            }
                        }
                    }
                    }
                    //只要重定位成功整个跟踪过程正常进行
                    else if(bOKReloc)
                    {
                        mbVO = false;
                    }
                    //有一个成功，状态就置为true
                    bOK = bOKReloc || bOKMM;
                }
            }
        }
        //当前帧的参考关键帧置为tracking线程的参考关键帧
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
        //如果是建图模式，跟踪局部地图，局部地图会增加更多地图点，也可能更新当前帧的参考关键帧
        if(!mbOnlyTracking)
        {
            if(bOK)
                bOK = TrackLocalMap();
        }
        //定位模式
        else
        {
            // mbVO为ture表示只有少量匹配的地图点，不跟踪局部地图，除非系统进行了重定位
            if(bOK && !mbVO)
                bOK = TrackLocalMap();
        }
        //更新跟踪状态
        if(bOK)
            mState = OK;
        else
            mState=LOST;
        //显示
        mpFrameDrawer->Update(this);
        //如果跟踪状态较好，更新运动模型并且判断是否插入关键帧
        if(bOK)
        {
            //更新运动模型
            if(!mLastFrame.mTcw.empty())
            {
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;
            }
            else
                mVelocity = cv::Mat();
            //显示
            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);
            //清除观测不到的地图点
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    if(pMP->Observations()<1) //Observations返回Obs值，该值表示能观测到该地图点的关键帧数量。如果小于1，在当前帧删除该地图点
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }
            //删除临时地图点(上面在当前帧中将这些MapPoints剔除，这里完全删除地图点)
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend = mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();
            //检查是否插入关键帧
            if(NeedNewKeyFrame())
                CreateNewKeyFrame();
            //在当前帧中删除那些在bundle adjustment中检测为outlier的地图点
            for(int i=0; i<mCurrentFrame.N;i++)
            {                
                //先执行判断, 因为前面的操作中可能删除了其中的地图点
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
        }
        //如果初始化不久，相机就跟踪失败，那么重置系统
        if(mState==LOST)
        {
            if(mpMap->KeyFramesInMap()<=5)
            {
                cout << "Track lost soon after initialisation, reseting..." << endl;
                mpSystem->Reset();
                return;
            }
        }
        //确保设置参考关键帧，当前帧的参考关键帧置为tracking线程的参考关键帧
        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;
        //重置上一帧变量
        mLastFrame = Frame(mCurrentFrame);
    }
    //存储帧位姿信息，以便后面恢复相机的完整轨迹
    if(!mCurrentFrame.mTcw.empty())
    {
        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
        mlbLost.push_back(mState==LOST);
    }
    else
    {
        //跟踪失败的情况也要存储相关信息
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferences.push_back(mlpReferences.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState==LOST);
    }
}

//双目相机初始化
void Tracking::StereoInitialization()
{
    if(mCurrentFrame.N>500)
    {
        // Set Frame pose to the origin
        mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
        // Create KeyFrame
        KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);
        // Insert KeyFrame in the map
        mpMap->AddKeyFrame(pKFini);
        // Create MapPoints and asscoiate to KeyFrame
        for(int i=0; i<mCurrentFrame.N;i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpMap);
                pNewMP->AddObservation(pKFini,i);
                pKFini->AddMapPoint(pNewMP,i);
                pNewMP->ComputeDistinctiveDescriptors();
                pNewMP->UpdateNormalAndDepth();
                mpMap->AddMapPoint(pNewMP);

                mCurrentFrame.mvpMapPoints[i]=pNewMP;
            }
        }
        cout << "New map created with " << mpMap->MapPointsInMap() << " points" << endl;
        mpLocalMapper->InsertKeyFrame(pKFini);
        mLastFrame = Frame(mCurrentFrame);
        mnLastKeyFrameId=mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;

        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints=mpMap->GetAllMapPoints();
        mpReferenceKF = pKFini;
        mCurrentFrame.mpReferenceKF = pKFini;

        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);
        mpMap->mvpKeyFrameOrigins.push_back(pKFini);
        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);
        mState=OK;
    }
}

//单目相机初始化函数，总共需要两帧合格的图像进行初始化
void Tracking::MonocularInitialization()
{
    //如果初始化器指针为空，没有合格的第一帧
    if(!mpInitializer)
    {
        //追踪线程的当前帧mCurrentFrame的特征点数量大于100
        if(mCurrentFrame.mvKeys.size()>100)
        {
            //给初始化帧赋值
            mInitialFrame = Frame(mCurrentFrame);
            //更新线程上一帧
            mLastFrame = Frame(mCurrentFrame);
            //用变量mvbPrevMatched记录"上一帧"所有特征点
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;
            //这里是多余的操作
            if(mpInitializer)
                delete mpInitializer;
            //用当前帧创建初始化器
            mpInitializer = new Initializer(mCurrentFrame,1.0,200); //1表示Initializer中的sigma值，200表示迭代次数
            //向量mvIniMatches存储匹配的点的id，这里初始化为-1表示没有任何匹配
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }
    }
    //如果初始化器指针非空，已经有合格的第一帧
    else
    {
        //如果当前帧特征点数太少把初始化器指针置为空，下一帧又会进入上面的分支重新初始化
        if((int)mCurrentFrame.mvKeys.size()<=100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }
        //查找连续两帧初始化帧的匹配点
        ORBmatcher matcher(0.9,true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);
        //如果初始化的两帧之间的匹配点太少，重新初始化
        if(nmatches<100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            return;
        }
        //声明位姿变量
        cv::Mat Rcw; //相机Rotation
        cv::Mat tcw; //相机Translation
        vector<bool> vbTriangulated; //三角化是否成功标志位(mvIniMatches)
        //使用初始化器进行初始化
        if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
        {
            //初始化成功后删除那些无法进行三角化的匹配点(mvIniMatches向量对应位置置为-1)
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }
            //将初始化的第一帧作为世界坐标系，因此第一帧变换矩阵为单位矩阵
            mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            //给第二帧也就是当前帧的位姿进行赋值
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            mCurrentFrame.SetPose(Tcw);
            //将3D点包装成MapPoint类型存入KeyFrame和Map中，创建完地图点后状态置为OK，就不再进行初始化
            CreateInitialMapMonocular();
        }
    }
}

//把初始化的两帧用来创建地图点和关键帧
void Tracking::CreateInitialMapMonocular()
{
    //把用于初始化的第一帧和第二帧创建为关键帧
    KeyFrame* pKFini = new KeyFrame(mInitialFrame, mpMap, mpKeyFrameDB); //关键帧里面都有tracking线程中的mpMap和mpKeyFrameDB
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);
    //分别计算词袋向量
    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();
    //把关键帧保存入地图
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);
    //创建地图点并把地图点关联到关键帧
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        //如果匹配点没有三角化成功
        if(mvIniMatches[i]<0)
            continue;
        //创建地图点
        cv::Mat worldPos(mvIniP3D[i]);
        MapPoint* pMP = new MapPoint(worldPos, pKFcur, mpMap);
        //关键帧添加地图点
        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);
        //地图点添加观测
        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);
        //更新该地图点平均观测方向以及观测距离
        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();
        //给当前帧添加地图点和外点信息
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;
        //把地图点添加到地图
        mpMap->AddMapPoint(pMP);
    }
    //在3D点和关键帧之间建立边，每个边有一个权重，边的权重是该关键帧与当前帧共同3D点的个数
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();
    //BA优化，同时优化所有位姿和三维点
    cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;
    Optimizer::GlobalBundleAdjustemnt(mpMap, 20);
    //取初始关键帧中所有地图点最大z方向距离的一半作为medianDepth值
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f/medianDepth;
    //平均深度要小于0或者当前帧中被观测到的地图点的数目小于100，重置系统
    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<100)
    {
        cout << "Wrong initialization, reseting..." << endl;
        Reset();
        return;
    }
    //将两帧初始帧之间的变换归一化到平均深度为1的尺度下
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->SetPose(Tc2w);
    //把地图点的尺度也归一化到1
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
        }
    }
    //局部地图也插入生成的两个关键帧
    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);
    //设置当前帧的位姿，更新上一帧的id和指针变量
    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;
    //把关键帧存到局部关键帧向量
    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    //把所有地图点存放到局部地图点向量
    mvpLocalMapPoints=mpMap->GetAllMapPoints();
    //给当前帧设置参考关键帧
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;
    //给tracking线程的上一帧变量进行赋值
    mLastFrame = Frame(mCurrentFrame);
    //设置参考地图点和初始关键帧
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);
    mpMap->mvpKeyFrameOrigins.push_back(pKFini);
    //显示
    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());
    //初始化成功
    mState=OK;
}

void Tracking::CheckReplacedInLastFrame()
{
    for(int i =0; i<mLastFrame.N; i++)
    {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];
        if(pMP)
        {
            MapPoint* pRep = pMP->GetReplaced();
            if(pRep)
            {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}


bool Tracking::TrackReferenceKeyFrame()
{
    //计算当前帧词袋向量
    mCurrentFrame.ComputeBoW();
    //与参考帧进行orb匹配，有足够的匹配点进行ba优化
    ORBmatcher matcher(0.7,true);  
    vector<MapPoint*> vpMapPointMatches;
    //过词袋BoW加速当前帧与参考帧之间的特征点匹配
    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);
    //匹配点小于15返回
    if(nmatches<15)
        return false;
    //把当前帧与其参考帧匹配得到的地图点赋值给普通帧的地图点向量
    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    //把上一帧的位姿赋值给当前帧
    mCurrentFrame.SetPose(mLastFrame.mTcw);
    //通过优化3D-2D的重投影误差来获得位姿
    Optimizer::PoseOptimization(&mCurrentFrame);
    //剔除优化过程中发现的外点
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);//外点对应地图点置为空指针
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false; //mbTrackInView为false表示该地图点未被当前帧跟踪到
                pMP->mnLastFrameSeen = mCurrentFrame.mnId; //mnLastFrameSeen值表示上一次被观测到的帧id
                nmatches--;
            }                
            //匹配的内点计数加一
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }
    //跟踪成功的数目超过10才认为跟踪成功，否则跟踪失败
    return nmatchesMap>=10;
}

void Tracking::UpdateLastFrame()
{
    //获取上一帧参考关键帧指针
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    //参考关键帧到上一帧的位姿变换
    cv::Mat Tlr = mlRelativeFramePoses.back();
    //设置上一帧相对世界坐标系的位姿
    mLastFrame.SetPose(Tlr*pRef->GetPose());
    //上一帧为关键帧或者是单目相机执行到这里结束
    if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR || !mbOnlyTracking)
        return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    vector<pair<float,int> > vDepthIdx;
    vDepthIdx.reserve(mLastFrame.N);
    for(int i=0; i<mLastFrame.N;i++)
    {
        float z = mLastFrame.mvDepth[i];
        if(z>0)
        {
            vDepthIdx.push_back(make_pair(z,i));
        }
    }

    if(vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(),vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint* pMP = mLastFrame.mvpMapPoints[i];
        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations()<1)
        {
            bCreateNew = true;
        }

        if(bCreateNew)
        {
            cv::Mat x3D = mLastFrame.UnprojectStereo(i);
            MapPoint* pNewMP = new MapPoint(x3D,mpMap,&mLastFrame,i);

            mLastFrame.mvpMapPoints[i]=pNewMP;

            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }

        if(vDepthIdx[j].first>mThDepth && nPoints>100)
            break;
    }
}

//根据恒定速度模型用上一帧地图点来对当前帧进行跟踪
bool Tracking::TrackWithMotionModel()
{
    ORBmatcher matcher(0.9,true);
    //更新上一帧的位姿，对于双目或深度相机还会生成临时地图点
    UpdateLastFrame();
    //用恒速模型得到当前帧的初始位姿
    mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);
    //清空当前帧的地图点，全置为空指针
    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
    //设置特征匹配过程中的搜索半径
    int th;
    //单目
    if(mSensor!=System::STEREO)
        th=15;
    //双目
    else
        th=7;
    //用上一帧地图点进行投影匹配，如果匹配点不够，则扩大搜索半径再来一次
    int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, th, mSensor==System::MONOCULAR);
    if(nmatches<20)
    {
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 2*th, mSensor==System::MONOCULAR);
    }
    //还是不能获得足够的匹配点,就认为跟踪失败
    if(nmatches<20)
        return false;
    //优化当前帧位姿
    Optimizer::PoseOptimization(&mCurrentFrame);
    //剔除外点
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;   //mbTrackInView为false表示该地图点未被当前帧跟踪到
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;   //mnLastFrameSeen值表示上一次被观测到的帧id
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }   
    //定位模式，更新视觉里程计标志mbVO
    if(mbOnlyTracking)
    {
        mbVO = nmatchesMap<10;//匹配的内点超过10个点就认为视觉里程计有效
        return nmatches>20;//定位模式下匹配的内点超过20个点才认为跟踪成功
    }
    //建图模式，匹配的内点超过10个点就认为跟踪成功
    return nmatchesMap>=10;
}

bool Tracking::TrackLocalMap()
{
    //更新局部关键帧mvpLocalKeyFrames和局部地图点mvpLocalMapPoints
    UpdateLocalMap();
    //筛选局部地图中新增的在视野范围内的地图点，投影到当前帧搜索匹配，得到更多的匹配关系
    SearchLocalPoints();
    //优化位姿
    Optimizer::PoseOptimization(&mCurrentFrame);
    mnMatchesInliers = 0;
    //更新当前帧的地图点被观测程度，并统计跟踪局部地图后匹配数目
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            //如果非外点（每次优化都会重置内外点标记）
            if(!mCurrentFrame.mvbOutlier[i])
            {
                //由于当前帧的地图点可以被当前帧观测到，其被观测统计量加1
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                //在建图模式
                if(!mbOnlyTracking)
                {
                    //如果该地图点被相机观测数目nObs大于0，匹配内点计数+1
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        mnMatchesInliers++;
                }
                else
                    mnMatchesInliers++;
            }
            else if(mSensor==System::STEREO)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
        }
    }
    // 如果最近刚刚发生了重定位，那么至少成功匹配50个点才认为是成功跟踪
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
        return false;
    //如果是正常的状态话只要跟踪的地图点大于30个就认为成功了
    if(mnMatchesInliers<30)
        return false;
    else
        return true;
}


bool Tracking::NeedNewKeyFrame()
{
    //定位模式下不插入关键帧
    if(mbOnlyTracking)
        return false;
    //如果局部地图线程被闭环检测使用，则不插入关键帧
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;
    //取出地图中的关键帧数量
    const int nKFs = mpMap->KeyFramesInMap();
    //如果距离上一次重定位比较近，并且关键帧数目超出最大限制，不插入关键帧
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
        return false;
    //地图点的最小观测次数
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    //获取参考关键帧跟踪到的地图点数量，参考关键帧的地图点中观测的数目大于等于nMinObs
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);
    //查询局部地图线程是否繁忙，当前能否接受新的关键帧
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();
    //对于双目或RGBD摄像头，统计成功跟踪的点的数量，如果跟踪到的点太少，没有跟踪到的点较多，可以插入关键帧
    int nNonTrackedClose = 0;
    int nTrackedClose= 0;
    if(mSensor!=System::MONOCULAR)
    {
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
            {
                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i]) //当前帧的地图点非空指针且不是外点，说明该点被跟踪到
                    nTrackedClose++;
                else                    //当前帧的地图点非空指针但是该点是外点，说明该点未被跟踪到
                    nNonTrackedClose++;
            }
        }
    }
    //双目或RGBD情况下，跟踪到的地图点中近点太少，同时没有跟踪到的三维点高于阈值，可以插入关键帧了
    bool bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);
    //设定比例阈值，当前帧和参考关键帧跟踪到点的比例，比例越大，越倾向于增加关键帧
    float thRefRatio = 0.75f;
    if(nKFs<2)
        thRefRatio = 0.4f;
    if(mSensor==System::MONOCULAR)
        thRefRatio = 0.9f;
    //满足插入关键帧的最小间隔
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    //满足插入关键帧的最小间隔并且localMapper处于空闲状态
    const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);
    //在双目，RGB-D的情况下当前帧跟踪到的点比参考关键帧的0.25倍还少，或者满足bNeedToInsertClose
    const bool c1c =  mSensor!=System::MONOCULAR && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;

    //和参考帧相比当前跟踪到的点更少或者满足bNeedToInsertClose，同时跟踪到的内点还不能太少
    const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio || bNeedToInsertClose) && mnMatchesInliers>15);
    if((c1a||c1b||c1c)&&c2)
    {
        //local mapping空闲时可以直接插入，不空闲的时候要根据情况插入
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR)
            {
                //队列里不能阻塞太多关键帧
                if(mpLocalMapper->KeyframesInQueue()<3)
                    return true;
                else
                    return false;
            }
            //非单目相机，local_mapping线程繁忙就不插入
            else
                return false;
        }
    }
    else
        return false;
}

//创建新的关键帧
void Tracking::CreateNewKeyFrame()
{
    //如果局部建图线程关闭了,就无法插入关键帧
    if(!mpLocalMapper->SetNotStop(true))
        return;
    //将当前帧构造成关键帧
    KeyFrame* pKF = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);
    //把该关键帧置为线程参考关键帧
    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;
    //如果不是单目相机
    if(mSensor!=System::MONOCULAR)
    {
        //根据Tcw计算mRcw、mtcw和mRwc、mOw
        mCurrentFrame.UpdatePoseMatrices();
        //获取当前帧有深度值的特征点（不一定是地图点）
        vector<pair<float,int> > vDepthIdx;
        vDepthIdx.reserve(mCurrentFrame.N);
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                //第一个元素是深度,第二个元素是对应的特征点的id
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        if(!vDepthIdx.empty())
        {
            //按照深度从小到大排序
            sort(vDepthIdx.begin(),vDepthIdx.end());
            int nPoints = 0;
            //遍历深度向量，从中找出不是地图点的生成临时地图点 
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;
                bool bCreateNew = false;
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                //如果这个点对应的地图点没有,或者创建后就没有被观测到,那么就生成一个临时的地图点
                if(!pMP)
                    bCreateNew = true;
                //如果该地图点没有观测，置为空地图点
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }
                //创建地图点
                if(bCreateNew)
                {
                    //根据深度值计算该点相对是世界坐标系的3D坐标
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint* pNewMP = new MapPoint(x3D, pKF, mpMap);
                    //以下操作是每次创建MapPoint后都要做的
                    pNewMP->AddObservation(pKF,i);
                    pKF->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);
                    //把地图点存到当前帧
                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }
                //如果深度值大于阈值并且新添的地图带你大于100，跳出循环
                if(vDepthIdx[j].first>mThDepth && nPoints>100)
                    break;
            }
        }
    }
    //建图线程插入关键帧
    mpLocalMapper->InsertKeyFrame(pKF);
    //插入好了，允许局部建图停止
    mpLocalMapper->SetNotStop(false);
    //当前帧成为新的关键帧，更新关键帧相关变量
    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}

void Tracking::SearchLocalPoints()
{
    //遍历当前帧的地图点，标记这些地图点不参与之后的投影搜索匹配
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                //更新能观测到该点的帧数加1(被当前帧观测了)
                pMP->IncreaseVisible();                       //mnVisible++
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;    //标记该点被当前帧观测到
                pMP->mbTrackInView = false;                   //标记该点在后面搜索匹配时不被投影，当前帧的地图点通过TrackWithMotionModel之类的函数已经匹配过了
            }
        }
    }
    //准备进行投影匹配的点的数目
    int nToMatch=0;
    //判断所有局部地图点中除当前帧地图点外的点，是否在当前帧视野范围内
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        //已经被当前帧观测到的地图点肯定在视野范围内，跳过
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;
        //判断地图点是否在在当前帧视野内
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible(); //mnVisible++
            nToMatch++;
        }
    }
    //如果需要进行投影匹配的点的数目大于0，就进行投影匹配，增加更多的匹配关系
    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        if(mSensor==System::RGBD)
            th=3;
        //如果不久前进行过重定位，那么进行一个更加宽泛的搜索，阈值需要增大
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;
        matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th);
    }
}

void Tracking::UpdateLocalMap()
{
    //把tracking线程的局部地图点指针赋值给地图的参考地图点指针
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);
    //更新局部地图点和关键帧
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}

//将局部关键帧的有效地图点添加到局部地图中
void Tracking::UpdateLocalPoints()
{
    //清空
    mvpLocalMapPoints.clear();
    //遍历局部关键帧
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();
        //遍历该帧的地图点
        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
                    continue;
            //用地图点的成员变量mnTrackReferenceForFrame记录当前帧的id，防止重复添加局部地图点
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
}

//遍历当前帧的地图点，将观测到这些地图点的关键帧和相邻的关键帧及其父子关键帧作为局部关键帧
void Tracking::UpdateLocalKeyFrames()
{
    //遍历当前帧的地图点，记录所有能观测到当前帧地图点的关键帧
    map<KeyFrame*, int> keyframeCounter;//第一项存放关键帧指针，第二项存放该关键帧与当前帧的公共地图点个数
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(!pMP->isBad())
            {
                const map<KeyFrame*,size_t> observations = pMP->GetObservations();
                for(map<KeyFrame*, size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;//该关键帧共享地图点个数加一
            }
            //地图点是坏点，当前帧对应地图点指针置为空
            else
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
            }
        }
    }
    //没找到共享地图点的关键帧返回
    if(keyframeCounter.empty())
        return;
    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);
    //清空局部关键帧容器，重置申请3倍内存大小
    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());
    //能观测到当前帧地图点的关键帧作为局部关键帧（一级共视关键帧）
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;
        if(pKF->isBad())
            continue;
        //记录最多的共享地图点个数和对应的关键帧
        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }
        //添加到局部关键帧的列表里
        mvpLocalKeyFrames.push_back(it->first);
        //用该关键帧的成员变量mnTrackReferenceForFrame记录当前帧的id，防止查找二级共视关键帧时重复添加局部关键帧
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }
    //遍历一级共视关键帧，寻找更多的局部关键帧（二级共视关键帧）
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        //处理的局部关键帧不超过80帧
        if(mvpLocalKeyFrames.size()>80)
            break;

        KeyFrame* pKF = *itKF;
        //一级共视关键帧的共视（前10个）关键帧，称为二级共视关键帧
        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);
        //vNeighs按照共视程度从大到小排列
        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                //mnTrackReferenceForFrame防止重复添加局部关键帧
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    //每个一级共视关键帧找到一个共视程度最高的二级共视关键帧就直接跳出循环
                    break;
                }
            }
        }
        //将一级共视关键帧的子关键帧作为局部关键帧
        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                //mnTrackReferenceForFrame防止重复添加局部关键帧
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    //找到一个就跳出循环
                    break;
                }
            }
        }
        //将一级共视关键帧的父关键帧作为局部关键帧
        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            //mnTrackReferenceForFrame防止重复添加局部关键帧
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                //找到一个就跳出循环
                break;
            }
        }
    }
    //更新当前帧的参考关键帧，与自己共视程度最高的关键帧作为参考关键帧
    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

bool Tracking::Relocalization()
{
    //当前帧计算词袋向量
    mCurrentFrame.ComputeBoW();
    //查询关键帧数据库，用词袋找到与当前帧相似的候选关键帧
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);
    //如果没有候选关键帧，则退出
    if(vpCandidateKFs.empty())
        return false;
    //候选帧数量
    const int nKFs = vpCandidateKFs.size();
    //初始化orb匹配器
    ORBmatcher matcher(0.75,true);
    //每个关键帧都有pnp解算器
    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);
    //每个候选关键帧和当前帧中特征点匹配的地图点集合
    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);
    //放弃某个关键帧的标记
    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);
    //遍历所有的候选关键帧，通过词袋进行快速匹配，用匹配结果初始化PnP Solver
    int nCandidates=0;
    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            //当前帧和候选关键帧用BoW进行快速匹配，匹配结果记录在vvpMapPointMatches，nmatches表示匹配的数目
            int nmatches = matcher.SearchByBoW(pKF, mCurrentFrame, vvpMapPointMatches[i]);
            //匹配点数量小于15抛弃
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            //匹配点足够给该关键帧初始化EPnPsolver解算器
            else
            {
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame, vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99, 10, 300, 4, 0.5, 5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }
    //Epnp每次迭代需要4个点
    bool bMatch = false;  //是否已经找到相匹配的关键帧的标志
    ORBmatcher matcher2(0.9, true);
    //没有找到匹配并且候选关键帧计数大于0继续循环
    while(nCandidates>0 && !bMatch)
    {
        //遍历当前所有的候选关键帧
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;
            vector<bool> vbInliers; //内点标记
            int nInliers;           //内点数量
            bool bNoMore;           //RANSAC迭代次数用完置为true
            //取出解算器
            PnPsolver* pSolver = vpPnPsolvers[i];
            //执行5次Ransac迭代
            cv::Mat Tcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers);
            //RANSAC迭代次数用完，抛弃该候选帧
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }
            //如果EPnP计算出了位姿，对内点进行BA优化
            if(!Tcw.empty())
            {
                //给当前帧位姿进行赋值
                Tcw.copyTo(mCurrentFrame.mTcw);
                //存放EPnP里RANSAC后的内点的集合
                set<MapPoint*> sFound;
                //内点数量
                const int np = vbInliers.size();
                //遍历所有内点，存放进sFound容器
                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];//给当前帧的地图点向量进行赋值
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }
                //对当前帧进行位姿优化
                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);
                //优化后的内点太少处理下一候选帧
                if(nGood<10)
                    continue;
                //把优化中找到的外点置为空节点
                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);
                //如果内点较少通过投影的方式对之前未匹配的点进行匹配，再进行优化求解(前面的匹配关系是用词袋匹得到的)
                if(nGood<50)
                {                    
                    //通过投影的方式将关键帧中未匹配的地图点投影到当前帧中, 生成新的匹配
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 10, 100);
                    //投影过程新增了足够的匹配特征点对
                    if(nadditional+nGood>=50)
                    {
                        //再次进行位姿优化
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);
                        //如果优化后内点数还是比较少(<50)但是还不至于太少(>30)，使用更小的搜索窗口再次优化
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 3, 64);
                            //匹配点足够则进行最后一次优化
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);
                                //剔除外点
                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }
                //如果找到足够的内点，停止循环，重定位成功
                if(nGood>=50)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }
    if(!bMatch)
    {
        return false;
    }
    //如果重定位成功，重置变量mnLastRelocFrameId
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }
}

//重置函数
void Tracking::Reset()
{
    cout << "System Reseting" << endl;
    //关闭图像界面
    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }
    //重置局部建图
    cout << "Reseting Local Mapper...";
    mpLocalMapper->RequestReset();
    cout << " done" << endl;
    //重置闭环检测
    cout << "Reseting Loop Closing...";
    mpLoopClosing->RequestReset();
    cout << " done" << endl;
    //清除关键帧数据库
    cout << "Reseting Database...";
    mpKeyFrameDB->clear();
    cout << " done" << endl;
    //清除地图(地图点和关键帧)
    mpMap->clear();
    //相关标志位置为初始状态
    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;
    //删除初始化器
    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }
    //相关容器进行清理
    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();
    //清理图像界面对象
    if(mpViewer)
        mpViewer->Release();
}
//改变相机内参
void Tracking::ChangeCalibration(const string &strSettingPath)
{
    //读取配置文件
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];
    //设置tracking线程内参矩阵
    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);
    //读取配置文件畸变参数
    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    //设置tracking线程畸变参数
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];
    Frame::mbInitialComputations = true;
}
//让traking线程只进行跟踪
void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}



} //namespace ORB_SLAM
