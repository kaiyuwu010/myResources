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


#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"Viewer.h"
#include"FrameDrawer.h"
#include"Map.h"
#include"LocalMapping.h"
#include"LoopClosing.h"
#include"Frame.h"
#include "ORBVocabulary.h"
#include"KeyFrameDatabase.h"
#include"ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"

#include <mutex>

namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class LocalMapping;
class LoopClosing;
class System;

class Tracking
{  

public:
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetViewer(Viewer* pViewer);

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    // TODO: Modify MapPoint::PredictScale to take into account focal lenght
    void ChangeCalibration(const string &strSettingPath);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);


public:
    //Track线程跟踪状态
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };
    //跟踪线程当前状态
    eTrackingState mState;
    //上一帧处理完成后的状态
    eTrackingState mLastProcessedState;

    //传感器类型
    int mSensor;
    //线程当前帧
    Frame mCurrentFrame;
    cv::Mat mImGray;

    //初始化变量(针对单目)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;//存储匹配的点的ID
    std::vector<cv::Point2f> mvbPrevMatched;//记录上一帧所有特征点
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;//初始化帧

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<cv::Mat> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    //只执行定位的标志位
    bool mbOnlyTracking;
    //重置函数
    void Reset();

protected:

    //主跟踪函数
    void Track();
    //双目和深度相机初始化
    void StereoInitialization();
    //单目地图初始化
    void MonocularInitialization();
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();

    bool Relocalization();
    //更新局部地图、局部地图点和局部关键帧
    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();
    //跟踪局部地图
    bool TrackLocalMap();
    void SearchLocalPoints();
    //判断是否生成新关键帧和新关键帧生成函数
    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();
    //当相对地图没有足够的匹配点，只进行定位时该标志位置为true。如果有足够的临时地图点会继续跟踪，这种情况下进行视觉里程计，系统会重定位恢复相对地图的零漂移定位
    bool mbVO;
    //其他线程指针
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;
    //ORB
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;
    //词典和关键帧数据库
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;
    //单目初始化器
    Initializer* mpInitializer;
    //局部地图相关变量，每次跟踪都会重置更新
    KeyFrame* mpReferenceKF;
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;
    //系统对象
    System* mpSystem;
    //Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    //Map指针(包括地图点和关键帧)
    Map* mpMap;
    //内参矩阵
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;
    //新关键帧生成条件(according to fps)
    int mMinFrames;
    int mMaxFrames;
    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;
    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;
    //当前匹配的内点数量
    int mnMatchesInliers;
    //上一次track的帧、关键帧和重定位信息
    KeyFrame* mpLastKeyFrame;
    Frame mLastFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;
    //运动模型
    cv::Mat mVelocity;
    //颜色顺序(true表示RGB, false表示BGR, 灰度图忽略)
    bool mbRGB;
    //临时地图点列表
    list<MapPoint*> mlpTemporalPoints;
};

} //namespace ORB_SLAM

#endif // TRACKING_H
