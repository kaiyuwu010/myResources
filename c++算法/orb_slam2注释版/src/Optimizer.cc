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

#include "Optimizer.h"

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

#include<Eigen/StdVector>

#include "Converter.h"

#include<mutex>

namespace ORB_SLAM2
{


void Optimizer::GlobalBundleAdjustemnt(Map* pMap, int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
    vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    vector<MapPoint*> vpMP = pMap->GetAllMapPoints();
    BundleAdjustment(vpKFs,vpMP,nIterations,pbStopFlag, nLoopKF, bRobust);
}

//给定所有关键帧和地图点，优化它们的位姿。把这些关键帧和地图点作为顶点，遍历地图点的观测帧，把它们间的图像坐标添加为边，进行优化。（这里的优化不会剔除外点，因为都是关键帧，都是很重要的数据）
void Optimizer::BundleAdjustment(const vector<KeyFrame *> &vpKFs, //所有的keyframes
                                 const vector<MapPoint *> &vpMP,  //所有的地图点
                                 int nIterations, 
                                 bool* pbStopFlag,   //强制停止标志位
                                 const unsigned long nLoopKF, //关键帧个数
                                 const bool bRobust)   //是否使用核函数
{
    vector<bool> vbNotIncludedMP;
    vbNotIncludedMP.resize(vpMP.size());
    //初始化g2o优化器
    g2o::SparseOptimizer optimizer;
    //创建线性求解器
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;
    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
    //创建BlockSolver矩阵块求解器 求解雅克比矩阵和海塞矩阵,并用上面定义的线性求解器初始化
    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
    //迭代算法solver，从GN, LM, DogLeg中选一个，再用BlockSolver初始化
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);
    //强制停止
    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);
    //记录添加了顶点的关键帧的最大id
    long unsigned int maxKFid = 0;
    //添加关键帧位姿顶点，所有的关键帧
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        //生成李代数表示的关键帧顶点
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        //设置顶点参数
        vSE3->setEstimate(Converter::toSE3Quat(pKF->GetPose()));
        vSE3->setId(pKF->mnId);
        vSE3->setFixed(pKF->mnId==0);
        //添加顶点
        optimizer.addVertex(vSE3);
        if(pKF->mnId>maxKFid)
            maxKFid=pKF->mnId;
    }

    const float thHuber2D = sqrt(5.99);
    const float thHuber3D = sqrt(7.815);
    //添加所有的MapPoints顶点
    for(size_t i=0; i<vpMP.size(); i++)
    {
        MapPoint* pMP = vpMP[i];
        if(pMP->isBad())
            continue;
        //生成地图点顶点
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        //设置参数
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        //地图顶点id是接在关键帧顶点id后面的
        const int id = pMP->mnId + maxKFid + 1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);
        //取出当前地图点的观测
        const map<KeyFrame*, size_t> observations = pMP->GetObservations();
        int nEdges = 0;
        //添加投影边，遍历MapPoint的所有观测
        for(map<KeyFrame*,size_t>::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
        {
            //取出地图点观测到的关键帧
            KeyFrame* pKF = mit->first;
            if(pKF->isBad() || pKF->mnId>maxKFid)
                continue;
            //边计数加一
            nEdges++;
            //取出地图点对应的图像坐标
            const cv::KeyPoint &kpUn = pKF->mvKeysUn[mit->second];
            //如果是单目
            if(pKF->mvuRight[mit->second]<0)
            {
                Eigen::Matrix<double,2,1> obs;
                obs << kpUn.pt.x, kpUn.pt.y;
                //生成g2o投影边
                g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();
                //设置边，给该地图点和对应的观测关键帧之间添加边，测量值是图像坐标系的坐标；把顶点0投影到顶点1求图像坐标进行比较
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                e->setMeasurement(obs);
                //根据金字塔层级，设置信息矩阵
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);
                //设置鲁棒核函数
                if(bRobust)
                {
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber2D);
                }
                //设置相机内参
                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;
                //添加边
                optimizer.addEdge(e);
            }
            //非单目情况
            else
            {
                Eigen::Matrix<double,3,1> obs;
                const float kp_ur = pKF->mvuRight[mit->second];
                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;
                g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                e->setInformation(Info);
                if(bRobust)
                {
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber3D);
                }
                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;
                e->bf = pKF->mbf;
                optimizer.addEdge(e);
            }
        }
        if(nEdges==0)
        {
            optimizer.removeVertex(vPoint);
            vbNotIncludedMP[i]=true;
        }
        else
        {
            vbNotIncludedMP[i]=false;
        }
    }
    //优化
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);
    //把优化后的关键帧位姿数据，赋值给关键帧
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        if(nLoopKF==0)
        {
            pKF->SetPose(Converter::toCvMat(SE3quat));
        }
        else
        {
            pKF->mTcwGBA.create(4, 4, CV_32F);
            Converter::toCvMat(SE3quat).copyTo(pKF->mTcwGBA);
            pKF->mnBAGlobalForKF = nLoopKF;
        }
    }
    //把优化后的地图点位姿数据，赋值给地图点
    for(size_t i=0; i<vpMP.size(); i++)
    {
        if(vbNotIncludedMP[i])
            continue;
        MapPoint* pMP = vpMP[i];
        if(pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));
        if(nLoopKF==0)
        {
            pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
            pMP->UpdateNormalAndDepth();
        }
        else
        {
            pMP->mPosGBA.create(3, 1, CV_32F);
            Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
            pMP->mnBAGlobalForKF = nLoopKF;
        }
    }
}

//给定一个普通帧，设置地图点在该帧上的图像坐标为边，对该帧的位姿（顶点）进行优化，地图点在世界坐标系的位置是固定的。总优化4次，每次优化完会取出外点，在进行下次优化。
int Optimizer::PoseOptimization(Frame *pFrame)
{
    //创建稀疏优化器
    g2o::SparseOptimizer optimizer;
    //创建一个线性求解器LinearSolver
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;
    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();
    //创建BlockSolver矩阵块求解器 求解雅克比矩阵和海塞矩阵,并用上面定义的线性求解器初始化
    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
    //迭代算法solver，从GN,LM,DogLeg中选一个，再用BlockSolver初始化
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    //给优化器optimizer设置求解器solver
    optimizer.setAlgorithm(solver);

    int nInitialCorrespondences=0;
    //设置帧位姿顶点，VertexSE3Expmap为李代数位姿
    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
    vSE3->setId(0);
    vSE3->setFixed(false);//vSE3是要优化的变量不是固定值
    optimizer.addVertex(vSE3);//添加顶点,相机位姿用李代数表示
    //初始化存放边的容器和存放地图点索引的容器
    const int N = pFrame->N;
    vector<g2o::EdgeSE3ProjectXYZOnlyPose*> vpEdgesMono;
    vector<size_t> vnIndexEdgeMono;
    vpEdgesMono.reserve(N);
    vnIndexEdgeMono.reserve(N);
    //初始化双目存放边的容器和存放地图点索引的容器
    vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose*> vpEdgesStereo;
    vpEdgesStereo.reserve(N);
    vector<size_t> vnIndexEdgeStereo;
    vnIndexEdgeStereo.reserve(N);

    const float deltaMono = sqrt(5.991);
    const float deltaStereo = sqrt(7.815);
    {
    unique_lock<mutex> lock(MapPoint::mGlobalMutex);
    //遍历帧中的地图点，向optimizer添加边
    for(int i=0; i<N; i++)
    {
        MapPoint* pMP = pFrame->mvpMapPoints[i];
        if(pMP)
        {
            //单目的情况
            if(pFrame->mvuRight[i]<0)
            {
                nInitialCorrespondences++;
                pFrame->mvbOutlier[i] = false;
                //用obs存放特征点坐标
                Eigen::Matrix<double,2,1> obs;
                const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                obs << kpUn.pt.x, kpUn.pt.y;
                //创建边对象
                g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();
                //optimizer.vertex(0)返回的就是上边的vSE3结点，也就是当前帧的Tcw
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                //设置相机平面坐标obs
                e->setMeasurement(obs);
                //获取关键点所在金字塔层级的sigma2值
                const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);
                //设置RobustKernel
                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(deltaMono);
                //设置相机内参
                e->fx = pFrame->fx;
                e->fy = pFrame->fy;
                e->cx = pFrame->cx;
                e->cy = pFrame->cy;
                //设置地图点在世界坐标系的坐标
                cv::Mat Xw = pMP->GetWorldPos();
                e->Xw[0] = Xw.at<float>(0);
                e->Xw[1] = Xw.at<float>(1);
                e->Xw[2] = Xw.at<float>(2);
                //添加边
                optimizer.addEdge(e);
                //把边和地图点索引存入变量
                vpEdgesMono.push_back(e);
                vnIndexEdgeMono.push_back(i);
            }
            //双目的情况
            else 
            {
                nInitialCorrespondences++;
                pFrame->mvbOutlier[i] = false;
                //SET EDGE
                Eigen::Matrix<double,3,1> obs;
                const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                const float &kp_ur = pFrame->mvuRight[i];
                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;
                g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose();
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                e->setMeasurement(obs);
                const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                e->setInformation(Info);
                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(deltaStereo);
                e->fx = pFrame->fx;
                e->fy = pFrame->fy;
                e->cx = pFrame->cx;
                e->cy = pFrame->cy;
                e->bf = pFrame->mbf;
                cv::Mat Xw = pMP->GetWorldPos();
                e->Xw[0] = Xw.at<float>(0);
                e->Xw[1] = Xw.at<float>(1);
                e->Xw[2] = Xw.at<float>(2);
                optimizer.addEdge(e);
                vpEdgesStereo.push_back(e);
                vnIndexEdgeStereo.push_back(i);
            }
        }
    }
    }
    //添加的边的数量小于3返回
    if(nInitialCorrespondences<3)
        return 0;
    //总共优化四次，每次优化后将观测分为outlier和inlier，outlier不参与下次优化
    //每次优化后是对所有的观测进行outlier和inlier判别，因此之前被判别为outlier有可能变成inlier，反之亦然
    const float chi2Mono[4]={5.991,5.991,5.991,5.991};
    const float chi2Stereo[4]={7.815,7.815,7.815, 7.815};
    const int its[4]={10,10,10,10};    

    int nBad=0;
    //优化四次
    for(size_t it=0; it<4; it++)
    {
        //设置要优化的位姿变量，每次优化都是在前一次优化的基础上进行
        vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
        //只对level为0的边进行优化，此处0代表inlier，1代表outlier（对内点进行优化，第一次优化全是内点）
        optimizer.initializeOptimization(0);
        //进行图优化
        optimizer.optimize(its[it]);
        //记录坏点数量
        nBad=0;
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
        {
            //从容器中取出边
            g2o::EdgeSE3ProjectXYZOnlyPose* e = vpEdgesMono[i];
            //从容器中取出边对应的地图点索引
            const size_t idx = vnIndexEdgeMono[i];
            //若该地图点被标记为外点，对外点边计算误差，外点可能转化为内点
            if(pFrame->mvbOutlier[idx])
            {
                e->computeError();//计算边误差
            }
            const float chi2 = e->chi2();
            //误差大于阈值
            if(chi2>chi2Mono[it])
            {                
                pFrame->mvbOutlier[idx]=true;//标记为外点
                e->setLevel(1);//当前边设置为outlier
                nBad++;
            }
            //误差小于阈值
            else
            {
                pFrame->mvbOutlier[idx]=false;//标记为内点
                e->setLevel(0);//当前边设置为inlier
            }
            //前两次优化需要RobustKernel
            if(it==2)
                e->setRobustKernel(0);
        }
        //双目的情况
        for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++)
        {
            g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = vpEdgesStereo[i];
            const size_t idx = vnIndexEdgeStereo[i];
            if(pFrame->mvbOutlier[idx])
            {
                e->computeError();
            }
            const float chi2 = e->chi2();
            if(chi2>chi2Stereo[it])
            {
                pFrame->mvbOutlier[idx]=true;
                e->setLevel(1);
                nBad++;
            }
            else
            {                
                e->setLevel(0);
                pFrame->mvbOutlier[idx]=false;
            }
            if(it==2)
                e->setRobustKernel(0);
        }
        if(optimizer.edges().size()<10)
            break;
    }    
    //输出优化后的位姿
    g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    cv::Mat pose = Converter::toCvMat(SE3quat_recov);
    pFrame->SetPose(pose);//把优化后的位姿赋给当前帧
    //返回内点个数
    return nInitialCorrespondences-nBad;
}

//局部关键帧：当前关键帧的共视关键帧和当前关键帧本身
//局部地图点：局部关键帧能观测到的地图点集合
//输入关键帧，把当前关键帧及其共视关键帧建立顶点，把能观测到局部地图点但不属于局部关键帧的关键帧设置为固定顶点，遍历局部地图点把局部地图点设置为顶点并添加局部地图点和与之相连的关键帧之间的投影边
void Optimizer::LocalBundleAdjustment(KeyFrame *pKF, bool* pbStopFlag, Map* pMap)
{    
    // Local KeyFrames: First Breath Search from Current Keyframe
    list<KeyFrame*> lLocalKeyFrames;
    //将当前关键帧加入lLocalKeyFrames
    lLocalKeyFrames.push_back(pKF);
    //将局部BA的id置为当前关键帧id
    pKF->mnBALocalForKF = pKF->mnId;
    //获取当前关键帧的共视关键帧
    const vector<KeyFrame*> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
    for(int i=0, iend=vNeighKFs.size(); i<iend; i++)
    {
        KeyFrame* pKFi = vNeighKFs[i];
        //将局部BA的id置为当前关键帧id
        pKFi->mnBALocalForKF = pKF->mnId;
        //将共视关键帧加入lLocalKeyFrames
        if(!pKFi->isBad())
            lLocalKeyFrames.push_back(pKFi);
    }
    //初始化容器存放局部关键帧的局部地图点
    list<MapPoint*> lLocalMapPoints;
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin() , lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        vector<MapPoint*> vpMPs = (*lit)->GetMapPointMatches();
        for(vector<MapPoint*>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
        {
            MapPoint* pMP = *vit;
            if(pMP)
                if(!pMP->isBad())
                    //标记一下地图点，防止重复添加
                    if(pMP->mnBALocalForKF!=pKF->mnId)
                    {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF=pKF->mnId;
                    }
        }
    }
    //得到能被局部MapPoints观测到，但不属于局部关键帧的关键帧，这些关键帧在局部BA优化时不优化（作为固定顶点）
    list<KeyFrame*> lFixedCameras;
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        map<KeyFrame*,size_t> observations = (*lit)->GetObservations();
        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;
            //判断不属于局部关键帧；并且没有没标记为mnBAFixedForKF的关键帧（防止重复添加）
            if(pKFi->mnBALocalForKF!=pKF->mnId && pKFi->mnBAFixedForKF!=pKF->mnId)
            {                
                pKFi->mnBAFixedForKF=pKF->mnId;
                if(!pKFi->isBad())
                    lFixedCameras.push_back(pKFi);
            }
        }
    }
    //设置优化器
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;
    //创建线性优化器
    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
    //创建BlockSolver矩阵块求解器 求解雅克比矩阵和海塞矩阵,并用上面定义的线性求解器初始化
    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
    //迭代算法solver，从GN,LM,DogLeg中选一个，再用BlockSolver初始化
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);
    //判断是否强制停止
    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    unsigned long maxKFid = 0;
    //设置局部关键帧顶点
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        //设置顶点参数
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(pKFi->mnId==0);
        //添加顶点
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
    }
    //设置固定关键帧顶点
    for(list<KeyFrame*>::iterator lit=lFixedCameras.begin(), lend=lFixedCameras.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        //设置顶点参数
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        //设置为固定顶点
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
    }
    //设置地图点顶点
    const int nExpectedSize = (lLocalKeyFrames.size()+lFixedCameras.size())*lLocalMapPoints.size();
    //创建存放单目边的容器
    vector<g2o::EdgeSE3ProjectXYZ*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);
    //创建存放单目边对应关键帧的容器
    vector<KeyFrame*> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);
    //创建存放单目边对应地图点的容器
    vector<MapPoint*> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);
    //创建非单目的相关容器
    vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);
    vector<KeyFrame*> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);
    vector<MapPoint*> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);
    //初始化参数
    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);
    //遍历局部地图点
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        //创建地图点顶点
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        //计算地图点id
        int id = pMP->mnId+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        //添加顶点
        optimizer.addVertex(vPoint);
        //获取局部地图点的观测
        const map<KeyFrame*, size_t> observations = pMP->GetObservations();
        //设置边，遍历可观测到局部地图点的关键帧
        for(map<KeyFrame*, size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;
            if(!pKFi->isBad())
            {                
                const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];
                //单目情况
                if(pKFi->mvuRight[mit->second]<0)
                {
                    //取出图像坐标
                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;
                    //创建边
                    g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();
                    //设置边顶点和观测
                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    //提取金字塔层级对应的参数，构造信息矩阵
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);
                    //构造鲁棒核
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);
                    //设置相机内参
                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;
                    //添加边
                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);
                }
                //非单目相机
                else
                {
                    Eigen::Matrix<double,3,1> obs;
                    const float kp_ur = pKFi->mvuRight[mit->second];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;
                    g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();
                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                    e->setInformation(Info);
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);
                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;
                    e->bf = pKFi->mbf;
                    optimizer.addEdge(e);
                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKFi);
                    vpMapPointEdgeStereo.push_back(pMP);
                }
            }
        }
    }
    if(pbStopFlag)
        if(*pbStopFlag)
            return;
    //优化
    optimizer.initializeOptimization();
    optimizer.optimize(5);
    bool bDoMore= true;
    if(pbStopFlag)
        if(*pbStopFlag)
            bDoMore = false;
    //如果要排除外点，再优化的话
    if(bDoMore)
    {
        //遍历边，检测outlier，并设置下次不优化
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
        {
            //取出边和对应地图点
            g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
            MapPoint* pMP = vpMapPointEdgeMono[i];
            if(pMP->isBad())
                continue;
            //检测误差
            if(e->chi2()>5.991 || !e->isDepthPositive())
            {
                e->setLevel(1);  //不优化
            }
            //设置鲁棒核
            e->setRobustKernel(0);
        }
        //非单目情况，遍历边，检测outlier，并设置下次不优化
        for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
        {
            g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
            MapPoint* pMP = vpMapPointEdgeStereo[i];
            if(pMP->isBad())
                continue;

            if(e->chi2()>7.815 || !e->isDepthPositive())
            {
                e->setLevel(1);
            }
            e->setRobustKernel(0);
        }
        //排除误差较大的outlier后，再次优化
        optimizer.initializeOptimization(0);
        optimizer.optimize(10);
    }
    //存放要删除的边的关键帧和地图定对
    vector<pair<KeyFrame*, MapPoint*>> vToErase;
    vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());
    //优化后重新计算误差，剔除连接误差比较大的关键帧和地图点      
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        //取出单目边
        g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];
        if(pMP->isBad())
            continue;
        //检测误差，误差超过阈值存入vToErase容器
        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi, pMP));
        }
    }
    //双目情况
    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
    {
        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
        MapPoint* pMP = vpMapPointEdgeStereo[i];
        if(pMP->isBad())
            continue;
        if(e->chi2()>7.815 || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }
    //加锁，对于误差较大的边，对于的关键帧和地图点相互删除观测关系
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);
    if(!vToErase.empty())
    {
        for(size_t i=0;i<vToErase.size();i++)
        {
            KeyFrame* pKFi = vToErase[i].first;
            MapPoint* pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }
    //把优化后的数据，赋值给对于的关键帧和地图点
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKF = *lit;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        //设置新的位姿
        pKF->SetPose(Converter::toCvMat(SE3quat));
    }
    //设置地图点位置和更新观测方向和深度
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
    }
}

//把所有关键帧添加为顶点，然后添加四种边，一种是闭环时新产生的连接关系作为边，二是遍历所有关键帧建立其与父关键帧的相对位姿为边，
//三是遍历关键帧时若该关键帧有回环边添加其与回环边的相对位姿为边，四是遍历关键帧时与其共视地图点大于100的关键帧间的相对位姿建立边
void Optimizer::OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections, 
                                       const bool &bFixScale)
{
    //设置g2o优化器optimizer
    //创建稀疏优化器
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    //创建线性求解器
    g2o::BlockSolver_7_3::LinearSolverType * linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
    //创建BlockSolver矩阵块求解器 求解雅克比矩阵和海塞矩阵,并用上面定义的线性求解器初始化
    g2o::BlockSolver_7_3 * solver_ptr= new g2o::BlockSolver_7_3(linearSolver);
    //迭代算法solver，从GN,LM,DogLeg中选一个，再用BlockSolver初始化
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    solver->setUserLambdaInit(1e-16);
    //给优化器optimizer设置求解器solver
    optimizer.setAlgorithm(solver);
    //获取所有关键帧和地图点
    const vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    const vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();
    //取出最大关键帧id
    const unsigned int nMaxKFid = pMap->GetMaxKFid();
    //存放关键帧顶点的位姿
    vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(nMaxKFid+1);
    vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vCorrectedSwc(nMaxKFid+1);
    //声明g2o顶点向量
    vector<g2o::VertexSim3Expmap*> vpVertices(nMaxKFid+1);
    const int minFeat = 100;
    //设置关键帧顶点
    for(size_t i=0, iend=vpKFs.size(); i<iend;i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();
        //获取id
        const int nIDi = pKF->mnId;
        //获取关键帧矫正过的位姿，存放到容器vScw，并用于g2o设置
        LoopClosing::KeyFrameAndPose::const_iterator it = CorrectedSim3.find(pKF);
        if(it!=CorrectedSim3.end())
        {
            vScw[nIDi] = it->second;
            VSim3->setEstimate(it->second);
        }
        //如果没有矫正过的位姿，取出跟踪过程中得到的位姿
        else
        {
            Eigen::Matrix<double,3,3> Rcw = Converter::toMatrix3d(pKF->GetRotation());
            Eigen::Matrix<double,3,1> tcw = Converter::toVector3d(pKF->GetTranslation());
            g2o::Sim3 Siw(Rcw, tcw, 1.0);
            vScw[nIDi] = Siw;
            VSim3->setEstimate(Siw);
        }
        //如果是回环关键帧，设置为固定
        if(pKF == pLoopKF)
            VSim3->setFixed(true);
        //设置id、边缘和固定缩放系数
        VSim3->setId(nIDi);
        VSim3->setMarginalized(false);
        VSim3->_fix_scale = bFixScale;
        //添加顶点
        optimizer.addVertex(VSim3);
        //存放到顶点向量
        vpVertices[nIDi]=VSim3;
    }
    //记录添加的边对应的两个顶点（关键帧）id
    set<pair<long unsigned int, long unsigned int> > sInsertedEdges;
    //设置单位阵
    const Eigen::Matrix<double,7,7> matLambda = Eigen::Matrix<double,7,7>::Identity();
    //设置回环边，LoopConnections是闭环时因为地图点调整而出现的新关键帧连接关系
    for(map<KeyFrame *, set<KeyFrame *> >::const_iterator mit = LoopConnections.begin(), mend=LoopConnections.end(); mit!=mend; mit++)
    {
        //取出关键帧
        KeyFrame* pKF = mit->first;
        const long unsigned int nIDi = pKF->mnId;
        //取出共视关键帧集合
        const set<KeyFrame*> &spConnections = mit->second;
        //取出位姿
        const g2o::Sim3 Siw = vScw[nIDi];
        const g2o::Sim3 Swi = Siw.inverse();
        //遍历共视关键帧
        for(set<KeyFrame*>::const_iterator sit=spConnections.begin(), send=spConnections.end(); sit!=send; sit++)
        {
            //取出共视帧id
            const long unsigned int nIDj = (*sit)->mnId;
            if((nIDi!=pCurKF->mnId || nIDj!=pLoopKF->mnId) && pKF->GetWeight(*sit)<minFeat)
                continue;
            //取出共视帧位姿
            const g2o::Sim3 Sjw = vScw[nIDj];
            //计算关键帧i相对于j的位姿
            const g2o::Sim3 Sji = Sjw * Swi;
            //定义边
            g2o::EdgeSim3* e = new g2o::EdgeSim3();
            //定义边的两个顶点
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            //Sji内部是经过了Sim调整的观测
            e->setMeasurement(Sji);
            //信息矩阵是单位阵，说明这类新增加的边对总误差的贡献也都是一样大的
            e->information() = matLambda;
            //添加边
            optimizer.addEdge(e);
            //记录添加的边对应的两个顶点（关键帧）id
            sInsertedEdges.insert(make_pair(min(nIDi, nIDj), max(nIDi, nIDj)));
        }
    }
    //添加跟踪时形成的边、闭环匹配成功形成的边
    // 遍历vpKFs，将vpKFs和其在拓展树中的父节点在g2o图中连接起来形成一条误差边，将vpKFs和其形成闭环的帧在g2o图中连接起来形成一条误差边
    for(size_t i=0, iend=vpKFs.size(); i<iend; i++)
    {
        //取出关键帧
        KeyFrame* pKF = vpKFs[i];
        //取出id
        const int nIDi = pKF->mnId;
        //取出未矫正的位姿
        g2o::Sim3 Swi;
        LoopClosing::KeyFrameAndPose::const_iterator iti = NonCorrectedSim3.find(pKF);
        if(iti!=NonCorrectedSim3.end())
            Swi = (iti->second).inverse();
        else
            Swi = vScw[nIDi].inverse();
        //取出父关键帧
        KeyFrame* pParentKF = pKF->GetParent();
        if(pParentKF)
        {
            //取出父关键帧id
            int nIDj = pParentKF->mnId;
            //取出未矫正的位姿
            g2o::Sim3 Sjw;
            LoopClosing::KeyFrameAndPose::const_iterator itj = NonCorrectedSim3.find(pParentKF);
            if(itj!=NonCorrectedSim3.end())
                Sjw = itj->second;
            else
                Sjw = vScw[nIDj];
            //计算子关键帧相对父关键帧的位姿
            g2o::Sim3 Sji = Sjw * Swi;
            //添加以父子关键帧为节点的边
            g2o::EdgeSim3* e = new g2o::EdgeSim3();
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            e->setMeasurement(Sji);
            //添加信息矩阵
            e->information() = matLambda;
            //添加边
            optimizer.addEdge(e);
        }
        //回环边，添加在CorrectLoop函数中AddLoopEdge函数添加的闭环关键帧和当前帧的边（第一次回环这里sLoopEdges为空）
        const set<KeyFrame*> sLoopEdges = pKF->GetLoopEdges();
        for(set<KeyFrame*>::const_iterator sit=sLoopEdges.begin(), send=sLoopEdges.end(); sit!=send; sit++)
        {
            //取出回环关键帧
            KeyFrame* pLKF = *sit;
            //回环关键帧id要比当前关键帧id小
            if(pLKF->mnId<pKF->mnId)
            {
                //取出回环关键帧未矫正的位姿
                g2o::Sim3 Slw;
                LoopClosing::KeyFrameAndPose::const_iterator itl = NonCorrectedSim3.find(pLKF);
                if(itl!=NonCorrectedSim3.end())
                    Slw = itl->second;
                else
                    Slw = vScw[pLKF->mnId];
                //计算相对位姿
                g2o::Sim3 Sli = Slw * Swi;
                g2o::EdgeSim3* el = new g2o::EdgeSim3();
                //添加顶点
                el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pLKF->mnId)));
                el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
                //添加边的测量
                el->setMeasurement(Sli);
                //添加信息矩阵
                el->information() = matLambda;
                optimizer.addEdge(el);
            }
        }
        //共视图边，添加共视地图点大于100的共视关键帧和当前帧的边
        const vector<KeyFrame*> vpConnectedKFs = pKF->GetCovisiblesByWeight(minFeat);
        for(vector<KeyFrame*>::const_iterator vit=vpConnectedKFs.begin(); vit!=vpConnectedKFs.end(); vit++)
        {
            //取出回环关键帧
            KeyFrame* pKFn = *vit;
            //避免和前面的边添加重复，共视关键帧不为空、不是当前帧的父关键帧、不是当前帧的子关键帧、不是当前帧的回环关键帧
            if(pKFn && pKFn!=pParentKF && !pKF->hasChild(pKFn) && !sLoopEdges.count(pKFn))
            {
                //共视关键帧不能为坏帧，并且id要小于当前关键帧的id
                if(!pKFn->isBad() && pKFn->mnId<pKF->mnId)
                {
                    if(sInsertedEdges.count(make_pair(min(pKF->mnId,pKFn->mnId),max(pKF->mnId,pKFn->mnId))))
                        continue;
                    //取出共视关键帧未矫正的位姿
                    g2o::Sim3 Snw;
                    LoopClosing::KeyFrameAndPose::const_iterator itn = NonCorrectedSim3.find(pKFn);
                    if(itn!=NonCorrectedSim3.end())
                        Snw = itn->second;
                    else
                        Snw = vScw[pKFn->mnId];
                    //计算相对位姿
                    g2o::Sim3 Sni = Snw * Swi;
                    //添加顶点
                    g2o::EdgeSim3* en = new g2o::EdgeSim3();
                    en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFn->mnId)));
                    en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
                    en->setMeasurement(Sni);
                    en->information() = matLambda;
                    optimizer.addEdge(en);
                }
            }
        }
    }
    //优化
    optimizer.initializeOptimization();
    optimizer.optimize(20);
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);
    //更新优化后的闭环检测位姿
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        //取出关键帧
        KeyFrame* pKFi = vpKFs[i];
        //取出id
        const int nIDi = pKFi->mnId;
        //通过id取出优化结果
        g2o::VertexSim3Expmap* VSim3 = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(nIDi));
        g2o::Sim3 CorrectedSiw =  VSim3->estimate();
        //计算逆矩阵
        vCorrectedSwc[nIDi] = CorrectedSiw.inverse();
        //取出旋转、平移和尺度部分
        Eigen::Matrix3d eigR = CorrectedSiw.rotation().toRotationMatrix();
        Eigen::Vector3d eigt = CorrectedSiw.translation();
        double s = CorrectedSiw.scale();
        eigt *=(1./s); //[R t/s;0 1]
        //更新位姿
        cv::Mat Tiw = Converter::toCvSE3(eigR, eigt);
        pKFi->SetPose(Tiw);
    }
    //优化得到关键帧的位姿后，地图点根据参考帧优化前后的相对关系调整自己的位置
    for(size_t i=0, iend=vpMPs.size(); i<iend; i++)
    {
        //取出地图点
        MapPoint* pMP = vpMPs[i];
        if(pMP->isBad())
            continue;
        //取出对应关键帧id
        int nIDr;
        //如果被当前关键帧矫正过，取当前关键帧id
        if(pMP->mnCorrectedByKF==pCurKF->mnId)
        {
            nIDr = pMP->mnCorrectedReference;
        }
        //找出参考关键帧id
        else
        {
            KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();
            nIDr = pRefKF->mnId;
        }
        //取出对应关键帧的位姿
        g2o::Sim3 Srw = vScw[nIDr];
        g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];
        //计算优化后地图点的位姿
        cv::Mat P3Dw = pMP->GetWorldPos();
        Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
        Eigen::Matrix<double,3,1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));
        //设置地图点相对世界坐标系的位姿
        cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
        pMP->SetWorldPos(cvCorrectedP3Dw);
        pMP->UpdateNormalAndDepth();
    }
}

//优化求解了sim3的两关键帧，用两帧相互匹配的地图点作为固定顶点，第二帧相对第一帧的sim3位姿作为优化位姿顶点，把两帧在各自相机坐标系下的地图点和第二帧光心之间添加投影边（两关键帧对地图点的观测（投影边）是比较准确的）
int Optimizer::OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches1, g2o::Sim3 &g2oS12, const float th2, const bool bFixScale)
{
    g2o::SparseOptimizer optimizer;
    //创建线性求解器
    g2o::BlockSolverX::LinearSolverType * linearSolver;
    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
    //创建BlockSolver矩阵块求解器 求解雅克比矩阵和海塞矩阵,并用上面定义的线性求解器初始化
    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);
    //迭代算法solver，从GN,LM,DogLeg中选一个，再用BlockSolver初始化
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);
    //取出相机内参
    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;
    //取出相机位姿
    const cv::Mat R1w = pKF1->GetRotation();
    const cv::Mat t1w = pKF1->GetTranslation();
    const cv::Mat R2w = pKF2->GetRotation();
    const cv::Mat t2w = pKF2->GetTranslation();
    //设置Sim3顶点
    g2o::VertexSim3Expmap * vSim3 = new g2o::VertexSim3Expmap(); 
    //设置sim3参数   
    vSim3->_fix_scale=bFixScale;
    vSim3->setEstimate(g2oS12);//设置第二个关键帧相对于第一个关键帧的相对位姿为顶点位姿，在优化图中世界坐标系在关键帧2处。
    vSim3->setId(0);
    vSim3->setFixed(false);
    vSim3->_principle_point1[0] = K1.at<float>(0,2);
    vSim3->_principle_point1[1] = K1.at<float>(1,2);
    vSim3->_focal_length1[0] = K1.at<float>(0,0);
    vSim3->_focal_length1[1] = K1.at<float>(1,1);
    vSim3->_principle_point2[0] = K2.at<float>(0,2);
    vSim3->_principle_point2[1] = K2.at<float>(1,2);
    vSim3->_focal_length2[0] = K2.at<float>(0,0);
    vSim3->_focal_length2[1] = K2.at<float>(1,1);
    //添加sim3顶点
    optimizer.addVertex(vSim3);
    //获取匹配点的数量
    const int N = vpMatches1.size();
    //获取关键帧1的地图点
    const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
    //创建存放约束边的容器
    vector<g2o::EdgeSim3ProjectXYZ*> vpEdges12;
    vector<g2o::EdgeInverseSim3ProjectXYZ*> vpEdges21;
    vector<size_t> vnIndexEdge;
    //保留空间大小
    vnIndexEdge.reserve(2*N);
    vpEdges12.reserve(2*N);
    vpEdges21.reserve(2*N);
    const float deltaHuber = sqrt(th2);

    int nCorrespondences = 0;
    //将匹配转化为归一化3d点作为g2o的顶点
    for(int i=0; i<N; i++)
    {
        //如果对应地图点没有匹配，跳过
        if(!vpMatches1[i])
            continue;
        //取出两关键帧对应的地图点
        MapPoint* pMP1 = vpMapPoints1[i];
        MapPoint* pMP2 = vpMatches1[i];

        const int id1 = 2*i+1;
        const int id2 = 2*(i+1);
        //获取地图点2在关键帧2的索引
        const int i2 = pMP2->GetIndexInKeyFrame(pKF2);
        //如果两地图点存在，创建地图点顶点
        if(pMP1 && pMP2)
        {
            if(!pMP1->isBad() && !pMP2->isBad() && i2>=0)
            {
                //创建顶点
                g2o::VertexSBAPointXYZ* vPoint1 = new g2o::VertexSBAPointXYZ();
                //获取地图点1在关键帧1相机坐标系的位置
                cv::Mat P3D1w = pMP1->GetWorldPos();
                cv::Mat P3D1c = R1w*P3D1w + t1w;
                //设置顶点参数
                vPoint1->setEstimate(Converter::toVector3d(P3D1c));
                vPoint1->setId(id1);
                vPoint1->setFixed(true);  //地图点顶点都是固定的
                optimizer.addVertex(vPoint1);

                //创建顶点
                g2o::VertexSBAPointXYZ* vPoint2 = new g2o::VertexSBAPointXYZ();
                //获取地图点2在关键帧2相机坐标系的位置
                cv::Mat P3D2w = pMP2->GetWorldPos();
                cv::Mat P3D2c = R2w*P3D2w + t2w;
                //设置顶点参数
                vPoint2->setEstimate(Converter::toVector3d(P3D2c));
                vPoint2->setId(id2);
                vPoint2->setFixed(true);  //地图点顶点都是固定的
                optimizer.addVertex(vPoint2);
            }
            else
                continue;
        }
        else
            continue;

        nCorrespondences++;
        //添加误差项边
        //取出关键帧1对应地图点的图像坐标
        Eigen::Matrix<double,2,1> obs1;
        const cv::KeyPoint &kpUn1 = pKF1->mvKeysUn[i];
        obs1 << kpUn1.pt.x, kpUn1.pt.y;
        //创建投影边
        g2o::EdgeSim3ProjectXYZ* e12 = new g2o::EdgeSim3ProjectXYZ();
        //设置边的顶点（地图点顶点和sim3顶点）
        e12->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id2))); //注意这里边e12是从顶点id2到sim3顶点，而下面是从sim3到id1顶点（这里相当于关键帧2的地图点投影到关键帧1）
        e12->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        e12->setMeasurement(obs1);
        //根据金字塔层级，计算信息矩阵
        const float &invSigmaSquare1 = pKF1->mvInvLevelSigma2[kpUn1.octave];
        e12->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare1);
        //添加鲁棒函数
        g2o::RobustKernelHuber* rk1 = new g2o::RobustKernelHuber;
        e12->setRobustKernel(rk1);
        rk1->setDelta(deltaHuber);
        //添加边
        optimizer.addEdge(e12);

        //取出关键帧2对应地图点的图像坐标
        Eigen::Matrix<double,2,1> obs2;
        const cv::KeyPoint &kpUn2 = pKF2->mvKeysUn[i2];
        obs2 << kpUn2.pt.x, kpUn2.pt.y;
        //创建投影边
        g2o::EdgeInverseSim3ProjectXYZ* e21 = new g2o::EdgeInverseSim3ProjectXYZ();
        //设置边的顶点（地图点顶点和sim3顶点）
        e21->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id1)));  //这里相当于关键帧1的地图点投影到关键帧2
        e21->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        e21->setMeasurement(obs2);
        //根据金字塔层级，计算信息矩阵
        float invSigmaSquare2 = pKF2->mvInvLevelSigma2[kpUn2.octave];
        e21->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare2);
        //添加鲁棒函数
        g2o::RobustKernelHuber* rk2 = new g2o::RobustKernelHuber;
        e21->setRobustKernel(rk2);
        rk2->setDelta(deltaHuber);
        optimizer.addEdge(e21);
        //把两条边存到容器
        vpEdges12.push_back(e12);
        vpEdges21.push_back(e21);
        //添加边
        vnIndexEdge.push_back(i);
    }
    //优化
    optimizer.initializeOptimization();
    optimizer.optimize(5);
    //检测内点
    int nBad=0;
    //遍历边
    for(size_t i=0; i<vpEdges12.size();i++)
    {
        //取出两条边
        g2o::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
        g2o::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
        if(!e12 || !e21)
            continue;
        //判断边的误差是否大于阈值
        if(e12->chi2()>th2 || e21->chi2()>th2)
        {
            //对应匹配地图点置为空
            size_t idx = vnIndexEdge[i];
            vpMatches1[idx] = static_cast<MapPoint*>(NULL);
            //移除边
            optimizer.removeEdge(e12);
            optimizer.removeEdge(e21);
            //对应边置为空
            vpEdges12[i]=static_cast<g2o::EdgeSim3ProjectXYZ*>(NULL);
            vpEdges21[i]=static_cast<g2o::EdgeInverseSim3ProjectXYZ*>(NULL);
            nBad++;
        }
    }
    //确定再次迭代的次数
    int nMoreIterations;
    if(nBad>0)
        nMoreIterations=10;
    else
        nMoreIterations=5;
    if(nCorrespondences-nBad<10)
        return 0;
    //再次用内点优化
    optimizer.initializeOptimization();
    optimizer.optimize(nMoreIterations);
    //声明变量，记录内点数量
    int nIn = 0;
    for(size_t i=0; i<vpEdges12.size();i++)
    {
        //取出两条边
        g2o::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
        g2o::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
        if(!e12 || !e21)
            continue;
        //判断边的误差是否大于阈值
        if(e12->chi2()>th2 || e21->chi2()>th2)
        {
            //对应匹配地图点置为空
            size_t idx = vnIndexEdge[i];
            vpMatches1[idx]=static_cast<MapPoint*>(NULL);
        }
        else
            nIn++;
    }
    //用优化后的sim3覆盖原来的sim3
    g2o::VertexSim3Expmap* vSim3_recov = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(0));
    g2oS12= vSim3_recov->estimate();
    return nIn;
}


} //namespace ORB_SLAM
