#pragma once
#include <ros/package.h>

#include <Eigen/Dense>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <iostream>
#include <vector>

#include "arm_planning/kinematics.h"
#include "fcl/broadphase/broadphase_collision_manager.h"
#include "fcl/broadphase/broadphase_dynamic_AABB_tree.h"
#include "fcl/broadphase/default_broadphase_callbacks.h"
#include "fcl/common/types.h"
#include "fcl/config.h"
#include "fcl/geometry/shape/box.h"
#include "fcl/geometry/shape/sphere.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/collision_object.h"
#include "fcl/narrowphase/collision_request.h"
#include "fcl/narrowphase/collision_result.h"
class CollisionDetection
{
  private:
    //文件头，共84字节
    struct Head
    {
        char partName[80];  //零件名称
        int faceNum;        //面的数目
    };
    //点，三个float类型的，大小为12字节
    struct Point
    {
        float x;
        float y;
        float z;
    };
    //法线
    struct Normal
    {
        float i;
        float j;
        float k;
    };
    //三角面，由一个法线，三个点，和一个两字节的保留项，一共50字节
    struct Face
    {
        Normal normal;
        Point p1;
        Point p2;
        Point p3;
        char info[2];  //保留数据，可以不用管
    };

  private:
    Kinematics kin;
    fcl::CollisionObjectd* link1;
    fcl::CollisionObjectd* link2;
    fcl::CollisionObjectd* link3;
    fcl::CollisionObjectd* link4;
    fcl::CollisionObjectd* link5;
    fcl::CollisionObjectd* link6;
    fcl::CollisionObjectd* camera;
    fcl::CollisionObjectd* gripper;
    std::vector<fcl::CollisionObjectd*> links;
    Eigen::Matrix<int, 6, 6> ACM;
    fcl::CollisionRequestd collisionRequest;
    fcl::CollisionResultd collisionResult;

  public:
    CollisionDetection();
    ~CollisionDetection();
    Face* readSTL(Head& head, const char fileName[]);
    fcl::CollisionObjectd* createMeshCollisionObject(const char fileName[]);
    class manager : public fcl::DynamicAABBTreeCollisionManagerd
    {
      public:
        void insertTestedSet(fcl::CollisionObjectd* a, fcl::CollisionObjectd* b)
        {
            fcl::BroadPhaseCollisionManagerd::insertTestedSet(a, b);
        }
    };
    bool isStateValid(const std::vector<double>& theta_input);

  private:
    manager* manager1 = new manager();
    manager* manager2 = new manager();
};
void unitTest();
