#include "arm_planning/collisionDetect.h"

CollisionDetection::CollisionDetection()
{
    std::cout << "碰撞检测模块:初始化碰撞检测模块！" << std::endl;
    //获取功能包路径
    std::string path_link1 = ros::package::getPath("arm_planning");
    //组合生成stl文件路径，并加载stl文件生成碰撞对象
    link1 = createMeshCollisionObject(
        (path_link1 + "/mesh/mesh_with_camera/link1.stl").data());
    link2 = createMeshCollisionObject(
        (path_link1 + "/mesh/mesh_with_camera/link2.stl").data());
    link3 = createMeshCollisionObject(
        (path_link1 + "/mesh/mesh_with_camera/link3.stl").data());
    link4 = createMeshCollisionObject(
        (path_link1 + "/mesh/mesh_with_camera/link4.stl").data());
    link5 = createMeshCollisionObject(
        (path_link1 + "/mesh/mesh_with_camera/link5.stl").data());
    link6 = createMeshCollisionObject(
        (path_link1 + "/mesh/mesh_with_camera/link6.stl").data());
    //把碰撞对象存入向量
    links.push_back(link1);
    links.push_back(link2);
    links.push_back(link3);
    links.push_back(link4);
    links.push_back(link5);
    links.push_back(link6);
    //设置自碰撞矩阵
    ACM << 0, 0, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0,
        1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    //设置碰撞对象组管理器
    // manager1->setup();
    // manager2->setup();
}
CollisionDetection::~CollisionDetection()
{
    delete link1;
    link1 = NULL;
    delete link2;
    link2 = NULL;
    delete link3;
    link3 = NULL;
    delete link4;
    link4 = NULL;
    delete link5;
    link5 = NULL;
    delete link6;
    link6 = NULL;
    delete camera;
    camera = NULL;
    delete gripper;
    gripper = NULL;
}
CollisionDetection::Face* CollisionDetection::readSTL(
    CollisionDetection::Head& head,
    const char fileName[])
{
    FILE* fp;  //声明文件流指针
    fp =
        fopen(fileName, "rb");  //以二进制方式打开路径所在文件，初始化文件流指针
    if (fp != NULL) {
        fread(head.partName,
              80,
              1,
              fp);  //读取stl文件前80字节，存入head.partName里，表示零件名
        fread(
            &head.faceNum,
            4,
            1,
            fp);  //读取stl文件81到84字节，存入head.faceNum里，表示三角面片的数目
    } else {
        std::cout << "碰撞检测模块:打开stl文件错误!" << std::endl;
    }
    Face* faces = (Face*) malloc(
        head.faceNum *
        sizeof(Face));  //根据三角面片的数目，创建一个Face类型的数组
    //循环读取三角面片数据
    for (int i = 0; i < head.faceNum; i++) {
        fread(&faces[i].normal, 12, 1, fp);  //读取12字节的法线数据
        fread(&faces[i].p1, 12, 1, fp);      //读取12字节的顶点1的数据
        fread(&faces[i].p2, 12, 1, fp);      //读取12字节的顶点2的数据
        fread(&faces[i].p3, 12, 1, fp);      //读取12字节的顶点3的数据
        fread(
            &faces[i].info,
            2,
            1,
            fp);  //读取2字节的保留项数据，这一项一般没什么用，这里选择读取是为了移动文件指针
    }
    fclose(fp);  //关闭文件
    // for (int i = 0;i < head.faceNum;i++)
    // {
    // 	printf("面%d\n", i + 1);
    // 	printf("法线：\n");
    // 	printf("%f  %f  %f\n", faces[i].normal.i, faces[i].normal.j,
    // faces[i].normal.k); 	printf("顶点数据：\n"); 	printf("%f  %f
    // %f\n", faces[i].p1.x, faces[i].p1.y, faces[i].p1.z); 	printf("%f  %f
    // %f\n", faces[i].p2.x, faces[i].p2.y, faces[i].p2.z); 	printf("%f  %f
    // %f\n", faces[i].p2.x, faces[i].p2.y, faces[i].p2.z);
    // }
    return faces;
}

fcl::CollisionObjectd* CollisionDetection::createMeshCollisionObject(
    const char fileName[])
{
    typedef fcl::BVHModel<fcl::OBBd>
        Model;  //定义obb包围盒类型组成的层次包围盒类型，BVH是树形结构根节点包围整个物体，子节点包围某个几何，这样有助于加速检测
    Model* model = new Model();  //生成层次包围盒模型Model，按层次进行碰撞检测
    std::shared_ptr<fcl::CollisionGeometryd> model_ptr(
        model);  //生成碰撞检测几何体指针
    model->beginModel();
    Head head;
    Face* faces =
        readSTL(head, fileName);  //根据文件路径读取三角面数量和三角面顶点等数据
    for (size_t i = 0; i < head.faceNum; i++) {
        fcl::Vector3<double> pp1{(double) faces[i].p1.x,
                                 (double) faces[i].p1.y,
                                 (double) faces[i].p1.z};
        fcl::Vector3<double> pp2{(double) faces[i].p2.x,
                                 (double) faces[i].p2.y,
                                 (double) faces[i].p2.z};
        fcl::Vector3<double> pp3{(double) faces[i].p3.x,
                                 (double) faces[i].p3.y,
                                 (double) faces[i].p3.z};
        model->addTriangle(pp1, pp2, pp3);  //添加顶点坐标到模型
    }
    model->endModel();
    int a = model->getNumBVs();
    std::cout << "碰撞检测模块:网格三角数量为:" << a << "个" << std::endl;
    model->computeLocalAABB();
    return new fcl::CollisionObjectd(
        model_ptr);  //通过model_ptr生成目标物体对应的碰撞检测对象
}

bool CollisionDetection::isStateValid(const std::vector<double>& theta_input)
{
    //根据输入的各关节角度，调用正运动学函数计算各个关节的位姿
    Eigen::Matrix4d transform1 = kin.kinematics(theta_input, 1);
    Eigen::Matrix4d transform2 = kin.kinematics(theta_input, 2);
    Eigen::Matrix4d transform3 = kin.kinematics(theta_input, 3);
    Eigen::Matrix4d transform4 = kin.kinematics(theta_input, 4);
    Eigen::Matrix4d transform5 = kin.kinematics(theta_input, 5);
    Eigen::Matrix4d transform6 = kin.kinematics(theta_input, 6);
    //给各个碰撞检测对象设置计算出的位姿
    link1->setTransform(transform1.block<3, 3>(0, 0),
                        transform1.topRightCorner(3, 1));
    link2->setTransform(transform2.block<3, 3>(0, 0),
                        transform2.topRightCorner(3, 1));
    link3->setTransform(transform3.block<3, 3>(0, 0),
                        transform3.topRightCorner(3, 1));
    link4->setTransform(transform4.block<3, 3>(0, 0),
                        transform4.topRightCorner(3, 1));
    link5->setTransform(transform5.block<3, 3>(0, 0),
                        transform5.topRightCorner(3, 1));
    link6->setTransform(transform6.block<3, 3>(0, 0),
                        transform6.topRightCorner(3, 1));
    //生成障碍物对象
    auto box1 =
        std::make_shared<fcl::Boxd>(1.0, 1.0, 1.0);  //生成box类型的几何体
    auto obstacle1 =
        std::make_shared<fcl::CollisionObjectd>(box1);  //生成碰撞检测对象
    fcl::Vector3d translation1(
        0,
        0,
        -0.5);  //次序:x,y,z  给定几何体的坐标系位于几何体中心//0.3, 0, 0.15
    fcl::Quaterniond rotation1(1, 0, 0, 0);            //四元数次序:w,x,y,z
    obstacle1->setTransform(rotation1, translation1);  //给碰撞检测物体设置位姿
    fcl::CollisionObjectd* obstacle_ptr =
        obstacle1.get();  // obstacle1是智能指针(对象)，get函数获取指针
    static int index1 = 0;
    static int index2 = 0;
    //进行机械臂与障碍物的碰撞检测，从最边缘的关节进行检测
    for (int i = 5; i >= 0; i--) {
        fcl::collide(
            links[i],
            obstacle_ptr,
            collisionRequest,
            collisionResult);  //调用collide函数进行碰撞检测，结果保存在collisionResult中
        if (collisionResult
                .isCollision()) {  //获取collisionResult中的碰撞检测结果
            collisionResult.clear();
            index1++;
            std::cout << "碰撞检测模块:检测到机械臂与障碍物发生碰撞!"
                         "当前与障碍物碰撞次数为"
                      << index1 << std::endl;
            return 0;
        } else {
            continue;
        }
    }
    //机械臂各关节的自碰撞检测，按照自碰撞矩阵确定是否进行两关节间的检测
    for (int i = 0; i < 6; i++) {
        for (int j = i + 1; j < 6; j++) {
            if (ACM(i, j) == 1)  // ACM矩阵中值为1，表示对应两关节要进行碰撞检测
            {
                fcl::collide(
                    links[i], links[j], collisionRequest, collisionResult);
                if (collisionResult.isCollision()) {
                    collisionResult.clear();
                    index2++;
                    std::cout << "碰撞检测模块:检测到机械臂发生自碰撞!"
                                 "当前自碰撞次数为: "
                              << index2 << std::endl;
                    return 0;
                }
            } else {
                continue;
            }
        }
    }
    return 1;
}
