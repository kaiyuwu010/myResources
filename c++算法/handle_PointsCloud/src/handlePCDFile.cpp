#include <iostream> //标准c++库输入输出相关头文件
#include <unistd.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/shared_ptr.hpp>
#include <pcl/io/pcd_io.h> // pcd读写相关头文件
#include <pcl/point_types.h> // pcl中支持的点类型头文件
#include <pcl/visualization/cloud_viewer.h> // pcl可视化显示头文件
#include <pcl_ros/point_cloud.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "handlePCDFile");
    // 创建点云对象
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    int c;
    // 参数s表示保存某个点云话题的一帧点云，参数v表示可视化某个路径的pcd文件
    while ((c = getopt(argc, argv, "s:v:")) != -1)
    {
        switch (c)
        {
            case 's':
            {            
                // 获取话题消息
                boost::shared_ptr<sensor_msgs::PointCloud2 const> msg_pc;
                msg_pc = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(std::string(optarg), ros::Duration(0.5));
                if( msg_pc!= NULL)
                {
                    pcl::fromROSMsg(*msg_pc, *point_cloud);
                }
                else
                {
                    PCL_ERROR("Couldn't get points cloud msg! \n");
                    return 0;
                }
                // 保存点云文件
                pcl::io::savePCDFile("points_cloud.pcd", *point_cloud);
                break;
            }
            case 'v':
            {
                // 读取PCD文件
                if (pcl::io::loadPCDFile<pcl::PointXYZI>(std::string(optarg), *point_cloud) == -1)
                {
                    PCL_ERROR("Couldn't read file! \n");
                    return (-1);
                }
                // 可视化点云
                pcl::visualization::CloudViewer viewer("Cloud Viewer");
                viewer.showCloud(point_cloud);
                // 等待直到可视化窗口关闭
                while (!viewer.wasStopped()){}
                break;
            }
            case '?':
            {
                printf("未知的选项: '%c'\n", c);
                break;
            }
        }
    }
    return (0);
}
