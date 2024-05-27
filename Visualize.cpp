#include "Visualize.h"
void visualize_cloud_two(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& filter_cloud) 
{
    //---------显示点云-----------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("显示点云"));

    int v1(0), v2(0);

    viewer->createViewPort(0.0, 0.0, 0.5, 1, v1);
    viewer->setBackgroundColor(0, 0, 0, v1);
    // 设置显示比例
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.5, "sample cloud", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.5, "cloud_filtered", v2);

    viewer->addText("point clouds", 10, 10, "v1_text", v1);
    viewer->createViewPort(0.5, 0.0, 1, 1, v2);
    viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
    viewer->addText("filtered point clouds", 10, 10, "v2_text", v2);

    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud", v1);
    viewer->addPointCloud<pcl::PointXYZ>(filter_cloud, "cloud_filtered", v2);
   
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sample cloud", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud_filtered", v2);
    //viewer->addCoordinateSystem(1.0);
    //viewer->initCameraParameters();
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}
void visualize_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    int v1(0), v2(0);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("显示点云"));
    viewer->setBackgroundColor(255, 255, 255);
    viewer->setWindowName("点云显示");
    //viewer->addText("point clouds", 10, 10, "v1_text");
   // viewer->addText("filtered point clouds", 10, 10, "v2_text");
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, "sample cloud", v1);
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}
void visualize_cloud_merge(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& filter_cloud)
{
    //---------显示点云-----------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("显示点云"));

    int v1(0), v2(0);

   
    viewer->setBackgroundColor(0, 0, 0);

    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud", v1);
    viewer->addPointCloud<pcl::PointXYZ>(filter_cloud, "cloud_filtered", v2);

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sample cloud", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud_filtered", v2);
    //viewer->addCoordinateSystem(1.0);
    //viewer->initCameraParameters();
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}