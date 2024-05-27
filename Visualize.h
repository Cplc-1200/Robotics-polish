#pragma once
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
void visualize_cloud_two(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& filter_cloud);
void visualize_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
void visualize_cloud_merge(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& filter_cloud);