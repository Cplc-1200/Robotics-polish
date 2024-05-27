#ifndef GET_CAMERA_HPP
#define GET_CAMERA_HPP
#include <chrono>
#include <iostream>
#include "epiceye.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <QMessageBox>
#include <QDebug>
#include <QString>
#include <pcl/filters/filter.h>
void Get_CameraData(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
#endif // !GET_CAMERA_HPP