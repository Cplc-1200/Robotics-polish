#pragma once
#ifndef PCL_FUNCTION_H
#define PCL_FUNCTION_H

#include <boost/thread/thread.hpp>
#include <QDebug>
//-----------------------------pcl------------------------------------
/**********************************pcl格式*****************************/
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/common/common.h>
/**********************************pcl显示***************************/
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>


//---------------------------体素滤波下采样----------------------------------
//#include <pcl/filters/extract_indices.h>   // 根据索引提取点云
#include <pcl/filters/voxel_grid.h>            // 体素滤波
void grid_filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& filter_cloud, const float num);

// -------------------------------聚类分割-------------------------------------
#include <pcl/kdtree/kdtree.h>                 
#include <pcl/segmentation/extract_clusters.h> // 欧式聚类
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_point(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster_cloud);

// -------------------------------特征点的孔洞边界提取-------------------------------------
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/boundary.h> //边界提取
std::vector< int> boundary_p(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& bound_p, double pi);
pcl::PointCloud<pcl::PointXYZ>::Ptr hole_extra(pcl::PointCloud<pcl::PointXYZ>::Ptr& orign_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<int>& bound_indice, pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster_cloud);

// -------------------------------点云平滑-------------------------------------
#include <pcl/filters/convolution_3d.h>  // 高斯滤波
#include <pcl/surface/mls.h>
#include <pcl/filters/statistical_outlier_removal.h>  //离群点
void guass_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& filter_cloud, float sigma = 2);
void least_squares(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_ori, pcl::PointCloud<pcl::PointXYZ>::Ptr& filter_cloud, float radii);

//---------------------------------孔洞修补---------------------
#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>
#include <pcl/surface/on_nurbs/triangulation.h>
pcl::PointCloud<pcl::PointXYZ>::Ptr fill_hole(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double mean, pcl::PolygonMesh& total_mesh);
void hole_fill(pcl::PointCloud<pcl::PointXYZ>::Ptr& orign_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& bound_cloud, std::vector<int>& bound_indice, pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud);

//--------------------------------三维重建----------------------
#include <vtkPLYWriter.h>
#include <vtkDelaunay2D.h>
void delau_recon(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PolygonMesh::Ptr& output_cloud, bool if_best = true);

//*********************************************************************************************
//---------------------------去重与离群点--------------------------
pcl::PointCloud<pcl::PointXYZ>::Ptr delete_point(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& delete_cloud);
void deduplication(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& filter_cloud, float num);
void outlier_removal(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& filter_cloud, float thresh);

//************************************轨迹生成*************************************************
#include <igl/boundary_loop.h>
#include <igl/readPLY.h>
#include <igl/writePLY.h>
#include <igl/writeOBJ.h>
#include <igl/writeSTL.h>
#include <igl/principal_curvature.h>
#include <igl/barycentric_coordinates.h>//面积坐标
//#include <igl/lscm.h>
#include <igl/bounding_box.h>
#include <igl/is_edge_manifold.h>
#include <igl/fast_find_intersections.h>
#include <igl/point_mesh_squared_distance.h>
#include <igl/per_vertex_normals.h>
#include <igl/fit_cubic_bezier.h>
#include <pcl/common/transforms.h>
#include <igl/centroid.h>
#include "Visualize.h"
void param_curve(const std::string& mesh_name, pcl::PointCloud<pcl::PointXYZ>::Ptr& curve, Eigen::MatrixXd& tran_v, std::vector<Eigen::MatrixXd>& rotation);

void one2four(Eigen::VectorXd& x_vector, Eigen::VectorXd& y_vector);

void decide_sort(Eigen::VectorXd& x_vector, Eigen::VectorXd& y_vector);

//************************************点云配准*************************************************
#include <pcl/filters/passthrough.h>

#include <pcl/registration/icp.h> //icp头文件
#include <pcl/filters/filter.h>
#include <pcl/features/fpfh_omp.h>     // fpfh加速计算的omp(多核并行计算)
#include <pcl/registration/ia_ransac.h>// sac_ia算法
#include <pcl/registration/ndt.h> //ndt头文件
//#include <pcl/registration/ia_kfpcs.h> //K4PCS算法头文件 ************源代码编译有问题***************
#include <pcl/recognition/ransac_based/trimmed_icp.h>
#include <pcl/common/centroid.h>// 计算质心的头文件或者用 #include <pcl/common/transforms.h>
//#include <pcl/common/transforms.h>
#include <pcl/registration/sample_consensus_prerejective.h>//　随机采样一致性配准
#include <pcl/registration/ppf_registration.h>
#include <pcl/features/ppf.h>


#include <pcl/features/ppf.h>
#include <pcl/registration/ppf_registration.h>

using namespace std;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// 预处理过程



#endif // PCL_FUNCTION_H