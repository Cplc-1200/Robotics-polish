#include "pcl_function.h"
using namespace std;
//---------------------------单个点云去重--------------------------
void deduplication(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& filter_cloud, float num)
{
	//---------------------------重复点去除-----------------------------------
	//---------------------------KD树半径搜索---------------------------------
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree2(new pcl::search::KdTree<pcl::PointXYZ>);
	kdtree2->setInputCloud(cloud);

	std::vector<int>nn_indices; // 存放近邻索引
	std::vector<float>nn_dists; // 存放近邻距离
	float radius = num;    // 距离阈值，若两点之间的距离为0.000001则认为是重合点
	std::vector<bool>label(cloud->size(), false); // 初始化点的标签

	// --------------对cloud中的每个点与邻域内的点进行比较-------------------
	for (int i = 0; i < cloud->size(); ++i)
	{
		if (label[i])
		{
			continue;
		}
		if (kdtree2->radiusSearch(cloud->points[i], radius, nn_indices, nn_dists) > 0)
		{
			for (auto& pi : nn_indices)
			{
				label[pi] = true;
			}
		}
		filter_cloud->push_back(cloud->points[i]);
	}
	qDebug() << "原始点云中有：" << cloud->size() << "个点";
	qDebug() << "去重之后有：" << filter_cloud->size() << "个点";
	qDebug() << "重复点有：" << cloud->size() - filter_cloud->size() << "个";
}

//---------------------------两个点云间去重--------------------------
pcl::PointCloud<pcl::PointXYZ>::Ptr delete_point(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& delete_cloud)
{
	pcl::StopWatch time;
	//---------------------------KD树半径搜索---------------------------------
	pcl::search::KdTree <pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);

	std::vector<int>nn_indices; // 存放近邻索引
	std::vector<float>nn_dists; // 存放近邻距离
	float radius = 0.01;    // 距离阈值，若两点之间的距离为0.01则认为是重合点
	std::vector<bool>label(cloud->size(), false); // 初始化点的标签
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
	// --------------对cloud中的每个点与邻域内的点进行比较-------------------
	for (int i = 0; i < cloud->size(); ++i)
	{
		if (i < delete_cloud->size())
		{
			if (tree.radiusSearch(delete_cloud->points[i], radius, nn_indices, nn_dists) >= 0)
			{
				for (auto& pi : nn_indices)
				{
					label[pi] = true;
				}
			}
		}
		if (label[i])
		{
			continue;
		}
		filtered->push_back(cloud->points[i]);
	}
	qDebug() << "点云去重用时：" << time.getTimeSeconds() << " s";
	qDebug() << "原始点云中有：" << cloud->size() << "个点";
	qDebug() << "去重之后有：" << filtered->size() << "个点";
	qDebug() << "重复点有：" << cloud->size() - filtered->size() << "个";

	//pcl::io::savePCDFileBinary("ro.pcd", *filtered);
	////---------------------------结果可视化----------------------------------
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	//viewer->setWindowName("删除点云中重叠的点");
	//viewer->addPointCloud<pcl::PointXYZ>(filtered, "sample cloud");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 1, "sample cloud");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.7, "sample cloud");// 设置透明度
	//while (!viewer->wasStopped())
	//{
	//	viewer->spinOnce(100);
	//	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	//}
	return filtered;
}

//*********************************************************************************************
//---------------------------体素滤波下采样----------------------------------
void grid_filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& filter_cloud, const float num)
{
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud(cloud);
	vg.setLeafSize(num, num, num);
	vg.filter(*filter_cloud);
	qDebug() << "体素滤波后还有: " << filter_cloud->points.size() << " 个.";
}

// -------------------------------聚类分割-------------------------------------
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_point(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster_cloud)
{
	// -------------------------------------------欧式聚类--------------------------------------------
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	std::vector<pcl::PointIndices> cluster_indices; // 聚类索引
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;// 欧式聚类对象
	ec.setClusterTolerance(1.5);               // 设置近邻搜索的搜索半径（也即两个不同聚类团点之间的最小欧氏距离）
	ec.setMinClusterSize(50);                 // 设置一个聚类需要的最少的点数目为100
	ec.setMaxClusterSize(cloud->size() * 0.8);               // 设置一个聚类需要的最大点数目为25000
	ec.setSearchMethod(tree);                  // 设置点云的搜索机制
	ec.setInputCloud(cloud);                   // 设置输入点云
	ec.extract(cluster_indices);               // 从点云中提取聚类，并将点云索引保存在cluster_indices中
	//------------------------------------保存聚类结果并可视化---------------------------------------
	//boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("planar segment")); ;
	//viewer->setBackgroundColor(0, 0, 0);
	//viewer->setWindowName("欧式聚类");
	int begin = 0;
	int maxsize = 0;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cluster_rgb_result(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_max(new pcl::PointCloud<pcl::PointXYZ>);
	for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (auto pit = it->indices.begin(); pit != it->indices.end(); pit++)
		{
			cloud_cluster->push_back(cloud->points[*pit]);
		}
		if (cloud_cluster->size() > maxsize)
		{
			maxsize = cloud_cluster->size();
			cloud_cluster_max = cloud_cluster;
		}
		/* std::stringstream ss;
		 ss << "Euclidean_cluster_" << begin + 1 << ".pcd";
		 pcl::io::savePCDFileBinary(ss.str(), *cloud_cluster);
		 qDebug()<< ss.str() << "保存完毕！！！" ;*/
		begin++;
		// 可视化相关的代码
		uint8_t R = rand() % (256) + 0;
		uint8_t G = rand() % (256) + 0;
		uint8_t B = rand() % (256) + 0;
		for (size_t i = 0; i < cloud_cluster->size(); i++)
		{
			cluster_rgb_result->points.push_back(pcl::PointXYZRGB(cloud_cluster->points.at(i).x, cloud_cluster->points.at(i).y, cloud_cluster->points.at(i).z, R, G, B));
		}
		//string str;
		/*ss >> str;*/
		//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud_cluster, R, G, B);
		// viewer->addPointCloud<pcl::PointXYZ>(cloud_cluster, color, str);
		 //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, str);
	}

	cluster_cloud = cloud_cluster_max;
	return cluster_rgb_result;
	/*while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}*/
}

// -------------------------------特征点的孔洞边界提取-------------------------------------
std::vector< int> boundary_p(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& bound_p, double pi)
{
	pcl::StopWatch time;
	//------------------------计算法向量---------------------------
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setRadiusSearch(3);
	n.setNumberOfThreads(8);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	n.compute(*normals);
	//-----------------------边界特征估计--------------------------
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst;
	boundEst.setInputCloud(cloud);
	boundEst.setInputNormals(normals);
	boundEst.setRadiusSearch(3);
	boundEst.setAngleThreshold(pi);//边界判断时的角度阈值
	boundEst.setSearchMethod(tree);
	pcl::PointCloud<pcl::Boundary> boundaries;
	boundEst.compute(boundaries);
	std::vector< int>  bound_indice;
	for (int i = 0; i < boundaries.size(); i++)
	{
		if (boundaries[i].boundary_point > 0)
		{
			bound_indice.push_back(i);
			bound_p->points.push_back(cloud->points[i]);
		}
	}
	qDebug() << "边界点个数:" << bound_indice.size();
	//pcl::io::savePCDFileASCII("YY11.pcd", *cloud_boundary);
	qDebug() << "边界提取算法运行时间:" << time.getTimeSeconds() << "秒";
	return bound_indice;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr hole_extra(pcl::PointCloud<pcl::PointXYZ>::Ptr& orign_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<int>& bound_indice, pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster_cloud)
{
	// -------------------------------------------欧式聚类--------------------------------------------
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	std::vector<pcl::PointIndices> cluster_indices; // 聚类索引
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;// 欧式聚类对象
	ec.setClusterTolerance(1.3);               // 设置近邻搜索的搜索半径（也即两个不同聚类团点之间的最小欧氏距离）
	ec.setMinClusterSize(2);                 // 设置一个聚类需要的最少的点数目为100
	ec.setMaxClusterSize(cloud->size() * 0.8);               // 设置一个聚类需要的最大点数目为25000
	ec.setSearchMethod(tree);                  // 设置点云的搜索机制
	ec.setInputCloud(cloud);                   // 设置输入点云
	ec.extract(cluster_indices);               // 从点云中提取聚类，并将点云索引保存在cluster_indices中
	tree.reset();
	//------------------------------------保存聚类结果并可视化---------------------------------------
	//boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("planar segment")); ;
	//viewer->setBackgroundColor(0, 0, 0);
	//viewer->setWindowName("欧式聚类");
	int maxsize = 0;
	int point_count = 0;
	vector<pcl::PointIndices>::iterator i_indices;
	for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		point_count += it->indices.size();
		if (it->indices.size() > maxsize)
		{
			maxsize = it->indices.size();
			i_indices = it;
		}
	}
	point_count -= maxsize;
	//存储最外围边界的点云数据
	pcl::PointCloud<pcl::PointXYZ>::Ptr out_bound(new pcl::PointCloud<pcl::PointXYZ>);
	for (auto ppit = i_indices->indices.begin(); ppit != i_indices->indices.end(); ppit++)
	{
		out_bound->points.push_back(cloud->points[*ppit]);
	}
	cluster_indices.erase(i_indices);
	qDebug() << cluster_indices.size() << "聚类数量";
	int mean = point_count / cluster_indices.size();
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
	//建立kd-tree
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree1(new pcl::search::KdTree<pcl::PointXYZ>);
	//建立kdtree对象
	kdtree1.setInputCloud(orign_cloud); //设置需要建立kdtree的点云指针
	int num_out_b = out_bound->size();
	for (size_t i = 0; i < num_out_b; i++)
	{
		vector<int> pointIdxRadiusSearch;  //保存每个近邻点的索引
		vector<float> pointRadiusSquaredDistance;  //保存每个近邻点与查找点之间的欧式距离平方
		if (kdtree1.radiusSearch(out_bound->points[i], 5, pointIdxRadiusSearch, pointRadiusSquaredDistance))
		{
			//50.0 * mean / it->indices.size())
			for (size_t i = 0; i < pointIdxRadiusSearch.size(); i++)
			{
				if (pointRadiusSquaredDistance[i] > 0)
				{
					//qDebug()<< "( distance: " << sqrt(pointRadiusSquaredDistance[i]) << ")" ;
					//pointRadiusSquaredDistance求出来的是对应点之间距离的平方，单位是米。这里求的是对应点之间的距离，单位是厘米。
					out_bound->push_back(orign_cloud->points[pointIdxRadiusSearch[i]]);
				}
			}
		}
	}

	for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		//过小点集的膨胀
			//半径搜索
		for (auto pit = it->indices.begin(); pit != it->indices.end(); pit++)
		{
			if (mean / 10 < it->indices.size() && it->indices.size() < 1.8 * mean)//如果小于二分之一均值，边界扩充
			{
				//qDebug()<< bound_indice.at(*pit) ;
				pcl::PointXYZ searchPoint = orign_cloud->points[bound_indice.at(*pit)]; //设置查找点
				vector<int> pointIdxRadiusSearch;  //保存每个近邻点的索引
				vector<float> pointRadiusSquaredDistance;  //保存每个近邻点与查找点之间的欧式距离平方
				float radius = 4.0 * mean / it->indices.size();  //设置查找半径范围
				/*
				qDebug()<< "K nearest neighbor search at (" << searchPoint.x
					<< " " << searchPoint.y
					<< " " << searchPoint.z<<")";
					*/
				if (kdtree1.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance, 60))
				{
					//50.0 * mean / it->indices.size())
					for (size_t i = 0; i < pointIdxRadiusSearch.size(); i++)
					{
						if (pointRadiusSquaredDistance[i] > 0)
						{
							//qDebug()<< "( distance: " << sqrt(pointRadiusSquaredDistance[i]) << ")" ;
							//pointRadiusSquaredDistance求出来的是对应点之间距离的平方，单位是米。这里求的是对应点之间的距离，单位是厘米。
							filtered->push_back(orign_cloud->points[pointIdxRadiusSearch[i]]);
						}
					}
				}
			}
			filtered->push_back(orign_cloud->points[bound_indice.at(*pit)]);
			//qDebug()<< "R = 0.1近邻点个数：" << pointIdxRadiusSearch.size() ;
		}
	}
	deduplication(filtered, cluster_cloud, 0.01);
	pcl::PointCloud<pcl::PointXYZ>::Ptr dedup_out_bound(new pcl::PointCloud<pcl::PointXYZ>);
	deduplication(out_bound, dedup_out_bound, 0.01);
	pcl::PointCloud<pcl::PointXYZ>::Ptr dedup_out_bound_1(new pcl::PointCloud<pcl::PointXYZ>);
	grid_filter(dedup_out_bound, dedup_out_bound_1, 1);
	qDebug() << "去重后的边界点云数量" << dedup_out_bound_1->size();
	return dedup_out_bound_1;
	//     std::stringstream ss;
	//     ss << "Euclidean_cluster_" << begin + 1 << ".pcd";
	//     //pcl::io::savePCDFileBinary(ss.str(), *cloud_cluster);
	//     //qDebug()<< ss.str() << "保存完毕！！！" ;
	//    begin++;
	//    // 可视化相关的代码
	//    uint8_t R = rand() % (256) + 0;
	//    uint8_t G = rand() % (256) + 0;
	//    uint8_t B = rand() % (256) + 0;
	//    string str;
	//    ss >> str;
	//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud_cluster, R, G, B);
	//     viewer->addPointCloud<pcl::PointXYZ>(cloud_cluster, color, str);
	//     viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, str);
	//}
	//while (!viewer->wasStopped())
	//{
	//    viewer->spinOnce(100);
	//    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	//}

}

// -------------------------------点云平滑-------------------------------------
void guass_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& filter_cloud, float sigma)
{
	pcl::StopWatch time;
	// ---------------------------- - 基于高斯核函数的卷积滤波实现-------------------------- -
	pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ> kernel;
	kernel.setSigma(sigma);//高斯函数的标准方差，决定函数的宽度
	kernel.setThresholdRelativeToSigma(3);//设置相对Sigma参数的距离阈值
	kernel.setThreshold(0.2);//设置距离阈值，若点间距离大于阈值则不予考虑
	qDebug() << "Kernel made";

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	qDebug() << "KdTree made";

	// -------------------------------设置Convolution 相关参数-----------------------------
	pcl::filters::Convolution3D<pcl::PointXYZ, pcl::PointXYZ, pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>> convolution;
	convolution.setKernel(kernel);//设置卷积核
	convolution.setInputCloud(cloud);
	convolution.setNumberOfThreads(8);
	convolution.setSearchMethod(tree);
	convolution.setRadiusSearch(3);
	qDebug() << "Convolution Start";
	convolution.convolve(*filter_cloud);
	qDebug() << "高斯滤波算法运行时间:" << time.getTimeSeconds() << "秒";
}

void outlier_removal(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& filter_cloud, float thresh)
{
	pcl::StopWatch time;
	// -----------------统计滤波-------------------
  // 创建滤波器，对每个点分析的临近点的个数设置为50 ，并将标准差的倍数设置为1  这意味着如果一
  // 个点的距离超出了平均距离一个标准差以上，则该点被标记为离群点，并将它移除，存储起来
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);   //设置待滤波的点云
	sor.setMeanK(10);           //设置在进行统计时考虑查询点邻近点数
	sor.setKeepOrganized(false);	// 设置true则保持点云的结构
	sor.setStddevMulThresh(thresh);  //设置判断是否为离群点的阈值，里边的数字表示标准差的倍数，1个标准差以上就是离群点。
	//即：当判断点的k近邻平均距离(mean distance)大于全局的1倍标准差+平均距离(global distances mean and standard)，则为离群点。
	sor.filter(*filter_cloud); //存储内点
	qDebug() << "滤除点数量" << cloud->size() - filter_cloud->size() << "outlier_removal运行时间" << time.getTimeSeconds() << "s";
}

void least_squares(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_ori, pcl::PointCloud<pcl::PointXYZ>::Ptr& filter_cloud, float radii)
{
	pcl::StopWatch time;
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//outlier_removal(cloud_ori, cloud,0.5);
	// -------------------------------点云平滑和数据重采样-------------------------------
	//MLS:MoveingLeastSquares(动态最小二乘法，后续查看API得知底层算法)
	//Step1：创建KDTree对象
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

	//Step2: 由于MLS功能包会同时计算点云的法向，因此创建一个用来存储包含法向信息的点云对象
	pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>);
	//Step3：创建MLS对象
	/*注意！ MovingLeastSquares模板类的第一个参数：将要被处理的点云类型，在输出这个模板类的时候，点云XYZ内容将被平滑；第二个参数：输出只包含法线的点云。*/
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::Ptr mls(new pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>);
	mls->setComputeNormals(true);  //设置在最小二乘计算中是否需要存储计算的法线
	//Step4：设置是否使用多项式拟合提高精度

	//Step5: 使用MLS进行平顺
	mls->setInputCloud(cloud_ori);
	mls->setNumberOfThreads(8);
	mls->setPolynomialOrder(2);  //设置多项式的最高阶数为2阶
	mls->setSearchMethod(tree);
	mls->setSearchRadius(radii); //设置在进行Kdtree中K邻域，的查找半径
	mls->setDilationIterations(5);
	mls->process(*mls_points);
	qDebug() << "最小二乘平滑算法运行时间:" << time.getTimeSeconds() << "秒";
	//Step7:存储点云到PCD文件
	pcl::io::savePCDFile("bun0_mls.pcd", *mls_points);
	std::cerr << " MLS completed, please check the viewer and PCD file for detail." << std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr tran_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("bun0_mls.pcd", *tran_cloud);

	outlier_removal(tran_cloud, filter_cloud, 5);

	//// 计算结果可视化
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("CloudShow"));
	//viewer->setBackgroundColor(0, 0, 0);
	//viewer->setWindowName("MLS计算点云法向量并显示");
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> v(mls_points, 0, 255, 0);
	//viewer->addPointCloud<pcl::PointNormal>(mls_points, v, "point");
	//viewer->addPointCloudNormals<pcl::PointNormal>(mls_points, 1, 10, "normal");

	//while (!viewer->wasStopped())
	//{
	//	viewer->spinOnce(100);
	//	boost::this_thread::sleep(boost::posix_time::microseconds(1000));
	//}

}

//---------------------------------孔洞修补---------------------
void PointCloud2Vector2d(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::on_nurbs::vector_vec3d& data)
{
	for (const auto& p : *cloud)
	{
		if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
			data.emplace_back(p.x, p.y, p.z);
	}
}

pcl::PointCloud<pcl::PointXYZ>::Ptr fill_hole(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double mean,pcl::PolygonMesh& total_mesh)
{
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("显示点云"));

	//viewer->setBackgroundColor(0, 0, 0);

	//fromPCLPointCloud2(cloud2, *cloud);
	pcl::on_nurbs::NurbsDataSurface data;
	qDebug() << cloud->size();
	PointCloud2Vector2d(cloud, data.interior);
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler(cloud, 0, 255, 0);
	//viewer->removePointCloud("cloud_cylinder");
	//viewer->addPointCloud<pcl::PointXYZ>(cloud, handler, "cloud_cylinder");

	//-----------B样条曲面重建------------------------

	// -----B样条曲面拟合的参数-----------------------
	unsigned order(3);//B样条曲面的模型多项式的阶数
	unsigned refinement(2);//拟合优化的迭代次数
	unsigned iterations(10);//完成拟合优化后的迭代次数
	unsigned mesh_resolution(9 * cloud->size() / mean);//每个参数方向上的采样点个数，用于对拟合得到的B样条曲面进行三角化
	bool two_dim = true;

	pcl::on_nurbs::FittingSurface::Parameter params;

	params.interior_smoothness = 0.2;//描述曲面本身的平滑性
	params.interior_weight = 1.0;//拟合优化时用到的权重
	params.boundary_smoothness = 0.2;//曲面边界（非裁剪边界）的平滑性
	params.boundary_weight = 1.0;//优化时的边界权重

	// --------初始化B样条曲面----------------------
	printf("  surface fitting ...\n");
	//构造局部坐标系
	ON_NurbsSurface nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox(order, &data);

	pcl::on_nurbs::FittingSurface fit(&data, nurbs);

	fit.setQuiet(false); //设置是否打印调试信息

	// ----------可视化曲面模型---------------------
	pcl::PolygonMesh mesh;
	pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<pcl::Vertices> mesh_vertices;
	std::string mesh_id = "";
	mesh_id = "mesh_nurbs";
	//pcl::on_nurbs::Triangulation::convertSurface2PolygonMesh(fit.m_nurbs, mesh, mesh_resolution);
	//viewer.removePolygonMesh(mesh_id);
	//viewer.addPolygonMesh(mesh, mesh_id);//可视化初始化的B样条曲面
	qDebug() << "Before refine";
	//----------- 表面精细化处理---------------------
	for (unsigned i = 0; i < refinement; i++)//每次迭代都会添加控制点数目
	{
		fit.refine(0);           //设置在参数方向0上添加控制点优化
		if (two_dim)fit.refine(1);// 设置在参数方向1上添加控制点优化
		fit.assemble(params);
		fit.solve();
		//pcl::on_nurbs::Triangulation::convertSurface2Vertices(fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
		/*viewer.updatePolygonMesh<pcl::PointXYZ>(mesh_cloud, mesh_vertices, mesh_id);*/
		qDebug() << "refine: " << i;
	}

	////----------以最终优化确定的控制点数量来进行多次迭代拟合求解-----------
	//for (unsigned i = 0; i < iterations; i++)
	//{
	//	fit.assemble(params);
	//	fit.solve();
	//	pcl::on_nurbs::Triangulation::convertSurface2Vertices(fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
	//	viewer.updatePolygonMesh<pcl::PointXYZ>(mesh_cloud, mesh_vertices, mesh_id);
	//	viewer.spinOnce(3000);
	//	std::qDebug()<< "iterations: " << i ;
	//}

	// ----------------------拟合B样条曲线-------------------------
	pcl::on_nurbs::FittingCurve2dAPDM::FitParameter curve_params;
	curve_params.addCPsAccuracy = 5e-2;//距离阈值，如果曲线的支撑域到最近数据点的距离大于该阈值，则添加一控制点
	curve_params.addCPsIteration = 3;  //不进行控制点插入时的内部迭代优化次数
	curve_params.maxCPs = 20;         //允许的最大控制点个数
	curve_params.accuracy = 1;      //曲线的平均拟合精度
	curve_params.iterations = 10;     //最大迭代次数

	curve_params.param.closest_point_resolution = 0;//每一个支撑域内控制点的个数
	curve_params.param.closest_point_weight = 1.0;//最近点对应的权重
	curve_params.param.closest_point_sigma2 = 0.1;//外点的最近点阈值，拟合时不考虑远离于曲线的外点的距离值大于该点的值
	curve_params.param.interior_sigma2 = 0.00001; //内点的最近点阈值，拟合时不考虑远离于曲线的内点的距离值大于该点的值
	curve_params.param.smooth_concavity = 1.0;    //平滑凹凸性，该值使曲线向内或外凹（=0没用，<0向内凹，>0向外凹）
	curve_params.param.smoothness = 1.0;          //平滑项的权重

	// 用最小的控制点个数表示的一个圆来初始化该拟合曲线
	printf("  curve fitting ...\n");
	pcl::on_nurbs::NurbsDataCurve2d curve_data;
	curve_data.interior = data.interior_param;
	curve_data.interior_weight_function.push_back(true);//设置进行带权重的B样条曲线拟合
	ON_NurbsCurve curve_nurbs = pcl::on_nurbs::FittingCurve2dAPDM::initNurbsCurve2D(order, curve_data.interior);

	//进行曲线拟合并可视化
	pcl::on_nurbs::FittingCurve2dASDM curve_fit(&curve_data, curve_nurbs);
	// curve_fit.setQuiet (false); // 设置是否打印调试信息
	curve_fit.fitting(curve_params);
	//visualizeCurve(curve_fit.m_nurbs, fit.m_nurbs, viewer);  //可视化运行结束后会报错

	//-------------------裁剪B样条曲面-----------------------
	printf("  triangulate trimmed surface ...\n");
	/*viewer.removePolygonMesh(mesh_id);*/
	//对B样条曲面进行三角化，并根据B样条曲线对属于外部的三角形进行裁剪，
	//对于裁剪掉的三角形与B样条曲线相交处用曲线表示
	pcl::on_nurbs::Triangulation::convertTrimmedSurface2PolygonMesh(fit.m_nurbs, curve_fit.m_nurbs, mesh, mesh_resolution);
	//viewer->addPolygonMesh(mesh, mesh_id);
	total_mesh.concatenate(total_mesh, mesh);
	printf("孔洞修补... done.\\n");
	/*while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}*/
	// Extract vertices from the PolygonMesh
	pcl::PointCloud<pcl::PointXYZ>::Ptr fill_hole_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(mesh.cloud, *fill_hole_cloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector <pcl::Vertices> mesh_vertic = mesh.polygons;
	for (size_t i = 0; i < mesh_vertic.size(); i++)
	{
		output_cloud->points.push_back(fill_hole_cloud->points[mesh_vertic[i].vertices[0]]);
		output_cloud->points.push_back(fill_hole_cloud->points[mesh_vertic[i].vertices[1]]);
		output_cloud->points.push_back(fill_hole_cloud->points[mesh_vertic[i].vertices[2]]);
	}
	/*boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("显示点云"));
	viewer->addPointCloud(output_cloud, "111");
	viewer->addPolygonMesh(mesh, "2222");
	viewer->spin();*/
	return output_cloud;
}

void hole_fill(pcl::PointCloud<pcl::PointXYZ>::Ptr& orign_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& bound_cloud, std::vector<int>& bound_indice, pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud)
{
	// -------------------------------------------欧式聚类--------------------------------------------
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(bound_cloud);
	vector<pcl::PointIndices> cluster_indices; // 聚类索引
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;// 欧式聚类对象
	ec.setClusterTolerance(1.3);               // 设置近邻搜索的搜索半径（也即两个不同聚类团点之间的最小欧氏距离）
	ec.setMinClusterSize(3);                 // 设置一个聚类需要的最少的点数目为100
	ec.setMaxClusterSize(bound_cloud->size() * 0.8);               // 设置一个聚类需要的最大点数目为25000
	ec.setSearchMethod(tree);                  // 设置点云的搜索机制
	ec.setInputCloud(bound_cloud);                   // 设置输入点云
	ec.extract(cluster_indices);               // 从点云中提取聚类，并将点云索引保存在cluster_indices中
	//------------------------------------保存聚类结果并可视化---------------------------------------
	/*boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("planar segment")); ;*/
	//viewer->setBackgroundColor(0, 0, 0);
	//viewer->setWindowName("欧式聚类");
	int maxsize = 0;
	int point_count = 0;
	vector<pcl::PointIndices>::iterator i_indices;
	for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		point_count += it->indices.size();
		if (it->indices.size() > maxsize)
		{
			maxsize = it->indices.size();
			i_indices = it;
		}
	}
	point_count -= maxsize;
	//提取最外围需平滑边界的点云数据
	std::vector<int> out_bound;		//索引
	pcl::PointCloud<pcl::PointXYZ>::Ptr smooth_bound(new	pcl::PointCloud<pcl::PointXYZ>);
	for (auto ppit = i_indices->indices.begin(); ppit != i_indices->indices.end(); ppit++)
	{
		out_bound.push_back(bound_indice.at(*ppit));
		smooth_bound->points.push_back(bound_cloud->points[*ppit]);
	}

	cluster_indices.erase(i_indices);
	qDebug() << cluster_indices.size() << "聚类数量";
	int mean = point_count / cluster_indices.size();

	//建立kd-tree
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree1(new pcl::search::KdTree<pcl::PointXYZ>);
	//建立kdtree对象
	kdtree1.setInputCloud(orign_cloud); //设置需要建立kdtree的点云指针
	pcl::PointCloud<pcl::PointXYZ>::Ptr fixedhole_cloud(new	pcl::PointCloud<pcl::PointXYZ>);//孔洞填补后的点云
	pcl::PolygonMesh total_mesh;
	for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr repeat_filtered(new pcl::PointCloud<pcl::PointXYZ>);
		//特征点孔洞的膨胀
		for (auto pit = it->indices.begin(); pit != it->indices.end(); pit++)
		{
			//qDebug()<< bound_indice.at(*pit) ;
			pcl::PointXYZ searchPoint = orign_cloud->points[bound_indice.at(*pit)]; //设置查找点
			vector<int> pointIdxRadiusSearch;  //保存每个近邻点的索引
			vector<float> pointRadiusSquaredDistance;  //保存每个近邻点与查找点之间的欧式距离平方
			float radius = 30;  //设置查找半径范围
			/*
			qDebug()<< "K nearest neighbor search at (" << searchPoint.x
				<< " " << searchPoint.y
				<< " " << searchPoint.z<<")";
				*/
			if (kdtree1.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance, 100))
			{
				//50.0 * mean / it->indices.size())
				for (size_t i = 0; i < pointIdxRadiusSearch.size(); i++)
				{
					if (pointRadiusSquaredDistance[i] > 0)
					{
						//qDebug()<< "( distance: " << sqrt(pointRadiusSquaredDistance[i]) << ")" ;
						//pointRadiusSquaredDistance求出来的是对应点之间距离的平方，单位是米。这里求的是对应点之间的距离，单位是厘米。
						repeat_filtered->push_back(orign_cloud->points[pointIdxRadiusSearch[i]]);
					}
				}
			}
			//filtered->push_back(orign_cloud->points[bound_indice.at(*pit)]);
			//qDebug()<< "R = 0.1近邻点个数：" << pointIdxRadiusSearch.size() ;
		}
		pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
		deduplication(repeat_filtered, filtered, 0.5);
		pcl::PointCloud<pcl::PointXYZ>::Ptr fill_after = fill_hole(filtered, mean, total_mesh);
		fixedhole_cloud->points.insert(fixedhole_cloud->points.end(), fill_after->points.begin(), fill_after->points.end());
	}
	pcl::io::savePLYFileBinary("model_file\\paper_picture\\fixedhole_cloud.ply", *fixedhole_cloud);
	fixedhole_cloud->points.insert(fixedhole_cloud->points.begin(), orign_cloud->points.begin(), orign_cloud->points.end());
	//compute_bound(smooth_bound, fixedhole_cloud, output_cloud);
	output_cloud = fixedhole_cloud;


	pcl::io::savePLYFileBinary("model_file\\paper_picture\\total_mesh.ply", total_mesh);


	//     std::stringstream ss;
	//     ss << "Euclidean_cluster_" << begin + 1 << ".pcd";
	//     //pcl::io::savePCDFileBinary(ss.str(), *cloud_cluster);
	//     //qDebug()<< ss.str() << "保存完毕！！！" ;
	//    begin++;
	//    // 可视化相关的代码
	//    uint8_t R = rand() % (256) + 0;
	//    uint8_t G = rand() % (256) + 0;
	//    uint8_t B = rand() % (256) + 0;
	//    string str;
	//    ss >> str;
	//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud_cluster, R, G, B);
	//     viewer->addPointCloud<pcl::PointXYZ>(cloud_cluster, color, str);
	//     viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, str);
	//}
	//while (!viewer->wasStopped())
	//{
	//    viewer->spinOnce(100);
	//    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	//}

}

//----------------------三维重建------------------------------
void delau_recon(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PolygonMesh::Ptr& output_cloud, bool if_best)
{
	//----------------------PCD格式转为VTK支持的数据-------------------------
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		points->InsertPoint(i, cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
	}
	//---------------------------读取数据到VTK-------------------------------
	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
	polydata->SetPoints(points);
	//--------------------------delaunay三角剖分-----------------------------
	qDebug() << "三角剖分开始";
	pcl::StopWatch time;
	vtkSmartPointer<vtkDelaunay2D> delaunay = vtkSmartPointer<vtkDelaunay2D>::New();
	delaunay->SetInputData(polydata);
	if (if_best)
	{
		delaunay->SetProjectionPlaneMode(VTK_BEST_FITTING_PLANE);
	}
	else
	{
		delaunay->SetProjectionPlaneMode(VTK_DELAUNAY_XY_PLANE);
	}

	delaunay->Update();
	qDebug() << "三角剖分结束" << time.getTimeSeconds();
	// -----------------------保存为.ply----------------------------
	vtkNew<vtkPLYWriter> plyWriter;
	plyWriter->SetFileName("model_file/delau_mesh.ply"); // 文件路径及名称
	plyWriter->SetInputConnection(delaunay->GetOutputPort());
	plyWriter->SetFileTypeToBinary();
	plyWriter->Write();
	printf("保存完毕!!!");
	pcl::io::loadPLYFile("model_file/delau_mesh.ply", *output_cloud);
}


//----------------------轨迹生成------------------------------
Eigen::MatrixXd V; // 点云坐标矩阵
Eigen::MatrixXd N; // 法向量矩阵
Eigen::MatrixXd UV; // UV坐标矩阵
Eigen::MatrixXi F; // 面索引矩阵

Eigen::MatrixXd K1, K2;// 定义变量来存储曲率和曲率方向
Eigen::MatrixXd dir1, dir2;
int sort_indi;  //
Eigen::VectorXd spacing, step;//步长和行距
// 构建网格1的加速结构
igl::AABB<Eigen::MatrixXd, 3> tree1;
std::vector<std::vector<Eigen::MatrixXd>> line_array;
std::vector<Eigen::MatrixXd> single_array;
int sdfa = 0;
double space_width = 5;
double step_width = 4;

bool if_reverse = false;
Eigen::Vector3d peak;
Eigen::Vector3d end_point;
bool delete_dulp(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
	if (if_reverse)
	{
		return a[sort_indi] > b[sort_indi];
	}
	else
	{
		return a[sort_indi] < b[sort_indi];
	}
}

int binomialCoefficient(int n, int i) {
	if (i == 0 || i == n)
	{
		return 1;
	}
	else
	{
		return binomialCoefficient(n - 1, i - 1) + binomialCoefficient(n - 1, i);
	}
}

//将点云转换为矢量（3维）
void PointCloud2Vector2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::on_nurbs::vector_vec3d& data)
{
	for (unsigned i = 0; i < cloud->size(); i++)
	{
		pcl::PointXYZ& p = cloud->at(i);
		data.push_back(Eigen::Vector3d(p.x, p.y, p.z));
	}
}

/// <summary>
/// 
/// </summary>
/// <param name="direction_step">步长方向</param>
/// <param name="direction_spacing">行距方向</param>
/// <param name="P">当前点</param>
/// <param name="indice">点前点所在面的索引</param>
/// <returns></returns>

std::vector<int> bnd;
int polish_pointCout = 0;
int cout_111 = 0;
void compute_spacing(const Eigen::MatrixXd& P, const int& space_derict, const bool if_first = true)
{
	if (P.rows() == 0)
	{
		return;
	}
	double max_sort = peak[sort_indi];
	//line_array.push_back(P);
	std::set<Eigen::Vector3d, decltype(delete_dulp)*>  s(&delete_dulp);
	for (size_t i = 0; i < P.rows(); i++)
	{
		if (if_first)
		{
			if (space_derict == -1 && P(i, sort_indi) <= max_sort)
			{
				if_reverse = false;
				s.insert(Eigen::Vector3d(P(i, 0), P(i, 1), P(i, 2)));
			}
			else if (space_derict == 1 && P(i, sort_indi) >= max_sort)
			{
				if_reverse = true;
				s.insert(Eigen::Vector3d(P(i, 0), P(i, 1), P(i, 2)));
			}
		}
		else
		{
			if (space_derict == 1 && P(i, sort_indi) <= max_sort)
			{
				if_reverse = false;
				s.insert(Eigen::Vector3d(P(i, 0), P(i, 1), P(i, 2)));
			}
			else if (space_derict == -1 && P(i, sort_indi) >= max_sort)
			{
				if_reverse = true;
				s.insert(Eigen::Vector3d(P(i, 0), P(i, 1), P(i, 2)));
			}
		}
	}
	if (s.size() >= 2)//修改两端点的位置至边界
	{
		auto it = s.end();
		Eigen::Vector3d pit = *(--it) - *(--it);
		pit.normalize();
		it++;
		if (if_first)
		{
			if ((space_derict == -1 && (*it)[sort_indi] < max_sort) || (space_derict == 1 && (*it)[sort_indi] > max_sort))
			{
				s.insert(*(it)+(max_sort - (*it)[sort_indi]) / pit[sort_indi] * pit);
				s.erase(it);
			}
		}
		else
		{
			if ((space_derict == 1 && (*it)[sort_indi] < max_sort) || (space_derict == -1 && (*it)[sort_indi] > max_sort))
			{
				s.insert(*(it)+(max_sort - (*it)[sort_indi]) / pit[sort_indi] * pit);
				s.erase(it);
			}
			/*auto safdaf = s.end();
			safdaf--;
			qDebug() << safdaf->x();
			qDebug() << safdaf->y();
			qDebug() << safdaf->z();
			qDebug() << "\r\n";
			s.erase(safdaf);*/
		}
	}
	Eigen::MatrixXd middle_P(s.size(), 3);
	int middle_num = 0;
	for (auto it = s.begin(); it != s.end(); it++)
	{
		middle_P.row(middle_num++) = *it;
	}

	const Eigen::Vector3d direction_step = step.head(3);
	const Eigen::Vector3d direction_spacing = spacing.head(3);
	Eigen::VectorXd sqrD;//最小距离
	Eigen::VectorXi I;//所在面索引
	Eigen::MatrixXd C;//最近点
	tree1.squared_distance(V, F, middle_P, sqrD, I, C);
	//cout << sqrD << endl;
	std::vector<int> delete_indic;
	for (int i = 0; i < sqrD.size(); i++)
	{
		if (sqrD[i] > 0.5)
		{
			delete_indic.push_back(i);
		}
		bool isManifold = igl::is_edge_manifold(F.row(I[i]));
	}
	Eigen::MatrixXd cut_C((sqrD.size() - delete_indic.size()), 3);
	//Eigen::VectorXi new_I((sqrD.size() - delete_indic.size()));//裁剪后的最近点
	int num11 = 0;
	for (size_t i = 0; i < I.size(); i++)
	{
		if (std::find(delete_indic.begin(), delete_indic.end(), i) == delete_indic.end())
		{
			cut_C.row(num11) = C.row(i);
			//new_I[num11] = I[i];
			num11++;
		}
	}
	std::vector<Eigen::MatrixXd> bez_CtlP;

	if (cut_C.rows() <= 1)
	{
		return;
	}
	//----------------三次贝塞尔拟合-------------------
	try
	{
		igl::fit_cubic_bezier(cut_C, 0.1, bez_CtlP);
	}
	catch (const std::exception&)
	{
		cout << "拟合错误" << endl;
	}
	//line_array.push_back(P);
	//根据控制点计算原拟合点对应点  t=0，1 为对应线段起始和终点  3
	int ctlPNum = bez_CtlP[0].rows() - 1;//控制点数量
	std::vector<Eigen::Vector3d> fit_BezV;//重划分
	Eigen::Vector3d point, cur_point;
	int row_num = 0;
	double excess = 0;

	double total_length = 0;
	for (size_t i = 0; i < bez_CtlP.size(); i++)
	{
		double t = 0;
		//三次贝塞尔曲线参数方程        B(t) = (1 - t) ^ 3 * P0 + 3 * (1 - t) ^ 2 * t * P1 + 3 * (1 - t) * t ^ 2 * P2 + t ^ 3 * P3
		for (size_t k = 0; k < 100; k++)
		{
			cur_point << 0, 0, 0;
			for (int j = 0; j <= ctlPNum; j++)
			{
				cur_point += binomialCoefficient(ctlPNum, j) * std::pow(1 - t, ctlPNum - j) * std::pow(t, j) * bez_CtlP[i].row(j).transpose();
			}
			if (k != 0)
			{
				total_length += (cur_point - point).norm();
			}
			point = cur_point;
			t += 1.0 / 100.0;
		}
	}
	cur_point << 0, 0, 0;
	point << 0, 0, 0;
	double new_step_width = total_length / (std::ceil(total_length / step_width));
	for (int i = 0; i < bez_CtlP.size(); i++)/*******三次样条端数量少于边界点*******/
	{
		double t = 0;
		double length = 0;
		//三次贝塞尔曲线参数方程        B(t) = (1 - t) ^ 3 * P0 + 3 * (1 - t) ^ 2 * t * P1 + 3 * (1 - t) * t ^ 2 * P2 + t ^ 3 * P3
		for (size_t k = 0; k < 100; k++)
		{
			cur_point << 0, 0, 0;
			for (int j = 0; j <= ctlPNum; j++)
			{
				cur_point += binomialCoefficient(ctlPNum, j) * std::pow(1 - t, ctlPNum - j) * std::pow(t, j) * bez_CtlP[i].row(j).transpose();
			}
			if (k != 0)
			{
				length += (cur_point - point).norm();
			}
			point = cur_point;
			t += 1.0 / 100.0;
		}
		//int num_point = std::ceil(length / new_step_width);//向上取整

		int num_point;
		if (i == bez_CtlP.size() - 1)
		{
			num_point = std::round((length + excess) / new_step_width);//四舍五入
		}
		else
		{
			num_point = std::floor((length + excess) / new_step_width);//向下取整
		}

		if (num_point == 0)
		{
			excess += length;
		}
		int last_or_other;
		if (i == 0)//当前曲线段的最后一个控制点是下一曲线段的起始点，不能重叠，否侧会出现曲线变形
		{
			last_or_other = num_point + 1;
		}
		else
		{
			last_or_other = num_point;
		}
		//last_or_other = num_point;
		t = 0;
		for (int k = 0; k < last_or_other; k++)
		{
			point << 0, 0, 0;
			if (t > 1)
			{
				t = 1;
			}

			if (excess >= 0 && i != 0 && k == 0)
			{
				t += (new_step_width - excess) / length;
				excess = std::fmod(length + excess, new_step_width);
			}
			else if (i == 0 && k == 0)
			{
				excess = std::fmod(length + excess, new_step_width);
			}
			for (int j = 0; j <= ctlPNum; j++)
			{
				point += binomialCoefficient(ctlPNum, j) * std::pow(1 - t, ctlPNum - j) * std::pow(t, j) * bez_CtlP[i].row(j).transpose();
			}
			t += new_step_width / length;
			//t += 1.0 / num_point;
			//cout << point << endl;
			fit_BezV.push_back(point);
		}
		//qDebug() << 33333;
	}
	//fit_BezV.push_back(bez_CtlP[bez_CtlP.size() - 1].row(3));
	pcl::PointCloud<pcl::PointXYZ>::Ptr coud9(new pcl::PointCloud<pcl::PointXYZ>);
	for (size_t i = 0; i < bez_CtlP.size(); i++)
	{
		coud9->points.push_back(pcl::PointXYZ(bez_CtlP[i](0, 0), bez_CtlP[i](0, 1), bez_CtlP[i](0, 2)));
		coud9->points.push_back(pcl::PointXYZ(bez_CtlP[i](1, 0), bez_CtlP[i](1, 1), bez_CtlP[i](1, 2)));
		coud9->points.push_back(pcl::PointXYZ(bez_CtlP[i](2, 0), bez_CtlP[i](2, 1), bez_CtlP[i](2, 2)));
		coud9->points.push_back(pcl::PointXYZ(bez_CtlP[i](3, 0), bez_CtlP[i](3, 1), bez_CtlP[i](3, 2)));
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr coud10(new pcl::PointCloud<pcl::PointXYZ>);
	for (size_t i = 0; i < fit_BezV.size(); i++)
	{
		coud10->points.push_back(pcl::PointXYZ(fit_BezV[i](0), fit_BezV[i](1), fit_BezV[i](2)));
	}
	if (cout_111 == 0)
	{
		pcl::io::savePLYFileASCII("dafhdfhakjdhfkajdhfkjahfk.ply", *coud9);
		pcl::io::savePLYFileASCII("2131111111111112222222222.ply", *coud10);
	}
	cout_111++;
	Eigen::MatrixXd regen_P(fit_BezV.size(), 3);
	for (size_t i = 0; i < fit_BezV.size(); i++)
	{
		regen_P.row(i) << fit_BezV[i][0], fit_BezV[i][1], fit_BezV[i][2];
	}

	Eigen::VectorXd new_sqrD;//最小距离
	Eigen::VectorXi new_I;//所在面索引
	Eigen::MatrixXd new_C;//最近点
	tree1.squared_distance(V, F, regen_P, new_sqrD, new_I, new_C);
	Eigen::Vector3d first_side = new_C.row(new_C.rows() - 1);
	Eigen::Vector3d second_side = new_C.row(0);
	double are_3d = ((first_side - end_point).cross(second_side - end_point)).norm();
	double arc_length = (first_side - second_side).norm();
	if (!(sdfa == 0 && space_derict == -1))
	{
		single_array.push_back(new_C);
	}
	polish_pointCout += new_C.rows();
	sdfa++;
	//cout << sdfa << endl;

	if (are_3d / (2 * arc_length) <= 0.7 * 0.5 * space_width)
	{
		return;
	}

	std::vector<Eigen::Vector3d> arry_new_P;
	int max_indice = 0;
	double max_z = 0;
	for (size_t i = 0; i < new_C.rows(); i++)
	{
		Eigen::MatrixXd bc(1, 3); // 点 P 在三角形 T 中的面积坐标
		Eigen::Vector3d single_P(new_C(i, 0), new_C(i, 1), new_C(i, 2));
		if (single_P.isApprox(Eigen::VectorXd(V.row(F(new_I[i], 0)))))
		{
			bc << 1, 0, 0;
		}
		else if (single_P.isApprox(Eigen::VectorXd(V.row(F(new_I[i], 1)))))
		{
			bc << 0, 1, 0;
		}
		else if (single_P.isApprox(Eigen::VectorXd(V.row(F(new_I[i], 2)))))
		{
			bc << 0, 0, 1;
		}
		else
		{
			try
			{
				double tri_area = (Eigen::Vector3d(V.row(F(new_I[i], 1))).cross(Eigen::Vector3d(V.row(F(new_I[i], 2))))).norm();
				bc << ((single_P - Eigen::Vector3d(V.row(F(new_I[i], 1)))).cross(single_P - Eigen::Vector3d(V.row(F(new_I[i], 2))))).norm() / tri_area, ((single_P - Eigen::Vector3d(V.row(F(new_I[i], 0)))).cross(single_P - Eigen::Vector3d(V.row(F(new_I[i], 2))))).norm() / tri_area, ((single_P - Eigen::Vector3d(V.row(F(new_I[i], 1)))).cross(single_P - Eigen::Vector3d(V.row(F(new_I[i], 0))))).norm() / tri_area;
				//igl::barycentric_coordinates(P, Eigen::VectorXd(V.row(F(indice, 0))), Eigen::VectorXd(V.row(F(indice, 1))), Eigen::VectorXd(V.row(F(indice, 2))), bc);
			}
			catch (const std::exception&)
			{
				cout << "面积坐标求解失败" << endl;
			}
		}
		//cout << "打电话副科级爱好" << bc << endl;
		//cout << F(new_I[i], 0) << endl;
		//cout << N.row(F(new_I[i], 0)) << endl;
		Eigen::Vector3d perv_normal = bc(0) * N.row(F(new_I[i], 0)) + bc(1) * N.row(F(new_I[i], 1)) + bc(2) * N.row(F(new_I[i], 2));
		perv_normal.normalize();
		Eigen::Vector3d perv_tangen;
		if (i == 0)
		{
			perv_tangen = Eigen::Vector3d(new_C(i + 1, 0), new_C(i + 1, 1), new_C(i + 1, 2)) - single_P;
		}
		else if (i == (new_C.rows() - 1))
		{
			perv_tangen = single_P - Eigen::Vector3d(new_C(i - 1, 0), new_C(i - 1, 1), new_C(i - 1, 2));
		}
		else
		{
			perv_tangen = (Eigen::Vector3d(new_C(i + 1, 0), new_C(i + 1, 1), new_C(i + 1, 2)) - Eigen::Vector3d(new_C(i - 1, 0), new_C(i - 1, 1), new_C(i - 1, 2))) / 2;
			//perv_tangen = (Eigen::Vector3d(new_C(i + 1, 0), new_C(i + 1, 1), new_C(i + 1, 2)) - Eigen::Vector3d(new_C(i, 0), new_C(i, 1), new_C(i, 2))) / 2;
		}
		perv_tangen.normalize();
		Eigen::Vector3d direction_3d;
		if ((perv_tangen[0] == perv_normal[0] && perv_tangen[1] == perv_normal[1] && perv_tangen[2] == perv_normal[2]) || (perv_tangen[0] == perv_tangen[1] == perv_tangen[2] && perv_tangen[0] == 0))
		{
			direction_3d = direction_step.cross(perv_normal);
		}
		else
		{
			direction_3d = perv_tangen.cross(perv_normal);
		}

		direction_3d.normalize();
		if (direction_3d.dot(space_derict * direction_spacing) < 0)
		{
			direction_3d = -direction_3d;
		}
		Eigen::VectorXd P0(dir1.row(F(new_I[i], 0)));
		Eigen::VectorXd P1(dir1.row(F(new_I[i], 1)));
		Eigen::VectorXd P2(dir1.row(F(new_I[i], 2)));
		//cout << "dafadf" << P0 << endl;
		//cout << "dafadf" << P1 << endl;
		//cout << "dafadf" << P2 << endl;
		//cout << "dafadf" << direction_3d << endl;
		double seita0 = direction_3d.dot(P0.head(3)) / direction_3d.norm() / (P0.head(3)).norm();
		double seita1 = direction_3d.dot(P1.head(3)) / direction_3d.norm() / (P1.head(3)).norm();
		double seita2 = direction_3d.dot(P2.head(3)) / direction_3d.norm() / (P2.head(3)).norm();
		double ks0 = K1(F(new_I[i], 0), 0) * cos(seita0) * cos(seita0) + K2(F(new_I[i], 0), 0) * sin(seita0) * sin(seita0);
		double ks1 = K1(F(new_I[i], 1), 0) * cos(seita1) * cos(seita1) + K2(F(new_I[i], 1), 0) * sin(seita1) * sin(seita1);
		double ks2 = K1(F(new_I[i], 2), 0) * cos(seita2) * cos(seita2) + K2(F(new_I[i], 2), 0) * sin(seita2) * sin(seita2);

		double ks = bc(0) * ks0 + bc(1) * ks1 + bc(2) * ks2;

		if (ks == 0)
		{
			std::cout << "曲率计算错误,错误点所在面为face" << new_I[i] << std::endl;
		}
		//计算轨迹间距
		double ρ = 1 / abs(ks);//曲率半径
		//cout << ρ << endl;
		double h = 0.5;//残留高度
		double w = 2 * ρ * sqrt(h * h + 2 * ρ * h) / (ρ + h);
		if (w > 1.5)
		{
			w = space_width;
		}
		if (single_P.z() > max_z)
		{
			max_z = single_P.z();
			max_indice = i;
		}
		Eigen::Vector3d cur_point;

		direction_3d.normalize();
		cur_point = Eigen::Vector3d(single_P + direction_3d * w);
		arry_new_P.push_back(cur_point);
		//new_P.row(i) = cur_point;
		//double cos_seita = direction_3d.dot(direction_spacing) / direction_3d.norm() / direction_spacing.norm();
		//w *= cos_seita;
	}
	Eigen::MatrixXd new_P(arry_new_P.size(), 3);
	for (size_t i = 0; i < arry_new_P.size(); i++)
	{
		new_P.row(i) = arry_new_P[i];
	}
	compute_spacing(new_P, space_derict, if_first);
}

bool compareFunction(Eigen::Vector3d a, Eigen::Vector3d b)
{
	// 从小到大
	return a[sort_indi] < b[sort_indi];
}

void decide_sort(Eigen::VectorXd& x_vector, Eigen::VectorXd& y_vector, bool x_or_y)
{
	if (x_or_y)
	{
		if (x_vector.norm() > y_vector.norm())//第一线方向校正
		{
			spacing = x_vector;
			sort_indi = 1;
			step = y_vector;
		}
		else
		{
			spacing = y_vector;
			sort_indi = 0;
			step = x_vector;
		}
	}
	else
	{
		if (x_vector.norm() > y_vector.norm())//第一线方向校正
		{
			spacing = y_vector;
			sort_indi = 0;
			step = x_vector;
		}
		else
		{
			spacing = x_vector;
			sort_indi = 1;
			step = y_vector;
		}
	}
}

void one2four(const Eigen::VectorXd& x_vector, const Eigen::VectorXd& y_vector, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& seg_cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr delau_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (size_t i = 0; i < V.rows(); i++)
	{
		delau_cloud->points.push_back(pcl::PointXYZ(V(i, 0), V(i, 1), V(i, 2)));
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr filter_up(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr filter_down(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(delau_cloud);
	if (sort_indi == 1)
	{
		pass.setFilterFieldName("x");
		pass.setFilterLimits(-1, x_vector.norm() / 2 - 6);
	}
	else if (sort_indi == 0)
	{
		pass.setFilterFieldName("y");
		pass.setFilterLimits(-1, y_vector.norm() / 2);
	}
	// pass.setKeepOrganized(true); // 保持有序点云结构，该功能用于有序点云才有意义。
	pass.setNegative(false); //设置保留范围内的点还是过滤掉范围内的点，标志为false时保留范围内的点
	pass.filter(*filter_down);
	pass.setNegative(true); //设置保留范围内的点还是过滤掉范围内的点，标志为false时保留范围内的点
	pass.filter(*filter_up);


	pcl::PointCloud<pcl::PointXYZ>::Ptr grid_0(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr grid_1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr grid_2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr grid_3(new pcl::PointCloud<pcl::PointXYZ>);
	pass.setInputCloud(filter_down);
	if (sort_indi == 1)
	{
		pass.setFilterFieldName("y");
		pass.setFilterLimits(-1, y_vector.norm() / 2);
	}
	else if (sort_indi == 0)
	{
		pass.setFilterFieldName("x");
		pass.setFilterLimits(-1, x_vector.norm() / 2 - 6);
	}
	// pass.setKeepOrganized(true); // 保持有序点云结构，该功能用于有序点云才有意义。
	pass.setNegative(false); //设置保留范围内的点还是过滤掉范围内的点，标志为false时保留范围内的点
	pass.filter(*grid_0);
	pass.setNegative(true); //设置保留范围内的点还是过滤掉范围内的点，标志为false时保留范围内的点
	pass.filter(*grid_1);

	pass.setInputCloud(filter_up);
	if (sort_indi == 1)
	{
		pass.setFilterFieldName("y");
		pass.setFilterLimits(-1, y_vector.norm() / 2);
	}
	else if (sort_indi == 0)
	{
		pass.setFilterFieldName("x");
		pass.setFilterLimits(-1, x_vector.norm() / 2 - 6);
	}
	// pass.setKeepOrganized(true); // 保持有序点云结构，该功能用于有序点云才有意义。
	pass.setNegative(false); //设置保留范围内的点还是过滤掉范围内的点，标志为false时保留范围内的点
	pass.filter(*grid_3);
	pass.setNegative(true); //设置保留范围内的点还是过滤掉范围内的点，标志为false时保留范围内的点
	pass.filter(*grid_2);
	seg_cloud.push_back(grid_0);
	seg_cloud.push_back(grid_1);
	seg_cloud.push_back(grid_2);
	seg_cloud.push_back(grid_3);
}


void param_curve(const std::string& mesh_name, pcl::PointCloud<pcl::PointXYZ>::Ptr& curve, Eigen::MatrixXd& tran_v, std::vector<Eigen::MatrixXd>& rotation)
{
	bool success;
	success = igl::readPLY(mesh_name, V, F);

	if (success)
	{
		// 打印读取到的数据
		std::cout << "读取成功" << std::endl;
		std::cout << "点数量" << V.rows() << "边数量" << F.rows() << std::endl;
	}
	else
	{
		std::cerr << "Failed to read STL file." << std::endl;
	}
	//// 移除网格自交叉
	//Eigen::MatrixXd V_remeshed;
	//Eigen::MatrixXi F_remeshed;
	//Eigen::MatrixXi _1;
	//Eigen::VectorXi _2, _3;
	//igl::copyleft::cgal::remesh_self_intersections(V, F, igl::copyleft::cgal::RemeshSelfIntersectionsParam(false, false, false), V_remeshed, F_remeshed, _1, _2, _3);
	//igl::writePLY("重划分网格.ply", V_remeshed, F_remeshed);

	//igl::collapse_small_triangles(,);

	//// Fix two points on the boundary
	//Eigen::VectorXi bnd, b(2, 1);
	//igl::boundary_loop(F, bnd);
	//b(0) = bnd(0);
	//b(1) = bnd(bnd.size() / 2);
	//Eigen::MatrixXd bc(2, 2);				//lscm参数化展开算法
	//bc << 0, 0, 1, 0;
	//Eigen::SparseMatrix<double> Q;
	//// LSCM parametrization
	//igl::lscm(V, F, b, bc, V_uv, Q);
	// Scale the uv
	//V_uv *= 5;

	//---------------------不展开，在三维网格上生成校正曲线的方法--------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	for (size_t i = 0; i < V.rows(); i++)
	{
		cloud->points.push_back(pcl::PointXYZ(V(i, 0), V(i, 1), V(i, 2)));
	}
	// 计算点云质心和协方差矩阵
	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*cloud, pcaCentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
	// 协方差矩阵分解求特征值特征向量
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
	// 校正主方向间垂直
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
	eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
	eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));

	int max_axis = 0;
	int min_axis = 0;
	for (size_t i = 0; i < eigenValuesPCA.size(); i++)
	{
		if (eigenValuesPCA[i] > max_axis)
		{
			max_axis = i;
		}
		if (eigenValuesPCA[i] < min_axis)
		{
			min_axis = i;
		}
	}
	Eigen::MatrixXf translate(3, 3);
	translate.block<3, 1>(0, 0) = eigenVectorsPCA.col(max_axis);
	translate.block<3, 1>(0, 1) = eigenVectorsPCA.col((max_axis + min_axis) / 2);
	translate.block<3, 1>(0, 2) = -eigenVectorsPCA.col(min_axis);
	cout << "特征值va(3x1):\n" << eigenValuesPCA << endl; // Eigen计算出来的特征值默认是从小到大排列
	cout << "特征向量ve(3x3):\n" << eigenVectorsPCA << endl;
	cout << "质心点(4x1):\n" << pcaCentroid << endl;
	/*
	// 另一种计算点云协方差矩阵特征值和特征向量的方式:通过pcl中的pca接口，如下，这种情况得到的特征向量相似特征向量
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCA<pcl::PointXYZ> pca;
	pca.setInputCloud(cloudSegmented);
	pca.project(*cloudSegmented, *cloudPCAprojection);
	std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;//计算特征向量
	std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;//计算特征值
	*/
	// 将输入点云转换至原点
	Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();     // 定义变换矩阵 
	Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity(); // 定义变换矩阵的逆
	tm.block<3, 3>(0, 0) = translate.transpose();   // 旋转矩阵R.
	tm.block<3, 1>(0, 3) = -1.0f * (translate.transpose()) * (pcaCentroid.head<3>());// 平移向量 -R*t
	tm_inv = tm.inverse();

	std::cout << "变换矩阵tm(4x4):\n" << tm << std::endl;
	std::cout << "逆变矩阵tm'(4x4):\n" << tm_inv << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>);
	//***************************************************************************************

	pcl::transformPointCloud(*cloud, *transformedCloud, tm);
#pragma region pca旋转可视化部分
	//PointType min_p1, max_p1;
	//Eigen::Vector3f c1, c;//c原始点云的形心
	//pcl::getMinMax3D(*transformedCloud, min_p1, max_p1);
	//c1 = 0.5f * (min_p1.getVector3fMap() + max_p1.getVector3fMap());

	//cout << "型心c1(3x1):\n" << c1 << endl;

	//Eigen::Affine3f tm_inv_aff(tm_inv);
	//pcl::transformPoint(c1, c, tm_inv_aff);

	//Eigen::Vector3f whd, whd1;
	//whd1 = max_p1.getVector3fMap() - min_p1.getVector3fMap();
	//whd = whd1;
	//float sc1 = (whd1(0) + whd1(1) + whd1(2)) / 3;  //点云平均尺度，用于设置主方向箭头大小

	//cout << "width1=" << whd1(0) << endl;
	//cout << "heght1=" << whd1(1) << endl;
	//cout << "depth1=" << whd1(2) << endl;
	//cout << "scale1=" << sc1 << endl;

	//const Eigen::Quaternionf bboxQ1(Eigen::Quaternionf::Identity());
	//const Eigen::Vector3f    bboxT1(c1);

	//const Eigen::Quaternionf bboxQ(tm_inv.block<3, 3>(0, 0));
	//const Eigen::Vector3f    bboxT(c);

	//Eigen::Vector3f px, py, pz;
	//Eigen::Affine3f tm_aff(tm);
	//pcl::transformVector(Eigen::Vector3f(eigenVectorsPCA.col(0)), px, tm_aff);
	//pcl::transformVector(Eigen::Vector3f(eigenVectorsPCA.col(1)), py, tm_aff);
	//pcl::transformVector(Eigen::Vector3f(eigenVectorsPCA.col(2)), pz, tm_aff);
	//Eigen::Vector3d op, pcaX, pcaY, pcaZ;


	//// 变换到原点的点云主方向
	//PointType op;
	//op.x = 0.0;
	//op.y = 0.0;
	//op.z = 0.0;
	//PointType pcaX;
	//pcaX.x = sc1 * px(0);
	//pcaX.y = sc1 * px(1);
	//pcaX.z = sc1 * px(2);
	//PointType pcaY;
	//pcaY.x = sc1 * py(0);
	//pcaY.y = sc1 * py(1);
	//pcaY.z = sc1 * py(2);
	//PointType pcaZ;
	//pcaZ.x = sc1 * pz(0);
	//pcaZ.y = sc1 * pz(1);
	//pcaZ.z = sc1 * pz(2);
	//// 初始点云的主方向
	//PointType cp;
	//cp.x = pcaCentroid(0);
	//cp.y = pcaCentroid(1);
	//cp.z = pcaCentroid(2);
	//PointType pcX;
	//pcX.x = sc1 * eigenVectorsPCA(0, 0) + cp.x;
	//pcX.y = sc1 * eigenVectorsPCA(1, 0) + cp.y;
	//pcX.z = sc1 * eigenVectorsPCA(2, 0) + cp.z;
	//PointType pcY;
	//pcY.x = sc1 * eigenVectorsPCA(0, 1) + cp.x;
	//pcY.y = sc1 * eigenVectorsPCA(1, 1) + cp.y;
	//pcY.z = sc1 * eigenVectorsPCA(2, 1) + cp.z;
	//PointType pcZ;
	//pcZ.x = sc1 * eigenVectorsPCA(0, 2) + cp.x;
	//pcZ.y = sc1 * eigenVectorsPCA(1, 2) + cp.y;
	//pcZ.z = sc1 * eigenVectorsPCA(2, 2) + cp.z;


	//// 可视化
	//pcl::visualization::PCLVisualizer viewer;
	//viewer.setBackgroundColor(1.0, 1.0, 1.0);
	//viewer.setWindowName("PCA获取点云包围盒");
	////输入的初始点云
	//pcl::visualization::PointCloudColorHandlerCustom<PointType> color_handler(cloud, 255, 0, 0);
	//viewer.addPointCloud(cloud, color_handler, "cloud");
	//viewer.addCube(bboxT, bboxQ, whd(0), whd(1), whd(2), "bbox");
	//viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bbox");
	//viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "bbox");

	//viewer.addArrow(pcX, cp, 1.0, 0.0, 0.0, false, "arrow_x");
	//viewer.addArrow(pcY, cp, 0.0, 1.0, 0.0, false, "arrow_y");
	//viewer.addArrow(pcZ, cp, 0.0, 0.0, 1.0, false, "arrow_z");

	////转换到原点的点云
	//pcl::visualization::PointCloudColorHandlerCustom<PointType> tc_handler(transformedCloud, 0, 255, 0);
	//viewer.addPointCloud(transformedCloud, tc_handler, "transformCloud");
	//viewer.addCube(bboxT1, bboxQ1, whd1(0), whd1(1), whd1(2), "bbox1");
	//viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bbox1");
	//viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "bbox1");

	//viewer.addArrow(pcaX, op, 1.0, 0.0, 0.0, false, "arrow_X");
	//viewer.addArrow(pcaY, op, 0.0, 1.0, 0.0, false, "arrow_Y");
	//viewer.addArrow(pcaZ, op, 0.0, 0.0, 1.0, false, "arrow_Z");

	//viewer.addCoordinateSystem(0.5f * sc1);

	//while (!viewer.wasStopped())
	//{
	//	viewer.spinOnce(100);
	//	boost::this_thread::sleep(boost::posix_time::microseconds(10000));
	//}
#pragma endregion

	for (size_t i = 0; i < V.rows(); i++)
	{
		V.row(i) << transformedCloud->points.at(i).x, transformedCloud->points.at(i).y, transformedCloud->points.at(i).z;
	}
	Eigen::MatrixXd box_p, box_e;
	igl::bounding_box(V, box_p, box_e);   //求出的box_p.row(7)为最小点
	//std::cout << "最大边界点" << box_e << std::endl;
	//std::cout << "最小边界点" << box_p << std::endl;
	if (box_p.rows() != 8)
	{
		std::cout << "包容盒求取错误" << std::endl;
	}
	Eigen::VectorXd x_vector, y_vector, z_vector, origin_p;
	origin_p = Eigen::VectorXd(box_p.row(7));
	Eigen::MatrixXd tran_mat(V.rows(), 3);
	tran_mat.col(0).setConstant(origin_p[0]);
	tran_mat.col(1).setConstant(origin_p[1]);
	tran_mat.col(2).setConstant(origin_p[2]);
	V -= tran_mat;

	for (int i = 0; i < box_p.rows(); i++)//将包容盒的左下角点移至原点    delau投影三角化时已将模型摆正，不需求取旋转矩阵
	{
		box_p.row(i) = Eigen::VectorXd(box_p.row(i)) - Eigen::VectorXd(box_p.row(7));
		if (box_p(i, 0) != 0 && box_p(i, 1) == 0 && box_p(i, 2) == 0)
		{
			x_vector = box_p.row(i);
		}
		else if (box_p(i, 0) == 0 && box_p(i, 1) != 0 && box_p(i, 2) == 0)
		{
			y_vector = box_p.row(i);
		}
		else if (box_p(i, 0) == 0 && box_p(i, 1) == 0 && box_p(i, 2) != 0)
		{
			z_vector = box_p.row(i);
		}
	}
	cout << "最小边界点" << box_p << endl;

	Eigen::MatrixXd V_connect(4, 3);//中间相切面的点定义（第一相切）
	if (x_vector.norm() > y_vector.norm())
	{
		spacing = x_vector;
		sort_indi = 1;
		step = y_vector;
		V_connect << Eigen::VectorXd(box_p.row(7))[0] + spacing[0] / 2 - 6, Eigen::VectorXd(box_p.row(7))[1], Eigen::VectorXd(box_p.row(7))[2],
			step[0] + spacing[0] / 2 - 6, step[1], step[2],
			Eigen::VectorXd(box_p.row(7))[0] + spacing[0] / 2 - 6, Eigen::VectorXd(box_p.row(7))[1], z_vector[2],
			step[0] + spacing[0] / 2 - 6, step[1], z_vector[2];
	}
	else
	{
		spacing = y_vector;
		sort_indi = 0;
		step = x_vector;
		V_connect << Eigen::VectorXd(box_p.row(7))[0], Eigen::VectorXd(box_p.row(7))[1] + spacing[1] / 2, Eigen::VectorXd(box_p.row(7))[2],
			step[0], step[1] + spacing[1] / 2, step[2],
			Eigen::VectorXd(box_p.row(7))[0], Eigen::VectorXd(box_p.row(7))[1] + spacing[1] / 2, z_vector[2],
			step[0], step[1] + spacing[1] / 2, z_vector[2];
	}
	cout << "中间相切面" << V_connect << endl;

	pcl::StopWatch time;
	igl::principal_curvature(V, F, dir1, dir2, K1, K2);// 计算每个点的曲率  //计算需60s
	std::cout << "曲率的计算时间" << time.getTimeSeconds() << std::endl;

	igl::writeSTL("model_file/re_mesh.stl", V, F);

	//--------------------------计算线与面交点-------速度慢，有多余点
	Eigen::MatrixXi F_connect(2, 3);
	F_connect << 0, 1, 2, 1, 3, 2;
	Eigen::MatrixXd ei, pi;
	time.reset();
	tree1.init(V, F);
	igl::fast_find_intersections(tree1, V, F, V_connect, F_connect, ei, pi);//线与面的求交求不到交点
	//std::cout << "相交点数量" << pi << "时间" << time.getTimeSeconds() << std::endl;
	//std::cout << "相交点数量" << ei << std::endl;
	/*std::vector<Eigen::Vector3d> conn_point;
	for (size_t i = 0; i < pi.rows(); i++)
	{
		conn_point.push_back(Eigen::Vector3d(pi(i, 0), pi(i, 1), pi(i, 2)));
	}*/
	//std::sort(conn_point.begin(), conn_point.end(), compareFunction);
	//auto last = std::unique(conn_point.begin(), conn_point.end());
	//conn_point.erase(last, conn_point.end());
	if_reverse = false;
	std::set<Eigen::Vector3d, decltype(delete_dulp)*>  s(&delete_dulp);
	for (size_t i = 0; i < pi.rows(); i++)
	{
		s.insert(Eigen::Vector3d(pi(i, 0), pi(i, 1), pi(i, 2)));
	}
	Eigen::MatrixXd middle_P(s.size(), 3);
	int middle_num = 0;
	for (auto it = s.begin(); it != s.end(); it++)
	{
		middle_P.row(middle_num++) = *it;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1111(new pcl::PointCloud<pcl::PointXYZ>);
	for (size_t i = 0; i < s.size(); i++)
	{
		cloud1111->points.push_back(pcl::PointXYZ(middle_P(i, 0), middle_P(i, 1), middle_P(i, 2)));
	}
	pcl::io::savePLYFileASCII("first_line.ply", *cloud1111);
	//visualize_cloud(cloud1111);


	Eigen::MatrixXd V_connect2(4, 3);//中间相切面的点定义（第二相切）
	if (x_vector.norm() > y_vector.norm())
	{
		spacing = y_vector;
		sort_indi = 0;
		step = x_vector;
		V_connect2 << Eigen::VectorXd(box_p.row(7))[0], Eigen::VectorXd(box_p.row(7))[1] + spacing[1] / 2, Eigen::VectorXd(box_p.row(7))[2],
			step[0], step[1] + spacing[1] / 2, step[2],
			Eigen::VectorXd(box_p.row(7))[0], Eigen::VectorXd(box_p.row(7))[1] + spacing[1] / 2, z_vector[2],
			step[0], step[1] + spacing[1] / 2, z_vector[2];
	}
	else
	{
		spacing = x_vector;
		sort_indi = 1;
		step = y_vector;
		V_connect2 << Eigen::VectorXd(box_p.row(7))[0] + spacing[0] / 2, Eigen::VectorXd(box_p.row(7))[1], Eigen::VectorXd(box_p.row(7))[2],
			step[0] + spacing[0] / 2, step[1], step[2],
			Eigen::VectorXd(box_p.row(7))[0] + spacing[0] / 2, Eigen::VectorXd(box_p.row(7))[1], z_vector[2],
			step[0] + spacing[0] / 2, step[1], z_vector[2];
	}
	cout << "中间第二相切面" << V_connect2 << endl;



	//--------------------------计算线与面交点-------速度慢，有多余点
	Eigen::MatrixXd ei2, pi2;
	igl::fast_find_intersections(tree1, V, F, V_connect2, F_connect, ei2, pi2);//线与面的求交求不到交点
	//std::cout << "相交点数量" << pi << "时间" << time.getTimeSeconds() << std::endl;
	//std::cout << "相交点数量" << ei << std::endl;
	/*std::vector<Eigen::Vector3d> conn_point;
	for (size_t i = 0; i < pi.rows(); i++)
	{
		conn_point.push_back(Eigen::Vector3d(pi(i, 0), pi(i, 1), pi(i, 2)));
	}*/
	//std::sort(conn_point.begin(), conn_point.end(), compareFunction);
	//auto last = std::unique(conn_point.begin(), conn_point.end());
	//conn_point.erase(last, conn_point.end());
	if_reverse = false;
	std::set<Eigen::Vector3d, decltype(delete_dulp)*>  s2(&delete_dulp);
	for (size_t i = 0; i < pi2.rows(); i++)
	{
		s2.insert(Eigen::Vector3d(pi2(i, 0), pi2(i, 1), pi2(i, 2)));
	}
	Eigen::MatrixXd middle_P2(s2.size(), 3);
	int middle_num2 = 0;
	for (auto it = s2.begin(); it != s2.end(); it++)
	{
		middle_P2.row(middle_num2++) = *it;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2222(new pcl::PointCloud<pcl::PointXYZ>);
	for (size_t i = 0; i < s2.size(); i++)
	{
		cloud2222->points.push_back(pcl::PointXYZ(middle_P2(i, 0), middle_P2(i, 1), middle_P2(i, 2)));
	}
	pcl::io::savePLYFileASCII("second_line.ply", *cloud2222);



	decide_sort(x_vector, y_vector, true);//第一相切方向 ******步长方向

	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_middle_o2p(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_middle_p2e(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_middle2_o2p(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_middle2_p2e(new pcl::PointCloud<pcl::PointXYZ>);
	int peak_next;
	bool if_first = true;
	for (size_t i = 0; i < middle_P.rows(); i++)
	{
		if (sort_indi == 1)
		{
			if (middle_P(i, sort_indi) > y_vector.norm() / 2)
			{
				if (if_first)
				{
					peak_next = i;
					if_first = false;
				}
				pcl_middle_p2e->points.insert(pcl_middle_p2e->points.begin(), pcl::PointXYZ(middle_P(i, 0), middle_P(i, 1), middle_P(i, 2)));
			}
			else if (middle_P(i, sort_indi) < y_vector.norm() / 2)
			{
				pcl_middle_o2p->points.push_back(pcl::PointXYZ(middle_P(i, 0), middle_P(i, 1), middle_P(i, 2)));
			}
		}
		if (sort_indi == 0)
		{
			if (middle_P(i, sort_indi) > x_vector.norm() / 2 - 6)
			{
				if (if_first)
				{
					peak_next = i;
					if_first = false;
				}
				pcl_middle_p2e->points.insert(pcl_middle_p2e->points.begin(), pcl::PointXYZ(middle_P(i, 0), middle_P(i, 1), middle_P(i, 2)));
			}
			else if (middle_P(i, sort_indi) < x_vector.norm() / 2 - 6)
			{
				pcl_middle_o2p->points.push_back(pcl::PointXYZ(middle_P(i, 0), middle_P(i, 1), middle_P(i, 2)));
			}
		}
	}
	decide_sort(x_vector, y_vector, false);//第二相切方向 ******行距方向
	for (size_t i = 0; i < middle_P2.rows(); i++)
	{
		if (sort_indi == 1)
		{
			if (middle_P2(i, sort_indi) > y_vector.norm() / 2)
			{
				pcl_middle2_p2e->points.insert(pcl_middle2_p2e->points.begin(), pcl::PointXYZ(middle_P2(i, 0), middle_P2(i, 1), middle_P2(i, 2)));
			}
			else if (middle_P2(i, sort_indi) < y_vector.norm() / 2)
			{
				pcl_middle2_o2p->points.push_back(pcl::PointXYZ(middle_P2(i, 0), middle_P2(i, 1), middle_P2(i, 2)));
			}
		}
		if (sort_indi == 0)
		{
			if (middle_P2(i, sort_indi) > x_vector.norm() / 2 - 6)
			{
				pcl_middle2_p2e->points.insert(pcl_middle2_p2e->points.begin(), pcl::PointXYZ(middle_P2(i, 0), middle_P2(i, 1), middle_P2(i, 2)));
			}
			else if (middle_P2(i, sort_indi) < x_vector.norm() / 2 - 6)
			{
				pcl_middle2_o2p->points.push_back(pcl::PointXYZ(middle_P2(i, 0), middle_P2(i, 1), middle_P2(i, 2)));
			}
		}
	}

	pcl::io::savePLYFile("model_file/first_o2p.ply", *pcl_middle_o2p);
	pcl::io::savePLYFile("model_file/first_p2e.ply", *pcl_middle_p2e);
	pcl::io::savePLYFile("model_file/second_o2p.ply", *pcl_middle2_o2p);
	pcl::io::savePLYFile("model_file/second_p2e.ply", *pcl_middle2_p2e);

	decide_sort(x_vector, y_vector, true);//第一相切方向 ******行距方向
	Eigen::MatrixXd bez_CtlP(4, 3);
	bez_CtlP.row(0) = middle_P.row(peak_next - 2);
	bez_CtlP.row(1) = middle_P.row(peak_next - 1);
	bez_CtlP.row(2) = middle_P.row(peak_next);
	bez_CtlP.row(3) = middle_P.row(peak_next + 1);
	double t;
	t = std::abs((bez_CtlP(0, sort_indi) - step.norm() / 2) / (bez_CtlP(3, sort_indi) - bez_CtlP(0, sort_indi)));
	for (int i = 0; i <= 3; i++)
	{
		peak += binomialCoefficient(3, i) * std::pow(1 - t, 3 - i) * std::pow(t, i) * bez_CtlP.row(i);
	}
	//**********************************点云分割*******************
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> seg_cloud;
	one2four(x_vector, y_vector, seg_cloud);
	seg_cloud[0]->insert(seg_cloud[0]->end(), pcl_middle_o2p->begin(), pcl_middle_o2p->end());
	seg_cloud[0]->insert(seg_cloud[0]->end(), pcl_middle2_o2p->begin(), pcl_middle2_o2p->end());
	seg_cloud[0]->points.push_back(pcl::PointXYZ(peak[0], peak[1], peak[2]));
	seg_cloud[1]->insert(seg_cloud[1]->end(), pcl_middle_p2e->begin(), pcl_middle_p2e->end());
	seg_cloud[1]->insert(seg_cloud[1]->end(), pcl_middle2_o2p->begin(), pcl_middle2_o2p->end());
	seg_cloud[1]->points.push_back(pcl::PointXYZ(peak[0], peak[1], peak[2]));
	seg_cloud[2]->insert(seg_cloud[2]->end(), pcl_middle_p2e->begin(), pcl_middle_p2e->end());
	seg_cloud[2]->insert(seg_cloud[2]->end(), pcl_middle2_p2e->begin(), pcl_middle2_p2e->end());
	seg_cloud[2]->points.push_back(pcl::PointXYZ(peak[0], peak[1], peak[2]));
	seg_cloud[3]->insert(seg_cloud[3]->end(), pcl_middle_o2p->begin(), pcl_middle_o2p->end());
	seg_cloud[3]->insert(seg_cloud[3]->end(), pcl_middle2_p2e->begin(), pcl_middle2_p2e->end());
	seg_cloud[3]->points.push_back(pcl::PointXYZ(peak[0], peak[1], peak[2]));
	for (int i = 0; i < seg_cloud.size(); i++)
	{
		std::stringstream seg_name;
		seg_name << "segment_" << i << "_" << seg_cloud.size() << ".ply";
		//pcl::io::savePLYFileBinary (seg_name.str(), *seg_cloud[i]);//debug报错  release不报错
	}
	pcl::PolygonMesh::Ptr mesh_0(new pcl::PolygonMesh);
	delau_recon(seg_cloud[0], mesh_0, false);
	pcl::io::savePLYFileBinary("model_file/mesh_0.ply", *mesh_0);
	tran_v = V;//保存整体点云
	auto tran_f = F;
	igl::readPLY("model_file/mesh_0.ply", V, F);
	igl::per_vertex_normals(V, F, N);// 计算每个顶点的法向量
	//igl::boundary_loop(F, bnd);
	Eigen::MatrixXd seed_0(pcl_middle_o2p->size(), 3);
	for (size_t i = 0; i < pcl_middle_o2p->size(); i++)
	{
		auto it = pcl_middle_o2p->points.at(i);
		seed_0.row(i) << it.x, it.y, it.z;
	}
	qDebug() << 3333333;
	tree1.init(V, F);
	end_point = Eigen::Vector3d(pcl_middle2_o2p->points.at(0).x, pcl_middle2_o2p->points.at(0).y, pcl_middle2_o2p->points.at(0).z);

	compute_spacing(seed_0, -1);
	for (int i = 0; i < single_array.size(); ++i)
	{
		if (i % 2 == 1)
		{
			for (size_t j = 0; j < single_array[i].rows() / 2; j++)
			{
				single_array[i].row(j).swap(single_array[i].row(single_array[i].rows() - 1 - j));
			}
		}
	}


	line_array.push_back(single_array);

	pcl::PointCloud<pcl::PointXYZ>::Ptr tempory_cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	for (size_t i = 0; i < single_array.size(); i++)
	{
		for (size_t j = 0; j < single_array[i].rows(); j++)
		{
			tempory_cloud1->points.push_back(pcl::PointXYZ(single_array[i](j, 0), single_array[i](j, 1), single_array[i](j, 2)));
		}
	}
	qDebug() << single_array.size();
	pcl::io::savePLYFileBinary("single_cloud_0.ply", *tempory_cloud1);

	sdfa = 0;
	pcl::PolygonMesh::Ptr mesh_1(new pcl::PolygonMesh);;
	delau_recon(seg_cloud[1], mesh_1, false);
	pcl::io::savePLYFileBinary("model_file/mesh_1.ply", *mesh_1);
	igl::readPLY("model_file/mesh_1.ply", V, F);
	tree1.init(V, F);
	igl::per_vertex_normals(V, F, N);// 计算每个顶点的法向量
	Eigen::MatrixXd seed_1(pcl_middle_p2e->size(), 3);
	for (size_t i = 0; i < pcl_middle_p2e->size(); i++)
	{
		auto it = pcl_middle_p2e->points.at(i);
		seed_1.row(i) << it.x, it.y, it.z;
	}
	single_array.clear();
	compute_spacing(seed_1, -1, false);
	for (int i = 0; i < single_array.size(); ++i)
	{
		if (i % 2 == 1)
		{
			for (size_t j = 0; j < single_array[i].rows() / 2; j++)
			{
				single_array[i].row(j).swap(single_array[i].row(single_array[i].rows() - 1 - j));
			}
		}
	}
	line_array.push_back(single_array);

	pcl::PointCloud<pcl::PointXYZ>::Ptr tempory_cloud3(new pcl::PointCloud<pcl::PointXYZ>);
	for (size_t i = 0; i < single_array.size(); i++)
	{
		for (size_t j = 0; j < single_array[i].rows(); j++)
		{
			tempory_cloud3->points.push_back(pcl::PointXYZ(single_array[i](j, 0), single_array[i](j, 1), single_array[i](j, 2)));
		}
	}
	pcl::io::savePLYFileBinary("single_cloud_1.ply", *tempory_cloud3);

	sdfa = 0;
	pcl::PolygonMesh::Ptr mesh_2(new pcl::PolygonMesh);;
	delau_recon(seg_cloud[2], mesh_2, false);
	pcl::io::savePLYFileBinary("model_file/mesh_2.ply", *mesh_2);
	igl::readPLY("model_file/mesh_2.ply", V, F);
	tree1.init(V, F);
	igl::per_vertex_normals(V, F, N);// 计算每个顶点的法向量
	Eigen::MatrixXd seed_2(pcl_middle_p2e->size(), 3);
	for (size_t i = 0; i < pcl_middle_p2e->size(); i++)
	{
		auto it = pcl_middle_p2e->points.at(i);
		seed_2.row(i) << it.x, it.y, it.z;
	}
	end_point = Eigen::Vector3d(pcl_middle2_p2e->points.at(0).x, pcl_middle2_p2e->points.at(0).y, pcl_middle2_p2e->points.at(0).z);
	single_array.clear();
	compute_spacing(seed_2, 1);
	for (int i = 0; i < single_array.size(); ++i)
	{
		if (i % 2 == 1)
		{
			for (size_t j = 0; j < single_array[i].rows() / 2; j++)
			{
				single_array[i].row(j).swap(single_array[i].row(single_array[i].rows() - 1 - j));
			}
		}
	}
	line_array.push_back(single_array);

	pcl::PointCloud<pcl::PointXYZ>::Ptr tempory_cloud5(new pcl::PointCloud<pcl::PointXYZ>);
	for (size_t i = 0; i < single_array.size(); i++)
	{
		for (size_t j = 0; j < single_array[i].rows(); j++)
		{
			tempory_cloud5->points.push_back(pcl::PointXYZ(single_array[i](j, 0), single_array[i](j, 1), single_array[i](j, 2)));
		}
	}
	pcl::io::savePLYFileBinary("single_cloud_2.ply", *tempory_cloud5);

	sdfa = 0;
	pcl::PolygonMesh::Ptr mesh_3(new pcl::PolygonMesh);;
	delau_recon(seg_cloud[3], mesh_3, false);
	pcl::io::savePLYFileBinary("model_file/mesh_3.ply", *mesh_3);
	igl::readPLY("model_file/mesh_3.ply", V, F);
	tree1.init(V, F);
	igl::per_vertex_normals(V, F, N);// 计算每个顶点的法向量
	Eigen::MatrixXd seed_3(pcl_middle_o2p->size(), 3);
	for (size_t i = 0; i < pcl_middle_o2p->size(); i++)
	{
		auto it = pcl_middle_o2p->points.at(i);
		seed_3.row(i) << it.x, it.y, it.z;
	}
	single_array.clear();
	compute_spacing(seed_3, 1, false);
	for (int i = 0; i < single_array.size(); ++i)
	{
		if (i % 2 == 1)
		{
			for (size_t j = 0; j < single_array[i].rows() / 2; j++)
			{
				single_array[i].row(j).swap(single_array[i].row(single_array[i].rows() - 1 - j));
			}
		}
	}
	line_array.push_back(single_array);

	pcl::PointCloud<pcl::PointXYZ>::Ptr tempory_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (size_t i = 0; i < single_array.size(); i++)
	{
		for (size_t j = 0; j < single_array[i].rows(); j++)
		{
			tempory_cloud->points.push_back(pcl::PointXYZ(single_array[i](j, 0), single_array[i](j, 1), single_array[i](j, 2)));
		}
	}
	pcl::io::savePLYFileBinary("single_cloud_3.ply", *tempory_cloud);


	sdfa = 0;
	int postive = 1;//切向反向
	V = tran_v;
	F = tran_f;
	tree1.init(V, F);
	igl::per_vertex_normals(V, F, N);// 计算每个顶点的法向量
	for (size_t k = 0; k < line_array.size(); k++)//四个拓扑片
	{
		for (size_t j = 0; j < line_array[k].size(); j++)//每片上的轨迹行
		{
			postive = 1;
			if (j % 2 == 1)//双数行切向反向
			{
				postive = -1;
			}
			Eigen::MatrixXd regen_P = line_array[k][j];
			Eigen::VectorXi new_I;//所在面索引
			Eigen::MatrixXd new_C;//最近点
			Eigen::VectorXd sqrD;//最小距离
			tree1.squared_distance(V, F, regen_P, sqrD, new_I, new_C);
			regen_P = new_C;
			Eigen::MatrixXd bc(1, 3); // 点 P 在三角形 T 中的面积坐标
			std::vector<Eigen::MatrixXd> single_Line;
			for (size_t i = 0; i < regen_P.rows(); i++)
			{
				bc << 0, 0, 0;
				curve->points.push_back(pcl::PointXYZ(regen_P(i, 0), regen_P(i, 1), regen_P(i, 2)));
				Eigen::MatrixXd single_rota(3, 3);
				Eigen::Vector3d single_P = regen_P.row(i);
				if (single_P.isApprox(Eigen::VectorXd(V.row(F(new_I[i], 0)))))
				{
					bc << 1, 0, 0;
				}
				else if (single_P.isApprox(Eigen::VectorXd(V.row(F(new_I[i], 1)))))
				{
					bc << 0, 1, 0;
				}
				else if (single_P.isApprox(Eigen::VectorXd(V.row(F(new_I[i], 2)))))
				{
					bc << 0, 0, 1;
				}
				else
				{
					try
					{
						double tri_area = (Eigen::Vector3d(V.row(F(new_I[i], 1))).cross(Eigen::Vector3d(V.row(F(new_I[i], 2))))).norm();
						bc << ((single_P - Eigen::Vector3d(V.row(F(new_I[i], 1)))).cross(single_P - Eigen::Vector3d(V.row(F(new_I[i], 2))))).norm() / tri_area, ((single_P - Eigen::Vector3d(V.row(F(new_I[i], 0)))).cross(single_P - Eigen::Vector3d(V.row(F(new_I[i], 2))))).norm() / tri_area, ((single_P - Eigen::Vector3d(V.row(F(new_I[i], 1)))).cross(single_P - Eigen::Vector3d(V.row(F(new_I[i], 0))))).norm() / tri_area;
						//igl::barycentric_coordinates(P, Eigen::VectorXd(V.row(F(indice, 0))), Eigen::VectorXd(V.row(F(indice, 1))), Eigen::VectorXd(V.row(F(indice, 2))), bc);
					}
					catch (const std::exception&)
					{
						cout << "面积坐标求解失败" << endl;
					}
				}
				//cout << "打电话副科级爱好" << bc << endl;
				//cout << F(new_I[i], 0) << endl;
				//cout << N.row(F(new_I[i], 0)) << endl;
				N.row(F(new_I[i], 0));
				Eigen::Vector3d perv_normal = bc(0) * N.row(F(new_I[i], 0)) + bc(1) * N.row(F(new_I[i], 1)) + bc(2) * N.row(F(new_I[i], 2));
				perv_normal.normalize();
				perv_normal *= -1;  //工具坐标系法向量朝up
				single_rota.col(2) = perv_normal;
				Eigen::Vector3d perv_tangen;
				if (i == 0)
				{
					perv_tangen = Eigen::Vector3d(new_C(i + 1, 0), new_C(i + 1, 1), new_C(i + 1, 2)) - single_P;
				}
				else if (i == (new_C.rows() - 1))
				{
					perv_tangen = single_P - Eigen::Vector3d(new_C(i - 1, 0), new_C(i - 1, 1), new_C(i - 1, 2));
				}
				else
				{
					perv_tangen = (Eigen::Vector3d(new_C(i + 1, 0), new_C(i + 1, 1), new_C(i + 1, 2)) - Eigen::Vector3d(new_C(i - 1, 0), new_C(i - 1, 1), new_C(i - 1, 2))) / 2;
				}
				perv_tangen.normalize();
				perv_tangen *= postive;
				single_rota.col(0) = perv_tangen;
				single_rota.col(1) = perv_normal.cross(perv_tangen);
				single_Line.push_back(single_rota);
				rotation.push_back(single_rota);
			}
		}
	}
	pcl::io::savePLYFileBinary("model_file/whole_line.ply", *curve);

	igl::writePLY("model_file/delau_mesh.ply", tran_v, tran_f, igl::FileEncoding::Binary);
}



