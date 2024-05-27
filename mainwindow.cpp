#include "mainwindow.h"
#include "ui_mainwindow.h"

//#include <ccHObject.h>
//#include <ccPointCloud.h>
//#include <ccRegistrationTools.h>

#include <CCCoreLib/PointCloud.h>
#include <CCCorelib/RegistrationTools.h>
#include <CCCoreLib/GenericIndexedCloudPersist.h>

MainWindow::MainWindow(QWidget* parent)
	: QMainWindow(parent)
	, ui(new Ui::MainWindow)
{
	
	ui->setupUi(this);
	//创建TCP客户端
	qDebug("SlotHasConnected ThreadID:%d", QThread::currentThreadId());
	m_client = new TcpClient();
	m_thread = new QThread();
	m_client->moveToThread(m_thread);
	m_thread->start();

	//连接到机械臂的TCP服务
	connect(this, &MainWindow::ClientConnect, this, [&](const QString& address, quint16 port) {
		try
		{
			ui->debug_txt->append(address);
			ui->debug_txt->append(QString::number(port));
			m_client->ClientConnectToHost(address, port);
		}
		catch (const std::exception& sa)
		{
			qDebug() << sa.what();
			ui->debug_txt->append("机械臂连接失败!!!");
		}}, Qt::AutoConnection);
	//文件发送
	connect(this, &MainWindow::ClientSending, this, [&](const std::vector<std::vector<double>>& crave0, const double& speed0) {
		try
		{
			if (!m_client->IsCom)
			{
				m_client->ClientSendingData(crave0, speed0);
			}
			else
			{
				qDebug() << "正在和机械臂通信中！！！！";
				ui->debug_txt->append("正在和机械臂通信中！！！！");
			}
		}
		catch (const std::exception&)
		{
			ui->debug_txt->append("轨迹发送失败!!!");
		}}, Qt::AutoConnection);
	//接收服务端发送的数据 /从子线程到主线程的队列连接
	connect(MainWindow::m_client, &TcpClient::SignalPublishFormatRecvData, this, [&](const QString c_btaData) {
		if (ui->debug_txt->toPlainText().size() > 2 * 1024 * 1024)
			ui->debug_txt->clear();
		ui->debug_txt->append(c_btaData);
		ui->debug_txt->moveCursor(QTextCursor::End);
		}, Qt::AutoConnection);
	//connect(ui->check_com_btn, &QPushButton::clicked, this, &MainWindow::SendData);
	connect(ui->check_com_btn, &QPushButton::clicked, this, [&]() {
		run_all();
		SendData();
		});

	connect(ui->stop_btn, &QPushButton::clicked, this, [&]()
		{
			if (!m_client->IsCom)
			{
				m_client->Single_ClientSendingData("@Z3&");
			}
			else
			{
				qDebug() << "正在和机械臂通信中！！！！";
				ui->debug_txt->append("正在和机械臂通信中！！！！");
			}
			//emit ClientSending("@Z3&");
		});

	model = new QStandardItemModel(ui->treVcloud_info);
	model->setHorizontalHeaderLabels(QStringList() << QStringLiteral("--cloud--DB-Tree--"));
	ui->treVcloud_info->setHeaderHidden(false);
	model->setItem(0, 0, new QStandardItem("三维扫描"));
	model->setItem(1, 0, new QStandardItem("模型处理"));
	model->setItem(2, 0, new QStandardItem("打磨轨迹"));
	model->setItem(3, 0, new QStandardItem("在线匹配结果"));
	ui->treVcloud_info->setModel(model);
	ui->treVcloud_info->expandAll();
	ui->treVcloud_info->setSelectionMode(QAbstractItemView::SingleSelection);//设置单选
	//打开右键菜单属性
	ui->treVcloud_info->setContextMenuPolicy(Qt::CustomContextMenu);
	//右键菜单
	QMenu* tre_menu = new QMenu(ui->treVcloud_info);
	tre_menu->addAction("添加");
	connect(tre_menu->addAction("删除"), &QAction::triggered, this, [=]() {
		QModelIndex selectedIndex = ui->treVcloud_info->selectionModel()->currentIndex();
		if (selectedIndex.isValid()) {
			QStandardItem* parentItem = model->itemFromIndex(selectedIndex.parent());
			if (parentItem) {
				//iqDebug() << "Selected Item: " << parentItem->text();
				QStandardItem* selectedItem = model->itemFromIndex(selectedIndex);
				view_deledata(selectedItem->parent()->text().toStdString() + "_" + selectedItem->text().toStdString());
				parentItem->removeRow(selectedIndex.row());
			}
		}});
	//响应右键菜单信号槽
	connect(ui->treVcloud_info, &QTreeView::customContextMenuRequested,
		this, [=](const QPoint& pos) {
			QModelIndex selectedIndex = ui->treVcloud_info->currentIndex();
			if (selectedIndex.parent().isValid())
			{
				tre_menu->exec(ui->treVcloud_info->mapToGlobal(pos));
			}});
	// 连接treeview选中项变动信号
	connect(ui->treVcloud_info->selectionModel(), &QItemSelectionModel::selectionChanged,
		this, [=](const QItemSelection& selected, const QItemSelection& deselected) {//selected新选中，deselected取消选中项
			QModelIndex selectedIndex = ui->treVcloud_info->selectionModel()->currentIndex();
			if (selectedIndex.parent().isValid())
			{
				QStandardItem* selectedItem = model->itemFromIndex(selectedIndex);
				view_updata(selectedItem->parent()->text().toStdString() + "_" + selectedItem->text().toStdString());
				qDebug() << "Selection changed: selected items count = " << selected.indexes().count();
			}
		});

	/**************************************初始化******************************************/
		//初始化PCL显示控件
	viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	vtkNew<vtkGenericOpenGLRenderWindow> window;
	window->AddRenderer(viewer->getRendererCollection()->GetFirstRenderer());
	ui->qvtkWidget->setRenderWindow(window.Get());
	ui->qvtkWidget->update();
	/**************************************菜单设置******************************************/
	ui->menubar->setStyleSheet("font-size : 12px");
	connect(ui->open_action, &QAction::triggered, this, [=]() {Open_clicked(); });//读取文件
	connect(ui->save_action, &QAction::triggered, this, [=]() {Save_clicked(); });//保存文件
	connect(ui->quit_action, &QAction::triggered, this, [=]() {close(); });
	connect(ui->arm_set_act, &QAction::triggered, this, [=]() {Open_clicked(); });
	//------------------------------------------------------------------

}

MainWindow::~MainWindow()
{
	delete ui;
}

//-------------------------------------function----------------------------------------------------//
//读取有逗号的xyzi
pcl::PointCloud<pcl::PointXYZ>::Ptr get_pointcloud_from_txt(const std::string& file_path)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new  pcl::PointCloud<pcl::PointXYZ>);
	std::ifstream file(file_path.c_str());//c_str()：生成一个const char*指针，指向以空字符终止的数组。
	while (file)
	{
		std::string s;
		if (!getline(file, s)) break;

		std::istringstream ss(s);
		std::vector <std::string> record;
		std::vector <float> point;
		pcl::PointXYZ p;
		while (ss)
		{
			std::string s;
			if (!getline(ss, s, ',')) break;
			point.push_back(atof(s.c_str()));

		}

		p.x = point[0];
		p.y = point[1];
		p.z = point[2];
		//        p.intensity = point[3];

		cloud->push_back(p);

	}
	file.close();

	return cloud;
}

//-------------------------------------file-----------------------------------------------------
//读取点云
void MainWindow::Open_clicked()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	QString fileName = QFileDialog::getOpenFileName(this, tr("open  file"),
		"", tr("pcb files(*.pcd *.ply *.xyz) ;;All files (*.*)"));

	if (fileName.isEmpty())
	{
		return;
	}
	QFileInfo fileInfo(fileName);
	QString baseName = fileInfo.fileName(); // 获取文件名
	if (fileName.endsWith("ply"))
	{
		qDebug() << fileName;
		if (pcl::io::loadPLYFile<pcl::PointXYZ>(fileName.toStdString(), *cloud) == -1) //* load the file
		{
			qDebug() << "Couldn't read file  \n";
			return;
		}
	}
	else if (fileName.endsWith("pcd"))
	{
		qDebug() << fileName;
		if (pcl::io::loadPCDFile<pcl::PointXYZ>(fileName.toStdString(), *cloud) == -1) //* load the file
		{
			qDebug() << "Couldn't read pcd file  \n";
			return;
		}
	}
	else if (fileName.endsWith("xyz"))
	{
		QFuture<pcl::PointCloud<pcl::PointXYZ>::Ptr> future =
			QtConcurrent::run(get_pointcloud_from_txt, std::string(fileName.toStdString()));
		while (!future.isFinished())
		{
			QApplication::processEvents(QEventLoop::AllEvents, 100);
		}
		cloud = future.result();
	}
	else {
		QMessageBox::warning(this, "Warning", "点云读取格式错误！");
	}
	AddPointView(baseName, cloud, 0);
	//-----------------------------------------------------
	//cloud_vec.push_back(cloud.makeShared());
	//cloud_index.push_back(1);

}

template<typename Tyn>
void  MainWindow::AddPointView(const QString& baseName, const Tyn& cloud, const int& row_indice)
{
	std::string key_str = model->item(row_indice, 0)->text().toStdString() + "_" + baseName.toStdString();
	string_Pcloud[key_str] = *cloud;
	QStandardItem* itemFolder = new QStandardItem(m_publicIconMap[QStringLiteral("treeItem_folder")], baseName);
	//itemFolder->setCheckable(true);
	//itemFolder->setCheckState(Qt::Checked);//获取选中状态
	model->item(row_indice, 0)->appendRow(itemFolder);
	QModelIndex parent_index = model->item(row_indice, 0)->index();
	QModelIndex childIndex = model->index(model->itemFromIndex(parent_index)->rowCount() - 1, 0, parent_index);
	ui->treVcloud_info->setCurrentIndex(childIndex);
	//    item = new QStandardItem(m_publicIconMap[QStringLiteral("treeItem_dataItem")],QStringLiteral("%1").arg(cloud_vec.size()-1));
	//    item->setCheckable(true);
	//    item->setCheckState(Qt::Checked);//获取选中状态
	//    itemFolder->appendRow(item);
		//-----------------------------------------------------------
	view_updata(key_str);
}

void MainWindow::view_updata(std::string cloud_name)
{
	auto update_cloud = string_Pcloud.find(cloud_name);
	if (update_cloud == string_Pcloud.end())
	{
		QMessageBox::warning(this, "Warning", "所选点云为空！");
	}

	int size;
	viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	vtkNew<vtkGenericOpenGLRenderWindow> window;
	window->AddRenderer(viewer->getRendererCollection()->GetFirstRenderer());
	ui->qvtkWidget->setRenderWindow(window.Get());

	viewer->removeAllPointClouds();
	viewer->removeAllShapes();

	int v1(0), v2(0);
	viewer->setBackgroundColor(0, 0, 0);
	//viewer->setWindowName("点云显示");
	//viewer->addText("point clouds", 10, 10, "v1_text");
   // viewer->addText("filtered point clouds", 10, 10, "v2_text");
	//viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud", v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, "sample cloud", v1);

	//qDebug() << typeid(update_cloud->second).name();
	if (std::holds_alternative<pcl::PointCloud<pcl::PointXYZ>>(update_cloud->second))
	{
		pcl::PointCloud<pcl::PointXYZ> up_c = std::get<pcl::PointCloud<pcl::PointXYZ>>(update_cloud->second);
		size = up_c.size();
		if (size == 0)
		{
			string_Pcloud.erase(cloud_name);
			QMessageBox::warning(this, "Warning", "点云数量为零，不添加或删除该点云！");
		}
		//pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ>render(, "intensity");
		viewer->addPointCloud(up_c.makeShared(), cloud_name, v1);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, cloud_name, v1);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, cloud_name, v1);
	}
	else if (std::holds_alternative<pcl::PointCloud<pcl::PointXYZRGB>>(update_cloud->second))
	{
		pcl::PointCloud<pcl::PointXYZRGB> up_c = std::get<pcl::PointCloud<pcl::PointXYZRGB>>(update_cloud->second);
		//pcl::io::savePCDFileASCII(cloud_name+".pcd", up_c);
		size = up_c.size();
		if (size == 0)
		{
			string_Pcloud.erase(cloud_name);
			QMessageBox::warning(this, "Warning", "点云数量为零，不添加或删除该点云！");
		}
		//pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ>render(, "intensity");
		viewer->addPointCloud(up_c.makeShared(), cloud_name, v1);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, cloud_name, v1);
		int scale = 5;
		if (cloud_name == "在线匹配结果_tranpath")
		{
			for (size_t i = 0; i < rotation.size(); i += 4)
			{
				for (size_t j = 0; j < rotation[i].cols(); j++)
				{

					pcl::PointXYZ p1 = pcl::PointXYZ(up_c.at(i).x, up_c.at(i).y, up_c.at(i).z);
					pcl::PointXYZ p2 = pcl::PointXYZ(up_c.at(i).x + scale * rotation[i](0, j), up_c.at(i).y + scale * rotation[i](1, j), up_c.at(i).z + scale * rotation[i](2, j));
					stringstream ss;
					ss << "rotation" << i << "col" << j;
					if (j==0)
					{
						viewer->addLine<pcl::PointXYZ>(p1, p2, 255, 0, 0, ss.str());
					}
					else if (j==1)
					{
						viewer->addLine<pcl::PointXYZ>(p1, p2, 0, 255, 0, ss.str());
					}
					else
					{
						viewer->addLine<pcl::PointXYZ>(p1, p2, 0, 0, 255, ss.str());
					}
					//viewer->addArrow<pcl::PointXYZ>(p1, p2, 0, 0, 255, ss.str());  //带箭头
				}
			}

		}
		
	}
	else if (std::holds_alternative<pcl::PolygonMesh>(update_cloud->second))
	{
		pcl::PolygonMesh up_c = std::get<pcl::PolygonMesh>(update_cloud->second);
		size = up_c.cloud.data.size();
		if (size == 0)
		{
			string_Pcloud.erase(cloud_name);
			QMessageBox::warning(this, "Warning", "点云数量为零，不添加或删除该点云！");
		}
		viewer->addPolygonMesh(up_c, cloud_name);
	}

	QString PointSize = QString("%1").arg(size);

	viewer->resetCamera();
	ui->qvtkWidget->update();
}

void MainWindow::Save_clicked()
{
	/*QString filename = QFileDialog::getSaveFileName(this, tr("Open point cloud"), "", tr("Point cloud data (*.pcd *.ply)"));

	if (cloud.empty())
	{
		return;
	}
	else
	{

		if (filename.isEmpty())
			return;

		int return_status;
		if (filename.endsWith(".pcd", Qt::CaseInsensitive))
			return_status = pcl::io::savePCDFileBinary(filename.toStdString(), *cloud);
		else if (filename.endsWith(".ply", Qt::CaseInsensitive))
			return_status = pcl::io::savePLYFileBinary(filename.toStdString(), cloud);
		else
		{
			filename.append(".ply");
			return_status = pcl::io::savePLYFileBinary(filename.toStdString(), cloud);
		}

		if (return_status != 0)
		{
			PCL_ERROR("Error writing point cloud %s\n", filename.toStdString().c_str());
			return;
		}
	}*/

}
//-------------------------------------view-----------------------------------------------------
void MainWindow::view_deledata(std::string cloud_name)
{
	viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	vtkNew<vtkGenericOpenGLRenderWindow> window;
	window->AddRenderer(viewer->getRendererCollection()->GetFirstRenderer());
	ui->qvtkWidget->setRenderWindow(window.Get());
	viewer->removeAllPointClouds();
	viewer->removeAllShapes();
	ui->qvtkWidget->update();
	auto delete_pc = string_Pcloud.find(cloud_name);
	if (delete_pc != string_Pcloud.end())
	{
		string_Pcloud.erase(delete_pc);
	}
	else
	{
		qDebug() << "没有找到删除索引所对应的数据";
	}
}


void MainWindow::point_cloud_process(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	// -------------------------------体素下采样-------------------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	grid_filter(cloud, cloud_filtered, 1.0);
	//visualize_cloud(cloud_filtered);
	AddPointView("下采样", cloud_filtered, 1);
	// -------------------------------聚类分割-------------------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_p(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_view(new pcl::PointCloud<pcl::PointXYZRGB>);
	cluster_view = cluster_point(cloud_filtered, cluster_p);
	AddPointView("聚类结果", cluster_view, 1);
	AddPointView("聚类分割", cluster_p, 1);
	pcl::io::savePLYFileBinary("model_file\\paper_picture\\cluster_p.ply", *cluster_p);
	//visualize_cloud(cluster_p);
	// -------------------------------特征点的孔洞边界提取-------------------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr bound_p(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr bound_p_max(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector< int> bound_indice = boundary_p(cluster_p, bound_p, M_PI / 6);
	//visualize_cloud_merge(cluster_p, bound_p);  
	pcl::PointCloud<pcl::PointXYZ>::Ptr out_bound = hole_extra(cluster_p, bound_p, bound_indice, bound_p_max);//out_bound为外围边界点云
	//visualize_cloud(bound_p_max);
	// -------------------------------特征点边缘去除-------------------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr delete_cloud = delete_point(cluster_p, bound_p_max);
	AddPointView("特征点删除", bound_p_max, 1);
	pcl::io::savePLYFileBinary("model_file\\paper_picture\\delete_tips_cloud.ply", *delete_cloud);
	pcl::io::savePLYFileBinary("model_file\\paper_picture\\delete_tips.ply", *bound_p_max);
	//pcl::io::savePLYFile("特征点删除后.ply", *delete_cloud);
	//visualize_cloud(delete_cloud);
	//mean_filter(cluster_p, filter_2);
	//visualize_cloud(filter_2);
	// -------------------------------点云平滑-------------------------------------
	//整体点云高斯平滑
	pcl::StopWatch t11;
	pcl::PointCloud<pcl::PointXYZ>::Ptr smooth_1(new pcl::PointCloud<pcl::PointXYZ>);
	//guass_filter(delete_cloud, smooth_1);
	//pcl::io::savePLYFileBinary("model_file\\paper_picture\\guass_smooth_end.ply", *smooth_1);
	//pcl::io::savePLYFile("整体高斯平滑.ply", *smooth_1,true);
	//pcl::io::savePLYFile("边界点云.ply", *out_bound,true);
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr out_bound(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr smooth_1(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::loadPLYFile("边界点云.ply", *out_bound);
	pcl::io::loadPLYFile("整体高斯平滑.ply", *smooth_1);*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr smooth_2(new pcl::PointCloud<pcl::PointXYZ>);
	least_squares(delete_cloud, smooth_2, 10);
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr smooth_3(new pcl::PointCloud<pcl::PointXYZ>);
	least_squares(smooth_2, smooth_3,4);*/
	//平滑合并
	smooth_1->insert(smooth_1->end(), smooth_2->begin(), smooth_2->end());
	qDebug() << t11.getTime();
	//pcl::io::savePLYFile("整体高斯+边缘最小二乘平滑.ply", *smooth_1,true);
	//visualize_cloud_merge(cluster_p, smooth_1);
	pcl::PointCloud<pcl::PointXYZ>::Ptr filter_end(new pcl::PointCloud<pcl::PointXYZ>);
	deduplication(smooth_1, filter_end, 0.2);
	/*smooth_1->clear();
	grid_filter(filter_end, smooth_1, 0.3);*/
	//pcl::io::savePLYFile("整体高斯+边缘最小二乘平滑（再次下采样）.ply", *filter_end,true);
	AddPointView("点云平滑", filter_end, 1);
	pcl::io::savePLYFileBinary("model_file\\paper_picture\\smooth_end.ply", *filter_end);
	//visualize_cloud(filter_end);
	//--------------------------------孔洞修补---------------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr bound_p_fill(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector< int> bound_indice_fill = boundary_p(smooth_1, bound_p_fill, M_PI / 3);
	pcl::PointCloud<pcl::PointXYZ>::Ptr fix_hole_bound(new pcl::PointCloud<pcl::PointXYZ>);
	hole_fill(smooth_1, bound_p_fill, bound_indice_fill, fix_hole_bound);
	pcl::PointCloud<pcl::PointXYZ>::Ptr after_hole_fill10(new pcl::PointCloud<pcl::PointXYZ>);
	grid_filter(fix_hole_bound, after_hole_fill10, 0.5);
	//pcl::io::savePLYFile("填补孔洞与修复边界后的点云信息.ply", *after_hole_fill10,true);

	AddPointView("孔洞修补", after_hole_fill10, 1);
	pcl::io::savePLYFileBinary("model_file\\paper_picture\\after_hole_fill10.ply", *after_hole_fill10);
	//visualize_cloud(after_hole_fill10);
	pcl::PointCloud<pcl::PointXYZ>::Ptr after_hole_fill15(new pcl::PointCloud<pcl::PointXYZ>);
	grid_filter(after_hole_fill10, after_hole_fill15, 3);
	pcl::io::savePLYFileBinary("model_file\\paper_picture\\after_hole_fill15.ply", *after_hole_fill15);
	//pcl::io::savePLYFile("下采样后的最终点云信息.ply", *after_hole_fill15,true);
	AddPointView("修补下采样", after_hole_fill15, 1);
	//visualize_cloud(after_hole_fill15);
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr after_hole_fill158(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPLYFile("下采样后的最终点云信息.ply", *after_hole_fill158);*/
	pcl::PolygonMesh::Ptr delau_cloud(new pcl::PolygonMesh);
	delau_recon(after_hole_fill15, delau_cloud);
	AddPointView("三角剖分", delau_cloud, 1);
	//return delau_cloud;
}


void MainWindow::pcloud_match(pcl::PointCloud<pcl::PointXYZ>::Ptr& camera_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_src, Eigen::Matrix4f& transformation_matrix1)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr grid_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	grid_filter(camera_cloud, grid_cloud, 1.5);
	pcl::PointCloud<pcl::PointXYZ>::Ptr z_filter(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(grid_cloud);
	pass.setFilterFieldName("z"); //滤波字段名被设置为Z轴方向
	pass.setFilterLimits(950, 1200); //设置在过滤方向上的过滤范围  840
	// pass.setKeepOrganized(true); // 保持有序点云结构，该功能用于有序点云才有意义。
	pass.setNegative(false); //设置保留范围内的点还是过滤掉范围内的点，标志为false时保留范围内的点
	pass.filter(*z_filter);
	AddPointView("z_filter", z_filter, 3);
	pcl::PointCloud<pcl::PointXYZ>::Ptr x_filter(new pcl::PointCloud<pcl::PointXYZ>);
	pass.setInputCloud(z_filter);
	pass.setFilterFieldName("x"); //滤波字段名被设置为Z轴方向
	pass.setFilterLimits(-120, 260); //设置在过滤方向上的过滤范围
	// pass.setKeepOrganized(true); // 保持有序点云结构，该功能用于有序点云才有意义。
	pass.setNegative(false); //设置保留范围内的点还是过滤掉范围内的点，标志为false时保留范围内的点
	pass.filter(*x_filter);
	AddPointView("x_filter", x_filter, 3);
	pcl::PointCloud<pcl::PointXYZ>::Ptr out_filer(new pcl::PointCloud<pcl::PointXYZ>);
	outlier_removal(x_filter, out_filer, 1.5);
	AddPointView("out_filer", out_filer, 3);
	// -------------------------------------------欧式聚类--------------------------------------------
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(out_filer);
	std::vector<pcl::PointIndices> cluster_indices; // 聚类索引
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;// 欧式聚类对象
	ec.setClusterTolerance(2.5);               // 设置近邻搜索的搜索半径（也即两个不同聚类团点之间的最小欧氏距离）
	ec.setMinClusterSize(100);                 // 设置一个聚类需要的最少的点数目为100
	ec.setMaxClusterSize(out_filer->size());               // 设置一个聚类需要的最大点数目为25000
	ec.setSearchMethod(tree);                  // 设置点云的搜索机制
	ec.setInputCloud(out_filer);                   // 设置输入点云
	ec.extract(cluster_indices);               // 从点云中提取聚类，并将点云索引保存在cluster_indices中
	// 遍历每个点云簇索引
	pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (const auto& cluster : cluster_indices) {
		// 遍历当前簇的所有索引
		for (const auto& idx : cluster.indices) {
			pcl::PointXYZ point = out_filer->points[idx];
			cluster_cloud->points.push_back(point);
		}
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud_gird(new pcl::PointCloud<pcl::PointXYZ>);
	grid_filter(cluster_cloud, cluster_cloud_gird, 3);

	AddPointView("cluster_cloud", cluster_cloud_gird, 3);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tar_scan(new pcl::PointCloud<pcl::PointXYZ>);
	guass_filter(cluster_cloud_gird, cloud_tar_scan, 4);
	std::vector<int> indices_tar; //保存去除的点的索引
	pcl::removeNaNFromPointCloud(*cloud_tar_scan, *cloud_tar_scan, indices_tar);
	AddPointView("cloud_tar", cloud_tar_scan, 3);
	Eigen::Matrix4f rota_angle{
		{-1.000000, 0.000000, -0.000000, 995.94},
		{ 0.000000, 1.000000, 0.000000, 266.912 },
		{ 0.000000, 0.000000, -1.000000, 1519.95 },
		{ 0.000000, 0.000000, 0.000000, 1.000000 }
	};
	Eigen::Matrix4f rota_pose{
		{0.997860372066, -0.042968012393, -0.049279283732, 0},
		{ 0.041406821460, 0.998620927334, -0.032275848091, 0 },
		{ 0.050598151982, 0.030166292563, 0.998263418674, 0 },
		{ 0.000000000000, 0.000000000000, 0.000000000000, 1 }
	};
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tar(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*cloud_tar_scan, *cloud_tar, rota_angle * rota_pose.inverse());
	pcl::io::savePLYFile("model_file/post_camera_cloud.ply", *cloud_tar, true);
	transformation_matrix1 = Eigen::Matrix4f{
		{1.000000, 0.000000, 0.000000,0},
		{ 0.000000, 1.000000, 0.000000, 0 },
		{ 0.000000, 0.000000, 1.000000, 0 },
		{ 0.000000, 0.000000, 0.000000, 1.000000 }
	};


	//Eigen::Vector4f centroid_src, centroid_tar;					// 质心
	//pcl::compute3DCentroid(*scan_cloud, centroid_src);	// 齐次坐标，（c0,c1,c2,1）
	//pcl::compute3DCentroid(*cloud_tar, centroid_tar);
	//Eigen::Matrix4f src2tar(4, 4);
	//src2tar.block<4, 1>(0, 0) << 1, 0, 0, 0;
	//src2tar.block<4, 1>(0, 1) << 0, 1, 0, 0;
	//src2tar.block<4, 1>(0, 2) << 0, 0, 1, 0;
	//src2tar.block<4, 1>(0, 3) = centroid_tar - centroid_src;
	//src2tar(3, 3) = 1;
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::transformPointCloud(*scan_cloud, *cloud_src, src2tar);
	qDebug() << "模型处理开始";
	clock_t start = clock();
	////TAR DEAL
	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_tar_PN(new pcl::PointCloud<pcl::PointNormal>());
	//pcl::PointCloud<pcl::Normal>::Ptr cloud_scene_normals(new pcl::PointCloud<pcl::Normal>);
	//pcl:: NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation_filter;
	//normal_estimation_filter.setInputCloud(cloud_tar);
	//pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree(new pcl::search::KdTree<pcl::PointXYZ>);
	//normal_estimation_filter.setSearchMethod(search_tree);
	//normal_estimation_filter.setRadiusSearch(5);
	//normal_estimation_filter.compute(*cloud_scene_normals);
	//pcl::concatenateFields(*cloud_tar, *cloud_scene_normals, *cloud_tar_PN);

	////MODEL DEAL
	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_src_PN(new pcl::PointCloud<pcl::PointNormal>());
	//pcl::PointCloud<pcl::Normal>::Ptr cloud_model_normals(new pcl::PointCloud<pcl::Normal>());
	//pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation_filter2;
	//normal_estimation_filter2.setInputCloud(cloud_src);
	//pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree2(new pcl::search::KdTree<pcl::PointXYZ>);
	//normal_estimation_filter2.setSearchMethod(search_tree2);
	//normal_estimation_filter2.setRadiusSearch(5);
	//normal_estimation_filter2.compute(*cloud_model_normals);
	//pcl::concatenateFields(*cloud_src, *cloud_model_normals, *cloud_src_PN);


	//pcl::search::KdTree<pcl::PointNormal>::Ptr search_tree3(new pcl::search::KdTree<pcl::PointNormal>);
	//pcl::PointCloud<pcl::PPFSignature>::Ptr cloud_model_ppf(new pcl::PointCloud<pcl::PPFSignature>());
	//pcl::PPFEstimation<pcl::PointNormal, pcl::PointNormal, pcl::PPFSignature> ppf_estimator;
	//ppf_estimator.setInputCloud(cloud_src_PN);
	//ppf_estimator.setInputNormals(cloud_src_PN);
	//ppf_estimator.setSearchMethod(search_tree3);
	//ppf_estimator.setRadiusSearch(13);
	//ppf_estimator.compute(*cloud_model_ppf);//之前一直出现指针报错？？？，加多维向量AGX后解决
	//pcl::PPFHashMapSearch::Ptr hashmap_search(new pcl::PPFHashMapSearch(2 * float(M_PI) / 20, 0.1f));
	//hashmap_search->setInputFeatureCloud(cloud_model_ppf);

	////将源点云和目标点云都转化为无序点云
	//	//cloud_model_input->height = 1;
	//	//cloud_model_input->is_dense = false;
	//	//cloud_scene_input->height = 1;
	//	//cloud_scene_input->is_dense = false;
	//qDebug() << "ppf处理开始";
	//pcl::PPFRegistration<pcl::PointNormal, pcl::PointNormal> ppf_registration;
	//// set parameters for the PPF registration procedure
	//ppf_registration.setSceneReferencePointSamplingRate(10);
	//ppf_registration.setPositionClusteringThreshold(2.0f);
	//ppf_registration.setRotationClusteringThreshold(12.0f / 180.0f * float(M_PI));
	//ppf_registration.setSearchMethod(hashmap_search);
	//ppf_registration.setInputSource(cloud_src_PN);
	//ppf_registration.setInputTarget(cloud_tar_PN);

	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_output_subsampled(new pcl::PointCloud<pcl::PointNormal>());

	//ppf_registration.align(*cloud_output_subsampled);
	////出现数组越界访问，无序点云OR有序点云，  //有疑问的地方？？？？？？？？？？？？？？？？？？？？？？？？？？？？？
	////修改ppf_registration.hpp中的const auto aux_size = static_cast<std::size_t>(
	//	//   std::floor(2 * M_PI / search_method_->getAngleDiscretizationStep() + 1));
	//qDebug() << "ppf处理结束";
	////转换点云XYZ格式
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output_subsampled_xyz(new pcl::PointCloud<pcl::PointXYZ>());
	//for (const auto& point : (*cloud_output_subsampled).points)
	//	cloud_output_subsampled_xyz->points.emplace_back(point.x, point.y, point.z);

	//transformation_matrix1 = ppf_registration.getFinalTransformation();
	//qDebug()<< "PPF score:" << ppf_registration.getFitnessScore();
	Eigen::Matrix4f tran_mat;
	try
	{
		HTuple hv_Pose, hv_Score, hv_Hom3d;
		Halcon_PPFMatch(&hv_Pose, &hv_Score, &hv_Hom3d);
		double* arr_Hom3d = hv_Hom3d.ToDArr();
		tran_mat.setZero();
		tran_mat << arr_Hom3d[0], arr_Hom3d[1], arr_Hom3d[2], arr_Hom3d[3] * 1000,
			arr_Hom3d[4], arr_Hom3d[5], arr_Hom3d[6], arr_Hom3d[7] * 1000,
			arr_Hom3d[8], arr_Hom3d[9], arr_Hom3d[10], arr_Hom3d[11] * 1000,
			0, 0, 0, 1;
		qDebug() << hv_Hom3d.ToString();
	}
	catch (const std::exception&)
	{
		QMessageBox::warning(this, "Warning", "匹配结果错误！");
	}

	//delete[] arr_pose;
	//arr_pose = nullptr;
	pcl::PointCloud < pcl::PointXYZ>::Ptr ppf_point(new pcl::PointCloud < pcl::PointXYZ>);
	pcl::transformPointCloud(*cloud_src, *ppf_point, tran_mat);

	/*CCCoreLib::PointCloud modecloud, datacloud;
	for (size_t i = 0; i < cloud_tar->size(); i++)
	{
		modecloud.addPoint(CCVector3(cloud_tar->points.at(i).x, cloud_tar->points.at(i).y, cloud_tar->points.at(i).z));
	}
	for (size_t i = 0; i < ppf_point->size(); i++)
	{
		datacloud.addPoint(CCVector3(ppf_point->points.at(i).x, ppf_point->points.at(i).y, ppf_point->points.at(i).z));
	}
	double finalRMS;
	unsigned int finalpcount;
	CCCoreLib::RegistrationTools::ScaledTransformation totaltran;
	CCCoreLib::ICPRegistrationTools::Parameters parameters;
	parameters.convType = CCCoreLib::ICPRegistrationTools::MAX_ERROR_CONVERGENCE;
	parameters.minRMSDecrease = 1.0e-5;
	parameters.nbMaxIterations = 80;
	parameters.adjustScale = false;
	parameters.filterOutFarthestPoints = false;
	parameters.samplingLimit = 50000;
	parameters.finalOverlapRatio = 0.8;
	parameters.modelWeights = nullptr;
	parameters.dataWeights = nullptr;
	parameters.transformationFilters = CCCoreLib::ICPRegistrationTools::SKIP_NONE;
	parameters.maxThreadCount = 0;
	parameters.useC2MSignedDistances = false;
	parameters.normalsMatching = CCCoreLib::ICPRegistrationTools::NORMALS_MATCHING::DOUBLE_SIDED_NORMALS;
	CCCoreLib::ICPRegistrationTools icp_ccc;
	icp_ccc.Register((CCCoreLib::GenericIndexedCloudPersist*)&modecloud, nullptr, (CCCoreLib::GenericIndexedCloudPersist*)&datacloud, parameters, totaltran, finalRMS, finalpcount);
	auto rotation = totaltran.R;
	auto transtation = totaltran.T;
	try
	{
		transformation_matrix1.row(0) << rotation.getValue(0, 0), rotation.getValue(0, 1), rotation.getValue(0, 2), transtation.x;
		transformation_matrix1.row(1) << rotation.getValue(1, 0), rotation.getValue(1, 1), rotation.getValue(1, 2), transtation.y;
		transformation_matrix1.row(2) << rotation.getValue(2, 0), rotation.getValue(2, 1), rotation.getValue(2, 2), transtation.z;
		transformation_matrix1.row(3) << 0, 0, 0, 1;
	}
	catch (const std::exception&)
	{
		QMessageBox::warning(this, "Warning", "匹配结果错误！");
	}

	clock_t icp = clock();
	qDebug() << "Applied Trimmed ICP iteration(s) in " << icp - start;
	pcl::PointCloud<pcl::PointXYZ>::Ptr Tricp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*cloud_src, *Tricp_cloud, transformation_matrix1);*/ // cloudcompare

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Tricp_cloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (auto& point_i : *cloud_tar)
	{
		Tricp_cloudRGB->points.push_back(pcl::PointXYZRGB(point_i.x, point_i.y, point_i.z, 255, 0, 0));
	}
	for (auto& point_i : *ppf_point)
	{
		Tricp_cloudRGB->points.push_back(pcl::PointXYZRGB(point_i.x, point_i.y, point_i.z, 0, 255, 0));
	}
	AddPointView("match_result", Tricp_cloudRGB, 3);
	camera_cloud->clear();
	camera_cloud->insert(camera_cloud->begin(), cloud_tar->begin(), cloud_tar->end());
	transformation_matrix1 = tran_mat;
}

void MainWindow::run_all()
{
	pcl::PointCloud < pcl::PointXYZ>::Ptr orign_scan(new pcl::PointCloud < pcl::PointXYZ>);
	pcl::io::loadPLYFile("model_file/orign_scan.ply", *orign_scan);
	point_cloud_process(orign_scan);
	pcl::PointCloud < pcl::PointXYZ>::Ptr curve(new pcl::PointCloud < pcl::PointXYZ>);
	Eigen::MatrixXd V;
	rotation.clear();
	param_curve("model_file/delau_mesh.ply", curve, V, rotation);
	pcl::PointCloud < pcl::PointXYZ>::Ptr model(new pcl::PointCloud < pcl::PointXYZ>);
	for (size_t i = 0; i < V.rows(); i++)
	{
		model->points.push_back(pcl::PointXYZ(V(i, 0), V(i, 1), V(i, 2)));
	}
	std::vector<int> indices_src; //保存去除的点的索引
	pcl::removeNaNFromPointCloud(*model, *model, indices_src);
	pcl::PointCloud < pcl::PointXYZRGB>::Ptr curve_ON_model(new pcl::PointCloud < pcl::PointXYZRGB>);
	for (auto& point_i : *curve)
	{
		curve_ON_model->points.push_back(pcl::PointXYZRGB(point_i.x, point_i.y, point_i.z, 0, 255, 0));
	}
	AddPointView("path", curve_ON_model, 2);
	for (auto& point_i : *model)
	{
		curve_ON_model->points.push_back(pcl::PointXYZRGB(point_i.x, point_i.y, point_i.z, 0, 0, 255));
	}
	AddPointView("path_on_model", curve_ON_model, 2);
	Eigen::Matrix4f tranMat;
	pcl::PointCloud < pcl::PointXYZ>::Ptr camera_data(new pcl::PointCloud < pcl::PointXYZ>);
	//Get_CameraData(camera_data);
	//pcl::io::savePLYFileASCII("model_file/Helmet_cloud.ply", *camera_data);
	pcl::io::loadPLYFile("model_file/Helmet_cloud.ply", *camera_data);
	string_Pcloud["camera_data.ply"] = *camera_data;
	AddPointView("camera_data.ply", camera_data, 3);
	pcloud_match(camera_data, model, tranMat);

	pcl::PointCloud < pcl::PointXYZ>::Ptr curve_tranout(new pcl::PointCloud < pcl::PointXYZ>);
	pcl::transformPointCloud(*curve, *curve_tranout, tranMat);
	Eigen::MatrixXd tranMat_d(3, 3);
	for (size_t i = 0; i < tranMat_d.rows(); i++)
	{
		for (size_t j = 0; j < tranMat_d.cols(); j++)
		{
			tranMat_d(i, j) = (double)tranMat(i, j);
		}
	}
	carve_data.clear();

	remove("model_file/rotation_tran.txt");
	for (size_t i = 0; i < rotation.size(); i++)
	{
		if (rotation.size() != curve->points.size())
		{
			QMessageBox::warning(this, "Warning", "曲线生成错误，点与欧拉角数量不符!!!!");
			break;
		}
		rotation[i] = tranMat_d * rotation[i];
		ofstream ofm;
		ofm.open("model_file/rotation_tran.txt", std::ios::out | ios::app);
		ofm << rotation[i];
		ofm << i + "\r\n" << std::endl;
		ofm.close();
		// 使用egigen将旋转矩阵转换为欧拉角	
		Eigen::Vector3d eulerAngle1 = Eigen::Matrix3d(rotation[i]).eulerAngles(2, 1, 0); //zyx顺序

		std::vector<double> tempalte;
		Eigen::Vector3d tempory(curve_tranout->points.at(i).x, curve_tranout->points.at(i).y, curve_tranout->points.at(i).z);
		//tempory += -7 * rotation[i].col(2);
		tempalte.push_back(tempory[0]);
		tempalte.push_back(tempory[1]);
		tempalte.push_back(tempory[2]);

		double sy = std::sqrt(rotation[i](0, 0) * rotation[i](0, 0) + rotation[i](1, 0) * rotation[i](1, 0));
		eulerAngle1[0] = std::atan2(rotation[i](2, 1), rotation[i](2, 2));
		eulerAngle1[1] = std::atan2(-rotation[i](2, 0), sy);
		eulerAngle1[2] = std::atan2(rotation[i](1, 0), rotation[i](0, 0));
		tempalte.push_back(eulerAngle1[0] * archer2angle);
		tempalte.push_back(eulerAngle1[1] * archer2angle);
		tempalte.push_back(eulerAngle1[2] * archer2angle);
		carve_data.push_back(tempalte);
	}

	carve_data.insert(carve_data.begin(), std::vector<double>{carve_data[0][0], carve_data[0][1], carve_data[0][2] + 200, carve_data[0][3], carve_data[0][4], carve_data[0][5]});
	pcl::PointCloud < pcl::PointXYZRGB>::Ptr curve_ON_target(new pcl::PointCloud < pcl::PointXYZRGB>);
	pcl::io::savePLYFileASCII("sffsdf.ply", *curve_tranout);
	for (auto& point_i : *curve_tranout)
	{
		curve_ON_target->points.push_back(pcl::PointXYZRGB(point_i.x, point_i.y, point_i.z, 0, 255, 0));
	}
	AddPointView("tranpath", curve_ON_target, 3);
	for (auto& point_i : *camera_data)
	{
		curve_ON_target->points.push_back(pcl::PointXYZRGB(point_i.x, point_i.y, point_i.z, 0, 0, 255));
	}
	//AddPointView("path_on_target", curve_ON_target, 3);
}


void MainWindow::SendData()
{
	QString ss;
	//****************************初始化**********************
	if (m_client->IsOnline() == false)
	{
		ui->debug_txt->append("通讯未连接，开始连接机械臂");
		try
		{
			emit ClientConnect(ui->arm_IP_txt->text(), ui->arm_Port_txt->text().toInt());
		}
		catch (const std::exception&)
		{
			QMessageBox::warning(this, "Warning", "IP或Port输入错误!!!!");
		}
	}
	else
	{
		ui->debug_txt->append("通讯已连接，开始发送数据");
	}
	emit ClientSending(carve_data, speedSet);
}

