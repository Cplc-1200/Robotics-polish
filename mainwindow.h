#pragma execution_character_set("utf-8")
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <vector>
#include "pcl_function.h"
#include "get_cameradata.h"
#include "halcon_PPF.h"
#include "TcpClient.h"
//----filter
#include "QVTKOpenGLNativeWidget.h"
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkNamedColors.h>
#include <vtkProperty.h>
#include <vtkSmartPointer.h>
#include "vtkAutoInit.h"
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingVolumeOpenGL2);
VTK_MODULE_INIT(vtkRenderingFreeType);
//-----------------------------vtk---------------------------------

/**********************************qt*****************************/
#include <QMainWindow>
#include <QDebug>
#include <QColorDialog>
#include <QMessageBox>
#include <QFileDialog>
#include <QTime>
#include <QDir>
#include <QFile>
#include <QtMath>
#include <QDirIterator>
#include <QFuture>
#include <QFutureWatcher>
#include <QtConcurrent/QtConcurrentRun>
#include <QMap>
#include <QIcon>
#include <QStandardItemModel>
#include <QTcpSocket>
#include <QThread>

//创建菜单栏，工具栏，状态栏应当包含的头文件
#include <QMenuBar>
#include <QMenu>
#include <QToolBar>
#include <QStatusBar>
#include <QAbstractAnimation>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

#define angle2archer 0.0174533
#define archer2angle 57.29578

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	explicit MainWindow(QWidget* parent = 0);
	~MainWindow();
	// 负责通信的套接字
	TcpClient* m_client;
	QThread* m_thread;
	double motionspeed=10;
	double speedSet = 10;
	//--------------------------------------slot funaction----------------------------------------------
private slots:
	/*********************************************基本操作***********************************************/
	void Open_clicked();//读取点云

	void Save_clicked();//保存点云

	//void on_treeView_clicked(const QModelIndex& index);//点云显示槽函数

	void point_cloud_process(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

	void run_all();

signals:
	void ClientConnect(const QString& address, quint16 port);
signals:
	void ClientSending(const std::vector<std::vector<double>>& crave0, const double& speed0);
private:
	Ui::MainWindow* ui;

	QMap<QString, QIcon> m_publicIconMap;///< 存放公共图标

	QStandardItemModel* model;

	// QStandardItem* item;

	QModelIndex index_cloud;

	std::vector<int> cloud_index;
	int point_size = 1;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

	std::vector<std::vector<double>> carve_data;
	std::vector<Eigen::MatrixXd> rotation;

	std::map<std::string, std::variant<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointXYZRGB>, pcl::PolygonMesh>> string_Pcloud;

	void view_updata(std::string cloud_name);
	void view_deledata(std::string cloud_name);

	void pcloud_match(pcl::PointCloud<pcl::PointXYZ>::Ptr& camera_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_src, Eigen::Matrix4f& transformation_matrix1);
	void SendData();
	template<typename Tyn>
	void AddPointView(const QString& baseName, const Tyn& cloud, const int& row_indice);
};
#endif // MAINWINDOW_H
