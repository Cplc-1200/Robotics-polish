#include "get_cameradata.h"
#include <mutex>


class TikTokTimer {
public:
	void tik() {
		start_time_ = std::chrono::high_resolution_clock::now();
	}
	void tik(std::string info) {
		if (!info.empty())
			qDebug() << QString::fromStdString(info);
		start_time_ = std::chrono::high_resolution_clock::now();
	}
	void tok(std::string info) {
		auto end = std::chrono::high_resolution_clock::now();
		auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start_time_);
		qDebug() << QString::fromStdString(info) << ": " << elapsed.count() << " ms.";
	}

	std::chrono::time_point<std::chrono::high_resolution_clock,
		std::chrono::duration<long int, std::ratio<1, 1000000000> > >
		start_time_;
};

//最好放到一个单独的线程里操作
std::mutex mtx;
 void Get_CameraData(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	qDebug() << "SDKVersion:" << QString::fromStdString(TFTech::EpicEye::getSDKVersion());

	std::string ip="192.168.110.244";
	TikTokTimer timer;

	//// search camera
	//qDebug() << "---------------search camera---------------";
	//std::vector<TFTech::EpicEyeInfo> cameraList;
	//if (TFTech::EpicEye::searchCamera(cameraList)) {
	//	qDebug() << "Camera found: " << cameraList.size();
	//	for (int i = 0; i < cameraList.size(); i++) {
	//		qDebug() << i << ": " << QString::fromStdString(cameraList[i].IP) << " " << QString::fromStdString(cameraList[i].SN);
	//	}
	//	ip = cameraList[0].IP;
	//	qDebug() << "using IP: " << QString::fromStdString(ip);
	//}
	//else {
	//	ip = "127.0.0.1";
	//	qDebug() << "Camera not found!";
	//	qDebug() << "using fallback ip: " << QString::fromStdString(ip);
	//}

	// get camera info
	qDebug() << "---------------EpicEyeInfo---------------";
	timer.tik();
	TFTech::EpicEyeInfo info;
	if (!TFTech::EpicEye::getInfo(ip, info)) {
		qDebug() << "getInfo failed!";
	}
	qDebug() << "SN: " << QString::fromStdString(info.SN);
	qDebug() << "IP: " << QString::fromStdString(info.IP);
	qDebug() << "model: " << QString::fromStdString(info.model);
	qDebug() << "alias: " << QString::fromStdString(info.alias);
	qDebug() << "resolution: " << info.width << "x" << info.height;
	timer.tok("getInfo");

	//TFTech::EpicEyeConfig config = {2, 26, 30, true, 20, true, true};
	////get config
	//qDebug() << "---------------EpicEyeConfig---------------";
	//timer.tik();

	//if (!TFTech::EpicEye::getConfig(ip, config)) {
	//	qDebug() << "getConfig failed!";
	//}
	//qDebug() << "projectorBrightness: " << config.projectorBrightness;
	//qDebug() << "expTime2D: " << config.expTime2D;
	//qDebug() << "expTime3D: " << config.expTime3D;
	//qDebug() << "useHDR: " << config.useHDR;
	//qDebug() << "expTimeHDR: " << config.expTimeHDR;
	//qDebug() << "useSF: " << config.useSF;
	//qDebug() << "usePF: " << config.usePF;
	TFTech::EpicEyeConfig config = { 2, 26, 30, true, 20, true, true };
	//set config
	qDebug() << "---------------setConfig---------------";
	timer.tik();
	if (!TFTech::EpicEye::setConfig(ip, config)) {
		qDebug() << "setconfig failed!";
	}
	timer.tok("setconfig");

	//get Intrinsic
	qDebug() << "---------------Intrinsic---------------";
	timer.tik();
	std::vector<float> cameraMatrix;
	if (!TFTech::EpicEye::getCameraMatrix(ip, cameraMatrix)) {
		qDebug() << "getCameraMatrix failed!";
	}
	qDebug() << "cameraMatrix: ";
	for (const auto& element : cameraMatrix)
		qDebug() << element << " ";
	qDebug();
	std::vector<float> distortion;
	if (!TFTech::EpicEye::getDistortion(ip, distortion)) {
		qDebug() << "getDistortion failed!";
	}
	qDebug() << "distortion: ";
	for (const auto& element : distortion)
		qDebug() << element << " ";
	qDebug();
	// trigger one frame
	qDebug() << "---------------TriggerFrame---------------";
	timer.tik();
	std::string frameID;
	bool requestPointCloud = true;
	if (!TFTech::EpicEye::triggerFrame(ip, frameID, requestPointCloud)) {
		qDebug() << "triggerFrame failed!";
	}
	qDebug() << "frameID: " << QString::fromStdString(frameID);
	timer.tok("triggerFrame");

	//get point cloud
	qDebug() << "---------------get point cloud---------------";
	timer.tik();
	float* pointCloudPtr = (float*)malloc(info.width * info.height * 3 * sizeof(float));
	if (!TFTech::EpicEye::getPointCloud(ip, frameID, pointCloudPtr)) {
		qDebug() << "getPointCloud failed!";
		free(pointCloudPtr);
	}
	// 在访问 pointCloudPtr 之前加锁
	std::unique_lock<std::mutex> lock(mtx);
	try
	{
		for (size_t i = 0; i < info.height * info.width; i++)
		{
			cloud->push_back(pcl::PointXYZ(pointCloudPtr[i * 3 + 0], pointCloudPtr[i * 3 + 1], pointCloudPtr[i * 3 + 2]));
		}
	}
	catch (const std::exception&)
	{
		qDebug() << "相机拍摄失败!!!!!!!!";
	}
	//去除NAN点
	std::vector<int> indices_src; //保存去除的点的索引
	pcl::removeNaNFromPointCloud(*cloud, *cloud, indices_src);
	// 在操作完成后解锁
	lock.unlock();
	free(pointCloudPtr);
	qDebug() << "点云数量" << cloud->size();
	timer.tok("getPointCloud");
}