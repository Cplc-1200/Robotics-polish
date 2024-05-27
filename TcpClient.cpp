#include "TcpClient.h"

using namespace std;
TcpClient::TcpClient(QObject* parent)
	: QTcpSocket(parent)
{
	//自动连接在信号发射时被识别为队列连接/信号在主线程发射
	connect(this, &TcpClient::SignalConnectToHost, this, [&](const QString& address, quint16 port) {
		//test record# in child thread id 20588
		qDebug("SlotConnectToHost ThreadID:%d", QThread::currentThreadId());
		//
		this->connectToHost(QHostAddress(address), port, QIODevice::ReadWrite);
		QThread::msleep(500);
		}, Qt::AutoConnection);

	//连接了TCP服务端
	QObject::connect(this, &QAbstractSocket::connected, this, [&]() {
		//test record# in child thread id 20588
		qDebug("SlotHasConnected ThreadID:%d", QThread::currentThreadId());
		//
		m_bOnLine = true;
		}, Qt::DirectConnection);

	//断开了TCP服务端
	QObject::connect(this, &QAbstractSocket::disconnected, this, [&]() {
		//test record# in child thread id 20588
		qDebug("SlotHasDisconnected ThreadID:%d", QThread::currentThreadId());
		//
		m_bOnLine = false;
		}, Qt::DirectConnection);

	//收到了TCP服务的数据
	QObject::connect(this, &QIODevice::readyRead, this, [&]() {
		//test record# in child thread id 20588
		qDebug("SlotIODeviceReadyRead ThreadID:%d", QThread::currentThreadId());

		}, Qt::DirectConnection);

	//执行数据发送过程
	QObject::connect(this, &TcpClient::SignalSendingData, this, [&](const std::vector<std::vector<double>> carve_data, double speedSet) {
		//test record# in child thread id 20588
		//qDebug("SlotSendingData ThreadID:%d", QThread::currentThreadId());
		//
		IsCom = true;
		QString ss, ss_txt;
		//清除报警
		ss = "@Y2&";
		ss_txt.append(ss);
		this->write(ss.toUtf8());
		Receive_data();
		//1. 按下急停并解除急停后，控制器会处于报警状态，这时发送清除报警后示教盒报警状态消除反馈“@Acode = 1010 & ”
		//2.当急停被触发时候，发送清除报警的命令后返回急停状态清楚无效反馈“@Acode = 1033 &
		//使能
		ss = NULL;
		ss = "@Y1&";
		ss_txt.append(ss);
		this->write(ss.toUtf8());
		Receive_data();
		//1.在已上伺服的状态下发送后返回“@Acode = 1006 &”；
		//2.在未上伺服的状态下发送后返回“@Acode = 1008 &”
		//停止命令
		//设置工具值
		//修改运动速度其中的50为运动速度   速度取值范围 1 - 100
		try
		{
			ss = NULL;
			ss = tr("@Z4speed=%1&").arg(speedSet);
			ss_txt.append(ss);
			this->write(ss.toUtf8());
			Receive_data();
		}
		catch (const std::exception&)
		{
		}
		//1.设置成功后反馈1020
		//2.输入小于1大于等于101的值会回复1021参数数据值错误

		//m_thread->msleep(500);

		/*double[] wait1 = pointWait;
		//机器人点到点运动
		ss = "@MOTIONSTARTWOSFN" + wait1[0].ToString() + "," + wait1[1].ToString() + "," + wait1[2].ToString()
		+ "," + wait1[3].ToString() + "," + wait1[4].ToString() + "," + wait1[5].ToString() + ",2,200&";
		Send(ss);
		Received(socketSend);
		ss = "@WDO12=0&";
		Send(ss);
		//@Acode1032缺少&字符  @Acode1011模式错误  @Acode 1083参数错误  @Acode 1001机器人正在执行  @Acode 1002运动程序执行出现错误  @Acode 1003运动程序编译出现错误      @Acode 1084运动到指定点成功
		Received(socketSend);*/
		//加载文件
		ss = NULL;
		ss = "@Lspline&";
		ss_txt.append(ss);
		this->write(ss.toUtf8());
		Receive_data();
		//1、程序加载成功反馈“@Acode = 1029 & ”
		//2、程序加载失败反馈“@Acode = 1030 & ”
		/**************ML指令点到点直线运动***************/
		/*for (int i = 0; i < dataX.Length; i++)
		{
			//写入全局点信息， *1代表全局点编号 * 2代表要写入的部分
			ss = "@WRITEGP" + i + ",X=" + dataX[i] + ",Y=" + dataY[i] + ",Z=" + dataZ[i] + ",A=" + AngleX[i] + ",B=" + AngleY[i] + ",C=" + AngleZ[i] + "&";
			Send(ss);
			Received(socketSend);
		}*/
		/**************spline指令运动***************/
		ss = tr("@WRITEGP0,X=%1,Y=%2,Z=%3,A=%4,B=%5,C=%6&").arg(carve_data[0][0]).arg(carve_data[0][1]).arg(carve_data[0][2]).arg(carve_data[0][3]).arg(carve_data[0][4]).arg(carve_data[0][5]);
		ss_txt.append(ss);
		//ss = "@WRITEGP0," + "X=" + carve_data[0][0] + ",Y=" + carve_data[0][1] + ",Z=" + carve_data[0][2] + ",A=" + carve_data[0][3] + ",B=" + carve_data[0][4] + ",C=" + carve_data[0][5] + "&";
		this->write(ss.toUtf8());
		Receive_data();

		//设置样条曲线速度
		if (speedSet != 0)
		{
			ss = QString("@SETTRAVEL%1&").arg(speedSet);
		}
		else
		{
			ss = "@SETTRAVEL10.0&";
		}
		ss_txt.append(ss);
		this->write(ss.toUtf8());
		Receive_data();

		ss = NULL;

		//for (int i = 0; i < carve_data.size(); i++)
		//{
		//	if (i == 0)
		//	{
		//		QString	ss0 = NULL;
		//		ss0.append("\r\n");
		//		ss0.append(tr("@VISIONPOINT; %1,%2,%3,%4,%5,%6,%7;").arg(i).arg(carve_data[i][0]).arg(carve_data[i][1]).arg(carve_data[i][2]).arg(carve_data[i][3]).arg(carve_data[i][4]).arg(carve_data[i][5]));
		//		ss_txt.append(ss0);
		//		this->write(ss0.toUtf8());
		//		Receive_data();
		//	}
		//	else if (i == carve_data.size() - 1)
		//	{
		//		QString	ssn = NULL;
		//		ssn.append("\r\n");
		//		ssn.append(tr("%1,%2,%3,%4,%5,%6,%7&").arg(i).arg(carve_data[i][0]).arg(carve_data[i][1]).arg(carve_data[i][2]).arg(carve_data[i][3]).arg(carve_data[i][4]).arg(carve_data[i][5]));
		//		ss_txt.append(ssn);
		//		this->write(ssn.toUtf8());
		//		Receive_data();
		//		ss = NULL;
		//		ss = "@MOTIONSTART&";
		//		ss_txt.append(ss);

		//		this->write(ss.toUtf8());
		//		Receive_data();
		//		ss = NULL;
		//	}
		//	else
		//	{
		//		QString ssi = NULL;
		//		/*ssi.append("\r\n");
		//		ssi.append(tr("%1,%2,%3,%4,%5,%6,%7;").arg(i).arg(carve_data[i][0]).arg(carve_data[i][1]).arg(carve_data[i][2]).arg(carve_data[i][3]).arg(carve_data[i][4]).arg(carve_data[i][5]));
		//		ss_txt.append(ssi);*/
		//		for (size_t j = 0; j < 300; j++)
		//		{
		//			ssi.append("\r\n");
		//			ssi.append(tr("%1,%2,%3,%4,%5,%6,%7;").arg(i).arg(carve_data[i][0]).arg(carve_data[i][1]).arg(carve_data[i][2]).arg(carve_data[i][3]).arg(carve_data[i][4]).arg(carve_data[i][5]));
		//			if (i == carve_data.size() - 2);
		//			{
		//				break;
		//			}
		//			i++;
		//		}
		//		ss_txt.append(ssi);
		//		this->write(ssi.toUtf8());
		//		Receive_data();
		//		//i--;
		//	}
		//}
		QString	ss0 = NULL;
		for (int i = 0; i < carve_data.size(); i++)
		{
			if (i == 0)
			{
				ss0.append("\r\n");
				ss0.append(tr("@VISIONPOINT; %1,%2,%3,%4,%5,%6,%7;").arg(i).arg(carve_data[i][0]).arg(carve_data[i][1]).arg(carve_data[i][2]).arg(carve_data[i][3]).arg(carve_data[i][4]).arg(carve_data[i][5]));
			}
			else if (i == 400)
			{
				ss0.append("\r\n");
				ss0.append(tr("%1,%2,%3,%4,%5,%6,%7&").arg(i).arg(carve_data[i][0]).arg(carve_data[i][1]).arg(carve_data[i][2]).arg(carve_data[i][3]).arg(carve_data[i][4]).arg(carve_data[i][5]));
				ss_txt.append(ss0);
				this->write(ss0.toUtf8());
				Receive_data();
				ss = NULL;
				ss = "@MOTIONSTART&";
				ss_txt.append(ss);

				this->write(ss.toUtf8());
				Receive_data();
				ss = NULL;
				break;
			}
			else
			{
				ss0.append("\r\n");
				ss0.append(tr("%1,%2,%3,%4,%5,%6,%7;").arg(i).arg(carve_data[i][0]).arg(carve_data[i][1]).arg(carve_data[i][2]).arg(carve_data[i][3]).arg(carve_data[i][4]).arg(carve_data[i][5]));
			}
		}
		ss = "@Z1&";
		ss_txt.append(ss);
		this->write(ss.toUtf8());
		Receive_data();
		if (remove("model_file\\arm_run_data.txt") == 0)
		{
			qDebug() << "删除文件成功";
		}
		else
		{
			qDebug() << "删除文件失败";
		}
		ofstream ofs;
		ofs.open("model_file/arm_run_data.txt", std::ios::out);
		ofs << ss_txt.toStdString() << std::endl;
		ofs.close();
		IsCom = false;
		//QThread::msleep(0.1);
		}, Qt::AutoConnection);
	//执行数据发送过程
	QObject::connect(this, &TcpClient::Single_SignalSendingData, this, [&](const QString c_btaData) {
		//test record# in child thread id 20588
		//qDebug("SlotSendingData ThreadID:%d", QThread::currentThreadId());
		IsCom = true;
		this->write(c_btaData.toUtf8());
		Receive_data();
		IsCom = false;
		}, Qt::AutoConnection);
}

void TcpClient::Receive_data()
{
	if (this->waitForReadyRead())
	{
		//读取全部数据
		m_btaReceiveFromService.clear();
		m_btaReceiveFromService.append(this->readAll());
		QString str = m_btaReceiveFromService;
		int len = m_btaReceiveFromService.size();
		if (len == 0)
		{
			return;
		}
		//Console.WriteLine("是GetV");
		//if (isWork)
		//{ data_handle(num);
		//if(myDisplayMsgs!=null)
		//myDisplayMsgs("是GetV，数据已发送");
		//}
		//else
		//{
		//emit SignalPublishFormatRecvData("程序未启动，无法发送点位数据,请先启动程序");
		//myDisplayMsg("程序未启动，无法发送点位数据,请先启动程序");
		//}
		//num += 1;
		if (str == "@Acode=1074&")
		{
			emit SignalPublishFormatRecvData("视觉点位接受正常");
		}
		else if (str == "@Acode=1071&")
		{
			emit SignalPublishFormatRecvData("设置样条曲线速度成功");
		}
		else if (str == "@Acode=1020&")
		{
			emit SignalPublishFormatRecvData("设置全局速度成功");
		}
		else if (str == "@Acode=1044&")
		{
			emit SignalPublishFormatRecvData("设置IO状态成功");
		}
		else if (str == "@Acode=1084&")
		{
			emit SignalPublishFormatRecvData("运动到指定点成功");
		}
		else if (str == "@Acode=1086&")
		{
			emit SignalPublishFormatRecvData("设置百分比正确");
		}
		else if (str == "@Acode=1001&")
		{
			emit SignalPublishFormatRecvData("机器人成功执行该指令");
		}
		else if (str == "@Acode=1006&")
		{
			emit SignalPublishFormatRecvData("已经上使能");
		}
		else if (str == "@Acode=1008&")
		{
			emit SignalPublishFormatRecvData("上使能");
		}
		else if (str == "@Acode=1011&")
		{
			emit SignalPublishFormatRecvData("处于示教模式");
		}
		else if (str == "@Acode=1023&")
		{
			emit SignalPublishFormatRecvData("设置VA变量成功");
		}
		else if (str == "@Acode=1053&")
		{
			emit SignalPublishFormatRecvData("设置GP变量成功");
		}
		else if (str == "@Acode=1076&")
		{
			emit SignalPublishFormatRecvData("机器人停止成功");
		}
		else if (str == "@Acode=1010&")
		{
			emit SignalPublishFormatRecvData("清除警报");
		}
		else if (str == "@Acode=1079&")
		{
			emit SignalPublishFormatRecvData("设置工具号正确");
		}
		else if (str == "@Acode=1029&")
		{
			emit SignalPublishFormatRecvData("加载文件成功");
		}
		else if (str == "@Acode=1075&")
		{
			emit SignalPublishFormatRecvData("机器人启动，走视觉点位");
		}
		else if (str == "@Acode=1014&")
		{
			emit SignalPublishFormatRecvData("启动示教程序");
		}
		else
		{
			emit SignalPublishFormatRecvData(str);
		}
	}
}


//
TcpClient::~TcpClient()
{
}

//跨线程转换
void TcpClient::ClientConnectToHost(const QString& address, quint16 port)
{
	emit SignalConnectToHost(address, port);
}

//跨线程转换
void TcpClient::ClientSendingData(const std::vector<std::vector<double>> carve_data, double speedset)
{
	emit SignalSendingData(carve_data, speedset);
}
//跨线程转换
void TcpClient::Single_ClientSendingData(const QString c_btaData)
{
	emit Single_SignalSendingData(c_btaData);
}


//是否在线
bool TcpClient::IsOnline()
{
	return m_bOnLine;
}

