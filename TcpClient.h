#pragma once
#include <QThread>
#include <QDebug>
#include <QHostAddress>
#include <QTcpSocket>
#include <fstream>
class TcpClient : public QTcpSocket
{
    Q_OBJECT

public:
    TcpClient(QObject* parent = NULL);
    ~TcpClient();
public:
    //
    void ClientConnectToHost(const QString& address, quint16 port);
    //
    void ClientSendingData(const std::vector<std::vector<double>> carve_data,const double speedset);
    void Single_ClientSendingData(const QString c_btaData);
    //
    bool IsOnline();
    bool IsCom=false;
signals:
    //转换来自主线程的链接操作
    void SignalConnectToHost(const QString& address, quint16 port);
signals:
    //转换来自主线程的发送操作
    void SignalSendingData(const std::vector<std::vector<double>>& carve_data,const double& speedset);
signals:
    //转换来自主线程的发送操作
    void Single_SignalSendingData(const QString c_btaData);
signals:
    //在次线程中缓冲并滑动解析TCP流后/按约定格式再发布
    void SignalPublishFormatRecvData(const QString c_btaData);

private:
    void Receive_data();
    //标记连接情况
    bool m_bOnLine = false;
    //缓冲收到的流数据
    QByteArray m_btaReceiveFromService;
  
};

