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
    //ת���������̵߳����Ӳ���
    void SignalConnectToHost(const QString& address, quint16 port);
signals:
    //ת���������̵߳ķ��Ͳ���
    void SignalSendingData(const std::vector<std::vector<double>>& carve_data,const double& speedset);
signals:
    //ת���������̵߳ķ��Ͳ���
    void Single_SignalSendingData(const QString c_btaData);
signals:
    //�ڴ��߳��л��岢��������TCP����/��Լ����ʽ�ٷ���
    void SignalPublishFormatRecvData(const QString c_btaData);

private:
    void Receive_data();
    //����������
    bool m_bOnLine = false;
    //�����յ���������
    QByteArray m_btaReceiveFromService;
  
};

