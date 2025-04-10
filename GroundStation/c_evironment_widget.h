#ifndef C_EVIRONMENT_WIDGET_H
#define C_EVIRONMENT_WIDGET_H
#include <QPainter>
#include <QFrame>
#include <QUdpSocket>
#include "c_drone.h"
struct RobotParam
{
  QByteArray paramName;
  float paraValue;
};

class c_evironment_widget : public QFrame
{
    Q_OBJECT
    void draw_grid(QPainter *p);
    void draw_drones(QPainter *p, c_drone drone);
public:
    QString robotIP;
        int rows,cols;
QPointF homePos;
QString dcam1,dcam2,dmcu,dfr,dmcuparam;
    QHash<QString,c_drone> mDroneList;
    QByteArray lastMsg;
    QHash<QByteArray,QByteArray> dataTable;
    std::vector<RobotParam> paramTable;
//    QHash<QByteArray,QByteArray> paramTable;
    void sendCommand(QByteArray command, bool isRec = true);
    explicit c_evironment_widget(QWidget *parent = nullptr);
    bool connectCom();
    void sendMsg(QByteArray command, int cstype=0);
    void disconnectCom();
    int frameCount = 0;
    bool newCommand = true;
    int getFPS()
    {
        int output = frameCount;
//        if(output==0)
//            connect(udpSocket,SIGNAL(readyRead()),this,SLOT(udpDataReceive()));
        frameCount=0;
        return output;
    }
        void setParam(QByteArray id, float value);

    bool getParamUpdated();

    void uploadParams(QString id, QString value);
    QStringList commands;
    bool getIsRecording() const;

    void processDebugString(QString addr);
    int getTagID() const;

    float getYawTagID() const;

    int getRobotStat();
    int getWarningLevel();
    int warningLevel;
    float getCurSpeedR() const;

    float getDesSpeedL() const;

    float getCurSpeedL() const;

    QPoint getCellPos(QPoint scrPos);
    void draw_pos_cam(QPainter *p);
private:
    float tagx =0;
    float tagy=0;
    float camAngle=0;
    QImage *roboticon;
//    float desx=0;
//    float desy=0;
    float errx = 0,erry = 0;
    int tagID = -1;
    float yawTagID = -1;
    int robotStat = 0;
    float angleError=0;
    float posError = 0;
    float liftError = 0;
    bool paramUpdated=false;
    float desSpeed=0;
    float curSpeedL = 0;
    float desSpeedRot = 0;
    float curSpeedR = 0;
    float curSpeedLift = 0;
    float desSpeedL = 0;
    int lastSyncSec=0;
    int lastSyncSeco=0;
    float desSpeedR = 0;
    float desSpeedLift = 0;
    float liftAngle = 0;
    bool isRecording = false;
    int recordCount=0;
    QUdpSocket* udpSocket;
    QByteArray debugString;
    bool isMoving =false;
    uint timeIter=0;
    float rollAngle = 0;
    QSerialPort *serial;
    float mSpeedLeft=0,mSpeedRight=0;
    float robotAngle=0;
    void readSerial();
    void sendReport();
    void processDebugLine();
    void addRecord();
    void draw_graphs(QPainter *p);
    float loadParam(QByteArray id, float defaultValue = 0);

private slots:
    void udpDataReceive();
protected slots:
    void paintEvent(QPaintEvent *event);
    void timerEvent(QTimerEvent *event);
    void mousePressEvent(QMouseEvent *event);
signals:

};

#endif // C_EVIRONMENT_WIDGET_H
