#include "mainwindow.h"
#include "ui_mainwindow.h"
#define BUFF_SIZE 1000
#include <QPainter>
#include <QSerialPort>
#include <QTimer>
#include <QWheelEvent>
QSerialPort *serial;
unsigned char serialDataBuff[BUFF_SIZE];
int buffIndex = -1;
float mapData[90*15];
float angleScale=1;
QPixmap *renderMap;
QTimer *timer1s;
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    serial = new QSerialPort();
    serial->setPortName("COM4");
    serial->setBaudRate(460800);
    serial->open(QIODevice::ReadWrite);

//    renderMap = new QPixmap(200,200);
//    renderMap->fill(Qt::black);
    connect(serial, SIGNAL(readyRead()), this, SLOT(serialData()));
    startTimer(30);
    this->resize(1400,1000);
    timer1s = new QTimer(this);
    connect(timer1s, SIGNAL(timeout()), this, SLOT(callback1s()));
    timer1s->start(1000);

}

void MainWindow::callback1s()
{
    fps = frameCount;
    frameCount=0;
    bps = byteCount;
    byteCount=0;
}
void MainWindow::timerEvent(QTimerEvent *event)
{
    repaint();
}
unsigned char oldbyte;
void MainWindow::serialData()
{
    while (serial->bytesAvailable()) {

        unsigned char inbyte;
        serial->read(( char *)(&inbyte),1);
        byteCount++;

        if(inbyte==0xaa&&oldbyte==0xff)
        {

//            QByteArray dataFrame(serialDataBuff,buffIndex);
            processFrameHex(serialDataBuff);
            buffIndex=0;
        }else
        {
            serialDataBuff[buffIndex]=inbyte;
            buffIndex++;
            if(buffIndex>=BUFF_SIZE)buffIndex=0;
        }
        oldbyte =inbyte;

    }
}
int ministep=16;
void MainWindow::processFrame(QByteArray data)
{
    QByteArrayList datalist = data.split(' ');
    if(datalist.size()<17)
        return;
    float angle = abs(QString(datalist[0]).toInt());
    if(angle<0||angle>90)return;
    for(int miniangle = 0;miniangle<16;miniangle++)
    {
        int realAngle = angle*ministep+miniangle;
        float range  = QString(datalist[1+miniangle]).toFloat();
//        if(range<1) {printf("\n%s %d",data.data(),miniangle); break;}
        mapData[realAngle] = range;
    }
}
void MainWindow::processFrameHex(unsigned char* data)
{
    float angle = data[0]-160;
    if(angle<0||angle>90)return;
    for(int miniangle = 0;miniangle<16;miniangle++)
    {
        int realAngle = angle*ministep+miniangle;
        float range  = data[3+miniangle*2] +data[4+miniangle*2]*256 ;
        mapData[realAngle] = range;
    }
    frameCount++;
}
void MainWindow::wheelEvent(QWheelEvent *event)
{
    if(event->delta()>0)mScale*=1.2;
    else mScale/=1.2;
}
void MainWindow::paintEvent(QPaintEvent *event)
{

    int ctx=width()/2;
    int cty=height()/2;
    QPainter p(this);
    p.fillRect(this->rect(),Qt::black);
    p.setPen(QPen(QColor(255,0,0,150),2));
    int x = 0,y=0;
    for(int az=0;az<90*ministep;az++)
    {
        float realAz = -az*3.1415926535*2.0/(90.0*ministep);
        float range = mapData[az]/mScale;

        if (range<1){ continue;}
        int x1 = x;
        int y1 = y;
        x = ctx + range*cos(realAz);
        y = cty + range*sin(realAz);
        float ed = sqrt((x-x1)*(x-x1)+(y-y1)*(y-y1));
        if(ed<50){if(x1!=0)p.drawLine(x,y,x1,y1);}
        else p.drawPoint(x,y);
    }

    p.drawText(0,height(),"fps:"+QString::number(fps)+"bps:"+QString::number(bps));
    p.setPen(QPen(QColor(0,255,0,150),1));
    int pixStep = 1000.0/mScale;
    for(float r=pixStep;r<10*pixStep;r+=pixStep)
    {
        p.drawEllipse(ctx-r,cty-r,2*r,2*r);//one meter circle
        p.drawText(ctx+3+r,cty,QString::number(r/pixStep)+"meter");
    }

}
MainWindow::~MainWindow()
{
    serial->close();
    delete ui;
}
