#include "mainwindow.h"
#include "ui_mainwindow.h"
#define BUFF_SIZE 1000
#include <QPainter>
#include <QSerialPort>
#include <QTimer>
#include <QWheelEvent>
QSerialPort *serial;
unsigned char serialDataBuff[BUFF_SIZE];
int buffIndex = 0;
float mapData[90*16];
float warning[90*16];
float angleScale=1;
QPixmap *renderMap;
QTimer *timer1s;
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    serial = new QSerialPort();
    serial->setPortName("COM42");
    serial->setBaudRate(115200);
    serial->open(QIODevice::ReadWrite);

    //    renderMap = new QPixmap(200,200);
    //    renderMap->fill(Qt::black);
    connect(serial, SIGNAL(readyRead()), this, SLOT(serialData()));
    startTimer(10);
    this->resize(1400,1000);
    timer1s = new QTimer(this);
    connect(timer1s, SIGNAL(timeout()), this, SLOT(callback1s()));
    timer1s->start(100);

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
unsigned char oldbyte,oldbyte1;
void MainWindow::serialData()
{
    if(1)// lidar mode
    {
        while (serial->bytesAvailable()) {///

            unsigned char inbyte;
            serial->read(( char *)(&inbyte),1);
            byteCount++;

            if(inbyte=='\n'&&oldbyte=='\r')
            {

                //            QByteArray dataFrame(serialDataBuff,buffIndex);
                processFrame(QByteArray((const char*)serialDataBuff,buffIndex+1));
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
    else
    {
        while (serial->bytesAvailable()) {
            unsigned char inbyte;
            serial->read(( char *)(&inbyte),1);
            byteCount++;

            if(inbyte==0xf0&&oldbyte==0xfa&&oldbyte1==0xff)
            {

                //            QByteArray dataFrame(serialDataBuff,buffIndex);
                processFrameTOF(serialDataBuff);
                buffIndex=0;
            }else
            {
                serialDataBuff[buffIndex]=inbyte;
                printf(" %d",inbyte);
                buffIndex++;
                if(buffIndex>=BUFF_SIZE)buffIndex=0;
            }
            oldbyte1=oldbyte;
            oldbyte =inbyte;


        }

    }
}
int ministep=16;
void MainWindow::processFrame(QByteArray data)
{
    QByteArrayList datalist = data.split(',');
    if(datalist.size()<3)
        return;
    float angle = (QString(datalist[0]).toFloat());
    int warnLevel = (QString(datalist[2]).toInt());
    printf("%f,\n",angle);
    if(angle<-180||angle>180)return;
    int miniangle = angle*4;
    if(miniangle<0)miniangle+=1440;
    float range  = QString(datalist[1]).toFloat();
    mapData[miniangle] = range;
    warning[miniangle] = warnLevel;
//    printf("%s,\n",datalist[0].data());
//    for(int miniangle = 0;miniangle<16;miniangle++)
//    {
//        int realAngle = angle*ministep+miniangle;
//        float range  = QString(datalist[1+miniangle]).toFloat();
//        mapData[realAngle] = range;
//    }
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
void MainWindow::processFrameTOF(unsigned char* data)
{
    memcpy((char*)tofDataOld,(char*)tofData,16*2);
    memcpy((char*)tofData,data,16*2);
    for (int i=0;i<16;i++)
    {
        tofDataDiff[i] = tofData[i]-tofDataOld[i];
    }
    newDataPending=true;
    repaint();

}
void MainWindow::wheelEvent(QWheelEvent *event)
{
    if(event->delta()>0)mScale*=1.2;
    else mScale/=1.2;
}
void MainWindow::paintEvent(QPaintEvent *event)
{

    if(1){
        int ctx=width()/2;
        int cty=2*height()/3;
        QPainter p(this);
        p.fillRect(this->rect(),Qt::black);
        p.setPen(QPen(QColor(255,0,0,150),2));
        int x = 0,y=0;
        for(int az=0;az<1440;az++)
        {

//            if(az==1440)az=0;

            float realAz = az*180*2.0/(1440);
            if(realAz<-180)realAz+=(360);
            if(realAz>180)realAz-=(360);

//            if((realAz<-70)||(realAz>70))continue;
            float range = mapData[az];
            int warnLevel = warning[az];
//            printf("%d,%f\n",az,range);
            if (range<1){ continue;}
            float xmm = range*sin(realAz/57.2957795);
            float ymm = range*cos(realAz/57.2957795);
//            if(abs(xmm)<400)
//            {
//                if(ymm<300)
//                    p.setPen(QPen(QColor(255,0,0,255),1));
//                else
//                    p.setPen(QPen(QColor(255,150,0,255),1));
//            }
//            else if(abs(xmm)<1000)
//            {
//                if(ymm<600)
//                    p.setPen(QPen(QColor(255,150,0,255),1));
//                else
//                    p.setPen(QPen(QColor(0,255,0,255),1));

//            }
//            else p.setPen(QPen(QColor(0,255,0,255),1));
            if(warnLevel == 3)p.setPen(QPen(QColor(255,0,0,255),1));
            else if(warnLevel == 2)p.setPen(QPen(QColor(255,150,0,255),1));
            else if(warnLevel == 1)p.setPen(QPen(QColor(0,150,0,255),1));
            else p.setPen(QPen(QColor(0,150,255,255),1));
            int x1 = x;
            int y1 = y;
            x = ctx + xmm/mScale;
            y = cty - ymm/mScale;;
            float ed = sqrt((x-x1)*(x-x1)+(y-y1)*(y-y1));
            if(ed<50){if(x1!=0)p.drawLine(x,y,x1,y1);}
            else p.drawPoint(x,y);
        }
        p.setPen(QPen(QColor(0,255,0,150),1));
        p.drawRect(ctx-300/mScale,cty-150/mScale,600/mScale,600/mScale);
        p.drawText(0,height(),"fps:"+QString::number(fps)+"bps:"+QString::number(bps));

        int pixStep = 1000.0/mScale;
        for(float r=pixStep;r<10*pixStep;r+=pixStep)
        {
            p.drawEllipse(ctx-r,cty-r,2*r,2*r);//one meter circle
            p.drawText(ctx+3+r,cty,QString::number(r/pixStep)+"meter");
        }
    }
    else if(newDataPending)
    {
        int resolution=4;
        newDataPending=false;
        QPainter p(this);
        int cellSize = 50;
        p.setPen(Qt::black);
        QRect rect(0,0,cellSize,cellSize);
        for (int i=0;i<resolution;i++)
            for(int j=0;j<resolution;j++)
            {
                rect.setRect(i*cellSize,j*cellSize,cellSize,cellSize);
                unsigned short value = tofData[i*resolution+j];
                p.drawText(rect,QString::number(value));
                rect.setRect(i*cellSize+500,j*cellSize,cellSize,cellSize);
                value/=4;
                if(value>255)value=255;
                p.fillRect(rect,QColor(value,100,255-value));
            }

    }

}
MainWindow::~MainWindow()
{
    serial->close();
    delete ui;
}
