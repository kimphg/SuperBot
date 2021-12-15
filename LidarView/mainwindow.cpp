#include "mainwindow.h"
#include "ui_mainwindow.h"
#define BUFF_SIZE 1000
#include <QPainter>
#include <QSerialPort>
QSerialPort *serial;
char serialDataBuff[BUFF_SIZE];
int buffIndex = -1;
float mapData[90*15];
float angleScale=1;
QPixmap *renderMap;
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    serial = new QSerialPort();
    serial->setPortName("COM4");
    serial->setBaudRate(480600);
    serial->open(QIODevice::ReadWrite);

    renderMap = new QPixmap(200,200);
    renderMap->fill(Qt::black);
    connect(serial, SIGNAL(readyRead()), this, SLOT(serialData()));
    startTimer(100);
}
void MainWindow::timerEvent(QTimerEvent *event)
{
    repaint();
}
void MainWindow::serialData()
{
    while (serial->bytesAvailable()) {

        char inbyte;
        serial->read(&inbyte,1);
        buffIndex++;
        if(buffIndex>=BUFF_SIZE)buffIndex=0;
        if(inbyte=='$')
        {

            QByteArray dataFrame(serialDataBuff,buffIndex);
            processFrame(dataFrame);
            buffIndex=0;
        }
        serialDataBuff[buffIndex]=inbyte;


    }
}
void MainWindow::processFrame(QByteArray data)
{
    QByteArrayList datalist = data.split(' ');
    if(datalist.size()<17)
        return;
    float angle = QString(datalist[1]).toInt();

    QPainter p(renderMap);
    p.setPen(QPen(QColor(255,0,0,255)));
    for(int miniangle = 0;miniangle<16;miniangle++)
    {
        int realAngle = angle*15+miniangle;//(angle+miniangle/15.0)/90.0*3.1415926535*2.0;

        mapData[realAngle] = QString(datalist[2+miniangle]).toFloat();


//        float range = ;
//        for(int drawRange=0;drawRange<20000;drawRange+=20)
//        {
//            int x = realAngle;//100+ range*5*cos(realAngle);
//            int y = drawRange/20.0;//100+ range*5*sin(realAngle);
//            p.setPen(QPen(QColor(0,0,0,255)));
//            p.drawPoint(x,y);
//        }
//        int x = realAngle;//100+ range*5*cos(realAngle);
//        int y = range/40.0;//100+ range*5*sin(realAngle);
//        p.setPen(QPen(QColor(255,0,0,150)));
//        p.drawPoint(x,y);

    }
//    repaint();

}
void MainWindow::paintEvent(QPaintEvent *event)
{
    QPainter p(this);
    p.fillRect(this->rect(),Qt::black);
    p.setPen(QPen(QColor(255,0,0,150)));
    for(int az=0;az<90*15;az++)
    {
        float realAz = -az*3.1415926535*2.0/(90.0*15.0);
        float range = mapData[az]/20.0;
        int x = 500+ range*5*cos(realAz);
        int y = 500+ range*5*sin(realAz);
        p.drawPoint(x,y);
    }
//    p.drawPixmap(this->rect(),*renderMap,renderMap->rect());
//    angleScale = ui->textEdit->toPlainText().toFloat();
}
MainWindow::~MainWindow()
{
    delete ui;
}
