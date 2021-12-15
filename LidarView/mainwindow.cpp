#include "mainwindow.h"
#include "ui_mainwindow.h"
#define BUFF_SIZE 1000
#include <QPainter>
#include <QSerialPort>
QSerialPort *serial;
char serialDataBuff[BUFF_SIZE];
int buffIndex = -1;
float mapData[1000][1000];
float angleScale=1;
QPixmap *renderMap;
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    serial = new QSerialPort();
    serial->setPortName("COM8");
    serial->setBaudRate(480600);
    serial->open(QIODevice::ReadWrite);

    renderMap = new QPixmap(200,200);
    renderMap->fill(Qt::black);
    connect(serial, SIGNAL(readyRead()), this, SLOT(serialData()));
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
    for(int miniangle = 0;miniangle<15;miniangle++)
    {
        float realAngle = (angle+miniangle/15.0)/90.0*3.1415926535*2.0;

        float range = QString(datalist[2+miniangle]).toInt();

//        float brigtness = amp/2000.0;
//        if(brigtness>1)brigtness=1;
        int x = realAngle*50;//100+ range*5*cos(realAngle);
        int y = range/5.0;//100+ range*5*sin(realAngle);
        p.setBrush(QBrush(QColor(255,0,0,255)));
//        p.setBrush();
        p.drawEllipse(x,y,4,4);

    }
    repaint();

}
void MainWindow::paintEvent(QPaintEvent *event)
{
    QPainter p(this);
    p.drawPixmap(this->rect(),*renderMap,renderMap->rect());
//    angleScale = ui->textEdit->toPlainText().toFloat();
}
MainWindow::~MainWindow()
{
    delete ui;
}
