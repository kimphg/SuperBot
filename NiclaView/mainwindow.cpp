#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    videoSocket = new QUdpSocket(this);
    udpSocket = new QUdpSocket(this);
    bool isOK = udpSocket->bind(QHostAddress("192.168.1.14"),31000);
    if(!isOK)QApplication::exit();
    isOK = videoSocket->bind(QHostAddress("192.168.1.14"),31001);
    if(!isOK)QApplication::exit();
    connect(udpSocket,SIGNAL(readyRead()),this,SLOT(udpDataReceive()));
    connect(videoSocket,SIGNAL(readyRead()),this,SLOT(videoReceive()));

    header.append(0xff);
    header.append(0xff);
    header.append(0xff);
    header.append(0xff);
//    startTimer(10);
//    header.append(0xe0);

}
void MainWindow::timerEvent(QTimerEvent *event)
{
    //    downloadImage();
//    if(videoSocket->isOpen())
//    videoReceive();
}
void MainWindow::videoReceive()
{
    while (videoSocket->hasPendingDatagrams())
    {
        QByteArray datagram;
        datagram.resize(videoSocket->pendingDatagramSize());
        videoSocket->readDatagram(datagram.data(),datagram.length());

        //    if(jpegData.length()<2000)return;
        if(datagram.size()==4)//(input.at(0)==0x0d)&&(input.at(1)==0x0a)&&(input.at(2)==0x2d)&&(input.at(4)==0x2d))
        {
//            printf("tcpSocket:%s\n",input.toHex().data());_flushall();
            QPixmap pixmap;
            pixmap.loadFromData(jpegData);
            if(!pixmap.isNull())
            {

                ui->label_img->setPixmap(pixmap);
                this->repaint();
            }
            jpegData.clear();

        }
        else jpegData.append(datagram);
    }


}
#define PI 3.1415926535
void MainWindow::udpDataReceive()
{

    QByteArray datagram;
    datagram.resize(udpSocket->pendingDatagramSize());
    udpSocket->readDatagram(datagram.data(),datagram.size());
//
    QByteArrayList datalist = datagram.split(',');
    if(datalist.size()<7)return;

    float accelX = datalist.at(1).toFloat();
    float accelY = datalist.at(2).toFloat();
    float accelZ = datalist.at(3).toFloat();
    float pitch = 180 * atan2(accelX, sqrt(accelY*accelY + accelZ*accelZ))/PI+90;
    float roll = 180 * atan2(accelY, sqrt(accelX*accelX + accelZ*accelZ))/PI;
    ui->openGLWidget->setEulerAngle(pitch,roll,180);
    printf("roll:%f pitch:%f\n",pitch,roll);_flushall();
}
void MainWindow::downloadImage()
{

    //    QUrl url("http://192.168.1.18:8080");
    //    QNetworkRequest request(url);
    //    m_netwManager->get(request);
    //    printf("request sent\n");_flushall();

}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_pushButton_clicked()
{

    videoSocket->connectToHost("192.168.1.18",8080);
    videoSocket->waitForConnected();
    videoSocket->write("GET / \r\n\r\n");

}
