#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFileDialog>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    videoSocket = new QUdpSocket(this);
    udpSocket = new QUdpSocket(this);
    bool isOK = udpSocket->bind(31000);
    if(!isOK)QApplication::exit();
    isOK = videoSocket->bind(31001);
    if(!isOK)QApplication::exit();
    connect(udpSocket,SIGNAL(readyRead()),this,SLOT(udpDataReceive()));
    connect(videoSocket,SIGNAL(readyRead()),this,SLOT(videoReceive()));

    header.append(0xff);
    header.append(0xff);
    header.append(0xff);
    header.append(0xff);
    roll_max=-1000;
    roll_min=1000;
    pitch_min=1000;
    pitch_max=-1000;
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
                imgReady=true;
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
    if(datalist.size()<4)return;

    float roll = -datalist.at(1).toFloat();


    float pitch = datalist.at(2).toFloat();
    float yaw = datalist.at(3).toFloat();


    if(abs(roll)>90)
    {
        pitch=180-pitch;
//        roll=yaw;//-=180;
        while(roll<-180)roll+=360;
    }
    if(pitch<pitch_min)
    {
        pitch_min=pitch;
        if(imgReady)ui->label_img_bot->setPixmap(*(ui->label_img->pixmap()));
    }

    if(pitch>pitch_max)
    {
        pitch_max = pitch;
        if(imgReady)ui->label_img_top->setPixmap(*(ui->label_img->pixmap()));
    }
    if(roll<roll_min)
    {
        roll_min=roll;
        if(imgReady)ui->label_img_left->setPixmap(*(ui->label_img->pixmap()));
    }
    if(roll>roll_max)
    {
        roll_max = roll;
        if(imgReady)ui->label_img_right->setPixmap(*(ui->label_img->pixmap()));
    }
//    float pitch = 180 * atan2(accelX, sqrt(accelY*accelY + accelZ*accelZ))/PI+90;
//    float roll = 180 * atan2(accelY, sqrt(accelX*accelX + accelZ*accelZ))/PI;
    float rrol=pitch+135;
    float rpit = -roll;
    float ryaw =180;
//        float rrol=ui->horizontalSlider->value();
//        float rpit = ui->horizontalSlider_2->value();
//        float ryaw =ui->horizontalSlider_3->value();
    ui->openGLWidget->setEulerAngle(rrol,rpit,ryaw);
    ui->label_pitch->setText(QString::number(pitch-90));
    ui->label_roll->setText(QString::number(roll));
    ui->label_pitch_min->setText(QString::number(pitch_min-90));
    ui->label_pitch_max->setText(QString::number(pitch_max-90));
    ui->label_roll_min->setText(QString::number(roll_min));
    ui->label_roll_max->setText(QString::number(roll_max));
//    ui->label_yaw->setText(QString::number(yaw));
//    printf("pitch:%f roll:%f\n",pitch,roll);_flushall();
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

    roll_max=-1000;
    roll_min=1000;
    pitch_min=1000;
    pitch_max=-1000;

}

void MainWindow::on_pushButton_2_clicked()
{
    QString fileName = QFileDialog::getSaveFileName();
    QPixmap pixmap = QPixmap::grabWidget(this);
    QFile file(fileName+".png");
    file.open(QIODevice::WriteOnly);
    pixmap.save(&file, "PNG");
}
