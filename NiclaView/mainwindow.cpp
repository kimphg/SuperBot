#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFileDialog>
#include <QPainter>
#include <QDateTime>

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
    discoverySocket = new QUdpSocket(this);
    discoverySocket->bind(31003, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);
    connect(discoverySocket,SIGNAL(readyRead()),this,SLOT(discoveryReceive()));
    panoramaView  =QPixmap(900,900);
    header.append(0xff);
    header.append(0xff);
    header.append(0xff);
    header.append(0xff);
    roll_max=-1000;
    roll_min=1000;
    pitch_min=1000;
    pitch_max=-1000;

    heartbeatTimer = new QTimer(this);
    connect(heartbeatTimer, SIGNAL(timeout()), this, SLOT(sendHeartbeat()));
    heartbeatTimer->start(1000);  // send PING every 1 s

    updateButtonStates();
    this->showMaximized();

//    startTimer(10);
//    header.append(0xe0);

}
void MainWindow::timerEvent(QTimerEvent *event)
{
    //    downloadImage();
//    if(videoSocket->isOpen())
//    videoReceive();
}

void MainWindow::discoveryReceive()
{
    while (discoverySocket->hasPendingDatagrams())
    {
        QByteArray datagram;
        datagram.resize(discoverySocket->pendingDatagramSize());
        QHostAddress sender;
        quint16 senderPort;
        discoverySocket->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);
        if (datagram.startsWith("NICLA_HELLO") && niclaAddress.isNull())
        {
            niclaAddress = sender;
            statusBar()->showMessage("Nicla discovered: " + sender.toString());
        }
    }
}

void MainWindow::updateButtonStates()
{
    static const QString styleActive   = "background-color:rgb(0,150,0);color:rgb(255,255,255);font:10pt \"MS Shell Dlg 2\";border:3px solid gray;";
    static const QString styleDanger   = "background-color:rgb(180,0,0);color:rgb(255,255,255);font:10pt \"MS Shell Dlg 2\";border:3px solid gray;";
    static const QString styleNormal   = "background-color:rgb(32,64,128);color:rgb(255,255,255);font:10pt \"MS Shell Dlg 2\";border:3px solid gray;";
    static const QString styleDisabled = "background-color:rgb(96,96,96);color:rgb(160,160,160);font:10pt \"MS Shell Dlg 2\";border:3px solid gray;";

    if (isMeasuring) {
        ui->pushButton->setStyleSheet(styleDisabled);
        ui->pushButton->setEnabled(false);
        ui->pushButton_2->setStyleSheet(styleDanger);   // Stop = red, prominent
        ui->pushButton_2->setEnabled(true);
        ui->pushButton_4->setStyleSheet(styleDisabled);
        ui->pushButton_4->setEnabled(false);
    } else {
        ui->pushButton->setStyleSheet(styleActive);     // Start = green, prominent
        ui->pushButton->setEnabled(true);
        ui->pushButton_2->setStyleSheet(styleDisabled);
        ui->pushButton_2->setEnabled(false);
        ui->pushButton_4->setStyleSheet(styleNormal);
        ui->pushButton_4->setEnabled(true);
    }
}

void MainWindow::sendHeartbeat()
{
    if (!niclaAddress.isNull())
        udpSocket->writeDatagram("PING\n", niclaAddress, 31002);

    bool connected = (lastImuMs > 0) &&
                     (QDateTime::currentMSecsSinceEpoch() - lastImuMs < 3000);
    if (!connected && wasConnected)
        jpegData.clear();   // discard partial video frame accumulated while link was live
    wasConnected = connected;
    statusBar()->showMessage(connected ? "Connected" : "Disconnected");
}
void MainWindow::videoReceive()
{
    while (videoSocket->hasPendingDatagrams())
    {
        QByteArray datagram;
        datagram.resize(videoSocket->pendingDatagramSize());
        videoSocket->readDatagram(datagram.data(), datagram.length());

        if (datagram.size() == 4)   // frame-end sentinel \x02\x03\x05\x07
        {
            pixmap.loadFromData(jpegData);
            if (!pixmap.isNull())
            {
                ui->label_img_center->setPixmap(pixmap);
                this->repaint();
                imgReady = true;
            }
            jpegData.clear();
        }
        else
        {
            jpegData.append(datagram);
        }
    }
}
#define PI 3.1415926535
void MainWindow::udpDataReceive()
{
    while (udpSocket->hasPendingDatagrams())
    {
        QByteArray datagram;
        datagram.resize(udpSocket->pendingDatagramSize());
        QHostAddress sender;
        quint16 senderPort;
        udpSocket->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);

        // Fallback discovery: capture sender IP if not yet found via beacon
        if (niclaAddress.isNull())
            niclaAddress = sender;

        lastImuMs = QDateTime::currentMSecsSinceEpoch();

        // Packet format: IMU,<seq>,<ts_ms>,<roll>,<pitch>,<yaw>
        QByteArrayList datalist = datagram.split(',');
        if (datalist.size() < 6) continue;

        roll = -datalist.at(3).toFloat();
        ui->plotter->addRecord(0, roll);

        pitch = datalist.at(4).toFloat();
        if (pitch > 85) pitch -= 10;
        float yaw = datalist.at(5).toFloat();

        if (abs(roll) > 90)
        {
            pitch = 180 - pitch;
            while (roll < -180) roll += 360;
        }
        ui->plotter->addRecord(1, pitch);
        ui->label_pitch->setText(QString::number(pitch - 90));
        ui->label_roll->setText(QString::number(roll));
        if (!isMeasuring) continue;

        if (pitch < pitch_min)
        {
            pitch_min = pitch;
            if (!pixmap.isNull()) ui->label_img_bot->setPixmap(pixmap);
        }
        if (pitch > pitch_max)
        {
            pitch_max = pitch;
            if (!pixmap.isNull()) ui->label_img_top->setPixmap(pixmap);
        }
        if (roll < roll_min)
        {
            roll_min = roll;
            if (!pixmap.isNull()) ui->label_img_left->setPixmap(pixmap);
        }
        if (roll > roll_max)
        {
            roll_max = roll;
            if (!pixmap.isNull()) ui->label_img_right->setPixmap(pixmap);
        }

        rrol = pitch + 90;
        rpit = -roll;

        ui->label_pitch_min->setText(QString::number(pitch_min));
        ui->label_pitch_max->setText(QString::number(pitch_max));
        ui->label_roll_min->setText(QString::number(roll_min));
        ui->label_roll_max->setText(QString::number(roll_max));
    }
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
    isMeasuring = true;
    roll_max=-1000;
    roll_min=1000;
    pitch_min=1000;
    pitch_max=-1000;
    ui->plotter->resetRecord();
    ui->label_mes_count->setText(QString::number(report.getCount_rec()));
    updateButtonStates();
}

void MainWindow::on_pushButton_2_clicked()
{
    isMeasuring = false;
    ui->plotter->stopRecord();
    updateButtonStates();
}

void MainWindow::on_pushButton_3_clicked()
{
    QDateTime currentDateTime = QDateTime::currentDateTime();


    QString bnName = ui->lineEdit->text();
    QString bnCode = ui->lineEdit_2->text();
    QString bnNS = ui->lineEdit_3->text();
    QString tiltdata =  ui->label_pitch_min->text()+ "/"+ui->label_pitch_max->text();
    QString pandata =  ui->label_roll_min->text()+ "/"+ui->label_roll_max->text();
    QString date = currentDateTime.toString("dd_MM_yyyy_hh_mm")+"_"+bnName;
    QString fileName = "D:/record/"+date;//QFileDialog::getSaveFileName();
    QPixmap pixmap = QPixmap::grabWidget(ui->plotter);
    pixmap = pixmap.scaled(700,350);
    QFile file(date+".png");
    file.open(QIODevice::WriteOnly);
    pixmap.save(&file, "PNG");
    pixmap = QPixmap::grabWidget(ui->frame);
    pixmap = pixmap.scaled(700,350);
    QFile file2(date+"video.png");
    file2.open(QIODevice::WriteOnly);
    pixmap.save(&file2, "PNG");
    report.insertRecord(bnName,bnCode,bnNS,tiltdata,pandata);
    ui->label_mes_count->setText(QString::number(report.getCount_rec()));
     ui->plotter->saveCsv(date);
}

void MainWindow::on_pushButton_4_clicked()
{
    report.resetData();
    updateButtonStates();
}

void MainWindow::on_pushButton_5_clicked()
{
    QDesktopServices::openUrl(report.lastFileName);
}
