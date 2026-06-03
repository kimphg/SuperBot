#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFileDialog>
#include <QPainter>
#include <QDateTime>

// ── Frame protocol constants (must match firmware) ────────────────────────────
static const QByteArray FRAME_MAGIC = "NICL";
static const int  HEADER_SIZE = 9;   // 4 magic + 1 type + 4 length
static const quint8 TYPE_IMU  = 0x01;
static const quint8 TYPE_VIDEO= 0x02;

// ── STMicroelectronics USB VID (Nicla Vision / OpenMV) ───────────────────────
static const quint16 STM_VID = 0x0483;

// ─────────────────────────────────────────────────────────────────────────────
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    panoramaView = QPixmap(900, 900);
    roll_max = -1000; roll_min = 1000;
    pitch_min = 1000; pitch_max = -1000;

    // Serial port
    serialPort = new QSerialPort(this);
    serialPort->setBaudRate(QSerialPort::Baud115200); // ignored by USB CDC
    connect(serialPort, &QSerialPort::readyRead,
            this, &MainWindow::serialReceive);
    connect(serialPort, &QSerialPort::errorOccurred,
            this, [this](QSerialPort::SerialPortError err) {
        if (err == QSerialPort::ResourceError) {
            serialPort->close();
            serialBuffer.clear();
            statusBar()->showMessage("Nicla disconnected — reconnecting…");
            retryTimer->start(2000);
        }
    });

    // Retry / auto-connect timer
    retryTimer = new QTimer(this);
    retryTimer->setSingleShot(false);
    connect(retryTimer, &QTimer::timeout, this, &MainWindow::retryConnection);

    autoConnectSerial();
    updateButtonStates();
    this->showMaximized();
}

// ── Serial auto-connect ───────────────────────────────────────────────────────
void MainWindow::autoConnectSerial()
{
    for (const QSerialPortInfo &info : QSerialPortInfo::availablePorts()) {
        if (info.vendorIdentifier() == STM_VID) {
            connectSerial(info.portName());
            return;
        }
    }
    statusBar()->showMessage("Nicla not found — plug in USB cable");
    retryTimer->start(2000);
}

void MainWindow::connectSerial(const QString &portName)
{
    serialPort->setPortName(portName);
    if (serialPort->open(QIODevice::ReadOnly)) {
        retryTimer->stop();
        statusBar()->showMessage("Connected: " + portName);
    } else {
        statusBar()->showMessage("Cannot open " + portName + " — retrying…");
        retryTimer->start(2000);
    }
}

void MainWindow::retryConnection()
{
    if (!serialPort->isOpen())
        autoConnectSerial();
}

// ── Serial data reception & binary frame parsing ──────────────────────────────
void MainWindow::serialReceive()
{
    serialBuffer.append(serialPort->readAll());
    processSerialBuffer();
}

void MainWindow::processSerialBuffer()
{
    while (true) {
        // Find magic
        int idx = serialBuffer.indexOf(FRAME_MAGIC);
        if (idx < 0) {
            // Keep last 3 bytes so a magic spanning reads isn't lost
            if (serialBuffer.size() > 3)
                serialBuffer = serialBuffer.right(3);
            break;
        }
        if (idx > 0) {
            serialBuffer.remove(0, idx);
            continue;
        }
        // Magic is at position 0; need at least a full header
        if (serialBuffer.size() < HEADER_SIZE)
            break;

        quint8  type   = static_cast<quint8>(serialBuffer[4]);
        quint32 length = (static_cast<quint8>(serialBuffer[5]) << 24)
                       | (static_cast<quint8>(serialBuffer[6]) << 16)
                       | (static_cast<quint8>(serialBuffer[7]) <<  8)
                       |  static_cast<quint8>(serialBuffer[8]);

        if (static_cast<int>(HEADER_SIZE + length) > serialBuffer.size())
            break;  // payload not fully received yet

        QByteArray payload = serialBuffer.mid(HEADER_SIZE, length);
        serialBuffer.remove(0, HEADER_SIZE + length);

        if (type == TYPE_IMU)
            parseIMU(payload);
        else if (type == TYPE_VIDEO)
            parseVideo(payload);
    }
}

// ── IMU frame: "IMU,seq,ts_ms,roll,pitch,yaw\n" ──────────────────────────────
void MainWindow::parseIMU(const QByteArray &payload)
{
    QByteArrayList datalist = payload.split(',');
    if (datalist.size() < 6) return;

    roll = -datalist.at(3).toFloat();
    ui->plotter->addRecord(0, roll);

    pitch = datalist.at(4).toFloat();
    if (pitch > 85) pitch -= 10;

    if (abs(roll) > 90) {
        pitch = 180 - pitch;
        while (roll < -180) roll += 360;
    }
    ui->plotter->addRecord(1, pitch);
    ui->label_pitch->setText(QString::number(pitch - 90));
    ui->label_roll->setText(QString::number(roll));

    if (!isMeasuring) return;

    if (pitch < pitch_min) {
        pitch_min = pitch;
        if (!pixmap.isNull()) ui->label_img_bot->setPixmap(pixmap);
    }
    if (pitch > pitch_max) {
        pitch_max = pitch;
        if (!pixmap.isNull()) ui->label_img_top->setPixmap(pixmap);
    }
    if (roll < roll_min) {
        roll_min = roll;
        if (!pixmap.isNull()) ui->label_img_left->setPixmap(pixmap);
    }
    if (roll > roll_max) {
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

// ── Video frame: raw JPEG bytes ───────────────────────────────────────────────
void MainWindow::parseVideo(const QByteArray &jpegData)
{
    pixmap.loadFromData(jpegData);
    if (!pixmap.isNull()) {
        ui->label_img_center->setPixmap(pixmap);
        repaint();
        imgReady = true;
    }
}

// ── Button state styling ──────────────────────────────────────────────────────
void MainWindow::updateButtonStates()
{
    static const QString styleActive   = "background-color:rgb(0,150,0);color:rgb(255,255,255);font:10pt \"MS Shell Dlg 2\";border:3px solid gray;";
    static const QString styleDanger   = "background-color:rgb(180,0,0);color:rgb(255,255,255);font:10pt \"MS Shell Dlg 2\";border:3px solid gray;";
    static const QString styleNormal   = "background-color:rgb(32,64,128);color:rgb(255,255,255);font:10pt \"MS Shell Dlg 2\";border:3px solid gray;";
    static const QString styleDisabled = "background-color:rgb(96,96,96);color:rgb(160,160,160);font:10pt \"MS Shell Dlg 2\";border:3px solid gray;";

    if (isMeasuring) {
        ui->pushButton->setStyleSheet(styleDisabled);  ui->pushButton->setEnabled(false);
        ui->pushButton_2->setStyleSheet(styleDanger);  ui->pushButton_2->setEnabled(true);
        ui->pushButton_4->setStyleSheet(styleDisabled);ui->pushButton_4->setEnabled(false);
    } else {
        ui->pushButton->setStyleSheet(styleActive);    ui->pushButton->setEnabled(true);
        ui->pushButton_2->setStyleSheet(styleDisabled);ui->pushButton_2->setEnabled(false);
        ui->pushButton_4->setStyleSheet(styleNormal);  ui->pushButton_4->setEnabled(true);
    }
}

// ── Stubs ─────────────────────────────────────────────────────────────────────
void MainWindow::timerEvent(QTimerEvent *) {}
void MainWindow::downloadImage() {}

// ── Button handlers ───────────────────────────────────────────────────────────
void MainWindow::on_pushButton_clicked()
{
    isMeasuring = true;
    roll_max = -1000; roll_min = 1000;
    pitch_min = 1000; pitch_max = -1000;
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
    QString bnName  = ui->lineEdit->text();
    QString bnCode  = ui->lineEdit_2->text();
    QString bnNS    = ui->lineEdit_3->text();
    QString tiltdata = ui->label_pitch_min->text() + "/" + ui->label_pitch_max->text();
    QString pandata  = ui->label_roll_min->text()  + "/" + ui->label_roll_max->text();
    QString date     = currentDateTime.toString("dd_MM_yyyy_hh_mm") + "_" + bnName;

    QPixmap grab = QPixmap::grabWidget(ui->plotter);
    grab = grab.scaled(700, 350);
    QFile f1(date + ".png"); f1.open(QIODevice::WriteOnly); grab.save(&f1, "PNG");

    grab = QPixmap::grabWidget(ui->frame);
    grab = grab.scaled(700, 350);
    QFile f2(date + "video.png"); f2.open(QIODevice::WriteOnly); grab.save(&f2, "PNG");

    report.insertRecord(bnName, bnCode, bnNS, tiltdata, pandata);
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

MainWindow::~MainWindow()
{
    delete ui;
}
