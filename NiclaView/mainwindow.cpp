#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFileDialog>
#include <QPainter>
#include <QDateTime>
#include <QHBoxLayout>

// ── Frame protocol (must match firmware) ─────────────────────────────────────
static const QByteArray FRAME_MAGIC  = "NICL";
static const int        HEADER_SIZE  = 9;      // 4 magic + 1 type + 4 length
static const quint8     TYPE_IMU     = 0x01;
static const quint8     TYPE_VIDEO   = 0x02;

// Known USB VIDs for Nicla Vision / OpenMV
static const quint16 VID_STM32   = 0x0483;
static const quint16 VID_ARDUINO = 0x2341;

// ─────────────────────────────────────────────────────────────────────────────
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    panoramaView = QPixmap(900, 900);
    roll_max = -1000; roll_min = 1000;
    pitch_min = 1000; pitch_max = -1000;

    initChart();

    // Serial port
    serialPort = new QSerialPort(this);
    serialPort->setBaudRate(QSerialPort::Baud115200);
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

    retryTimer = new QTimer(this);
    retryTimer->setSingleShot(false);
    connect(retryTimer, &QTimer::timeout, this, &MainWindow::retryConnection);

    autoConnectSerial();
    updateButtonStates();
    this->showMaximized();
}

// ── Chart setup ───────────────────────────────────────────────────────────────
void MainWindow::initChart()
{
    rollSeries  = new QLineSeries();
    pitchSeries = new QLineSeries();
    rollSeries->setName(QString::fromUtf8("Góc xoay (roll)"));
    pitchSeries->setName(QString::fromUtf8("Góc ngẩng (pitch)"));

    QPen rp(QColor(220, 0, 0));   rp.setWidth(2); rollSeries->setPen(rp);
    QPen pp(QColor(0, 0, 200));   pp.setWidth(2); pitchSeries->setPen(pp);

    chart = new QChart();
    chart->addSeries(rollSeries);
    chart->addSeries(pitchSeries);
    chart->setTitle("");
    chart->setMargins(QMargins(4, 4, 4, 4));
    chart->legend()->setVisible(true);
    chart->legend()->setAlignment(Qt::AlignTop);

    xAxis = new QValueAxis();
    xAxis->setTitleText(QString::fromUtf8("Thời gian (s)"));
    xAxis->setRange(0, 10);
    xAxis->setTickCount(11);
    xAxis->setLabelFormat("%.0f");
    xAxis->setGridLineVisible(true);

    yAxis = new QValueAxis();
    yAxis->setTitleText(QString::fromUtf8("Góc (°)"));
    yAxis->setRange(-180, 180);
    yAxis->setTickCount(13);
    yAxis->setLabelFormat("%.0f");
    yAxis->setGridLineVisible(true);

    chart->addAxis(xAxis, Qt::AlignBottom);
    chart->addAxis(yAxis, Qt::AlignLeft);
    rollSeries->attachAxis(xAxis);  rollSeries->attachAxis(yAxis);
    pitchSeries->attachAxis(xAxis); pitchSeries->attachAxis(yAxis);

    chartView = new QChartView(chart, ui->plotter);
    chartView->setRenderHint(QPainter::Antialiasing);

    QHBoxLayout *layout = new QHBoxLayout(ui->plotter);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(chartView);
}

// ── Serial auto-connect ───────────────────────────────────────────────────────
void MainWindow::autoConnectSerial()
{
    const auto ports = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &info : ports) {
        quint16 vid = info.vendorIdentifier();
        if (vid == VID_STM32 || vid == VID_ARDUINO) {
            connectSerial(info.portName());
            return;
        }
    }
    for (const QSerialPortInfo &info : ports) {
        QString desc = info.description().toLower();
        if (desc.contains("openmv") || desc.contains("nicla") ||
            desc.contains("stm32") || desc.contains("virtual com")) {
            connectSerial(info.portName());
            return;
        }
    }
    statusBar()->showMessage(
        "Nicla not found — install OpenMV IDE driver, then plug in USB");
    retryTimer->start(2000);
}

void MainWindow::connectSerial(const QString &portName)
{
    serialPort->setPortName(portName);
    if (serialPort->open(QIODevice::ReadWrite)) {
        serialPort->setDataTerminalReady(true);
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

// ── Serial frame parsing ──────────────────────────────────────────────────────
void MainWindow::serialReceive()
{
    serialBuffer.append(serialPort->readAll());
    processSerialBuffer();
}

void MainWindow::processSerialBuffer()
{
    while (true) {
        int idx = serialBuffer.indexOf(FRAME_MAGIC);
        if (idx < 0) {
            if (serialBuffer.size() > 3)
                serialBuffer = serialBuffer.right(3);
            break;
        }
        if (idx > 0) { serialBuffer.remove(0, idx); continue; }
        if (serialBuffer.size() < HEADER_SIZE) break;

        quint8  type   = static_cast<quint8>(serialBuffer[4]);
        quint32 length = (static_cast<quint8>(serialBuffer[5]) << 24)
                       | (static_cast<quint8>(serialBuffer[6]) << 16)
                       | (static_cast<quint8>(serialBuffer[7]) <<  8)
                       |  static_cast<quint8>(serialBuffer[8]);

        if (serialBuffer.size() < HEADER_SIZE + (int)length) break;

        QByteArray payload = serialBuffer.mid(HEADER_SIZE, length);
        serialBuffer.remove(0, HEADER_SIZE + length);

        if (type == TYPE_IMU)   parseIMU(payload);
        else if (type == TYPE_VIDEO) parseVideo(payload);
    }
}

// ── IMU: "IMU,seq,ts_ms,roll,pitch,yaw\n" ────────────────────────────────────
void MainWindow::parseIMU(const QByteArray &payload)
{
    QByteArrayList d = payload.split(',');
    if (d.size() < 6) return;

    qint64 ts_ms = d.at(2).toLongLong();
    roll  = -d.at(3).toFloat();
    pitch =  d.at(4).toFloat();
    if (pitch > 85) pitch -= 10;

    if (abs(roll) > 90) {
        pitch = 180 - pitch;
        while (roll < -180) roll += 360;
    }

    float displayPitch = pitch - 90;   // 0 = level, + = looking up

    ui->label_pitch->setText(QString::number(displayPitch, 'f', 1));
    ui->label_roll->setText(QString::number(roll, 'f', 1));

    // ── chart update ─────────────────────────────────────────────────────
    if (firstTs < 0) firstTs = ts_ms;
    double elapsed = (ts_ms - firstTs) / 1000.0;

    if (isRecording) {
        rollSeries->append(elapsed, roll);
        pitchSeries->append(elapsed, displayPitch);

        // Auto-extend x-axis in 10-second steps
        if (elapsed >= maxElapsed) {
            maxElapsed = elapsed + 10.0;
            xAxis->setRange(0, maxElapsed);
            // Adjust tick spacing: 1 tick per 10 s, max 20 ticks
            int ticks = qMin((int)(maxElapsed / 10) + 1, 20);
            xAxis->setTickCount(ticks + 1);
        }
    }

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

    ui->label_pitch_min->setText(QString::number(pitch_min - 90, 'f', 1));
    ui->label_pitch_max->setText(QString::number(pitch_max - 90, 'f', 1));
    ui->label_roll_min->setText(QString::number(roll_min, 'f', 1));
    ui->label_roll_max->setText(QString::number(roll_max, 'f', 1));
}

// ── Video: raw JPEG ───────────────────────────────────────────────────────────
void MainWindow::parseVideo(const QByteArray &jpegData)
{
    pixmap.loadFromData(jpegData);
    if (!pixmap.isNull()) {
        ui->label_img_center->setPixmap(pixmap);
        repaint();
        imgReady = true;
    }
}

// ── CSV export ────────────────────────────────────────────────────────────────
void MainWindow::saveCsv(const QString &filename)
{
    QFile file(filename + ".csv");
    if (!file.open(QIODevice::WriteOnly)) return;
    QByteArray out;
    out.append("STT,Time(s),Roll(deg),Pitch(deg)\n");
    int n = qMin(rollSeries->count(), pitchSeries->count());
    for (int i = 0; i < n; i++) {
        QPointF r = rollSeries->at(i);
        QPointF p = pitchSeries->at(i);
        out += QByteArray::number(i + 1) + ","
             + QByteArray::number(r.x(), 'f', 3) + ","
             + QByteArray::number(r.y(), 'f', 2) + ","
             + QByteArray::number(p.y(), 'f', 2) + "\n";
    }
    file.write(out);
    file.close();
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
void MainWindow::on_pushButton_clicked()    // Start
{
    isMeasuring  = true;
    isRecording  = true;
    roll_max = -1000; roll_min = 1000;
    pitch_min = 1000; pitch_max = -1000;

    rollSeries->clear();
    pitchSeries->clear();
    firstTs    = -1;
    maxElapsed = 10.0;
    xAxis->setRange(0, maxElapsed);
    xAxis->setTickCount(11);

    ui->label_mes_count->setText(QString::number(report.getCount_rec()));
    updateButtonStates();
}

void MainWindow::on_pushButton_2_clicked()  // Stop
{
    isMeasuring = false;
    isRecording = false;
    updateButtonStates();
}

void MainWindow::on_pushButton_3_clicked()  // Save
{
    QDateTime dt = QDateTime::currentDateTime();
    QString bnName   = ui->lineEdit->text();
    QString bnCode   = ui->lineEdit_2->text();
    QString bnNS     = ui->lineEdit_3->text();
    QString tiltdata = ui->label_pitch_min->text() + "/" + ui->label_pitch_max->text();
    QString pandata  = ui->label_roll_min->text()  + "/" + ui->label_roll_max->text();
    QString base     = dt.toString("dd_MM_yyyy_hh_mm") + "_" + bnName;

    QPixmap g = QPixmap::grabWidget(ui->plotter);
    g = g.scaled(700, 350);
    QFile f1(base + ".png"); f1.open(QIODevice::WriteOnly); g.save(&f1, "PNG");

    g = QPixmap::grabWidget(ui->frame);
    g = g.scaled(700, 350);
    QFile f2(base + "video.png"); f2.open(QIODevice::WriteOnly); g.save(&f2, "PNG");

    report.insertRecord(bnName, bnCode, bnNS, tiltdata, pandata);
    ui->label_mes_count->setText(QString::number(report.getCount_rec()));
    saveCsv(base);
}

void MainWindow::on_pushButton_4_clicked()  // Reset
{
    report.resetData();
    updateButtonStates();
}

void MainWindow::on_pushButton_5_clicked()  // Open PDF
{
    QDesktopServices::openUrl(report.lastFileName);
}

MainWindow::~MainWindow()
{
    delete ui;
}
