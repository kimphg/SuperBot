#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QTimer>
#include <QDateTime>
#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include "pdfreport.h"
#include <QDesktopWidget>

QT_CHARTS_USE_NAMESPACE

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

protected slots:
    void timerEvent(QTimerEvent *event);

private slots:
    void serialReceive();
    void retryConnection();
    void on_pushButton_clicked();
    void on_pushButton_2_clicked();
    void on_pushButton_3_clicked();
    void on_pushButton_4_clicked();
    void on_pushButton_5_clicked();

private:
    // ── serial ────────────────────────────────────────────────────────────
    QSerialPort *serialPort;
    QTimer      *retryTimer;
    QByteArray   serialBuffer;

    void autoConnectSerial();
    void connectSerial(const QString &portName);
    void processSerialBuffer();
    void parseIMU(const QByteArray &payload);
    void parseVideo(const QByteArray &payload);

    // ── chart ─────────────────────────────────────────────────────────────
    QChart      *chart;
    QChartView  *chartView;
    QLineSeries *rollSeries;
    QLineSeries *pitchSeries;
    QValueAxis  *xAxis;
    QValueAxis  *yAxis;
    qint64       firstTs   = -1;   // firmware ts_ms of first received packet
    double       maxElapsed = 10.0; // current right edge of x-axis (s)
    bool         isRecording = false;

    void initChart();
    void saveCsv(const QString &filename);

    // ── measurement state ─────────────────────────────────────────────────
    PdfReport report;
    QPixmap   pixmap;
    bool      isMeasuring = false;
    float     rrol = 0, rpit = 0;
    float     roll = 0, pitch = 0;
    QPixmap   panoramaView;
    bool      imgReady = false;
    float     roll_min, roll_max;
    float     pitch_min, pitch_max;

    Ui::MainWindow *ui;
    void updateButtonStates();
    void downloadImage();
};

#endif // MAINWINDOW_H
