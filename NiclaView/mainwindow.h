#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QTimer>
#include <QDateTime>
#include "pdfreport.h"
#include <QDesktopWidget>

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
    QByteArray header;

    Ui::MainWindow *ui;
    void updateButtonStates();
    void downloadImage();   // stub, kept for compat
};

#endif // MAINWINDOW_H
