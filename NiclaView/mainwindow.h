#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QAuthenticator>
#include <QMainWindow>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QUdpSocket>
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
    void videoReceive();
    void on_pushButton_clicked();

    void udpDataReceive();
    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_4_clicked();

    void on_pushButton_5_clicked();

private:
    PdfReport report;
    QPixmap pixmap;
    bool isMeasuring = false;
    float rrol=0;
    float rpit=0;
    float roll=0;float pitch = 0;
    QPixmap panoramaView;
    bool imgReady=false;
    float roll_min,roll_max;
    float pitch_min,pitch_max;
    QByteArray jpegData,header ;
    Ui::MainWindow *ui;
    QNetworkAccessManager* m_netwManager ;
    void downloadImage();
    QUdpSocket *videoSocket;
    QUdpSocket *udpSocket;
};
#endif // MAINWINDOW_H
