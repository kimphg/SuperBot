#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QAuthenticator>
#include <QMainWindow>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QUdpSocket>

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
private:
    QByteArray jpegData,header ;
    Ui::MainWindow *ui;
    QNetworkAccessManager* m_netwManager ;
    void downloadImage();
    QUdpSocket *videoSocket;
    QUdpSocket *udpSocket;
};
#endif // MAINWINDOW_H
