#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void processFrameHex(unsigned char* data);
public slots:
    void serialData();
    void callback1s();
protected slots:
    void paintEvent(QPaintEvent *event);
    void timerEvent(QTimerEvent *event);
    void wheelEvent(QWheelEvent *event);
private:
    float fps=0;
    float bps=0;
    int frameCount=0;
    float mScale = 10.0;// mỗi pixel tương ứng 10mm
    int byteCount = 0;
    Ui::MainWindow *ui;
    void processFrame(QByteArray data);
};

#endif // MAINWINDOW_H
