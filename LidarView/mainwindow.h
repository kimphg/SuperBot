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
    unsigned short tofData[64];
    unsigned short tofDataOld[64];
    int tofDataDiff[64];
    float fps=0;
    float bps=0;
    int frameCount=0;
    float mScale = 10.0;// mỗi pixel tương ứng 10mm
    int byteCount = 0;
    bool newDataPending = false;
    Ui::MainWindow *ui;
    void processFrame(QByteArray data);
    void processFrameTOF(unsigned char *data);
};

#endif // MAINWINDOW_H
