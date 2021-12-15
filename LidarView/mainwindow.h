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

public slots:
    void serialData();
protected slots:
    void paintEvent(QPaintEvent *event);
private:
    Ui::MainWindow *ui;
    void processFrame(QByteArray data);
};

#endif // MAINWINDOW_H
