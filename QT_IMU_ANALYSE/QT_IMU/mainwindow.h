#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "imu_driver.h"

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    IMU_driver imu;

private slots:
    void on_actionConnect_triggered();

    void on_actionConfig_mode_triggered();

    void on_actionMeasure_mode_triggered();

private:
    Ui::MainWindow *ui;
};
#endif // MAINWINDOW_H
