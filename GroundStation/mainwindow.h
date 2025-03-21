#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#define MAP_PATH_VecTor QString("C:/MAPDATA/Vector")  //IMG
#include <QMainWindow>
#include "c_drone.h"
#include <cmapwidget.h>

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
    void paintEvent(QPaintEvent *event);
    void timerEvent(QTimerEvent *event);
    void keyPressEvent(QKeyEvent *event);
private slots:
    void on_pushButton_2_clicked();

    void on_actionConnect_triggered();

    void on_pushButton_clicked();

    void on_pushButton_6_clicked();

    void on_pushButton_7_clicked(bool checked);

    void on_pushButton_5_clicked();

    void on_pushButton_4_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_stop_clicked();

    void on_pushButton_stop_2_clicked();

    void on_pushButton_up_clicked();

    void on_pushButton_down_clicked();

    void on_pushButton_8_clicked();

    void on_pushButton_9_clicked();

    void on_pushButton_send_command_clicked();

    void on_horizontalSlider_2_valueChanged(int value);

    void on_pushButton_stop_3_clicked();

    void on_pushButton_stop_4_clicked();

    void on_pushButton_7_clicked();

    void on_pushButton_10_clicked();

    void on_pushButton_2_clicked(bool checked);

    void on_pushButton_11_clicked();

    void on_comboBox_robot_select_currentTextChanged(const QString &arg1);

private:
    QRect mapRect;
    c_drone mDrone;
    Ui::MainWindow *ui;
};
#endif // MAINWINDOW_H
