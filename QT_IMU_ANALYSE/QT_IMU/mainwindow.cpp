#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    QObject::connect(imu, &SerialPortReader::dataReceived, this, &DeviceAirmarHandler::onDataReceived);

}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_actionConnect_triggered()
{
    bool connectOK = imu.Connect();
    if(connectOK)ui->statusbar->showMessage("Connected");
    else ui->statusbar->showMessage("Connection failed");
}

void MainWindow::on_actionConfig_mode_triggered()
{
    bool OK = imu.gotoConfig();
    if(OK)ui->statusbar->showMessage("Config mode OK");
    else ui->statusbar->showMessage("Config mode failed");
}

void MainWindow::on_actionMeasure_mode_triggered()
{
    bool OK = imu.gotoMeasurement();
    if(OK)ui->statusbar->showMessage("Measure mode OK");
    else ui->statusbar->showMessage("Measure mode failed");
}
