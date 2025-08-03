#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QByteArray>
#include <QPainter>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    startTimer(100);
    this->showMaximized();
    ui->tabWidget->setCurrentIndex(1);
    repaint();
}
void MainWindow::timerEvent(QTimerEvent *event)
{
//    QPointF position = mDrone.getPosition();
//    float newX = position.x()+5;
//    float newY = position.y()+5;
//    if(newX>mapRect.width()/2)
//        newX=0;
//    if(newY>mapRect.height()/2)
//        newY=0;
//    position.setX(newX);
//    position.setY(newY);
//    mDrone.setPosition(position);
//    repaint();
//    QByteArray requestPacket;
//    requestPacket.append(0xaa);
//    requestPacket.append(0x55);
//    requestPacket.append(0xab);
//    ui->frame_view->sendMsg(requestPacket);
    foreach(c_drone drone, ui->frame_view->mDroneList)
    {
        QString id = drone.name;
//        QString name = id.toString();
        QStringList itemsInComboBox;
        for (int index = 0; index < ui->comboBox_robot_select->count(); index++)
            itemsInComboBox << ui->comboBox_robot_select->itemText(index);
        if(itemsInComboBox.contains(id))break;
        ui->comboBox_robot_select->addItem(id);

    }

    ui->label_tag_id->setText(QString::number(ui->frame_view->getTagID()));
    ui->label_tag_id_3->setText(QString::number(ui->frame_view->getRobotStat()));
    ui->label_tag_id_4->setText(QString::number(ui->frame_view->getWarningLevel()));
    ui->label_tag_id_2->setText(QString::number(ui->frame_view->getYawTagID()/1.0));
    ui->label_speed_L->setText(QString::number(ui->frame_view->getDesSpeedL()));
    ui->label_speed_R->setText(QString::number(ui->frame_view->getDesSpeedL()));
    int nreport = ui->frame_view->getFPS();
    float fps = nreport;
    ui->label_connection_fps->setText(QString::number(fps*10));
    int nItems = ui->frame_view->dataTable.size();
    ui->tableWidget->setRowCount(nItems);
    int rowcount = 0;
    ui->label_connection_last_msg->setText(QString::fromLatin1(ui->frame_view->lastMsg));
    for(QHash<QByteArray ,QByteArray>::iterator i=ui->frame_view->dataTable.begin();i!=ui->frame_view->dataTable.end();++i)
    {
        QString id = QString::fromLatin1(i.key());
        QString value = QString::fromLatin1(i.value());
        ui->tableWidget->setItem(rowcount,0,new QTableWidgetItem(id));
        ui->tableWidget->setItem(rowcount,1,new QTableWidgetItem(value));
        rowcount++;
    }
    if(ui->frame_view->getParamUpdated()==true){
        nItems = ui->frame_view->paramTable.size();
        ui->tableWidget_2->setRowCount(nItems+1);
        ui->tableWidget_2->setColumnCount(3);
        ui->tableWidget_2->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        for(uint i=0;i< ui->frame_view->paramTable.size();++i)
        {
            QString id = ui->frame_view->paramTable.at(i).paramName;
            float value = ui->frame_view->paramTable.at(i).paraValue;
            ui->tableWidget_2->setItem(i,0,new QTableWidgetItem(id));
            ui->tableWidget_2->setItem(i,1,new QTableWidgetItem(QString::number(value)));
            rowcount++;
        }
    }
    if(ui->frame_view->newCommand==true){
    ui->textBrowser->setText(ui->frame_view->commands.join(' '));
    ui->frame_view->newCommand=false;
    }

    ui->frame_view->sendCommand("$COM,sync");
}
void MainWindow::paintEvent(QPaintEvent *event)
{
//    mapRect = ui->frame_map->rect();
//    int wid = mapRect.width();
//    int hei = mapRect.height();
////    QPoint cennterPoint = QPoint(wid/2,hei/2);
//    QPainter p(this);
//    p.drawRect(mapRect);
//    p.drawLine(0,hei/2,wid,hei/2);
//    p.drawLine(wid/2,0,wid/2,hei);
//    // start drawing drone
//    QPoint cennterPoint = QPoint(wid/2,hei/2);
//    QPointF position = mDrone.getPosition();
//    position.setX(position.x()+wid/2);
//    position.setY(position.y()+hei/2);
//    int size = 15;
//    p.drawEllipse(position,size,size);
//    // done drawing drone

}
MainWindow::~MainWindow()
{
//    delete ui;
}


void MainWindow::on_pushButton_2_clicked()
{
    ui->frame_view->sendCommand("$COM,d,1");//tiến 1 ô
}

void MainWindow::on_actionConnect_triggered()
{
    ui->frame_view->connectCom();
}

void MainWindow::on_pushButton_clicked()
{
    QByteArray cmd = QByteArray::fromHex(ui->lineEdit_msg_1->text().toUtf8());

    ui->frame_view->sendMsg(cmd,1);
}

void MainWindow::on_pushButton_6_clicked()
{
    QByteArray cmd = (ui->lineEdit_msg_2->text().toUtf8());
        cmd.append('\n');
    ui->frame_view->sendCommand(cmd);
}

void MainWindow::on_pushButton_7_clicked(bool checked)
{
//    if(checked){

//        if(ui->frame_view->connectCom())
//        {
//            ui->pushButton_7->setChecked(true);
//            ui->label_connection_stat->setText("Connected");
//        }
//    }
//    else {
//        ui->frame_view->disconnectCom();
//        ui->pushButton_7->setChecked(false);
//        ui->label_connection_stat->setText("Not connected");
//    }
}

void MainWindow::on_pushButton_5_clicked()
{
    ui->frame_view->sendCommand("$COM,d,2");//tiến 2 ô
}

void MainWindow::on_pushButton_4_clicked()
{
    ui->frame_view->sendCommand("$COM,r,0");//xoay phải
}

void MainWindow::on_pushButton_3_clicked()
{
    ui->frame_view->sendCommand("$COM,r,2");//xoay trái
}

void MainWindow::on_pushButton_stop_clicked()
{
    ui->frame_view->sendCommand("$COM,s",false);//stop and standby
}

void MainWindow::on_pushButton_stop_2_clicked()
{
    ui->frame_view->sendCommand("$COM,s");//stop and standby
    ui->frame_view->sendCommand("$COM,rp");//stop and standby
}

void MainWindow::on_pushButton_up_clicked()
{
    ui->frame_view->sendCommand("$COM,lift,1");//stop and standby
}

void MainWindow::on_pushButton_down_clicked()
{
    ui->frame_view->sendCommand("$COM,lift,-1");
}

void MainWindow::on_pushButton_8_clicked()
{
    ui->frame_view->dataTable.clear();
    ui->frame_view->commands.clear();
    ui->textBrowser->clear();
}
void MainWindow::keyPressEvent(QKeyEvent *event)
{
    if( event->key() == Qt::Key_Space )
        {
            ui->frame_view->sendCommand("$COM,s",false);
        }
}
void MainWindow::on_pushButton_9_clicked()
{
    for (int i=0;i<ui->tableWidget_2->rowCount();i++) {
        QTableWidgetItem *item = ui->tableWidget_2->item(i,2);
        QString strvalue ="";
        if(item)strvalue=item->text();
        {
            bool ok=false;
            strvalue.toDouble(&ok);
            if(ok)
            {
                QString strid = ui->tableWidget_2->item(i,0)->text();
                ui->frame_view->uploadParams(strid,strvalue);
            }
        }
//        ui->frame_view->setParam(ui->tableWidget_2->itemAt(i,0)->text().toLatin1(),txt.toDouble());

    }

}

void MainWindow::on_pushButton_send_command_clicked()
{
    float x = ui->lineEdit_des_x->text().toDouble();
    float y = ui->lineEdit_des_y->text().toDouble();
    unsigned short id = ui->lineEdit_des_id->text().toUInt();
    char angle = ui->horizontalSlider_2->value();
    if(angle==2)angle=3;
    if(angle==-1)angle=2;

    QByteArray navCommand;
    QByteArray address("\xF2\x00");
    QByteArray commandCode("\x02\x00");
    navCommand.append("\xaa\x55\xaa");
    navCommand.append(address,2);
    navCommand.append(commandCode,2);
    navCommand.append("\x01");//speed 0.6
    short sx = x*10;
    short sy = y*10;
    navCommand.append((char*)&sx+1,1);
    navCommand.append((char*)&sx,1);
    navCommand.append((char*)&sy+1,1);
    navCommand.append((char*)&sy,1);

    navCommand.append((char*)&id+1,1);
    navCommand.append((char*)&id,1);
    unsigned char action_byte=0;
    action_byte+=angle;
    if(ui->pushButton_lift_before->isChecked())action_byte+=8;
    if(ui->pushButton_lift_after->isChecked())action_byte+=4;
    navCommand.append((char*)&action_byte,1);

    ui->frame_view->sendMsg(navCommand);
}

void MainWindow::on_horizontalSlider_2_valueChanged(int value)
{
    ui->label_des_a->setText(QString::number(value*90));
}

void MainWindow::on_pushButton_stop_3_clicked()
{
    ui->frame_view->sendCommand("$COM,home,0");
}

void MainWindow::on_pushButton_stop_4_clicked()
{
    ui->frame_view->sendCommand("$COM,param,0");
}

void MainWindow::on_pushButton_7_clicked()
{
     ui->frame_view->sendCommand("$COM,r,1");//xoay phải
}

void MainWindow::on_pushButton_10_clicked()
{
     ui->frame_view->sendCommand("$COM,r,3");//xoay phải
}

void MainWindow::on_pushButton_2_clicked(bool checked)
{

}

void MainWindow::on_pushButton_11_clicked()
{
    ui->frame_view->sendCommand("$COM,d,0.667");//xoay phải
}

void MainWindow::on_comboBox_robot_select_currentTextChanged(const QString &arg1)
{
    ui->frame_view->robotIP = arg1;
}

void MainWindow::on_comboBox_robot_select_currentIndexChanged(int index)
{
    ui->frame_view->robotIP = ui->comboBox_robot_select->currentText();

}

void MainWindow::on_pushButton_stop_5_clicked()
{
    ui->frame_view->sendCommand("$COM,dock,1000,2200");//xoay phải
}
