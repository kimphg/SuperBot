#include "c_evironment_widget.h"

#include <QFile>
#include <QIODevice>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPushButton>
#define COMMAND_LEN_MAX 100
uint8_t commandBuff[COMMAND_LEN_MAX];
uint8_t reportPacket[20];
int commandBuffIndex = 0;
uint8_t last_byte = 0;
int grid_size=50;

float dt=0.02;
uint8_t gencrc8(uint8_t *data, size_t len) {
  uint8_t crc = 0x00;
  size_t i;
  for (i = 0; i < len; i++) {
    crc ^= data[i];

  }
  return crc;
}
QList<QColor> *colorList = new QList<QColor>\
        ({
             QColor(0  ,0  ,0),//0
             QColor(220,0,0),
             QColor(220,140,0),//2
             QColor(220,220,0),
             QColor(140,250,0),//4
             QColor(0  ,220,0),
             QColor(140  ,140,140),//6
             QColor(0  ,220,220),
             QColor(0  ,140,220),//8
             QColor(0  ,0  ,220),
             QColor(140  ,0  ,220),//10
             QColor(220  ,0  ,140),
             QColor(255  ,255  ,255)//12
         });
QStringList labelList
({  "Time",
    "Speed Left",
    "Speed Right",
    "Speed Lift",
    "DesSpeed Left",
    "DesSpeed Right",
    "DesSpeed Lift",
    "Angle Error",
    "Lift Error",
    "DesRotSpeed"
 });
#define GRAPGH_COUNT 10
#define RECORD_COUNT_MAX 500
int plot_data[10][RECORD_COUNT_MAX];
uint8_t calcCS8(uint8_t* startbyte, uint8_t len)
{
    int cs = 0;
    for (int i = 0; i < len; i++) {
        cs ^= startbyte[i];
    }
    return cs;
}
#define CRC16 0x8005

uint16_t gen_crc16(const uint8_t *data, uint16_t size)
{
    uint16_t out = 0;
    int bits_read = 0, bit_flag;

    /* Sanity check: */
    if(data == NULL)
        return 0;

    while(size > 0)
    {
        bit_flag = out >> 15;

        /* Get next bit: */
        out <<= 1;
        out |= (*data >> bits_read) & 1; // item a) work from the least significant bits

        /* Increment bit counter: */
        bits_read++;
        if(bits_read > 7)
        {
            bits_read = 0;
            data++;
            size--;
        }

        /* Cycle check: */
        if(bit_flag)
            out ^= CRC16;

    }

    // item b) "push out" the last 16 bits
    int i;
    for (i = 0; i < 16; ++i) {
        bit_flag = out >> 15;
        out <<= 1;
        if(bit_flag)
            out ^= CRC16;
    }

    // item c) reverse the bits
    uint16_t crc = 0;
    i = 0x8000;
    int j = 0x0001;
    for (; i != 0; i >>=1, j <<= 1) {
        if (i & out) crc |= j;
    }

    return crc;
}
c_evironment_widget::c_evironment_widget(QWidget *parent) : QFrame(parent)
{

//    mDrone.setSpeed(0.3);
//    QPushButton *pos_button = new QPushButton(this);
//    pos_button->setText(tr("abc"));
//    pos_button->setGeometry(this->width()/2,this->height()/2,30,30);
    startTimer(20);
    //    mDrone.openSerialPort();
    serial= new QSerialPort;
    udpSocket = new QUdpSocket(this);
    udpSocket->bind(3333);
    serial->setPortName("COM17");
    serial->setBaudRate(57600);
    //        connect(udpSocket,SIGNAL(readyRead()),this,SLOT(udpDataReceive()));
    //    if (serial->open(QIODevice::ReadWrite)) {
    //        printf("Connected to Serial");
    //    } else {
    //        printf("Serial error");
    //    }
    reportPacket[0]=0xaa;
    reportPacket[1]=0x55;
    reportPacket[2]=0xf1;
    reportPacket[3]=0xf2;
    reportPacket[4]=0xff;
    reportPacket[5]=0x03;
    QFile file("pallet_map.csv");
    if(!file.open(QIODevice::ReadOnly)) {
        QMessageBox::information(0, "error", file.errorString());
    }

    QTextStream in(&file);
    rows=0;cols=0;
    while(!in.atEnd()) {
        QString line = in.readLine();
        QStringList fields = line.split(",");
        cols = fields.size();
        rows++;

        //        model->appendRow(fields);
    }

    file.close();

    int envWidth = grid_size*cols;
    int envHeight = grid_size*rows;
    this->setMinimumSize(envWidth,envHeight);
    homePos.setX(0+grid_size/2);
    homePos.setY(envHeight-grid_size/2);
}
void c_evironment_widget::mousePressEvent(QMouseEvent *event)
{
    if(event->button() == Qt::RightButton)
    {
        QPoint cellPos = getCellPos(event->pos());
        QString command = "$COM,m,";
        command.append(QString::number(cellPos.x()*1000));
        command.append(",");
        command.append(QString::number(cellPos.y()*1000));
//        command.append(",#\n");
        sendCommand(command.toUtf8(),false);

    }

}
QPoint c_evironment_widget::getCellPos(QPoint scrPos)
{
    QPoint output;
    output.setX(((scrPos.x()-homePos.x())/grid_size+0.5));
    output.setY(((homePos.y()-scrPos.y())/grid_size+0.5));
    return output;

}
bool c_evironment_widget::connectCom()
{
    if (serial->open(QIODevice::ReadWrite)) {
        printf("Connected to Serial");
        return  true;
    } else {
        printf("Serial error");
    }
    return false;
}
void c_evironment_widget::disconnectCom()
{
    serial->close();
}

bool c_evironment_widget::getParamUpdated()
{
    if(paramUpdated){
        paramUpdated=false;
        return true;
    }
    return paramUpdated;
}
void c_evironment_widget::udpDataReceive()
{
    while (udpSocket->hasPendingDatagrams()){
        int dataLen = udpSocket->pendingDatagramSize();
        QByteArray inputString;
        inputString.resize(dataLen);
        QHostAddress senderIP = QHostAddress();
        quint16 portx = 0;
        udpSocket->readDatagram(inputString.data(),dataLen,&senderIP,&portx);
        debugString = inputString;
        //        QByteArrayList dataList = inputString.split('!');
        //        for(int i=0;i<dataList.size();i++)
        //        {
        //            debugString = dataList[i];
        processDebugString(senderIP.toString());
        debugString="";
        ////            processDebugLine();
        //        }

    }

}
void c_evironment_widget::paintEvent(QPaintEvent *event)
{
    QPainter p(this);
    //    draw_graphs(&p);
    draw_grid(&p);

    foreach(c_drone drone, mDroneList)
    {
        draw_drones(&p,drone);
    }


}

void c_evironment_widget::draw_graphs(QPainter *p)
{
    int shifty=350;
    p->setPen(colorList->at(12));
    p->setBrush(colorList->at(12));
    p->drawRect(0,shifty,1000,950);


    //    int nData =GRAPGH_COUNT;
    for (int dataid=0;dataid<GRAPGH_COUNT;dataid++)
    {
        p->setBrush(colorList->at(dataid));
        p->drawRect(20+100*dataid,shifty+20,20,20);

    }
    p->setPen(colorList->at(0));
    for (int dataid=0;dataid<GRAPGH_COUNT;dataid++)
    {
        QString label = labelList.at(dataid);
        p->setBrush(colorList->at(dataid));
        p->drawText(40+100*dataid,shifty+20,100,20,0,label);
    }
    p->setPen(Qt::gray);
    for (int i=-4;i<=4;i++)
    {
        int value = shifty+380+(50*i);
        p->drawLine(0,value,1000,value);
    }
    p->setPen(Qt::black);
    for (int i=-4;i<=4;i+=2)
    {
        int value = shifty+380+(50*i);
        p->drawLine(0,value,1000,value);
    }
    for (int dataid=0;dataid<GRAPGH_COUNT;dataid++)
    {
        p->setPen(colorList->at(dataid));
        for (int x=1;x<=RECORD_COUNT_MAX;x++)
        {
            int x0=x-1;

            p->drawLine(x0*2,shifty+380-plot_data[dataid][x0],x*2,shifty+380-plot_data[dataid][x]);

        }
    }
    p->setPen(colorList->at(0));
    if(getIsRecording())p->drawText(10,shifty+70,"Recording");


}
void c_evironment_widget::draw_grid(QPainter *p)
{


    for (int i=0;i<=cols;i++)
        p->drawLine(i*grid_size,0,i*grid_size,rows*grid_size);
    for(int j=0;j<=rows;j++)
        p->drawLine(0,j*grid_size,cols*grid_size,j*grid_size);


}
void c_evironment_widget::draw_drones(QPainter *p,c_drone drone)
{
    QPen pen(Qt::blue);
    pen.setWidth(5);
    p->setPen(pen);
    int robotSize = 0.6*grid_size;
    QPointF pos = drone.getPosition();
    pos.setY(-pos.y());
    pos*=grid_size;
    float angle = drone.getAngle()/180.0*3.1415926535;
    float loadAngle = (90/180.0)*3.1415926535;
    //    printf("%f\n",mDrone.getAngle());
    pos+=homePos;
    //    pos*=grid_size;
    //    pos.setY(rows*grid_size-pos.y());
    QPointF dirPoint(pos.x()+2*robotSize*sin(angle),pos.y()-2*robotSize*cos(angle));
    p->drawLine(pos,dirPoint);
    p->drawEllipse(pos.x()-robotSize/2,pos.y()-robotSize/2,robotSize,robotSize);
    p->drawText(pos,drone.name);
    QPointF dirLoad(pos.x()+80*sin(loadAngle),pos.y()-80*cos(loadAngle));
    //    p->drawLine(pos,dirLoad);
    QPointF desPos(drone.desX,drone.desY);
    desPos.setY(-desPos.y());
    //    desPos+=QPointF(cols/2.0,1);
    desPos*=grid_size;
    desPos+=homePos;
    //    desPos.setY(rows*grid_size-desPos.y());
    QPen pen2(Qt::red);
    pen2.setWidth(4);
    p->setPen(pen2);
    p->drawLine(pos,desPos);
    p->drawEllipse(QPoint(width()-200,200),2,2);
    p->drawLine(QPoint(width()-400,200),QPoint(width(),200));
    p->drawLine(QPoint(width()-200,0),QPoint(width()-200,400));
    pen2.setWidth(2);
    p->setPen(pen2);
    p->drawLine(QPoint(width()-400,100),QPoint(width(),100));
    p->drawLine(QPoint(width()-100,0),QPoint(width()-100,400));
    p->drawLine(QPoint(width()-400,300),QPoint(width(),300));
    p->drawLine(QPoint(width()-300,0),QPoint(width()-300,400));
    pen2.setColor(Qt::blue);
    pen2.setWidth(4);
    p->setPen(pen2);
    p->drawEllipse(QPoint(width()-200+errx*1000,200-erry*1000),2,2);
}

void c_evironment_widget::processDebugString(QString addr)
{
    addr.remove(QChar(':'));
    addr.remove(QChar('f'));
    if(addr==this->robotIP){
        printf("\n%s:\t",addr.toStdString().data());
        printf("%s",debugString.toStdString().data());
    }
    if(!mDroneList.contains(addr))
    {
        c_drone newDrone;

        newDrone.name=addr;
        newDrone.setPosition(QPointF(0,0));
        mDroneList.insert(addr,newDrone);

    }
    frameCount++;
    if(debugString.front()!='$')
    {
        debugString.clear();
        return;
    }
    QByteArrayList dataFields= debugString.split(',');
    if(dataFields.size()<2){debugString="";return;}
    QByteArray msgID = dataFields[0];
    if(msgID=="$MCU")
    {
        //        int len= dataFields.length();
        if(dataFields.length()==20){
            float angle = dataFields.at(3).toDouble();
            mDroneList[addr].setAngle( angle);
            float botx  = dataFields.at(1).toDouble()/1000.0;
            float boty  = dataFields.at(2).toDouble()/1000.0;
            mDroneList[addr].setPosition(QPointF(botx,boty));
            mDroneList[addr].desX   = dataFields.at(9).toDouble()/1000.0;
            mDroneList[addr].desY        = dataFields.at(10).toDouble()/1000.0;
            errx = botx-mDroneList[addr].desX;
            erry = boty-mDroneList[addr].desY;
            liftAngle = dataFields.at(4).toDouble();
            tagID = dataFields.at(11).toInt();
            yawTagID = dataFields.at(12).toInt();
            robotStat = dataFields.at(13).toInt();
            warningLevel = dataFields.at(14).toInt();
            debugString.replace('\n',' ');
            curSpeedL = dataFields.at(5).toDouble();
            curSpeedR = dataFields.at(6).toDouble();
            //        printf("\n%s",debugString.toStdString().data());
        }
//        printf("%s:",addr.toStdString().data());
//        printf("\n%s",debugString.toStdString().data());

    }
    else if(msgID=="$CAM1")
    {
        debugString.replace('\n',' ');
        dcam1 = debugString;
//        printf("%s:",addr.toStdString().data());
//        printf("\n%s",debugString.toStdString().data());
    }
    else if(msgID=="$CAM2")
    {
        debugString.replace('\n',' ');
        dcam2=debugString;
//        printf("%s:",addr.toStdString().data());
//        printf("\n%s",debugString.toStdString().data());
    }
    else if(msgID=="$FRB")
    {
        debugString.replace('\n',' ');

    }
    else if(msgID=="$MCUPARAM")
    {

        if(dataFields.length()>=3){
            QByteArray paramName = dataFields.at(1);
            //             paramName.replace('\r',' ');
            //             paramName.replace('\n',' ');
            //             paramName.replace(' ',QByteArray());
            float paraValue = dataFields.at(2).toDouble();
            setParam(paramName,paraValue);
        }
        debugString="";

        return;
    }
    else if(msgID=="$BINCOMACK")
    {


        debugString="";
        return;
    }

    debugString="";
}

int c_evironment_widget::getTagID() const
{
    return tagID;
}
int c_evironment_widget::getRobotStat()
{
    return robotStat;
}
int c_evironment_widget::getWarningLevel()
{
    return warningLevel;
}

float c_evironment_widget::getCurSpeedR() const
{
    return curSpeedR;
}

float c_evironment_widget::getDesSpeedL() const
{
    return desSpeedL;
}

float c_evironment_widget::getCurSpeedL() const
{
    return curSpeedL;
}
int c_evironment_widget::getYawTagID() const
{
    return yawTagID;
}
void c_evironment_widget::processDebugLine()
{


}
void c_evironment_widget::addRecord()
{
    if(lastSyncSec%2==0)
    {
        plot_data[0][recordCount]=200.0;
    }
    else plot_data[0][recordCount]=-200;
    plot_data[1][recordCount]=curSpeedL*400.0;
    plot_data[2][recordCount]=curSpeedR*400.0;
    plot_data[3][recordCount]=curSpeedLift*800.0;
    plot_data[4][recordCount]=desSpeedL*400.0;
    plot_data[5][recordCount]=desSpeedR*400.0;
    plot_data[6][recordCount]=desSpeedLift*700.0;
    plot_data[7][recordCount]=angleError*2.0;
    plot_data[8][recordCount]=liftError*20.0;//posError;
    plot_data[9][recordCount]=desSpeedRot*100.0;//posError;
    recordCount++;
    if(recordCount>=RECORD_COUNT_MAX)
    {
        recordCount=0;
        isRecording = false;
    }
}
void c_evironment_widget::readSerial()
{

}
void c_evironment_widget::uploadParams(QString id, QString value)
{
    QByteArray command;
    command.append("$COM,set,");
    command.append(id.toLatin1());
    command.append(',');
    command.append(value.toLatin1());
    command.append(',');
    command.append('\n');

    udpSocket->writeDatagram(command,QHostAddress(robotIP),1234);
}

bool c_evironment_widget::getIsRecording() const
{
    return isRecording;
}
void c_evironment_widget::sendCommand(QByteArray command, bool isRec)
{
    command.append(',');
    int crc = gencrc8((uint8_t*)command.data(),command.length());
    command.append(QString::number(crc).toUtf8());
    command.append('#');
    command.append('\n');
    udpSocket->writeDatagram(command,QHostAddress(robotIP),1234);

    if(isRec)
    {
        isRecording = isRec;
        recordCount=0;
    }

}
void c_evironment_widget::sendMsg(QByteArray command, int cstype)
{
    //    if(serial->isOpen())
    //    {

    //        if(cstype==1){
    //            uint8_t cs = calcCS8((uint8_t*)command.data(),command.length());
    //            command.append((char*)&cs,1);
    //        }
    //        serial->write(command);
    //    }
    uint16_t crc = gen_crc16((uint8_t*)command.data(),command.length());
    command.append((char*)&crc+1,1);
    command.append((char*)&crc,1);
    udpSocket->writeDatagram(command,QHostAddress(robotIP),1234);

}

void c_evironment_widget::timerEvent(QTimerEvent *event)
{
    if(isRecording)addRecord();
    udpDataReceive();
    //    readSerial();
    //    float dt=0.02;
    //    float robotSpeed = 0.5*(mSpeedRight+mSpeedLeft);
    //    float robotRotation = (mSpeedLeft-mSpeedRight)/0.5;
    //    mDrone.setSpeed(robotSpeed);
    //    mDrone.setAngleSpeed(robotRotation);
    //    mDrone.update(dt);
    //    sendReport();
    this->repaint();
    _flushall();
}
void c_evironment_widget::sendReport()
{

}
float c_evironment_widget::loadParam(QByteArray id, float defaultValue)
{
    for (uint i=0;i<paramTable.size();i++ )
    {
        if(paramTable[i].paramName == (id))return paramTable[i].paraValue;
    }
    RobotParam newParam;
    newParam.paraValue = defaultValue;
    newParam.paramName = id;
    paramTable.push_back(newParam);
    return defaultValue;
}
void c_evironment_widget::setParam(QByteArray id, float value)
{

    for (uint i=0;i<paramTable.size();i++ )
    {
        if(paramTable[i].paramName==(id))
        {
            if(paramTable[i].paraValue!=value)
            {
                paramTable[i].paraValue = value;
                paramUpdated=true;
            }
            return;
        }
    }
    RobotParam newParam;
    newParam.paraValue = value;
    newParam.paramName = id;
    paramUpdated=true;
    paramTable.push_back(newParam);
    return ;
}
