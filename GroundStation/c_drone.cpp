#include "c_drone.h"
#define PI 3.141592535
c_drone::c_drone()
{


}

QPointF c_drone::getPosition()
{
    return(QPointF(this->x,this->y));
}

void c_drone::setPosition(QPointF newpos)
{
    this->x = newpos.x();
    this->y = newpos.y();
}

void c_drone::setSpeed(float speed)
{
    this->speed=speed;
}

void c_drone::setAngle(float angle)
{
    this->angle = angle;
}

void c_drone::update(float dt)
{
    if(m_serial)
    {
        if(m_serial->isOpen())
        {
            if(m_serial->bytesAvailable())
            {
                QByteArray input = m_serial->readAll();
                printf("%s\n",input.data());
                _flushall();
            }
        }
    }
    setAngle(angle+angleSpeed*dt);
    this->x+= speed*sin(angle*PI/180.0)*dt;
    this->y+= speed*cos(angle*PI/180.0)*dt;

}

void c_drone::setAngleSpeed(float value)
{
    angleSpeed = value;
}

void c_drone::openSerialPort()
{
    m_serial= new QSerialPort;
    m_serial->setPortName("COM26");
    m_serial->setBaudRate(115200);
    if (m_serial->open(QIODevice::ReadWrite)) {
        printf("Connected to Serial");
    } else {
        printf("Serial error");
    }

}

void c_drone::sendCommand(QString command)
{
    m_serial->write(command.toUtf8());

}

float c_drone::getAngle() const
{
    return angle;
}


