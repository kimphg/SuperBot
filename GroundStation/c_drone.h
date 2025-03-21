#ifndef C_DRONE_H
#define C_DRONE_H

#include <QPointF>
#include <QSerialPort>


class c_drone
{
public:

    c_drone();
    QString name;
    QPointF getPosition();
    void setPosition(QPointF newpos);
    void setSpeed(float speed);
    void setAngle(float angle);
    void update(float dt);
    void setAngleSpeed(float value);
    void openSerialPort();
    void sendCommand(QString command);
    float getAngle() const;
    float desX=0;
    float desY=0;
private:
    QSerialPort *m_serial=0;
    float x=0;
    float y=0;

    float speed=0;
    float angle=0;
    float angleSpeed=0;
};

#endif // C_DRONE_H
