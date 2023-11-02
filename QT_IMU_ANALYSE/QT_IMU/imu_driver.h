#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H
#include <qthread.h>
#include <QSerialPort>
typedef struct
{

}IMUData;
class IMU_driver : public QThread
{
public:
    Q_OBJECT
    IMU_driver()
    {

    }
    bool Connect()
      {
        port.setBaudRate(921600);
        port.setPortName("COM30");
        isConnected = false;
        if(port.isOpen())port.close();
        bool isOpen = port.open(QIODevice::ReadWrite);
        if(!isOpen){
            return false;
        }
        printf("\nport opened :%s; rate: %d",port.portName().toStdString().data(),port.baudRate());
        return gotoConfig();


    }

    bool getIsConnected() const;
    bool gotoConfig()
    {
        int trycount=0;
        bool isSuccess = false;
        unsigned char req[] = {0xFA,0xFF,0x30,0x00,0xD1};
        unsigned char ansb[] = {0xFA,0xFF,0x31,0x00,0xD0};
        QByteArray ans((char*)ansb,5);
        while(trycount<5){
            trycount++;
            port.write((char*)req,5);
            port.flush();
            port.waitForReadyRead(100);
            QByteArray data = port.readAll();
            int res = data.compare(ans);
            if(res==0) isSuccess=true;
        }
        return isSuccess;
    }
    bool gotoMeasurement()
    {
        int trycount=0;
        bool isSuccess = false;
        unsigned char req[] = {0xFA,0xFF,0x10,0x00,0xF1};
        unsigned char ansb[] = {0xFA,0xFF,0x11,0x00,0xF0};
        QByteArray ans((char*)ansb,5);
        while(trycount<5){
            trycount++;
            port.write((char*)req,5);
            port.flush();
            port.waitForReadyRead(100);
            QByteArray data = port.readAll();
            int res = data.compare(ans);
            if(res==0) isSuccess=true;
        }
        return isSuccess;
    }
    IMUData getMeasurement() const;
private:
    IMUData measurement;
    bool isConnected;
    QSerialPort port;

};

#endif // IMU_DRIVER_H
