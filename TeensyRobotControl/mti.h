
typedef struct 
{
    
}IMUData;
class IMU_driver 
{
public:
    IMU_driver(Stream& porti)
    {
      this->port = &porti;
      Connect();
    }
    bool Connect()
      {
        port.begin(921600);
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
    bool updateData()
    {
      //to be implemented
    }
    IMUData getMeasurement() 
    {
      return measurement;
      }
private:
    IMUData measurement;
    Stream& port; 

};
