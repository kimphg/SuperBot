#ifndef CMAP_H
#define CMAP_H
#include <QtNetwork/QNetworkAccessManager>
#include <QtGui>
#include <QPixmap>
#include <QUrl>
#include <QFrame>
#include <QHash>
#include <QUdpSocket>
#define MAP_PATH_1 "D:/DATA/MapData/SEA"
#define MAP_PATH_2 "D:/DATA/MapData/OSM"
#define MAP_PATH_3 "D:/DATA/MapData/TR"

#define MAP_PATH_DoSau QString("C:/MAPDATA/navionics_sonarchart")   //SQL
#define MAP_PATH_VeTinh QString("C:/MAPDATA/SAT/7-12")  //IMG
#define MAP_PATH_VecTor QString("C:/MAPDATA/Vector")  //IMG
#define MAP_PATH_Thuong QString("C:/MAPDATA/NORMAL/7-11")  //IMG
#define MAP_PATH_Null QString("C:/MAPDATA/")  //IMG


#define DEG_RAD 57.295779513
#define radians(x) ((x)/57.295779513)
#define degrees(x) ((x)*57.295779513)
#include<QtSql/QSql>
#include<QtSql>
#include<QSqlDatabase>
#include<QSqlQuery>
#include<QFile>
#include<QLabel>
#include <QVBoxLayout>
#include <QGroupBox>
//#include "cmapwidget_global.h"
#include<QDebug>

#include<QtMath>
#include<QMessageBox>
#include<QApplication>
#include<QBrush>
#include<QColor>
//#include <QGeoCoordinate>

class QNetworkReply;
class QPainter;

typedef  struct
{
    float lat,lng, sog, cog;
    int lost, min, type, idLog, utcPos;
    QString mmsi,vsnm, Class;
}ShipDataJson;

typedef  struct
{
    ShipDataJson Ship;
    double Distance_With_Focus_Ship;
}ShipPoint;


struct PointDouble
{
    double x,y;
};
typedef  struct
{
    PointDouble pos;
    QString name;
    QString id;
    bool insideDisplay;
    bool highligted;
    bool selected;
    QString description;
    float cog,sog;
}MapItem;
enum mouseMode
{
    MouseNormal =       0,
    MouseDrag =         0x0008,
    MouseMeasuring =    0x0010,
    MouseELB =          0x0020,
    MouseVRM =          0x0040
};
struct PosStruct
{
    QPoint pos;
    int animation;
};
//typedef std::map<std::pair<int,int>, MapObj> MapDataText;
class CMapWidget: public QFrame
{
    Q_OBJECT
public:
    CMapWidget(QWidget *parent = 0);
    virtual ~CMapWidget();
    void setCenterPos(double lat, double lon);
    void setImgSize(int width, int height);
    void setPath_IMG(QString path);
    void setPath(QString path);
    void LoadText(QString path);
    QPixmap getImage(double scale);
    void ConvKmToWGS_precise(double x, double y, double *m_Long, double *m_Lat);
    void ConvKmToWGS(double x, double y, double *m_Long, double *m_Lat);
    void ConvWGSToKm(double *x, double *y, double m_Long, double m_Lat);
    int getScaleRatio();
    double getLat();
    double getLon();
    QUdpSocket *socket;

    void setMapItem(QString id, QString name, float lat, float lng, float sog, float cog);
    PointDouble ConvScrPointToWGS(int x, int y);
    bool checkInsideRect(PointDouble tl, PointDouble br, PointDouble p);
    void clearMapItem();
    void setPath();
    float getDepth(float lat,float lon);
    void requestTile(int x, int y, int z, int type);

    void RepaintMap();
    void cleanItems();

   bool IsDataMapSQL = true;
public slots:
    void timerEvent(QTimerEvent *event);
signals:
    void updated(const QRect &rect);
private slots:
    void LoadMap_IMG();
    void LoadMap_SQL();
//    void changeMapType(int typeId);
    void SetMapDir(QString dir);
    void processIncomingData();
protected:
    QRect tileRect(const QPoint &tp);
protected:
    bool event(QEvent *event);
    void paintEvent(QPaintEvent *event);
    void resizeEvent(QResizeEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);
    void hoverMove(QHoverEvent *event);
    void mouseClickEvent(QMouseEvent *event);
    void keyPressEvent(QKeyEvent *e);
private:
//    void OpenMIF(const char* fileName);
    QLabel *labelName ;
    QLabel *infoLabel ;
    QLabel *idLabel;
    QLabel *zlabel;
    QLabel *labelCoor;
    QGroupBox *infoBox;
    QVBoxLayout *leftLayout ;
    QVBoxLayout *infLayout;
    QSpacerItem * my_spacer2 ;
    QHBoxLayout *mainLayout;
    QSpacerItem * my_spacer ;
    QComboBox *mapSelector;
    QString idSelected;
    QString idHovered;
    PointDouble scrTLWSG,scrBRWSG;
    int dx=0,dy=0;
    QHash<QString, MapItem> mapItems;

    void render(QPainter *p, const QRect &rect);
    void invalidate();

    double getScaleKm();
    void UpdateImage();
    QPoint m_offset;
    QRect m_tilesRect;
    QPixmap m_emptyTile;
    QHash<QPoint, QPixmap> m_tilePixmaps;
    QUrl m_url;
    QString mPath,mPathraw;
    QPixmap *mapImage;//pkp
    int scaleMin,scaleMax;
    int mMapWidth;
    int mMapHeight;


//double metersPerPixel;
    void drawItem(QPainter *p, MapItem mapitem);
    void setMouseMode(mouseMode mode, bool isOn);
    void ConvWGS2Scr(double m_Long, double m_Lat);
    void ConvWGS2Scr(double *x, double *y, double m_Long, double m_Lat);
    void setupInfoPanel();
//    QImage getDataFromDB(QPoint grab);
    QImage getDataFromDB2(QPoint grab, int scale);
    PointDouble mousePoint;
    float mouseDepth;
    bool itemChanged=false;
    int mouseposx,mouseposy;

    void drawBackground(QPainter&);
    void drawScale(QPainter &pai);
    void drawScanning(QPainter& pai);
    void drawLock(QPainter&, PosStruct&);

public:
    std::vector<ShipPoint> nearby_ships;
    double toRadian(double degree);
    double calculateDistance(double lat1, double lon1, double lat2, double lon2);
    void Draw_enemy_ship(QPainter *p, float lat, float lng,float cog);
    void Draw_all_enemy(QPainter *p);

    int COUT_SHIP_DRAWED = 0, Count_Drawing = 0;
    float CalcDistanceKm(double Long, double Lat, double m_Long, double m_Lat);
    bool MousePressCheck = false;

     void Draw_Circle(QPainter* p);
     QPoint CenterOfCircle;
     QRect Rect;
     int m_r = 0;
     QTimer* animation_time = nullptr;
     int arc = 280;

     double mCenterLat;
     double mCenterLon;

     std::vector<PosStruct> *vec;
     void MoveBackCenterOfCircle();
     void dragMapToPosition(CMapWidget *widget, QPoint mapPoint, QPoint screenPoint);
      bool setScaleRatio(int scale);
      int mScale;
      int RulerFirstValue, RulerStep;

private slots:
    void timeOut();
};

#endif // CMAP_H
