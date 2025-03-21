#include <math.h>
#include <QtGui>
#include <QFileDialog>
//#include <QtNetwork>

#include "cmapwidget.h"
const double MaxDistance = 1000.0; //KM
const double EARTH_RADIUS = 6371.0; // Đường kính Trái đất (đơn vị: km)
ShipPoint Near_Ships[5];
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
//MapDataText mapText;

uint qHash(const QPoint& p)
{
    return p.x() * 17 ^ p.y();
}
// tile size in pixels
const int tdim = 256;
static QPen penYellow(QBrush(QColor(50,50,255 ,255)),1);
static QPen penEnemyShips(QBrush(QColor(50,50,255,255)),2);
static QPen penRed(QBrush(QColor(255,50,50 ,255)),2);
static QPen penDarkBlue(QBrush(Qt::darkBlue),2);
static QPen penGreen(QBrush(QColor(0, 255, 0, 255)),2);

static mouseMode mouse_mode = MouseNormal;
static int mMouseLastX = 0, mMouseLastY=0;

// xac dinh manh ban do tu dau ra dau vao
QPointF tileForCoordinate(qreal lat, qreal lng, int zoom)
{
    qreal zn = static_cast<qreal>(1 << zoom);
    qreal tx = (lng + 180.0) / 360.0;
    qreal ty = (1.0 - log(tan(lat * M_PI / 180.0) +
                          1.0 / cos(lat * M_PI / 180.0)) / M_PI) / 2.0;
    return QPointF(tx * zn, ty * zn);
}

//tra ve kinh do tu manh ban do
qreal longitudeFromTile(qreal tx, int zoom)
{
    qreal zn = static_cast<qreal>(1 << zoom);
    qreal lat = tx / zn * 360.0 - 180.0;
    return lat;
}

//tra ve vi do tu manh ban do
qreal latitudeFromTile(qreal ty, int zoom)
{
    qreal zn = static_cast<qreal>(1 << zoom);
    qreal n = M_PI - 2 * M_PI * ty / zn;
    qreal lng = 180.0 / M_PI * atan(0.5 * (exp(n) - exp(-n)));
    return lng;
}
QImage img;
void CMapWidget::mousePressEvent(QMouseEvent *event)
{
    MousePressCheck = true; //Đã press chuột để chọn tàu ta, bắt đầu truyền dữ liệu
    if(event->buttons() & Qt::LeftButton)
    {
        int posx = event->x();
        int posy = event->y();
        if(posx)mMouseLastX= posx;
        if(posy)mMouseLastY= posy;
        setMouseMode(MouseDrag,true);
        bool selectionOK=false;
        foreach (MapItem mapitem_, mapItems)
        {
            if(mapitem_.insideDisplay)
            {
                double itemx,itemy;
                ConvWGS2Scr(&itemx,&itemy,mapitem_.pos.x,mapitem_.pos.y);
                double dx = posx-itemx;
                double dy = posy-itemy;
                if(dx*dx+dy*dy<100)
                {
                    nearby_ships.clear();
                    idSelected = mapitem_.id;
//                                        setCenterPos(mapitem_.pos.y,mapitem_.pos.x);
//                    MoveBackCenterOfCircle();

                    selectionOK=true;
                    foreach (MapItem mapitem_1, mapItems)
                    {
                        float distance = CalcDistanceKm(mapitem_.pos.x,mapitem_.pos.y,mapitem_1.pos.x,mapitem_1.pos.y);
                        if(distance<100&&mapitem_1.id!=mapitem_.id)
                        {
                            ShipPoint newShip;
                            newShip.Ship.lat = mapitem_1.pos.y;
                            newShip.Ship.lng = mapitem_1.pos.x;
                            newShip.Ship.cog = mapitem_1.cog;
                            newShip.Distance_With_Focus_Ship = distance;
                            nearby_ships.push_back(newShip);
                        }
                    }
                    for (int m=0; m< int(nearby_ships.size());m++)
                    {
                        for (int n=m;n< int(nearby_ships.size());n++)
                        {
                            if(nearby_ships[m].Distance_With_Focus_Ship>nearby_ships[n].Distance_With_Focus_Ship)
                            {
                                ShipPoint tempship =   nearby_ships[m];
                                nearby_ships[m] = nearby_ships[n];
                                nearby_ships[n] = tempship;
                            }
                        }
                    }
                }
                if(nearby_ships.size()<5)
                    if(dx*dx+dy*dy<500)
                    {
                        idSelected = mapitem_.id;
//                         setCenterPos(mapitem_.pos.y,mapitem_.pos.x);
//                        MoveBackCenterOfCircle();

                        selectionOK=true;
                        foreach (MapItem mapitem_1, mapItems)
                        {
                            float distance = CalcDistanceKm(mapitem_.pos.x,mapitem_.pos.y,mapitem_1.pos.x,mapitem_1.pos.y);
                            if(distance<100&&mapitem_1.id!=mapitem_.id)
                            {
                                ShipPoint newShip;
                                newShip.Ship.lat = mapitem_1.pos.y;
                                newShip.Ship.lng = mapitem_1.pos.x;
                                newShip.Ship.cog = mapitem_1.cog;
                                newShip.Distance_With_Focus_Ship = distance;
                                nearby_ships.push_back(newShip);
                            }
                        }
                        for (int m=0; m<int(nearby_ships.size());m++)
                        {
                            for (int n=m;n<int(nearby_ships.size());n++)
                            {
                                if(nearby_ships[m].Distance_With_Focus_Ship>nearby_ships[n].Distance_With_Focus_Ship)
                                {
                                    ShipPoint tempship =   nearby_ships[m];
                                    nearby_ships[m] = nearby_ships[n];
                                    nearby_ships[n] = tempship;
                                }
                            }
                        }
                    }
            }
        }
        if(selectionOK)
        {
            //infoBox->setHidden(false);
        }

    }
}

void CMapWidget::dragMapToPosition(CMapWidget *widget, QPoint mapPoint, QPoint screenPoint)
{

    setMouseMode(MouseDrag,true);
    // Tạo sự kiện nhấn chuột tại điểm trên bản đồ
    QMouseEvent pressEvent(QMouseEvent::MouseButtonPress, mapPoint, Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
    QCoreApplication::sendEvent(widget, &pressEvent);

    // Lấy tọa độ cũ của chuột
    QPoint oldPos = QCursor::pos();

    // Di chuyển chuột đến điểm khác trên màn hình
    QCursor::setPos(screenPoint);

    // Tính toán khoảng cách di chuyển của chuột
    int dx = screenPoint.x() - mapPoint.x();
    int dy = screenPoint.y() - mapPoint.y();

    // Tạo sự kiện di chuyển chuột tại điểm mới
    QMouseEvent moveEvent(QMouseEvent::MouseMove, screenPoint, mapPoint + QPoint(dx, dy), Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
    QCoreApplication::sendEvent(widget, &moveEvent);

    // Tạo sự kiện thả chuột
    QMouseEvent releaseEvent(QMouseEvent::MouseButtonRelease, screenPoint, Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
    QCoreApplication::sendEvent(widget, &releaseEvent);

    // Đặt lại tọa độ chuột cũ
    QCursor::setPos(oldPos);

    setMouseMode(MouseDrag,false);
}

void CMapWidget::keyPressEvent(QKeyEvent *e)
{
    switch(e->key())
    {
    case Qt::Key_E:
        QApplication::quit();
        break;
    default:
        QWidget::keyPressEvent(e);
    }
}

PointDouble CMapWidget::ConvScrPointToWGS(int x,int y)
{
    PointDouble output;
    output.y  = mCenterLat -  ((y-height()/2)/getScaleKm())/(111.132954);
    double refLat = (mCenterLat +(output.y))*0.00872664625997;
    output.x = (x-width()/2)/getScaleKm()/(111.31949079327357*cos(refLat))+ mCenterLon;
    return output;
}
void CMapWidget::mouseReleaseEvent(QMouseEvent *event)
{
    Q_UNUSED(event);
    setMouseMode(MouseDrag,false);
    scrTLWSG = ConvScrPointToWGS(0,0);
    scrBRWSG = ConvScrPointToWGS(width(),height());
    foreach (MapItem mapitem_, mapItems) {
        mapitem_.insideDisplay = checkInsideRect(scrTLWSG,scrBRWSG,mapitem_.pos);

    }
    repaint();
}
bool CMapWidget::checkInsideRect(PointDouble tl,PointDouble br, PointDouble p)
{
    if(p.x>tl.x&&p.x<br.x)
        if(p.y<tl.y&&p.y>br.y)
            return true;
    return false;

}
void CMapWidget::setMouseMode(mouseMode mode,bool isOn)
{
    if(isOn)
    {
        mouse_mode = static_cast<mouseMode>(mouse_mode|mode) ;
    }
    else
    {
        mouse_mode = static_cast<mouseMode>(mouse_mode-(mode&mouse_mode));
    }
}
void CMapWidget::mouseMoveEvent(QMouseEvent *event)
{
   /* if( (mouse_mode&MouseDrag) && (event->buttons() & Qt::LeftButton) )
    {
        {
            dx = mMouseLastX-event->x();
            dy = mMouseLastY-event->y();
            float dxKM = dx/getScaleKm();
            float dyKM = -dy/getScaleKm();
            double newLat,newLon;
            ConvKmToWGS(dxKM,dyKM,&newLon,&newLat);
            setCenterPos(newLat,newLon);
//                         MoveBackCenterOfCircle();
            mMouseLastX=event->x();
            mMouseLastY=event->y();

        }
        repaint();
    }*/

}
void CMapWidget::MoveBackCenterOfCircle()
{
    dx = mMouseLastX-CenterOfCircle.x();
    dy = mMouseLastY-CenterOfCircle.y();

    float dxKM = dx/getScaleKm();
    float dyKM = -dy/getScaleKm();

    double newLat,newLon;
    ConvKmToWGS(dxKM,dyKM,&newLon,&newLat);

    setCenterPos(newLat,newLon);
    mMouseLastX = CenterOfCircle.x();
    mMouseLastY = CenterOfCircle.y();

    repaint();
}
CMapWidget::CMapWidget(QWidget *parent): QFrame(parent)
{
    mapImage = 0;
    scaleMin = 4;
    scaleMax = 15;
//    mCenterLat = 21;
//    mCenterLon = 105;
    this->setScaleRatio(9);
    this->RulerFirstValue = 45;

    //khoi tao mot vung pixmap co kich thuoc 256 X 256 va boi day no voi mau lightGray
    m_emptyTile = QPixmap(tdim, tdim);
//    m_emptyTile.fill(Qt::gray);
    m_emptyTile.fill(Qt::black);


    this->setImgSize(mMapWidth,mMapHeight);
    socket = new QUdpSocket(this);
    int port;
    for(port=1650; port<1660;port++)
        if(socket->bind(port))
        {
            break;
        }
    printf("\nService started in port:%d",port);

    setMouseTracking(true);
    setAttribute(Qt::WA_Hover);
    startTimer(100);

    //    mPath = "C:/MAP/navionics_sonarchart/navionics_sonarchart";
    animation_time = new QTimer(this);
    connect(animation_time, &QTimer::timeout, this, &CMapWidget::timeOut);
    animation_time->setInterval(40);
    animation_time->start();

    vec = new std::vector<PosStruct>;
    vec->reserve(10);
}

void CMapWidget::timeOut()
{
    arc -= 3;
    if (arc == 0)
        arc = 360;

    if (rand() % 50 == 0)
    {
        PosStruct tmp;
        int r = rand() % m_r;
        tmp.pos.setX(r * cos(arc * 3.1415926 / 180) + this->width() / 2);
        tmp.pos.setY(-r * sin(arc * 3.1415926 / 180) + this->height() / 2);
        tmp.animation = width() > height() ? height() / 2 : width() / 2;
        vec->push_back(tmp);
    }

    this->update();
}

double CMapWidget::toRadian(double degree)
{
    return degree*M_PI/180.0;
}

double CMapWidget::calculateDistance(double lat1, double lon1, double lat2, double lon2)
{
    double dLat = toRadian(lat2 - lat1);
    double dLon = toRadian(lon2 - lon1);

    // Áp dụng công thức haversine
    double a = qSin(dLat / 2) * qSin(dLat / 2) +
            qCos(toRadian(lat1)) * qCos(toRadian(lat2)) *
            qSin(dLon / 2) * qSin(dLon / 2);
    double c = 2 * qAtan2(qSqrt(a), qSqrt(1 - a));
    double distance = EARTH_RADIUS * c;

    return distance;
}

void CMapWidget::timerEvent(QTimerEvent *event)
{
    if(itemChanged)
    {
        repaint();
        itemChanged=false;
    }
}
QVector<int> BAtoIA(QByteArray dataBuff)
{
    int len = dataBuff.size()/4;
    QVector<int> output(len);
    char* beginPt = (char*)(&output[0]);
    for (int i=0;i<len;i++)
    {
        memcpy(beginPt + i*4, dataBuff.data()+i*4, 4);

    }
    return output;
}
QByteArray IAtoBA(QVector<int> dataBuff)
{
    int len = dataBuff.size()*4;
    QByteArray output;
    output.resize(len);
    char* beginPt = (char*)(output.data());
    for (int i=0;i<len;i++)
    {
        memcpy(beginPt + i, (char*)&dataBuff[0] + i, 1);
    }
    return output;
}
void CMapWidget::processIncomingData()
{
    while(socket->hasPendingDatagrams())
    {
        QByteArray dataBuff;
        dataBuff.resize(socket->pendingDatagramSize());
        socket->readDatagram(dataBuff.data(),1000);

        //        QVector<int> data = BAtoIA(dataBuff);
        ////      int requesttype = data[0];
        ////      int maptype = data[1];
        ////      int mz = data[2];
        //        int my = data[3];
        //        int mx = data[4];
        //        QImage img;
        //        img.loadFromData((uchar*)dataBuff.data()+20,dataBuff.size()-20);
        //        QPoint grab(mx,my);
        //        m_tilePixmaps[grab]=QPixmap::fromImage( img);
    }
}
void CMapWidget::setupInfoPanel()
{
    //    infoLabel->setText("..");
    //    idLabel->setText("..");
    //    infoBox->setStyleSheet("color:white; background-color: rgb( 60, 120, 60);");

    //    infoBox->setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred));
    //    infoBox->setMinimumWidth(300);

    //infoBox->setSizePolicy(QSizePolicy ::Preferred , QSizePolicy ::Preferred );

    /*infLayout->addWidget(idLabel);

    infLayout->addWidget(labelName);
    infLayout->addWidget(labelCoor);
    infLayout->addWidget(infoLabel);

    infoBox->setLayout(infLayout);
    leftLayout->addWidget(infoBox);

    leftLayout->addSpacerItem(my_spacer2);*/

    //    QDir directory(MAP_PATH);
    //    QStringList map_types = directory.entryList(QStringList(),QDir::Dirs);
    /* foreach(QString dirname, map_types) {
        if(dirname=="."||dirname=="..")continue;
        mapSelector->addItem(dirname);
    }

    leftLayout->addWidget(mapSelector);
    leftLayout->addWidget(zlabel);
    mainLayout->addLayout(leftLayout);
    mainLayout->addSpacerItem(my_spacer);


    connect(mapSelector,SIGNAL(currentIndexChanged(QString)),this,SLOT(SetMapDir(QString)));
    mapSelector->setCurrentIndex(1);
    this->setLayout(mainLayout);*/
    //    infoBox->setVisible(false);
}
void CMapWidget::resizeEvent(QResizeEvent *event)
{
    Q_UNUSED(event);
    mMapWidth = this->geometry().width();
    mMapHeight = this->geometry().height();
    this->setImgSize(mMapWidth,mMapHeight);
    repaint();
}

float CMapWidget::getDepth(float lat,float lon)
{
    QPointF ct;
    int cScale;
    for(cScale=15;cScale>6;cScale--)
    {
        ct = tileForCoordinate(lat, lon, cScale);
        img = getDataFromDB2(QPoint(ct.x(),ct.y()),cScale);
        if(img.width()==256)break;
    }
    QRgb color = img.pixel(256*(ct.x()-int(ct.x())),256*(ct.y()-int(ct.y())));
    QColor qc = QColor::fromRgb(color);
    qc = qc.toHsv();
    int h,s,v;
    qc.getHsv(&h,&s,&v);
    {
        if((abs(h)+abs(s)<3)&&(v>200))return 11;
        if(abs(h-200)>4)return -1;
        if(abs(s-222)<4)return 0.5;
        if(abs(s-181)<4)return 1;
        if(abs(s-165)<4)return 1.5;
        if(abs(s-148)<4)return 2;
        if(abs(s-132)<4)return 3;
        if(abs(s-123)<4)return 3.5;
        if(abs(s-115)<4)return 4;
        if(abs(s-107)<4)return 4.5;
        if(abs(s-99)<4)return 5;
        if(abs(s-90)<4)return 6;
        if(abs(s-82)<4)return 7;
        if(abs(s-74)<4)return 8;
        if(abs(s-66)<4)return 9;
        if(abs(s-57)<4)return 10;
    }
    printf("hsv:%d  %d  %d",h,s,v);
    return -1;
}

void CMapWidget::hoverMove(QHoverEvent *event)
{
    mouseposx = event->pos().x();
    mouseposy = event->pos().y();
    mousePoint =  ConvScrPointToWGS(mouseposx,mouseposy);
    //tinh toan do sau
    {
        float depth = getDepth(mousePoint.y,mousePoint.x);
        if(depth<=0)
        {
            int dist=4;
            float depthpoint[8];
            PointDouble movedmousePoint;
            movedmousePoint = ConvScrPointToWGS(mouseposx+dist,mouseposy-dist);    depthpoint[0] = getDepth(movedmousePoint.y,movedmousePoint.x);
            movedmousePoint = ConvScrPointToWGS(mouseposx+dist,mouseposy+dist);    depthpoint[1] = getDepth(movedmousePoint.y,movedmousePoint.x);
            movedmousePoint = ConvScrPointToWGS(mouseposx-dist,mouseposy-dist);    depthpoint[2] = getDepth(movedmousePoint.y,movedmousePoint.x);
            movedmousePoint = ConvScrPointToWGS(mouseposx-dist,mouseposy+dist);    depthpoint[3] = getDepth(movedmousePoint.y,movedmousePoint.x);
            movedmousePoint = ConvScrPointToWGS(mouseposx+dist,mouseposy);      depthpoint[4] = getDepth(movedmousePoint.y,movedmousePoint.x);
            movedmousePoint = ConvScrPointToWGS(mouseposx,mouseposy+dist);      depthpoint[5] = getDepth(movedmousePoint.y,movedmousePoint.x);
            movedmousePoint = ConvScrPointToWGS(mouseposx-dist,mouseposy);      depthpoint[6] = getDepth(movedmousePoint.y,movedmousePoint.x);
            movedmousePoint = ConvScrPointToWGS(mouseposx,mouseposy-dist);      depthpoint[7] = getDepth(movedmousePoint.y,movedmousePoint.x);
            int vcount[] = {0,0,0,0,0,0,0,0};
            for (int i=0;i<8;i++)
                for (int j=i;j<8;j++)
                {
                    if(depthpoint[i]>0)
                        if(depthpoint[i]==depthpoint[j])
                        {
                            vcount[i]++;
                            vcount[j]++;
                        }
                }

            int vmax=0;
            float depthmax;
            for (int i=0;i<8;i++)
            {
                if(vmax<vcount[i]){vmax=vcount[i];depthmax=depthpoint[i];}
            }
            depth =  depthmax;
        }

        mouseDepth=depth;
    }
    repaint();
}
bool CMapWidget::event(QEvent *event)
{
    switch(event->type())
    {
    case QEvent::Type::HoverMove:
        hoverMove(static_cast<QHoverEvent*>(event));
        return true;
        break;
    default:
        break;
    }
    return QWidget::event(event);
}

void CMapWidget::Draw_Circle(QPainter *p)
{

    p->setRenderHint(QPainter::Antialiasing, true);

    int centerX = width() / 2;
    int centerY = height() / 2;
    int radius = qMin(centerX, centerY);

    p->setPen(QPen(Qt::black, 1));
    //    this->adjustSize();
    //    painter.setBrush(Qt::red);


    p->drawEllipse(centerX - radius, centerY - radius,radius*2, radius*2);


    m_r = radius;
    CenterOfCircle.setX(centerX);
    CenterOfCircle.setY(centerY);
    Rect.setRect(centerX - radius, centerY - radius,radius*2, radius*2);

    radius = radius + 400;  //Đường tròn để tô kín bản đồ
    p->setPen(QPen(Qt::black, 800));
    p->drawEllipse(centerX - radius, centerY - radius,radius*2, radius*2);
}

//----------------------------------------------
void CMapWidget::drawBackground(QPainter& pai)
{
    //   pai.setPen(QPen(Qt::green, 3.0, Qt::DotLine));
    pai.setPen(QPen(QColor(5,236,244),2.0,Qt::DotLine));

    pai.drawEllipse(Rect.center(), m_r / 5 * 1, m_r / 5 * 1);
    pai.drawEllipse(Rect.center(), m_r / 5 * 2, m_r / 5 * 2);
    pai.drawEllipse(Rect.center(), m_r / 5 * 3, m_r / 5 * 3);
    pai.drawEllipse(Rect.center(), m_r / 5 * 4, m_r / 5 * 4);

    //   pai.setPen(QPen(Qt::green, 3.0, Qt::SolidLine));
    pai.setPen(QPen(QColor(5,236,244),3.0,Qt::SolidLine));
    pai.drawEllipse(Rect.center(), m_r, m_r);
    //    pai.setPen(QPen(Qt::green, 3.0, Qt::DotLine));
    pai.setPen(QPen(QColor(5,236,244),3.0, Qt::DotLine));
    pai.drawEllipse(Rect.center(), m_r - 10, m_r - 10);


    //   pai.setPen(QPen(Qt::green, 0.2, Qt::SolidLine));
    pai.setPen(QPen(QColor(5,236,244),0.2,Qt::SolidLine));
    for (int i = 0; i < 10; i++)
    {
        int w = sqrt(pow(m_r, 2) - pow(m_r / 10 * i, 2));
        int h = m_r / 10 * i;
        pai.drawLine(QPoint(this->width() / 2 - w, this->height() / 2 - h), QPoint(this->width() / 2 + w, this->height() / 2 - h));
        pai.drawLine(QPoint(this->width() / 2 - w, this->height() / 2 + h), QPoint(this->width() / 2 + w, this->height() / 2 + h));

        h = sqrt(pow(m_r, 2) - pow(m_r / 10 * i, 2));
        w = m_r / 10 * i;
        pai.drawLine(QPoint(this->width() / 2 - w, this->height() / 2 - h), QPoint(this->width() / 2 - w, this->height() / 2 + h));
        pai.drawLine(QPoint(this->width() / 2 + w, this->height() / 2 - h), QPoint(this->width() / 2 + w, this->height() / 2 + h));
    }


    int value = RulerFirstValue;
    for (int i = 1; i < 5; i++)
    {
        pai.drawText(QPoint(this->width() / 2 - 15, this->height() / 2 - m_r / 5 * i - 2), QString::number(value*1.852) + " KM");
        pai.drawText(QPoint(this->width() / 2 - 15, this->height() / 2 + m_r / 5 * i + 14), QString::number(value*1.852) + " KM");
        value += RulerFirstValue;
    }
}
void CMapWidget::drawScale(QPainter &pai)
{
    pai.save();
    //   pai.setPen(QPen(Qt::green, 2.0, Qt::SolidLine));
    pai.setPen(QPen(QColor(5,236,244),2.0,Qt::SolidLine));
    //  pai.setPen(Qt::green);
    //  pai.setPen(QColor(65,132,159));
    pai.translate(this->width() / 2, this->height() / 2);
    int value = 15;

    for (int i = 0; i < 24; i++)
    {
        pai.rotate(360 / 24);
        pai.drawText(QPoint(0, -m_r - 4), value == 360 ? "0" : QString::number(value));
        value += 15;
    }

    for (int i = 0; i < 120; i++)
    {
        pai.rotate(3);
        pai.drawLine(QPoint(0, m_r - 10), QPoint(0, m_r));
    }

    pai.restore();
    //    qDebug()<<"line/n";
}
void CMapWidget::drawScanning(QPainter& pai)
{
    // ��ɨ����
    QConicalGradient conical(this->width() / 2, this->height() / 2, arc);
    //  QColor color1("#00FF00");
    // ������ɫ ��ɫ 1C86EE //
    QColor color1("#AFEEEE");
    color1.setAlpha(200);
    QColor color2("#000000");
    color2.setAlpha(0);

    conical.setColorAt(0, color1);
    conical.setColorAt(0.13, color2);

    pai.setPen(Qt::NoPen);
    pai.setBrush(conical);
    pai.drawPie(QRect(QPoint(this->width() / 2 - m_r, this->height() / 2 - m_r), QSize(m_r * 2, m_r * 2)), arc * 16, 60 * 16);
}
void CMapWidget::drawLock(QPainter& pai, PosStruct &ps)
{
    //	pai.setPen(QPen(Qt::yellow, 2, Qt::SolidLine));
    pai.setPen(QPen(QColor(5,236,244,200), 2.0, Qt::SolidLine));

    int r = m_r / 8;
    if (ps.animation - 40 > r)
    {
        ps.animation -= 40;
        r = ps.animation;
    }
}


//-----------------------------------------------
void CMapWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
    RepaintMap();
    QPainter p(this);
    if(mapImage)
    {
        p.drawPixmap(mapImage->rect(),
                     *mapImage);
    }
    p.drawRect(mapImage->rect());
//    foreach (MapItem mapitem_, mapItems) {
//        if(mapitem_.insideDisplay)
//            drawItem(&p,mapitem_);
//    }
    int n=0;
    p.drawText(mouseposx,mouseposy,QString::number(mousePoint.x)+"N"+QString::number(mousePoint.y)+"E\n"+QString::number(mouseDepth)+"m");

//    Draw_Circle(&p);

    //----------set background
    p.setRenderHint(QPainter::Antialiasing);
    p.fillRect(rect(), QColor(5,236,244,10));

    for (auto &item:*vec) // c++11--new method
        drawLock(p, item);

    drawBackground(p);
    drawScale(p);
//    drawScanning(p);
}
void CMapWidget::wheelEvent(QWheelEvent *event)
{
 /*   if(event->delta()>0)
        setScaleRatio(mScale+1);
    else setScaleRatio(mScale-1);
    //    zlabel->setText(QString::number(mScale));
    repaint();*/
}

void CMapWidget::Draw_enemy_ship(QPainter *p, float lat, float lng,float cog)
{
    float radHeading = cog;
    float shipSize = 0.7;
    double itemx,itemy;
    ConvWGS2Scr(&itemx,&itemy,lng,lat);

    //    int x = itemx;//(itemx*getScaleKm())+width()/2 ;
    //    int y = itemy;//height()/2+(itemy*getScaleKm());

    QPointF centerofCirle (itemx,itemy);
    p->setPen(penEnemyShips);


    //    QMessageBox::information(this, "report", "Draw enemy ship");

    //Vẽ Ký hiệu tàu
    QPoint p1(itemx+shipSize*30*sin(radians(radHeading)),
              itemy-shipSize*30*cos(radians(radHeading)));
    QPoint p2(itemx+shipSize*15*sin(radians(radHeading+45)),
              itemy-shipSize*15*cos(radians(radHeading+45)));
    QPoint p3(itemx+shipSize*25*sin(radians(radHeading+155)),
              itemy-shipSize*25*cos(radians(radHeading+155)));
    QPoint p4(itemx+shipSize*25*sin(radians(radHeading-155)),
              itemy-shipSize*25*cos(radians(radHeading-155)));
    QPoint p5(itemx+shipSize*15*sin(radians(radHeading-45)),
              itemy-shipSize*15*cos(radians(radHeading-45)));
    QPoint p0(itemx+shipSize*50*sin(radians(radHeading)),
              itemy-shipSize*50*cos(radians(radHeading)));
    //    p->drawLine(p1,mBorderPoint1);
    p->drawLine(p1,p2);
    p->drawLine(p2,p3);
    p->drawLine(p3,p4);
    p->drawLine(p4,p5);
    p->drawLine(p5,p1);
    p->drawLine(p1,p0);

}
void CMapWidget::Draw_all_enemy(QPainter *p)
{

    p->setPen(penYellow);
    for (int i = 0 ; i < 5 ; ++i)
    {
        Draw_enemy_ship(p, Near_Ships[0].Ship.lat, Near_Ships[0].Ship.lng, Near_Ships[0].Ship.cog);
        //        qDebug() << "======++++=====DRAW ENEMY SHIP ";/// << i+1 << "th\n";
    }
}

int COUNT = 0;
bool clicked_check = false;
bool drawIte_Check = false;
/*void CMapWidget::drawItem(QPainter* p, MapItem mapitem)
{
    drawIte_Check = true;
    float radHeading = mapitem.cog;
    float shipSize = 0.5;
    double itemx,itemy;
    ConvWGS2Scr(&itemx,&itemy,mapitem.pos.x,mapitem.pos.y);

    int x = itemx;//(itemx*getScaleKm())+width()/2 ;
    int y = itemy;//height()/2+(itemy*getScaleKm());
    //    p->drawRect(x-10,y-10,20,20);
qDebug() << "****************************************************count_drawing: " << this->Count_Drawing << "\n";
    if(mapitem.id==idSelected)   ///                        NẾU LÀ TÀU ĐƯỢC CLICK CHỌN
    {
//          QMessageBox::information(this, "report", "CLICKED");
        clicked_check = true;

        //        float px=width()/2.0,py=height()/2.0;
        QPointF centerofCirle (itemx,itemy);
        p->setPen(penRed);
        for (int i=1;i<6;i++)  ///VẼ CÁC ĐƯỜNG TRÒN
        {
            float radius = 18.52*i*getScaleKm();
            p->drawEllipse(centerofCirle,radius,radius);
        }

        //Vẽ Ký hiệu tàu
        QPoint p1(itemx+shipSize*30*sin(radians(radHeading)),
                  itemy-shipSize*30*cos(radians(radHeading)));
        QPoint p2(itemx+shipSize*15*sin(radians(radHeading+45)),
                  itemy-shipSize*15*cos(radians(radHeading+45)));
        QPoint p3(itemx+shipSize*25*sin(radians(radHeading+155)),
                  itemy-shipSize*25*cos(radians(radHeading+155)));
        QPoint p4(itemx+shipSize*25*sin(radians(radHeading-155)),
                  itemy-shipSize*25*cos(radians(radHeading-155)));
        QPoint p5(itemx+shipSize*15*sin(radians(radHeading-45)),
                  itemy-shipSize*15*cos(radians(radHeading-45)));
        QPoint p0(itemx+shipSize*50*sin(radians(radHeading)),
                  itemy-shipSize*50*cos(radians(radHeading)));
        //    p->drawLine(p1,mBorderPoint1);
        p->drawLine(p1,p2);
        p->drawLine(p2,p3);
        p->drawLine(p3,p4);
        p->drawLine(p4,p5);
        p->drawLine(p5,p1);
        p->drawLine(p1,p0);

        //THỬ DUYỆT XEM HẾT ĐƯỢC CÁC TÀU ĐÃ VẼ KHÔNG
        QHashIterator<QString, MapItem> iter(mapItems);
        while (iter.hasNext())
        {
            iter.next();
            float lat, lng;
            lat = iter.value().pos.y;
            lng = iter.value().pos.x;
            QString id = iter.value().id;
            //            qDebug() << "lat:" << lat << "Lng:" << lng << "\n";

            //TÍNH TOÁN CẬP NHẬT 5 TÀU GẦN TÀU TA NHẤT
            if(mapitem.id != id) //KHÔNG PHẢI LÀ TÀU TA THÌ TÍNH KHOẢNG CÁCH
            {
                double DISTANCE = calculateDistance(mapitem.pos.y,mapitem.pos.x,lat, lng);
                for(int i = 0 ; i < 5 ; i++)
                {
                    if(DISTANCE < Near_Ships[i].Distance_With_Focus_Ship)
                    {
                        Near_Ships[i].Ship.lat = mapitem.pos.y;
                        Near_Ships[i].Ship.lng = mapitem.pos.x;
                        Near_Ships[i].Ship.cog = mapitem.cog;
                        qDebug() <<"i:" << i << "lat:" << mapitem.pos.y << "Lng:" << mapitem.pos.x << "\n";
                        //.vv... VÀ CÁC TRƯỜNG KHÁC NỮA
                        Near_Ships[i].Distance_With_Focus_Ship = DISTANCE;
                        break;
                    }
                }
            }
        }
        Count_Drawing++;
    }
    else
    {
        //        p->setPen(penYellow);
        p->drawEllipse(x, y, 6, 6);

        p->setPen(penYellow);  //penYellow  NHƯNG MÀ LÀ MÀU TÍM À NHA  ^^!!
        p->drawEllipse(x+2, y+2, 2, 2);

        Count_Drawing++;
    }

    //    p->setBackground(Qt::white) ;
    //    QFont font("times", 24);
    //    QFontMetrics fm(font);
    //    int pixelsWide = fm.size(Qt::TextSingleLine ,mapitem.id).width()/2.8;
    //    int pixelsHigh = fm.height()/2.8;
    //    p->setBrush(Qt::white);
    //    p->setPen(Qt::NoPen);
    //    p->drawRect(x-12,y+27-pixelsHigh,pixelsWide+1,pixelsHigh);
    //    p->setPen(penDarkBlue);
    //    p->setBrush(Qt::NoBrush);
    //    p->drawRect(x-8,y-8,16,16);
    //    p->drawText(x-10,y+25,mapitem.id);

    //KHANG ĐẸP TRAI-- VẼ CHO CÁC ĐỐI TƯỢNG VÀ ĐỐI ĐƯỢNG ĐƯỢC CHỌN (TÀU TA)
    if(Count_Drawing == this->COUT_SHIP_DRAWED && clicked_check == true)
    {
        drawIte_Check = false;
        Draw_all_enemy();
    }
    else if(Count_Drawing == COUT_SHIP_DRAWED)
    {
        drawIte_Check = false;
        Count_Drawing = 0;
    }
}
*/
void CMapWidget::drawItem(QPainter* p, MapItem mapitem)
{
    drawIte_Check = true;
    float radHeading = mapitem.cog;
    float shipSize = 0.8;
    double itemx,itemy;
    ConvWGS2Scr(&itemx,&itemy,mapitem.pos.x,mapitem.pos.y);

    int x = itemx;//(itemx*getScaleKm())+width()/2 ;
    int y = itemy;//height()/2+(itemy*getScaleKm());
    //    p->drawRect(x-10,y-10,20,20);

    if(mapitem.name=="Tau ta")   ///                        NẾU LÀ TÀU ĐƯỢC CLICK CHỌN
    {
        //          QMessageBox::information(this, "report", "CLICKED");
        clicked_check = true;

        //        float px=width()/2.0,py=height()/2.0;

        QPointF centerofCirle (itemx,itemy);

        /*QPoint point;
        point.setX(itemx);
        point.setY(itemy);
        setCenterPos(mapitem.pos.x,mapitem.pos.y);*/
        //        MoveBackCenterOfCircle(&point);
        //        dragMapToPosition(this, point, CenterOfCircle);

        p->setPen(penRed);
        for (int i=1;i<2;i++)  ///VẼ CÁC ĐƯỜNG TRÒN
        {
            float radius = 18.52*i*getScaleKm();
            p->drawEllipse(centerofCirle,radius,radius);
        }

        QBrush greyBrush(QColor(212,175,55));

        //        greyBrush.setColour(&QColor(120,60,55));

        //Vẽ Ký hiệu tàu
        QPoint p1(itemx+shipSize*30*sin(radians(radHeading)),
                  itemy-shipSize*30*cos(radians(radHeading)));
        QPoint p2(itemx+shipSize*15*sin(radians(radHeading+45)),
                  itemy-shipSize*15*cos(radians(radHeading+45)));
        QPoint p3(itemx+shipSize*25*sin(radians(radHeading+155)),
                  itemy-shipSize*25*cos(radians(radHeading+155)));
        QPoint p4(itemx+shipSize*25*sin(radians(radHeading-155)),
                  itemy-shipSize*25*cos(radians(radHeading-155)));
        QPoint p5(itemx+shipSize*15*sin(radians(radHeading-45)),
                  itemy-shipSize*15*cos(radians(radHeading-45)));
        QPoint p0(itemx+shipSize*50*sin(radians(radHeading)),
                  itemy-shipSize*50*cos(radians(radHeading)));
        //    p->drawLine(p1,mBorderPoint1);
        p->drawLine(p1,p2);
        p->drawLine(p2,p3);
        p->drawLine(p3,p4);
        p->drawLine(p4,p5);
        p->drawLine(p5,p1);
        p->drawLine(p1,p0);
        //        p->setBrush(greyBrush);
        //QCursor::setPos(CenterOfCircle );
    }
    else
    {
        //        p->setPen(penYellow);
       /* p->drawEllipse(x, y, 6, 6);
        p->setPen(penYellow);  //penYellow  NHƯNG MÀ LÀ MÀU TÍM À NHA  ^^!!
        p->drawEllipse(x+2, y+2, 2, 2);*/

        Draw_enemy_ship(p, mapitem.pos.y, mapitem.pos.x, mapitem.cog);

    }

    //KHANG ĐẸP TRAI-- VẼ CHO CÁC ĐỐI TƯỢNG VÀ ĐỐI ĐƯỢNG ĐƯỢC CHỌN (TÀU TA)
    if(Count_Drawing == this->COUT_SHIP_DRAWED && clicked_check == true)
    {
        drawIte_Check = false;
        Draw_all_enemy(p);
    }
    else if(Count_Drawing == COUT_SHIP_DRAWED)
    {
        drawIte_Check = false;
        Count_Drawing = 0;
    }
}

void CMapWidget::mouseClickEvent(QMouseEvent *event)
{
    Q_UNUSED(event);
}
void CMapWidget::clearMapItem()
{
    mapItems.clear();
    repaint();
}
void CMapWidget::setMapItem(QString id, QString name, float lat, float lng, float sog = 0, float cog=0)
{
    MapItem data;

    data.id = id;
    data.name = name;
    data.pos.y = lat;
    data.pos.x = lng;

    data.cog = cog;
    data.sog = sog;
    mapItems.insert(id,data);
    itemChanged = true;
}

CMapWidget::~CMapWidget()
{

}

int CMapWidget::getScaleRatio()
{
    return mScale;
}

void CMapWidget::RepaintMap()
{
    invalidate();
    UpdateImage();

}
void CMapWidget::cleanItems()
{
    mapItems.clear();
}
void CMapWidget::setPath_IMG(QString path)
{
    printf("\nLoad map path :");
    printf(path.toStdString().data());
    //qDebug() << QImageReader::supportedImageFormats ();
    mPath = path + "/%1/%2/%3";
    mPathraw = path + "/gs_%1_%2_%3";
    //    scaleMin = 4;
    //    scaleMax = 12;
    for(int i=1;i<20;i++)
    {
        QString str = path+"/"+QString::number(i);
        if(QDir(str).exists())
        {
            if(scaleMin>i)scaleMin = i;
            if(scaleMax<i)scaleMax = i;
        }
    }
    m_tilePixmaps.clear();
}
void CMapWidget::setPath(QString path)
{
    printf("\nLoad map path :");
    printf("%s",path.toStdString().data());
    //qDebug() << QImageReader::supportedImageFormats ();
    mPath = path;
    m_tilePixmaps.clear();
    invalidate();
    repaint();

}
void CMapWidget::setPath()
{
    QString fileNames = QFileDialog::getExistingDirectory(this);
    mPath = fileNames;
    m_tilePixmaps.clear();
    invalidate();

}
void CMapWidget::LoadText(QString path)
{
    path="";
    //    QFile mifFile(path);
    //    if(!mifFile.exists())return;
    //    if (!mifFile.open(QIODevice::ReadOnly | QIODevice::Text))
    //    {
    //        return ;
    //    }
    //    QTextStream in(&mifFile);
    //    in.setCodec("UTF-8");
    //    for(;;)
    //    {
    //        if (in.atEnd()) break;
    //        QString line = in.readLine();
    //        if(line=="Text")
    //        {
    //            QString text(in.readLine());
    //            QStringList latlon = (in.readLine().split(" "));
    //            QString font(in.readLine());
    //            if(font.contains("Tahoma"))
    //            {
    //                if(latlon.size()<4)continue;
    //                double lat =(latlon.at(4).toDouble()+latlon.at(6).toDouble())/2.0;
    //                double lon =(latlon.at(5).toDouble()+latlon.at(7).toDouble())/2.0;
    //                mapText.insert(std::make_pair(std::make_pair(int(lat),int(lon)),text));
    //            }
    //        }
    //    }
    //    mifFile.close();

}

void CMapWidget::setCenterPos(double lat, double lon)
{
    mCenterLat = lat;
    mCenterLon = lon;
    RepaintMap();
    return;
}

bool CMapWidget::setScaleRatio(int scale)
{
    mScale = scale;
    if(mScale>scaleMax){mScale = scaleMax;return false;}
    if(mScale<scaleMin){mScale = scaleMin;return false;}

    return true;

}

void CMapWidget::setImgSize(int width, int height)
{
    if(mapImage!=0)delete mapImage;

    mapImage = new QPixmap(width,height);
}

//ham lay ti le m tren pixel
double CMapWidget::getScaleKm()
{
    //The distance represented by one pixel (S) is given by
    //S=C*cos(y)/2^(z+8)
    //where...
    //C is the (equatorial) circumference of the Earth
    //z is the zoom level
    //y is the latitude of where you're interested in the scale.

    //double metersPerPixel = 156543.03392 * cos(mCenterLat * M_PI / 180) / pow(2, mScale);
    double scale = (double)(1<<mScale)/156.54303392 / cos(mCenterLat * M_PI / 180.0);
    return  scale;

}

//ham kien tao lai cac thong so khi thay doi
void CMapWidget::invalidate()
{
    if (mMapWidth <= 0 || mMapHeight <= 0)
        return;

    // xac dinh diem ban do
    QPointF ct = tileForCoordinate(mCenterLat, mCenterLon, mScale);
    qreal tx = ct.x();
    qreal ty = ct.y();


    // top-left corner of the center tile
    //diem tren cung ben trai cua manh ban do trung tam
    int xp = mMapWidth / 2 - (tx - floor(tx)) * tdim;
    int yp = mMapHeight / 2 - (ty - floor(ty)) * tdim;

    // first tile vertical and horizontal
    // manh ban do dau tien theo chieu doc va ngang
    int xa = (xp + tdim - 1) / tdim;
    int ya = (yp + tdim - 1) / tdim;
    int xs = static_cast<int>(tx) - xa;
    int ys = static_cast<int>(ty) - ya;

    // offset for top-left tile
    // offset cho manh tren cung ben trai
    m_offset = QPoint(xp - xa * tdim, yp - ya * tdim);

    // last tile vertical and horizontal
    // manh cuoi cung doc va ngang
    int xe = static_cast<int>(tx) + (mMapWidth - xp - 1) / tdim;
    int ye = static_cast<int>(ty) + (mMapHeight - yp - 1) / tdim;

    // build a rect
    // xay dung ban do
    m_tilesRect = QRect(xs, ys, xe - xs + 1, ye - ys + 1);

    //neu duong link ko rong thi thuc hien dơnload
    if(this->IsDataMapSQL == true)
    {
        LoadMap_SQL();
    }
    else if(this->IsDataMapSQL == false)
    {
        LoadMap_IMG();
    };

    emit updated(QRect(0, 0, mMapWidth, mMapHeight));
}

//ham viet du lieu hien thi tren su kien Painter
void CMapWidget::render(QPainter *p, const QRect &rect)
{
    for (int x = 0; x <= m_tilesRect.width(); ++x)
        for (int y = 0; y <= m_tilesRect.height(); ++y) {
            QPoint tp(x + m_tilesRect.left(), y + m_tilesRect.top());
            QRect box = tileRect(tp);
            if (rect.intersects(box)) {
                if (m_tilePixmaps.contains(tp))
                    p->drawPixmap(box, m_tilePixmaps.value(tp));
                else
                    p->drawPixmap(box, m_emptyTile);
            }
            p->drawRect(box);
        }
}

void CMapWidget::UpdateImage()
{
    if(mapImage)
    {
        QPainter p(mapImage);
        p.setBrush(Qt::NoBrush);

        render(&p,mapImage->rect());

    }
}

QPixmap CMapWidget::getImage(double scale)
{
    // recursive algorithm to get value of zoomRatio between 0.8 and 1.6
    double curScale = this->getScaleKm();
    double zoomRatio = scale/curScale;
    if(zoomRatio<0.8)
    {
        if(this->setScaleRatio(getScaleRatio()-1))return getImage(scale);
        else return m_emptyTile;

    }
    else if(zoomRatio>=1.6)
    {
        if(this->setScaleRatio(getScaleRatio()+1))return getImage(scale);
        else return m_emptyTile;

    }
    // repaint, rescale and return mapImage
    if(!mapImage)
    {
        printf("\nMap image size have not set.");
        return m_emptyTile;
    }
    //    return m_emptyTile;
    RepaintMap();
    return mapImage->scaled(mapImage->width()*zoomRatio,mapImage->height()*zoomRatio,Qt::IgnoreAspectRatio,Qt::SmoothTransformation );
}
void CMapWidget::ConvWGSToKm(double* x, double *y, double m_Long,double m_Lat)
{
    double refLat = (mCenterLat + (m_Lat))*0.00872664625997;
    *x	= (((m_Long)-mCenterLon) * 111.31949079327357)*cos(refLat);
    *y	= ((mCenterLat - (m_Lat)) * 111.132954);
    //tinh toa do xy KM so voi diem center khi biet lat-lon
}
float CMapWidget::CalcDistanceKm(double Long,double Lat, double m_Long,double m_Lat)
{
    double refLat = (mCenterLat + (m_Lat))*0.00872664625997;
    float dx	= (((m_Long)-mCenterLon) * 111.31949079327357)*cos(refLat);
    float dy	= ((mCenterLat - (m_Lat)) * 111.132954);
    return sqrt(dx*dx+dy*dy);
    //tinh toa do xy KM so voi diem center khi biet lat-lon
}
void CMapWidget::ConvWGS2Scr(double* x, double *y,double m_Long, double m_Lat)
{
    double refLat = (mCenterLat + (m_Lat))*0.00872664625997;
    *x	= getScaleKm()*(((m_Long) - mCenterLon) * 111.31949079327357)*cos(refLat);
    *y	= getScaleKm()*((mCenterLat - (m_Lat)) * 111.132954);
    *x   += width()/2;   //DỜI TÂM BẢN ĐỒ VỀ TÂM ĐƯỜNG TRÒN
    *y   += height()/2;
}
void CMapWidget::ConvKmToWGS(double x, double y, double *m_Long, double *m_Lat)
{
    *m_Lat  = mCenterLat +  (y)/(111.132954);
    double refLat = (mCenterLat +(*m_Lat))*0.00872664625997;//3.14159265358979324/180.0/2;
    *m_Long = (x)/(111.31949079327357*cos(refLat))+ mCenterLon;
    //tinh toa do lat-lon khi biet xy km (truong hop coi trai dat hinh cau)
}
void CMapWidget::ConvKmToWGS_precise(double x, double y, double *m_Long, double *m_Lat)
{
    *m_Lat  = mCenterLat +  (y)/(111.132954);
    double refLat = (mCenterLat +(*m_Lat))*0.00872664625997;//3.14159265358979324/180.0/2;
    *m_Long = (x)/(0.01745329252*6378.137*cos(atan(6356.7523142/6378.1370*tan(refLat))))+ mCenterLon;
    *m_Lat  = mCenterLat +  (y)/(111.132954-0.559822*cos(2.0*refLat)+0.001175*cos(4.0*refLat));
    //tinh toa do lat-lon khi biet xy km (truong hop coi trai dat hinh ellipsoid)
}

QImage CMapWidget::getDataFromDB2(QPoint grab, int rscale=-1)
{
    QString dbName = "";
    QString x1 = QString::number(grab.x()/1024);
    QString y1 = QString::number(grab.y()/1024);
    QString x2 = QString::number(grab.x()/256);
    QString y2 = QString::number(grab.y()/256);
    if(rscale<0)
    {
        dbName = mPath + "/z" + QString::number( int(mScale +1)) + "/" + x1+ "/" + y1 + "/"+x2 + "."+ y2+ ".sqlitedb";
    }
    else// for depth estimation
    {
        dbName = MAP_PATH_DoSau + "/z" + QString::number( int(rscale +1)) + "/" + x1+ "/" + y1 + "/"+x2 + "."+ y2+ ".sqlitedb";
        //                dbName = QString("C:/Users/DELL/Documents/GitHub/MAP/navionics_sonarchart/z3/0/0/0.0.sqlitedb");
    }

    printf("\n%s",dbName.toStdString().data());
    QImage img = QImage();
    {
        //        QSqlDatabase db;
        //        db.addDatabase("QSQLITE");

        QSqlDatabase db = QSqlDatabase::addDatabase( "QSQLITE" ,"map");
        db.setDatabaseName( dbName );

        if (!db.open())
        {
            qDebug() << "REPORT" << db.lastError().text();
        }

        if(db.open() == true)
        {
            qDebug() << "Database Open";
        }
        else
        {
            qDebug() << "Database NOT Open";
        }
        db.open();
        QSqlQuery query = QSqlQuery( db );
        QString StrQuery = "SELECT x, y, b FROM t WHERE x = " + QString::number(grab.x()) + " and y = " + QString::number(grab.y());

        // Get image data back from database
        query.exec( StrQuery );
        //        qDebug() << "Error getting image from table:\n" << query.lastError();

        //    query.first();
        QByteArray outByteArray;
        int X, Y;
        while (query.next())
        {
            X = query.value( 0 ).toInt();
            Y = query.value( 1 ).toInt();
            //        qDebug() << "KQ: X=" << X << "Y=" << Y;
            if(X == grab.x() && Y == grab.y())
            {
                outByteArray = query.value( 2 ).toByteArray();
                //            qDebug() << "KQ CHUAN: X=" << X << "Y=" << Y;
                break;
            }
        }

        img.loadFromData(outByteArray);
        //    qDebug() << "Query error: " << query.lastError();

        db.close();
    }
    QSqlDatabase::removeDatabase("map");
    return img;
}

void CMapWidget::requestTile(int mx,int my,int mz,int type)
{
    printf("requestTile: %d, %d,%d,%d",mx,my,mz,type);
    QVector<int> data(5) ;
    data[0] =0;
    data[1] =type;
    data[2] =mz;
    data[3] =my;
    data[4] =mx;
    QByteArray bdata = IAtoBA(data);
    socket->writeDatagram(bdata,QHostAddress("127.0.0.1"),1550);
}

void CMapWidget::LoadMap_IMG()
{
    //-------------------------------------------------------
    QPoint grab(0, 0);
    for (int x = 0; x <= m_tilesRect.width(); ++x)
        for (int y = 0; y <= m_tilesRect.height(); ++y) {
            QPoint tp = m_tilesRect.topLeft() + QPoint(x, y);
            if (!m_tilePixmaps.contains(tp)) {
                grab = tp;
                break;
            }
        }
    if (grab == QPoint(0, 0)) {
        m_url = QUrl();
        return;
    }
    QString imageMapPath = mPath.arg(mScale).arg(grab.x()).arg(grab.y());
    bool fileExist = QFile(imageMapPath+".png").exists();
    QImage img;

    bool success = img.load(imageMapPath+".png","jpg");
    if(! success) img.load(imageMapPath+".png","png");
    if (img.isNull())
    {
        fileExist = QFile(imageMapPath+".jpg").exists();
        if(fileExist)
            img = QImage(imageMapPath+".jpg");
        else
        {
            printf("\nfile not found:");
            printf((imageMapPath+".jpg").toStdString().data());

        }
    }
    if (img.isNull())
    {
        QString imageMapPathraw = mPathraw.arg(grab.x()).arg(grab.y()).arg(mScale)+".jpg";
        imageMapPath+=".jpg";
        if(QFile::exists(imageMapPathraw))
            QFile::copy(imageMapPathraw,imageMapPath);
    }


    m_tilePixmaps[grab] = QPixmap::fromImage(img);
    if (img.isNull())
        m_tilePixmaps[grab] = m_emptyTile;
    emit updated(tileRect(grab));

    // purge unused spaces
    QRect bound = m_tilesRect.adjusted(-2, -2, 2, 2);
    foreach(QPoint grab, m_tilePixmaps.keys())
        if (!bound.contains(grab))
            m_tilePixmaps.remove(grab);

    LoadMap_IMG();

}


void CMapWidget::LoadMap_SQL()
{

    QPoint grab(0, 0);
    for (int x = 0; x <= m_tilesRect.width(); ++x)
        for (int y = 0; y <= m_tilesRect.height(); ++y) {
            QPoint tp = m_tilesRect.topLeft() + QPoint(x, y);
            if (!m_tilePixmaps.contains(tp)) {
                grab = tp;
                break;
            }
        }
    if (grab == QPoint(0, 0)) {
        m_url = QUrl();
        return;
    }
    ////////////////////
    /// \brief imageMapPath
    ///

    QImage img = getDataFromDB2(grab,-1);//********************SQLITE
    if (img.isNull())
    {
        requestTile(grab.x(),grab.y(),mScale,1);
        m_tilePixmaps[grab] = m_emptyTile;
    }
    else m_tilePixmaps[grab] = QPixmap::fromImage(img);

    emit updated(tileRect(grab));

    // purge unused spaces
    QRect bound = m_tilesRect.adjusted(-2, -2, 2, 2);
    foreach(QPoint grab, m_tilePixmaps.keys())
        if (!bound.contains(grab))
            m_tilePixmaps.remove(grab);

    LoadMap_SQL();

}

void CMapWidget::SetMapDir(QString dir)
{
    //    m_tilePixmaps.clear();
    //setPath(MAP_PATH_DoSau+dir);

}


// ham xac dinh vi tri manh ban do voi diem top-left cua no la diem tp
QRect CMapWidget::tileRect(const QPoint &tp)
{
    QPoint t = tp - m_tilesRect.topLeft();
    int x = t.x() * tdim + m_offset.x();
    int y = t.y() * tdim + m_offset.y();
    return QRect(x, y, tdim, tdim);
}
double CMapWidget::getLat()
{
    return mCenterLat;
}
double CMapWidget::getLon()
{
    return mCenterLon;
}
