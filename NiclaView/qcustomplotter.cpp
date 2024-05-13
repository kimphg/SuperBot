#include "QCustomPlotter.h"

#include <QPainter>
#define RECORD_COUNT_MAX 800
int plot_data[10][RECORD_COUNT_MAX];
int recordCount[]={0,0,0,0,0,0,0,0,0,0};
QList<QColor> *colorList = new QList<QColor>\
({
//     QColor(0  ,0  ,0),//0
     QColor(220,0,0),
    QColor(0  ,0  ,220),
    QColor(220,140,0),//2
    QColor(220,220,0),
    QColor(140,250,0),//4
    QColor(0  ,220,0),
     QColor(140  ,140,140),//6
     QColor(0  ,220,220),
     QColor(0  ,140,220),//8

     QColor(140  ,0  ,220),//10
     QColor(220  ,0  ,140),
     QColor(0  ,0  ,0),
     QColor(255  ,255  ,255)//12
});
QCustomPlotter::QCustomPlotter(QWidget *parent) : QFrame(parent)
{
    setMouseTracking(true);
    setAttribute(Qt::WA_Hover);
    this->setCursor(Qt::ArrowCursor);
    resetView();
    startTimer(100);
}
void QCustomPlotter::timerEvent(QTimerEvent *event)
{
    repaint();
}
void QCustomPlotter::hoverEnter(QHoverEvent *)
{

    highLight();
}

void QCustomPlotter::hoverLeave(QHoverEvent *)
{
    resetView();
}

void QCustomPlotter::hoverMove(QHoverEvent *)
{
    highLight();

}
void QCustomPlotter::highLight()
{
//    this->setStyleSheet("background-color: rgb(16, 32, 64);color:rgb(255, 255, 255);font: bold 12pt \"MS Shell Dlg 2\";border : 3px solid gray;");
    repaint();
}
void QCustomPlotter::resetView()
{
    //this->setStyleSheet("background-color: rgb(16, 32, 64);color:rgb(255, 255, 255);");
    this->setStyleSheet("background-color: rgb(255, 255, 255);color:rgb(255, 255, 255);font: 12pt \"MS Shell Dlg 2\";");
    repaint();

}
void QCustomPlotter::paintEvent(QPaintEvent *)
{
    QPainter p(this);
    draw_graphs(&p);
}

void QCustomPlotter::draw_graphs(QPainter *p)
{
    int shifty=height()/2;
    float scale90 = height()/6.0/90.0;
//    p->setBrush(QColor(220,140,0));
//    p->drawRect(this->rect());
//    p->setPen(QPen(Qt::gray,2));
//    for (int i=-12;i<=12;i++)
//    {
//        int value = shifty +(shifty/12*i);
//        p->drawLine(0,value,1000,value);

//    }
    p->setPen(Qt::black);
    for (int i=-4;i<=4;i+=2)
    {
        int value = shifty +(shifty/6*i);
        p->drawLine(0,value,1000,value);
        p->drawText(10,-2+value,QString::number(-i*45));
    }
    for (int dataid=0;dataid<2;dataid++)
    {
        p->setPen(QPen(colorList->at(dataid),2));
        for (int x=1;x<=recordCount[dataid];x++)
        {
            int x0=x-1;
            float scale = 1.2;
            p->drawLine(x0*scale,shifty-plot_data[dataid][x0]*scale90,x*scale,shifty-plot_data[dataid][x]*scale90);

        }
    }
    p->setPen(Qt::black);
    for (int dataid=0;dataid<2;dataid++)
    {
        p->setBrush(colorList->at(dataid));
        p->drawRect(20+100*dataid,20,20,20);

    }
    for (int dataid=0;dataid<2;dataid++)
    {
        QString label ;
        if(dataid==0)label = QString::fromUtf8("Góc xoay");
        if(dataid==1)label = QString::fromUtf8("Góc ngẩng");
        p->setBrush(Qt::white);
        p->drawText(40+100*dataid,20,100,20,0,label);
    }
        p->setPen(QPen(Qt::blue,8));
        p->setBrush(Qt::NoBrush);
        p->drawRect(this->rect());

}
void QCustomPlotter::resetRecord()
{
    memset(&recordCount[0],0,10);
    memset(&plot_data[0][0],0,10*RECORD_COUNT_MAX);
    isRecording=true;
}

void QCustomPlotter::stopRecord()
{
    isRecording=false;
}

void QCustomPlotter::saveCsv(QString filename)
{

    QByteArray output;
    output.append("STT,Up angle,Right angle\n");
        for (int x=1;x<=RECORD_COUNT_MAX;x++)
        {
            output.append(QByteArray::number(x));

            for (int dataid=0;dataid<2;dataid++)
            {
                output.append(",");
                output.append(QByteArray::number(plot_data[dataid][x]));

            }
            output.append("\n");
        }

    QFile file(filename +".csv");
    if (file.open(QIODevice::WriteOnly))
    {
        file.write(output);
        file.close();
    }

}
void QCustomPlotter::addRecord(int dataid,float value)
{
    if(dataid>=10)return;
    if(recordCount[dataid]>=RECORD_COUNT_MAX)
    {
//        recordCount[dataid]=0;
//        isRecording = false;
        return;
    }
    if(isRecording)
    {
        plot_data[dataid][recordCount[dataid]] = value*1.5;
        recordCount[dataid]+=1;
    }
    repaint();

}
bool QCustomPlotter::event(QEvent *event)
{
//    switch(event->type())
//    {
//    case QEvent::HoverEnter:
//        hoverEnter(static_cast<QHoverEvent*>(event));
//        return true;
//        break;
//    case QEvent::HoverLeave:
//        hoverLeave(static_cast<QHoverEvent*>(event));
//        return true;
//        break;
//    case QEvent::HoverMove:
//        hoverMove(static_cast<QHoverEvent*>(event));
//        return true;
//        break;
//    default:
//        break;
//    }
    return QWidget::event(event);
}
