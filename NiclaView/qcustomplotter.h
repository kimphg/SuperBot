#ifndef QCUSTOM_FRAME_H
#define QCUSTOM_FRAME_H

#include <QWidget>
#include <QFrame>
#include <qevent.h>
class QCustomPlotter: public QFrame
{
    Q_OBJECT
    void draw_graphs(QPainter *p);
public:
    explicit QCustomPlotter(QWidget *parent = 0);

    void addRecord(int dataid, float value);
    void resetRecord();
    void stopRecord();
    void saveCsv(QString filename);
protected:
    void highLight();
    void resetView();
    void hoverEnter(QHoverEvent *event);
    void hoverLeave(QHoverEvent *event);
    void hoverMove(QHoverEvent *event);
    bool event(QEvent *event);
private:

    bool isRecording =false;
signals:

public slots:
protected slots:
    void paintEvent(QPaintEvent *);
    void timerEvent(QTimerEvent *event);
};

#endif // QCUSTOM_FRAME_H
