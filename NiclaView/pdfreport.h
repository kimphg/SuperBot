#ifndef PDFREPORT_H
#define PDFREPORT_H

#include <QObject>
#include <QString>
#include <QDebug>
#include <QThread>
#include <QTimer>
 #include <QtPrintSupport/QtPrintSupport>

class PdfReport : public QObject
{
    Q_OBJECT
public:
    PdfReport();
//    void insertRecord(QString &html, const QString &index, const QString &device,
//                      const QString &time, const QString &loc, const QString &type,
//                      const QString &value);
    void insertRecord(const QString &device, const QString &code, const QString &age,
                      const QString &tilt,
                      const QString &pan);
    void saveToPdf(QString filename, QString html_data); // export all data of devices
    int getCount_rec() const;

    void resetData();
    QString lastFileName;
private:

    int count_rec = 0;
    QString report_data;
    QTimer* auto_save_timer;

public slots:
    void saveReport();
    void saveReport(const QString path);
};

#endif // PDFREPORT_H
