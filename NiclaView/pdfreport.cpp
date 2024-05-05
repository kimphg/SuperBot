#include "pdfreport.h"
#include <QDate>
#include <QDateTime>
#include <QPrinter>
#include <QTextDocument>
#include <QString>
#include <QTimer>
#include <QDebug>

PdfReport::PdfReport()
{

    auto_save_timer = new QTimer(this);
    resetData();
//    QObject::connect(auto_save_timer, SIGNAL(timeout()), this, SLOT(saveReport()));
//    auto_save_timer->start(1000*30); // auto save report every 30s
}
void PdfReport::resetData()
{// Get the current date and time
    QDateTime currentDateTime = QDateTime::currentDateTime();
    QDate currentDate = currentDateTime.date();

    // Format the date as needed
    QString currentDateStr = currentDate.toString("dd/MM/yyyy");

    report_data =
            "<head>\
                <style>\
                    * {\
                        margin: 0;\
                        padding: 0;\
                    }\
                    .imgbox {\
                        display: grid;\
                        height: 100%;\
                    }\
                    .center-fit {\
                        max-width: 100%;\
                        max-height: 100vh;\
                        margin: auto;\
                    }\
                </style>\
            </head>"
        "<div align=right>"
        "   Hà Nội, " + currentDateStr +
        "</div>"

        "<div align=left>"
        "    BỆNH VIỆN TRUNG ƯƠNG QUÂN ĐỘI 108 <br>"
        "   KHOA CHẤN THƯƠNG CHỈNH HÌNH CỘT SỐNG <br>"
        " Số 1 Trần Hưng Đạo, Hai Bà Trưng, Hà Nội"
        "</div>"
        "<div align=left>"
        "<img src=\"logo.png\" alt=\"\">"
        "</div>"
        "<h1 align=center>KẾT QUẢ ĐO GÓC CỘT SỐNG</h1>"
        "<div></div>"
        "<table border=1 width=\"100%\" cellpadding=5 cellspacing=0>"
        "    <tr>"
        "        <th>STT</th>"
        "        <th>Tên BN</th>"
        "        <th>Mã BN</th>"
        "        <th>Năm sinh BN</th>"
        "        <th>Thời gian</th>"
        "        <th>Biên độ cúi ngẩng</th>"
        "        <th>Biên độ xoay ngang</th>"
        "    </tr>";
}
void PdfReport::insertRecord(const QString &name, const QString &code,const QString &age,const QString &tilt, const QString &pan)
{
    count_rec += 1;
    QString index = QString::number(count_rec);

    QDateTime currentDateTime = QDateTime::currentDateTime();
    QString time = currentDateTime.toString("dd-MM-yyyy hh:mm:ss");
    QString fielname = currentDateTime.toString("dd_MM_yyyy_hh_mm")+"_"+name;
    // Append a new row to the table
    report_data += "<tr>";
    report_data += "<td>" + index + "</td>";
    report_data += "<td>" + name + "</td>";
    report_data += "<td>" + code + "</td>";
    report_data += "<td>" + age + "</td>";
    report_data += "<td>" + time + "</td>";
    report_data += "<td>" + tilt + "</td>";
    report_data += "<td>" + pan + "</td>";
    report_data += "</tr>";
    report_data += "</table>";
    report_data +="<h3 align=left>Kết quả chi tiết: </h1>";
    report_data += "<div align=left>";
    report_data +="<img class=\"center-fit\" src=\"";
    report_data +=fielname;
    report_data +="\" alt=\"\">";
    report_data +="</div>";

    lastFileName = currentDateTime.toString("dd_MM_yyyy_hh_mm")+"_"+name+".pdf";
    saveReport(lastFileName); // auto save if there are new record
}

void PdfReport::saveToPdf(QString file_name, QString html_data)
{
    QTextDocument document;
    document.setHtml(html_data);
    QPrinter printer(QPrinter::PrinterResolution);
    printer.setOutputFormat(QPrinter::PdfFormat);
    printer.setPaperSize(QPrinter::A4);
    printer.setOutputFileName(file_name);
    printer.setPageMargins(QMarginsF(15, 15, 15, 15));
    document.print(&printer);
}

int PdfReport::getCount_rec() const
{
    return count_rec+1;
}

//void PdfReport::insertRecord(QString &html, const QString &index, const QString &device,
//                             const QString &time, const QString &loc, const QString &type,
//                             const QString &value)
//{
//    // Append a new row to the table
//    report_data += "<tr>";
//    report_data += "<td>" + index + "</td>";
//    report_data += "<td>" + device + "</td>";
//    report_data += "<td>" + time + "</td>";
//    report_data += "<td>" + loc + "</td>";
//    report_data += "<td>" + type + "</td>";
//    report_data += "<td>" + value + "</td>";
//    report_data += "</tr>";
//}

void PdfReport::saveReport()
{
    QString default_file = "Report_autosave.pdf";
//    qDebug() << "Auto save report: " << default_file;
    QTextDocument document;
    document.setHtml(report_data);

    QPrinter printer(QPrinter::PrinterResolution);
    printer.setOutputFormat(QPrinter::PdfFormat);
    printer.setPaperSize(QPrinter::A4);
    printer.setOutputFileName(default_file);
    printer.setPageMargins(QMarginsF(15, 15, 15, 15));
    document.print(&printer);
}

void PdfReport::saveReport(const QString path)
{
    QTextDocument document;
    document.setHtml(report_data);

    QPrinter printer(QPrinter::PrinterResolution);
    printer.setOutputFormat(QPrinter::PdfFormat);
    printer.setPaperSize(QPrinter::A4);
    printer.setOutputFileName(path);
    printer.setPageMargins(QMarginsF(15, 15, 15, 15));
    document.print(&printer);
}
