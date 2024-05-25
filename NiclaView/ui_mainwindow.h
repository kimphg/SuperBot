/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.14.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>
#include <qcustomplotter.h>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QGridLayout *gridLayout;
    QGroupBox *groupBox;
    QGridLayout *gridLayout_2;
    QLineEdit *lineEdit_2;
    QLabel *label_10;
    QLineEdit *lineEdit_3;
    QLabel *label_9;
    QLabel *label_8;
    QLineEdit *lineEdit;
    QFrame *frame;
    QGridLayout *gridLayout_4;
    QLabel *label_img_right;
    QLabel *label_img_left;
    QLabel *label_img_center;
    QLabel *label_img_top;
    QLabel *label_img_bot;
    QLabel *label_12;
    QLabel *label;
    QGroupBox *groupBox_2;
    QGridLayout *gridLayout_3;
    QLabel *label_7;
    QLabel *label_3;
    QLabel *label_5;
    QLabel *label_roll_min;
    QLabel *label_6;
    QLabel *label_4;
    QLabel *label_2;
    QLabel *label_pitch_min;
    QLabel *label_roll;
    QLabel *label_pitch;
    QLabel *label_pitch_max;
    QLabel *label_roll_max;
    QPushButton *pushButton_5;
    QPushButton *pushButton_3;
    QFrame *frame_2;
    QGridLayout *gridLayout_5;
    QLabel *label_11;
    QLabel *label_mes_count;
    QPushButton *pushButton;
    QPushButton *pushButton_2;
    QPushButton *pushButton_4;
    QCustomPlotter *plotter;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1913, 1021);
        QFont font;
        font.setPointSize(8);
        MainWindow->setFont(font);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        gridLayout = new QGridLayout(centralwidget);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        groupBox = new QGroupBox(centralwidget);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setStyleSheet(QString::fromUtf8("background-color: rgb(222, 255, 222);color:rgb(0, 0, 0);font: 10pt \\\"MS Shell Dlg 2\\\";border : 2px solid gray;"));
        gridLayout_2 = new QGridLayout(groupBox);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        lineEdit_2 = new QLineEdit(groupBox);
        lineEdit_2->setObjectName(QString::fromUtf8("lineEdit_2"));
        lineEdit_2->setMinimumSize(QSize(30, 40));
        lineEdit_2->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);color:rgb(0, 0, 0);"));

        gridLayout_2->addWidget(lineEdit_2, 1, 2, 1, 1);

        label_10 = new QLabel(groupBox);
        label_10->setObjectName(QString::fromUtf8("label_10"));

        gridLayout_2->addWidget(label_10, 2, 0, 1, 1);

        lineEdit_3 = new QLineEdit(groupBox);
        lineEdit_3->setObjectName(QString::fromUtf8("lineEdit_3"));
        lineEdit_3->setMinimumSize(QSize(30, 40));
        lineEdit_3->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);color:rgb(0, 0, 0);"));

        gridLayout_2->addWidget(lineEdit_3, 2, 2, 1, 1);

        label_9 = new QLabel(groupBox);
        label_9->setObjectName(QString::fromUtf8("label_9"));

        gridLayout_2->addWidget(label_9, 1, 0, 1, 1);

        label_8 = new QLabel(groupBox);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        gridLayout_2->addWidget(label_8, 0, 0, 1, 1);

        lineEdit = new QLineEdit(groupBox);
        lineEdit->setObjectName(QString::fromUtf8("lineEdit"));
        lineEdit->setMinimumSize(QSize(30, 40));
        lineEdit->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);color:rgb(0, 0, 0);"));

        gridLayout_2->addWidget(lineEdit, 0, 2, 1, 1);


        gridLayout->addWidget(groupBox, 5, 0, 4, 2);

        frame = new QFrame(centralwidget);
        frame->setObjectName(QString::fromUtf8("frame"));
        frame->setMinimumSize(QSize(850, 750));
        frame->setStyleSheet(QString::fromUtf8("background-color: rgb(222, 255, 222);color:rgb(0, 0, 0);font: 10pt \\\"MS Shell Dlg 2\\\";border : 4px solid blue;"));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        gridLayout_4 = new QGridLayout(frame);
        gridLayout_4->setSpacing(2);
        gridLayout_4->setObjectName(QString::fromUtf8("gridLayout_4"));
        gridLayout_4->setContentsMargins(2, 2, 2, 2);
        label_img_right = new QLabel(frame);
        label_img_right->setObjectName(QString::fromUtf8("label_img_right"));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(label_img_right->sizePolicy().hasHeightForWidth());
        label_img_right->setSizePolicy(sizePolicy);
        label_img_right->setMinimumSize(QSize(320, 240));

        gridLayout_4->addWidget(label_img_right, 1, 2, 1, 1);

        label_img_left = new QLabel(frame);
        label_img_left->setObjectName(QString::fromUtf8("label_img_left"));
        sizePolicy.setHeightForWidth(label_img_left->sizePolicy().hasHeightForWidth());
        label_img_left->setSizePolicy(sizePolicy);
        label_img_left->setMinimumSize(QSize(320, 240));

        gridLayout_4->addWidget(label_img_left, 1, 0, 1, 1);

        label_img_center = new QLabel(frame);
        label_img_center->setObjectName(QString::fromUtf8("label_img_center"));
        sizePolicy.setHeightForWidth(label_img_center->sizePolicy().hasHeightForWidth());
        label_img_center->setSizePolicy(sizePolicy);
        label_img_center->setMinimumSize(QSize(320, 240));

        gridLayout_4->addWidget(label_img_center, 1, 1, 1, 1);

        label_img_top = new QLabel(frame);
        label_img_top->setObjectName(QString::fromUtf8("label_img_top"));
        sizePolicy.setHeightForWidth(label_img_top->sizePolicy().hasHeightForWidth());
        label_img_top->setSizePolicy(sizePolicy);
        label_img_top->setMinimumSize(QSize(320, 240));

        gridLayout_4->addWidget(label_img_top, 0, 1, 1, 1);

        label_img_bot = new QLabel(frame);
        label_img_bot->setObjectName(QString::fromUtf8("label_img_bot"));
        sizePolicy.setHeightForWidth(label_img_bot->sizePolicy().hasHeightForWidth());
        label_img_bot->setSizePolicy(sizePolicy);
        label_img_bot->setMinimumSize(QSize(320, 240));

        gridLayout_4->addWidget(label_img_bot, 2, 1, 1, 1);


        gridLayout->addWidget(frame, 0, 0, 5, 2);

        label_12 = new QLabel(centralwidget);
        label_12->setObjectName(QString::fromUtf8("label_12"));
        QFont font1;
        font1.setPointSize(12);
        label_12->setFont(font1);

        gridLayout->addWidget(label_12, 4, 5, 1, 2);

        label = new QLabel(centralwidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setFont(font1);

        gridLayout->addWidget(label, 3, 5, 1, 4);

        groupBox_2 = new QGroupBox(centralwidget);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        QFont font2;
        font2.setFamily(QString::fromUtf8("MS Shell Dlg 2"));
        font2.setPointSize(8);
        font2.setBold(false);
        font2.setItalic(false);
        font2.setWeight(50);
        groupBox_2->setFont(font2);
        groupBox_2->setStyleSheet(QString::fromUtf8("background-color: rgb(222, 255, 222);color:rgb(0, 0, 0);"));
        gridLayout_3 = new QGridLayout(groupBox_2);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        gridLayout_3->setContentsMargins(-1, 15, -1, -1);
        label_7 = new QLabel(groupBox_2);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        gridLayout_3->addWidget(label_7, 0, 4, 1, 1);

        label_3 = new QLabel(groupBox_2);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        gridLayout_3->addWidget(label_3, 1, 0, 1, 1);

        label_5 = new QLabel(groupBox_2);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        gridLayout_3->addWidget(label_5, 1, 4, 1, 1);

        label_roll_min = new QLabel(groupBox_2);
        label_roll_min->setObjectName(QString::fromUtf8("label_roll_min"));

        gridLayout_3->addWidget(label_roll_min, 1, 3, 1, 1);

        label_6 = new QLabel(groupBox_2);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        gridLayout_3->addWidget(label_6, 0, 2, 1, 1);

        label_4 = new QLabel(groupBox_2);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        gridLayout_3->addWidget(label_4, 1, 2, 1, 1);

        label_2 = new QLabel(groupBox_2);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setStyleSheet(QString::fromUtf8(""));

        gridLayout_3->addWidget(label_2, 0, 0, 1, 1);

        label_pitch_min = new QLabel(groupBox_2);
        label_pitch_min->setObjectName(QString::fromUtf8("label_pitch_min"));

        gridLayout_3->addWidget(label_pitch_min, 0, 3, 1, 1);

        label_roll = new QLabel(groupBox_2);
        label_roll->setObjectName(QString::fromUtf8("label_roll"));

        gridLayout_3->addWidget(label_roll, 1, 1, 1, 1);

        label_pitch = new QLabel(groupBox_2);
        label_pitch->setObjectName(QString::fromUtf8("label_pitch"));

        gridLayout_3->addWidget(label_pitch, 0, 1, 1, 1);

        label_pitch_max = new QLabel(groupBox_2);
        label_pitch_max->setObjectName(QString::fromUtf8("label_pitch_max"));

        gridLayout_3->addWidget(label_pitch_max, 0, 5, 1, 1);

        label_roll_max = new QLabel(groupBox_2);
        label_roll_max->setObjectName(QString::fromUtf8("label_roll_max"));

        gridLayout_3->addWidget(label_roll_max, 1, 5, 1, 1);

        pushButton_5 = new QPushButton(groupBox_2);
        pushButton_5->setObjectName(QString::fromUtf8("pushButton_5"));
        pushButton_5->setMinimumSize(QSize(0, 40));
        pushButton_5->setStyleSheet(QString::fromUtf8("background-color: rgb(32, 64, 128);color:rgb(255, 255, 255);font: 10pt \\\"MS Shell Dlg 2\\\";border : 3px solid gray;"));

        gridLayout_3->addWidget(pushButton_5, 2, 5, 1, 1);

        pushButton_3 = new QPushButton(groupBox_2);
        pushButton_3->setObjectName(QString::fromUtf8("pushButton_3"));
        pushButton_3->setMinimumSize(QSize(0, 40));
        pushButton_3->setStyleSheet(QString::fromUtf8("background-color: rgb(32, 64, 128);color:rgb(255, 255, 255);font: 10pt \\\"MS Shell Dlg 2\\\";border : 3px solid gray;"));

        gridLayout_3->addWidget(pushButton_3, 2, 0, 1, 5);


        gridLayout->addWidget(groupBox_2, 5, 5, 4, 4);

        frame_2 = new QFrame(centralwidget);
        frame_2->setObjectName(QString::fromUtf8("frame_2"));
        frame_2->setStyleSheet(QString::fromUtf8("background-color: rgb(222, 255, 222);color:rgb(0, 0, 0);font: 10pt \\\"MS Shell Dlg 2\\\";border : 2px solid gray;"));
        frame_2->setFrameShape(QFrame::StyledPanel);
        frame_2->setFrameShadow(QFrame::Raised);
        gridLayout_5 = new QGridLayout(frame_2);
        gridLayout_5->setObjectName(QString::fromUtf8("gridLayout_5"));
        label_11 = new QLabel(frame_2);
        label_11->setObjectName(QString::fromUtf8("label_11"));

        gridLayout_5->addWidget(label_11, 0, 0, 1, 1);

        label_mes_count = new QLabel(frame_2);
        label_mes_count->setObjectName(QString::fromUtf8("label_mes_count"));

        gridLayout_5->addWidget(label_mes_count, 0, 1, 1, 1);

        pushButton = new QPushButton(frame_2);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));
        pushButton->setMinimumSize(QSize(0, 40));
        QPalette palette;
        QBrush brush(QColor(255, 255, 255, 255));
        brush.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::WindowText, brush);
        QBrush brush1(QColor(32, 64, 128, 255));
        brush1.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Button, brush1);
        palette.setBrush(QPalette::Active, QPalette::Text, brush);
        palette.setBrush(QPalette::Active, QPalette::ButtonText, brush);
        palette.setBrush(QPalette::Active, QPalette::Base, brush1);
        palette.setBrush(QPalette::Active, QPalette::Window, brush1);
        QBrush brush2(QColor(255, 255, 255, 128));
        brush2.setStyle(Qt::NoBrush);
#if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette.setBrush(QPalette::Active, QPalette::PlaceholderText, brush2);
#endif
        palette.setBrush(QPalette::Inactive, QPalette::WindowText, brush);
        palette.setBrush(QPalette::Inactive, QPalette::Button, brush1);
        palette.setBrush(QPalette::Inactive, QPalette::Text, brush);
        palette.setBrush(QPalette::Inactive, QPalette::ButtonText, brush);
        palette.setBrush(QPalette::Inactive, QPalette::Base, brush1);
        palette.setBrush(QPalette::Inactive, QPalette::Window, brush1);
        QBrush brush3(QColor(255, 255, 255, 128));
        brush3.setStyle(Qt::NoBrush);
#if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette.setBrush(QPalette::Inactive, QPalette::PlaceholderText, brush3);
#endif
        palette.setBrush(QPalette::Disabled, QPalette::WindowText, brush);
        palette.setBrush(QPalette::Disabled, QPalette::Button, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Text, brush);
        palette.setBrush(QPalette::Disabled, QPalette::ButtonText, brush);
        palette.setBrush(QPalette::Disabled, QPalette::Base, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Window, brush1);
        QBrush brush4(QColor(255, 255, 255, 128));
        brush4.setStyle(Qt::NoBrush);
#if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette.setBrush(QPalette::Disabled, QPalette::PlaceholderText, brush4);
#endif
        pushButton->setPalette(palette);
        pushButton->setStyleSheet(QString::fromUtf8("background-color: rgb(32, 64, 128);color:rgb(255, 255, 255);font: 10pt \\\"MS Shell Dlg 2\\\";border : 3px solid gray;"));
        pushButton->setFlat(false);

        gridLayout_5->addWidget(pushButton, 1, 0, 1, 2);

        pushButton_2 = new QPushButton(frame_2);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));
        pushButton_2->setMinimumSize(QSize(0, 40));
        QPalette palette1;
        palette1.setBrush(QPalette::Active, QPalette::WindowText, brush);
        palette1.setBrush(QPalette::Active, QPalette::Button, brush1);
        palette1.setBrush(QPalette::Active, QPalette::Text, brush);
        palette1.setBrush(QPalette::Active, QPalette::ButtonText, brush);
        palette1.setBrush(QPalette::Active, QPalette::Base, brush1);
        palette1.setBrush(QPalette::Active, QPalette::Window, brush1);
        QBrush brush5(QColor(255, 255, 255, 128));
        brush5.setStyle(Qt::NoBrush);
#if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette1.setBrush(QPalette::Active, QPalette::PlaceholderText, brush5);
#endif
        palette1.setBrush(QPalette::Inactive, QPalette::WindowText, brush);
        palette1.setBrush(QPalette::Inactive, QPalette::Button, brush1);
        palette1.setBrush(QPalette::Inactive, QPalette::Text, brush);
        palette1.setBrush(QPalette::Inactive, QPalette::ButtonText, brush);
        palette1.setBrush(QPalette::Inactive, QPalette::Base, brush1);
        palette1.setBrush(QPalette::Inactive, QPalette::Window, brush1);
        QBrush brush6(QColor(255, 255, 255, 128));
        brush6.setStyle(Qt::NoBrush);
#if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette1.setBrush(QPalette::Inactive, QPalette::PlaceholderText, brush6);
#endif
        palette1.setBrush(QPalette::Disabled, QPalette::WindowText, brush);
        palette1.setBrush(QPalette::Disabled, QPalette::Button, brush1);
        palette1.setBrush(QPalette::Disabled, QPalette::Text, brush);
        palette1.setBrush(QPalette::Disabled, QPalette::ButtonText, brush);
        palette1.setBrush(QPalette::Disabled, QPalette::Base, brush1);
        palette1.setBrush(QPalette::Disabled, QPalette::Window, brush1);
        QBrush brush7(QColor(255, 255, 255, 128));
        brush7.setStyle(Qt::NoBrush);
#if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette1.setBrush(QPalette::Disabled, QPalette::PlaceholderText, brush7);
#endif
        pushButton_2->setPalette(palette1);
        pushButton_2->setStyleSheet(QString::fromUtf8("background-color: rgb(32, 64, 128);color:rgb(255, 255, 255);font: 10pt \\\"MS Shell Dlg 2\\\";border : 3px solid gray;"));
        pushButton_2->setFlat(false);

        gridLayout_5->addWidget(pushButton_2, 2, 0, 1, 2);

        pushButton_4 = new QPushButton(frame_2);
        pushButton_4->setObjectName(QString::fromUtf8("pushButton_4"));
        pushButton_4->setMinimumSize(QSize(0, 40));
        QPalette palette2;
        palette2.setBrush(QPalette::Active, QPalette::WindowText, brush);
        palette2.setBrush(QPalette::Active, QPalette::Button, brush1);
        palette2.setBrush(QPalette::Active, QPalette::Text, brush);
        palette2.setBrush(QPalette::Active, QPalette::ButtonText, brush);
        palette2.setBrush(QPalette::Active, QPalette::Base, brush1);
        palette2.setBrush(QPalette::Active, QPalette::Window, brush1);
        QBrush brush8(QColor(255, 255, 255, 128));
        brush8.setStyle(Qt::NoBrush);
#if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette2.setBrush(QPalette::Active, QPalette::PlaceholderText, brush8);
#endif
        palette2.setBrush(QPalette::Inactive, QPalette::WindowText, brush);
        palette2.setBrush(QPalette::Inactive, QPalette::Button, brush1);
        palette2.setBrush(QPalette::Inactive, QPalette::Text, brush);
        palette2.setBrush(QPalette::Inactive, QPalette::ButtonText, brush);
        palette2.setBrush(QPalette::Inactive, QPalette::Base, brush1);
        palette2.setBrush(QPalette::Inactive, QPalette::Window, brush1);
        QBrush brush9(QColor(255, 255, 255, 128));
        brush9.setStyle(Qt::NoBrush);
#if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette2.setBrush(QPalette::Inactive, QPalette::PlaceholderText, brush9);
#endif
        palette2.setBrush(QPalette::Disabled, QPalette::WindowText, brush);
        palette2.setBrush(QPalette::Disabled, QPalette::Button, brush1);
        palette2.setBrush(QPalette::Disabled, QPalette::Text, brush);
        palette2.setBrush(QPalette::Disabled, QPalette::ButtonText, brush);
        palette2.setBrush(QPalette::Disabled, QPalette::Base, brush1);
        palette2.setBrush(QPalette::Disabled, QPalette::Window, brush1);
        QBrush brush10(QColor(255, 255, 255, 128));
        brush10.setStyle(Qt::NoBrush);
#if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette2.setBrush(QPalette::Disabled, QPalette::PlaceholderText, brush10);
#endif
        pushButton_4->setPalette(palette2);
        pushButton_4->setStyleSheet(QString::fromUtf8("background-color: rgb(32, 64, 128);color:rgb(255, 255, 255);font: 10pt \\\"MS Shell Dlg 2\\\";border : 3px solid gray;"));
        pushButton_4->setFlat(false);

        gridLayout_5->addWidget(pushButton_4, 3, 0, 1, 2);


        gridLayout->addWidget(frame_2, 2, 5, 1, 4);

        plotter = new QCustomPlotter(centralwidget);
        plotter->setObjectName(QString::fromUtf8("plotter"));
        plotter->setMinimumSize(QSize(0, 350));
        plotter->setFrameShape(QFrame::StyledPanel);
        plotter->setFrameShadow(QFrame::Raised);

        gridLayout->addWidget(plotter, 0, 5, 2, 4);

        MainWindow->setCentralWidget(centralwidget);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "Ph\341\272\247n m\341\273\201m ki\341\273\203m tra g\303\263c c\341\273\231t s\341\273\221ng", nullptr));
        groupBox->setTitle(QCoreApplication::translate("MainWindow", "Th\303\264ng tin b\341\273\207nh nh\303\242n", nullptr));
        label_10->setText(QCoreApplication::translate("MainWindow", "N\304\203m sinh:", nullptr));
        label_9->setText(QCoreApplication::translate("MainWindow", "M\303\243 BN:", nullptr));
        label_8->setText(QCoreApplication::translate("MainWindow", "T\303\252n BN:", nullptr));
        label_img_right->setText(QCoreApplication::translate("MainWindow", "video", nullptr));
        label_img_left->setText(QCoreApplication::translate("MainWindow", "video", nullptr));
        label_img_center->setText(QCoreApplication::translate("MainWindow", "video", nullptr));
        label_img_top->setText(QCoreApplication::translate("MainWindow", "video", nullptr));
        label_img_bot->setText(QCoreApplication::translate("MainWindow", "video", nullptr));
        label_12->setText(QCoreApplication::translate("MainWindow", "Ch\303\272 th\303\255ch: gi\303\241 tr\341\273\213 \304\221o \304\221\306\260\341\273\243c c\303\263 \304\221\306\241n v\341\273\213 l\303\240 \304\221\341\273\231, \n"
"gi\303\241 tr\341\273\213 t\304\203ng d\341\272\247n v\341\273\201 h\306\260\341\273\233ng t\306\260\306\241ng \341\273\251ng v\341\273\233i chi\341\273\201u t\341\273\253 tr\303\241i qua ph\341\272\243i, t\304\203ng d\341\272\247n v\341\273\201 t\341\272\247m t\306\260\306\241ng t\341\273\251ng chi\341\273\201u t\341\273\253 d\306\260\341\273\233i l\303\252n tr\303\252n", nullptr));
        label->setText(QCoreApplication::translate("MainWindow", "H\306\260\341\273\233ng d\341\272\253n: Thi\341\272\277t b\341\273\213 \304\221o \304\221\306\260\341\273\243c \304\221eo l\303\252n m\341\272\257t b\341\273\207nh nh\303\242n sao cho h\303\254nh \341\272\243nh camera tr\303\271ng kh\341\273\233p v\341\273\233i h\306\260\341\273\233ng nh\303\254n\n"
"c\341\273\247a b\341\273\207nh nh\303\242n.\n"
"Ng\306\260\341\273\235i ki\341\273\203m tra \304\221\341\273\251ng ph\303\255a sau b\341\273\207nh nh\303\242n \304\221\341\273\203 h\306\260\341\273\233ng d\341\272\253n v\303\240 gi\303\241m s\303\241t.\n"
"B\341\273\207nh nh\303\242n l\341\272\247n l\306\260\341\273\243t th\341\273\261c hi\341\273\207n c\303\241c \304\221\341\273\231ng t\303\241c c\303\272i, ng\341\272\251ng, xoay tr\303\241i v\303\240 xoay ph\341\272\243i \304\221\341\272\277n t\341\273\233i h\341\272\241n cho ph\303\251p.\n"
"B\341\273\207nh nh\303\242n th\341\273\261c hi\341\273\207n \304\221i b\341\273\231 theo \304\221\306\260\341\273\235ng th\341\272\263ng trong 30 gi\303\242y.\n"
"Ng\306\260"
                        "\341\273\235i ki\341\273\203m tra nh\341\272\245n n\303\272t ho\303\240n th\303\240nh \304\221\341\273\203 l\306\260u k\341\272\277t qu\341\272\243 \304\221o.", nullptr));
        groupBox_2->setTitle(QCoreApplication::translate("MainWindow", "K\341\272\277t qu\341\272\243:", nullptr));
        label_7->setText(QCoreApplication::translate("MainWindow", "G\303\263c ng\341\272\251ng cao nh\341\272\245t:", nullptr));
        label_3->setText(QCoreApplication::translate("MainWindow", "G\303\263c xoay tr\303\241i ph\341\272\243i:", nullptr));
        label_5->setText(QCoreApplication::translate("MainWindow", "T\341\273\233i h\341\272\241n ph\341\272\243i:", nullptr));
        label_roll_min->setText(QCoreApplication::translate("MainWindow", "--", nullptr));
        label_6->setText(QCoreApplication::translate("MainWindow", "G\303\263c c\303\272i th\341\272\245p nh\341\272\245t:", nullptr));
        label_4->setText(QCoreApplication::translate("MainWindow", "T\341\273\233i h\341\272\241n tr\303\241i:", nullptr));
        label_2->setText(QCoreApplication::translate("MainWindow", "G\303\263c ng\341\272\251ng hi\341\273\207n t\341\272\241i:", nullptr));
        label_pitch_min->setText(QCoreApplication::translate("MainWindow", "--", nullptr));
        label_roll->setText(QCoreApplication::translate("MainWindow", "--", nullptr));
        label_pitch->setText(QCoreApplication::translate("MainWindow", "--", nullptr));
        label_pitch_max->setText(QCoreApplication::translate("MainWindow", "--", nullptr));
        label_roll_max->setText(QCoreApplication::translate("MainWindow", "--", nullptr));
        pushButton_5->setText(QCoreApplication::translate("MainWindow", "M\341\273\237 file b\303\241o c\303\241o", nullptr));
        pushButton_3->setText(QCoreApplication::translate("MainWindow", "L\306\260u k\341\272\277t qu\341\272\243", nullptr));
        label_11->setText(QCoreApplication::translate("MainWindow", "L\341\272\247n \304\221o:", nullptr));
        label_mes_count->setText(QCoreApplication::translate("MainWindow", "--", nullptr));
        pushButton->setText(QCoreApplication::translate("MainWindow", "B\341\272\257t \304\221\341\272\247u \304\221o", nullptr));
        pushButton_2->setText(QCoreApplication::translate("MainWindow", "D\341\273\253ng", nullptr));
        pushButton_4->setText(QCoreApplication::translate("MainWindow", "\304\220o l\341\272\241i t\341\273\253 \304\221\341\272\247u", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
