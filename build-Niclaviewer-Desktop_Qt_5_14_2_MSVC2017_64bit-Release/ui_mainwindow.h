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
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QGridLayout *gridLayout;
    QLabel *label_img;
    QPushButton *pushButton;
    QPushButton *pushButton_2;
    QLabel *label;
    QGroupBox *groupBox;
    QGridLayout *gridLayout_2;
    QLabel *label_8;
    QLineEdit *lineEdit;
    QLineEdit *lineEdit_4;
    QLineEdit *lineEdit_2;
    QLabel *label_11;
    QLineEdit *lineEdit_3;
    QLabel *label_9;
    QLabel *label_10;
    QLabel *label_12;
    QLineEdit *lineEdit_5;
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
    QPushButton *pushButton_3;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1488, 1017);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        gridLayout = new QGridLayout(centralwidget);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        label_img = new QLabel(centralwidget);
        label_img->setObjectName(QString::fromUtf8("label_img"));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(label_img->sizePolicy().hasHeightForWidth());
        label_img->setSizePolicy(sizePolicy);
        label_img->setMinimumSize(QSize(320, 240));

        gridLayout->addWidget(label_img, 0, 1, 1, 1);

        pushButton = new QPushButton(centralwidget);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));

        gridLayout->addWidget(pushButton, 3, 0, 1, 1);

        pushButton_2 = new QPushButton(centralwidget);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));

        gridLayout->addWidget(pushButton_2, 4, 0, 1, 1);

        label = new QLabel(centralwidget);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout->addWidget(label, 1, 2, 1, 6);

        groupBox = new QGroupBox(centralwidget);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        gridLayout_2 = new QGridLayout(groupBox);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        label_8 = new QLabel(groupBox);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        gridLayout_2->addWidget(label_8, 0, 0, 1, 1);

        lineEdit = new QLineEdit(groupBox);
        lineEdit->setObjectName(QString::fromUtf8("lineEdit"));

        gridLayout_2->addWidget(lineEdit, 0, 2, 1, 1);

        lineEdit_4 = new QLineEdit(groupBox);
        lineEdit_4->setObjectName(QString::fromUtf8("lineEdit_4"));

        gridLayout_2->addWidget(lineEdit_4, 3, 2, 1, 1);

        lineEdit_2 = new QLineEdit(groupBox);
        lineEdit_2->setObjectName(QString::fromUtf8("lineEdit_2"));

        gridLayout_2->addWidget(lineEdit_2, 1, 2, 1, 1);

        label_11 = new QLabel(groupBox);
        label_11->setObjectName(QString::fromUtf8("label_11"));

        gridLayout_2->addWidget(label_11, 3, 0, 1, 1);

        lineEdit_3 = new QLineEdit(groupBox);
        lineEdit_3->setObjectName(QString::fromUtf8("lineEdit_3"));

        gridLayout_2->addWidget(lineEdit_3, 2, 2, 1, 1);

        label_9 = new QLabel(groupBox);
        label_9->setObjectName(QString::fromUtf8("label_9"));

        gridLayout_2->addWidget(label_9, 1, 0, 1, 1);

        label_10 = new QLabel(groupBox);
        label_10->setObjectName(QString::fromUtf8("label_10"));

        gridLayout_2->addWidget(label_10, 2, 0, 1, 1);

        label_12 = new QLabel(groupBox);
        label_12->setObjectName(QString::fromUtf8("label_12"));

        gridLayout_2->addWidget(label_12, 4, 0, 1, 1);

        lineEdit_5 = new QLineEdit(groupBox);
        lineEdit_5->setObjectName(QString::fromUtf8("lineEdit_5"));

        gridLayout_2->addWidget(lineEdit_5, 4, 2, 1, 1);


        gridLayout->addWidget(groupBox, 5, 0, 3, 2);

        groupBox_2 = new QGroupBox(centralwidget);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        gridLayout_3 = new QGridLayout(groupBox_2);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
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
        label_2->setStyleSheet(QString::fromUtf8("border-color: rgb(0, 0, 0);"));

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

        pushButton_3 = new QPushButton(groupBox_2);
        pushButton_3->setObjectName(QString::fromUtf8("pushButton_3"));

        gridLayout_3->addWidget(pushButton_3, 2, 1, 1, 3);


        gridLayout->addWidget(groupBox_2, 3, 2, 5, 4);

        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 1488, 21));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        label_img->setText(QCoreApplication::translate("MainWindow", "video", nullptr));
        pushButton->setText(QCoreApplication::translate("MainWindow", "B\341\272\257t \304\221\341\272\247u \304\221o", nullptr));
        pushButton_2->setText(QCoreApplication::translate("MainWindow", "D\341\273\253ng", nullptr));
        label->setText(QCoreApplication::translate("MainWindow", "Thi\341\272\277t b\341\273\213 \304\221o \304\221\306\260\341\273\243c \304\221eo l\303\252n m\341\272\257t b\341\273\207nh nh\303\242n sao cho h\303\254nh \341\272\243nh camera tr\303\271ng kh\341\273\233p v\341\273\233i h\306\260\341\273\233ng nh\303\254n ch\303\255nh di\341\273\207n\n"
" c\341\273\247a  b\341\273\207nh nh\303\242n.\n"
"Ng\306\260\341\273\235i ki\341\273\203m tra \304\221\341\273\251ng ph\303\255a sau b\341\273\207nh nh\303\242n \304\221\341\273\203 h\306\260\341\273\233ng d\341\272\253n v\303\240 gi\303\241m s\303\241t.\n"
"B\341\273\207nh nh\303\242n l\341\272\247n l\306\260\341\273\243t th\341\273\261c hi\341\273\207n c\303\241c \304\221\341\273\231ng t\303\241c c\303\272i, ng\341\272\251ng, xoay tr\303\241i v\303\240 xoay ph\341\272\243i \304\221\341\272\277n t\341\273\233i h\341\272\241n cho ph\303\251p.\n"
"Ng\306\260\341\273\235i ki\341\273\203m tra nh\341\272\245n n\303\272t ho\303\240n th\303\240nh \304\221\341\273\203 l\306\260u k\341\272\277t qu\341\272\243 \304\221o.", nullptr));
        groupBox->setTitle(QCoreApplication::translate("MainWindow", "Th\303\264ng tin b\341\273\207nh nh\303\242n", nullptr));
        label_8->setText(QCoreApplication::translate("MainWindow", "T\303\252n BN:", nullptr));
        label_11->setText(QCoreApplication::translate("MainWindow", "L\341\272\247n \304\221o:", nullptr));
        label_9->setText(QCoreApplication::translate("MainWindow", "M\303\243 BN:", nullptr));
        label_10->setText(QCoreApplication::translate("MainWindow", "SN:", nullptr));
        label_12->setText(QCoreApplication::translate("MainWindow", "Ng\303\240y \304\221o:", nullptr));
        groupBox_2->setTitle(QCoreApplication::translate("MainWindow", "GroupBox", nullptr));
        label_7->setText(QCoreApplication::translate("MainWindow", "G\303\263c ng\341\272\251ng cao nh\341\272\245t:", nullptr));
        label_3->setText(QCoreApplication::translate("MainWindow", "G\303\263c xoay tr\303\241i ph\341\272\243i:", nullptr));
        label_5->setText(QCoreApplication::translate("MainWindow", "T\341\273\233i h\341\272\241n ph\341\272\243i:", nullptr));
        label_roll_min->setText(QCoreApplication::translate("MainWindow", "--", nullptr));
        label_6->setText(QCoreApplication::translate("MainWindow", "G\303\263c c\303\272i th\341\272\245p nh\341\272\245ti:", nullptr));
        label_4->setText(QCoreApplication::translate("MainWindow", "T\341\273\233i h\341\272\241n tr\303\241i:", nullptr));
        label_2->setText(QCoreApplication::translate("MainWindow", "G\303\263c ng\341\272\251ng hi\341\273\207n t\341\272\241i:", nullptr));
        label_pitch_min->setText(QCoreApplication::translate("MainWindow", "--", nullptr));
        label_roll->setText(QCoreApplication::translate("MainWindow", "--", nullptr));
        label_pitch->setText(QCoreApplication::translate("MainWindow", "--", nullptr));
        label_pitch_max->setText(QCoreApplication::translate("MainWindow", "--", nullptr));
        label_roll_max->setText(QCoreApplication::translate("MainWindow", "--", nullptr));
        pushButton_3->setText(QCoreApplication::translate("MainWindow", "L\306\260u k\341\272\277t qu\341\272\243", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
