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
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>
#include <mainwidget.h>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QGridLayout *gridLayout;
    QLabel *label_pitch;
    QLabel *label_5;
    MainWidget *openGLWidget;
    QLabel *label_6;
    QLabel *label_pitch_max;
    QLabel *label_3;
    QLabel *label_4;
    QLabel *label_2;
    QLabel *label_roll_max;
    QLabel *label_7;
    QLabel *label;
    QLabel *label_img;
    QLabel *label_pitch_min;
    QLabel *label_roll_min;
    QLabel *label_roll;
    QPushButton *pushButton;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1484, 717);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        gridLayout = new QGridLayout(centralwidget);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        label_pitch = new QLabel(centralwidget);
        label_pitch->setObjectName(QString::fromUtf8("label_pitch"));

        gridLayout->addWidget(label_pitch, 2, 3, 1, 1);

        label_5 = new QLabel(centralwidget);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        gridLayout->addWidget(label_5, 3, 6, 1, 1);

        openGLWidget = new MainWidget(centralwidget);
        openGLWidget->setObjectName(QString::fromUtf8("openGLWidget"));
        openGLWidget->setMinimumSize(QSize(600, 600));

        gridLayout->addWidget(openGLWidget, 0, 2, 1, 6);

        label_6 = new QLabel(centralwidget);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        gridLayout->addWidget(label_6, 2, 4, 1, 1);

        label_pitch_max = new QLabel(centralwidget);
        label_pitch_max->setObjectName(QString::fromUtf8("label_pitch_max"));

        gridLayout->addWidget(label_pitch_max, 2, 7, 1, 1);

        label_3 = new QLabel(centralwidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        gridLayout->addWidget(label_3, 3, 2, 1, 1);

        label_4 = new QLabel(centralwidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        gridLayout->addWidget(label_4, 3, 4, 1, 1);

        label_2 = new QLabel(centralwidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setStyleSheet(QString::fromUtf8("border-color: rgb(0, 0, 0);"));

        gridLayout->addWidget(label_2, 2, 2, 1, 1);

        label_roll_max = new QLabel(centralwidget);
        label_roll_max->setObjectName(QString::fromUtf8("label_roll_max"));

        gridLayout->addWidget(label_roll_max, 3, 7, 1, 1);

        label_7 = new QLabel(centralwidget);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        gridLayout->addWidget(label_7, 2, 6, 1, 1);

        label = new QLabel(centralwidget);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout->addWidget(label, 2, 0, 1, 1);

        label_img = new QLabel(centralwidget);
        label_img->setObjectName(QString::fromUtf8("label_img"));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(label_img->sizePolicy().hasHeightForWidth());
        label_img->setSizePolicy(sizePolicy);
        label_img->setMinimumSize(QSize(640, 480));

        gridLayout->addWidget(label_img, 0, 1, 1, 1);

        label_pitch_min = new QLabel(centralwidget);
        label_pitch_min->setObjectName(QString::fromUtf8("label_pitch_min"));

        gridLayout->addWidget(label_pitch_min, 2, 5, 1, 1);

        label_roll_min = new QLabel(centralwidget);
        label_roll_min->setObjectName(QString::fromUtf8("label_roll_min"));

        gridLayout->addWidget(label_roll_min, 3, 5, 1, 1);

        label_roll = new QLabel(centralwidget);
        label_roll->setObjectName(QString::fromUtf8("label_roll"));

        gridLayout->addWidget(label_roll, 3, 3, 1, 1);

        pushButton = new QPushButton(centralwidget);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));

        gridLayout->addWidget(pushButton, 1, 2, 1, 1);

        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 1484, 20));
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
        label_pitch->setText(QCoreApplication::translate("MainWindow", "--", nullptr));
        label_5->setText(QCoreApplication::translate("MainWindow", "T\341\273\233i h\341\272\241n ph\341\272\243i:", nullptr));
        label_6->setText(QCoreApplication::translate("MainWindow", "T\341\273\233i h\341\272\241n c\303\272i:", nullptr));
        label_pitch_max->setText(QCoreApplication::translate("MainWindow", "--", nullptr));
        label_3->setText(QCoreApplication::translate("MainWindow", "G\303\263c xoay tr\303\241i ph\341\272\243i:", nullptr));
        label_4->setText(QCoreApplication::translate("MainWindow", "T\341\273\233i h\341\272\241n tr\303\241i:", nullptr));
        label_2->setText(QCoreApplication::translate("MainWindow", "G\303\263c ng\341\272\251ng:", nullptr));
        label_roll_max->setText(QCoreApplication::translate("MainWindow", "--", nullptr));
        label_7->setText(QCoreApplication::translate("MainWindow", "T\341\273\233i h\341\272\241n ng\341\272\251ng:", nullptr));
        label->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        label_img->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        label_pitch_min->setText(QCoreApplication::translate("MainWindow", "--", nullptr));
        label_roll_min->setText(QCoreApplication::translate("MainWindow", "--", nullptr));
        label_roll->setText(QCoreApplication::translate("MainWindow", "--", nullptr));
        pushButton->setText(QCoreApplication::translate("MainWindow", "B\341\272\257t \304\221\341\272\247u \304\221o", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
