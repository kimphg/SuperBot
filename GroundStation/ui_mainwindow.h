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
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QWidget>
#include <c_evironment_widget.h>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionConnect;
    QAction *actionDisconnect;
    QWidget *centralwidget;
    QGridLayout *gridLayout_4;
    QTabWidget *tabWidget;
    QWidget *tab;
    QGridLayout *gridLayout_3;
    QTextBrowser *textBrowser;
    QPushButton *pushButton_8;
    QPushButton *pushButton_9;
    QTableWidget *tableWidget_2;
    QPushButton *pushButton_stop_4;
    QWidget *tab_2;
    QGridLayout *gridLayout_2;
    c_evironment_widget *frame_view;
    QTableWidget *tableWidget;
    QGroupBox *groupBox;
    QGridLayout *gridLayout;
    QGroupBox *groupBox_2;
    QFormLayout *formLayout;
    QLineEdit *lineEdit_des_x;
    QLabel *label_connection_fps_2;
    QLineEdit *lineEdit_des_y;
    QLabel *label_connection_fps_3;
    QLineEdit *lineEdit_des_id;
    QLabel *label_connection_fps_4;
    QLabel *label_des_a;
    QSlider *horizontalSlider_2;
    QPushButton *pushButton_send_command;
    QPushButton *pushButton_lift_before;
    QPushButton *pushButton_lift_after;
    QLabel *label_connection_stat;
    QPushButton *pushButton_stop_3;
    QLabel *label_tag_id_2;
    QLabel *label_5;
    QLineEdit *lineEdit_msg_2;
    QPushButton *pushButton_5;
    QPushButton *pushButton_11;
    QPushButton *pushButton_stop;
    QLabel *label_connection_stat_2;
    QPushButton *pushButton_7;
    QLabel *label_tag_id_3;
    QPushButton *pushButton_stop_2;
    QPushButton *pushButton_2;
    QLabel *label_connection_last_msg;
    QLabel *label_speed_L;
    QLabel *label_tag_id;
    QPushButton *pushButton_10;
    QPushButton *pushButton_4;
    QLabel *label_connection_fps;
    QPushButton *pushButton_down;
    QPushButton *pushButton_up;
    QLabel *label_3;
    QLabel *label_4;
    QLabel *label_tag_id_4;
    QLabel *label_speed_R;
    QPushButton *pushButton_6;
    QLineEdit *lineEdit_msg_1;
    QPushButton *pushButton;
    QLabel *label_2;
    QLabel *label;
    QPushButton *pushButton_3;
    QComboBox *comboBox_robot_select;
    QMenuBar *menubar;
    QMenu *menuFile;
    QMenu *menuView;
    QMenu *menuTool;
    QMenu *menuOthers;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1836, 1010);
        actionConnect = new QAction(MainWindow);
        actionConnect->setObjectName(QString::fromUtf8("actionConnect"));
        actionDisconnect = new QAction(MainWindow);
        actionDisconnect->setObjectName(QString::fromUtf8("actionDisconnect"));
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        gridLayout_4 = new QGridLayout(centralwidget);
        gridLayout_4->setObjectName(QString::fromUtf8("gridLayout_4"));
        tabWidget = new QTabWidget(centralwidget);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        gridLayout_3 = new QGridLayout(tab);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        textBrowser = new QTextBrowser(tab);
        textBrowser->setObjectName(QString::fromUtf8("textBrowser"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(textBrowser->sizePolicy().hasHeightForWidth());
        textBrowser->setSizePolicy(sizePolicy);

        gridLayout_3->addWidget(textBrowser, 1, 3, 1, 1);

        pushButton_8 = new QPushButton(tab);
        pushButton_8->setObjectName(QString::fromUtf8("pushButton_8"));

        gridLayout_3->addWidget(pushButton_8, 3, 3, 1, 1);

        pushButton_9 = new QPushButton(tab);
        pushButton_9->setObjectName(QString::fromUtf8("pushButton_9"));

        gridLayout_3->addWidget(pushButton_9, 2, 3, 1, 1);

        tableWidget_2 = new QTableWidget(tab);
        if (tableWidget_2->columnCount() < 2)
            tableWidget_2->setColumnCount(2);
        QTableWidgetItem *__qtablewidgetitem = new QTableWidgetItem();
        tableWidget_2->setHorizontalHeaderItem(0, __qtablewidgetitem);
        QTableWidgetItem *__qtablewidgetitem1 = new QTableWidgetItem();
        tableWidget_2->setHorizontalHeaderItem(1, __qtablewidgetitem1);
        tableWidget_2->setObjectName(QString::fromUtf8("tableWidget_2"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Expanding);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(tableWidget_2->sizePolicy().hasHeightForWidth());
        tableWidget_2->setSizePolicy(sizePolicy1);
        tableWidget_2->setSortingEnabled(true);
        tableWidget_2->horizontalHeader()->setVisible(false);
        tableWidget_2->horizontalHeader()->setCascadingSectionResizes(false);
        tableWidget_2->horizontalHeader()->setMinimumSectionSize(30);
        tableWidget_2->horizontalHeader()->setDefaultSectionSize(200);
        tableWidget_2->horizontalHeader()->setHighlightSections(true);
        tableWidget_2->horizontalHeader()->setStretchLastSection(true);

        gridLayout_3->addWidget(tableWidget_2, 5, 3, 4, 1);

        pushButton_stop_4 = new QPushButton(tab);
        pushButton_stop_4->setObjectName(QString::fromUtf8("pushButton_stop_4"));

        gridLayout_3->addWidget(pushButton_stop_4, 4, 3, 1, 1);

        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        gridLayout_2 = new QGridLayout(tab_2);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        frame_view = new c_evironment_widget(tab_2);
        frame_view->setObjectName(QString::fromUtf8("frame_view"));
        QSizePolicy sizePolicy2(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(frame_view->sizePolicy().hasHeightForWidth());
        frame_view->setSizePolicy(sizePolicy2);
        frame_view->setMinimumSize(QSize(0, 0));
        frame_view->setFrameShape(QFrame::NoFrame);
        frame_view->setFrameShadow(QFrame::Plain);
        frame_view->setLineWidth(0);

        gridLayout_2->addWidget(frame_view, 0, 1, 1, 1);

        tableWidget = new QTableWidget(tab_2);
        if (tableWidget->columnCount() < 2)
            tableWidget->setColumnCount(2);
        QTableWidgetItem *__qtablewidgetitem2 = new QTableWidgetItem();
        tableWidget->setHorizontalHeaderItem(0, __qtablewidgetitem2);
        QTableWidgetItem *__qtablewidgetitem3 = new QTableWidgetItem();
        tableWidget->setHorizontalHeaderItem(1, __qtablewidgetitem3);
        tableWidget->setObjectName(QString::fromUtf8("tableWidget"));
        QSizePolicy sizePolicy3(QSizePolicy::Minimum, QSizePolicy::Expanding);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(tableWidget->sizePolicy().hasHeightForWidth());
        tableWidget->setSizePolicy(sizePolicy3);
        tableWidget->setMinimumSize(QSize(200, 0));
        tableWidget->setMaximumSize(QSize(350, 16777215));
        tableWidget->setSortingEnabled(true);
        tableWidget->horizontalHeader()->setVisible(false);
        tableWidget->horizontalHeader()->setCascadingSectionResizes(false);
        tableWidget->horizontalHeader()->setMinimumSectionSize(30);
        tableWidget->horizontalHeader()->setDefaultSectionSize(200);
        tableWidget->horizontalHeader()->setHighlightSections(true);
        tableWidget->horizontalHeader()->setStretchLastSection(true);

        gridLayout_2->addWidget(tableWidget, 0, 0, 1, 1);

        tabWidget->addTab(tab_2, QString());

        gridLayout_4->addWidget(tabWidget, 0, 1, 3, 1);

        groupBox = new QGroupBox(centralwidget);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        QSizePolicy sizePolicy4(QSizePolicy::Fixed, QSizePolicy::Preferred);
        sizePolicy4.setHorizontalStretch(0);
        sizePolicy4.setVerticalStretch(0);
        sizePolicy4.setHeightForWidth(groupBox->sizePolicy().hasHeightForWidth());
        groupBox->setSizePolicy(sizePolicy4);
        groupBox->setMinimumSize(QSize(400, 0));
        gridLayout = new QGridLayout(groupBox);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        groupBox_2 = new QGroupBox(groupBox);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        formLayout = new QFormLayout(groupBox_2);
        formLayout->setObjectName(QString::fromUtf8("formLayout"));
        lineEdit_des_x = new QLineEdit(groupBox_2);
        lineEdit_des_x->setObjectName(QString::fromUtf8("lineEdit_des_x"));

        formLayout->setWidget(0, QFormLayout::FieldRole, lineEdit_des_x);

        label_connection_fps_2 = new QLabel(groupBox_2);
        label_connection_fps_2->setObjectName(QString::fromUtf8("label_connection_fps_2"));

        formLayout->setWidget(1, QFormLayout::LabelRole, label_connection_fps_2);

        lineEdit_des_y = new QLineEdit(groupBox_2);
        lineEdit_des_y->setObjectName(QString::fromUtf8("lineEdit_des_y"));

        formLayout->setWidget(1, QFormLayout::FieldRole, lineEdit_des_y);

        label_connection_fps_3 = new QLabel(groupBox_2);
        label_connection_fps_3->setObjectName(QString::fromUtf8("label_connection_fps_3"));

        formLayout->setWidget(2, QFormLayout::LabelRole, label_connection_fps_3);

        lineEdit_des_id = new QLineEdit(groupBox_2);
        lineEdit_des_id->setObjectName(QString::fromUtf8("lineEdit_des_id"));

        formLayout->setWidget(2, QFormLayout::FieldRole, lineEdit_des_id);

        label_connection_fps_4 = new QLabel(groupBox_2);
        label_connection_fps_4->setObjectName(QString::fromUtf8("label_connection_fps_4"));

        formLayout->setWidget(3, QFormLayout::LabelRole, label_connection_fps_4);

        label_des_a = new QLabel(groupBox_2);
        label_des_a->setObjectName(QString::fromUtf8("label_des_a"));

        formLayout->setWidget(3, QFormLayout::FieldRole, label_des_a);

        horizontalSlider_2 = new QSlider(groupBox_2);
        horizontalSlider_2->setObjectName(QString::fromUtf8("horizontalSlider_2"));
        horizontalSlider_2->setMinimum(-1);
        horizontalSlider_2->setMaximum(2);
        horizontalSlider_2->setPageStep(1);
        horizontalSlider_2->setOrientation(Qt::Horizontal);
        horizontalSlider_2->setTickPosition(QSlider::TicksAbove);

        formLayout->setWidget(4, QFormLayout::FieldRole, horizontalSlider_2);

        pushButton_send_command = new QPushButton(groupBox_2);
        pushButton_send_command->setObjectName(QString::fromUtf8("pushButton_send_command"));

        formLayout->setWidget(6, QFormLayout::SpanningRole, pushButton_send_command);

        pushButton_lift_before = new QPushButton(groupBox_2);
        pushButton_lift_before->setObjectName(QString::fromUtf8("pushButton_lift_before"));
        pushButton_lift_before->setCheckable(true);

        formLayout->setWidget(5, QFormLayout::LabelRole, pushButton_lift_before);

        pushButton_lift_after = new QPushButton(groupBox_2);
        pushButton_lift_after->setObjectName(QString::fromUtf8("pushButton_lift_after"));
        pushButton_lift_after->setCheckable(true);

        formLayout->setWidget(5, QFormLayout::FieldRole, pushButton_lift_after);

        label_connection_stat = new QLabel(groupBox_2);
        label_connection_stat->setObjectName(QString::fromUtf8("label_connection_stat"));

        formLayout->setWidget(0, QFormLayout::LabelRole, label_connection_stat);


        gridLayout->addWidget(groupBox_2, 11, 0, 1, 5);

        pushButton_stop_3 = new QPushButton(groupBox);
        pushButton_stop_3->setObjectName(QString::fromUtf8("pushButton_stop_3"));

        gridLayout->addWidget(pushButton_stop_3, 9, 3, 1, 1);

        label_tag_id_2 = new QLabel(groupBox);
        label_tag_id_2->setObjectName(QString::fromUtf8("label_tag_id_2"));

        gridLayout->addWidget(label_tag_id_2, 4, 1, 1, 1);

        label_5 = new QLabel(groupBox);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        gridLayout->addWidget(label_5, 10, 0, 1, 1);

        lineEdit_msg_2 = new QLineEdit(groupBox);
        lineEdit_msg_2->setObjectName(QString::fromUtf8("lineEdit_msg_2"));

        gridLayout->addWidget(lineEdit_msg_2, 13, 2, 1, 3);

        pushButton_5 = new QPushButton(groupBox);
        pushButton_5->setObjectName(QString::fromUtf8("pushButton_5"));

        gridLayout->addWidget(pushButton_5, 4, 2, 1, 1);

        pushButton_11 = new QPushButton(groupBox);
        pushButton_11->setObjectName(QString::fromUtf8("pushButton_11"));

        gridLayout->addWidget(pushButton_11, 2, 3, 1, 1);

        pushButton_stop = new QPushButton(groupBox);
        pushButton_stop->setObjectName(QString::fromUtf8("pushButton_stop"));

        gridLayout->addWidget(pushButton_stop, 7, 2, 1, 1);

        label_connection_stat_2 = new QLabel(groupBox);
        label_connection_stat_2->setObjectName(QString::fromUtf8("label_connection_stat_2"));

        gridLayout->addWidget(label_connection_stat_2, 1, 0, 1, 1);

        pushButton_7 = new QPushButton(groupBox);
        pushButton_7->setObjectName(QString::fromUtf8("pushButton_7"));

        gridLayout->addWidget(pushButton_7, 7, 3, 1, 1);

        label_tag_id_3 = new QLabel(groupBox);
        label_tag_id_3->setObjectName(QString::fromUtf8("label_tag_id_3"));

        gridLayout->addWidget(label_tag_id_3, 5, 1, 1, 1);

        pushButton_stop_2 = new QPushButton(groupBox);
        pushButton_stop_2->setObjectName(QString::fromUtf8("pushButton_stop_2"));

        gridLayout->addWidget(pushButton_stop_2, 9, 0, 1, 1);

        pushButton_2 = new QPushButton(groupBox);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));

        gridLayout->addWidget(pushButton_2, 2, 2, 1, 1);

        label_connection_last_msg = new QLabel(groupBox);
        label_connection_last_msg->setObjectName(QString::fromUtf8("label_connection_last_msg"));

        gridLayout->addWidget(label_connection_last_msg, 1, 2, 1, 3);

        label_speed_L = new QLabel(groupBox);
        label_speed_L->setObjectName(QString::fromUtf8("label_speed_L"));

        gridLayout->addWidget(label_speed_L, 10, 1, 1, 1);

        label_tag_id = new QLabel(groupBox);
        label_tag_id->setObjectName(QString::fromUtf8("label_tag_id"));

        gridLayout->addWidget(label_tag_id, 2, 1, 1, 1);

        pushButton_10 = new QPushButton(groupBox);
        pushButton_10->setObjectName(QString::fromUtf8("pushButton_10"));

        gridLayout->addWidget(pushButton_10, 7, 1, 1, 1);

        pushButton_4 = new QPushButton(groupBox);
        pushButton_4->setObjectName(QString::fromUtf8("pushButton_4"));

        gridLayout->addWidget(pushButton_4, 6, 2, 1, 1);

        label_connection_fps = new QLabel(groupBox);
        label_connection_fps->setObjectName(QString::fromUtf8("label_connection_fps"));

        gridLayout->addWidget(label_connection_fps, 1, 1, 1, 1);

        pushButton_down = new QPushButton(groupBox);
        pushButton_down->setObjectName(QString::fromUtf8("pushButton_down"));

        gridLayout->addWidget(pushButton_down, 9, 2, 1, 1);

        pushButton_up = new QPushButton(groupBox);
        pushButton_up->setObjectName(QString::fromUtf8("pushButton_up"));

        gridLayout->addWidget(pushButton_up, 9, 1, 1, 1);

        label_3 = new QLabel(groupBox);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        gridLayout->addWidget(label_3, 5, 0, 1, 1);

        label_4 = new QLabel(groupBox);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        gridLayout->addWidget(label_4, 6, 0, 1, 1);

        label_tag_id_4 = new QLabel(groupBox);
        label_tag_id_4->setObjectName(QString::fromUtf8("label_tag_id_4"));

        gridLayout->addWidget(label_tag_id_4, 6, 1, 1, 1);

        label_speed_R = new QLabel(groupBox);
        label_speed_R->setObjectName(QString::fromUtf8("label_speed_R"));

        gridLayout->addWidget(label_speed_R, 10, 2, 1, 1);

        pushButton_6 = new QPushButton(groupBox);
        pushButton_6->setObjectName(QString::fromUtf8("pushButton_6"));

        gridLayout->addWidget(pushButton_6, 13, 0, 1, 1);

        lineEdit_msg_1 = new QLineEdit(groupBox);
        lineEdit_msg_1->setObjectName(QString::fromUtf8("lineEdit_msg_1"));

        gridLayout->addWidget(lineEdit_msg_1, 12, 2, 1, 3);

        pushButton = new QPushButton(groupBox);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));

        gridLayout->addWidget(pushButton, 12, 0, 1, 1);

        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout->addWidget(label_2, 4, 0, 1, 1);

        label = new QLabel(groupBox);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout->addWidget(label, 2, 0, 1, 1);

        pushButton_3 = new QPushButton(groupBox);
        pushButton_3->setObjectName(QString::fromUtf8("pushButton_3"));

        gridLayout->addWidget(pushButton_3, 8, 2, 1, 1);

        comboBox_robot_select = new QComboBox(groupBox);
        comboBox_robot_select->setObjectName(QString::fromUtf8("comboBox_robot_select"));

        gridLayout->addWidget(comboBox_robot_select, 0, 1, 1, 3);


        gridLayout_4->addWidget(groupBox, 0, 0, 1, 1);

        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 1836, 21));
        menuFile = new QMenu(menubar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        menuView = new QMenu(menubar);
        menuView->setObjectName(QString::fromUtf8("menuView"));
        menuTool = new QMenu(menubar);
        menuTool->setObjectName(QString::fromUtf8("menuTool"));
        menuOthers = new QMenu(menubar);
        menuOthers->setObjectName(QString::fromUtf8("menuOthers"));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        menubar->addAction(menuFile->menuAction());
        menubar->addAction(menuView->menuAction());
        menubar->addAction(menuTool->menuAction());
        menubar->addAction(menuOthers->menuAction());
        menuTool->addAction(actionConnect);
        menuTool->addAction(actionDisconnect);

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "GCS", nullptr));
        actionConnect->setText(QCoreApplication::translate("MainWindow", "Connect", nullptr));
        actionDisconnect->setText(QCoreApplication::translate("MainWindow", "Disconnect", nullptr));
        pushButton_8->setText(QCoreApplication::translate("MainWindow", "Reset data", nullptr));
        pushButton_9->setText(QCoreApplication::translate("MainWindow", "Upload param", nullptr));
        QTableWidgetItem *___qtablewidgetitem = tableWidget_2->horizontalHeaderItem(0);
        ___qtablewidgetitem->setText(QCoreApplication::translate("MainWindow", "Tham s\341\273\221", nullptr));
        QTableWidgetItem *___qtablewidgetitem1 = tableWidget_2->horizontalHeaderItem(1);
        ___qtablewidgetitem1->setText(QCoreApplication::translate("MainWindow", "Gi\303\241 tr\341\273\213", nullptr));
        pushButton_stop_4->setText(QCoreApplication::translate("MainWindow", "GET PARAMS", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab), QCoreApplication::translate("MainWindow", "Setup", nullptr));
        QTableWidgetItem *___qtablewidgetitem2 = tableWidget->horizontalHeaderItem(0);
        ___qtablewidgetitem2->setText(QCoreApplication::translate("MainWindow", "Tham s\341\273\221", nullptr));
        QTableWidgetItem *___qtablewidgetitem3 = tableWidget->horizontalHeaderItem(1);
        ___qtablewidgetitem3->setText(QCoreApplication::translate("MainWindow", "Gi\303\241 tr\341\273\213", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QCoreApplication::translate("MainWindow", "Management", nullptr));
        groupBox->setTitle(QCoreApplication::translate("MainWindow", "GroupBox", nullptr));
        groupBox_2->setTitle(QCoreApplication::translate("MainWindow", "GroupBox", nullptr));
        lineEdit_des_x->setText(QCoreApplication::translate("MainWindow", "0", nullptr));
        label_connection_fps_2->setText(QCoreApplication::translate("MainWindow", "Des Y", nullptr));
        lineEdit_des_y->setText(QCoreApplication::translate("MainWindow", "0", nullptr));
        label_connection_fps_3->setText(QCoreApplication::translate("MainWindow", "Des ID", nullptr));
        lineEdit_des_id->setText(QCoreApplication::translate("MainWindow", "0", nullptr));
        label_connection_fps_4->setText(QCoreApplication::translate("MainWindow", "Des Angle", nullptr));
        label_des_a->setText(QCoreApplication::translate("MainWindow", "--", nullptr));
        pushButton_send_command->setText(QCoreApplication::translate("MainWindow", "Send", nullptr));
        pushButton_lift_before->setText(QCoreApplication::translate("MainWindow", "Lift", nullptr));
        pushButton_lift_after->setText(QCoreApplication::translate("MainWindow", "Lift", nullptr));
        label_connection_stat->setText(QCoreApplication::translate("MainWindow", "Des X", nullptr));
        pushButton_stop_3->setText(QCoreApplication::translate("MainWindow", "HOME", nullptr));
        label_tag_id_2->setText(QCoreApplication::translate("MainWindow", "--", nullptr));
        label_5->setText(QCoreApplication::translate("MainWindow", "Speed L-R", nullptr));
        lineEdit_msg_2->setText(QCoreApplication::translate("MainWindow", "$COM,m,4,5,", nullptr));
        pushButton_5->setText(QCoreApplication::translate("MainWindow", "FORWARD 2", nullptr));
        pushButton_11->setText(QCoreApplication::translate("MainWindow", "FORWARD 0.7", nullptr));
        pushButton_stop->setText(QCoreApplication::translate("MainWindow", "STOP", nullptr));
        label_connection_stat_2->setText(QCoreApplication::translate("MainWindow", "Connection FPS", nullptr));
        pushButton_7->setText(QCoreApplication::translate("MainWindow", "3H", nullptr));
        label_tag_id_3->setText(QCoreApplication::translate("MainWindow", "--", nullptr));
        pushButton_stop_2->setText(QCoreApplication::translate("MainWindow", "RESET POSITION", nullptr));
        pushButton_2->setText(QCoreApplication::translate("MainWindow", "FORWARD 1", nullptr));
        label_connection_last_msg->setText(QCoreApplication::translate("MainWindow", "--", nullptr));
        label_speed_L->setText(QCoreApplication::translate("MainWindow", "--", nullptr));
        label_tag_id->setText(QCoreApplication::translate("MainWindow", "--", nullptr));
        pushButton_10->setText(QCoreApplication::translate("MainWindow", "9H", nullptr));
        pushButton_4->setText(QCoreApplication::translate("MainWindow", "0H", nullptr));
        label_connection_fps->setText(QCoreApplication::translate("MainWindow", "--", nullptr));
        pushButton_down->setText(QCoreApplication::translate("MainWindow", "DOWN", nullptr));
        pushButton_up->setText(QCoreApplication::translate("MainWindow", "UP", nullptr));
        label_3->setText(QCoreApplication::translate("MainWindow", "Robot status:", nullptr));
        label_4->setText(QCoreApplication::translate("MainWindow", "Warning level", nullptr));
        label_tag_id_4->setText(QCoreApplication::translate("MainWindow", "--", nullptr));
        label_speed_R->setText(QCoreApplication::translate("MainWindow", "--", nullptr));
        pushButton_6->setText(QCoreApplication::translate("MainWindow", "Custom Message 2", nullptr));
        lineEdit_msg_1->setText(QCoreApplication::translate("MainWindow", "aa558300ab0101", nullptr));
        pushButton->setText(QCoreApplication::translate("MainWindow", "Custom Message1", nullptr));
        label_2->setText(QCoreApplication::translate("MainWindow", "Tag lost time:", nullptr));
        label->setText(QCoreApplication::translate("MainWindow", "Tag ID:", nullptr));
        pushButton_3->setText(QCoreApplication::translate("MainWindow", "6H", nullptr));
        menuFile->setTitle(QCoreApplication::translate("MainWindow", "File", nullptr));
        menuView->setTitle(QCoreApplication::translate("MainWindow", "View", nullptr));
        menuTool->setTitle(QCoreApplication::translate("MainWindow", "Tool", nullptr));
        menuOthers->setTitle(QCoreApplication::translate("MainWindow", "Others", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
