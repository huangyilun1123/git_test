/********************************************************************************
** Form generated from reading UI file 'node_check.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_NODECHECK_H
#define UI_NODECHECK_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDialog>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>

QT_BEGIN_NAMESPACE

class Ui_Dialog_NodeCheck
{
public:
    QGroupBox *groupBox_2;
    QLabel *label_25;
    QLineEdit *edt_lidar_msg;
    QLabel *label_lidar_hb;
    QLineEdit *edt_radar_msg;
    QLabel *label_27;
    QLabel *label_radar_hb;
    QLineEdit *edt_ccd_msg;
    QLabel *label_ccd_hb;
    QLabel *label_29;
    QLineEdit *edt_imu_msg;
    QLabel *label_30;
    QLabel *label_imu_hb;
    QLineEdit *edt_gps_msg;
    QLabel *label_31;
    QLabel *label_gps_hb;
    QLineEdit *edt_scan1_msg;
    QLabel *label_32;
    QLabel *label_scan1_hb;
    QLineEdit *edt_scan2_msg;
    QLabel *label_scan2_hb;
    QLabel *label_33;
    QLineEdit *edt_scan3_msg;
    QLabel *label_scan3_hb;
    QLabel *label_34;
    QLineEdit *edt_scan4_msg;
    QLabel *label_35;
    QLabel *label_scan4_hb;
    QGroupBox *groupBox_4;
    QLabel *label_37;
    QLineEdit *edt_plc_msg;
    QLabel *label_plc_hb;
    QLineEdit *edt_mqtt_msg;
    QLabel *label_38;
    QLabel *label_mqtt_hb;
    QGroupBox *groupBox_5;
    QLabel *label_39;
    QLineEdit *edt_pathplan_msg;
    QLabel *label_pathplan_hb;
    QLineEdit *edt_pathtrack_msg;
    QLabel *label_pathtrack_hb;
    QLabel *label_41;
    QLineEdit *edt_work_msg;
    QLabel *label_work_hb;
    QLabel *label_42;
    QLineEdit *edt_ctrdecision_msg;
    QLabel *label_47;
    QLabel *label_ctr_decision_hb;
    QLineEdit *edt_turntable_msg;
    QLabel *label_48;
    QLabel *label_turntable_hb;
    QLineEdit *edt_scancheck_msg;
    QLabel *label_43;
    QLabel *label_scancheck_hb;

    void setupUi(QDialog *Dialog_NodeCheck)
    {
        if (Dialog_NodeCheck->objectName().isEmpty())
            Dialog_NodeCheck->setObjectName(QString::fromUtf8("Dialog_NodeCheck"));
        Dialog_NodeCheck->resize(1184, 533);
        groupBox_2 = new QGroupBox(Dialog_NodeCheck);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        groupBox_2->setGeometry(QRect(20, 20, 561, 481));
        QFont font;
        font.setPointSize(14);
        groupBox_2->setFont(font);
        groupBox_2->setStyleSheet(QString::fromUtf8("background-color: rgb(114, 159, 207);\n"
"gridline-color: rgb(0, 0, 0);"));
        label_25 = new QLabel(groupBox_2);
        label_25->setObjectName(QString::fromUtf8("label_25"));
        label_25->setGeometry(QRect(20, 87, 81, 31));
        label_25->setFont(font);
        edt_lidar_msg = new QLineEdit(groupBox_2);
        edt_lidar_msg->setObjectName(QString::fromUtf8("edt_lidar_msg"));
        edt_lidar_msg->setGeometry(QRect(200, 87, 341, 31));
        edt_lidar_msg->setFont(font);
        edt_lidar_msg->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
        edt_lidar_msg->setReadOnly(true);
        label_lidar_hb = new QLabel(groupBox_2);
        label_lidar_hb->setObjectName(QString::fromUtf8("label_lidar_hb"));
        label_lidar_hb->setGeometry(QRect(130, 87, 51, 31));
        label_lidar_hb->setFont(font);
        label_lidar_hb->setStyleSheet(QString::fromUtf8("color: rgb(238, 238, 236);\n"
"background-color: rgb(143, 89, 2);"));
        label_lidar_hb->setFrameShape(QFrame::Box);
        label_lidar_hb->setFrameShadow(QFrame::Raised);
        label_lidar_hb->setAlignment(Qt::AlignCenter);
        edt_radar_msg = new QLineEdit(groupBox_2);
        edt_radar_msg->setObjectName(QString::fromUtf8("edt_radar_msg"));
        edt_radar_msg->setGeometry(QRect(200, 134, 341, 30));
        edt_radar_msg->setFont(font);
        edt_radar_msg->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
        edt_radar_msg->setReadOnly(true);
        label_27 = new QLabel(groupBox_2);
        label_27->setObjectName(QString::fromUtf8("label_27"));
        label_27->setGeometry(QRect(20, 134, 101, 30));
        label_27->setFont(font);
        label_radar_hb = new QLabel(groupBox_2);
        label_radar_hb->setObjectName(QString::fromUtf8("label_radar_hb"));
        label_radar_hb->setGeometry(QRect(130, 134, 51, 30));
        label_radar_hb->setFont(font);
        label_radar_hb->setStyleSheet(QString::fromUtf8("color: rgb(238, 238, 236);\n"
"background-color: rgb(143, 89, 2);"));
        label_radar_hb->setFrameShape(QFrame::Box);
        label_radar_hb->setFrameShadow(QFrame::Raised);
        label_radar_hb->setAlignment(Qt::AlignCenter);
        edt_ccd_msg = new QLineEdit(groupBox_2);
        edt_ccd_msg->setObjectName(QString::fromUtf8("edt_ccd_msg"));
        edt_ccd_msg->setGeometry(QRect(200, 225, 341, 31));
        edt_ccd_msg->setFont(font);
        edt_ccd_msg->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
        edt_ccd_msg->setReadOnly(true);
        label_ccd_hb = new QLabel(groupBox_2);
        label_ccd_hb->setObjectName(QString::fromUtf8("label_ccd_hb"));
        label_ccd_hb->setGeometry(QRect(130, 225, 51, 31));
        label_ccd_hb->setFont(font);
        label_ccd_hb->setStyleSheet(QString::fromUtf8("color: rgb(238, 238, 236);\n"
"background-color: rgb(143, 89, 2);"));
        label_ccd_hb->setFrameShape(QFrame::Box);
        label_ccd_hb->setFrameShadow(QFrame::Raised);
        label_ccd_hb->setAlignment(Qt::AlignCenter);
        label_29 = new QLabel(groupBox_2);
        label_29->setObjectName(QString::fromUtf8("label_29"));
        label_29->setGeometry(QRect(20, 225, 101, 31));
        label_29->setFont(font);
        edt_imu_msg = new QLineEdit(groupBox_2);
        edt_imu_msg->setObjectName(QString::fromUtf8("edt_imu_msg"));
        edt_imu_msg->setGeometry(QRect(200, 179, 341, 31));
        edt_imu_msg->setFont(font);
        edt_imu_msg->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
        edt_imu_msg->setReadOnly(true);
        label_30 = new QLabel(groupBox_2);
        label_30->setObjectName(QString::fromUtf8("label_30"));
        label_30->setGeometry(QRect(20, 179, 101, 31));
        label_30->setFont(font);
        label_imu_hb = new QLabel(groupBox_2);
        label_imu_hb->setObjectName(QString::fromUtf8("label_imu_hb"));
        label_imu_hb->setGeometry(QRect(130, 179, 51, 31));
        label_imu_hb->setFont(font);
        label_imu_hb->setStyleSheet(QString::fromUtf8("color: rgb(238, 238, 236);\n"
"background-color: rgb(143, 89, 2);"));
        label_imu_hb->setFrameShape(QFrame::Box);
        label_imu_hb->setFrameShadow(QFrame::Raised);
        label_imu_hb->setAlignment(Qt::AlignCenter);
        edt_gps_msg = new QLineEdit(groupBox_2);
        edt_gps_msg->setObjectName(QString::fromUtf8("edt_gps_msg"));
        edt_gps_msg->setGeometry(QRect(200, 42, 341, 31));
        edt_gps_msg->setFont(font);
        edt_gps_msg->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
        edt_gps_msg->setReadOnly(true);
        label_31 = new QLabel(groupBox_2);
        label_31->setObjectName(QString::fromUtf8("label_31"));
        label_31->setGeometry(QRect(20, 42, 81, 31));
        label_31->setFont(font);
        label_gps_hb = new QLabel(groupBox_2);
        label_gps_hb->setObjectName(QString::fromUtf8("label_gps_hb"));
        label_gps_hb->setGeometry(QRect(130, 42, 51, 31));
        label_gps_hb->setFont(font);
        label_gps_hb->setStyleSheet(QString::fromUtf8("color: rgb(238, 238, 236);\n"
"background-color: rgb(143, 89, 2);"));
        label_gps_hb->setFrameShape(QFrame::Box);
        label_gps_hb->setFrameShadow(QFrame::Raised);
        label_gps_hb->setAlignment(Qt::AlignCenter);
        edt_scan1_msg = new QLineEdit(groupBox_2);
        edt_scan1_msg->setObjectName(QString::fromUtf8("edt_scan1_msg"));
        edt_scan1_msg->setGeometry(QRect(200, 272, 341, 31));
        edt_scan1_msg->setFont(font);
        edt_scan1_msg->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
        edt_scan1_msg->setReadOnly(true);
        label_32 = new QLabel(groupBox_2);
        label_32->setObjectName(QString::fromUtf8("label_32"));
        label_32->setGeometry(QRect(20, 272, 101, 31));
        label_32->setFont(font);
        label_scan1_hb = new QLabel(groupBox_2);
        label_scan1_hb->setObjectName(QString::fromUtf8("label_scan1_hb"));
        label_scan1_hb->setGeometry(QRect(130, 272, 51, 31));
        label_scan1_hb->setFont(font);
        label_scan1_hb->setStyleSheet(QString::fromUtf8("color: rgb(238, 238, 236);\n"
"background-color: rgb(143, 89, 2);"));
        label_scan1_hb->setFrameShape(QFrame::Box);
        label_scan1_hb->setFrameShadow(QFrame::Raised);
        label_scan1_hb->setAlignment(Qt::AlignCenter);
        edt_scan2_msg = new QLineEdit(groupBox_2);
        edt_scan2_msg->setObjectName(QString::fromUtf8("edt_scan2_msg"));
        edt_scan2_msg->setGeometry(QRect(200, 320, 341, 31));
        edt_scan2_msg->setFont(font);
        edt_scan2_msg->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
        edt_scan2_msg->setReadOnly(true);
        label_scan2_hb = new QLabel(groupBox_2);
        label_scan2_hb->setObjectName(QString::fromUtf8("label_scan2_hb"));
        label_scan2_hb->setGeometry(QRect(130, 320, 51, 31));
        label_scan2_hb->setFont(font);
        label_scan2_hb->setStyleSheet(QString::fromUtf8("color: rgb(238, 238, 236);\n"
"background-color: rgb(143, 89, 2);"));
        label_scan2_hb->setFrameShape(QFrame::Box);
        label_scan2_hb->setFrameShadow(QFrame::Raised);
        label_scan2_hb->setAlignment(Qt::AlignCenter);
        label_33 = new QLabel(groupBox_2);
        label_33->setObjectName(QString::fromUtf8("label_33"));
        label_33->setGeometry(QRect(20, 320, 101, 31));
        label_33->setFont(font);
        edt_scan3_msg = new QLineEdit(groupBox_2);
        edt_scan3_msg->setObjectName(QString::fromUtf8("edt_scan3_msg"));
        edt_scan3_msg->setGeometry(QRect(200, 368, 341, 31));
        edt_scan3_msg->setFont(font);
        edt_scan3_msg->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
        edt_scan3_msg->setReadOnly(true);
        label_scan3_hb = new QLabel(groupBox_2);
        label_scan3_hb->setObjectName(QString::fromUtf8("label_scan3_hb"));
        label_scan3_hb->setGeometry(QRect(130, 368, 51, 31));
        label_scan3_hb->setFont(font);
        label_scan3_hb->setStyleSheet(QString::fromUtf8("color: rgb(238, 238, 236);\n"
"background-color: rgb(143, 89, 2);"));
        label_scan3_hb->setFrameShape(QFrame::Box);
        label_scan3_hb->setFrameShadow(QFrame::Raised);
        label_scan3_hb->setAlignment(Qt::AlignCenter);
        label_34 = new QLabel(groupBox_2);
        label_34->setObjectName(QString::fromUtf8("label_34"));
        label_34->setGeometry(QRect(20, 368, 101, 31));
        label_34->setFont(font);
        edt_scan4_msg = new QLineEdit(groupBox_2);
        edt_scan4_msg->setObjectName(QString::fromUtf8("edt_scan4_msg"));
        edt_scan4_msg->setGeometry(QRect(200, 418, 341, 31));
        edt_scan4_msg->setFont(font);
        edt_scan4_msg->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
        edt_scan4_msg->setReadOnly(true);
        label_35 = new QLabel(groupBox_2);
        label_35->setObjectName(QString::fromUtf8("label_35"));
        label_35->setGeometry(QRect(20, 418, 101, 31));
        label_35->setFont(font);
        label_scan4_hb = new QLabel(groupBox_2);
        label_scan4_hb->setObjectName(QString::fromUtf8("label_scan4_hb"));
        label_scan4_hb->setGeometry(QRect(130, 418, 51, 31));
        label_scan4_hb->setFont(font);
        label_scan4_hb->setStyleSheet(QString::fromUtf8("color: rgb(238, 238, 236);\n"
"background-color: rgb(143, 89, 2);"));
        label_scan4_hb->setFrameShape(QFrame::Box);
        label_scan4_hb->setFrameShadow(QFrame::Raised);
        label_scan4_hb->setAlignment(Qt::AlignCenter);
        groupBox_4 = new QGroupBox(Dialog_NodeCheck);
        groupBox_4->setObjectName(QString::fromUtf8("groupBox_4"));
        groupBox_4->setGeometry(QRect(600, 360, 561, 141));
        groupBox_4->setFont(font);
        groupBox_4->setStyleSheet(QString::fromUtf8("background-color: rgb(114, 159, 207);\n"
"gridline-color: rgb(0, 0, 0);"));
        label_37 = new QLabel(groupBox_4);
        label_37->setObjectName(QString::fromUtf8("label_37"));
        label_37->setGeometry(QRect(20, 40, 101, 31));
        label_37->setFont(font);
        edt_plc_msg = new QLineEdit(groupBox_4);
        edt_plc_msg->setObjectName(QString::fromUtf8("edt_plc_msg"));
        edt_plc_msg->setGeometry(QRect(200, 40, 341, 31));
        edt_plc_msg->setFont(font);
        edt_plc_msg->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
        edt_plc_msg->setReadOnly(true);
        label_plc_hb = new QLabel(groupBox_4);
        label_plc_hb->setObjectName(QString::fromUtf8("label_plc_hb"));
        label_plc_hb->setGeometry(QRect(130, 40, 51, 31));
        label_plc_hb->setFont(font);
        label_plc_hb->setStyleSheet(QString::fromUtf8("color: rgb(238, 238, 236);\n"
"background-color: rgb(143, 89, 2);"));
        label_plc_hb->setFrameShape(QFrame::Box);
        label_plc_hb->setFrameShadow(QFrame::Raised);
        label_plc_hb->setAlignment(Qt::AlignCenter);
        edt_mqtt_msg = new QLineEdit(groupBox_4);
        edt_mqtt_msg->setObjectName(QString::fromUtf8("edt_mqtt_msg"));
        edt_mqtt_msg->setGeometry(QRect(200, 85, 341, 31));
        edt_mqtt_msg->setFont(font);
        edt_mqtt_msg->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
        edt_mqtt_msg->setReadOnly(true);
        label_38 = new QLabel(groupBox_4);
        label_38->setObjectName(QString::fromUtf8("label_38"));
        label_38->setGeometry(QRect(20, 85, 101, 31));
        label_38->setFont(font);
        label_mqtt_hb = new QLabel(groupBox_4);
        label_mqtt_hb->setObjectName(QString::fromUtf8("label_mqtt_hb"));
        label_mqtt_hb->setGeometry(QRect(130, 85, 51, 31));
        label_mqtt_hb->setFont(font);
        label_mqtt_hb->setStyleSheet(QString::fromUtf8("color: rgb(238, 238, 236);\n"
"background-color: rgb(143, 89, 2);"));
        label_mqtt_hb->setFrameShape(QFrame::Box);
        label_mqtt_hb->setFrameShadow(QFrame::Raised);
        label_mqtt_hb->setAlignment(Qt::AlignCenter);
        groupBox_5 = new QGroupBox(Dialog_NodeCheck);
        groupBox_5->setObjectName(QString::fromUtf8("groupBox_5"));
        groupBox_5->setGeometry(QRect(600, 21, 561, 321));
        groupBox_5->setFont(font);
        groupBox_5->setStyleSheet(QString::fromUtf8("background-color: rgb(114, 159, 207);\n"
"gridline-color: rgb(0, 0, 0);"));
        label_39 = new QLabel(groupBox_5);
        label_39->setObjectName(QString::fromUtf8("label_39"));
        label_39->setGeometry(QRect(20, 40, 101, 31));
        label_39->setFont(font);
        edt_pathplan_msg = new QLineEdit(groupBox_5);
        edt_pathplan_msg->setObjectName(QString::fromUtf8("edt_pathplan_msg"));
        edt_pathplan_msg->setGeometry(QRect(200, 40, 341, 31));
        edt_pathplan_msg->setFont(font);
        edt_pathplan_msg->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
        edt_pathplan_msg->setReadOnly(true);
        label_pathplan_hb = new QLabel(groupBox_5);
        label_pathplan_hb->setObjectName(QString::fromUtf8("label_pathplan_hb"));
        label_pathplan_hb->setGeometry(QRect(130, 40, 51, 31));
        label_pathplan_hb->setFont(font);
        label_pathplan_hb->setStyleSheet(QString::fromUtf8("color: rgb(238, 238, 236);\n"
"background-color: rgb(143, 89, 2);"));
        label_pathplan_hb->setFrameShape(QFrame::Box);
        label_pathplan_hb->setFrameShadow(QFrame::Raised);
        label_pathplan_hb->setAlignment(Qt::AlignCenter);
        edt_pathtrack_msg = new QLineEdit(groupBox_5);
        edt_pathtrack_msg->setObjectName(QString::fromUtf8("edt_pathtrack_msg"));
        edt_pathtrack_msg->setGeometry(QRect(200, 85, 341, 31));
        edt_pathtrack_msg->setFont(font);
        edt_pathtrack_msg->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
        edt_pathtrack_msg->setReadOnly(true);
        label_pathtrack_hb = new QLabel(groupBox_5);
        label_pathtrack_hb->setObjectName(QString::fromUtf8("label_pathtrack_hb"));
        label_pathtrack_hb->setGeometry(QRect(130, 85, 51, 31));
        label_pathtrack_hb->setFont(font);
        label_pathtrack_hb->setStyleSheet(QString::fromUtf8("color: rgb(238, 238, 236);\n"
"background-color: rgb(143, 89, 2);"));
        label_pathtrack_hb->setFrameShape(QFrame::Box);
        label_pathtrack_hb->setFrameShadow(QFrame::Raised);
        label_pathtrack_hb->setAlignment(Qt::AlignCenter);
        label_41 = new QLabel(groupBox_5);
        label_41->setObjectName(QString::fromUtf8("label_41"));
        label_41->setGeometry(QRect(20, 85, 101, 31));
        label_41->setFont(font);
        edt_work_msg = new QLineEdit(groupBox_5);
        edt_work_msg->setObjectName(QString::fromUtf8("edt_work_msg"));
        edt_work_msg->setGeometry(QRect(200, 131, 341, 31));
        edt_work_msg->setFont(font);
        edt_work_msg->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
        edt_work_msg->setReadOnly(true);
        label_work_hb = new QLabel(groupBox_5);
        label_work_hb->setObjectName(QString::fromUtf8("label_work_hb"));
        label_work_hb->setGeometry(QRect(130, 131, 51, 31));
        label_work_hb->setFont(font);
        label_work_hb->setStyleSheet(QString::fromUtf8("color: rgb(238, 238, 236);\n"
"background-color: rgb(143, 89, 2);"));
        label_work_hb->setFrameShape(QFrame::Box);
        label_work_hb->setFrameShadow(QFrame::Raised);
        label_work_hb->setAlignment(Qt::AlignCenter);
        label_42 = new QLabel(groupBox_5);
        label_42->setObjectName(QString::fromUtf8("label_42"));
        label_42->setGeometry(QRect(20, 131, 101, 31));
        label_42->setFont(font);
        edt_ctrdecision_msg = new QLineEdit(groupBox_5);
        edt_ctrdecision_msg->setObjectName(QString::fromUtf8("edt_ctrdecision_msg"));
        edt_ctrdecision_msg->setGeometry(QRect(200, 225, 341, 31));
        edt_ctrdecision_msg->setFont(font);
        edt_ctrdecision_msg->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
        edt_ctrdecision_msg->setReadOnly(true);
        label_47 = new QLabel(groupBox_5);
        label_47->setObjectName(QString::fromUtf8("label_47"));
        label_47->setGeometry(QRect(20, 225, 101, 31));
        label_47->setFont(font);
        label_ctr_decision_hb = new QLabel(groupBox_5);
        label_ctr_decision_hb->setObjectName(QString::fromUtf8("label_ctr_decision_hb"));
        label_ctr_decision_hb->setGeometry(QRect(130, 225, 51, 31));
        label_ctr_decision_hb->setFont(font);
        label_ctr_decision_hb->setStyleSheet(QString::fromUtf8("color: rgb(238, 238, 236);\n"
"background-color: rgb(143, 89, 2);"));
        label_ctr_decision_hb->setFrameShape(QFrame::Box);
        label_ctr_decision_hb->setFrameShadow(QFrame::Raised);
        label_ctr_decision_hb->setAlignment(Qt::AlignCenter);
        edt_turntable_msg = new QLineEdit(groupBox_5);
        edt_turntable_msg->setObjectName(QString::fromUtf8("edt_turntable_msg"));
        edt_turntable_msg->setGeometry(QRect(200, 270, 341, 31));
        edt_turntable_msg->setFont(font);
        edt_turntable_msg->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
        edt_turntable_msg->setReadOnly(true);
        label_48 = new QLabel(groupBox_5);
        label_48->setObjectName(QString::fromUtf8("label_48"));
        label_48->setGeometry(QRect(20, 270, 101, 31));
        label_48->setFont(font);
        label_turntable_hb = new QLabel(groupBox_5);
        label_turntable_hb->setObjectName(QString::fromUtf8("label_turntable_hb"));
        label_turntable_hb->setGeometry(QRect(130, 270, 51, 31));
        label_turntable_hb->setFont(font);
        label_turntable_hb->setStyleSheet(QString::fromUtf8("color: rgb(238, 238, 236);\n"
"background-color: rgb(143, 89, 2);"));
        label_turntable_hb->setFrameShape(QFrame::Box);
        label_turntable_hb->setFrameShadow(QFrame::Raised);
        label_turntable_hb->setAlignment(Qt::AlignCenter);
        edt_scancheck_msg = new QLineEdit(groupBox_5);
        edt_scancheck_msg->setObjectName(QString::fromUtf8("edt_scancheck_msg"));
        edt_scancheck_msg->setGeometry(QRect(200, 180, 341, 31));
        edt_scancheck_msg->setFont(font);
        edt_scancheck_msg->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
        edt_scancheck_msg->setReadOnly(true);
        label_43 = new QLabel(groupBox_5);
        label_43->setObjectName(QString::fromUtf8("label_43"));
        label_43->setGeometry(QRect(20, 180, 101, 31));
        label_43->setFont(font);
        label_scancheck_hb = new QLabel(groupBox_5);
        label_scancheck_hb->setObjectName(QString::fromUtf8("label_scancheck_hb"));
        label_scancheck_hb->setGeometry(QRect(130, 180, 51, 31));
        label_scancheck_hb->setFont(font);
        label_scancheck_hb->setStyleSheet(QString::fromUtf8("color: rgb(238, 238, 236);\n"
"background-color: rgb(143, 89, 2);"));
        label_scancheck_hb->setFrameShape(QFrame::Box);
        label_scancheck_hb->setFrameShadow(QFrame::Raised);
        label_scancheck_hb->setAlignment(Qt::AlignCenter);

        retranslateUi(Dialog_NodeCheck);

        QMetaObject::connectSlotsByName(Dialog_NodeCheck);
    } // setupUi

    void retranslateUi(QDialog *Dialog_NodeCheck)
    {
        Dialog_NodeCheck->setWindowTitle(QApplication::translate("Dialog_NodeCheck", "\345\205\250\347\263\273\347\273\237\347\212\266\346\200\201\346\243\200\346\265\213", nullptr));
        groupBox_2->setTitle(QApplication::translate("Dialog_NodeCheck", "\344\274\240\346\204\237\345\231\250", nullptr));
        label_25->setText(QApplication::translate("Dialog_NodeCheck", "\346\277\200\345\205\211\351\233\267\350\276\276", nullptr));
        edt_lidar_msg->setText(QApplication::translate("Dialog_NodeCheck", "OK", nullptr));
        label_lidar_hb->setText(QApplication::translate("Dialog_NodeCheck", "0", nullptr));
        edt_radar_msg->setText(QApplication::translate("Dialog_NodeCheck", "OK", nullptr));
        label_27->setText(QApplication::translate("Dialog_NodeCheck", "\346\257\253\347\261\263\346\263\242\351\233\267\350\276\276", nullptr));
        label_radar_hb->setText(QApplication::translate("Dialog_NodeCheck", "0", nullptr));
        edt_ccd_msg->setText(QApplication::translate("Dialog_NodeCheck", "OK", nullptr));
        label_ccd_hb->setText(QApplication::translate("Dialog_NodeCheck", "0", nullptr));
        label_29->setText(QApplication::translate("Dialog_NodeCheck", "CCD", nullptr));
        edt_imu_msg->setText(QApplication::translate("Dialog_NodeCheck", "OK", nullptr));
        label_30->setText(QApplication::translate("Dialog_NodeCheck", "IMU\345\247\277\346\200\201\350\256\241", nullptr));
        label_imu_hb->setText(QApplication::translate("Dialog_NodeCheck", "0", nullptr));
        edt_gps_msg->setText(QApplication::translate("Dialog_NodeCheck", "OK", nullptr));
        label_31->setText(QApplication::translate("Dialog_NodeCheck", "\345\267\256\345\210\206GPS", nullptr));
        label_gps_hb->setText(QApplication::translate("Dialog_NodeCheck", "0", nullptr));
        edt_scan1_msg->setText(QApplication::translate("Dialog_NodeCheck", "OK", nullptr));
        label_32->setText(QApplication::translate("Dialog_NodeCheck", "\347\272\277\346\277\200\345\205\211\345\231\2501", nullptr));
        label_scan1_hb->setText(QApplication::translate("Dialog_NodeCheck", "0", nullptr));
        edt_scan2_msg->setText(QApplication::translate("Dialog_NodeCheck", "OK", nullptr));
        label_scan2_hb->setText(QApplication::translate("Dialog_NodeCheck", "0", nullptr));
        label_33->setText(QApplication::translate("Dialog_NodeCheck", "\347\272\277\346\277\200\345\205\211\345\231\2502", nullptr));
        edt_scan3_msg->setText(QApplication::translate("Dialog_NodeCheck", "OK", nullptr));
        label_scan3_hb->setText(QApplication::translate("Dialog_NodeCheck", "0", nullptr));
        label_34->setText(QApplication::translate("Dialog_NodeCheck", "\347\272\277\346\277\200\345\205\211\345\231\2503", nullptr));
        edt_scan4_msg->setText(QApplication::translate("Dialog_NodeCheck", "OK", nullptr));
        label_35->setText(QApplication::translate("Dialog_NodeCheck", "\347\272\277\346\277\200\345\205\211\345\231\2504", nullptr));
        label_scan4_hb->setText(QApplication::translate("Dialog_NodeCheck", "0", nullptr));
        groupBox_4->setTitle(QApplication::translate("Dialog_NodeCheck", "\346\225\260\346\215\256\351\200\232\350\256\257", nullptr));
        label_37->setText(QApplication::translate("Dialog_NodeCheck", "PLC\351\200\232\350\256\257", nullptr));
        edt_plc_msg->setText(QApplication::translate("Dialog_NodeCheck", "OK", nullptr));
        label_plc_hb->setText(QApplication::translate("Dialog_NodeCheck", "0", nullptr));
        edt_mqtt_msg->setText(QApplication::translate("Dialog_NodeCheck", "OK", nullptr));
        label_38->setText(QApplication::translate("Dialog_NodeCheck", "\350\277\234\347\250\213\351\200\232\350\256\257", nullptr));
        label_mqtt_hb->setText(QApplication::translate("Dialog_NodeCheck", "0", nullptr));
        groupBox_5->setTitle(QApplication::translate("Dialog_NodeCheck", "\345\257\274\350\210\252\346\216\247\345\210\266", nullptr));
        label_39->setText(QApplication::translate("Dialog_NodeCheck", "\350\267\257\345\276\204\350\247\204\345\210\222", nullptr));
        edt_pathplan_msg->setText(QApplication::translate("Dialog_NodeCheck", "OK", nullptr));
        label_pathplan_hb->setText(QApplication::translate("Dialog_NodeCheck", "0", nullptr));
        edt_pathtrack_msg->setText(QApplication::translate("Dialog_NodeCheck", "OK", nullptr));
        label_pathtrack_hb->setText(QApplication::translate("Dialog_NodeCheck", "0", nullptr));
        label_41->setText(QApplication::translate("Dialog_NodeCheck", "\350\267\237\350\270\252\346\216\247\345\210\266", nullptr));
        edt_work_msg->setText(QApplication::translate("Dialog_NodeCheck", "OK", nullptr));
        label_work_hb->setText(QApplication::translate("Dialog_NodeCheck", "0", nullptr));
        label_42->setText(QApplication::translate("Dialog_NodeCheck", "\344\275\234\344\270\232\346\216\247\345\210\266", nullptr));
        edt_ctrdecision_msg->setText(QApplication::translate("Dialog_NodeCheck", "OK", nullptr));
        label_47->setText(QApplication::translate("Dialog_NodeCheck", "\351\232\234\347\242\215\346\243\200\346\265\213", nullptr));
        label_ctr_decision_hb->setText(QApplication::translate("Dialog_NodeCheck", "0", nullptr));
        edt_turntable_msg->setText(QApplication::translate("Dialog_NodeCheck", "OK", nullptr));
        label_48->setText(QApplication::translate("Dialog_NodeCheck", "\344\274\272\346\234\215\350\275\254\345\217\260", nullptr));
        label_turntable_hb->setText(QApplication::translate("Dialog_NodeCheck", "0", nullptr));
        edt_scancheck_msg->setText(QApplication::translate("Dialog_NodeCheck", "OK", nullptr));
        label_43->setText(QApplication::translate("Dialog_NodeCheck", "\350\275\256\350\203\216\345\256\232\344\275\215", nullptr));
        label_scancheck_hb->setText(QApplication::translate("Dialog_NodeCheck", "0", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Dialog_NodeCheck: public Ui_Dialog_NodeCheck {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_NODECHECK_H
