/********************************************************************************
** Form generated from reading UI file 'global_plan_sim.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef GLOBAL_PLAN_SIM_H
#define GLOBAL_PLAN_SIM_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Panel_Global_Plan_Sim
{
public:
    QLabel *lab1;
    QLabel *label;
    QLabel *lab2;
    QPushButton *btn_pickcarpath;
    QPushButton *btn_releasecarpath;
    QPushButton *btn_stop;
    QPushButton *btn_movepath;
    QPushButton *btn_cleartrack;
    QPushButton *btn_syscheck;
    QComboBox *cb_pathdir;
    QPushButton *btn_turnmode;
    QComboBox *cb_turnmode;
    QPushButton *btn_mqtt_sim;
    QComboBox *cb_tasktype;
    QLabel *lab3;

    void setupUi(QWidget *Panel_Global_Plan_Sim)
    {
        if (Panel_Global_Plan_Sim->objectName().isEmpty())
            Panel_Global_Plan_Sim->setObjectName(QString::fromUtf8("Panel_Global_Plan_Sim"));
        Panel_Global_Plan_Sim->resize(346, 417);
        lab1 = new QLabel(Panel_Global_Plan_Sim);
        lab1->setObjectName(QString::fromUtf8("lab1"));
        lab1->setGeometry(QRect(4, 32, 331, 40));
        QFont font;
        font.setPointSize(14);
        lab1->setFont(font);
        label = new QLabel(Panel_Global_Plan_Sim);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(40, 10, 261, 20));
        QFont font1;
        font1.setFamily(QString::fromUtf8("Ubuntu Condensed"));
        font1.setPointSize(15);
        font1.setBold(true);
        font1.setWeight(75);
        label->setFont(font1);
        label->setAlignment(Qt::AlignCenter);
        lab2 = new QLabel(Panel_Global_Plan_Sim);
        lab2->setObjectName(QString::fromUtf8("lab2"));
        lab2->setGeometry(QRect(4, 75, 331, 40));
        lab2->setFont(font);
        btn_pickcarpath = new QPushButton(Panel_Global_Plan_Sim);
        btn_pickcarpath->setObjectName(QString::fromUtf8("btn_pickcarpath"));
        btn_pickcarpath->setGeometry(QRect(20, 158, 141, 41));
        btn_pickcarpath->setStyleSheet(QString::fromUtf8("background-color: rgb(211, 215, 207);\n"
"font: 16pt \"Ubuntu\";"));
        btn_releasecarpath = new QPushButton(Panel_Global_Plan_Sim);
        btn_releasecarpath->setObjectName(QString::fromUtf8("btn_releasecarpath"));
        btn_releasecarpath->setGeometry(QRect(180, 158, 141, 41));
        btn_releasecarpath->setStyleSheet(QString::fromUtf8("background-color: rgb(211, 215, 207);\n"
"font: 16pt \"Ubuntu\";"));
        btn_stop = new QPushButton(Panel_Global_Plan_Sim);
        btn_stop->setObjectName(QString::fromUtf8("btn_stop"));
        btn_stop->setGeometry(QRect(20, 354, 141, 41));
        btn_stop->setStyleSheet(QString::fromUtf8("font: 16pt \"Ubuntu\";\n"
"background-color: rgb(239, 41, 41);"));
        btn_movepath = new QPushButton(Panel_Global_Plan_Sim);
        btn_movepath->setObjectName(QString::fromUtf8("btn_movepath"));
        btn_movepath->setGeometry(QRect(20, 208, 111, 41));
        btn_movepath->setStyleSheet(QString::fromUtf8("background-color: rgb(211, 215, 207);\n"
"font: 16pt \"Ubuntu\";"));
        btn_cleartrack = new QPushButton(Panel_Global_Plan_Sim);
        btn_cleartrack->setObjectName(QString::fromUtf8("btn_cleartrack"));
        btn_cleartrack->setGeometry(QRect(200, 208, 121, 41));
        btn_cleartrack->setStyleSheet(QString::fromUtf8("background-color: rgb(211, 215, 207);\n"
"font: 16pt \"Ubuntu\";"));
        btn_syscheck = new QPushButton(Panel_Global_Plan_Sim);
        btn_syscheck->setObjectName(QString::fromUtf8("btn_syscheck"));
        btn_syscheck->setGeometry(QRect(179, 353, 141, 41));
        btn_syscheck->setStyleSheet(QString::fromUtf8("background-color: rgb(211, 215, 207);\n"
"font: 16pt \"Ubuntu\";"));
        cb_pathdir = new QComboBox(Panel_Global_Plan_Sim);
        cb_pathdir->addItem(QString());
        cb_pathdir->addItem(QString());
        cb_pathdir->setObjectName(QString::fromUtf8("cb_pathdir"));
        cb_pathdir->setGeometry(QRect(137, 212, 51, 31));
        QFont font2;
        font2.setPointSize(16);
        cb_pathdir->setFont(font2);
        btn_turnmode = new QPushButton(Panel_Global_Plan_Sim);
        btn_turnmode->setObjectName(QString::fromUtf8("btn_turnmode"));
        btn_turnmode->setGeometry(QRect(20, 258, 111, 41));
        btn_turnmode->setStyleSheet(QString::fromUtf8("background-color: rgb(211, 215, 207);\n"
"font: 16pt \"Ubuntu\";"));
        cb_turnmode = new QComboBox(Panel_Global_Plan_Sim);
        cb_turnmode->addItem(QString());
        cb_turnmode->addItem(QString());
        cb_turnmode->addItem(QString());
        cb_turnmode->setObjectName(QString::fromUtf8("cb_turnmode"));
        cb_turnmode->setGeometry(QRect(137, 262, 61, 31));
        cb_turnmode->setFont(font2);
        btn_mqtt_sim = new QPushButton(Panel_Global_Plan_Sim);
        btn_mqtt_sim->setObjectName(QString::fromUtf8("btn_mqtt_sim"));
        btn_mqtt_sim->setGeometry(QRect(20, 306, 111, 41));
        btn_mqtt_sim->setStyleSheet(QString::fromUtf8("background-color: rgb(211, 215, 207);\n"
"font: 16pt \"Ubuntu\";"));
        cb_tasktype = new QComboBox(Panel_Global_Plan_Sim);
        cb_tasktype->addItem(QString());
        cb_tasktype->addItem(QString());
        cb_tasktype->addItem(QString());
        cb_tasktype->setObjectName(QString::fromUtf8("cb_tasktype"));
        cb_tasktype->setGeometry(QRect(139, 310, 141, 31));
        cb_tasktype->setFont(font2);
        lab3 = new QLabel(Panel_Global_Plan_Sim);
        lab3->setObjectName(QString::fromUtf8("lab3"));
        lab3->setGeometry(QRect(4, 117, 331, 40));
        lab3->setFont(font);

        retranslateUi(Panel_Global_Plan_Sim);

        QMetaObject::connectSlotsByName(Panel_Global_Plan_Sim);
    } // setupUi

    void retranslateUi(QWidget *Panel_Global_Plan_Sim)
    {
        Panel_Global_Plan_Sim->setWindowTitle(QApplication::translate("Panel_Global_Plan_Sim", "\344\274\272\346\234\215\350\275\254\345\217\260", nullptr));
        lab1->setText(QApplication::translate("Panel_Global_Plan_Sim", "\347\212\266\346\200\201:", nullptr));
        label->setText(QApplication::translate("Panel_Global_Plan_Sim", "\350\275\254\350\277\220\350\275\246\347\233\221\346\216\247", nullptr));
        lab2->setText(QApplication::translate("Panel_Global_Plan_Sim", "\347\233\256\346\240\207\347\202\271:", nullptr));
        btn_pickcarpath->setText(QApplication::translate("Panel_Global_Plan_Sim", "\345\217\221\351\200\201\345\217\226\350\275\246\350\267\257\345\276\204", nullptr));
        btn_releasecarpath->setText(QApplication::translate("Panel_Global_Plan_Sim", "\345\217\221\351\200\201\346\224\276\350\275\246\350\267\257\345\276\204", nullptr));
        btn_stop->setText(QApplication::translate("Panel_Global_Plan_Sim", "\345\201\234\346\255\242", nullptr));
        btn_movepath->setText(QApplication::translate("Panel_Global_Plan_Sim", "\345\217\221\351\200\201\347\247\273\345\212\250", nullptr));
        btn_cleartrack->setText(QApplication::translate("Panel_Global_Plan_Sim", "\346\270\205\351\231\244\350\275\250\350\277\271", nullptr));
        btn_syscheck->setText(QApplication::translate("Panel_Global_Plan_Sim", "\347\212\266\346\200\201\346\243\200\346\237\245", nullptr));
        cb_pathdir->setItemText(0, QApplication::translate("Panel_Global_Plan_Sim", "\346\255\243", nullptr));
        cb_pathdir->setItemText(1, QApplication::translate("Panel_Global_Plan_Sim", "\345\217\215", nullptr));

        btn_turnmode->setText(QApplication::translate("Panel_Global_Plan_Sim", "\350\277\220\345\212\250\346\250\241\345\274\217", nullptr));
        cb_turnmode->setItemText(0, QApplication::translate("Panel_Global_Plan_Sim", "100", nullptr));
        cb_turnmode->setItemText(1, QApplication::translate("Panel_Global_Plan_Sim", "0", nullptr));
        cb_turnmode->setItemText(2, QApplication::translate("Panel_Global_Plan_Sim", "3", nullptr));

        btn_mqtt_sim->setText(QApplication::translate("Panel_Global_Plan_Sim", "mqtt sim", nullptr));
        cb_tasktype->setItemText(0, QApplication::translate("Panel_Global_Plan_Sim", "move task", nullptr));
        cb_tasktype->setItemText(1, QApplication::translate("Panel_Global_Plan_Sim", "pick task", nullptr));
        cb_tasktype->setItemText(2, QApplication::translate("Panel_Global_Plan_Sim", "release task", nullptr));

        lab3->setText(QApplication::translate("Panel_Global_Plan_Sim", "\347\233\256\346\240\207\347\202\271:", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Panel_Global_Plan_Sim: public Ui_Panel_Global_Plan_Sim {};
} // namespace Ui

QT_END_NAMESPACE

#endif // GLOBAL_PLAN_SIM_H
