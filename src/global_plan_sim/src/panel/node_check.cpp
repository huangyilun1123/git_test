#include "node_check.h"

dialog_node_check::dialog_node_check(QWidget *parent) :
    QDialog(parent),
    ui(new Ui_Dialog_NodeCheck)
{
    ui->setupUi(this);
    nh=new ros::NodeHandle("~");

    qtmr.start(100);
    connect(&qtmr, SIGNAL(timeout()), this, SLOT(qtmrfunc()));
}

void dialog_node_check::update_rate_msg(string pre_param, QLabel *lab, QLineEdit *edt)
{
    char buf[200];
    float value = 0;
    pre_param+="/check";
    nh->getParam(pre_param + "/node_rate", value);
    sprintf(buf, "%.1f", value);
    lab->setText(buf);
    string msg = "";
    nh->getParam(pre_param+"/msg", msg);
    edt->setText(QString::fromStdString(msg));
}

void dialog_node_check::qtmrfunc()
{
    update_rate_msg("/gps_pro", ui->label_gps_hb, ui->edt_gps_msg);
    update_rate_msg("/turntable", ui->label_turntable_hb, ui->edt_turntable_msg);
    update_rate_msg("/cloud_tf_obs", ui->label_lidar_hb, ui->edt_lidar_msg);
    update_rate_msg("/can_comm", ui->label_plc_hb, ui->edt_plc_msg);

    update_rate_msg("/lidar_radar_obs", ui->label_lidar_hb, ui->edt_lidar_msg);
    update_rate_msg("/ARS_Ladar", ui->label_radar_hb, ui->edt_radar_msg);

    update_rate_msg("/mqtt_comm", ui->label_mqtt_hb, ui->edt_mqtt_msg);
    update_rate_msg("/imudata_deg", ui->label_imu_hb, ui->edt_imu_msg);

    update_rate_msg("/c200_lidar_node1", ui->label_scan1_hb, ui->edt_scan1_msg);
    update_rate_msg("/c200_lidar_node2", ui->label_scan2_hb, ui->edt_scan2_msg);
    update_rate_msg("/c200_lidar_node3", ui->label_scan3_hb, ui->edt_scan3_msg);
    update_rate_msg("/c200_lidar_node4", ui->label_scan4_hb, ui->edt_scan4_msg);

    update_rate_msg("/local_path_plan", ui->label_pathplan_hb, ui->edt_pathplan_msg);
    update_rate_msg("/pathtrack", ui->label_pathtrack_hb, ui->edt_pathtrack_msg);
    update_rate_msg("/pawcontrol", ui->label_work_hb, ui->edt_work_msg);

    update_rate_msg("/laserscan_check", ui->label_scancheck_hb, ui->edt_scancheck_msg);
}

dialog_node_check::~dialog_node_check()
{
    delete ui;
}
