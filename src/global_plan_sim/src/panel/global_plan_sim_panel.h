#ifndef ABC_H
#define ABC_H

//所需要包含的头文件
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/panel.h> //plugin基类的头文件
#include <std_msgs/String.h>
#include <QTimer>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <gps/MyGPS_msg.h>
#include <data_comm/car_state.h>
#include <data_comm/car_ctr.h>
#endif

#include "ui_global_plan_sim.h"
#include "node_check.h"
#include <yaml-cpp/yaml.h>
#include <mqtt_comm/task.h>
#include <mqtt_comm/path_point.h>

using namespace std;

namespace rviz_gui
{
    // 所有的plugin都必须是rviz::Panel的子类
    class Panel_Global_Plan_Sim : public rviz::Panel
    {
        // 后边需要用到Qt的信号和槽，都是QObject的子类，所以需要声明Q_OBJECT宏
        Q_OBJECT
    public:
        // 构造函数，在类中会用到QWidget的实例来实现GUI界面，这里先初始化为0即可
        Panel_Global_Plan_Sim(QWidget *parent = 0);

        // 公共槽.
    public Q_SLOTS:

        // 内部槽.
    protected Q_SLOTS:
        void qtmrfunc();

        void btn_release_onclick();
        void btn_pick_onclick();
        void btn_move_onclick();
        void btn_cleartrack_onclick();
        void btn_stop_onclick();
        void btn_syscheck_onclick();
        void btn_enable_onclick();
        void btn_backup_onclick();

        // 内部变量.
    protected:
        ros::NodeHandle *nh, *nh_local;
        ros::Publisher task_pub, trackpath_pub, simpose_pub;
        ros::Subscriber gps_sub, carstate_sub, carctr_sub, target_sub;
        geometry_msgs::PoseStamped pick_pose, release_pose, target_pose;
        double utm_x_zero = 0, utm_y_zero = 0;
        geometry_msgs::PoseStamped pose_zero;
        nav_msgs::Path track_path;
        data_comm::car_ctr car_ctr;
        bool run_enable = false;
        string pathfilepath;
        
        mqtt_comm::task pick_task, release_task, move_task, cur_task;
        void LoadPath(string fn);

        void Path2Task(string cmd, nav_msgs::Path path);
        void GetPickReleasePose();
        void DispAgvState();
        void PubTrackPath();
        void PubTask(string cmd);
        void GPSDataCallback(const gps::MyGPS_msg::ConstPtr &msg);
        void CarStateCallback(const data_comm::car_state::ConstPtr &msg);
        void CarCtrCallback(const data_comm::car_ctr::ConstPtr &msg);
        void TargetCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

        int CheckRosNode(string name);
        void UpdateErrCode();

    private:
        Ui::Panel_Global_Plan_Sim *ui;
        dialog_node_check *d_nodecheck;
        QTimer qtmr;

        int show_action_flag = 0;
        void SendTurnMoveAction(mqtt_comm::path_point &show_point, float turn_angle, float move_dis, float vel);
        void SendReturnAction(mqtt_comm::path_point &show_point, float move_dis, float vel);
        void SendHYAction(mqtt_comm::path_point &show_point, float move_dis, float vel);
        void ShowActionProcess();
    };
}

#endif
