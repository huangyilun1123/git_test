#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QDebug>
#include <std_msgs/String.h>
#include <common/public.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>
#include <common/mydisplay.h>

#include <QDir>

#include "global_plan_sim_panel.h"

using namespace std;

namespace rviz_gui
{
    QStringList filenameInDir(string path)
    {
        //存储文件名称
        QStringList string_list;

        //判断路径是否存在
        QDir dir(QString::fromStdString(path));
        if (!dir.exists())  return string_list;

        //查看路径中后缀为.yaml格式的文件
        QStringList filters;
        filters << QString("*.yaml");
        dir.setFilter(QDir::Files | QDir::NoSymLinks); //设置类型过滤器，只为文件格式
        dir.setNameFilters(filters);                   //设置文件名称过滤器，只为filters格式

        //统计文件个数
        int dir_count = dir.count();
        // printf("%d\n", dir_count);
        // // if (dir_count <= 0)  return;

        for (int i = 0; i < dir_count; i++)
        {
            QString file_name = dir[i]; //文件名称
            file_name=file_name.left(file_name.size()-5);
            string_list.append(file_name);
        }
        return string_list;
    }

    Panel_Global_Plan_Sim::Panel_Global_Plan_Sim(QWidget *parent)
        : rviz::Panel(parent), ui(new Ui::Panel_Global_Plan_Sim)
    {
        ui->setupUi(this);

        nh = new ros::NodeHandle();
        nh_local = new ros::NodeHandle("~");

        trackpath_pub = nh->advertise<nav_msgs::Path>("/track_path", 10);
        task_pub = nh->advertise<mqtt_comm::task>("/task_cmd", 10);
        simpose_pub = nh->advertise<geometry_msgs::PoseStamped>("/sim_pose", 10);

        gps_sub = nh->subscribe<gps::MyGPS_msg>("/gps_base/GPS_Base", 10, &Panel_Global_Plan_Sim::GPSDataCallback, this);
        carstate_sub = nh->subscribe<data_comm::car_state>("/can_comm/car_state", 10, &Panel_Global_Plan_Sim::CarStateCallback, this);
        carctr_sub = nh->subscribe<data_comm::car_ctr>("/pathtrack/ctr_cmd", 10, &Panel_Global_Plan_Sim::CarCtrCallback, this);
        target_sub = nh->subscribe<geometry_msgs::PoseStamped>("/local_path_plan/target_pose", 10, &Panel_Global_Plan_Sim::TargetCallback, this);

        pose_zero.header.frame_id = "base_link";
        pose_zero.pose.position.x = pose_zero.pose.position.y = pose_zero.pose.position.z = 0;
        pose_zero.pose.orientation.x = pose_zero.pose.orientation.y = pose_zero.pose.orientation.z = 0;
        pose_zero.pose.orientation.w = 1;

        qtmr.start(200);
        connect(&qtmr, SIGNAL(timeout()), this, SLOT(qtmrfunc()));

        connect(ui->btn_pick, SIGNAL(clicked()), this, SLOT(btn_pick_onclick()));
        connect(ui->btn_release, SIGNAL(clicked()), this, SLOT(btn_release_onclick()));
        connect(ui->btn_move, SIGNAL(clicked()), this, SLOT(btn_move_onclick()));
        connect(ui->btn_cleartrack, SIGNAL(clicked()), this, SLOT(btn_cleartrack_onclick()));

        connect(ui->btn_stop, SIGNAL(clicked()), this, SLOT(btn_stop_onclick()));
        connect(ui->btn_syscheck, SIGNAL(clicked()), this, SLOT(btn_syscheck_onclick()));
        connect(ui->btn_enable, SIGNAL(clicked()), this, SLOT(btn_enable_onclick()));
        connect(ui->btn_backup, SIGNAL(clicked()), this, SLOT(btn_backup_onclick()));

        char filename[1000];
        GetPackagePath("global_plan_sim", filename);
        int pos = strlen(filename);
        if (pos > 0)  filename[pos-1] = 0;
        sprintf(filename, "%s/path", filename);
        pathfilepath=filename;

        QStringList filenames = filenameInDir(pathfilepath);
        ui->cb_load->clear();
        ui->cb_load->addItems(filenames);

        string agvid, ver;
        nh->getParam("/agvId", agvid);
        nh->getParam("/version", ver);
        agvid="转运车: "+agvid+" "+ver;
        ui->label_agvid->setText(QString::fromStdString(agvid));
    }

    int Panel_Global_Plan_Sim::CheckRosNode(string name)
    {
        string msg="";
        nh->getParam(name+"/check/msg", msg);
        
        if(msg.find("OK")==msg.npos) return 1;
        else return 0;
    }

    void Panel_Global_Plan_Sim::UpdateErrCode()
    {
        unsigned int errcode=0;
        SetBit(errcode, 0, CheckRosNode("/gps_pro"));
        SetBit(errcode, 1, CheckRosNode("/mqtt_comm"));
        SetBit(errcode, 2, CheckRosNode("/can_comm"));
        SetBit(errcode, 3, CheckRosNode("/c200_lidar_node1"));
        SetBit(errcode, 4, CheckRosNode("/c200_lidar_node2"));
        SetBit(errcode, 5, CheckRosNode("/c200_lidar_node3"));
        SetBit(errcode, 6, CheckRosNode("/c200_lidar_node4"));
        SetBit(errcode, 7, CheckRosNode("/lidar_radar_obs"));
        SetBit(errcode, 8, CheckRosNode("/turntable"));

        int stopcode=0;
        nh->getParam("/pathtrack/stop_code", stopcode);
        
        char buf[100];
        sprintf(buf,"ErrCode: %05x  StopCode: %05x", errcode, stopcode); 
        ui->lab7->setText(QString::fromStdString(buf));

        nh->setParam("/err_code", (int)errcode);

        char str[100];
        if(errcode!=0)  
        {
            sprintf(str, "ErrCode: %05x", errcode);
            ROS_ERROR_STREAM_THROTTLE(1, str);
        }
        if(stopcode!=0)  
        {
            sprintf(str, "StopCode: %05x", stopcode);
            // ROS_INFO_STREAM_THROTTLE(1, str);
        }
    }

    void Panel_Global_Plan_Sim::PubTrackPath()  // 发布车辆走过的轨迹
    {
        track_path.header.frame_id = "map";
        track_path.header.stamp = ros::Time::now();

        geometry_msgs::PoseStamped pose_map;
        pose_zero.header.stamp = ros::Time::now();
        transformPose("map", pose_zero, pose_map, "XXX");

        bool add_flag = true;
        if (track_path.poses.size() > 0)
        {
            geometry_msgs::PoseStamped last_pose = *(track_path.poses.end() - 1);
            float dx = last_pose.pose.position.x - pose_map.pose.position.x;
            float dy = last_pose.pose.position.y - pose_map.pose.position.y;
            float ds = sqrt(pow(dx, 2) + pow(dy, 2));
            add_flag = (ds > 0.5);
        }
        if (add_flag)  track_path.poses.push_back(pose_map);
        //  保留1000m轨迹
        if(track_path.poses.size()>1000)  track_path.poses.erase(track_path.poses.begin());

        trackpath_pub.publish(track_path);
    }

    void Panel_Global_Plan_Sim::qtmrfunc()  // 定时
    {
        nh_local->getParam("/gps_base/utmx_zero", utm_x_zero);
        nh_local->getParam("/gps_base/utmy_zero", utm_y_zero);

        string paw_state;
        nh_local->getParam("/pawcontrol/paw_state", paw_state);
        char buf[200];
        sprintf(buf, "paw_state: %s", paw_state.c_str());
        ui->lab5->setText(QString::fromUtf8(buf));

        string stop_str;
        nh_local->getParam("/pathtrack/stop_reason", stop_str);
        float obs_dis_laser = 999, obs_dis_lidar = 999, obs_dis;
        nh_local->getParam("/laser_radar_obs/obstacle_dis", obs_dis_laser);
        nh_local->getParam("/lidar_radar_obs/obstacle_dis", obs_dis_lidar);
        obs_dis = min(obs_dis_laser, obs_dis_lidar);
        // float obs_dis=999;
        // // nh_local->getParam("/lidar_radar_obs/obstacle_dis", obs_dis);
        // nh_local->getParam("/laser_radar_obs/obstacle_dis", obs_dis);
        sprintf(buf, "stop: %s   obs:%.1f", stop_str.c_str(), obs_dis);
        ui->lab6->setText(QString::fromUtf8(buf));

        PubTrackPath();

        ui->btn_pick->setStyleSheet("background-color: rgb(186, 189, 182);");
        ui->btn_release->setStyleSheet("background-color: rgb(186, 189, 182);");
        ui->btn_move->setStyleSheet("background-color: rgb(186, 189, 182);");

        if (cur_task.cmd=="pick task")  ui->btn_pick->setStyleSheet("background-color: rgb(0, 255, 0);");
        else if (cur_task.cmd=="release task")  ui->btn_release->setStyleSheet("background-color: rgb(0, 255, 0);");
        else if (cur_task.cmd=="move task")  ui->btn_move->setStyleSheet("background-color: rgb(0, 255, 0);");

        nh_local->getParam("/pathtrack/run_enable", run_enable);
        if(run_enable)  ui->btn_enable->setStyleSheet("background-color: rgb(0, 255, 0);");
        else ui->btn_enable->setStyleSheet("background-color: rgb(186, 189, 182);");

        UpdateErrCode();

        static TTimer tmr;
        if(tmr.GetValue()>0.5)
        {
           tmr.Clear();
           string str=ui->cb_load->currentText().toStdString();
           LoadPath(str);     
        }

        ShowActionProcess();
    }

    void Panel_Global_Plan_Sim::GPSDataCallback(const gps::MyGPS_msg::ConstPtr &msg)  //  接收gps信息
    {
        char buf[200];
        sprintf(buf, "gps: x=%.2f y=%.2f a=%.1f° v=%.2f", msg->map_x, msg->map_y, msg->Angle, msg->Vel);
        ui->lab1->setText(QString::fromUtf8(buf));
    }

    void Panel_Global_Plan_Sim::CarStateCallback(const data_comm::car_state::ConstPtr &msg)  // 接收车辆状态
    {
        char buf[200];
        sprintf(buf, "car_state:work=%d turn=%d ctr=%d vel=%.1f", msg->workmode, msg->turnmode, msg->ctrmode, msg->speed[0]);
        ui->lab3->setText(QString::fromUtf8(buf));
    }

    void Panel_Global_Plan_Sim::CarCtrCallback(const data_comm::car_ctr::ConstPtr &msg)  // 接收车辆状态
    {
        char buf[200];
        sprintf(buf, "car_ctr:work=%d turn=%d vel=%.1f a=%.1f", msg->workmode, msg->turnmode, msg->speed, msg->angle);
        ui->lab4->setText(QString::fromUtf8(buf));
    }

    void Panel_Global_Plan_Sim::TargetCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        char buf[200];
        float angle=GetYawFromPose(*msg)*180/M_PI;
        sprintf(buf, "target:x=%.2f y=%.2f angle=%.2f", msg->pose.position.x, msg->pose.position.y, angle);
        ui->lab2->setText(QString::fromUtf8(buf));
    }

    void Panel_Global_Plan_Sim::LoadPath(string fn)
    {
        fn=pathfilepath+"/"+fn+".yaml";
        // printf("%s\n", fn.c_str());
        // return ;

        YAML::Node config = YAML::LoadFile(fn);

        pick_task.path.clear();     pick_task.cmd="pick task";
        release_task.path.clear();  release_task.cmd="release task";
        move_task.path.clear();     move_task.cmd="move task"; 

        mqtt_comm::path_point point;
        for(int i=0;i<1000;i++)
        {
            char name[100];
            sprintf(name,"pose%d",i);
            string pathname="pick_path";
            if (config[pathname][name]["map_x"].IsDefined())
            {
                point.pointX = config[pathname][name]["map_x"].as<float>();
                point.pointY = config[pathname][name]["map_y"].as<float>();
                point.pointHA = config[pathname][name]["heading"].as<float>();
                point.vehSpeed = config[pathname][name]["vel"].as<float>();
                pick_task.path.push_back(point);
            }
            else break;
        }

        for (int i = 0; i < 1000; i++)
        {
            char name[100];
            sprintf(name, "pose%d", i);
            string pathname = "release_path";
            if (config[pathname][name]["map_x"].IsDefined())
            {
                point.pointX = config[pathname][name]["map_x"].as<float>();
                point.pointY = config[pathname][name]["map_y"].as<float>();
                point.pointHA = config[pathname][name]["heading"].as<float>();
                point.vehSpeed = config[pathname][name]["vel"].as<float>();
                release_task.path.push_back(point);
            }
            else break;
        }

        // // 移动到车后N米,添加车当前点和车后点,演示用
        // geometry_msgs::PoseStamped pose_map;
        // pose_zero.header.stamp = ros::Time::now();
        // transformPose("map", pose_zero, pose_map, "XXX");
        // point.pointX=pose_map.pose.position.x;
        // point.pointY=pose_map.pose.position.y;
        // point.pointHA=GetYawFromPose(pose_map)*180/M_PI;
        // point.vehSpeed=0.3;  //  运动速度
        // move_task.path.push_back(point);
        
        // pose_map=GetExtendPoseByPose(pose_map,-6);   // 后退点
        // point.pointX=pose_map.pose.position.x;
        // point.pointY=pose_map.pose.position.y;
        // point.pointHA=GetYawFromPose(pose_map)*180/M_PI;
        // point.vehSpeed=0.3;
        // move_task.path.push_back(point);

        for (int i = 0; i < 1000; i++)
        {
            char name[100];
            sprintf(name, "pose%d", i);
            string pathname = "move_path";
            if (config[pathname][name]["map_x"].IsDefined())
            {
                point.pointX = config[pathname][name]["map_x"].as<float>();
                point.pointY = config[pathname][name]["map_y"].as<float>();
                point.pointHA = config[pathname][name]["heading"].as<float>();
                point.vehSpeed = config[pathname][name]["vel"].as<float>();
                // cout << point.vehSpeed <<endl;
                move_task.path.push_back(point);
            }
            else break;
        }

        // mqtt_comm::path_point point1, point2;
        // point1 = move_task.path[0];
        // point2 = move_task.path[1];
        // float dx = point2.pointX - point1.pointX, dy = point2.pointY - point1.pointY;
        // float x = cos(point1.pointHA / 180 * M_PI), y = sin(point1.pointHA / 180 * M_PI);
        // float mul = dx * x + dy * y;
        // float add_x = dx - mul * x, add_y = dy - mul * y;
        // x = point1.pointX + add_x;
        // y = point1.pointY + add_y;
        // float zeta_st = atan2(point1.pointY - y, point1.pointX - x);
        // float zeta_en = atan2(point2.pointY - y, point2.pointX - x);
        // float r = sqrt(add_x * add_x + add_y * add_y);
        // float d_zeta = zeta_en - zeta_st
        // for (int i=0;i<20;i++)
        // {

        // }
    }

    void Panel_Global_Plan_Sim::PubTask(string cmd)
    {
        btn_stop_onclick();
        usleep(50000);

        if(cmd=="pick task")  cur_task=pick_task;
        else if(cmd=="release task")  cur_task=release_task;
        else if(cmd=="move task")  cur_task=move_task;
        else return;

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id="map";
        pose.header.stamp=cur_task.stamp;
        pose.pose.position.x=cur_task.path.front().pointX;
        pose.pose.position.y=cur_task.path.front().pointY;
        pose.pose.orientation=tf::createQuaternionMsgFromYaw(cur_task.path.front().pointHA/180.0*M_PI);
        simpose_pub.publish(pose);
        usleep(50000);
        track_path.poses.clear();

        cur_task.stamp = ros::Time::now();
        task_pub.publish(cur_task);
    }

    void Panel_Global_Plan_Sim::btn_pick_onclick()    //  发送取车路径
    {
        PubTask("pick task");
    }

    void Panel_Global_Plan_Sim::btn_release_onclick()    //  发送放车路径
    {
        PubTask("release task");
    }

    void Panel_Global_Plan_Sim::btn_move_onclick()    //  发送移动路径
    {
        PubTask("move task");
    }

    void Panel_Global_Plan_Sim::btn_stop_onclick()    //  发送停车指令
    {
        show_action_flag=0;
        cur_task.stamp = ros::Time::now();
        cur_task.cmd = "stop task";
        cur_task.path.clear();
        task_pub.publish(cur_task);
    }

    void Panel_Global_Plan_Sim::btn_cleartrack_onclick()    //  清除跟踪轨迹
    {
        track_path.poses.clear();
    }

    void Panel_Global_Plan_Sim::btn_syscheck_onclick()   //  系统状态检查
    {
        d_nodecheck=new dialog_node_check(this);
        d_nodecheck->setModal(true);
        d_nodecheck->show();
    }

    void Panel_Global_Plan_Sim::btn_enable_onclick() //  系统使能
    {
        nh_local->setParam("/pathtrack/run_enable", !run_enable);
    }

    void Panel_Global_Plan_Sim::SendTurnMoveAction(mqtt_comm::path_point &show_point, float turn_angle, float move_dis, float vel)
    {
        cur_task.cmd="move task";
        cur_task.stamp = ros::Time::now();
        cur_task.path.clear();    
        // mqtt_comm::path_point point=show_point;
        show_point.vehSpeed=vel;
        show_point.pointHA+=turn_angle;
        // point.pointHA=stop_angle;
        // point.pointX=p_map.pose.position.x;
        // point.pointY=p_map.pose.position.y;
        cur_task.path.push_back(show_point);

        // transformPose("map", pose_zero, p_map);
        // float stop_angle=GetYawFromPose(p_map)*180.0/M_PI+turn_angle;
        geometry_msgs::PoseStamped p_map;
        p_map.header.frame_id="map";
        p_map.pose.position.x=show_point.pointX;
        p_map.pose.position.y=show_point.pointY;
        p_map.pose.orientation=tf::createQuaternionMsgFromYaw(show_point.pointHA/180.0*M_PI);
        p_map=GetExtendPoseByPose(p_map, move_dis);
        show_point.pointX = p_map.pose.position.x;
        show_point.pointY = p_map.pose.position.y;
        cur_task.path.push_back(show_point);

        task_pub.publish(cur_task);
    }

    void Panel_Global_Plan_Sim::SendReturnAction(mqtt_comm::path_point &show_point, float move_dis, float vel)
    {
        // geometry_msgs::PoseStamped p_map;
        // transformPose("map", pose_zero, p_map);
        // float angle=GetYawFromPose(p_map)*180.0/M_PI;
            
        cur_task.cmd="move task";
        cur_task.stamp = ros::Time::now();
        cur_task.path.clear();    
        // mqtt_comm::path_point point;
        // point.vehSpeed=vel;
        // point.pointHA=angle;
        // point.pointX=p_map.pose.position.x;
        // point.pointY=p_map.pose.position.y;
        show_point.vehSpeed=vel;
        cur_task.path.push_back(show_point);

        geometry_msgs::PoseStamped p_map;
        p_map.header.frame_id="map";
        p_map.pose.position.x=show_point.pointX;
        p_map.pose.position.y=show_point.pointY;
        p_map.pose.orientation=tf::createQuaternionMsgFromYaw(show_point.pointHA/180.0*M_PI);
        p_map=GetExtendPoseByPose(p_map, -fabs(move_dis));
        show_point.pointX = p_map.pose.position.x;
        show_point.pointY = p_map.pose.position.y;
        cur_task.path.push_back(show_point);

        task_pub.publish(cur_task);
    }

    void Panel_Global_Plan_Sim::SendHYAction(mqtt_comm::path_point &show_point, float move_dis, float vel)
    {
        cur_task.cmd="move task";
        cur_task.stamp = ros::Time::now();
        cur_task.path.clear();    
        show_point.vehSpeed=vel;
        cur_task.path.push_back(show_point);

        geometry_msgs::PoseStamped p_map;
        p_map.header.frame_id="map";
        p_map.pose.position.x=show_point.pointX;
        p_map.pose.position.y=show_point.pointY;
        p_map.pose.orientation=tf::createQuaternionMsgFromYaw((show_point.pointHA+90)/180.0*M_PI);
        p_map=GetExtendPoseByPose(p_map, move_dis);
        show_point.pointX = p_map.pose.position.x;
        show_point.pointY = p_map.pose.position.y;
        cur_task.path.push_back(show_point);

        task_pub.publish(cur_task);
    }

    void Panel_Global_Plan_Sim::ShowActionProcess()
    {
        static mqtt_comm::path_point show_point;
        bool enable=false;
        nh_local->getParam("/show_action_enable", enable);
        if(enable) 
        {
            show_action_flag=1;
            nh_local->setParam("/show_action_enable", false);
        }

        static TTimer tmr;
        if(show_action_flag==1)
        {
            geometry_msgs::PoseStamped p_map;
            transformPose("map", pose_zero, p_map);
            show_point.pointHA=GetYawFromPose(p_map)*180.0/M_PI;
            show_point.pointX=p_map.pose.position.x;
            show_point.pointY=p_map.pose.position.y;

            SendHYAction(show_point, 3, 0.5);
            show_action_flag++;   tmr.Clear();
        }
        else if(show_action_flag==2 && tmr.GetValue()>17)
        {
            SendHYAction(show_point, -3, 0.5);
            show_action_flag++;   tmr.Clear();
        }
        else if(show_action_flag==3 && tmr.GetValue()>15)
        {
            SendTurnMoveAction(show_point,45, -3, 2);
            show_action_flag++;   tmr.Clear(); 
        }
        else if(show_action_flag==4 && tmr.GetValue()>25)
        {
            SendTurnMoveAction(show_point, 0, 3, 2);
            show_action_flag++;   tmr.Clear(); 
        }
        else if(show_action_flag==5 && tmr.GetValue()>13)
        {
            SendTurnMoveAction(show_point,45, 0.15, 2);
            show_action_flag++;   tmr.Clear(); 
        }
        else if(show_action_flag==6 && tmr.GetValue()>10)
        {
            show_action_flag=0;   tmr.Clear();
        }


        if(show_action_flag)  nh_local->setParam("/pathtrack/obs_stop_enable", false);
        else  nh_local->setParam("/pathtrack/obs_stop_enable", true);

        // else if(show_action_flag==4 && tmr.GetValue()>36)
        // {

        //     SendReturnAction(show_point, -2, 0.5);
        //     show_action_flag++;   tmr.Clear(); 
        // }
        // else if(show_action_flag==6 && tmr.GetValue()>10)
        // {
        //     SendTurnMoveAction(show_point, -45, 2, 0.5);
        //     show_action_flag++;   tmr.Clear(); 
        // }
        // else if(show_action_flag==7 && tmr.GetValue()>28)
        // {
        //     SendReturnAction(show_point, 2, 0.5);
        //     show_action_flag++;   tmr.Clear(); 
        // }
        // if(show_action_flag==8 && tmr.GetValue()>10)
        // {
        //     SendHYAction(show_point, 2, 0.5);
        //     show_action_flag++;   tmr.Clear(); 
        // }
        // else if(show_action_flag==9 && tmr.GetValue()>17)
        // {
        //     SendHYAction(show_point,-2, 0.5);
        //     show_action_flag++;   tmr.Clear(); 
        // }
        // else if(show_action_flag==10 && tmr.GetValue()>17)
        // {
        //     SendTurnMoveAction(show_point, 0, 2, 0.5);
        //     show_action_flag++;   tmr.Clear(); 
        // }
        // else if(show_action_flag==11 && tmr.GetValue()>17)
        // {
        //     SendReturnAction(show_point, 2, 0.5);
        //     show_action_flag++;   tmr.Clear(); 
        // }
        // else if(show_action_flag==12 && tmr.GetValue()>17)
        // {
        //     SendTurnMoveAction(show_point, -90, 0.2, 0.5);
        //     show_action_flag++;   tmr.Clear(); 
        // }

        // ROS_INFO("%d", show_action_flag);
    }

    void Panel_Global_Plan_Sim::btn_backup_onclick() //  备用按钮,用于零食调试
    {
        //  SendTurnMoveAction(-90, 0.2, 0.5);
        // SendHYAction(-2,0.5);
        nh_local->setParam("/show_action_enable", true);

        // cur_task.cmd = "show task";
        // cur_task.path.clear();
        // mqtt_comm::path_point point;
        // point.pointHA = -180;
        // cur_task.path.push_back(point);
        // cur_task.stamp = ros::Time::now();
        // task_pub.publish(cur_task);
        
        // int n=track_path.poses.size();
        // if(n<3)  return;
        
        // geometry_msgs::PoseStamped p0, p1, p2;
        // p0=track_path.poses.front();
        // p2=track_path.poses.back();        
        // p1=track_path.poses[n/2];

        // float a1=GetAngleByPoints(p0.pose.position, p1.pose.position)*180/M_PI;
        // float a2=GetAngleByPoints(p1.pose.position, p2.pose.position)*180/M_PI;
        // ROS_INFO("angle=%.3fdeg", a2-a1);
    }

} // end namespace rviz_teleop_commander

// 声明此类是一个rviz的插件
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_gui::Panel_Global_Plan_Sim, rviz::Panel)
// END_TUTORIAL
