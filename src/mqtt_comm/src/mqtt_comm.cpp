#include "ros/ros.h"
#include "tf/tf.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include <common/public.h>
#include <common/mydisplay.h>
#include "geometry_msgs/PoseStamped.h"


#include "mqtt_comm/controls.h"
#include "mqtt_comm/path_point.h"
#include "mqtt_comm/task.h"

#include "mqtt_comm/resp_agvstate.h"
#include "mqtt_comm/resp_task.h"
#include "mqtt_comm/resp_video.h"
#include "mqtt_comm/resp_ctrl.h"

#include <gps/MyGPS_msg.h>
#include <data_comm/car_state.h>
#include <data_comm/car_ctr.h>
#include <data_comm/paw_state.h>
#include <data_comm/battery_info.h>

using namespace std;

ros::NodeHandle *nh;
ros::Publisher resp_agvstate_pub, resp_task_pub, resp_video_pub, resp_ctrl_pub;
ros::Publisher task_pub;
double utm_x_zero = 0, utm_y_zero = 0;
mqtt_comm::task task_msg;
data_comm::paw_state paw_state;
TNodeCheck *nodecheck;

string agvId = "123";
mqtt_comm::resp_agvstate agvstate_msg;
float remain_path_length=999;
int next_task_path_id=-1;



void UpdateTaskPathId()
{
    if(task_msg.path.size()==0)
    {
        next_task_path_id=-1;
        return;
    }
    
    geometry_msgs::PoseStamped p_map;
    p_map.header.frame_id="map";
    p_map.header.stamp=ros::Time::now();
    p_map.pose.position.x=task_msg.path[next_task_path_id].pointX;
    p_map.pose.position.y=task_msg.path[next_task_path_id].pointY;
    float ds=GetDistance(p_map);
    if(ds<0.5 && next_task_path_id<task_msg.path.size()-1)  next_task_path_id++;
}

void GPSDataCallback(const gps::MyGPS_msg::ConstPtr &msg) //  gps信息
{
    agvstate_msg.lng = msg->Lon;
    agvstate_msg.lat = msg->Lat;
    agvstate_msg.posX = msg->UTM_X;
    agvstate_msg.posY = msg->UTM_Y;
    // agvstate_msg.vehSpeed = msg->Vel*3.6; //  km/h
    agvstate_msg.angle = msg->mqtt_angle;
}

void CarStateCallback(const data_comm::car_state::ConstPtr &msg) //  运动模式
{
    agvstate_msg.steerControlMode = msg->turnmode;
    agvstate_msg.autoDriveEnable = 1;//msg->ctrmode;
    agvstate_msg.vehCtrlMode = agvstate_msg.autoDriveEnable;
    agvstate_msg.cargoloadingStatus = msg->holdcar;
    // agvstate_msg.vehSpeed=msg->speed[0]*3.6;
    if(msg->turnmode==2)  agvstate_msg.vehSpeed=0.2;   //  在自转中给定虚拟速度
}

void CarCtrCallback(const data_comm::car_ctr::ConstPtr &msg) // 接收车辆控制指令
{
    agvstate_msg.vehSpeed=msg->speed*3.6;
    if(msg->turnmode==2)  agvstate_msg.vehSpeed=0.2;   //  在自转中给定虚拟速度
}

void PawStateCallback(const data_comm::paw_state::ConstPtr &msg)
{
    paw_state=*msg;
}

void BatteryCallback(const data_comm::battery_info::ConstPtr &msg) //  电池信息
{
    agvstate_msg.batterySOC = msg->SOC;
    agvstate_msg.batterySOH = msg->SOH;
    agvstate_msg.batteryCurrent = msg->current;
    agvstate_msg.batteryVoltage = msg->voltage;

    // printf("%.0f %.0f %.1f %.1f\n", agvstate_msg.batterySOC, agvstate_msg.batterySOH, agvstate_msg.batteryCurrent, agvstate_msg.batteryVoltage);
}

void RemainPathCallback(const std_msgs::Float64::ConstPtr &msg)
{
    remain_path_length=msg->data;
    agvstate_msg.remain_path=msg->data;
    // ROS_INFO("%.2f", remain_path_length); 
}

void Pub_AgvState() //  发布车辆状态
{
    agvstate_msg.msgType = "agvinfo";
    agvstate_msg.agvId = agvId;
    agvstate_msg.timestamp = ros::Time::now().toSec() * 1000;
    agvstate_msg.cargoVin = agvId;
    string paw_state_str;
    nh->getParam("/pawcontrol/paw_state", paw_state_str);

    float task_runtime=(ros::Time::now()-task_msg.stamp).toSec();  // 任务运行时间
    if (task_msg.cmd == "pick task")
    {
        if (paw_state_str == "wait_for_paw_drop")
            agvstate_msg.taskStatus = 2; // 自动导航中
        else if (paw_state_str == "paw_dropping")
            agvstate_msg.taskStatus = 5; // 自动取车中
        else if (paw_state_str == "baojia_done" && task_runtime>4)
        {
            agvstate_msg.taskStatus = 15; // 取车完成
        }
    }
    else if (task_msg.cmd == "release task")
    {
        if (paw_state_str == "baojia_done")
            agvstate_msg.taskStatus = 2; // 自动导航中
        else if (paw_state_str == "car_dropping")
            agvstate_msg.taskStatus = 8; // 自动放车中
        else if (paw_state_str == "wait_for_paw_drop" && task_runtime>4)
        {
            agvstate_msg.taskStatus = 15; // 放车完成 
        }
    }
    else if (task_msg.cmd == "move task")
    {
        if(remain_path_length<0.01 && task_runtime>4)  agvstate_msg.taskStatus=15; // 完成
        else agvstate_msg.taskStatus=2;
        // ROS_INFO("%.2f", remain_path_length);
    }

    // nh->getParam("/rfid_reader/cargoVin", agvstate_msg.cargoVin);
    // if(car_state.carhold_state==0x04 && agvstate_msg.cargoVin.find("e200")!=string::npos) 
    //     agvstate_msg.cargoloadingStatus=1;
    // else 
    //     agvstate_msg.cargoloadingStatus=0;
    
    // ROS_INFO("%d", agvstate_msg.cargoloadingStatus);

    // int errcode=0, stopcode=0;
    nh->getParam("/pathtrack/stop_code", agvstate_msg.stopcode);
    nh->getParam("/err_code", agvstate_msg.errcode);

    UpdateTaskPathId();
    agvstate_msg.nextTaskPathId = next_task_path_id;

    resp_agvstate_pub.publish(agvstate_msg);
}

void Resp_Controls(mqtt_comm::controls m) //  回应信号
{
    // printf("%s\n", m.msgType.c_str());

    if (m.msgType == "task") //  回应任务信号
    {
        mqtt_comm::resp_task msgx;
        msgx.agvId = agvId;
        msgx.msgType = "recvTask";
        msgx.timestamp = ros::Time::now().toSec() * 1000;
        msgx.taskId = m.taskId;
        msgx.subTaskIndex = m.subtaskIndex;
        resp_task_pub.publish(msgx);

        printf("任务反馈=%s\n", m.msgType.c_str());
    }
    else if (m.msgType == "videoControl") //  回应视频信号
    {
        mqtt_comm::resp_video msgx;
        msgx.agvId = agvId;
        msgx.msgType = "videoCtrlResp";
        msgx.timestamp = ros::Time::now().toSec() * 1000;
        msgx.result = 1;
        resp_video_pub.publish(msgx);

        // printf("%s\n", msgType.c_str());
    }
    else if (m.msgType == "ctrl") //  回应控制信号
    {
        mqtt_comm::resp_ctrl msgx;
        msgx.agvId = agvId;
        msgx.msgType = "respCtrl";
        msgx.cmdId = m.cmdId;
        msgx.timestamp = ros::Time::now().toSec() * 1000;
        msgx.ctrlType = m.ctrlType;
        resp_ctrl_pub.publish(msgx);

        // printf("%s\n", m.ctrlType.c_str());
    }
    else if (m.msgType == "sync")
    {
        nodecheck->Find("node_rate")->Beat();
        // ROS_INFO("beat\n");
    }
}

void UpdatePathByShow(string action, mqtt_comm::task &task)
{
    float distance2target=0;
    geometry_msgs::PoseStamped p_map;
    p_map.header.frame_id="map";
    p_map.header.stamp=ros::Time::now();
    if(task.path.size()>0)
    {
        p_map.pose.position.x=task.path.front().pointX;
        p_map.pose.position.y=task.path.front().pointY;
        p_map.pose.orientation=tf::createQuaternionMsgFromYaw(task.path.front().pointHA/180.0*M_PI);
        distance2target=GetDistance(p_map);
    }
    
    if(action=="pick show")  
    {
        p_map=GetExtendPoseByPose(p_map, -distance2target-1);
        mqtt_comm::path_point point;
        point.pointHA=task.path.front().pointHA;
        point.pointX=p_map.pose.position.x;
        point.pointY=p_map.pose.position.y;
        if(distance2target<7)  point.vehSpeed=0.2;
        else point.vehSpeed=0.5;
        task.path.insert(task.path.begin(), point);
    }
    else if(action=="return show")
    {
        task.path.clear();

        geometry_msgs::PoseStamped p_base;
        p_base.header.frame_id="base_link";
        p_base.header.stamp=ros::Time::now();
        p_base.pose.orientation.w=1;
        transformPose("map", p_base, p_map);
        mqtt_comm::path_point point;
        point.pointHA = GetYawFromPose(p_map)*180.0/M_PI;
        point.pointX = p_map.pose.position.x;
        point.pointY = p_map.pose.position.y;
        point.vehSpeed=0.5;
        task.path.push_back(point);

        float slow_distance=3;
        if(distance2target>slow_distance)
        {
            p_map=GetExtendPoseByPose(p_map, -slow_distance);
            point.pointX = p_map.pose.position.x;
            point.pointY = p_map.pose.position.y;
            point.vehSpeed=1;
            task.path.push_back(point);

            p_map=GetExtendPoseByPose(p_map, -(distance2target-slow_distance));
            point.pointX = p_map.pose.position.x;
            point.pointY = p_map.pose.position.y;
            point.vehSpeed=0.2;
            task.path.push_back(point);
        }
        else 
        {
            p_map=GetExtendPoseByPose(p_map, -distance2target);
            point.pointX = p_map.pose.position.x;
            point.pointY = p_map.pose.position.y;
            point.vehSpeed=0.2;
            task.path.push_back(point);
        }
    }
    else if(action=="move front")
    {
        task.path.clear();

        geometry_msgs::PoseStamped p_base;
        p_base.header.frame_id="base_link";
        p_base.header.stamp=ros::Time::now();
        p_base.pose.orientation.w=1;
        transformPose("map", p_base, p_map);
        mqtt_comm::path_point point;
        point.pointHA = GetYawFromPose(p_map)*180.0/M_PI;
        point.pointX = p_map.pose.position.x;
        point.pointY = p_map.pose.position.y;
        point.vehSpeed=0.3;
        task.path.push_back(point);

        p_map=GetExtendPoseByPose(p_map, 1.22);
        point.pointX = p_map.pose.position.x;
        point.pointY = p_map.pose.position.y;
        point.vehSpeed=0.2;
        task.path.push_back(point);
    }
    
   // printf("%s\n", action.c_str());
}

void Controls_Proc(mqtt_comm::controls m) //  处理控制指令
{
    if (m.msgType == "task") //  处理任务指令
    {
        // mqtt_comm::task task_msg;
        if (m.taskType == 2)
            task_msg.cmd = "pick task";
        else if (m.taskType == 3)
            task_msg.cmd = "release task";
        else if (m.taskType == 10)
            task_msg.cmd = "move task";
        else if (m.taskType == 8)
            task_msg.cmd = "charge task";
        else if (m.taskType == 11)  //  表演运动
        {
            task_msg.cmd = "none task";
            nh->setParam("/show_action_enable", true);
        }
        else if (m.taskType==12 || m.taskType==13)  //  表演取车 退出
        {
            if(m.taskType==13)  task_msg.cmd = "move task";
            else if(m.taskType==12)  task_msg.cmd = "pick task";
            mqtt_comm::path_point p;
            p.pointHA = m.targetHA;
            p.pointX = m.targetX;
            p.pointY = m.targetY;
            m.path.clear();
            m.path.push_back(p);
        }
        else if (m.taskType==15)   
        {
            task_msg.cmd = "move task";
            m.path.clear();
        }
        else
            task_msg.cmd = "none task",  m.path.clear();

        remain_path_length=999;    
        // printf("%s pathlength=%d\n", task_msg.cmd.c_str(), m.path.size());

        task_msg.stamp = ros::Time::now();
        task_msg.path = m.path;
        for (auto &p : task_msg.path)
        {
            if (fabs(p.pointX) > 100000 || fabs(p.pointY) > 100000) //  come from diaodu
            {
                p.pointX -= utm_x_zero, p.pointY -= utm_y_zero;
                p.pointHA += 90;
                if (p.pointHA >= 180)  p.pointHA -= 360;
                else if (p.pointHA < -180)   p.pointHA += 360;
                // printf("%.2f %.2f %.2f %.2f\n", p.pointX, p.pointY, p.pointHA, p.vehSpeed);
                p.vehSpeed/=3.6;  
            }
            
            (task_msg.path.end()-1)->vehSpeed = 0.2;
            next_task_path_id=0;
        }

        if(m.taskType==12)  UpdatePathByShow("pick show", task_msg);
        else if(m.taskType==13)  UpdatePathByShow("return show", task_msg);
        else if(m.taskType==15)  UpdatePathByShow("move front", task_msg);
        // printf("%d\n",m.taskType);

        task_pub.publish(task_msg);

        if(m.path.size()>0)
        {
            agvstate_msg.targetX = m.path.back().pointX;
            agvstate_msg.targetY = m.path.back().pointY;
            agvstate_msg.taskType = m.taskType;
            agvstate_msg.taskId = m.taskId;
        }
    }

    if (m.msgType == "ctrl") //  处理控制指令
    {
        // ROS_INFO("%s", m.msgType.c_str());
        mqtt_comm::task ctr_msg;
        ctr_msg.cmd=m.ctrlType+" ctrl";
        ctr_msg.stamp=ros::Time::now();
        if(m.ctrlType=="trafficLight")
        {
            if(m.lightStatus==1)  ctr_msg.subcmd="green";
            else ctr_msg.subcmd="red";
        }
        else if(m.ctrlType=="speedLimit")    //限速
        {
            mqtt_comm::path_point p;
            p.vehSpeed=m.value/3.6;
            ctr_msg.path.push_back(p);

            agvstate_msg.speedLimit=m.value;
        }
        else if(m.ctrlType=="taskStatus")    //任务状态
        {
            char buf[10];
            sprintf(buf, "%d", m.value);
            ctr_msg.subcmd=buf;
        }
        else if(m.ctrlType=="obstacleDetectionDisable")
        {
            char buf[10];
            sprintf(buf, "%d", m.value);
            ctr_msg.subcmd=buf;
        }
        task_pub.publish(ctr_msg);
    }
}

void controlsCallback(const mqtt_comm::controls::ConstPtr &msg)
{
    Controls_Proc(*msg); //  处理控制指令
    Resp_Controls(*msg); //  回应信号
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "mqtt_comm");
    nh = new ros::NodeHandle("~");

    nh->getParam("/agvId", agvId);
    ROS_INFO("agvId=%s", agvId.c_str());

    resp_agvstate_pub = nh->advertise<mqtt_comm::resp_agvstate>("/resp_agvstate", 10);
    resp_task_pub = nh->advertise<mqtt_comm::resp_task>("/resp_task", 10);
    resp_video_pub = nh->advertise<mqtt_comm::resp_video>("/resp_video", 10);
    resp_ctrl_pub = nh->advertise<mqtt_comm::resp_ctrl>("/resp_ctrl", 10);
    task_pub = nh->advertise<mqtt_comm::task>("/task_cmd", 10);

    ros::Subscriber controls_sub = nh->subscribe<mqtt_comm::controls>("/controls", 10, controlsCallback);
    ros::Subscriber gps_sub = nh->subscribe<gps::MyGPS_msg>("/gps_base/GPS_Base", 10, GPSDataCallback);
    ros::Subscriber carstate_sub = nh->subscribe<data_comm::car_state>("/can_comm/car_state", 10, CarStateCallback);
    ros::Subscriber carctr_sub = nh->subscribe<data_comm::car_ctr>("/pathtrack/ctr_cmd", 10, CarCtrCallback);
    ros::Subscriber battery_sub = nh->subscribe<data_comm::battery_info>("/can_comm/battery_info", 10, BatteryCallback);
    ros::Subscriber remainpath_sub = nh->subscribe<std_msgs::Float64>("/local_path_plan/remainpath", 10, RemainPathCallback);
    ros::Subscriber paw_state_sub = nh->subscribe<data_comm::paw_state>("/can_comm/paw_state", 10, PawStateCallback);

    float check_duration=4;
    nodecheck = new TNodeCheck(nh, "node_rate",check_duration);
    nodecheck->Find("node_rate")->SetLimit(0.1);
    
    ros::Rate loop_rate(15);
    while (ros::ok())
    {
        nh->getParam("/gps_base/utmx_zero", utm_x_zero);
        nh->getParam("/gps_base/utmy_zero", utm_y_zero);

        Pub_AgvState(); //  发送状态心跳

        // if (nodecheck->Find("node_rate")->value < 0.2)  ROS_ERROR_STREAM_THROTTLE(1, "mqtt comm error!");
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
