#include "ros/ros.h"
#include "tf/tf.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include <common/public.h>
#include <common/mydisplay.h>
#include "geometry_msgs/PoseStamped.h"
#include "mqtt_comm/controls.h"

#include "mqtt_comm/cmd_Header.h"
#include "mqtt_comm/iot_controls.h"
#include "mqtt_comm/iot_taskpath.h"
#include "mqtt_comm/iot_tasks.h"

#include "mqtt_comm/fault_info.h"
#include "mqtt_comm/resp_iot.h"

#include "mqtt_comm/path_point.h"
#include "mqtt_comm/iot_path_point.h"

#include "mqtt_comm/task.h"
#include "mqtt_comm/cmd_resp.h"

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

ros::NodeHandle *nh_local,*nh;
ros::Publisher resp_iot_tasks_pub,resp_iot_taskpath_pub,resp_iot_controls_pub;
ros::Publisher resp_iot_pub;
ros::Publisher task_pub;
double utm_x_zero = 0, utm_y_zero = 0;
mqtt_comm::controls iot_control;
mqtt_comm::task task_msg;
data_comm::paw_state paw_state;
data_comm::car_state car_state;

TNodeCheck *nodecheck;
string agvId = "123";
mqtt_comm::resp_iot agvstate_msg;
float remain_path_length=999;
int next_task_path_id=-1;

//=====================================2023_9_06修改编译已通过==========================================================//
//          修改响应云端任务、控制、路径指令的发布的话题， 原来只在一个resp_iot_tasks话题中发布
//          已改为在不同响应任务发布不同的话题 /resp_iot_tasks resp_iot_taskpath resp_iot_controls
//------------------------------------message修改--------------------------------------------------------------//
//              修改resp_iot中的tasktype数据类型为int cmakelist已修改
//              修改resp_iot中的fault故障代码的数据类型为数组 新增fault_info.msg  cmakelist已修改
//              删去iot_control、iot_task、iot_taskpath 中未使用的timestamp等 与FMS协议保持一致 cmakelistlist已修改
//              status_code 名字修改为cmd_resp cmakelist已修改

//--------------------------------1. 接收IOT消息(控制指令)---------------------------------------------------//
void controls_Proc(mqtt_comm::iot_controls m)    //处理控制信号
{
    mqtt_comm::task ctr_msg;
    ctr_msg.cmd = "ctrl";
    //m.type
    // if(m.ctrlType=="taskStatus")
    // {
    //     char buf[10];
    //     sprintf(buf, "%d", m.value);
    //     ctr_msg.subcmd=buf;
    // }
    task_pub.publish(ctr_msg);    //给车端发路径指令
}
void Resp_controls(mqtt_comm::iot_controls m)    //回应控制信号
{
    double image_time =ros::Time::now().toSec();

    mqtt_comm::cmd_resp resp_ctrl_code;
    resp_ctrl_code.header = m.header;
    resp_ctrl_code.header.timestamp = to_string(image_time);
    resp_ctrl_code.code = 202;
    resp_iot_controls_pub.publish(resp_ctrl_code);
}
void controlsCallback(const mqtt_comm::iot_controls::ConstPtr &msg)   
{
    controls_Proc(*msg);   //处理控制信号
    Resp_controls(*msg);       //回应控制信号
    //iot_control.cmdId
}

//----------2. 接收IOT消息(路径指令)-------------//
void taskpath_Proc(mqtt_comm::iot_taskpath m)    //处理路径指令
{   
    task_msg.stamp = ros::Time::now();
    task_msg.cmd = "task";
    
    //-------赋值path坐标-------//
    task_msg.path.clear();
    task_msg.path.resize(m.path.size());
    int n=m.path.size();
    // printf("n=%d\n",n);
    for(int i=0;i<n;i++)
    {
        task_msg.path[i].pointX = m.path[i].pointX;
        task_msg.path[i].pointY = m.path[i].pointY;
        task_msg.path[i].pointHA = m.path[i].pointHA;
        task_msg.path[i].vehSpeed = m.path[i].speed_vmax;
    } 

    for(int i=0;i<n;i++)
        printf("path=%.2f",task_msg.path[i].pointX);
    printf("\n");

    //----path路径点的坐标变换------// 
    for (auto &p : task_msg.path)
    {
        if (fabs(p.pointX) > 100000 || fabs(p.pointY) > 100000) //  come from diaodu
        {
            p.pointX -= utm_x_zero, p.pointY -= utm_y_zero;
            p.pointHA=p.pointHA/M_PI*180;
            // p.pointHA += 90;
            // p.pointHA +=55;
            if (p.pointHA >= 180)  p.pointHA -= 360;
            else if (p.pointHA < -180)   p.pointHA += 360;
            // printf("%.2f %.2f %.2f %.2f\n", p.pointX, p.pointY, p.pointHA, p.vehSpeed);

            p.vehSpeed/=3.6;  
        }
        (task_msg.path.end()-1)->vehSpeed = 0.2;
        next_task_path_id=0;
    }
    task_pub.publish(task_msg);    //给车端发路径指令

    // if(m.path.size()>0)
    // {
    //     agvstate_msg.targetX = m.path.back().pointX;
    //     agvstate_msg.targetY = m.path.back().pointY;
    //     agvstate_msg.taskType = m.taskType;
    //     agvstate_msg.taskId = m.taskId;
    // }

    agvstate_msg.navi_id = m.navi_id;
}
void Resp_taskpath(mqtt_comm::iot_taskpath m)         //回应路径指令
{
    double image_time =ros::Time::now().toSec();

    mqtt_comm::cmd_resp resp_taskpath_code;
    resp_taskpath_code.header = m.header;
    resp_taskpath_code.header.timestamp = to_string(image_time);
    resp_taskpath_code.code = 202;
    resp_iot_taskpath_pub.publish(resp_taskpath_code);
}
void taskpathCallback(const mqtt_comm::iot_taskpath::ConstPtr &msg)   
{
    // ROS_INFO("111");
    taskpath_Proc(*msg);   //处理路径信号
    // ROS_INFO("222");
    Resp_taskpath(*msg);       //回应路径信号
    // ROS_INFO("333");
}

//----------3. 接收IOT消息(任务指令)-------------//
void tasks_Proc(mqtt_comm::iot_tasks m)     //处理任务指令
{
    task_msg.cmd = "task";
    if (m.taskType == 12)
        task_msg.cmd = "pick task";
    else if (m.taskType == 13)
        task_msg.cmd = "release task";
    else if (m.taskType == 7)
        task_msg.cmd = "move task";    
    else if (m.taskType == 14)
        task_msg.cmd = "charge task";
    else
        task_msg.cmd = "none task";

    agvstate_msg.task_id = m.taskId;
    agvstate_msg.task_type = m.taskType;
    agvstate_msg.target = m.target;
    agvstate_msg.sub_target = m.sub_target;
}
void Resp_tasks(mqtt_comm::iot_tasks m)  //回应任务指令
{
    double image_time =ros::Time::now().toSec();
    mqtt_comm::cmd_resp resp_tasks_code;
    resp_tasks_code.header = m.header;
    resp_tasks_code.header.timestamp = to_string(image_time);
    resp_tasks_code.code = 202;
    resp_iot_tasks_pub.publish(resp_tasks_code);
}
void tasksCallback(const mqtt_comm::iot_tasks::ConstPtr &msg)   
{
    //ROS_INFO("AAAAAAAAAAAAAAAAAAAAAA");
    tasks_Proc(*msg);   //处理任务指令
    Resp_tasks(*msg);       //回应任务信号
}

//----------1. 接收车端消息(夹爪消息)-------------//
void PawStateCallback(const data_comm::paw_state::ConstPtr &msg)
{
    //ROS_INFO("BBBBBBBBBBBBBBBBBBBBBBB");
    paw_state=*msg;
}
//----------2. 接收车端消息(速度消息)-------------//
void CarStateCallback(const data_comm::car_state::ConstPtr &msg)
{
    car_state=*msg;
}

//----------3. 接收车端消息(任务指令)-------------//
//----------4. 接收车端消息(任务指令)-------------//
//----------5. 接收车端消息(任务指令)-------------//

//----------上行数据-------------//
void PubIOT_AgvState()   //  发布车辆状态
{
    string paw_state_str;
    nh->getParam("/pawcontrol/paw_state", paw_state_str);
    float task_runtime=(ros::Time::now()-task_msg.stamp).toSec();  // 任务运行时间
    if (task_msg.cmd == "pick task")
    {
        if (paw_state_str == "wait_for_paw_drop")
            agvstate_msg.state_flow = 1; // 行驶中
        else if (paw_state_str == "paw_dropping")
            agvstate_msg.state_flow = 9; // 自动取车中
        else if (paw_state_str == "car_stop")
            agvstate_msg.state_flow = 13; // 到达取车准备点
        else if (paw_state_str == "baojia_done" && task_runtime>4)
            agvstate_msg.state_flow = 14; // 取车完成
    }
    else if (task_msg.cmd == "release task")
    {
        if (paw_state_str == "baojia_done")
            agvstate_msg.state_flow = 1; // 行驶中
        // else if (paw_state_str == "baojia_done")
        //     agvstate_msg.state_flow = 2; // 到达路径终点           
        else if (paw_state_str == "car_dropping")
            agvstate_msg.state_flow = 9; // 自动放车中
        else if (paw_state_str == "wait_for_paw_drop" && task_runtime>4)
            agvstate_msg.state_flow = 14; // 放车完成 
    }
    else if (task_msg.cmd == "move task")
    {
        if(remain_path_length<0.01 && task_runtime>4)  
            agvstate_msg.state_flow=15;    // 完成
        else agvstate_msg.state_flow=2;    // 自动导航中
    }

    agvstate_msg.task_mode = 1; //派发模式

    if(car_state.speed[0]>0) agvstate_msg.drv_direction = 1;//agv 行驶方向 前进 后退 停止
    else if(car_state.speed[0]<0) agvstate_msg.drv_direction = 2;
    else agvstate_msg.drv_direction = 0;

    agvstate_msg.act_type = 3; //车辆类型 固定为3

    resp_iot_pub.publish(agvstate_msg);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "iot_comm");
    nh_local = new ros::NodeHandle("~");
    nh = new ros::NodeHandle();

    // 接收IOT消息(任务and路径and控制)
    ros::Subscriber iot_tasks_sub = nh_local->subscribe<mqtt_comm::iot_tasks>("/iot_task", 10, tasksCallback);
    ros::Subscriber iot_taskpath_sub = nh_local->subscribe<mqtt_comm::iot_taskpath>("/iot_path", 10, taskpathCallback);
    ros::Subscriber iot_controls_sub = nh_local->subscribe<mqtt_comm::iot_controls>("/iot_control", 10, controlsCallback);

    // 接收车端消息
    ros::Subscriber paw_state_sub = nh_local->subscribe<data_comm::paw_state>("/can_comm/paw_state", 10, PawStateCallback);
    // ros::Subscriber gps_sub = nh->subscribe<gps::MyGPS_msg>("/gps_base/GPS_Base", 10, GPSDataCallback);
    ros::Subscriber carstate_sub = nh_local->subscribe<data_comm::car_state>("/can_comm/car_state", 10, CarStateCallback);
    // ros::Subscriber carctr_sub = nh->subscribe<data_comm::car_ctr>("/pathtrack/ctr_cmd", 10, CarCtrCallback);
    // ros::Subscriber battery_sub = nh->subscribe<data_comm::battery_info>("/can_comm/battery_info", 10, BatteryCallback);
    // ros::Subscriber remainpath_sub = nh->subscribe<std_msgs::Float64>("/local_path_plan/remainpath", 10, RemainPathCallback);
    // ros::Subscriber paw_state_sub = nh->subscribe<data_comm::paw_state>("/can_comm/paw_state", 10, PawStateCallback);

    // 回应IOT消息
    resp_iot_tasks_pub = nh_local->advertise<mqtt_comm::cmd_resp>("/resp_iot_tasks", 10);
    resp_iot_taskpath_pub = nh_local->advertise<mqtt_comm::cmd_resp>("/resp_iot_taskpath", 10);
    resp_iot_controls_pub = nh_local->advertise<mqtt_comm::cmd_resp>("/resp_iot_controls", 10);

    resp_iot_pub = nh_local->advertise<mqtt_comm::resp_iot>("/resp_iot", 10);

    // 发布给车端消息
    task_pub = nh_local->advertise<mqtt_comm::task>("/task_cmd", 10);

    ros::Rate loop_rate(50);
    while (ros::ok())
    {

        nh->getParam("/gps_base/utmx_zero", utm_x_zero);
        nh->getParam("/gps_base/utmy_zero", utm_y_zero);
        PubIOT_AgvState();   //  发布车辆状态
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}