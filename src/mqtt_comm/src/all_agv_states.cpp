#include "ros/ros.h"
#include "tf/tf.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <visualization_msgs/MarkerArray.h>

#include <common/public.h>
#include <common/mydisplay.h>
#include "geometry_msgs/PoseStamped.h"
#include "mqtt_comm/resp_agvstate.h"

using namespace std;

ros::NodeHandle *nh;
// ros::Publisher task_pub;
double utm_x_zero = 0, utm_y_zero = 0;
TNodeCheck *nodecheck;

string agvId = "123";
const int agv_count = 20;
mqtt_comm::resp_agvstate agvstate_msg[agv_count];
ros::Publisher agvstate_pub[agv_count], agvpath_pub[agv_count];

void PubAgvState(int id)
{
    visualization_msgs::MarkerArray  markers;
    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "agv_marker";
    marker.lifetime = ros::Duration(2);
    marker.frame_locked = true;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r = marker.color.g = marker.color.b = 0.0f;
    marker.color.a = 0.1f;
    if(agvstate_msg[id].errcode==0)  marker.color.g=1.0f;   //  绿色表示正常
    else  marker.color.r=1.0f;  //  红色表示故障
    marker.pose.position.x = agvstate_msg[id].posX-utm_x_zero;
    marker.pose.position.y = agvstate_msg[id].posY-utm_y_zero;
    marker.pose.orientation = tf::createQuaternionMsgFromYaw((90+agvstate_msg[id].angle)/180.0*M_PI);
    marker.scale.x = 5.5;
    marker.scale.y = 3;
    marker.scale.z = 0.2;
    marker.id = 0;
    markers.markers.push_back(marker);

    // printf("%.2f, %.2f\n", marker.pose.position.x, marker.pose.position.y);

    marker.type = visualization_msgs::Marker::ARROW;
    marker.color.a = 0.2f;
    marker.scale.x = 2;
    marker.scale.y = 0.5;
    marker.id = 1;
    markers.markers.push_back(marker);

    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.color.r = marker.color.g = marker.color.b = 1.0f;
    marker.color.a = 1.0f;
    marker.scale.z = 0.6;
    marker.pose.position.z=2;
    geometry_msgs::PoseStamped pp;
    pp.pose=marker.pose;
    marker.pose= GetExtendPoseByPose(pp,-1.5).pose;

    string task_str;
    if(agvstate_msg[id].taskType==2)  task_str="pick";
    else if(agvstate_msg[id].taskType==3)  task_str="release";
    else if(agvstate_msg[id].taskType==10)  task_str="move";
    else if(agvstate_msg[id].taskType==8)  task_str="charge";
    else task_str="none";

    char agvstr[200];
    sprintf(agvstr, "agv%03d tsk:%s\nerr:%04x stp:%04x\nrem:%.1f soc:%.0f", id + 1, task_str.c_str(), agvstate_msg[id].errcode, agvstate_msg[id].stopcode, agvstate_msg[id].remain_path, agvstate_msg[id].batterySOC);
    marker.text = agvstr;
    marker.text=agvstr;
    marker.id = 2;
    markers.markers.push_back(marker);

    agvstate_pub[id].publish(markers);
    // if(id+1==5)  printf("AAAAA\n", id);
}

void AgvStateCallback(const mqtt_comm::resp_agvstate::ConstPtr &msg) 
{
    // printf("%s\n", msg->agvId.c_str());
    // if(agvId==msg->agvId)  return;

    int pos=msg->agvId.find("agv");
    if(pos==string::npos)  return;

    int id=atoi(msg->agvId.substr(pos+3,100).c_str())-1;
    if(id>=agv_count)  return;
    
    agvstate_msg[id]=*msg;
    PubAgvState(id);
}

void AgvTaskPathCallback(const nav_msgs::Path::ConstPtr &msg, int id)
{
    agvpath_pub[id].publish(*msg);
    // printf("tid=%d\n",id);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "all_agv_states");
    nh = new ros::NodeHandle("~");

    nh->getParam("/agvId", agvId);
    // ROS_INFO("agvId=%s", agvId.c_str());

    ros::Subscriber agvstate_sub=nh->subscribe<mqtt_comm::resp_agvstate>("/all_agvinfo", 10, AgvStateCallback);

    ros::Subscriber agv_taskpath_sub[agv_count];
    for(int i=0;i<agv_count;i++)
    {  
        char topicname[200];
        sprintf(topicname, "/all_agv_taskpath/A0000000000agv%03d", i+1);
        agv_taskpath_sub[i]=nh->subscribe<nav_msgs::Path>(topicname, 10, boost::bind(&AgvTaskPathCallback,_1,i));

        sprintf(topicname, "agv_marker_%d", i+1);
        agvstate_pub[i]=nh->advertise<visualization_msgs::MarkerArray>(topicname,10);

        sprintf(topicname, "agv_path_%d", i+1);
        agvpath_pub[i]=nh->advertise<nav_msgs::Path>(topicname,10);
    }

    nodecheck = new TNodeCheck(nh, "node_rate", 4);
    nodecheck->Find("node_rate")->SetLimit(0.1);

    ros::Rate loop_rate(20);
    while (ros::ok())
    {
        nh->getParam("/gps_base/utmx_zero", utm_x_zero);
        nh->getParam("/gps_base/utmy_zero", utm_y_zero);
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
