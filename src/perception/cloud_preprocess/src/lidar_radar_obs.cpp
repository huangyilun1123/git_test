#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <common/public.h>
#include <data_comm/car_state.h>
#include <data_comm/car_ctr.h>
#include "sensor_msgs/PointCloud2.h"
#include "cloud_preprocess.h"

using namespace std;

ros::NodeHandle *nh;
ros::Subscriber lidar_sub, radar_sub;
ros::Publisher obs_pub, region_pub, cloud_pub;
THeartbeat ht_lidar, ht_radar;
visualization_msgs::MarkerArray lidar_obs, radar_obs, lidar_region;
nav_msgs::Path localpath;

int car_turnmode=0;
float car_vel=0;

TNodeCheck *nodecheck;

void GetScaleXYByMarker(visualization_msgs::Marker marker, float &x_min, float &x_max, float &y_min, float &y_max)
{
    x_max = marker.scale.x * 0.5, x_min = -marker.scale.x * 0.5;
    y_max = marker.scale.y * 0.5, y_min = -marker.scale.y * 0.5;
    x_max += marker.pose.position.x, x_min += marker.pose.position.x;
    y_max += marker.pose.position.y, y_min += marker.pose.position.y;
}

//  判断点在多边形内距离窄边的距离, -1表示点不在多边形内部
bool GetObsDistanceInRegion(visualization_msgs::Marker marker, geometry_msgs::Point p, float &dx, float &dy)
{
    float x_max, x_min, y_max, y_min;
    GetScaleXYByMarker(marker, x_min, x_max, y_min, y_max);

    if(p.x>x_max || p.x<x_min || p.y>y_max || p.y<y_min)  return false;
    
    if(marker.scale.x>marker.scale.y && p.x>0)  dx=p.x-x_min, dy=p.y;
    else if(marker.scale.x>marker.scale.y && p.x<0)  dx=p.x-x_max, dy=p.y;
    
    if(marker.scale.x<marker.scale.y && p.y>0)  dx=p.y-y_min, dy=p.x;
    else if(marker.scale.x<marker.scale.y && p.y<0)  dx=p.y-y_max, dy=p.x;

    dx=fabs(dx),  dy=fabs(dy);

    return true;
}

void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &c_msg)
{
    double start_time = ros::Time::now().toSec();
    nodecheck->Find("node_rate")->Beat();

    PointCloud cloud;
    pcl::fromROSMsg(*c_msg, cloud);

    float x_min, x_max, y_min, y_max;
    GetScaleXYByMarker(lidar_region.markers.front(), x_min, x_max, y_min, y_max);
    // printf("%.2f-%.2f %.2f-%.2f\n", x_min, x_max, y_min, y_max);

    cloud = Cloud_PassThrough(cloud, x_min, x_max, y_min, y_max, -5, 5);
    cloud = CloudTf(cloud, "base_link");
    cloud = Cloud_PassThrough(cloud, x_min, x_max, y_min, y_max, 0.4, 2);
    lidar_obs=Clould_Cluster(cloud, 0.15,5,10000);

    sensor_msgs::PointCloud2 msgx;
    pcl::toROSMsg(cloud, msgx);
    msgx.header.frame_id = "base_link";
    cloud_pub.publish(msgx);

    // printf("size=%d\n", ma.markers.size());
    double stop_time = ros::Time::now().toSec();
    // ROS_INFO("spend time=%.3f\n", stop_time-start_time);
}

void LocalPathCallback(const nav_msgs::Path::ConstPtr &msg) // 接收局部路径
{
    localpath=*msg;    
}

void RadarObsCallback(const visualization_msgs::MarkerArray::ConstPtr &c_msg)
{
    // ht_radar.Beat();
    // radar_obs=*c_msg;

    // //  转换为base
    // for(auto &m:radar_obs.markers)
    // {
    //     geometry_msgs::PoseStamped pose, pose_base;
    //     pose.header=m.header;
    //     pose.pose=m.pose;
    //     transformPose("base_link", pose, pose_base);
    //     m.header=pose_base.header;
    //     m.pose=pose.pose;
    // }
}

void CarCtrCallback(const data_comm::car_ctr::ConstPtr &msg) //  接收车体状态信息
{
    car_turnmode = msg->turnmode;
    car_vel=msg->speed;
}

bool FindMarkerinArray(visualization_msgs::Marker m, visualization_msgs::MarkerArray ma)
{
    bool res=false;

    // printf("%.1f %.1f\n", m.pose.position.x, m.pose.position.y);
    for(int i=0;i<ma.markers.size();i++)
    {
        float dx = fabs(m.pose.position.x - ma.markers[i].pose.position.x);
        float dy = fabs(m.pose.position.y - ma.markers[i].pose.position.y);

        if(dx<=m.scale.x*1 && dy<=m.scale.y*1) res=true; 
    }
    return res;
}

float GetVelByDistnce(float max_d, float max_vel, float min_d, float min_vel, float cur_d)
{
    if(cur_d>=max_d)  return max_vel;
    else if(cur_d<=min_d)  return min_vel;
    else
    {
        float v=min_vel+(cur_d-min_d)/(max_d-min_d)*(max_vel-min_vel);
        return v;
    }
}

void InitLidarRegion(string caption)
{
    lidar_region.markers.clear();
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "lidar_region_marker";
    marker.lifetime = ros::Duration(0.2);
    marker.frame_locked = true;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r=marker.color.g=1.0f;
    marker.color.a = 0.1f;
    marker.scale.z = 0.1;
    marker.pose.orientation.w = 1;

    if(caption=="front" || caption=="back")
    {
        marker.scale.x = 15;
        marker.scale.y = 3;
        marker.pose.position.x=marker.scale.x*0.5+3.3;
        if(caption=="back")  marker.pose.position.x*=-1;
        marker.pose.position.y=0;
        marker.id = 0;
        lidar_region.markers.push_back(marker);
    }
    else if(caption=="right" || caption=="left")
    {
        marker.scale.x = 6.5;
        marker.scale.y = 15;
        marker.pose.position.y=marker.scale.y*0.5+1.6;
        if(caption=="right")  marker.pose.position.y*=-1;
        marker.pose.position.x=0;
        marker.id = 0;
        lidar_region.markers.push_back(marker);
    }
    // lidar_region.markers[1].color.r=1.0f;
}

float GetDistanceToPath(geometry_msgs::Point p)
{
    float res=0;
    if(localpath.poses.size()==0)  return res;

    geometry_msgs::PointStamped ps1_base;
    ps1_base.point=p;
    ps1_base.header.frame_id="base_link";
    ps1_base.header.stamp=ros::Time::now();
    int id=0;
    FindNearestPointInPath(localpath, ps1_base, id);

    geometry_msgs::PointStamped ps2_base, ps2;
    ps2.header=localpath.header;
    ps2.point=localpath.poses[id].pose.position;
    transformPoint("base_link", ps2, ps2_base);

    res=GetDistanceXY(ps1_base.point, ps2_base.point);
    return res;
}

void LidarRadarObsCheck()
{
    visualization_msgs::MarkerArray obs_msg;
    // if(ht_lidar.value>5.0 && ht_radar.value>5.0)
    // {
    //     // int n = 0;
    //     for (auto it = lidar_obs.markers.begin(); it != lidar_obs.markers.end(); ++it)
    //     {
    //         // for(int i=0;i<lidar_obs.markers.size();i++)  printf("%.1f %.1f\n", lidar_obs.markers[i].pose.position.x, lidar_obs.markers[i].pose.position.y);
    //         // printf("%.1f %.1f\n", it->pose.position.x, it->pose.position.y);
    
    //         if(FindMarkerinArray(*it, radar_obs))  
    //         {
    //             it->color.a = 0.4f;
    //             it->lifetime = ros::Duration(1.0);
    //             obs_msg.markers.push_back(*it);
    //             // n++;
    //             // printf("%d\n", n);
    //         }
    //     }

    //     //printf("%d\n", obs_msg.markers.size());
    // }
    // else if(ht_lidar.value>5)  obs_msg=lidar_obs;
    // else if(ht_radar.value>5)  obs_msg=radar_obs;

    obs_msg=lidar_obs;    //  仅使用激光雷达信息,毫米波雷达超前安装,转台需要转动才可以融合
    if(obs_msg.markers.size()>0)  obs_pub.publish(obs_msg);

    // 根据转向模式和路点方向判断有效检测区域
    float aim_angle=0;
    if(localpath.poses.size()>0) 
    {
        geometry_msgs::PoseStamped p_base;
        transformPose("base_link", localpath.poses.back(), p_base);
        aim_angle=atan2(p_base.pose.position.y, p_base.pose.position.x)*180/M_PI;

        // printf("%.1f\n", aim_angle);
    } 

    string direction="front";
    if(fabs(aim_angle-90)<20)  direction="left";
    else if (fabs(aim_angle+90)<20) direction="right";
    else if (fabs(aim_angle-180)<20 || fabs(aim_angle+180)<20) direction="back";
    InitLidarRegion(direction);
    region_pub.publish(lidar_region);

    float obs_dis=999;
    for(auto m:obs_msg.markers)
    {
        geometry_msgs::Point p[9];
        p[0]=m.pose.position;  
        p[1]=m.pose.position,  p[1].x+=m.scale.x*0.5,  p[1].y+=m.scale.y*0.5;
        p[2]=m.pose.position,  p[2].x-=m.scale.x*0.5,  p[2].y+=m.scale.y*0.5;
        p[3]=m.pose.position,  p[3].x+=m.scale.x*0.5,  p[3].y-=m.scale.y*0.5;
        p[4]=m.pose.position,  p[4].x-=m.scale.x*0.5,  p[4].y-=m.scale.y*0.5;

        p[5].x=0.5*(p[1].x+p[2].x),  p[5].y=0.5*(p[1].y+p[2].y);
        p[6].x=0.5*(p[2].x+p[4].x),  p[6].y=0.5*(p[2].y+p[4].y);
        p[7].x=0.5*(p[4].x+p[3].x),  p[7].y=0.5*(p[4].y+p[3].y);
        p[8].x=0.5*(p[1].x+p[3].x),  p[8].y=0.5*(p[1].y+p[3].y);

        for(int i=0;i<9;i++)   
        {
            float dx, dy;
            if(GetObsDistanceInRegion(lidar_region.markers[0], p[i], dx, dy))
            {
                if(localpath.poses.size()>0)  dy=GetDistanceToPath(p[i]);
                // printf("%f,%f\n", dx,dy);  
                if((direction=="front" || direction=="back") && dy<1.3)  dy=0;
                else if((direction=="left" || direction=="right") && dy<2)  dy=0;
                float my_ds=dx+dy*2;
                if (my_ds<obs_dis)  obs_dis = my_ds;
            }
        } 
    }

    float speedlimit=999;
    speedlimit=GetVelByDistnce(15,3,2.5,0,obs_dis);

    // printf("dis=%.2f vel=%.2f\n", obs_dis, speedlimit);
    nh->setParam("obstacle_dis", obs_dis);
    nh->setParam("speedlimit", speedlimit);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_radar_obs");
    nh=new ros::NodeHandle("~");

    nodecheck = new TNodeCheck(nh, "node_rate");
    nodecheck->Find("node_rate")->SetLimit(10);

    ros::Subscriber cloud_sub = nh->subscribe<sensor_msgs::PointCloud2>("/rslidar_points", 10, &PointCloudCallback);
    obs_pub = nh->advertise<visualization_msgs::MarkerArray>("obstale", 10);
    cloud_pub = nh->advertise<sensor_msgs::PointCloud2>("cloud_filter", 10);
    region_pub = nh->advertise<visualization_msgs::MarkerArray>("obs_check_region", 10);

    ros::Subscriber carctr_sub = nh->subscribe<data_comm::car_ctr>("/pathtrack/ctr_cmd", 1, CarCtrCallback);
    ros::Subscriber localpath_sub = nh->subscribe<nav_msgs::Path>("/local_path_plan/localpath", 10, LocalPathCallback);
    
    ros::Rate looprate(20);
    while(ros::ok())
    {
        LidarRadarObsCheck();
  
        ros::spinOnce();
        looprate.sleep();
    }
    return 0;
};
