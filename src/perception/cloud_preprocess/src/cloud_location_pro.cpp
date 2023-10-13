#define PCL_NO_PRECOMPILE

#include "cloud_preprocess.h"

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64.h>
#include <common/public.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mqtt_comm/task.h>
#include <nav_msgs/Path.h>
#include <pcl/io/pcd_io.h>

#include <opencv2/opencv.hpp>

#include <gps/MyGPS_msg.h>
#include <data_comm/car_ctr.h>

#include <vector>


using namespace std;

ros::NodeHandle *nh;
ros::Publisher turntablectr_pub, target_cloud_pub, base_cloud_pub, laserscan_target_pub;
int location_ctr = 0;

float start_pitchangle = 0, stop_pitchangle = -20, vel_pitchangle = 2;
float act_pitchangle;

float gps_angle = 0, localpath_target_angle = 0;
int car_turnmode = 2;

int reached_flag = 0;
PointCloud merge_target_cloud, merge_base_cloud;
TMinBoundingBox *mbb;

mqtt_comm::task cur_task;
float remain_path_length = 100;
bool lidar_work_enable = false;
bool laserscan_work_enable = false;

void pointcloudOBBPositionCalculationByOpenCV(const PointCloud &cloud_in, pcl::PointXYZ &min_point, pcl::PointXYZ &max_point, pcl::PointXYZ &centroid, double &angle)
{
    min_point.z = FLT_MAX;
    max_point.z = -FLT_MAX;

    for (auto point : cloud_in.points)
    {
        if (point.z < min_point.z)
            min_point.z = point.z;
        if (point.z > max_point.z)
            max_point.z = point.z;
    }

    std::vector<cv::Point2f> pts;
    for (auto point : cloud_in.points)
    {
        cv::Point2f pt;
        pt.x = point.x;
        pt.y = point.y;
        pts.push_back(pt);
    }

    cv::RotatedRect rect = cv::minAreaRect(pts);
    min_point.x = rect.center.x - rect.size.width / 2.0;
    min_point.y = rect.center.y - rect.size.height / 2.0;
    max_point.x = rect.center.x + rect.size.width / 2.0;
    max_point.y = rect.center.y + rect.size.height / 2.0;

    centroid.x = (max_point.x + min_point.x) / 2;
    centroid.y = (max_point.y + min_point.y) / 2;
    centroid.z = (max_point.z + min_point.z) / 2;

    angle = rect.angle; // degree
}

void TaskCallback(const mqtt_comm::task::ConstPtr &msg)
{
    if (msg->cmd.find("task") == string::npos)
        return;

    cur_task = *msg;
    location_ctr = 0;

    if (cur_task.cmd == "pick task" || cur_task.cmd == "release task")
    {
        location_ctr = 1;
        // printf("%s\n", cur_task.cmd.c_str());
    }

    // printf("target=%s\n", target_pose.header.frame_id.c_str());
}

void RemainPathCallback(const std_msgs::Float64::ConstPtr &msg)
{
    if (msg->data > 0.01)
        remain_path_length = msg->data;
    else remain_path_length = 999;
}

void LocalPathCallback(const nav_msgs::Path::ConstPtr &msg)
{
    if (msg->poses.size() == 0)
        return;
    auto pose = msg->poses.back();

    double roll, pitch, yaw;
    tf::Quaternion q;
    tf::quaternionMsgToTF(pose.pose.orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    localpath_target_angle = yaw * 180.0 / M_PI;
}

void CarCtrCallback(const data_comm::car_ctr::ConstPtr &msg) // 接收控制信号
{
    car_turnmode = msg->turnmode;
}

void GPSDataCallback(const gps::MyGPS_msg::ConstPtr &msg) //  接收gps信息
{
    gps_angle = msg->Angle;
}

void LidarPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &c_msg)
{
    if (location_ctr != 3 || !lidar_work_enable)
        return;

    double start_time = ros::Time::now().toSec();

    PointCloud raw_cloud, cloud;
    pcl::fromROSMsg(*c_msg, raw_cloud);

    raw_cloud = Cloud_PassThrough(raw_cloud, 1, 15, -2, 2, -5, 5);
    // raw_cloud = CloudTf(raw_cloud, "base_link");

    // 扫描前车
    cloud = CloudTf(raw_cloud, "target_link");
    cloud = Cloud_PassThrough(cloud, -2.3, 2.3, -1.2, 1.2, 0.4, 2);
    // cloud = Cloud_RadiusFilter(cloud, 0.1, 2);
    merge_target_cloud.header.frame_id = cloud.header.frame_id;
    merge_target_cloud += cloud;
    sensor_msgs::PointCloud2 msgx;
    pcl::toROSMsg(merge_target_cloud, msgx);
    msgx.header.frame_id = "target_link";
    target_cloud_pub.publish(msgx);

    if (cur_task.cmd == "release task") //  扫描本车
    {
        cloud = CloudTf(raw_cloud, "base_link");
        cloud = Cloud_PassThrough(cloud, 1.5, 3, -0.6, 0.6, 0.4, 2);
        cloud = Cloud_RadiusFilter(cloud, 0.1, 2);

        merge_base_cloud.header.frame_id = cloud.header.frame_id;
        merge_base_cloud += cloud;
        sensor_msgs::PointCloud2 msgx;
        pcl::toROSMsg(merge_base_cloud, msgx);
        msgx.header.frame_id = "base_link";
        base_cloud_pub.publish(msgx);
    }

    double stop_time = ros::Time::now().toSec();
    // ROS_INFO("spend time=%.3f\n", stop_time-start_time);
}

void LaserscanPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &c_msg)
{
    if (location_ctr != 1 || !laserscan_work_enable)
        return;

    location_ctr = 0;

    PointCloud raw_cloud, cloud;
    pcl::fromROSMsg(*c_msg, raw_cloud);

    raw_cloud = Cloud_PassThrough(raw_cloud, 1, 15, -2, 2, -5, 5);
    // raw_cloud = CloudTf(raw_cloud, "base_link");

    // 扫描前车
    cloud = CloudTf(raw_cloud, "target_link");
    cloud = Cloud_PassThrough(cloud, -2.3, 2.3, -1.2, 1.2, 0.4, 2);
    // cloud = Cloud_RadiusFilter(cloud, 0.1, 2);

    pcl::PointXYZ min_point, max_point, centroid;
    double angle;
    pointcloudOBBPositionCalculationByOpenCV(cloud, min_point, max_point, centroid, angle);
    // TODO

    visualization_msgs::MarkerArray markers;
    geometry_msgs::PoseStamped pose;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);

    
    markers.markers.clear();
    visualization_msgs::Marker marker;
    marker.header.frame_id = cloud.header.frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "minboudingbox";
    marker.lifetime = ros::Duration(1000);
    marker.frame_locked = true;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 0.2f;
    marker.pose = pose.pose;
    marker.scale.x = max_point.x - min_point.x;
    marker.scale.y = max_point.y - min_point.y;
    marker.scale.z = 0.2;
    marker.id = 0;
    markers.markers.push_back(marker);

    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a = 1.0f;
    marker.scale.x = 0.5 * marker.scale.x;
    marker.scale.y = 0.2* marker.scale.y;
    marker.id = 1;
    markers.markers.push_back(marker);

    laserscan_target_pub.publish(markers);

}

void TurntableCallback(const std_msgs::String::ConstPtr &msg)
{
    vector<string> ss = split(msg->data, ";");
    for (int i = 0; i < ss.size(); i++)
    {
        vector<string> subs = split(ss[i], " ");
        if (subs[0] == "pitch")
        {
            act_pitchangle = atof(subs[2].c_str());
            reached_flag = atoi(subs[3].c_str());
        }
    }
}

void PubTargetPose()
{
    TTimer tmr;
    geometry_msgs::Point scale;
    scale.x = scale.y = scale.z = 0.02;
    merge_target_cloud = Cloud_DownSampling(merge_target_cloud, scale);
    // pcl::io::savePCDFile("/home/wsk/ROS_Programing/pcd/k2.pcd", merge_target_cloud);

    mbb->Calculate(merge_target_cloud, 0.1, -10, 10);
    float angle = GetYawFromPose(mbb->pose);

    // printf("angle=%.2f time=%.3f\n", angle * 180 / M_PI, tmr.GetValue());

    if (cur_task.cmd == "pick task" && merge_target_cloud.size() > 100)
    {
        mbb->Publish();
    }
    else if (cur_task.cmd == "release task")
    {
        geometry_msgs::PointStamped p_target, p_base;
        p_target.header.frame_id = merge_target_cloud.header.frame_id;
        p_target.point.x = -mbb->scale.x * 0.5 + mbb->pose.pose.position.x;
        p_target.point.y = mbb->pose.pose.position.y;
        transformPoint("base_link", p_target, p_base, "SSSSSS");

        pcl::PointXYZI minpoint, maxpoint;
        pcl::getMinMax3D(merge_base_cloud, minpoint, maxpoint);

        // float ds2front = 0.3;
        // // geometry_msgs::PoseStamped px=GetExtendPoseByPose(mbb.pose, -0.5*mbb.scale.x-maxpoint.x-ds2front);
        // geometry_msgs::PoseStamped px = GetExtendPoseByPose(mbb.pose, -0.5 * mbb.scale.x); //-maxpoint.x-ds2front);
        // pose_pub.publish(px);

        // geometry_msgs::PoseStamped px_base;
        // transformPose("base_link", px, px_base);

        float distance = p_base.point.x - maxpoint.x;
        ROS_INFO("%.3f %.3f %.3f\n", distance, p_base.point.x, maxpoint.x);
    }
}

void TurntableCtr()
{
    char strbuf[500] = {0};
    const float lidar_location_distance = 7;
    const float diff_angle = 15;

    static TTimer run_tmr;
    // 任务检查、剩余长度检查、角度检查、转向模式检查
    if (location_ctr == 1 && (cur_task.cmd == "pick task" || cur_task.cmd == "release task") && 
        (remain_path_length < lidar_location_distance) && 
        (fabs(gps_angle - localpath_target_angle) < diff_angle) && (car_turnmode != 2))//转台复位
    {
        // flag = true;
        sprintf(strbuf, "PITCH MOV %.2f 40", start_pitchangle);
        location_ctr++;
        run_tmr.Clear();
    }
    else if (location_ctr == 2 && (reached_flag || act_pitchangle < start_pitchangle) && run_tmr.GetValue() > 0.3)//转台开始转
    {
        sprintf(strbuf, "PITCH MOV %.2f %.1f", stop_pitchangle, vel_pitchangle);
        merge_target_cloud.clear();
        merge_base_cloud.clear();
        location_ctr++;
        run_tmr.Clear();
    }
    else if (location_ctr == 3 && (reached_flag || fabs(act_pitchangle - stop_pitchangle) < 0.2) && run_tmr.GetValue() > 0.3)//结束
    {
        location_ctr = 0;
        PubTargetPose();
        sprintf(strbuf, "PITCH MOV %.2f 20", start_pitchangle);
    }
    // else if (location_ctr==4)  mbb->Publish();

    nh->setParam("location_ctr", location_ctr);

    if (strlen(strbuf) > 0)
    {
        std_msgs::String str_msg;
        str_msg.data = strbuf;
        turntablectr_pub.publish(str_msg);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_location_pro");
    nh = new ros::NodeHandle("~");
    mbb = new TMinBoundingBox("lidar_target");

    ros::Subscriber lidar_cloud_sub = nh->subscribe<sensor_msgs::PointCloud2>("/rslidar_points", 10, &LidarPointCloudCallback);
    ros::Subscriber laserscan_cloud_sub = nh->subscribe<sensor_msgs::PointCloud2>("/rslidar_points", 10, &LaserscanPointCloudCallback);

    // ros::Subscriber lidar_cloud_sub = nh->subscribe<sensor_msgs::PointCloud2>("/rslidar_points3", 10, &LidarPointCloudCallback);
    // ros::Subscriber laserscan_cloud_sub = nh->subscribe<sensor_msgs::PointCloud2>("/rslidar_points3", 10, &LaserscanPointCloudCallback);
    
    // ros::Subscriber lidar_cloud_sub = nh->subscribe<sensor_msgs::PointCloud2>("/rslidar_points2", 10, &LidarPointCloudCallback);
    // ros::Subscriber laserscan_cloud_sub = nh->subscribe<sensor_msgs::PointCloud2>("/rslidar_points2", 10, &LaserscanPointCloudCallback);
    


    ros::Subscriber turntable_sub = nh->subscribe<std_msgs::String>("/turntable/table_state", 10, &TurntableCallback);
    ros::Subscriber task_sub = nh->subscribe<mqtt_comm::task>("/task_cmd", 10, &TaskCallback);
    ros::Subscriber remainpath_sub = nh->subscribe<std_msgs::Float64>("/local_path_plan/remainpath", 10, &RemainPathCallback);
    ros::Subscriber localpath_sub = nh->subscribe<nav_msgs::Path>("/local_path_plan/localpath", 10, &LocalPathCallback);
    ros::Subscriber gps_sub = nh->subscribe<gps::MyGPS_msg>("/gps_base/GPS_Base", 10, &GPSDataCallback);
    ros::Subscriber carctr_sub = nh->subscribe<data_comm::car_ctr>("/pathtrack/ctr_cmd", 10, &CarCtrCallback);

    turntablectr_pub = nh->advertise<std_msgs::String>("/turntable_ctr", 10);
    target_cloud_pub = nh->advertise<sensor_msgs::PointCloud2>("merge_target_cloud", 10);
    base_cloud_pub = nh->advertise<sensor_msgs::PointCloud2>("merge_base_cloud", 10);
    laserscan_target_pub = nh->advertise<visualization_msgs::MarkerArray>("laserscan_target", 10);//TODO

    ROS_INFO("start");
    ros::Rate looprate(50);
    while (ros::ok())
    {
        nh->getParam("lidar_work_enable", lidar_work_enable);
        nh->getParam("laserscan_work_enable", laserscan_work_enable);

        if (lidar_work_enable)
            TurntableCtr();
        if (laserscan_work_enable)
            ; // TODO
        ros::spinOnce();
        looprate.sleep();
    }
    return 0;
};
