#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_cloud.h>
#include "pointcloud_type.h"

using namespace std;

string savefilepath="~/pcd/";
bool save_enable=false;

string DateTime2String()
{
    time_t now = time(NULL);
    struct tm *curTime;
    curTime = localtime(&now);

    char str[500];
    sprintf(str, "%04d%02d%02d_%02d%02d%02d", curTime->tm_year+1900, curTime->tm_mon+1, curTime->tm_mday, 
                                                   curTime->tm_hour, curTime->tm_min, curTime->tm_sec);

    return str;                                               
}

void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    if(!save_enable) return;

    PointCloud cloud;
    pcl::fromROSMsg(*msg, cloud);

    string filename=savefilepath+DateTime2String()+".pcd";
    // ROS_INFO("%s",filename.c_str());
    pcl::io::savePCDFile(filename, cloud);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "save_cloud");
    ros::NodeHandle nh("~");
    ROS_INFO("save cloud");

    nh.getParam("save_enable", save_enable);

    string input_topic;
    nh.getParam("input_topic", input_topic);
    ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(input_topic, 10, PointCloudCallback);//订阅激光雷达原始点云
    // ros::Publisher cloud_pub1 = nh.advertise<sensor_msgs::PointCloud2>("pcd_cloud1", 10);

    nh.getParam("savefile_path", savefilepath);
    if(savefilepath.length()>0 && savefilepath.back()!='/') savefilepath+="/";
    mkdir(savefilepath.c_str(),0777); 
    
    ros::Rate looprate(20);
    while(ros::ok())
    {
        // ROS_INFO("%s", DateTime2String().c_str());

        ros::spinOnce();
        looprate.sleep();
    }
    return 0;
};