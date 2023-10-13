#define PCL_NO_PRECOMPILE

#include <common/public.h>

#include <Eigen/Core>
#include <vector>
#include <algorithm>

#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <laser_geometry/laser_geometry.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>

//  本程序输入为线激光雷达点云,经过三维点云数据转换,范围滤波和聚类处理后,获得轮胎扫描线的长度,中心位置和距离
// BUG 速度1Hz，去掉欧式聚类

using namespace std;

void LaserScan2PointCloud(const sensor_msgs::LaserScan::ConstPtr &c_msg, pcl::PointCloud<pcl::PointXYZI> &cloud)
{
    //  转换为三维点云
    laser_geometry::LaserProjection projection;
    sensor_msgs::PointCloud2 cloud_msg;
    projection.projectLaser(*c_msg, cloud_msg);
    pcl::fromROSMsg(cloud_msg, cloud);
}

class Laserscan_Check
{
private:
    ros::NodeHandle *nh;
    ros::Subscriber laserscan_sub[4];
    ros::Publisher wheel_dis_pub, cloud_pub[4], distancemarker_pub;

    TNodeCheck *nodecheck;

public:
    float box_x_min = 0.05, box_x_max = 0.4;
    float box_y_min = -0.4, box_y_max = 0.4;

    geometry_msgs::Point center_point[4];
    
    Laserscan_Check()
    {
        nh = new ros::NodeHandle("~");

        // nh->getParam("box_x_min", box_x_min);
        // nh->getParam("box_x_max", box_x_max);
        // nh->getParam("box_y_min", box_y_min);
        // nh->getParam("box_y_max", box_y_max);

        laserscan_sub[0] = nh->subscribe<sensor_msgs::LaserScan>("/olelidar1/scan", 10, &Laserscan_Check::LaserScanCallback1, this);
        laserscan_sub[1] = nh->subscribe<sensor_msgs::LaserScan>("/olelidar2/scan", 10, &Laserscan_Check::LaserScanCallback2, this);
        laserscan_sub[2] = nh->subscribe<sensor_msgs::LaserScan>("/olelidar3/scan", 10, &Laserscan_Check::LaserScanCallback3, this);
        laserscan_sub[3] = nh->subscribe<sensor_msgs::LaserScan>("/olelidar4/scan", 10, &Laserscan_Check::LaserScanCallback4, this);
        
        cloud_pub[0] = nh->advertise<sensor_msgs::PointCloud2>("cloud1_filter", 10);
        cloud_pub[1] = nh->advertise<sensor_msgs::PointCloud2>("cloud2_filter", 10);
        cloud_pub[2] = nh->advertise<sensor_msgs::PointCloud2>("cloud3_filter", 10);
        cloud_pub[3] = nh->advertise<sensor_msgs::PointCloud2>("cloud4_filter", 10);

        wheel_dis_pub = nh->advertise<std_msgs::Float32MultiArray>("wheel_distance", 10);
        distancemarker_pub = nh->advertise<visualization_msgs::MarkerArray>("distance_markers", 10);

        nodecheck = new TNodeCheck(nh, "node_rate");
        nodecheck->Find("node_rate")->SetLimit(10);
    }

    void LaserScanCallback1(const sensor_msgs::LaserScan::ConstPtr &c_msg);
    void LaserScanCallback2(const sensor_msgs::LaserScan::ConstPtr &c_msg);
    void LaserScanCallback3(const sensor_msgs::LaserScan::ConstPtr &c_msg);
    void LaserScanCallback4(const sensor_msgs::LaserScan::ConstPtr &c_msg);

    pcl::PointCloud<pcl::PointXYZI> Cloud_PassThrough(pcl::PointCloud<pcl::PointXYZI> cp_src);
    void CloudProcess(int id, const sensor_msgs::LaserScan::ConstPtr &c_msg);
    void WheelDistancePublish();
};


// 范围滤波
pcl::PointCloud<pcl::PointXYZI> Laserscan_Check::Cloud_PassThrough(pcl::PointCloud<pcl::PointXYZI> cp_src)
{
    pcl::PointCloud<pcl::PointXYZI> res;

    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(cp_src.makeShared());
    pass.setFilterFieldName("x");
    pass.setFilterLimits(box_x_min, box_x_max);
    pass.setFilterLimitsNegative(false);
    pass.filter(res);

    pass.setInputCloud(res.makeShared());
    pass.setFilterFieldName("y");
    pass.setFilterLimits(box_y_min, box_y_max);
    pass.setFilterLimitsNegative(false);
    pass.filter(res);

    return res;
}


void Laserscan_Check::CloudProcess(int id, const sensor_msgs::LaserScan::ConstPtr &c_msg) // pcl::PointCloud<pcl::PointXYZI> cloud, geometry_msgs::Point &point)
{
    pcl::PointCloud<pcl::PointXYZI> cloud;
    LaserScan2PointCloud(c_msg, cloud);
    cloud = Cloud_PassThrough(cloud);

    // printf("%d=%d\n", id, cloud.size());

    sensor_msgs::PointCloud2 msgx;
    pcl::toROSMsg(cloud, msgx);
    msgx.header.frame_id = c_msg->header.frame_id;
    cloud_pub[id].publish(msgx);
    
    pcl::PointXYZI minpoint, maxpoint;
    pcl::getMinMax3D(cloud, minpoint, maxpoint);
    center_point[id].x = -1;   
    center_point[id].y = -1;   //  not reach
    center_point[id].z = 0;  
    if(cloud.points.size()>5)
    {
        center_point[id].x = (minpoint.x+maxpoint.x)*0.5;
        center_point[id].y = (minpoint.y+maxpoint.y)*0.5;
        center_point[id].z = maxpoint.y-minpoint.y;
        if(id==3 || id==0)  center_point[id].y*=-1; 
    }
}

void Laserscan_Check::WheelDistancePublish()
{
    bool working_flag[4];   //  判断雷达是否工作
    for(int i=0;i<4;i++)
    {
        float value=0; 
        char paramname[200];
        sprintf(paramname,"/olelidar%d/node_rate",i+1);
        nh->getParam(paramname, value);
        working_flag[i]=(value>5);
    }
    if(working_flag[1]!=working_flag[3])  
    {
        if(!working_flag[1])  center_point[1]=center_point[3];
        else if(!working_flag[3])  center_point[3]=center_point[1];
    }
    if(working_flag[0]!=working_flag[2])  
    {
        if(!working_flag[0])  center_point[0]=center_point[2];
        else if(!working_flag[2])  center_point[2]=center_point[0];
    }

    std_msgs::Float32MultiArray distance;
    for(int i=0;i<4;i++)
    {
        distance.data.push_back(center_point[i].x);
        distance.data.push_back(center_point[i].y);
        distance.data.push_back(center_point[i].z);
    }
    wheel_dis_pub.publish(distance);

    visualization_msgs::MarkerArray markerarray;
    visualization_msgs::Marker marker;
    marker.header.stamp=ros::Time::now();
    marker.ns = "distance_markers";
    marker.lifetime = ros::Duration(0.4);
    marker.frame_locked = true;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    marker.pose.position.z = 2;
    marker.scale.z = 0.5;
    marker.pose.position.x=-0.5;

    for(int i=0;i<4;i++)
    {
        char text[100];
        sprintf(text, "scan%d", i+1);
        marker.header.frame_id = text;
        marker.id = i;
        sprintf(text, "wrk: %d\nx:%.3f\ny:%.3f", working_flag[i], center_point[i].x, center_point[i].y);
        marker.text = text;
        markerarray.markers.push_back(marker);
    }

    distancemarker_pub.publish(markerarray);

    nodecheck->Find("node_rate")->Beat();
}

void Laserscan_Check::LaserScanCallback1(const sensor_msgs::LaserScan::ConstPtr &c_msg)
{
    CloudProcess(0, c_msg);
    // printf("aaaaaaaa\n");
}
void Laserscan_Check::LaserScanCallback2(const sensor_msgs::LaserScan::ConstPtr &c_msg)
{
    CloudProcess(1, c_msg);
}
void Laserscan_Check::LaserScanCallback3(const sensor_msgs::LaserScan::ConstPtr &c_msg)
{
    CloudProcess(2, c_msg);
}
void Laserscan_Check::LaserScanCallback4(const sensor_msgs::LaserScan::ConstPtr &c_msg)
{
    CloudProcess(3, c_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laserscan_check");

    Laserscan_Check lk;
    ros::Rate looprate(20);

    while (ros::ok())
    {
        lk.WheelDistancePublish();

        ros::spinOnce();
        looprate.sleep();
    }
    return 0;
}
