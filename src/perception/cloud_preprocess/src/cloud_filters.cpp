#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <geometry_msgs/PolygonStamped.h>

#include "pointcloud_type.h"
#include "common/public.h"

#include <vector>
#include <algorithm>

using namespace std;

ros::NodeHandle *nh;
ros::Subscriber cloud_sub1,cloud_sub2,cloud_sub3;
ros::Publisher cloud_pub1,cloud_pub2,cloud_pub3, polygon_pub;
float box_x_min = -20, box_x_max = 20;
float box_y_min = -20, box_y_max = 20;
float box_z_min = -0.9, box_z_max = 0.5;

float box_x_min_inside = -6, box_x_max_inside = 0.1;
float box_y_min_inside = -0.07, box_y_max_inside = 0.35;
float box_z_min_inside = -1.1, box_z_max_inside = 3;

bool radius_filter_enable = true;
float radius_value = 0.1;
float radius_min_points = 10;

bool intensity_filter_enable = true;
float intensity_max = 260;
float intensity_min = 0;

bool downsampling_enable = true;
float downsampling_scale = 0.01;

TNodeCheck *nodecheck;

//滤出无效点
PointCloud remove_infinite_points(PointCloud cloud)
{
    PointCloud res;
    res.header = cloud.header;

    for (auto it = cloud.points.begin(); it != cloud.points.end(); ++it)
    {
        if (pcl_isfinite(it->x) && pcl_isfinite(it->y) && pcl_isfinite(it->z))
        {
            res.push_back(*it);
        }
    }

    return res;
}

//滤掉外面的
PointCloud Cloud_PassThrough(PointCloud cp_src,int leida)
{
    if(leida == 2 || leida == 3)
    {
        box_y_min = -5;
        box_y_max = 2.2;
    }
    if(leida == 1 || leida == 4)
    {   
        box_y_min = -2.2;
        box_y_max = 5;
    }

    PointCloud res;

    pcl::PassThrough<PointType> pass;
    pass.setInputCloud(cp_src.makeShared());
    pass.setFilterFieldName("z");
    pass.setFilterLimits(box_z_min, box_z_max);
    pass.setFilterLimitsNegative(false);
    pass.filter(res);

    pass.setInputCloud(res.makeShared());
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

//半径采样
PointCloud Cloud_RadiusFilter(PointCloud cp_src)
{
    PointCloud res;

    pcl::RadiusOutlierRemoval<PointType> outrem;
    outrem.setInputCloud(cp_src.makeShared());
    outrem.setRadiusSearch(radius_value);
    outrem.setMinNeighborsInRadius(radius_min_points);
    outrem.filter(res);

    return res;
}

//强度采样
PointCloud Cloud_IntensityFilter(PointCloud cp_src)
{
    PointCloud res;
    res.header = cp_src.header;

    for (auto it : cp_src)
    {
        if (it.intensity > intensity_min && it.intensity < intensity_max)
            res.push_back(it);
    }

    return res;
}

//将采样
PointCloud Cloud_DownSampling(PointCloud cp_src, geometry_msgs::Point scale)
{
    pcl::VoxelGrid<PointType> sor;
    sor.setInputCloud(cp_src.makeShared());
    sor.setLeafSize(scale.x, scale.y, scale.z);

    PointCloud res;
    sor.filter(res);
    return res;
}


//滤出内部
PointCloud Cloud_PassThroughInside(PointCloud cp_src,int leida)
{
    if(leida == 2 || leida == 3)
    {
        box_y_min_inside = -0.1;
        box_y_max_inside = 0.35;
    }
    if(leida == 1 || leida == 4)
    {   
        box_y_min_inside = -0.35;
        box_y_max_inside = 0.1;
    }

    PointCloud cloud_out;

    cloud_out.clear();
    cloud_out.header = cp_src.header;
    for (auto point : cp_src.points)
    {
        if (point.x > box_x_min_inside && point.x < box_x_max_inside && point.y > box_y_min_inside && point.y < box_y_max_inside && point.z > box_z_min_inside && point.z < box_z_max_inside)
            continue;
        cloud_out.points.push_back(point);
    }
    cloud_out.width = cloud_out.points.size();
    cloud_out.height = cp_src.height;

    return cloud_out;
}

void PointCloudCallback1(const sensor_msgs::PointCloud2::ConstPtr &c_msg)
{
    // 发布滤波范围多边形
    geometry_msgs::PolygonStamped polygon;
    polygon.header.frame_id = c_msg->header.frame_id;
    polygon.header.stamp = ros::Time::now();
    geometry_msgs::Point32 p;
    p.x=box_x_max, p.y=box_y_max,  polygon.polygon.points.push_back(p);
    p.x=box_x_max, p.y=box_y_min,  polygon.polygon.points.push_back(p);
    p.x=box_x_min, p.y=box_y_min,  polygon.polygon.points.push_back(p);
    p.x=box_x_min, p.y=box_y_max,  polygon.polygon.points.push_back(p);
    // polygon_pub.publish(polygon);

    // printf("AAAAAAAAAAAAAAAAAAAAAAAAAAAA\n");

    PointCloud cloud0;
    pcl::fromROSMsg(*c_msg, cloud0);

    cloud0 = remove_infinite_points(cloud0);
    cloud0 = Cloud_PassThrough(cloud0,1);
    cloud0 = Cloud_PassThroughInside(cloud0,1);

    if (intensity_filter_enable)
        cloud0 = Cloud_IntensityFilter(cloud0);
    if (radius_filter_enable)
        cloud0 = Cloud_RadiusFilter(cloud0);

    if (downsampling_enable)
    {
        geometry_msgs::Point scale;
        scale.x = scale.y = scale.z = downsampling_scale;
        cloud0 = Cloud_DownSampling(cloud0, scale);
    }

    sensor_msgs::PointCloud2 msgx;
    pcl::toROSMsg(cloud0, msgx);
    msgx.header.frame_id = cloud0.header.frame_id;
    cloud_pub1.publish(msgx);
    // printf("BBBBBBBBBBBBBBBBBBB\n");
    nodecheck->Find("cloudpoint_rate")->Beat();
}

void PointCloudCallback2(const sensor_msgs::PointCloud2::ConstPtr &c_msg)
{
    // 发布滤波范围多边形
    geometry_msgs::PolygonStamped polygon;
    polygon.header.frame_id = c_msg->header.frame_id;
    polygon.header.stamp = ros::Time::now();
    geometry_msgs::Point32 p;
    p.x=box_x_max, p.y=box_y_max,  polygon.polygon.points.push_back(p);
    p.x=box_x_max, p.y=box_y_min,  polygon.polygon.points.push_back(p);
    p.x=box_x_min, p.y=box_y_min,  polygon.polygon.points.push_back(p);
    p.x=box_x_min, p.y=box_y_max,  polygon.polygon.points.push_back(p);
    // polygon_pub.publish(polygon);

    // printf("AAAAAAAAAAAAAAAAAAAAAAAAAAAA\n");

    PointCloud cloud0;
    pcl::fromROSMsg(*c_msg, cloud0);

    cloud0 = remove_infinite_points(cloud0);
    cloud0 = Cloud_PassThrough(cloud0,2);
    cloud0 = Cloud_PassThroughInside(cloud0,2);

    if (intensity_filter_enable)
        cloud0 = Cloud_IntensityFilter(cloud0);
    if (radius_filter_enable)
        cloud0 = Cloud_RadiusFilter(cloud0);

    if (downsampling_enable)
    {
        geometry_msgs::Point scale;
        scale.x = scale.y = scale.z = downsampling_scale;
        cloud0 = Cloud_DownSampling(cloud0, scale);
    }

    sensor_msgs::PointCloud2 msgx;
    pcl::toROSMsg(cloud0, msgx);
    msgx.header.frame_id = cloud0.header.frame_id;
    cloud_pub2.publish(msgx);
    // printf("BBBBBBBBBBBBBBBBBBB\n");
    nodecheck->Find("cloudpoint_rate")->Beat();
}

void PointCloudCallback3(const sensor_msgs::PointCloud2::ConstPtr &c_msg)
{
    // 发布滤波范围多边形
    geometry_msgs::PolygonStamped polygon;
    polygon.header.frame_id = c_msg->header.frame_id;
    polygon.header.stamp = ros::Time::now();
    geometry_msgs::Point32 p;

    box_y_min_inside = 0.07;
    box_y_max_inside = 3;
    p.x=box_x_max_inside, p.y=box_y_max_inside,  polygon.polygon.points.push_back(p);
    p.x=box_x_max_inside, p.y=box_y_min_inside,  polygon.polygon.points.push_back(p);
    p.x=box_x_min_inside, p.y=box_y_min_inside,  polygon.polygon.points.push_back(p);
    p.x=box_x_min_inside, p.y=box_y_max_inside,  polygon.polygon.points.push_back(p);
    polygon_pub.publish(polygon);

    // printf("AAAAAAAAAAAAAAAAAAAAAAAAAAAA\n");

    PointCloud cloud0;
    pcl::fromROSMsg(*c_msg, cloud0);

    cloud0 = remove_infinite_points(cloud0);
    cloud0 = Cloud_PassThrough(cloud0,3);
    cloud0 = Cloud_PassThroughInside(cloud0,3);

    if (intensity_filter_enable)
        cloud0 = Cloud_IntensityFilter(cloud0);
    if (radius_filter_enable)
        cloud0 = Cloud_RadiusFilter(cloud0);

    if (downsampling_enable)
    {
        geometry_msgs::Point scale;
        scale.x = scale.y = scale.z = downsampling_scale;
        cloud0 = Cloud_DownSampling(cloud0, scale);
    }

    sensor_msgs::PointCloud2 msgx;cloud_pub3;
    pcl::toROSMsg(cloud0, msgx);
    msgx.header.frame_id = cloud0.header.frame_id;
    cloud_pub3.publish(msgx);
    // printf("BBBBBBBBBBBBBBBBBBB\n");
    nodecheck->Find("cloudpoint_rate")->Beat();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_filters");
    ros::NodeHandle nh("~");
    nodecheck = new TNodeCheck(&nh, "node_rate cloudpoint_rate", 1);
    nodecheck->Find("node_rate")->SetLimit(20);
    nodecheck->Find("cloudpoint_rate")->SetLimit(10);

    // sleep(1);
    string input_topic, output_topic;
    nh.getParam("input_topic", input_topic);
    nh.getParam("output_topic", output_topic);
    // cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(input_topic, 10, &PointCloudCallback); // 订阅激光雷达原始点云
    
    cloud_sub1 = nh.subscribe<sensor_msgs::PointCloud2>("/rslidar_points1", 10, &PointCloudCallback1);   
    cloud_sub2 = nh.subscribe<sensor_msgs::PointCloud2>("/rslidar_points2", 10, &PointCloudCallback2);   
    cloud_sub3 = nh.subscribe<sensor_msgs::PointCloud2>("/rslidar_points3", 10, &PointCloudCallback3); 
    
    cloud_pub1 = nh.advertise<sensor_msgs::PointCloud2>("filter_leida_1", 10);   
    cloud_pub2 = nh.advertise<sensor_msgs::PointCloud2>("filter_leida_2", 10);
    cloud_pub3 = nh.advertise<sensor_msgs::PointCloud2>("filter_leida_3", 10);

    polygon_pub = nh.advertise<geometry_msgs::PolygonStamped>("filter_region", 10);

    nh.getParam("box_x_max", box_x_max);
    nh.getParam("box_x_min", box_x_min);
    nh.getParam("box_y_max", box_y_max);
    nh.getParam("box_y_min", box_y_min);
    nh.getParam("box_z_max", box_z_max);
    nh.getParam("box_z_min", box_z_min);

    nh.getParam("box_x_max_inside", box_x_max_inside);
    nh.getParam("box_x_min_inside", box_x_min_inside);
    nh.getParam("box_y_max_inside", box_y_max_inside);
    nh.getParam("box_y_min_inside", box_y_min_inside);
    nh.getParam("box_z_max_inside", box_z_max_inside);
    nh.getParam("box_z_min_inside", box_z_min_inside);

    ros::Rate looprate(50);
    while (ros::ok())
    {
        nh.getParam("intensity_filter_enable", intensity_filter_enable);
        nh.getParam("intensity_max", intensity_max);
        nh.getParam("intensity_min", intensity_min);

        nh.getParam("radius_filter_enable", radius_filter_enable);
        nh.getParam("radius", radius_value);
        nh.getParam("radius_min_points", radius_min_points);

        nh.getParam("downsampling_enable", downsampling_enable);
        nh.getParam("downsampling_scale", downsampling_scale);

        nodecheck->Find("node_rate")->Beat();

        ros::spinOnce();
        looprate.sleep();
    }
    return 0;
};
