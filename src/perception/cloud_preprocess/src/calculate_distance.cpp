// 新添加，接受cloud_filter的消息，计算前后两车的距离

#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include <common/mydisplay.h>
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
#include <std_msgs/Float64.h>

#include "pointcloud_type.h"
#include "common/public.h"

#include <vector>
#include <algorithm>

using namespace std;

ros::NodeHandle *nh;
ros::Subscriber cloud_sub1,cloud_sub2,cloud_sub3;
ros::Publisher polygon_pub_car,polygon_pub_tocar,cardistance_pub,cloud_pub;

geometry_msgs::PolygonStamped polygon;
geometry_msgs::Point32 polygon_point;
pcl::PointXYZI minpoint_car_1, maxpoint_car_1,minpoint_tocar_1, maxpoint_tocar_1;
pcl::PointXYZI minpoint_car_3, maxpoint_car_3,minpoint_tocar_3, maxpoint_tocar_3;

float box_x_min = -20, box_x_max = 20;
float box_y_min = -20, box_y_max = 20;
float box_z_min = -0.9, box_z_max = 0.5;

float box_x_min_inside = -6, box_x_max_inside = 0.1;
float box_y_min_inside = -0.07, box_y_max_inside = 0.35;
float box_z_min_inside = -1.1, box_z_max_inside = 3;

bool radius_filter_enable = false;
float radius_value = 0.1;
float radius_min_points = 5;

bool intensity_filter_enable = false;
float intensity_max = 260;
float intensity_min = 0;

bool downsampling_enable = false;
float downsampling_scale = 0.05;

TNodeCheck *nodecheck;

std_msgs::Float64 distance_car;
// geometry_msgs::Point distance_car;

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
PointCloud Cloud_PassThrough(PointCloud cp_src,int leida, int goal)
{
    if(leida==3 && goal == 1)
    {
        box_x_min = -6; box_x_max = 0;
        box_y_min = 1; box_y_max = 2;
        box_z_min = -1.1; box_z_max = 3;
    }
    else if(leida==3 && goal == 2)
    {   
        box_x_min = 0; box_x_max = 6;
        box_y_min = 1; box_y_max = 2;
        box_z_min = -1.1; box_z_max = 3;
    }
    else if(leida==1 && goal == 1)
    {   
        box_x_min = -6; box_x_max = 0;
        box_y_min = -2; box_y_max = -1;
        box_z_min = -1.1; box_z_max = 3;
    }
    else if(leida==1 && goal == 2)
    {   
        box_x_min = 0; box_x_max = 6;
        box_y_min = -2; box_y_max = -1;
        box_z_min = -1.1; box_z_max = 3;
    }
    polygon.polygon.points.clear();
    polygon.header.frame_id = cp_src.header.frame_id;
    polygon.header.stamp = ros::Time::now();
    polygon_point.x=box_x_max, polygon_point.y=box_y_max,  polygon.polygon.points.push_back(polygon_point);
    polygon_point.x=box_x_max, polygon_point.y=box_y_min,  polygon.polygon.points.push_back(polygon_point);
    polygon_point.x=box_x_min, polygon_point.y=box_y_min,  polygon.polygon.points.push_back(polygon_point);
    polygon_point.x=box_x_min, polygon_point.y=box_y_max,  polygon.polygon.points.push_back(polygon_point);

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
        box_y_min_inside = -0.07;
        box_y_max_inside = 0.35;
    }
    if(leida == 1 || leida == 4)
    {   
        box_y_min_inside = -0.35;
        box_y_max_inside = 0.07;
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

    static tf::TransformListener listener;

    PointCloud cloud0,cloud_car,cloud_tocar;
    pcl::fromROSMsg(*c_msg, cloud0);

    cloud0 = remove_infinite_points(cloud0);
    if (intensity_filter_enable)
        cloud0 = Cloud_IntensityFilter(cloud0);
    cloud0 = Cloud_RadiusFilter(cloud0);

    // cloud0 = Cloud_PassThrough(cloud0,1);

    // for (auto &it : cloud0)
    // {
    //     geometry_msgs::PointStamped src_p, dst_p;
    //     src_p.header.frame_id = cloud0.header.frame_id;

    //     src_p.point.x = it.x;
    //     src_p.point.y = it.y;
    //     src_p.point.z = it.z;
    //     transformPoint("base_link", src_p, dst_p, "AAA");
    //     it.x = dst_p.point.x;
    //     it.y = dst_p.point.y;
    //     it.z = dst_p.point.z;
    // }

    cloud_car = Cloud_PassThrough(cloud0,1,1);
    polygon_pub_car.publish(polygon);

    cloud_tocar = Cloud_PassThrough(cloud0,1,2);
    polygon_pub_tocar.publish(polygon);

    pcl::getMinMax3D(cloud_car, minpoint_car_1, maxpoint_car_1);
    pcl::getMinMax3D(cloud_tocar, minpoint_tocar_1, maxpoint_tocar_1);
     
//     center_point[1].x = -1;   
//     center_point[1].y = -1;   //  not reach
//     center_point[1].z = 0;  
    
//     if(cloud0.points.size()>5)
//     {
//         center_point[1].z = minpoint_tocar.x-maxpoint_car.x;
//     }
//     // ROS_INFO("AAAAAAAAAAAAAAAA");
//     printf("distance = %.2f\n",center_point[1].z);
}


void PointCloudCallback3(const sensor_msgs::PointCloud2::ConstPtr &c_msg)
{
   static tf::TransformListener listener;

    PointCloud cloud0,cloud_car,cloud_tocar;
    pcl::fromROSMsg(*c_msg, cloud0);

    cloud0 = remove_infinite_points(cloud0);
    if (intensity_filter_enable)
        cloud0 = Cloud_IntensityFilter(cloud0);
    cloud0 = Cloud_RadiusFilter(cloud0);

    // cloud0 = Cloud_PassThrough(cloud0,1);

    // for (auto &it : cloud0)
    // {
    //     geometry_msgs::PointStamped src_p, dst_p;
    //     src_p.header.frame_id = cloud0.header.frame_id;

    //     src_p.point.x = it.x;
    //     src_p.point.y = it.y;
    //     src_p.point.z = it.z;
    //     transformPoint("base_link", src_p, dst_p, "AAA");
    //     it.x = dst_p.point.x;
    //     it.y = dst_p.point.y;
    //     it.z = dst_p.point.z;
    // }

    cloud_car = Cloud_PassThrough(cloud0,3,1);
    // polygon_pub_car.publish(polygon);

    cloud_tocar = Cloud_PassThrough(cloud0,3,2);
    // polygon_pub_tocar.publish(polygon);

    pcl::getMinMax3D(cloud_car, minpoint_car_3, maxpoint_car_3);
    pcl::getMinMax3D(cloud_tocar, minpoint_tocar_3, maxpoint_tocar_3);
     
    // center_point[2].x = -1;   
    // center_point[2].y = -1;   //  not reach
    // center_point[2].z = 0;  
    
    // if(cloud0.points.size()>5)
    // {
    //     center_point[2].z = minpoint_tocar.x-maxpoint_car.x;
    // }
    // // ROS_INFO("AAAAAAAAAAAAAAAA");
    // printf("distance = %.2f\n",center_point[2].z);
    sensor_msgs::PointCloud2 msgx;
    pcl::toROSMsg(cloud0, msgx);
    msgx.header.frame_id = cloud0.header.frame_id;
    cloud_pub.publish(msgx);
}

void pubdistance()
{
    distance_car.data = -1;

    float min_tocar,max_car;
    min_tocar = min(minpoint_tocar_3.x,minpoint_tocar_1.x);
    max_car = max(maxpoint_car_3.x,maxpoint_car_1.x);

    distance_car.data = min_tocar-max_car;
    printf("distance = %.2f\n",distance_car);

    cardistance_pub.publish(distance_car);
    // polygon_pub_car.publish(polygon);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "calculate_distance");
    ros::NodeHandle nh("~");
    nodecheck = new TNodeCheck(&nh, "node_rate cloudpoint_rate", 1);
    nodecheck->Find("node_rate")->SetLimit(20);
    nodecheck->Find("cloudpoint_rate")->SetLimit(10);

    // sleep(1);
    string input_topic, output_topic;
    nh.getParam("input_topic", input_topic);
    nh.getParam("output_topic", output_topic);
    // cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(input_topic, 10, &PointCloudCallback); // 订阅激光雷达原始点云
    
    cloud_sub1 = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_filter/filter_leida1", 10, &PointCloudCallback1);   
    cloud_sub3 = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_filter/filter_leida3", 10, &PointCloudCallback3); 
    

    polygon_pub_car = nh.advertise<geometry_msgs::PolygonStamped>("filter_region_car", 10);
    polygon_pub_tocar = nh.advertise<geometry_msgs::PolygonStamped>("filter_region_tocar", 10);

    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("filter", 10);  

    cardistance_pub = nh.advertise<std_msgs::Float64>("car_distance",10);

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

        pubdistance();
        ros::spinOnce();
        looprate.sleep();
    }
    return 0;
};
