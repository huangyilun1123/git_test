// 新添加，接受cloud_filter的消息，计算周围障碍物的距离

#define PCL_NO_PRECOMPILE
#include "cloud_preprocess.h"
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
ros::Subscriber cloud_sub1,cloud_sub2,cloud_sub3,cloud_sub4;
ros::Publisher polygon_pub_car,polygon_pub_tocar,cardistance_pub;

geometry_msgs::PolygonStamped polygon;
geometry_msgs::Point32 polygon_point;
pcl::PointXYZI minpoint_car_1, maxpoint_car_1,minpoint_tocar_1, maxpoint_tocar_1;
pcl::PointXYZI minpoint_car_3, maxpoint_car_3,minpoint_tocar_3, maxpoint_tocar_3;
pcl::PointXYZI minpoint_obstacle,maxpoint_obstacle;
float distance_obstacle1,distance_obstacle2,distance_obstacle3,distance_obstacle4;

float box_x_min = -20, box_x_max = 20;
float box_y_min = -20, box_y_max = 20;
float box_z_min = -0.9, box_z_max = 0.5;

float box_x_min_inside = -6, box_x_max_inside = 0.1;
float box_y_min_inside = -0.07, box_y_max_inside = 0.35;
float box_z_min_inside = -1.1, box_z_max_inside = 3;

bool radius_filter_enable = true;
float radius_value = 0.1;
float radius_min_points = 0.;

bool intensity_filter_enable = true;
float intensity_max = 260;
float intensity_min = 0;

bool downsampling_enable = true;
float downsampling_scale = 0.05;

TNodeCheck *nodecheck;

std_msgs::Float64 distance_car;
// geometry_msgs::Point distance_car;

// 滤出无效点
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

// 滤掉外面的
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

// 半径采样
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

// 强度采样
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

// 降采样
PointCloud Cloud_DownSampling(PointCloud cp_src, geometry_msgs::Point scale)
{
    pcl::VoxelGrid<PointType> sor;
    sor.setInputCloud(cp_src.makeShared());
    sor.setLeafSize(scale.x, scale.y, scale.z);

    PointCloud res;
    sor.filter(res);
    return res;
}


// 滤出内部
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

vector<pcl::PointIndices> Clould_Cluster3(const PointCloud &cp_src, float cluster_distance, int cluster_number_min, int cluster_number_max)
{
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
    tree->setInputCloud(cp_src.makeShared());

    vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(cluster_distance); // 设置聚类最小值，较小的值可能会导致对象被划分为多个簇，而较大的值可能会连接同一簇中的对象
    ec.setMinClusterSize(cluster_number_min); // 设置聚类的最小点云数
    ec.setMaxClusterSize(cluster_number_max); // 设置聚类的最大点云数
    ec.setSearchMethod(tree);
    ec.setInputCloud(cp_src.makeShared());
    ec.extract(cluster_indices);

    return cluster_indices;
}

float GetDis2CarRegion(const PointType &point)
{
    float cur_point_x = point.x, cur_point_y = point.y;
    float filter_point_x, filter_point_y;
    // if (cur_point_x < car_region.min_x)
    //     filter_point_x = car_region.min_x;
    // else if (cur_point_x > car_region.max_x)
    //     filter_point_x = car_region.max_x;
    // else filter_point_x = cur_point_x;

    // if (cur_point_y < car_region.min_y)
    //     filter_point_y = car_region.min_y;
    // else if (cur_point_y > car_region.max_y)
    //     filter_point_y = car_region.max_y;
    // else filter_point_y = cur_point_y;

    return P2P(filter_point_x, filter_point_y, cur_point_x, cur_point_y);
}



// 回调函数
void PointCloudCallback1(const sensor_msgs::PointCloud2::ConstPtr &c_msg)
{
    PointCloud cloud0;
    pcl::fromROSMsg(*c_msg, cloud0);
    cloud0 = remove_infinite_points(cloud0);
    pcl::getMinMax3D(cloud0, minpoint_obstacle, maxpoint_obstacle);
    
    // float distance_obstacle;
    distance_obstacle1 = minpoint_obstacle.x*minpoint_obstacle.x+maxpoint_obstacle.y*maxpoint_obstacle.y;
    distance_obstacle1 = sqrt(distance_obstacle1);
}

void PointCloudCallback2(const sensor_msgs::PointCloud2::ConstPtr &c_msg)
{
    PointCloud cloud0;
    pcl::fromROSMsg(*c_msg, cloud0);
    cloud0 = remove_infinite_points(cloud0);
    pcl::getMinMax3D(cloud0, minpoint_obstacle, maxpoint_obstacle);
    
    // float distance_obstacle;
    distance_obstacle2 = minpoint_obstacle.x*minpoint_obstacle.x+maxpoint_obstacle.y*maxpoint_obstacle.y;
    distance_obstacle2 = sqrt(distance_obstacle2);
}

void PointCloudCallback3(const sensor_msgs::PointCloud2::ConstPtr &c_msg)
{
    PointCloud cloud0;
    pcl::fromROSMsg(*c_msg, cloud0);
    cloud0 = remove_infinite_points(cloud0);
    pcl::getMinMax3D(cloud0, minpoint_obstacle, maxpoint_obstacle);


    distance_obstacle3 = 999;
    if (cloud0.size())
    {
        vector<pcl::PointIndices> cluster_indices = Clould_Cluster3(cloud0, 0.15, 5, 10000);
        for (auto indices : cluster_indices)
            for (auto pid : indices.indices)
            {
                PointType point = cloud0[pid];
                distance_obstacle3 = min(distance_obstacle3, GetDis2CarRegion(point));
            }
    }

    // float distance_obstacle;
    // distance_obstacle3 = minpoint_obstacle.x*minpoint_obstacle.x+maxpoint_obstacle.y*maxpoint_obstacle.y;
    // distance_obstacle3 = sqrt(distance_obstacle3);
    printf("distance3=%.2f\n",distance_obstacle3);
}

void PointCloudCallback4(const sensor_msgs::PointCloud2::ConstPtr &c_msg)
{
    PointCloud cloud0;
    pcl::fromROSMsg(*c_msg, cloud0);
    cloud0 = remove_infinite_points(cloud0);
    pcl::getMinMax3D(cloud0, minpoint_obstacle, maxpoint_obstacle);
    
    // float distance_obstacle;
    distance_obstacle4 = minpoint_obstacle.x*minpoint_obstacle.x+maxpoint_obstacle.y*maxpoint_obstacle.y;
    distance_obstacle4 = sqrt(distance_obstacle4);
}

// 发布话题
void pubdistance()
{

}

// main函数
int main(int argc, char **argv)
{
    ros::init(argc, argv, "check_obstacle");
    ros::NodeHandle nh("~");
    nodecheck = new TNodeCheck(&nh, "node_rate cloudpoint_rate", 1);
    nodecheck->Find("node_rate")->SetLimit(20);
    nodecheck->Find("cloudpoint_rate")->SetLimit(10);

    // cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(input_topic, 10, &PointCloudCallback); // 订阅激光雷达原始点云
    
    cloud_sub1 = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_filter/filter_leida1", 10, &PointCloudCallback1);   
    cloud_sub2 = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_filter/filter_leida2", 10, &PointCloudCallback2); 
    cloud_sub3 = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_filter/filter_leida3", 10, &PointCloudCallback3); 
    cloud_sub4 = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_filter/filter_leida4", 10, &PointCloudCallback4);     

    polygon_pub_car = nh.advertise<geometry_msgs::PolygonStamped>("filter_region_car", 10);
    polygon_pub_tocar = nh.advertise<geometry_msgs::PolygonStamped>("filter_region_tocar", 10);


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
