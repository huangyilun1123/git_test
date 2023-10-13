#define PCL_NO_PRECOMPILE

#include <common/public.h>

#include <Eigen/Core>
#include <vector>
#include <algorithm>

#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
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

class CLaserscan_Check
{
private:
    // 0~3内部线激光雷达，4~7外部线激光雷达
    ros::NodeHandle *nh_local;
    ros::Subscriber laserscan_sub[4], remainpath_sub, passedpath_sub;
    ros::Publisher wheel_err_pub, cloud_pub[4], distancemarker_pub;

    TNodeCheck *nodecheck;
    float remain_path_len = 0, passed_path_len = 0;
    float box_x_min = 0.05, box_x_max = 0.8 ;
    float box_y_min = 0, box_y_max = 5;
    geometry_msgs::Point center_point[4];

public:
    CLaserscan_Check(ros::NodeHandle* node);
    void LaserScanCallback1(const sensor_msgs::LaserScan::ConstPtr &c_msg);
    void LaserScanCallback2(const sensor_msgs::LaserScan::ConstPtr &c_msg);
    void LaserScanCallback3(const sensor_msgs::LaserScan::ConstPtr &c_msg);
    void LaserScanCallback4(const sensor_msgs::LaserScan::ConstPtr &c_msg);
    void RemainPathCallback(const std_msgs::Float64::ConstPtr &msg);
    void PassedPathCallback(const std_msgs::Float64::ConstPtr &msg);

    pcl::PointCloud<pcl::PointXYZI> Cloud_PassThrough(int id, pcl::PointCloud<pcl::PointXYZI>& cp_src);
    void CloudProcess(int id, const sensor_msgs::LaserScan::ConstPtr &c_msg);
    void WheelErrPublish();
};

CLaserscan_Check::CLaserscan_Check(ros::NodeHandle* node): nh_local(node)
{

    laserscan_sub[0] = nh_local->subscribe<sensor_msgs::LaserScan>("/olelidar1/scan", 10, &CLaserscan_Check::LaserScanCallback1, this);
    laserscan_sub[1] = nh_local->subscribe<sensor_msgs::LaserScan>("/olelidar2/scan", 10, &CLaserscan_Check::LaserScanCallback2, this);
    laserscan_sub[2] = nh_local->subscribe<sensor_msgs::LaserScan>("/olelidar3/scan", 10, &CLaserscan_Check::LaserScanCallback3, this);
    laserscan_sub[3] = nh_local->subscribe<sensor_msgs::LaserScan>("/olelidar4/scan", 10, &CLaserscan_Check::LaserScanCallback4, this);
    remainpath_sub = nh_local->subscribe<std_msgs::Float64>("/local_path_plan/remainpath", 10, &CLaserscan_Check::RemainPathCallback, this);
    passedpath_sub = nh_local->subscribe<std_msgs::Float64>("/local_path_plan/passedpath", 10, &CLaserscan_Check::PassedPathCallback, this);

    cloud_pub[0] = nh_local->advertise<sensor_msgs::PointCloud2>("cloud1_filter", 10);
    cloud_pub[1] = nh_local->advertise<sensor_msgs::PointCloud2>("cloud2_filter", 10);
    cloud_pub[2] = nh_local->advertise<sensor_msgs::PointCloud2>("cloud3_filter", 10);
    cloud_pub[3] = nh_local->advertise<sensor_msgs::PointCloud2>("cloud4_filter", 10);
    

    wheel_err_pub = nh_local->advertise<std_msgs::Float32MultiArray>("wheel_err", 10);

    nodecheck = new TNodeCheck(nh_local, "node_rate");
    nodecheck->Find("node_rate")->SetLimit(10);
}

void CLaserscan_Check::RemainPathCallback(const std_msgs::Float64::ConstPtr &msg)
{

    remain_path_len = msg->data;
}

void CLaserscan_Check::PassedPathCallback(const std_msgs::Float64::ConstPtr &msg)
{
    passed_path_len = msg->data;
}

// 范围滤波
pcl::PointCloud<pcl::PointXYZI> CLaserscan_Check::Cloud_PassThrough(int id, pcl::PointCloud<pcl::PointXYZI>& cp_src)
{
    // printf("%ld\n", cp_src.size());
    if (remain_path_len < 7 && remain_path_len > 0.5)
        box_y_max = remain_path_len; // box_y_min=-box_y_max;
    else if (remain_path_len < 1)
        box_y_max = 0.5;
    else if (passed_path_len < 7 && passed_path_len > 0.5)
        box_y_max = passed_path_len;
    else if (passed_path_len < 1)
        box_y_max = 0.5;

    float rear_laser_extend_y_min = 0;
    if (remain_path_len < 1 || passed_path_len < 1)
        rear_laser_extend_y_min = -0.5;

    pcl::PointCloud<pcl::PointXYZI> res;
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(cp_src.makeShared());
    pass.setFilterFieldName("x");
    pass.setFilterLimits(box_x_min, box_x_max);
    pass.setFilterLimitsNegative(false);
    pass.filter(res);
    pass.setInputCloud(res.makeShared());
    pass.setFilterFieldName("y");
    // printf("box_y_max:%f  min_y:%f\n",box_y_max, min_y);
    switch (id)
    {
    case 0:
    {
        pass.setFilterLimits(-box_y_max, -(box_y_min + rear_laser_extend_y_min));
        break;
    }
    case 1:
    {
        pass.setFilterLimits(-box_y_max, -box_y_min);
        break;
    }
    case 2:
    {
        pass.setFilterLimits(box_y_min + rear_laser_extend_y_min, box_y_max);
        break;
    }
    case 3:
    {
        pass.setFilterLimits(box_y_min, box_y_max);
        break;
    }
    default:
        break;
    }
    pass.setFilterLimitsNegative(false);
    pass.filter(res);

    // if (id == 4)printf("%ld %f %f\n", res.size(), -box_x_max[1], -box_x_min);

    return res;
}

void CLaserscan_Check::CloudProcess(int id, const sensor_msgs::LaserScan::ConstPtr &c_msg) // pcl::PointCloud<pcl::PointXYZI> cloud, geometry_msgs::Point &point)
{
    pcl::PointCloud<pcl::PointXYZI> cloud;

    laser_geometry::LaserProjection projection;
    sensor_msgs::PointCloud2 cloud_msg;
    projection.projectLaser(*c_msg, cloud_msg);
    pcl::fromROSMsg(cloud_msg, cloud);

    cloud = Cloud_PassThrough(id, cloud);
    // printf("%d=%d\n", id, cloud.size());

    sensor_msgs::PointCloud2 msgx;
    pcl::toROSMsg(cloud, msgx);
    msgx.header.frame_id = c_msg->header.frame_id;
    cloud_pub[id].publish(msgx);

    pcl::PointXYZI minpoint, maxpoint;
    pcl::getMinMax3D(cloud, minpoint, maxpoint);
    center_point[id].x = -1;
    center_point[id].y = -1; //  not reach
    center_point[id].z = 0;
    if (cloud.points.size() > 1)
    {
        center_point[id].x = minpoint.x; //+maxpoint.x)*0.5;
        // printf("%f\n", center_point[id].x);
        center_point[id].y = (minpoint.y + maxpoint.y) * 0.5;
        center_point[id].z = maxpoint.y - minpoint.y;
        // if (id == 1) ROS_INFO("%f, %f\n", center_point[1].x, center_point[3].x);
        if (id == 3 || id == 0)
            center_point[id].y *= -1;

        // printf("id=%d x=%.3f\n", id, minpoint.x);
    }
}

void CLaserscan_Check::WheelErrPublish()
{
    std_msgs::Float32MultiArray err;

    float dx = 999;
    if (center_point[0].x > 0 && center_point[2].x > 0)
        dx = center_point[2].x - center_point[0].x;
    err.data.push_back(dx);

    dx = 999;
    if (center_point[1].x > 0 && center_point[3].x > 0)
        dx = center_point[1].x - center_point[3].x;
    err.data.push_back(dx);

    wheel_err_pub.publish(err); // new
    nodecheck->Find("node_rate")->Beat();
}

void CLaserscan_Check::LaserScanCallback1(const sensor_msgs::LaserScan::ConstPtr &c_msg)
{
    CloudProcess(0, c_msg);
}
void CLaserscan_Check::LaserScanCallback2(const sensor_msgs::LaserScan::ConstPtr &c_msg)
{
    CloudProcess(1, c_msg);
}
void CLaserscan_Check::LaserScanCallback3(const sensor_msgs::LaserScan::ConstPtr &c_msg)
{
    CloudProcess(2, c_msg);
}
void CLaserscan_Check::LaserScanCallback4(const sensor_msgs::LaserScan::ConstPtr &c_msg)
{
    CloudProcess(3, c_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laserscan_check");
    ros::NodeHandle* node = new ros::NodeHandle("~");

    CLaserscan_Check lk(node);
    ros::Rate looprate(20);

    while (ros::ok())
    {
        lk.WheelErrPublish();

        ros::spinOnce();
        looprate.sleep();
    }
    return 0;
}
