#define PCL_NO_PRECOMPILE

#include "pointcloud_type.h"

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
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <vector>
#include <algorithm>
#include <geometry_msgs/Polygon.h>
#include <pcl/filters/passthrough.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/segmentation/extract_clusters.h>
#include "pcl/search/kdtree.h"
#include <common/mydisplay.h>


using namespace std;

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

//坐标转换
PointCloud CloudTf(PointCloud cp_src, string frame_id)
{
    static tf::TransformListener listener;

    PointCloud res;
    vector<int> aa;
    pcl::removeNaNFromPointCloud(cp_src, res, aa); //移除无效点，cp为初始点云，res为移除无效点后的点云
    
    if (cp_src.header.frame_id != frame_id)
    {
        for (auto &it:res)
        {
            geometry_msgs::PointStamped src_p, dst_p;
            src_p.header.frame_id = res.header.frame_id;

            src_p.point.x = it.x;
            src_p.point.y = it.y;
            src_p.point.z = it.z;
            transformPoint(frame_id, src_p, dst_p, "AAA");
            it.x = dst_p.point.x;
            it.y = dst_p.point.y;
            it.z = dst_p.point.z;
        }
        // pcl_ros::transformPointCloud(frame_id, res, res, listener); //第一个参数为目标坐标系。第二个参数为原始点云，第三个参数为目标点云，第四个参数为接受到的坐标
        res.header.frame_id = frame_id;
    }

    return res;
}


PointCloud Cloud_PassThrough(PointCloud cp_src, float x_min, float x_max, float y_min, float y_max, float z_min, float z_max)
{
    PointCloud res;

    pcl::PassThrough<PointType> pass;
    pass.setInputCloud(cp_src.makeShared());
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_min, z_max);
    pass.setFilterLimitsNegative(false);
    pass.filter(res);

    pass.setInputCloud(res.makeShared());
    pass.setFilterFieldName("x");
    pass.setFilterLimits(x_min, x_max);
    pass.setFilterLimitsNegative(false);
    pass.filter(res);

    pass.setInputCloud(res.makeShared());
    pass.setFilterFieldName("y");
    pass.setFilterLimits(y_min, y_max);
    pass.setFilterLimitsNegative(false);
    pass.filter(res);

    return res;
};

PointCloud Cloud_RadiusFilter(PointCloud cp_src, float radius, int min_count)
{
    PointCloud res=cp_src;
    if(cp_src.size()==0)  return res;

    pcl::RadiusOutlierRemoval<PointType> outrem;
    outrem.setInputCloud(cp_src.makeShared());
    outrem.setRadiusSearch(radius);
    outrem.setMinNeighborsInRadius(min_count);
    outrem.filter(res);

    return res;
}

void GetScaleXYByPolygon(geometry_msgs::Polygon polygon, float &x_min, float &x_max, float &y_min, float &y_max)
{
    x_max = -10000, x_min = 10000;
    y_max = -10000, y_min = 10000;
    for (auto pp : polygon.points)
    {
        if (x_max < pp.x)  x_max = pp.x;
        else if (x_min > pp.x)  x_min = pp.x;
        if (y_max < pp.y)  y_max = pp.y;
        else if (y_min > pp.y)  y_min = pp.y;
    }

}

PointCloud Cloud_DownSampling(PointCloud cp_src, geometry_msgs::Point scale)
{
    pcl::VoxelGrid<PointType> sor;
    sor.setInputCloud(cp_src.makeShared());
    sor.setLeafSize(scale.x, scale.y, scale.z);

    PointCloud res;
    sor.filter(res);
    return res;
}

class TMinBoundingBox
{
private:
    ros::NodeHandle *nh;
    ros::Publisher markers_pub;
    PointCloud cloud;
    float precision=0.1, min_angle=-10, max_angle=10;
    void minscalebyrotate(float angle, geometry_msgs::Point &scale, geometry_msgs::Point &center);
    
public:
    geometry_msgs::PoseStamped pose;
    geometry_msgs::Point scale;
    float bouding_angle = 0;

    TMinBoundingBox(string caption);
    void Calculate(PointCloud c, float pre=0.2, float min_a=-10, float max_a=10);
    void Publish();
};

TMinBoundingBox::TMinBoundingBox(string caption)
{
    nh=new ros::NodeHandle("~");
    markers_pub = nh->advertise<visualization_msgs::MarkerArray>(caption, 10);
};

void TMinBoundingBox::minscalebyrotate(float angle, geometry_msgs::Point &scale, geometry_msgs::Point &center)
{
    if (cloud.size() < 100)  return;

    float max_x = -1000, min_x = 1000, max_y = -1000, min_y = 1000;
    for (auto it : cloud)
    {
        float x, y;
        x = it.x * cos(angle) + it.y * sin(angle);
        y = it.y * cos(angle) - it.x * sin(angle);
        if (x > max_x)
            max_x = x;
        if (x < min_x)
            min_x = x;
        if (y > max_y)
            max_y = y;
        if (y < min_y)
            min_y = y;
    }
    scale.x = max_x - min_x;
    scale.y = max_y - min_y;

    center.x = (min_x + max_x) * 0.5;
    center.y = (min_y + max_y) * 0.5;
};

void TMinBoundingBox::Calculate(PointCloud c, float pre, float min_a, float max_a)
{
    cloud = c;
    precision = pre, min_angle = min_a, max_angle = max_a;
    pose.header.frame_id = c.header.frame_id;
    pose.header.stamp = ros::Time::now();

    ros::Time start_time=ros::Time::now();

    // FILE *fp=fopen("/home/wsk/test_data.txt", "w");
    
    int n = (max_angle - min_angle) / precision;
    geometry_msgs::Point bouding_scale, bouding_center;
    bouding_scale.x = bouding_scale.y = 10000;
    for (int i = 0; i < n; i++)
    {
        float angle = (min_angle + i * precision) / 180.0 * M_PI;
        geometry_msgs::Point scale, center;
        minscalebyrotate(angle, scale, center);
        if (bouding_scale.x + bouding_scale.y > scale.x + scale.y)
        // if(bouding_scale.y>scale.y)
        {
            bouding_scale = scale;
            bouding_angle = angle;
            bouding_center = center;

        }
    }
    // fclose(fp);

    float min_z = 10000, max_z = -10000;
    for (auto it : cloud)
    {
        if (it.z >= max_z)
            max_z = it.z;
        if (it.z <= min_z)
            min_z = it.z;

    }
    bouding_center.z = (max_z + min_z) * 0.5;
    bouding_scale.z = max_z - min_z;
    if (bouding_scale.z < 0.01)
        bouding_scale.z = 0.01;

    pose.header.frame_id = cloud.header.frame_id;
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = bouding_center.x * cos(bouding_angle) - bouding_center.y * sin(bouding_angle);
    pose.pose.position.y = bouding_center.x * sin(bouding_angle) + bouding_center.y * cos(bouding_angle);
    pose.pose.position.z = bouding_center.z;
    // if(bouding_scale.x<bouding_scale.y)  bouding_angle-=M_PI*0.5;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(bouding_angle);
    scale = bouding_scale;
    ros::Time stop_time = ros::Time::now();
    // ROS_INFO("spend time=%.3f", (stop_time-start_time).toSec());
};

void TMinBoundingBox::Publish()
{
    visualization_msgs::MarkerArray markers;

    markers.markers.clear();
    visualization_msgs::Marker marker;
    marker.header.frame_id = cloud.header.frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "minboudingbox";
    marker.lifetime = ros::Duration(0.2);
    marker.frame_locked = true;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 0.2f;
    marker.pose = pose.pose;
    // printf("%f\n", marker.pose.position.z);
    // marker.pose.position.z = 0.4;
    marker.scale.x = scale.x;
    marker.scale.y = scale.y;
    marker.scale.z = 0.2;
    marker.id = 0;
    markers.markers.push_back(marker);

    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a = 1.0f;
    marker.scale.x = 0.5 * scale.x;
    marker.scale.y = 0.2* scale.y;
    marker.id = 1;
    markers.markers.push_back(marker);

    markers_pub.publish(markers);
}

visualization_msgs::MarkerArray Clould_Cluster(pcl::PointCloud<pcl::PointXYZI> cp, float cluster_distance, int cluster_number_min, int cluster_number_max)
{
    visualization_msgs::MarkerArray markerarray;
    if (cp.size() <= 5)  return markerarray; // 点数过少，直接返回空数据

    // 建立KdTree对象用来搜索
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cp.makeShared());

    vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(cluster_distance); // 设置聚类最小值，较小的值可能会导致对象被划分为多个簇，而较大的值可能会连接同一簇中的对象
    ec.setMinClusterSize(cluster_number_min); // 设置聚类的最小点云数
    ec.setMaxClusterSize(cluster_number_max); // 设置聚类的最大点云数
    ec.setSearchMethod(tree);
    ec.setInputCloud(cp.makeShared());
    ec.extract(cluster_indices);
    // ROS_INFO("n==%d\n",cluster_indices.size());

    pcl::PointXYZI minpoint, maxpoint;
   
    visualization_msgs::Marker marker;
    marker.header.frame_id = cp.header.frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "lidar_obs";
    marker.lifetime = ros::Duration(1);
    marker.frame_locked = true;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.3f;
    marker.id=0;
    for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZI> cloud_cluster;
        for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            pcl::PointXYZI p = cp[*pit];
            cloud_cluster.push_back(p);
        }
        pcl::getMinMax3D(cloud_cluster, minpoint, maxpoint);

        marker.pose.position.x = (maxpoint.x + minpoint.x) / 2;
        marker.pose.position.y = (maxpoint.y + minpoint.y) / 2;
        marker.pose.position.z = (maxpoint.z + minpoint.z) / 2;
        marker.scale.x = maxpoint.x - minpoint.x;
        marker.scale.y = maxpoint.y - minpoint.y;
        marker.scale.z = maxpoint.z - minpoint.z;
        markerarray.markers.push_back(marker);
        marker.id++;
    }

    return markerarray;
}
