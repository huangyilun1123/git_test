// 接收滤波后的消息，
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
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include "pcl/search/kdtree.h"
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <vector>
#include <algorithm>

using namespace std;

ros::Subscriber cloud_sub, keyboard_sub;
ros::Publisher box_pub, marker_pub, cloud_pub;
string frame_id_dest;

float cluster_distance = 1;
int cluster_number_min = 50, cluster_number_max = 10000;

// float min_height=0, min_width=0, min_length=0;

jsk_recognition_msgs::BoundingBoxArray Cluster(pcl::PointCloud<pcl::PointXYZI> cp)
{

    jsk_recognition_msgs::BoundingBoxArray boxarray;
    boxarray.header.frame_id = frame_id_dest;
    // ROS_INFO("AAAAAAAAAAAA");
    if (cp.size() <= 5)  return boxarray; // 点数过少，直接返回空数据

    // printf("%d\n", cp.size());

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

    pcl::PointXYZI minpoint, maxpoint;
    ROS_INFO("cluster_indices.size=%d",cluster_indices.size());
    for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZI> cloud_cluster;
        for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            pcl::PointXYZI p = cp[*pit];
            cloud_cluster.push_back(p);
        }
        pcl::getMinMax3D(cloud_cluster, minpoint, maxpoint);

        jsk_recognition_msgs::BoundingBox bb;
        bb.header.frame_id = frame_id_dest;
        // printf("dt=%.7f \n",(t2-t1).toSec());

        bb.pose.position.x = (maxpoint.x + minpoint.x) / 2;
        bb.pose.position.y = (maxpoint.y + minpoint.y) / 2;
        bb.pose.position.z = (maxpoint.z + minpoint.z) / 2;
        bb.dimensions.x = maxpoint.x - minpoint.x;
        bb.dimensions.y = maxpoint.y - minpoint.y;
        bb.dimensions.z = maxpoint.z - minpoint.z;
        printf("x=%.2f\n",bb.dimensions.x);

        boxarray.boxes.push_back(bb);

        // //if (bb.dimensions.x > 1.5 && bb.dimensions.x < 5 && bb.dimensions.y > 1.5 && bb.dimensions.y < 5 && bb.dimensions.z > 0.5 && bb.dimensions.z < 3)
        // if(bb.dimensions.x>=min_length && bb.dimensions.y>=min_width && bb.dimensions.z>=min_height)
        // {
        //     boxarray.boxes.push_back(bb);

        //     sensor_msgs::PointCloud2 msgx;
        //     pcl::toROSMsg(cloud_cluster, msgx);
        //     msgx.header.frame_id = "rslidar";
        //     cloud_pub.publish(msgx);
        // }
    }

    visualization_msgs::MarkerArray markerarray;
    visualization_msgs::Marker marker;
    markerarray.markers.clear();
    marker.header.frame_id = frame_id_dest;
    marker.header.stamp = ros::Time::now();
    marker.ns = "";
    marker.lifetime = ros::Duration(0.4);
    marker.frame_locked = true;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    for (int i = 0; i < boxarray.boxes.size(); ++i)
    {
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 0.5f;

        marker.id = i;
        marker.pose.position = boxarray.boxes[i].pose.position;
        marker.scale = boxarray.boxes[i].dimensions;
        markerarray.markers.push_back(marker);
    }

    // printf("%d\n",markerarray.markers.size());

    marker_pub.publish(markerarray);
    box_pub.publish(boxarray);
    return boxarray;
}

pcl::PointCloud<pcl::PointXYZI> remove_infinite_points(pcl::PointCloud<pcl::PointXYZI> cloud)
{
    pcl::PointCloud<pcl::PointXYZI> res;
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

void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &c_msg)
{
    // ROS_INFO("BBBBBBBBBBBBBB");
    frame_id_dest=c_msg->header.frame_id;

    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*c_msg, cloud);
    // ROS_INFO("CCCCCCCCCCCCCCC");
    cloud = remove_infinite_points(cloud);
    // ROS_INFO("DDDDDDDDDDDDDDD");
    Cluster(cloud);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_cluster");
    ros::NodeHandle nh("~");

    // sleep(1);
    string input_topic, output_box_topic, output_marker_topic;
    nh.getParam("input_topic", input_topic);
    nh.getParam("output_box_topic", output_box_topic);
    nh.getParam("output_marker_topic", output_marker_topic);

    nh.getParam("cluster_distance", cluster_distance);
    nh.getParam("cluster_number_min", cluster_number_min);
    nh.getParam("cluster_number_max", cluster_number_max);

    // nh.getParam("obs_min_height", min_height);
    // nh.getParam("obs_min_width", min_width);
    // nh.getParam("obs_min_length", min_length);


    cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_filter/filter_leida3", 10, &PointCloudCallback); // 订阅激光雷达原始点云
    
    box_pub = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("bound", 10);     // 发布聚类边界框
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("marker", 10);
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloudpub", 10);

    ros::spin();

    return 0;
};
