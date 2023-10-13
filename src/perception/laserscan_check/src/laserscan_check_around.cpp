#define PCL_NO_PRECOMPILE

#include <common/public.h>
#include <common/mydisplay.h>

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

#include <common/public.h>
#include <mutex>

using namespace std;

float cluster_distance = 0.1;
int cluster_number_min = 5, cluster_number_max = 10000;

void LaserScan2PointCloud(const sensor_msgs::LaserScan::ConstPtr &c_msg, pcl::PointCloud<pcl::PointXYZI> &cloud)
{
    //  转换为三维点云
    laser_geometry::LaserProjection projection;
    sensor_msgs::PointCloud2 cloud_msg;
    projection.projectLaser(*c_msg, cloud_msg);
    pcl::fromROSMsg(cloud_msg, cloud);
}

class Laserscan_Check : Thread
{
private:
    int msg_flag = 0;
    ros::NodeHandle *nh;
    ros::Subscriber laserscan_sub[4], remainpath_sub;
    ros::Publisher wheel_err_pub, cloud_pub[4], distancemarker_pub, box_pub, marker_pub;
    ros::Publisher laserscan_check_around_pub;

    TNodeCheck *nodecheck;
    string frame_id_dest = "base_link";
    pcl::PointCloud<pcl::PointXYZI> cloud_part[4];

    void run();
    bool CheckRosNode(string name);
    pcl::PointCloud<pcl::PointXYZI> CloudTf(pcl::PointCloud<pcl::PointXYZI> cp_src, string frame_id);

public:
    float box_x_min = 0.05, box_x_max = 10;
    float box_y_min = -6, box_y_max = 6;
    pcl::PointCloud<pcl::PointXYZI> cloud_acc;

    geometry_msgs::Point center_point[4];

    Laserscan_Check()
    {
        nh = new ros::NodeHandle("~");

        laserscan_sub[0] = nh->subscribe<sensor_msgs::LaserScan>("/c200_lidar_new_node1/scan", 10, &Laserscan_Check::LaserScanCallback1, this);
        laserscan_sub[1] = nh->subscribe<sensor_msgs::LaserScan>("/c200_lidar_new_node2/scan", 10, &Laserscan_Check::LaserScanCallback2, this);
        laserscan_sub[2] = nh->subscribe<sensor_msgs::LaserScan>("/c200_lidar_new_node3/scan", 10, &Laserscan_Check::LaserScanCallback3, this);
        laserscan_sub[3] = nh->subscribe<sensor_msgs::LaserScan>("/c200_lidar_new_node4/scan", 10, &Laserscan_Check::LaserScanCallback4, this);
        // remainpath_sub = nh->subscribe<std_msgs::Float64>("/local_path_plan/remainpath", 10, &Laserscan_Check::RemainPathCallback, this);

        cloud_pub[0] = nh->advertise<sensor_msgs::PointCloud2>("cloud1_filter", 10);
        cloud_pub[1] = nh->advertise<sensor_msgs::PointCloud2>("cloud2_filter", 10);
        cloud_pub[2] = nh->advertise<sensor_msgs::PointCloud2>("cloud3_filter", 10);
        cloud_pub[3] = nh->advertise<sensor_msgs::PointCloud2>("cloud4_filter", 10);
        laserscan_check_around_pub = nh->advertise<sensor_msgs::PointCloud2>("cloud_around", 10);

        // wheel_err_pub = nh->advertise<std_msgs::Float32MultiArray>("wheel_err", 10);

        box_pub = nh->advertise<jsk_recognition_msgs::BoundingBoxArray>("box", 10); // 发布聚类边界框
        marker_pub = nh->advertise<visualization_msgs::MarkerArray>("marker", 10);

        nodecheck = new TNodeCheck(nh, "node_rate");
        nodecheck->Find("node_rate")->SetLimit(10);

        start();
    }

    void LaserScanCallback1(const sensor_msgs::LaserScan::ConstPtr &c_msg);
    void LaserScanCallback2(const sensor_msgs::LaserScan::ConstPtr &c_msg);
    void LaserScanCallback3(const sensor_msgs::LaserScan::ConstPtr &c_msg);
    void LaserScanCallback4(const sensor_msgs::LaserScan::ConstPtr &c_msg);
    // void RemainPathCallback(const std_msgs::Float64::ConstPtr &msg);

    pcl::PointCloud<pcl::PointXYZI> Cloud_PassThrough(int id, pcl::PointCloud<pcl::PointXYZI> cp_src);
    void Cluster();
    void CloudAccumulate();
    void CloudProcess(int id, const sensor_msgs::LaserScan::ConstPtr &c_msg);
    // void WheelErrPublish();
};

// void Laserscan_Check::RemainPathCallback(const std_msgs::Float64::ConstPtr &msg)
// {
//     // if(msg->data<6 && msg->data>0.5)
//     // {
//     //     box_y_max=msg->data;  // box_y_min=-box_y_max;
//     // }
// }

// 范围滤波
pcl::PointCloud<pcl::PointXYZI> Laserscan_Check::Cloud_PassThrough(int id, pcl::PointCloud<pcl::PointXYZI> cp_src)
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

    // switch (id)
    // {
    // case 1:
    //     pass.setFilterLimits(-box_y_max, -box_y_min);
    //     break;
    // case 2:
    //     break;
    // case 3:
    //     break;
    // case 4:
    //     break;
    // default:
    //     ROS_ERROR("Wrong ID.\n");
    //     break;
    // }
    // if(id==1) pass.setFilterLimits(-box_y_max, -box_y_min);
    // else pass.setFilterLimits(box_y_min, box_y_max);
    pass.setFilterLimits(box_y_min, box_y_max);
    pass.setFilterLimitsNegative(false);
    pass.filter(res);

    return res;
}

void Laserscan_Check::Cluster()
{
    pcl::PointCloud<pcl::PointXYZI> cp;
    cp.header.frame_id = frame_id_dest;
    for (int i = 0; i < 4; i++)
        if (msg_flag & (1 << i))
            cp += CloudTf(cloud_part[i], frame_id_dest);

    sensor_msgs::PointCloud2 msgx;
    pcl::toROSMsg(cp, msgx);
    msgx.header.frame_id = cp.header.frame_id;
    laserscan_check_around_pub.publish(msgx);

    // jsk_recognition_msgs::BoundingBoxArray boxarray;
    // boxarray.header.frame_id = frame_id_dest;
    // if (cp.size() <= 5)
    //     return boxarray; // 点数过少，直接返回空数据

    // // printf("%d\n", cp.size());

    // // 建立KdTree对象用来搜索
    // pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    // tree->setInputCloud(cp.makeShared());

    // vector<pcl::PointIndices> cluster_indices;
    // pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    // ec.setClusterTolerance(cluster_distance); // 设置聚类最小值，较小的值可能会导致对象被划分为多个簇，而较大的值可能会连接同一簇中的对象
    // ec.setMinClusterSize(cluster_number_min); // 设置聚类的最小点云数
    // ec.setMaxClusterSize(cluster_number_max); // 设置聚类的最大点云数
    // ec.setSearchMethod(tree);
    // ec.setInputCloud(cp.makeShared());
    // ec.extract(cluster_indices);
    // // printf("222222222222222222222222222\n");

    // pcl::PointXYZI minpoint, maxpoint;
    // // ROS_INFO("%d",cluster_indices.size());
    // for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    // {
    //     pcl::PointCloud<pcl::PointXYZI> cloud_cluster;
    //     for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
    //     {
    //         pcl::PointXYZI p = cp[*pit];
    //         cloud_cluster.push_back(p);
    //     }
    //     pcl::getMinMax3D(cloud_cluster, minpoint, maxpoint);

    //     jsk_recognition_msgs::BoundingBox bb;
    //     bb.header.frame_id = frame_id_dest;
    //     // printf("dt=%.7f \n",(t2-t1).toSec());

    //     bb.pose.position.x = (maxpoint.x + minpoint.x) / 2;
    //     bb.pose.position.y = (maxpoint.y + minpoint.y) / 2;
    //     bb.pose.position.z = (maxpoint.z + minpoint.z) / 2;
    //     bb.dimensions.x = maxpoint.x - minpoint.x;
    //     bb.dimensions.y = maxpoint.y - minpoint.y;
    //     bb.dimensions.z = maxpoint.z - minpoint.z;

    //     boxarray.boxes.push_back(bb);

    //     // //if (bb.dimensions.x > 1.5 && bb.dimensions.x < 5 && bb.dimensions.y > 1.5 && bb.dimensions.y < 5 && bb.dimensions.z > 0.5 && bb.dimensions.z < 3)
    //     // if(bb.dimensions.x>=min_length && bb.dimensions.y>=min_width && bb.dimensions.z>=min_height)
    //     // {
    //     //     boxarray.boxes.push_back(bb);

    //     //     sensor_msgs::PointCloud2 msgx;
    //     //     pcl::toROSMsg(cloud_cluster, msgx);
    //     //     msgx.header.frame_id = "rslidar";
    //     //     cloud_pub.publish(msgx);
    //     // }
    // }

    // // printf("33333333333333333\n");

    // visualization_msgs::MarkerArray markerarray;
    // visualization_msgs::Marker marker;
    // markerarray.markers.clear();
    // marker.header.frame_id = frame_id_dest;
    // marker.header.stamp = ros::Time::now();
    // marker.ns = "";
    // marker.lifetime = ros::Duration(0.4);
    // marker.frame_locked = true;
    // marker.type = visualization_msgs::Marker::CUBE;
    // marker.action = visualization_msgs::Marker::ADD;
    // for (int i = 0; i < boxarray.boxes.size(); ++i)
    // {
    //     marker.color.r = 0.0f;
    //     marker.color.g = 0.0f;
    //     marker.color.b = 1.0f;
    //     marker.color.a = 0.5f;

    //     marker.id = i;
    //     marker.pose.position = boxarray.boxes[i].pose.position;
    //     marker.scale = boxarray.boxes[i].dimensions;
    //     markerarray.markers.push_back(marker);
    // }

    // // printf("%d\n",markerarray.markers.size());

    // marker_pub.publish(markerarray);
    // box_pub.publish(boxarray);
    // return boxarray;
}

pcl::PointCloud<pcl::PointXYZI> Laserscan_Check::CloudTf(pcl::PointCloud<pcl::PointXYZI> cp_src, string frame_id)
{
    static tf::TransformListener listener;

    pcl::PointCloud<pcl::PointXYZI> res;
    vector<int> aa;
    pcl::removeNaNFromPointCloud(cp_src, res, aa); // 移除无效点，cp为初始点云，res为移除无效点后的点云

    if (cp_src.header.frame_id != frame_id)
    {
        for (auto &it : res)
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

void Laserscan_Check::CloudProcess(int id, const sensor_msgs::LaserScan::ConstPtr &c_msg) // pcl::PointCloud<pcl::PointXYZI> cloud, geometry_msgs::Point &point)
{
    pcl::PointCloud<pcl::PointXYZI> cloud;
    LaserScan2PointCloud(c_msg, cloud);
    // frame_id_dest=cloud.header.frame_id;
    // cloud = Cloud_PassThrough(id, cloud);
    // cloud = CloudTf(cloud, frame_id_dest);
    cloud_part[id] = cloud;

    msg_flag |= (1 << id);

    // jsk_recognition_msgs::BoundingBoxArray boxarray = Cluster(cloud);

    // printf("%d=%d\n", id, cloud.size());

    sensor_msgs::PointCloud2 msgx;
    pcl::toROSMsg(cloud, msgx);
    msgx.header.frame_id = cloud.header.frame_id;
    // if (id == 3) printf("%ld\n", msgx.data.size());
    cloud_pub[id].publish(msgx);

    // pcl::PointXYZI minpoint, maxpoint;
    // pcl::getMinMax3D(cloud, minpoint, maxpoint);
    // center_point[id].x = -1;
    // center_point[id].y = -1;   //  not reach
    // center_point[id].z = 0;
    // if(cloud.points.size()>1)
    // {
    //     center_point[id].x = minpoint.x;  //+maxpoint.x)*0.5;
    //     center_point[id].y = (minpoint.y+maxpoint.y)*0.5;
    //     center_point[id].z = maxpoint.y-minpoint.y;
    //     if(id==3 || id==0)  center_point[id].y*=-1;

    //     // printf("id=%d x=%.3f\n", id, minpoint.x);
    // }
}

// void Laserscan_Check::WheelErrPublish()
// {
//     // float dx=999;
//     // if(center_point[1].x>0 && center_point[3].x>0)  dx=center_point[1].x-center_point[3].x; //printf("dx1=%.3f %.3f %.3f\n", center_point[1].x-center_point[3].x, center_point[1].x, center_point[3].x);

//     // // printf("dx=%.3f\n", dx);

//     // std_msgs::Float32MultiArray err;
//     // err.data.push_back(dx);
//     // wheel_err_pub.publish(err);

//     // nodecheck->Find("node_rate")->Beat();
// }

void Laserscan_Check::LaserScanCallback1(const sensor_msgs::LaserScan::ConstPtr &c_msg)
{
    CloudProcess(0, c_msg);
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

bool Laserscan_Check::CheckRosNode(string name)
{
    string msg = "";
    nh->getParam(name + "/check/msg", msg);

    if (msg.find("OK") == msg.npos)
        return true;
    else
        return false;
}

void Laserscan_Check::run()
{
    ros::Rate looprate(20);
    while (ros::ok())
    {
        // lk.WheelErrPublish();
        int lidar_working_flag = 0;
        for (int id = 0; id < 4; id++)
        {
            string name = "c200_lidar_new_node";
            name = name + to_string(id + 1);
            if (CheckRosNode(name))
                lidar_working_flag |= (1 << id);
        }
        // printf("%d\n", msg_flag);
        if ((msg_flag & lidar_working_flag) == lidar_working_flag)
        {
            Cluster();
            msg_flag = 0;
            // cloud.clear();
        }
        // ros::spinOnce();
        looprate.sleep();
    }
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laserscan_check");

    Laserscan_Check lk;
    ros::Rate looprate(20);

    while (ros::ok())
    {
        ros::spinOnce();
        looprate.sleep();
    }
    return 0;
}
