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
#include <std_msgs/UInt16.h>
#include <pcl/io/pcd_io.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <common/public.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include "cloud_preprocess.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::NodeHandle nh("~");
    ROS_INFO("PCD start");

    // keyboard_sub = nh.subscribe<std_msgs::UInt16>("/keyboard", 10, &KeyboardCallback);
    // cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(input_topic, 10, &PointCloudCallback);//订阅激光雷达原始点云
    ros::Publisher cloud_pub1 = nh.advertise<sensor_msgs::PointCloud2>("pcd_cloud1", 10);
    ros::Publisher cloud_pub2 = nh.advertise<sensor_msgs::PointCloud2>("pcd_cloud2", 10);

    PointCloud pcd_data1, pcd_data2;
    pcl::io::loadPCDFile("/home/wsk/ROS_Programing/pcd/k1.pcd", pcd_data1);
    pcl::io::loadPCDFile("/home/wsk/ROS_Programing/pcd/k2.pcd", pcd_data2);

    TTimer tmr;
    pcd_data1.header.frame_id="base_link";
    pcd_data1 = Cloud_RadiusFilter(pcd_data1, 0.1, 10);
    pcd_data2.header.frame_id="base_link";
    pcd_data2 = Cloud_RadiusFilter(pcd_data2, 0.1, 10);

    // pcl::MomentOfInertiaEstimation <pcl::PointXYZI> feature_extractor;
    // feature_extractor.setInputCloud(pcd_data2.makeShared());
    // feature_extractor.compute();

    // printf("AAAAAAAAAAAAAAAA\n");
    // pcl::PointXYZI min_point_OBB, max_point_OBB, position_OBB;
    // Eigen::Matrix3f rotational_matrix_OBB;
    // feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

    // geometry_msgs::Point scale;
    // scale.x=scale.y=scale.z=0.05;
    // pcd_data1 = Cloud_DownSampling(pcd_data1, scale);
    // pcd_data2 = Cloud_DownSampling(pcd_data2, scale);

    // float angle1 = GetYawFromPose(mbb1.pose);
    // float angle2 = GetYawFromPose(mbb2.pose);
    // ROS_INFO("time=%.3f angle1=%.2f  angle2=%.2f, scale=%.3f %.3f\n", tmr.GetValue(), angle1 * 180 / M_PI, angle2 * 180 / M_PI, mbb1.scale.y, mbb2.scale.y);

    sensor_msgs::PointCloud2 msg1, msg2;
    pcl::toROSMsg(pcd_data1, msg1);
    msg1.header.frame_id = pcd_data1.header.frame_id;
    pcl::toROSMsg(pcd_data2, msg2);
    msg2.header.frame_id = pcd_data1.header.frame_id;

    TMinBoundingBox mbb1("b1"), mbb2("b2");
    mbb1.Calculate(pcd_data1, 0.1, -10, 10);
    mbb2.Calculate(pcd_data2, 0.1, -10, 10);

    ros::Rate looprate(20);
    while (1)
    {
        cloud_pub1.publish(msg1);
        cloud_pub2.publish(msg2);

        mbb1.Publish();
        mbb2.Publish();

        ros::spinOnce();
        looprate.sleep();
    }
    return 0;
};