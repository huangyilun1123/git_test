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
#include "pointcloud_type.h"
#include <common/mydisplay.h>
#include <common/public.h>
#include <vector>
#include <algorithm>

using namespace std;

ros::Subscriber cloud_sub, keyboard_sub;
ros::Publisher cloud_pub;
string frame_id_dest;

TNodeCheck *nodecheck;

//坐标转换
PointCloud CloudTf(PointCloud cp_src, string frame_id)
{
    static tf::TransformListener listener;

    PointCloud res;
    vector<int> aa;
    pcl::removeNaNFromPointCloud(cp_src, res, aa);//移除无效点，cp为初始点云，res为移除无效点后的点云
    
    if(res.header.frame_id!=frame_id)
    {
        //listener.waitForTransform(frame_id, res.header.frame_id, ros::Time::now(), ros::Duration(1.0));//等待1.0s判断有是否可用的transform
        // PointCloud r=res;
        for (auto it = res.begin(); it < res.end(); ++it)
        {
            geometry_msgs::PointStamped src_p, dst_p;
            src_p.header.frame_id=res.header.frame_id;
            src_p.header.stamp = ros::Time();

            src_p.point.x = it->x;
            src_p.point.y = it->y;
            src_p.point.z = it->z;
            transformPoint(frame_id, src_p, dst_p, "");
            it->x = dst_p.point.x;
            it->y = dst_p.point.y;
            it->z = dst_p.point.z;
        }
        // pcl_ros::transformPointCloud(frame_id, r, res, listener);//第一个参数为目标坐标系。第二个参数为原始点云，第三个参数为目标点云，第四个参数为接受到的坐标
        res.header.frame_id = frame_id;
    }
    return res;
}

void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& c_msg)
{
    nodecheck->Find("node_rate")->Beat();
    if(frame_id_dest=="") return;
    
    PointCloud cloud0, cloud1;
    pcl::fromROSMsg(*c_msg, cloud0);

    cloud1=CloudTf(cloud0,frame_id_dest);

    sensor_msgs::PointCloud2 msgx;
    pcl::toROSMsg(cloud1, msgx);
    msgx.header.frame_id = cloud1.header.frame_id;
    cloud_pub.publish(msgx);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cloud_tf");
    ros::NodeHandle nh("~");

    nodecheck=new TNodeCheck(&nh, "node_rate");
	nodecheck->Find("node_rate")->SetLimit(10);

    sleep(1);
    string input_topic, output_topic;
    nh.getParam("input_topic",input_topic);
    nh.getParam("output_topic",output_topic);
    
    cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(input_topic, 10, &PointCloudCallback);//订阅激光雷达原始点云
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(output_topic,10);

    ros::Rate looprate(50);
    while(ros::ok())
    {
        nh.getParam("frame_id_dest", frame_id_dest);
        ros::spinOnce();
        looprate.sleep();
    }
    return 0;
};
