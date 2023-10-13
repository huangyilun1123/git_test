#define PCL_NO_PRECOMPILE

#include "cloud_preprocess.h"


#include <ros/ros.h>
#include <data_comm/car_ctr.h>
#include <visualization_msgs/Marker.h>
#include <common/public.h>

const float filter_region_max_x = 3.2;
const float filter_region_min_x = -3.2;

const float filter_region_max_y = 1.7;
const float filter_region_min_y = -1.7;


struct FilterRegion
{
    float min_x, max_x, min_y, max_y;
};

class LaserRadarObs
{
    private:
        ros::NodeHandle *nh;

        ros::Subscriber laser_point_cloud_sub, car_cmd_sub, local_path_sub;
        ros::Publisher filter_region_pub, filter_cloud_pub;

        FilterRegion filter_region, car_region;//外围滤波框filter_region，内侧车辆框car_region
        bool filter_region_is_circle = false; //滤波范围为矩形还是圆形，false矩形，true圆形

        float obs_dis, speedlimit;//障碍距离obs_dis，最大速度speedlimit
        geometry_msgs::PoseStamped loacl_path_final_point;
        float aim_angle = 0;

        void LaserscanPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &c_msg); //线激光雷达点云回调，一体的baselink坐标点云

        void CarCmdCallBack(const data_comm::car_ctr::ConstPtr &c_msg);//车辆控制指令回调，用以更新避障区域
        void LocalPathCallBack(const nav_msgs::Path::ConstPtr &c_msg);


        PointCloud Cloud_Pass_Through(const PointCloud &cp_src,float x_min, float x_max, float y_min, float y_max, float z_min, float z_max);//矩形滤波滤外侧
        PointCloud Cloud_Pass_Through_Inside(const PointCloud &cp_src,float x_min, float x_max, float y_min, float y_max, float z_min, float z_max);//矩形滤波滤内侧
        PointCloud Cloud_Pass_Through_Redius(const PointCloud &cp_src, float max_redius);//距离滤波
        vector<pcl::PointIndices> Clould_Cluster(const PointCloud &cp_src, float cluster_distance, int cluster_number_min, int cluster_number_max);//激光雷达聚类
        float GetDis2CarRegion(const PointType &point);//点与内侧车辆框car_region距离
        float GetDis2FilterRegion(const PointType &point);//点与外围滤波框filter_region距离
        void PublishRegion(FilterRegion region);//用以发布外围滤波框filter_region，内侧车辆框car_region
    public:
        LaserRadarObs();//构造
};


LaserRadarObs::LaserRadarObs()
{
    nh = new ros::NodeHandle("~");

    laser_point_cloud_sub = nh->subscribe<sensor_msgs::PointCloud2>("/cloud_filter/filter_leida3", 10, &LaserRadarObs::LaserscanPointCloudCallback, this);
    car_cmd_sub = nh->subscribe<data_comm::car_ctr>("/pathtrack/ctr_cmd", 10, &LaserRadarObs::CarCmdCallBack, this);
    local_path_sub = nh->subscribe<nav_msgs::Path>("/local_path_plan/localpath", 10,&LaserRadarObs::LocalPathCallBack, this);

    filter_region_pub = nh->advertise<visualization_msgs::Marker>("obs_filter_region", 10);
    filter_cloud_pub = nh->advertise<sensor_msgs::PointCloud2>("filter_around_cloud", 10);

    car_region.max_x = 3; car_region.min_x = -3;
    car_region.max_y = 1.5; car_region.min_y = -1.5;//车辆长宽

    filter_region.max_x = filter_region_max_x; filter_region.min_x = filter_region_min_x; 
    filter_region.max_y = filter_region_max_y; filter_region.min_y = filter_region_min_y; //静止时滤波范围
}

PointCloud LaserRadarObs::Cloud_Pass_Through(const PointCloud &cp_src, float x_min, float x_max, float y_min, float y_max, float z_min = -5, float z_max = 5)
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
}

PointCloud LaserRadarObs::Cloud_Pass_Through_Inside(const PointCloud &cp_src,float x_min, float x_max, float y_min, float y_max, float z_min = -5, float z_max = 5)
{
    PointCloud res;
    res.header = cp_src.header;
    for (auto point : cp_src.points)
        if (!(x_max > point.x && point.x > x_min && y_max > point.y && point.y > y_min))
            res.points.push_back(point);
    res.width = res.points.size();
    res.height = 1;
    return res;
}

PointCloud LaserRadarObs::Cloud_Pass_Through_Redius(const PointCloud &cp_src, float max_redius)
{
    PointCloud res;
    res.header = cp_src.header;
    for (auto point : cp_src.points)
        if (point.x * point.x + point.y * point.y < max_redius * max_redius)
            res.points.push_back(point);
    res.width = res.points.size();
    res.height = 1;
    return res;
}

vector<pcl::PointIndices> LaserRadarObs::Clould_Cluster(const PointCloud &cp_src, float cluster_distance, int cluster_number_min, int cluster_number_max)
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

float LaserRadarObs::GetDis2CarRegion(const PointType &point)
{
    float cur_point_x = point.x, cur_point_y = point.y;
    float filter_point_x, filter_point_y;
    if (cur_point_x < car_region.min_x)
        filter_point_x = car_region.min_x;
    else if (cur_point_x > car_region.max_x)
        filter_point_x = car_region.max_x;
    else filter_point_x = cur_point_x;

    if (cur_point_y < car_region.min_y)
        filter_point_y = car_region.min_y;
    else if (cur_point_y > car_region.max_y)
        filter_point_y = car_region.max_y;
    else filter_point_y = cur_point_y;

    return P2P(filter_point_x, filter_point_y, cur_point_x, cur_point_y);
}

float LaserRadarObs::GetDis2FilterRegion(const PointType &point)
{
    float cur_x = point.x, cur_y = point.y;
    return min(min(min(fabs(cur_x - filter_region.min_x), fabs(cur_x - filter_region.max_x)), fabs(cur_y - filter_region.min_y)), fabs(cur_y - filter_region_max_y));
}

void LaserRadarObs::LaserscanPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &c_msg)
{
    PointCloud cloud;
    pcl::fromROSMsg(*c_msg, cloud);

    // cloud = Cloud_Pass_Through_Inside(cloud, car_region.min_x, car_region.max_x, car_region.min_y, car_region.max_y);
    // if (filter_region_is_circle) cloud = Cloud_Pass_Through_Redius(cloud, (filter_region.max_x - filter_region_min_x) / 2);
    // else cloud = Cloud_Pass_Through(cloud, filter_region.min_x, filter_region.max_x, filter_region.min_y, filter_region.max_y);

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    // filter_cloud_pub.publish(msg);
    
    obs_dis = 999;
    if (cloud.size())
    {
        vector<pcl::PointIndices> cluster_indices = Clould_Cluster(cloud, 0.15, 5, 10000);
        for (auto indices : cluster_indices)
            for (auto pid : indices.indices)
            {
                PointType point = cloud[pid];
                obs_dis = min(obs_dis, GetDis2CarRegion(point));
            }
    }
    printf("obs_dis=%.2f\n",obs_dis);
    // nh->setParam("obstacle_dis", obs_dis);
    // PublishRegion(filter_region);
    // PublishRegion(car_region);
}

void LaserRadarObs::CarCmdCallBack(const data_comm::car_ctr::ConstPtr &c_msg)
{
    if (c_msg -> turnmode == 2)
    {
        filter_region.max_x = 3.6; filter_region.min_x = -3.6; 
        filter_region.max_y = 3.6; filter_region.min_y = -3.6; 
        filter_region_is_circle = true;
    }
    else
    {
        filter_region.max_x = filter_region_max_x; filter_region.min_x = filter_region_min_x; 
        filter_region.max_y = filter_region_max_y; filter_region.min_y = filter_region_min_y; 
        const float path_safe_dis = 0.5; //路径安全距离
        if (fabs(aim_angle) < 20) filter_region.max_x += min(7., loacl_path_final_point.pose.position.x + path_safe_dis);
        else if (fabs(aim_angle-180)<20 || fabs(aim_angle+180)<20) filter_region.min_x -= min(7., -loacl_path_final_point.pose.position.x + path_safe_dis);
        else if(fabs(aim_angle-90)<20)  filter_region.max_y += min(7., loacl_path_final_point.pose.position.y + path_safe_dis);
        else if (fabs(aim_angle+90)<20) filter_region.min_y -= min(7., -loacl_path_final_point.pose.position.y + path_safe_dis);
        filter_region_is_circle = false;
    }

    // if (c_msg -> turnmode == 0) //阿克曼
    // {
    //     if (c_msg -> speed >= 0)
    //         filter_region.max_x += 7;
    //     else
    //         filter_region.min_x -= 7;
    // }
    // else if (c_msg -> turnmode == 2)//自转
    // {
    //     filter_region.max_y += 4;
    //     filter_region.min_y -= 4;
    // }
    // else if (c_msg -> turnmode == 3)//横移
    // {
    //     if (c_msg -> speed >= 0)
    //         filter_region.max_y += 7;
    //     else
    //         filter_region.min_y -= 7;
    // }
}

void LaserRadarObs::LocalPathCallBack(const nav_msgs::Path::ConstPtr &c_msg)
{
    if ((*c_msg).poses.size() == 0)return;
    transformPose("base_link", (*c_msg).poses.back(), loacl_path_final_point);
    aim_angle=atan2(loacl_path_final_point.pose.position.y, loacl_path_final_point.pose.position.x) * 180 / M_PI;
}

void LaserRadarObs::PublishRegion(FilterRegion region)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "lidar_region_marker";
    marker.lifetime = ros::Duration(0.2);
    marker.frame_locked = true;
    if (filter_region_is_circle) marker.type = visualization_msgs::Marker::SPHERE;
    else marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r = marker.color.g = marker.color.b = 1.0f;
    marker.color.a = 0.1f;
    marker.scale.z = 0.1;
    marker.pose.orientation.w = 1;

    marker.scale.x = region.max_x - region.min_x;
    marker.scale.y = region.max_y - region.min_y;
    marker.pose.position.x = (region.max_x + region.min_x) / 2;
    marker.pose.position.y = (region.max_y + region.min_y) / 2;
    // ROS_INFO("%f %f\n", region.max_x, region.min_x);
    marker.id = 0;
    filter_region_pub.publish(marker);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "check_obstacle2");
    LaserRadarObs lro;
    ros::spin();
}