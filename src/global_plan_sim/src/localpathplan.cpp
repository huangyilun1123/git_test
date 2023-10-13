#include <ros/ros.h>
#include <fstream>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <common/public.h>
#include <common/mydisplay.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <mqtt_comm/task.h>
#include <mqtt_comm/path_point.h>
#include <std_msgs/Float32MultiArray.h>

using namespace std;

const float last_path_precision = 0.02;

class TLocalPathPlan
{
private:
    ros::NodeHandle *nh_local, *nh;
    ros::Publisher taskpath_pub, globalpath_pub, localpath_pub, shotpose_pub, pathmarkers_pub, remainpath_pub, passedpath_pub;
    ros::Subscriber task_sub, lidar_target_sub, laserscan_target_sub, rviz_goal_sub, scanlidar_pos_sub;
    nav_msgs::Path task_path;   //  稀疏任务点
    nav_msgs::Path global_path; //  加密后的全局点(带速度规划,速度为正), 最后一段最密
    nav_msgs::Path local_path;  //  从global_path中抽取的局部路径
    bool is_lidar_global_path = false; //标记全局路径为多线机关雷达提供，防止错误的对passedpathlen进行更新
    geometry_msgs::PoseStamped shotpose;
    double utm_x_zero = 0, utm_y_zero = 0;

    int Nearest_ID = 0;
    mqtt_comm::task cur_task;
    TNodeCheck *nodecheck;
    float path_remain;
    float path_passed = 0;
    float path_passed_when_lidar_give_path = 0; //记录多线激光发布路径时的pathpassed，对passedpath提供持续更新


    TDataFilter center1_y, center3_y, center2_y, center4_y, center1_x, center3_x, center2_x, center4_x, len1, len2, len3, len4;

    tf::TransformListener listener;

public:
    TLocalPathPlan() : center1_x(2), center3_x(2), center2_x(2), center4_x(2), center1_y(2), center3_y(2), center2_y(2), center4_y(2), len1(2), len2(2), len3(2), len4(2)
    {
        nh_local = new ros::NodeHandle("~");
        nh = new ros::NodeHandle();
        localpath_pub = nh_local->advertise<nav_msgs::Path>("localpath", 10);
        globalpath_pub = nh_local->advertise<nav_msgs::Path>("globalpath", 10);
        taskpath_pub = nh_local->advertise<nav_msgs::Path>("taskpath", 10);
        shotpose_pub = nh_local->advertise<geometry_msgs::PoseStamped>("target_pose", 10);
        pathmarkers_pub = nh_local->advertise<visualization_msgs::MarkerArray>("path_markers", 10);
        remainpath_pub = nh_local->advertise<std_msgs::Float64>("remainpath", 10);
        passedpath_pub = nh_local->advertise<std_msgs::Float64>("passedpath", 10);

        task_sub = nh->subscribe<mqtt_comm::task>("/task_cmd", 10, &TLocalPathPlan::TaskCallback, this);
        lidar_target_sub = nh->subscribe<visualization_msgs::MarkerArray>("/cloud_location_pro/lidar_target", 10, &TLocalPathPlan::LidarTargetCallback, this);
        laserscan_target_sub = nh->subscribe<visualization_msgs::MarkerArray>("/cloud_location_pro/laserscan_target", 10, &TLocalPathPlan::LaserscanTargetCallback, this);
        rviz_goal_sub = nh->subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, &TLocalPathPlan::RvizGoalCallback, this);

        scanlidar_pos_sub = nh->subscribe<std_msgs::Float32MultiArray>("/laserscan_check/wheel_distance", 10, &TLocalPathPlan::ScanPosCallback, this);

        task_path.header.frame_id = "map";
        global_path.header.frame_id = "map";
        local_path.header.frame_id = "map";

        cur_task.cmd = "none task";
        cur_task.stamp = ros::Time::now();

        nodecheck = new TNodeCheck(nh_local, "node_rate");
        nodecheck->Find("node_rate")->SetLimit(15);

        // start();
    };

    void ScanPosCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);

    void TaskCallback(const mqtt_comm::task::ConstPtr &msg);
    void LidarTargetCallback(const visualization_msgs::MarkerArray::ConstPtr &msg);
    void LaserscanTargetCallback(const visualization_msgs::MarkerArray::ConstPtr &msg);
    void RvizGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    void GetGlobalPathFromTask(mqtt_comm::task task);
    void GlobalPathPlan();
    void LocalPathPlan();
    nav_msgs::Path LineInterpolation(geometry_msgs::Point p1, geometry_msgs::Point p2, float ds);
    void SpeedPlan(nav_msgs::Path &path, float min_vel, float safe_dis = 0.3);
    void SpeedPlan4Stop(nav_msgs::Path &path, float safe_dis);
    bool CheckStopAtNextPoint(nav_msgs::Path path, int id);
    void Run();
    // void run();

    nav_msgs::Path GenFinalPathByPose(geometry_msgs::PoseStamped pose);
    void BrTargetLink();
    int CheckMoveMode(nav_msgs::Path path, int id);
    void PubPathMarkers();
    bool CheckTaskFinished();
};

void TLocalPathPlan::ScanPosCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    center1_x.GetValue(msg->data[0]), center1_y.GetValue(msg->data[1]), len1.GetValue(msg->data[2]);
    center2_x.GetValue(msg->data[3]), center2_y.GetValue(msg->data[4]), len2.GetValue(msg->data[5]);
    center3_x.GetValue(msg->data[6]), center3_y.GetValue(msg->data[7]), len3.GetValue(msg->data[8]);
    center4_x.GetValue(msg->data[9]), center4_y.GetValue(msg->data[10]), len4.GetValue(msg->data[11]);
    return;
}

//  发布路径速度与方向标识 task_path
void TLocalPathPlan::PubPathMarkers()
{
    if (task_path.poses.size() == 0)
        return;

    visualization_msgs::MarkerArray markerarray;
    visualization_msgs::Marker marker;
    marker.header = task_path.header;
    marker.ns = "path_marker";
    marker.lifetime = ros::Duration(0.2);
    marker.frame_locked = true;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;
    marker.pose.position.z = 2;
    marker.scale.z = 1;

    marker.id = 0;
    for (int i = 0; i < task_path.poses.size(); i++)
    {
        marker.id++;
        marker.pose = GetExtendPoseByPose(task_path.poses[i], 1.5).pose;
        char text[100];
        if (i < task_path.poses.size() - 1)
            sprintf(text, "%.1f", task_path.poses[i].pose.position.z);
        else
            sprintf(text, "target");
        marker.text = text;
        markerarray.markers.push_back(marker);
    }

    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = 1.5;
    marker.scale.y = 0.5;
    // marker.color.r=marker.color.g=1.0f;  marker.color.b=0.0f;
    for (int i = 0; i < task_path.poses.size() - 1; i++)
    {
        marker.id++;
        geometry_msgs::PoseStamped p1 = task_path.poses[i];
        geometry_msgs::PoseStamped p2 = task_path.poses[i + 1];
        marker.pose.position.x = (p1.pose.position.x + p2.pose.position.x) * 0.5;
        marker.pose.position.y = (p1.pose.position.y + p2.pose.position.y) * 0.5;

        marker.pose.position.z = 1;
        marker.pose.orientation = GetQuaternionMsgByPoints(p1.pose.position, p2.pose.position);
        markerarray.markers.push_back(marker);
    }

    pathmarkers_pub.publish(markerarray);
}

//  在rviz里模拟目标点
void TLocalPathPlan::RvizGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    mqtt_comm::task task;
    mqtt_comm::path_point p;
    task.cmd = "move task";
    task.stamp = ros::Time::now();

    geometry_msgs::PoseStamped p0_base, p0_map;
    p0_base.header.frame_id = "base_link";
    p0_base.pose.orientation.w = 1;
    transformPose("map", p0_base, p0_map);
    p.pointHA = GetYawFromPose(p0_map) * 180 / M_PI;
    p.pointX = p0_map.pose.position.x;
    p.pointY = p0_map.pose.position.y;
    p.vehSpeed = 0.4;
    task.path.push_back(p);

    p.pointHA = GetYawFromPose(*msg) * 180 / M_PI;
    p.pointX = msg->pose.position.x;
    p.pointY = msg->pose.position.y;
    p.vehSpeed = 1;
    task.path.push_back(p);

    GetGlobalPathFromTask(task);
    GlobalPathPlan();
}

//  根据task生成全局路径 task_path 重要函数
void TLocalPathPlan::GetGlobalPathFromTask(mqtt_comm::task task)
{
    Nearest_ID = 0;

    is_lidar_global_path = false;

    task_path.header.frame_id = "map";
    task_path.header.stamp = ros::Time::now();
    task_path.poses.clear();

    shotpose.header.frame_id = "", global_path.poses.clear();

    // cout << task.path.size() << endl;

    if (task.path.empty())
        return;

    shotpose.header.frame_id = "map";
    shotpose.header.stamp = ros::Time::now();
    shotpose.pose.position.x = task.path.back().pointX;
    shotpose.pose.position.y = task.path.back().pointY;
    shotpose.pose.position.z = task.path.back().vehSpeed;
    shotpose.pose.orientation = tf::createQuaternionMsgFromYaw(task.path.back().pointHA * M_PI / 180);
    shotpose_pub.publish(shotpose); //  发布出去

    geometry_msgs::PoseStamped p_map;
    p_map.header.frame_id = "map";
    p_map.header.stamp = ros::Time::now();
    for (int i = 0; i < task.path.size(); i++) //  加入设置路径点
    {
        p_map.pose.position.x = task.path[i].pointX;
        p_map.pose.position.y = task.path[i].pointY;
        p_map.pose.position.z = task.path[i].vehSpeed;

        // cout << task.path[i].vehSpeed << endl;

        p_map.pose.orientation = tf::createQuaternionMsgFromYaw(task.path[i].pointHA * M_PI / 180);
        task_path.poses.push_back(p_map);
    }

    bool leading_enable = true;
    float leading_length = 6; // //  引导距离
    // nh_local->getParam("leading_enable",leading_enable);
    // nh_local->getParam("leading_length", leading_length);
    float heading = GetYawFromPose(task_path.poses.back());
    float line_angle = GetAngleByPoints((task_path.poses.end() - 2)->pose.position, (task_path.poses.end() - 1)->pose.position);

    bool if_go_ahead = fabs(heading - line_angle) * 180 / M_PI < 90;

    if (leading_enable && (task.cmd == "pick task" || task.cmd == "release task")) // 加入了引导点, 引导距离内千万不要放置点!!
    {
        if (if_go_ahead)
            p_map = GetExtendPoseByPose(task_path.poses.back(), -leading_length); //  在最后点前插入引导点
        else 
            p_map = GetExtendPoseByPose(task_path.poses.back(), leading_length);
        p_map.pose.position.z = 0.2;                                          //  引导速度

        bool add_flag = true; //  判断距离倒数第二点距离,太小就不加了
        if (task_path.poses.size() > 1)
        {
            float ds = GetDistanceXY((task_path.poses.end() - 1)->pose.position, (task_path.poses.end() - 2)->pose.position);
            if (ds < 8)
                add_flag = false; //  距离小 就不添加
        }

        if (add_flag)
            task_path.poses.insert(task_path.poses.end() - 1, p_map);
    }

    if (task.cmd == "pick task") //  取车路径终点延长xm,靠轮胎检测停车
    {
        float L = 0.3;
        if (!if_go_ahead)
            L = -L; //  判断是否倒车

        p_map = GetExtendPoseByPose(task_path.poses.back(), L);
        *(task_path.poses.end() - 1) = p_map;
    }
}

//  接收任务指令
void TLocalPathPlan::TaskCallback(const mqtt_comm::task::ConstPtr &msg)
{
    // printf("11111111111111111\n");
    if (msg->cmd.find("task") != string::npos)
    {
        cur_task = *msg;
        cur_task.stamp = ros::Time::now();

        GetGlobalPathFromTask(*msg);
        GlobalPathPlan();
        // ROS_ERROR("%s\n", msg->cmd.c_str());
    }
}

//  根据单个目标点pose生成最后路径,仅在近距离有效
nav_msgs::Path TLocalPathPlan::GenFinalPathByPose(geometry_msgs::PoseStamped pose)
{
    is_lidar_global_path = true;
    path_passed_when_lidar_give_path = path_passed;

    geometry_msgs::PoseStamped pose_map;
    // printf("%f", pose.pose.position.z);
    transformPose("map", pose, pose_map, "");

    geometry_msgs::PoseStamped p0_map, p1_map, p2_map, p0_base;
    p0_base.header.frame_id = "base_link";
    p0_base.header.stamp = pose.header.stamp;
    p0_base.pose.orientation.w = 1;
    transformPose("map", p0_base, p0_map);
    // printf("%.2f %.2f\n", p0_map.pose.position.x, p0_map.pose.position.y);

    float L1 = 15, L2 = 1;
    p2_map = GetExtendPoseByPose(pose_map, L2);
    p1_map = GetExtendPoseByPose(pose_map, -L1);
    p0_map.pose.orientation = p2_map.pose.orientation;

    nav_msgs::Path path0, path1;
    path1 = LineInterpolation(p1_map.pose.position, p2_map.pose.position, last_path_precision);

    int id = 0;
    FindNearestPointInPath(path1, id, L1);
    p1_map = path1.poses[id]; //  路径上最近点

    float min_dis = 4;
    if (GetDistanceXY(p1_map.pose.position, p2_map.pose.position) < min_dis)
    {
        p1_map = GetExtendPoseByPose(p2_map, -min_dis);
    }

    path1 = LineInterpolation(p1_map.pose.position, p2_map.pose.position, last_path_precision);
    path0 = LineInterpolation(p0_map.pose.position, p1_map.pose.position, last_path_precision);
    SpeedPlan(path1, 0.1, 0);

    task_path.poses.clear();
    task_path.poses.push_back(p0_map);
    task_path.poses.push_back(p1_map);
    task_path.poses.push_back(p2_map);

    // for(auto pose:task_path.poses)
    //     cout<<pose.pose.position.z<<" ";

    //  路径过短就不要了
    // printf("%.2f\n", GetPathLength(path0));
    if (GetPathLength(path0) < 0.2)
        path0.poses.clear(), task_path.poses.erase(task_path.poses.begin());

    Nearest_ID = 0;

    for (auto &p : path0.poses)
        p.pose.position.z = 0.1, p.pose.orientation = p0_map.pose.orientation;
    for (auto p : path1.poses)
    {
        p.pose.orientation = p1_map.pose.orientation;
        path0.poses.push_back(p);
    }

    return path0;
}

//  接收激光点云定位数据
void TLocalPathPlan::LidarTargetCallback(const visualization_msgs::MarkerArray::ConstPtr &msg)
{
    bool enable = false;
    nh_local->getParam("lidar_location_enable", enable);
    if (!enable)
        return;

    geometry_msgs::PoseStamped target;
    target.pose = msg->markers.front().pose;
    target.header = msg->markers.front().header;
    // printf("target.pose:%f,  %s\n", target.pose.position.z, target.header.frame_id.c_str());


    // transformPose("map", target, shotpose,"CCC");

    string *str = new string;
    try
    {
        target.header.stamp = ros::Time(0);
        listener.waitForTransform("map", target.header.frame_id, target.header.stamp, ros::Duration(2), ros::Duration(0.01), str);
        listener.transformPose("map", target, shotpose);
    }
    catch (exception e)
    {
        ROS_INFO("tag=%s %s", e.what(), str->c_str());
    }
    // printf("%d\n",);
    // printf("shotpose.pose:%f\n", shotpose.pose.position.z);
    shotpose.pose.position.z = 0.2; //接车速度

    global_path = GenFinalPathByPose(shotpose);


    Nearest_ID = 0;
}

void TLocalPathPlan::LaserscanTargetCallback(const visualization_msgs::MarkerArray::ConstPtr &msg)
{
    bool enable = false;
    nh_local->getParam("laserscan_location_enable", enable);
    if (!enable)
        return;

    geometry_msgs::PoseStamped target;
    target.pose = msg->markers.front().pose;
    target.header = msg->markers.front().header;


    transformPose("map", target, shotpose);

    shotpose.pose.position.z = 0.2; //接车速度

    global_path = GenFinalPathByPose(shotpose);
    Nearest_ID = 0;
}

//  两点之间插值生成路径
nav_msgs::Path TLocalPathPlan::LineInterpolation(geometry_msgs::Point p1, geometry_msgs::Point p2, float ds)
{
    //  曲线插值加密, ds为间隔距离, 最后一段特别密
    float dd = GetDistanceXY(p1, p2);
    float vel = p1.z;

    // cout << p1.z << endl;

    int n = dd / ds;
    if (n < 1)
        n = 1;

    float dx = (p2.x - p1.x) / n, dy = (p2.y - p1.y) / n;
    geometry_msgs::PoseStamped p;
    float angle = atan2(p2.y - p1.y, p2.x - p1.x);
    p.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
    p.header = global_path.header;
    p.pose.position = p1;
    // printf("i=%d %.2f\n", i, angle*180/M_PI);

    nav_msgs::Path path;
    path.header = global_path.header;
    for (int i = 0; i < n - 1; i++)
    {
        p.pose.position.x = p1.x + dx * i;
        p.pose.position.y = p1.y + dy * i;
        p.pose.position.z = vel; //  z 表示运动速度
        path.poses.push_back(p);
    }

    n = ds / last_path_precision;
    // if(n<1) n=1;
    if (n > 1)
    {
        p1 = p.pose.position;
        dx = (p2.x - p1.x) / n, dy = (p2.y - p1.y) / n;
        for (int i = 1; i < n; i++)
        {
            p.pose.position.x = p1.x + dx * i;
            p.pose.position.y = p1.y + dy * i;
            p.pose.position.z = vel; //  z 表示运动速度
            path.poses.push_back(p);
        }
    }

    return path;
}

// 根据车头航向角与路径方向判断横移动作
int TLocalPathPlan::CheckMoveMode(nav_msgs::Path path, int id)
{
    int res = 0;

    if (path.poses.size()) //  根据预瞄点判断是否横移
    {
        geometry_msgs::PoseStamped p1 = path.poses[id];
        geometry_msgs::PoseStamped p2 = path.poses[id + 1];
        float pose_angle = GetYawFromPose(p1);
        float path_angle = GetAngleByPoints(p1.pose.position, p2.pose.position);
        float anglex = (path_angle - pose_angle) * 180 / M_PI;

        // printf("%.2f %.2f %.2f\n", pose_angle * 180 / M_PI, path_angle * 180 / M_PI,  anglex);
        if (anglex > 180)
            anglex -= 360;
        else if (anglex < -180)
            anglex += 360;

        if (fabs(anglex + 90) < 20)
            res = -3; //  right move
        else if (fabs(anglex - 90) < 20)
            res = 3; // left move
    }

    return res;
}

//  检测是否停止
bool TLocalPathPlan::CheckStopAtNextPoint(nav_msgs::Path path, int id)
{
    bool res = false;
    if (id >= path.poses.size() - 2)
        return true;

    //  检查运动模式是否改变
    int cur_mode = CheckMoveMode(path, id);
    int next_mode = CheckMoveMode(path, id + 1);
    if (cur_mode != next_mode)
        return true;

    float line_varangle_max = 10; //  直线允许最大变化角度
    //  检查车头朝向是否改变
    float heading1 = GetYawFromPose(path.poses[id]) * 180 / M_PI;
    float heading2 = GetYawFromPose(path.poses[id + 1]) * 180 / M_PI;
    if (fabs(heading1 - heading2) > line_varangle_max)
        return true;

    //  检查路径方向是否改变
    float line_angle1 = GetAngleByPoints(path.poses[id].pose.position, path.poses[id + 1].pose.position) * 180 / M_PI;
    float line_angle2 = GetAngleByPoints(path.poses[id + 1].pose.position, path.poses[id + 2].pose.position) * 180 / M_PI;
    if (fabs(line_angle1 - line_angle2) > line_varangle_max)
        return true;

    return false;
}

//  速度规划
void TLocalPathPlan::SpeedPlan(nav_msgs::Path &path, float min_vel, float safe_dis)
{
    float acc_per_len = 6.0 / 10, dec_per_len = -6.0 / 20; //  速度/距离系数

    // 加速约束,从前往后
    path.poses.begin()->pose.position.z = min_vel;
    for (int i = 1; i < path.poses.size(); i++)
    {
        float ref_vel = path.poses[i].pose.position.z;
        float cur_vel = path.poses[i - 1].pose.position.z;
        float ds = GetDistanceXY(path.poses[i].pose.position, path.poses[i - 1].pose.position);
        float vk = (ref_vel - cur_vel) / ds;

        if (vk > acc_per_len)
            vk = acc_per_len;
        else if (vk < dec_per_len)
            vk = dec_per_len;
        path.poses[i].pose.position.z = cur_vel + vk * ds;
    }

    // 减速约束,从后往前
    float len = 0;
    (path.poses.end() - 1)->pose.position.z = min_vel;
    for (int i = path.poses.size() - 2; i > 0; i--)
    {
        float ref_vel = path.poses[i].pose.position.z;
        float cur_vel = path.poses[i + 1].pose.position.z;
        float ds = GetDistanceXY(path.poses[i].pose.position, path.poses[i + 1].pose.position);
        float vk = (ref_vel - cur_vel) / ds;
        len += ds;

        if (fabs(vk) < 0.001)
            break;

        if (vk > acc_per_len)
            vk = acc_per_len;
        else if (vk < dec_per_len)
            vk = dec_per_len;
        if (len <= safe_dis)
            path.poses[i].pose.position.z = min_vel;
        else
            path.poses[i].pose.position.z = cur_vel + vk * ds;
    }

    // for(auto p : path.poses)  printf("%.2f ", p.pose.position.z);
    // printf("\n\n");
}

//  根据task_path生成全局路径golbal_path 1 路径点插值 2 加减速规划
void TLocalPathPlan::GlobalPathPlan()
{
    Nearest_ID = 0;
    global_path.header.stamp = ros::Time::now();
    if (task_path.poses.size() < 2)
    {
        global_path.poses.clear();
        return;
    }

    global_path.poses.clear();
    nav_msgs::Path plan_path; //  速度规划的路径
    // ROS_ERROR("%d %s\n",task_path.poses.size(), cur_task.cmd.c_str());
    for (int i = 0; i < task_path.poses.size() - 1; i++)
    {
        geometry_msgs::Point p1, p2;
        p1.x = task_path.poses[i].pose.position.x;
        p1.y = task_path.poses[i].pose.position.y;
        p2.x = task_path.poses[i + 1].pose.position.x;
        p2.y = task_path.poses[i + 1].pose.position.y;
        p1.z = p2.z = task_path.poses[i].pose.position.z;

        //  路径加密处理
        float ds = 0.5;
        // if (i == task_path.poses.size() - 2)  ds = last_path_precision;
        nav_msgs::Path path = LineInterpolation(p1, p2, ds);
        for (auto &p : path.poses)
            p.pose.orientation = task_path.poses[i].pose.orientation;

        plan_path.header = path.header;
        for (auto p : path.poses)
            plan_path.poses.push_back(p);

        //  判断停止点
        if (CheckStopAtNextPoint(task_path, i)) // 有折点要停止,整体进行速度规划, 加入全局路径
        {
            if (i == task_path.poses.size() - 2 && cur_task.cmd == "pick task")
                ;
            else
            {
                float safe_dis = 0.3;
                SpeedPlan(plan_path, 0.1, safe_dis);
            }
            // float safe_dis = 0.3;
            // if (i==task_path.poses.size()-2 && cur_task.cmd=="pick task")   safe_dis=2;
            // SpeedPlan(plan_path, 0.1, safe_dis);
            for (auto p : plan_path.poses)
                global_path.poses.push_back(p);
            plan_path.poses.clear();
            //
        }
    }

    if (global_path.poses.size() > 0)
        (global_path.poses.end() - 1)->pose.position.z = 0;

    // printf("globalpath size=%d\n", global_path.poses.size());
}

bool TLocalPathPlan::CheckTaskFinished()
{
    float task_dt = (ros::Time::now() - cur_task.stamp).toSec(); //  任务运行时间
    string paw_state;
    nh_local->getParam("/pawcontrol/paw_state", paw_state);

    bool res = false;
    if ((global_path.poses.size() == 0 || path_remain < 0.01) && task_dt > 4 && cur_task.cmd.find("task") != string::npos)
        res = true;
    if (cur_task.cmd == "pick task" && paw_state == "baojia_done" && task_dt > 4)
        res = true;

    // if(res)
    // {
    //     cur_task.cmd="";
    //     task_path.poses.clear();
    //     global_path.poses.clear();
    //     ROS_INFO("clear task!");
    // }
    return res;
}

//  局部路径规划
void TLocalPathPlan::LocalPathPlan()
{
    local_path.header.stamp = ros::Time::now();
    local_path.poses.clear();

    // if(CheckStopByPawState()) return;

    FindNearestPointInPath(global_path, Nearest_ID, 2);

    // path_remain = GetPathLength(global_path, Nearest_ID);   //  获取剩余路程
    float path_passed_temp;
    // 获得已经走过的路程和剩余路程  待调试
    GetPathLength(global_path, Nearest_ID, path_passed_temp, path_remain);
    if (is_lidar_global_path)
        path_passed = path_passed_when_lidar_give_path + path_passed_temp;
    else path_passed = path_passed_temp;
    std_msgs::Float64 data_msg;
    data_msg.data = path_remain;
    remainpath_pub.publish(data_msg);
    data_msg.data = path_passed;
    passedpath_pub.publish(data_msg);

    if (CheckTaskFinished())
        return;

    // printf("task=%s path_remain=%.2f paw_state=%s\n", cur_task.cmd.c_str(), path_remain, paw_state.c_str());
    float ds = 0;
    for (int i = Nearest_ID; i < global_path.poses.size(); i++)
    {
        local_path.poses.push_back(global_path.poses[i]);

        if (i > 0)
        {
            geometry_msgs::Point p1 = global_path.poses[i].pose.position;
            geometry_msgs::Point p2 = global_path.poses[i - 1].pose.position;
            ds += GetDistanceXY(p1, p2);
        }

        if (ds > 20 || CheckStopAtNextPoint(global_path, i))
        {
            if (i < global_path.poses.size() - 1)
                local_path.poses.push_back(global_path.poses[i + 1]);
            break;
        }
    }

    float safe_dis = 0.2;
    static float slow_down_speed = 0;
    if (cur_task.cmd == "pick task" && path_remain < 1)
    {
        float len = 0.5 * (len2.value + len4.value);
        if (len > 0.1 && center2_x.value + center4_x.value > 0 && local_path.poses.size() > 0)
        {
            slow_down_speed = local_path.poses[0].pose.position.z;
            float center_y = fabs(0.5 * (center2_y.value + center4_y.value));
            if (center_y < safe_dis)
                slow_down_speed = 0.1;
            else if (center_y < safe_dis + 0.2)
                slow_down_speed = ((center_y - safe_dis) * slow_down_speed + (safe_dis + 0.2 - center_y) * 0.1) / 0.2;
        }
        local_path.poses[0].pose.position.z = slow_down_speed;
    }
    else if (local_path.poses.size() > 0)
        slow_down_speed = local_path.poses[0].pose.position.z;

    // for(auto p : local_path.poses)  printf("%.2f ", p.pose.position.z);
    // printf("\nBBB:\n");
    // ROS_INFO("%d\n", local_path.poses.size());
}

// // 发布target_link, 必须有 shotpose
void TLocalPathPlan::BrTargetLink()
{
    // ROS_INFO("aaa=%s",shotpose.header.frame_id.c_str());

    // printf("111111111111111111111\n");
    if (shotpose.header.frame_id == "")
        return;

    static tf::TransformBroadcaster br1;
    tf::Transform transform;
    tf::Quaternion quaternion;

    tf::Vector3 origin = {shotpose.pose.position.x, shotpose.pose.position.y, 0};
    transform.setOrigin(origin);

    double roll, pitch, yaw;
    tf::Quaternion q;
    tf::quaternionMsgToTF(shotpose.pose.orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    tf::quaternionMsgToTF(tf::createQuaternionMsgFromYaw(yaw), quaternion);
    transform.setRotation(quaternion); // base_link在map中的旋转四元数
    br1.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "target_link"));
    // ROS_INFO("222222222222222222222\n");


}

//  主运行函数
void TLocalPathPlan::Run()
{
    nh_local->getParam("/gps_base/utmx_zero", utm_x_zero);
    nh_local->getParam("/gps_base/utmy_zero", utm_y_zero);

    static TTimer tmr;
    if (tmr.GetValue() > 1)
    {
        tmr.Clear();
        taskpath_pub.publish(task_path);
        globalpath_pub.publish(global_path);
    }

    LocalPathPlan();
    localpath_pub.publish(local_path);
    PubPathMarkers();

    // printf("%d\n", local_path.poses.size());
    nodecheck->Find("node_rate")->Beat();
    BrTargetLink();
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "local path plan");
    TLocalPathPlan localpathplan;

    ros::Rate rate(50);
    while (ros::ok())
    {
        localpathplan.Run();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}