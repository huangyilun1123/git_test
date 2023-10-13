#include <serial/serial.h>
#include <NavConversion.h>
#include <common/public.h>
#include <gps/MyGPS_msg.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <fstream>
// User defined includes
#include <common/mydisplay.h>

#include <common/public.h>

using namespace std;

class TGPS_Pro : public Thread
{
private:
    ros::NodeHandle *nh, *nh_local;
    ros::Publisher path_cur_pub, path_load_pub, path_save_pub, globalpath_pub;
    ros::Publisher marker_pub, gnss_pose;
    ros::Subscriber gps_base_sub;

    vector<geometry_msgs::Point> PathSaveBuf, PathLoadBuf;
    bool saveflag;

public:
    TNodeCheck *nodecheck;

    TGPS_Pro();
    void GPSDataCallback(const gps::MyGPS_msg::ConstPtr &msg);
    void run();
    void SavePath(char *filename);
    void LoadPath(char *filename);
    // void GenFileName(char* filename);
};

TGPS_Pro::TGPS_Pro()
{
    nh = new ros::NodeHandle;
    nh_local = new ros::NodeHandle("~");

    nodecheck = new TNodeCheck(nh_local, "node_rate", 1.0);
    nodecheck->Find("node_rate")->SetLimit(10);

    // path_cur_pub = nh->advertise<nav_msgs::Path>("GPS_Path",10);
    path_load_pub = nh->advertise<nav_msgs::Path>("Path_Load", 10);
    path_save_pub = nh->advertise<nav_msgs::Path>("Path_Save", 10);
    gnss_pose = nh->advertise<geometry_msgs::PoseStamped>("/gnss_pose", 10);

    gps_base_sub = nh->subscribe<gps::MyGPS_msg>("/gps_base/GPS_Base", 10, &TGPS_Pro::GPSDataCallback, this);
    // gps_base_sub = nh->subscribe<std_msgs::String>("/gps_base/GPS_RawData", 1, &TGPS_Pro::GPSDataCallback,this);

    marker_pub = nh->advertise<visualization_msgs::Marker>("car_marker", 1);
    saveflag = false;
    start();
}

void TGPS_Pro::GPSDataCallback(const gps::MyGPS_msg::ConstPtr &msg)
{
    if(msg->Quality==4)  nodecheck->Find("node_rate")->Beat();

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "base_link"; 
    pose_stamped.pose.orientation.w=1;
    pose_stamped.pose.position.z = msg->Vel; //速度信息
    marker_pub.publish(displayCarPosition(pose_stamped, msg->Quality));
    pose_stamped.pose.position.x = msg->map_x;
    pose_stamped.pose.position.y = msg->map_y;
    pose_stamped.pose.position.z = 0;
    pose_stamped.pose.orientation =  tf::createQuaternionMsgFromYaw(msg->Angle / 180 * 3.1416);
    gnss_pose.publish(pose_stamped);


    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(msg->Angle / 180 * 3.1416);
    static tf::TransformBroadcaster br1;
    tf::Transform transform;
    tf::Quaternion quaternion;
    transform.setOrigin(tf::Vector3(msg->map_x, msg->map_y, 0)); // base_link在map中的位置

    tf::quaternionMsgToTF(goal_quat, quaternion);
    transform.setRotation(quaternion);  // base_link在map中的旋转四元数
    br1.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link")); // shu_1

    if (saveflag && PathSaveBuf.size() < 100000)
    {
        geometry_msgs::Point p1, p2;
        if (PathSaveBuf.size() > 0)
            p1 = PathSaveBuf[PathSaveBuf.size() - 1];
        else
            p1.x = p1.y = p1.z = 0;

        p2.x = msg->map_x, p2.y = msg->map_y, p2.z = 0;
        if (P2P(p1, p2) > 1)
        {
            PathSaveBuf.push_back(p2);

            nav_msgs::Path path_msg;
            path_msg.header.stamp = ros::Time::now();
            path_msg.header.frame_id = "map";
            for (int i = 0; i < PathSaveBuf.size(); i++)
            {
                geometry_msgs::PoseStamped pose_stamped;
                pose_stamped.pose.position.x = PathSaveBuf[i].x;
                pose_stamped.pose.position.y = PathSaveBuf[i].y;

                pose_stamped.header.stamp = ros::Time::now();
                pose_stamped.header.frame_id = "map";
                path_msg.poses.push_back(pose_stamped);
            }
            path_save_pub.publish(path_msg);
        }
    }
}

void TGPS_Pro::SavePath(char *filename)
{
    if (PathSaveBuf.size() == 0)  return;

    FILE *fp = fopen(filename, "w");

    double utmx_zero, utmy_zero;
    ros::param::get("/gps_base/utmx_zero", utmx_zero);
    ros::param::get("/gps_base/utmy_zero", utmy_zero);

    for (int i = 0; i < PathSaveBuf.size(); i++)
    {
        fprintf(fp, "%.2f %.2f\n", PathSaveBuf[i].x + utmx_zero, PathSaveBuf[i].y + utmy_zero);
    }
    fclose(fp);
    // ROS_INFO("I heard: [%d]", record_track_points.size());
}

void TGPS_Pro::LoadPath(char *filename)
{
    PathLoadBuf.clear();
    if (strcmp(filename, "clear") != 0)
    {
        double utmx_zero = 0, utmy_zero = 0;
        ros::param::get("/gps_base/utmx_zero", utmx_zero);
        ros::param::get("/gps_base/utmy_zero", utmy_zero);

        std::ifstream file_oxts(filename);
        std::string line = "";
        while (getline(file_oxts, line))
        {
            vector<string> ss = split(line, " ");
            geometry_msgs::Point p;
            p.x = atof(ss[0].data());
            p.y = atof(ss[1].data());

            if (fabs(utmx_zero) + fabs(utmy_zero) < 0.001)
                utmx_zero = p.x, utmy_zero = p.y;
            p.x -= utmx_zero, p.y -= utmy_zero;
            PathLoadBuf.push_back(p);
            // printf("%.2f %.2f\n",p.x,p.y);
        }
        printf("there are %d trajectory points.\n", PathLoadBuf.size());
    }
}

void TGPS_Pro::run()
{
    while (1)
    {
        usleep(10000);

    	// if(nodecheck->Find("node_rate")->value<3)  ROS_ERROR_STREAM_THROTTLE(1, "GPS signal error!");

        bool flag = false;
        nh_local->getParam("saveflag", flag);
        if (!saveflag && flag)
        {
            printf("begin save path!\n");
            saveflag = flag;
            PathSaveBuf.clear();
        }
        else if (saveflag && !flag)
        {
            printf("stop save path!\n");
            saveflag = flag;
        }
        else
        {
            string fname = "";
            nh_local->getParam("savefilename", fname);
            nh_local->setParam("savefilename", "");
            if (fname != "")
            {
                printf("stop save path %s!\n", fname.c_str());
                char buf[200];
                strcpy(buf, fname.c_str());
                SavePath(buf);
            }
        }

        string pathfilename = "";
        nh_local->getParam("loadfilename", pathfilename);

        if (pathfilename != "")
        {
            printf("load path %s\n", pathfilename.c_str());
            char buf[200];
            strcpy(buf, pathfilename.c_str());
            LoadPath(buf);
            nh_local->setParam("loadfilename", "");
        }

        ros::spinOnce();
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "GPS_Pro");
    ROS_INFO_STREAM("gps pro start");

    TGPS_Pro gps;
    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
