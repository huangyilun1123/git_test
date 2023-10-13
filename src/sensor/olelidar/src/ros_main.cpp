#include"ros_node.h"
#include <memory>
#include <string>
#include <utility>

int main(int argc, char **argv)
{
   ros::init(argc, argv, "olelidar");
   ros::NodeHandle pnh("~");
   olelidar::Ros_node node(pnh);
   ros::spin();
}