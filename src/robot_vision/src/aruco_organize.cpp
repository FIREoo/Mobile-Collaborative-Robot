#include "ros/ros.h"
#include <robot_vision/ArUcoList.h>

void chatterCallback(const robot_vision::ArUcoListConstPtr &msg)
{
    for (int i = 0; i < msg->dict.size(); i++)
    {
        ROS_INFO("[x%d]-[%d]", msg->dict[i], msg->id[i]);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Basic_Subscriber");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/kinect/aruco/list", 1, chatterCallback);

    ros::spin();

    return 0;
}