#include "ros/ros.h"
#include "string.h"
#include <deque>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <vision_msg/ArUcoList.h>

#define FIFO_SIZE 10

struct pose
{
    double translation_x;
    double translation_y;
    double translation_z;
    double rotation_x;
    double rotation_y;
    double rotation_z;
    double rotation_w;
};
struct arucoInfo
{
    int dict;
    int id;
    bool fixed;
    std::string name;

    std::deque<pose> transform;
};

std::map<std::string, arucoInfo> map_aruco_list;

void initArUco()
{
    arucoInfo info;
    info.dict = 4;
    info.id = 0;
    info.name = "tm_tcp";
    info.fixed = false;
    map_aruco_list["4x4/0"] = info;

    info = arucoInfo();
    info.dict = 5;
    info.id = 1;
    info.name = "desk1";
    info.fixed = true;
    map_aruco_list["5x5/1"] = info;

    info = arucoInfo();
    info.dict = 5;
    info.id = 2;
    info.name = "desk2";
    info.fixed = true;
    map_aruco_list["5x5/2"] = info;

    info = arucoInfo();
    info.dict = 5;
    info.id = 5;
    info.name = "m03";
    info.fixed = false;
    map_aruco_list["5x5/5"] = info;
}


// ros::Publisher pub_aruco;
void arucoListCallback(const vision_msg::ArUcoListConstPtr &msg)
{

    for (int i = 0; i < msg->dict.size(); i++)
    {
        std::string dict = std::to_string(msg->dict[i]);
        std::string id = std::to_string(msg->id[i]);
        std::string find_key = dict + "x" + dict + "/" + id;
        auto iter = map_aruco_list.find(find_key);
        if (iter != map_aruco_list.end())
        {
            // ROS_INFO("kinect find : %s", iter->second.name.c_str());
            pose trans = { msg->translation_x[i], msg->translation_y[i], msg->translation_z[i], msg->rotation_x[i], msg->rotation_y[i], msg->rotation_z[i], msg->rotation_w[i] };
            iter->second.transform.push_back(trans);
            if (iter->second.transform.size() > FIFO_SIZE)
                iter->second.transform.pop_front();

            if (iter->second.transform.size() == FIFO_SIZE && iter->second.name == "desk1")
            {
                double sum_x = 0;
                double sum_y = 0;
                double sum_z = 0;
                double sum_rx = 0;
                double sum_ry = 0;
                double sum_rz = 0;
                double sum_rw = 0;

                for (int i = 0; i < FIFO_SIZE; i++)
                {
                    sum_x += iter->second.transform[i].translation_x;
                    sum_y += iter->second.transform[i].translation_y;
                    sum_z += iter->second.transform[i].translation_z;
                    sum_rx += iter->second.transform[i].rotation_x;
                    sum_ry += iter->second.transform[i].rotation_y;
                    sum_rz += iter->second.transform[i].rotation_z;
                    sum_rw += iter->second.transform[i].rotation_w;
                }
                sum_x /= FIFO_SIZE;
                sum_y /= FIFO_SIZE;
                sum_z /= FIFO_SIZE;
                sum_rx /= FIFO_SIZE;
                sum_ry /= FIFO_SIZE;
                sum_rz /= FIFO_SIZE;
                sum_rw /= FIFO_SIZE;
                ROS_INFO("%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t", sum_x, sum_y, sum_z, sum_rx, sum_ry, sum_rz, sum_rw);
                /*send tf boradcaster*/
                static tf2_ros::TransformBroadcaster broadcaster;
                geometry_msgs::TransformStamped transformStamped;
                transformStamped.header.stamp = ros::Time::now();
                transformStamped.header.frame_id = msg->frame_id[i];
                transformStamped.child_frame_id = msg->frame_id[i] + "/track/aruco/" + dict + "x" + dict + "/" + id + "/avg";

                // transformStamped.transform.translation.x = 10;
                // transformStamped.transform.translation.y = 0;
                // transformStamped.transform.translation.z = 0;
                transformStamped.transform.rotation.x = 0;
                transformStamped.transform.rotation.y = 0;
                transformStamped.transform.rotation.z = 0;
                transformStamped.transform.rotation.w = 1;
                transformStamped.transform.translation.x = sum_x;
                transformStamped.transform.translation.y = sum_y;
                transformStamped.transform.translation.z = sum_z;
                // transformStamped.transform.rotation.x = sum_ry;
                // transformStamped.transform.rotation.y = sum_ry;
                // transformStamped.transform.rotation.z = sum_rz;
                // transformStamped.transform.rotation.w = sum_rw;
                broadcaster.sendTransform(transformStamped);

                // geometry_msgs::Pose p;
                // p.position.x = sum_x;
                // p.position.y = sum_y;
                // p.position.z = sum_z;
                // pub_aruco.publish(p);
            }
        }
    }


    // ROS_INFO(" ");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aruco_organize_node");
    ros::NodeHandle nh;

    initArUco();
    std::string id1 = "kinect";
    std::string id2 = "upper_camera1";
    std::string id3 = "upper_camera2";
    // ros::param::get("~frame_id1", id1);
    // ros::param::get("~frame_id2", id2);
    // ros::param::get("~frame_id3", id3);

    ros::Subscriber sub_aruco_list[3];
    sub_aruco_list[0] = nh.subscribe(id1 + "/aruco/list", 1, arucoListCallback);
    // sub_aruco_list[1] = nh.subscribe(id2 + "/aruco/list", 1, arucoListCallback);
    // sub_aruco_list[2] = nh.subscribe(id3 + "/aruco/list", 1, arucoListCallback);
    // pub_aruco = nh.advertise<geometry_msgs::Pose>(id1 + "/aruco/avg", 1);
    ros::spin();

    return 0;
}