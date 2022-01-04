#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <robot_vision/RotationTools.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
namespace Rt = Rotation;

tf2_ros::Buffer tfBuffer;
/*subfunction*/
geometry_msgs::TransformStamped lookupTransform(const std::string target_frame, const std::string source_frame);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "upper_camera");
    ros::NodeHandle nh;

    /*init tf*/
    static tf2_ros::StaticTransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "upper_camera/1";
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = 0;
    transformStamped.transform.translation.z = 0;
    transformStamped.transform.rotation.x = 0;
    transformStamped.transform.rotation.y = 0;
    transformStamped.transform.rotation.z = 0;
    transformStamped.transform.rotation.w = 1;
    broadcaster.sendTransform(transformStamped);
    ROS_INFO("--[Set] world->TM_robot/base to zero");
    tf2_ros::TransformListener tfListener(tfBuffer);

    /*camera*/
    int index;
    nh.getParam("device_index", index);
    cv::VideoCapture cap(index);
    // cv::VideoWriter codec('M', 'J', 'P', 'G');
    // cap.set(6, codec);
    cap.set(5, 30);//fps
    int width;
    nh.getParam("image_width", width);
    cap.set(3, width);
    int height;
    nh.getParam("image_height", height);
    cap.set(4, height);

    while (cap.isOpened() == false)
    {
        ROS_WARN("upper camera is not opened!");
        ros::Duration(1.0).sleep();
    }

    ROS_INFO("camera is open (%.1lf, %.1lf)", cap.get(3), cap.get(4));
    ros::Rate rate(10.0);
    while (ros::ok())
    {
        cv::Mat img_cam;
        bool ret = cap.read(img_cam);
        try
        {
            geometry_msgs::TransformStamped transformStamped;
            transformStamped = tfBuffer.lookupTransform("world", "track/aruco/5x5/1", ros::Time(0));
            ROS_INFO("tf T(Meter)[%lf,%lf,%lf]", transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();
}

geometry_msgs::TransformStamped lookupTransform(const std::string target_frame, const std::string source_frame)
{
    while (ros::ok())
    {
        try
        {
            geometry_msgs::TransformStamped transformStamped;
            transformStamped = tfBuffer.lookupTransform(target_frame, source_frame, ros::Time(0));
            return transformStamped;
            break;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
    }
    return geometry_msgs::TransformStamped();
}
