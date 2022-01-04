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

image_transport::Publisher pub_img;
/*subfunction*/
geometry_msgs::TransformStamped lookupTransform(const std::string target_frame, const std::string source_frame);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "upper_camera");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    pub_img = it.advertise("camera/image", 1);// camera/image

    /*camera*/
    int index;
    nh.getParam("device_index", index);
    cv::VideoCapture cap(index);
    // cv::VideoWriter codec('M', 'J', 'P', 'G');
    cv::VideoWriter codec;
    cap.set(6, codec.fourcc('M', 'J', 'P', 'G'));
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
        if (ret == false)
            continue;

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_cam).toImageMsg();
        pub_img.publish(msg);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
