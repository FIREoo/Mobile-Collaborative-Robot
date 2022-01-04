#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

void img_callback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        cv::Mat getImg = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::Mat dst;
        cv::Mat cameraMatrix = (cv::Mat_<float>(3, 3) << 978.451, 0.0, 1023.29, 0.0, 978.683, 779.833, 0.0, 0.0, 1.0);
        cv::Mat distCoeffs = (cv::Mat_<float>(8, 1) << 0.362504, -2.49193, 0.000118025, -0.000162683, 1.44097, 0.241069, -2.30814, 1.36445);
        //AVerMedia PW313D (L) 1920*1080 (6)
        //1628.224, 0.0, 960.870, 0, 1628.870, 538.115, 0, 0,1
        //[[ 0.15364596 -0.79704319  0.00101418  0.00209017  0.56643953]]

        //AVerMedia PW313D (R) 2592*1944 (2)
        //2009.421, 0.0, 1282.522, 0, 2011.409, 979.453, 0, 0,1
        //0.109171439, -0.791665577, -0.000326078853, -0.000941077195, 1.09959228



        cv::undistort(getImg, dst, cameraMatrix, distCoeffs);
        cv::imshow("getImg", getImg);
        cv::imshow("dst", dst);
        cv::imwrite("/home/fire/Desktop/NewEra_ws/RGB_cal.png", dst);
        cv::waitKey(3);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kinectVision_node");
    ros::NodeHandle n;
    image_transport::Subscriber sub;
    image_transport::ImageTransport it(n);
    sub = it.subscribe("/rgb/image_raw", 10, img_callback);
    cv::namedWindow("getImg");
    cv::namedWindow("dst");
    ros::spin();
}