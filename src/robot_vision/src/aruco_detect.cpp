#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <image_transport/image_transport.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <iostream>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <robot_vision/RotationTools.h>
namespace Rt = Rotation;

std::string frame_id;
// ros::Publisher pub_aruco;
image_transport::Subscriber sub;
image_transport::Publisher pub_aurcoImage;
ros::Publisher pub_aurco0_pose;

double fx = 978.451;
double fy = 978.683;
double cx = 1023.29;
double cy = 779.833;

double k1 = 0.362504;
double k2 = -2.49193;
double p1 = 0.000118025;
double p2 = -0.000162683;

double k3 = 1.44097;

double k4 = 0.241069;
double k5 = -2.30814;
double k6 = 1.36445;

void img_callback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        //cv::Mat getImg = cv_bridge::toCvCopy(msg, "bgr8")->image;//for sensor_msgs::CompressedImageConstPtr
        cv::Mat getImg = cv_bridge::toCvShare(msg, "bgr8")->image;

        cv::Mat showImg;
        getImg.copyTo(showImg);
        std::vector<int> ids_4x4;
        std::vector<int> ids_5x5;
        std::vector<std::vector<cv::Point2f>> corners_4x4;
        std::vector<std::vector<cv::Point2f>> corners_5x5;
        cv::Ptr<cv::aruco::Dictionary> dictionary_4x4 = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
        cv::Ptr<cv::aruco::Dictionary> dictionary_5x5 = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
        cv::aruco::detectMarkers(getImg, dictionary_4x4, corners_4x4, ids_4x4);
        cv::aruco::detectMarkers(getImg, dictionary_5x5, corners_5x5, ids_5x5);

        cv::Mat cameraMatrix = (cv::Mat_<float>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
        cv::Mat distCoeffs;
        if (k4 != 0)
            distCoeffs = (cv::Mat_<float>(8, 1) << k1, k2, p1, p2, k3, k4, k5, k6);
        else if (k4 == 0)
            distCoeffs = (cv::Mat_<float>(5, 1) << k1, k2, p1, p2, k3);
        else
            ROS_ERROR("camera value error!!");

        if (ids_5x5.size() > 0)
        {
            cv::aruco::drawDetectedMarkers(showImg, corners_5x5, ids_5x5);
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners_5x5, 0.08, cameraMatrix, distCoeffs, rvecs, tvecs);

            cv::Mat rotationMatrix = (cv::Mat_<float>(3, 3) << 0, 0, 0, 0, 0, 0, 0, 0, 0);
            for (int i = 0; i < ids_5x5.size(); i++)
            {
                try
                {
                    cv::Rodrigues(rvecs[i], rotationMatrix);
                }
                catch (const std::exception &e)
                {
                    ROS_ERROR("%s", e.what());
                }
                Rt::RotationMatrix RM = Rt::RotationMatrix(rotationMatrix);
                std::vector<Rt::EulerAngles> euler = Rt::rotationMatrix2eulerAngles(RM);

                cv::aruco::drawAxis(showImg, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.03);
                // ROS_INFO("T(M): % 1.3lf  % 1.3lf  % 1.3lf//R(deg): % 1.3lf  % 1.3lf  % 1.3lf", tvecs[i].val[0], tvecs[i].val[1], tvecs[i].val[2], toDeg(euler[0].x), toDeg(euler[0].y), toDeg(euler[0].z));

                /*send tf boradcaster*/
                static tf2_ros::TransformBroadcaster broadcaster;
                geometry_msgs::TransformStamped transformStamped;
                transformStamped.header.stamp = ros::Time::now();
                transformStamped.header.frame_id = frame_id;
                transformStamped.child_frame_id = frame_id + "/track/aruco/5x5/" + std::to_string(ids_5x5[i]);
                transformStamped.transform.translation.x = tvecs[i].val[0];
                transformStamped.transform.translation.y = tvecs[i].val[1];
                transformStamped.transform.translation.z = tvecs[i].val[2];
                tf2::Quaternion q;
                q.setRPY(euler[0].x, euler[0].y, euler[0].z);
                transformStamped.transform.rotation.x = q.x();
                transformStamped.transform.rotation.y = q.y();
                transformStamped.transform.rotation.z = q.z();
                transformStamped.transform.rotation.w = q.w();
                broadcaster.sendTransform(transformStamped);
            }
        }
        if (ids_4x4.size() > 0)
        {
            cv::aruco::drawDetectedMarkers(showImg, corners_4x4, ids_4x4);
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners_4x4, 0.06, cameraMatrix, distCoeffs, rvecs, tvecs);

            cv::Mat rotationMatrix = (cv::Mat_<float>(3, 3) << 0, 0, 0, 0, 0, 0, 0, 0, 0);
            for (int i = 0; i < ids_4x4.size(); i++)
            {
                try
                {
                    cv::Rodrigues(rvecs[i], rotationMatrix);
                }
                catch (const std::exception &e)
                {
                    ROS_ERROR("%s", e.what());
                }
                Rt::RotationMatrix RM = Rt::RotationMatrix(rotationMatrix);
                std::vector<Rt::EulerAngles> euler = Rt::rotationMatrix2eulerAngles(RM);

                cv::aruco::drawAxis(showImg, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.03);
                // ROS_INFO("T(M): % 1.3lf  % 1.3lf  % 1.3lf//R(deg): % 1.3lf  % 1.3lf  % 1.3lf", tvecs[i].val[0], tvecs[i].val[1], tvecs[i].val[2], toDeg(euler[0].x), toDeg(euler[0].y), toDeg(euler[0].z));

                /*send tf boradcaster*/
                static tf2_ros::TransformBroadcaster broadcaster;
                geometry_msgs::TransformStamped transformStamped;
                transformStamped.header.stamp = ros::Time::now();
                transformStamped.header.frame_id = frame_id;
                transformStamped.child_frame_id = frame_id + "/track/aruco/4x4/" + std::to_string(ids_4x4[i]);
                transformStamped.transform.translation.x = tvecs[i].val[0];
                transformStamped.transform.translation.y = tvecs[i].val[1];
                transformStamped.transform.translation.z = tvecs[i].val[2];
                tf2::Quaternion q;
                q.setRPY(euler[0].x, euler[0].y, euler[0].z);
                transformStamped.transform.rotation.x = q.x();
                transformStamped.transform.rotation.y = q.y();
                transformStamped.transform.rotation.z = q.z();
                transformStamped.transform.rotation.w = q.w();
                broadcaster.sendTransform(transformStamped);
            }
        }
        sensor_msgs::ImagePtr msg_out = cv_bridge::CvImage(std_msgs::Header(), "bgr8", showImg).toImageMsg();
        pub_aurcoImage.publish(msg_out);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("aruco cv_bridge error : %s", e.what());
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "aruco_detect");
    ros::NodeHandle nh;

    ros::param::get("~frame_id", frame_id);
    ros::param::get("~fx", fx);
    ros::param::get("~fy", fy);
    ros::param::get("~cx", cx);
    ros::param::get("~cy", cy);
    ros::param::get("~k1", k1);
    ros::param::get("~k2", k2);
    ros::param::get("~p1", p1);
    ros::param::get("~p2", p2);
    ros::param::get("~k3", k3);
    ros::param::get("~k4", k4);
    ros::param::get("~k5", k5);
    ros::param::get("~k6", k6);


    // ImageTransport
    image_transport::ImageTransport it(nh);
    ros::Subscriber sub = nh.subscribe("/source_image", 1, img_callback);

    pub_aurcoImage = it.advertise(sub.getTopic() + "/aruco", 1);
    // pub_aurco0_pose = n.advertise<geometry_msgs::Pose>("img_process/track/aruco/5x5/0", 1);

    ros::spin();
}
