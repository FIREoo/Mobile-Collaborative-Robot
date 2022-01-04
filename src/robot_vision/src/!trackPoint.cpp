#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// TM Driver header
#include "tm_msgs/SendScript.h"
#include "tm_msgs/SetPositions.h"

bool servoOn = false;

void sendCMD(std::string cmd);

void getPointCallback(const geometry_msgs::Pose::ConstPtr &point)
{

    // ROS_INFO_STREAM("point = " + std::to_string(Pt.x) + "," + std::to_string(Pt.y));
    // ROS_INFO_STREAM("d = " + std::to_string(dx) + "," + std::to_string(dy));
    // ROS_INFO_STREAM("tm = " + std::to_string(tm_dx) + "," + std::to_string(tm_dy));
    // sendCMD("SetContinueVLine(" + std::to_string(tm_dx) + "," + std::to_string(tm_dy) + ",0,0,0,0)");
    // sleep(3);
    // sendCMD("SetContinueVLine(" + std::to_string(0) + "," + std::to_string(0) + ",0.00,0,0,0)");
    // sleep(1);
}
void servoPoint()
{
    ros::NodeHandle node;
    tf::TransformListener listener_TM;
    tf::TransformListener listener_track;
    ros::Rate rate(10.0);
    while (node.ok() && servoOn)
    {
        tf::StampedTransform transform;
        try
        {
            // TODO transform track -> TM is the velocity control value!!
            listener_TM.lookupTransform("world", "/TM_robot", ros::Time(0), transform);
            listener_track.lookupTransform("world", "/track", ros::Time(0), transform);
            double x = transform.getOrigin().x();
            double y = transform.getOrigin().y();
            ROS_INFO_STREAM("W-A x = " + std::to_string(x) + " y = " + std::to_string(y));
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tracker");
    ros::NodeHandle n;
    // sendCMD("StopAndClearBuffer(0)");
    // sendCMD("PTP(\"JPP\",-130,0,90,0,90,0,70,200,0,false)");
    sendCMD("PTP(\"CPP\",-450,-350,230,110,0,-45,100,200,0,false)"); // mm-deg
    sleep(5);
    sendCMD("StopContinueVmode()");
    sendCMD("SuspendContinueVmode()");
    sleep(1);
    sendCMD("ContinueVLine(1000,10000)");
    // ros::Subscriber sub = n.subscribe("/track/pose", 1000, getPointCallback);

    tf::TransformListener listener;

    ros::Rate rate(10.0);
    while (n.ok())
    {
        tf::StampedTransform transform;
        try
        {
            double x = 0;
            double y = 0;

            listener.lookupTransform("/world", "/TM_robot", ros::Time(0), transform);
            x = transform.getOrigin().x();
            y = transform.getOrigin().y();
            ROS_INFO_STREAM("TM\tx = " + std::to_string(x) + " y = " + std::to_string(y));

            // listener.lookupTransform("/world", "/kinect", ros::Time(0), transform);
            // x = transform.getOrigin().x();
            // y = transform.getOrigin().y();
            // ROS_INFO_STREAM("kinect\tx = " + std::to_string(x) + " y = " + std::to_string(y));

            listener.lookupTransform("/world", "/track", ros::Time(0), transform);
            x = transform.getOrigin().x();
            y = transform.getOrigin().y();
            ROS_INFO_STREAM("Point\tx = " + std::to_string(x) + " y = " + std::to_string(y));

            listener.lookupTransform("/TM_robot", "/track", ros::Time(0), transform);
            x = transform.getOrigin().x();
            y = transform.getOrigin().y();
            ROS_INFO_STREAM("vel\tx = " + std::to_string(x) + " y = " + std::to_string(y));
            double a = 0.2;
            double Vx = x + (a * x * x);
            double Vy = y + (a * y * y);
            sendCMD("SetContinueVLine(" + std::to_string(Vx) + "," + std::to_string(Vy) + ",0,0,0,0)");
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

void sendCMD(std::string cmd)
{
    if (cmd != "")
    {
        ros::NodeHandle nh_demo;
        ros::ServiceClient client = nh_demo.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");
        tm_msgs::SendScript srv;
        srv.request.id = "demo";
        srv.request.script = cmd;
        if (client.call(srv))
        {
            if (srv.response.ok)
                ROS_INFO_STREAM("Sent script to robot:" + cmd);
            else
                ROS_WARN_STREAM("Sent script to robot , but response not yet ok ");
        }
        else
        {
            ROS_ERROR_STREAM("Error send script to robot");
        }
    }
}