#ifndef TMROBOT_H
#define TMROBOT_H

#include <QAbstractItemView>
#include <QMainWindow>
#include <QStandardItemModel>
#include <QStringList>
#include <QStringListModel>
#include <map>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tm_msgs/ConnectTM.h"
#include "tm_msgs/FeedbackState.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


/**
 * @brief Tool space (Unit: meters degrees)
 */
class TmPose
{
  public:
    TmPose();
    TmPose(double _x, double _y, double _z, double _Rx, double _Ry, double _Rz, std::string unit_l, std::string unit_a);
    double x; //unit Meters
    double y; //unit Meters
    double z; //unit Meters
    double Rx;//unit degrees
    double Ry;//unit degrees
    double Rz;//unit degrees
    std::string ToCmdString(int speed_percent = 100);
    std::string ToString();
};


/**
 * @brief Joint space (Unit: degrees)
 */
class TmJoint
{
  public:
    TmJoint();
    TmJoint(double _j0, double _j1, double _j2, double _j3, double _j4, double _j5, std::string unit_a);
    double j0;//unit degrees
    double j1;//unit degrees
    double j2;//unit degrees
    double j3;//unit degrees
    double j4;//unit degrees
    double j5;//unit degrees

    std::string ToCmdString(int speed_percent = 100);
    std::string ToString();
};


class TmRobot
{
  private:
    ros::NodeHandle nh;

  public:
    TmRobot();
    bool checkConnect();
    bool sendScript(std::string cmd, bool print = true);
    bool moveTo(TmPose tcp, int speed_percent, bool block, bool print);
    bool moveTo(TmJoint joint, int speed_percent, bool block, bool print);

    TmPose tcp_pose;
    TmPose tcp_speed;
    TmJoint axis_joint;
};


#endif// TMROBOT_H
