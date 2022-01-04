#ifndef SUBCLASS_H
#define SUBCLASS_H

#include <QAbstractItemView>
#include <QMainWindow>
#include <QStandardItemModel>
#include <QStringList>
#include <QStringListModel>
#include <map>

#include "ros/ros.h"
#include "std_msgs/String.h"
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

// #include "myThreadHandle.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


class TmTcp
{
  public:
    TmTcp();
    TmTcp(double _x, double _y, double _z, double _Rx, double _Ry, double _Rz, std::string unit_l, std::string unit_a);
    //unit Meters / degrees
    double x;
    double y;
    double z;
    double Rx;
    double Ry;
    double Rz;
    std::string ToCmdString(int speed_percent = 100);
    // std::string ToString();
};

class TmJoint
{
  public:
    TmJoint();
    TmJoint(double _j0, double _j1, double _j2, double _j3, double _j4, double _j5, std::string unit_a);
    double j0;
    double j1;
    double j2;
    double j3;
    double j4;
    double j5;
    void ToCmdString(int speed_percent = 100);
    // void ToString();
};





#endif// SUBCLASS_H
