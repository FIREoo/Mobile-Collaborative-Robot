#ifndef TMROBOT_H
#define TMROBOT_H

// #include <QAbstractItemView>
// #include <QMainWindow>
// #include <QStandardItemModel>
// #include <QStringList>
// #include <QStringListModel>
#include <QtWidgets/QApplication>
#include <map>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tm_msgs/ConnectTM.h"
#include "tm_msgs/FeedbackState.h"
#include "tm_msgs/SendScript.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
#define STOP_MODE 0
#define JOINT_MODE 1
#define POSE_MODE 2
#define SERVO_POSE_MODE 3
#define SERVO_JOINT_MODE 4
#define SERVO_POSE_TEST 5
#define SERVO_NA 666
#define SERVO_NA_mm 666000

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
    ros::Publisher pub_jog;
    int jog_mode;

  public:
    TmRobot();
    bool checkConnect();
    bool sendScript(std::string cmd, bool print = true);
    bool moveTo(TmPose tcp, int speed_percent, bool block, bool print);
    bool moveTo(TmJoint joint, int speed_percent, bool block, bool print);

    void set_jog_mode(std::string joint_or_pose, bool set_mode);//velocity_mode
    bool jog(TmPose tcp, int speed_percent, bool print = true);
    bool jog(TmJoint joint, int speed_percent, bool print = true);

    void set_servo_mode(std::string joint_or_pose, bool set_mode);
    void test_servo_mode(std::string joint_or_pose, bool set_mode);
    bool servo(TmPose tcp, int speed_percent, bool print = true);
    bool servo_test(TmPose tcp, int speed_percent, bool print = true);
    TmPose tcp_pose;
    TmPose tcp_speed;
    TmPose tcp_force;
    TmJoint axis_joint;
};


#endif// TMROBOT_H
