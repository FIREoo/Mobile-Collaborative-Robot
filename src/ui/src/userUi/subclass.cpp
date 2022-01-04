#include "mainwindow.h"
#include "string.h"
#include "ui_mainwindow.h"
#include <QAbstractItemView>
#include <QMainWindow>
#include <QStandardItemModel>
#include <QStringList>
#include <QStringListModel>
#include <QtWidgets>
#include <map>
// ros
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/image_encodings.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
// TM, NewEra
#include "tm_msgs/FeedbackState.h"
#include "tm_msgs/SendScript.h"
#include "tm_msgs/SetIO.h"
#include "tm_msgs/SetPositions.h"
#include <robot_vision/PoseTrans.h>//service
#include <ui/DigitLEDControl.h>
// opencv
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// mine
#include "subclass.h"
// #include <robot_vision/RotationTools.h>//error multiple definition

TmTcp::TmTcp()
{
}
TmTcp::TmTcp(double _x, double _y, double _z, double _Rx, double _Ry, double _Rz, std::string unit_l, std::string unit_a)
{
    if (unit_l == "M" || unit_l == "m")
    {
        x = _x;
        y = _y;
        z = _z;
    }
    else if (unit_l == "cm")
    {
        x = _x / 100.0;
        y = _y / 100.0;
        z = _z / 100.0;
    }
    else if (unit_l == "mm")
    {
        x = _x / 1000.0;
        y = _y / 1000.0;
        z = _z / 1000.0;
    }
    else
    {
        throw std::runtime_error("Fail unit lenght");
    }

    if (unit_a == "rad" || unit_a == "r")
    {
        Rx = _Rx / M_PI * 180.0;
        Ry = _Ry / M_PI * 180.0;
        Rz = _Rz / M_PI * 180.0;
    }
    else if (unit_a == "deg" || unit_a == "d")
    {
        Rx = _Rx;
        Ry = _Ry;
        Rz = _Rz;
    }
    else
    {
        throw std::runtime_error("Fail unit angle");
    }
}

std::string TmTcp::ToCmdString(int speed_percent /*=100*/)
{
    std::stringstream str_x;
    std::stringstream str_y;
    std::stringstream str_z;
    std::stringstream str_Rx;
    std::stringstream str_Ry;
    std::stringstream str_Rz;

    //PTP inputs in unit:mm / deg
    str_x << std::fixed << std::setprecision(2) << x * 1000.0;
    str_y << std::fixed << std::setprecision(2) << y * 1000.0;
    str_z << std::fixed << std::setprecision(2) << z * 1000.0;
    str_Rx << std::fixed << std::setprecision(2) << Rx;
    str_Ry << std::fixed << std::setprecision(2) << Ry;
    str_Rz << std::fixed << std::setprecision(2) << Rz;

    std::string str_rtn = "PTP(\"CPP\",";
    str_rtn += str_x.str() + ",";
    str_rtn += str_y.str() + ",";
    str_rtn += str_z.str() + ",";
    str_rtn += str_Rx.str() + ",";
    str_rtn += str_Ry.str() + ",";
    str_rtn += str_Rz.str() + ",";
    str_rtn += std::to_string(speed_percent);
    str_rtn += ",300,0,false)";

    return str_rtn;
    //"PTP(\"CPP\",-450,-350,215,100,0,-45,100,200,0,false)"

    //"PTP(\"JPP\",0,0,90,0,90,0,70,200,0,false)"
    // 6 position
    //手臂末端移動速度百分比(%)
    //運動加速到最高速所花費時間(millisecond)
    //軌跡混合百分比(%)
    //取消精準到位
}


TmJoint::TmJoint()
{
}
TmJoint::TmJoint(double _j0, double _j1, double _j2, double _j3, double _j4, double _j5, std::string unit_a)
{
    if (unit_a == "rad" || unit_a == "r")
    {
        j0 = _j0 / M_PI * 180.0;
        j1 = _j1 / M_PI * 180.0;
        j2 = _j2 / M_PI * 180.0;
        j3 = _j3 / M_PI * 180.0;
        j4 = _j4 / M_PI * 180.0;
        j5 = _j5 / M_PI * 180.0;
    }
    else if (unit_a == "deg" || unit_a == "d")
    {
        j0 = _j0;
        j1 = _j1;
        j2 = _j2;
        j3 = _j3;
        j4 = _j4;
        j5 = _j5;
    }
    else
    {
        throw std::runtime_error("Fail unit angle");
    }
}

std::string TmJoint::ToCmdString(double _j0, double _j1, double _j2, double _j3, double _j4, double _j5, std::string unit_a)
{
    std::stringstream str_0;
    std::stringstream str_1;
    std::stringstream str_2;
    std::stringstream str_3;
    std::stringstream str_4;
    std::stringstream str_5;

    //PTP inputs in unit:mm / deg
    str_0 << std::fixed << std::setprecision(2) << j0;
    str_1 << std::fixed << std::setprecision(2) << j1;
    str_2 << std::fixed << std::setprecision(2) << j2;
    str_3 << std::fixed << std::setprecision(2) << j3;
    str_4 << std::fixed << std::setprecision(2) << j4;
    str_5 << std::fixed << std::setprecision(2) << j5;

    std::string str_rtn = "PTP(\"JPP\",";
    str_rtn += str_0.str() + ",";
    str_rtn += str_1.str() + ",";
    str_rtn += str_2.str() + ",";
    str_rtn += str_3.str() + ",";
    str_rtn += str_4.str() + ",";
    str_rtn += str_5.str() + ",";
    str_rtn += std::to_string(speed_percent);
    str_rtn += ",300,0,false)";

    return str_rtn;
}