#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "subclass.h"

//*****TM Pose*****//
TmPose::TmPose()
{
}
TmPose::TmPose(double _x, double _y, double _z, double _Rx, double _Ry, double _Rz, std::string unit_l, std::string unit_a)
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

std::string TmPose::ToCmdString(int speed_percent /*=100*/)
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
std::string TmPose::ToString()
{
    std::stringstream str_x;
    std::stringstream str_y;
    std::stringstream str_z;
    std::stringstream str_Rx;
    std::stringstream str_Ry;
    std::stringstream str_Rz;

    //PTP inputs in unit:mm / deg
    str_x << std::fixed << std::setprecision(1) << x * 1000.0;
    str_y << std::fixed << std::setprecision(1) << y * 1000.0;
    str_z << std::fixed << std::setprecision(1) << z * 1000.0;
    str_Rx << std::fixed << std::setprecision(1) << Rx;
    str_Ry << std::fixed << std::setprecision(1) << Ry;
    str_Rz << std::fixed << std::setprecision(1) << Rz;

    std::string str_rtn = "p[";
    str_rtn += str_x.str() + ",";
    str_rtn += str_y.str() + ",";
    str_rtn += str_z.str() + ",";
    str_rtn += str_Rx.str() + ",";
    str_rtn += str_Ry.str() + ",";
    str_rtn += str_Rz.str();
    str_rtn += "] mm-deg";

    return str_rtn;
}

//*****TM Joint*****//
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

std::string TmJoint::ToCmdString(int speed_percent /*=100*/)
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
std::string TmJoint::ToString()
{
    std::stringstream str_0;
    std::stringstream str_1;
    std::stringstream str_2;
    std::stringstream str_3;
    std::stringstream str_4;
    std::stringstream str_5;

    //PTP inputs in unit:mm / deg
    str_0 << std::fixed << std::setprecision(1) << j0;
    str_1 << std::fixed << std::setprecision(1) << j1;
    str_2 << std::fixed << std::setprecision(1) << j2;
    str_3 << std::fixed << std::setprecision(1) << j3;
    str_4 << std::fixed << std::setprecision(1) << j4;
    str_5 << std::fixed << std::setprecision(1) << j5;

    std::string str_rtn = "j[";
    str_rtn += str_0.str() + ",";
    str_rtn += str_1.str() + ",";
    str_rtn += str_2.str() + ",";
    str_rtn += str_3.str() + ",";
    str_rtn += str_4.str() + ",";
    str_rtn += str_5.str();
    str_rtn += "] deg";

    return str_rtn;
}

//*****TM Robot*****//
TmRobot::TmRobot()
{
}
bool TmRobot::checkConnect()
{
    ros::ServiceClient client = nh.serviceClient<tm_msgs::ConnectTM>("tm_diver/connect_tm");
    tm_msgs::ConnectTM srv;
    srv.request.server = srv.request.TMSVR;
    srv.request.reconnect = true;
    srv.request.timeout = 0;
    srv.request.timeval = 0;

    if (client.call(srv))
    {
        if (srv.response.ok)
        {
            ROS_INFO_STREAM("ConnectTM to robot");
            return true;
        }
        else
            ROS_WARN_STREAM("ConnectTM to robot , but response not yet ok ");
    }
    else
    {
        ROS_ERROR_STREAM("Error ConnectTM to robot");
    }

    return false;
}

bool TmRobot::sendScript(std::string cmd, bool print /*= true*/)
{
    if (cmd != "")
    {
        ros::ServiceClient client = nh.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");
        tm_msgs::SendScript srv;
        srv.request.id = "script";
        srv.request.script = cmd;
        if (client.call(srv))
        {
            if (srv.response.ok)
            {
                if (print)
                    ROS_INFO_STREAM("\033[0;32m[script]\033[0m\033[2m" + cmd + "\033[0m");
                return true;
            }
            else
                ROS_WARN_STREAM("Sent script to robot , but response not yet ok ");
        }
        else
        {
            ROS_ERROR_STREAM("Error send script to robot");
        }
    }
    return false;
}

bool TmRobot::moveTo(TmPose tcp, int speed_percent, bool block, bool print)
{
    sendScript(tcp.ToCmdString(speed_percent), false);
    if (print)
        ROS_INFO_STREAM("\033[1;32m[Move To]\033[0m  " + tcp.ToString());
    if (block)
    {
        ros::Rate rate(10);//10hz
        while (nh.ok())
        {
            // ROS_DEBUG_STREAM(tcp_pose.ToString());
            if (
                abs(tcp.x - tcp_pose.x) < 0.001 &&
                abs(tcp.y - tcp_pose.y) < 0.001 &&
                abs(tcp.z - tcp_pose.z) < 0.001)
                break;

            rate.sleep();
            ros::spinOnce();
        }
    }
    return true;
}
bool TmRobot::moveTo(TmJoint joint, int speed_percent, bool block, bool print)
{
    sendScript(joint.ToCmdString(speed_percent), false);
    if (print)
        ROS_INFO_STREAM("\033[1;32m[Move To]\033[0m  " + joint.ToString());
    if (block)
    {
        ros::Rate rate(10);//10hz
        while (nh.ok())
        {
            // ROS_DEBUG_STREAM(axis_joint.ToString());
            if (
                abs(joint.j0 - axis_joint.j0) < 0.5 &&
                abs(joint.j1 - axis_joint.j1) < 0.5 &&
                abs(joint.j2 - axis_joint.j2) < 0.5 &&
                abs(joint.j3 - axis_joint.j3) < 0.5 &&
                abs(joint.j4 - axis_joint.j4) < 0.5 &&
                abs(joint.j5 - axis_joint.j5) < 0.5)
                break;

            rate.sleep();
            qApp->processEvents();
            ros::spinOnce();
        }
    }
    return true;
}
