// ROS headers
// #include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
//tf
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
// std header
#include <cstdlib>
#include <sstream>

// TM Driver header
#include "tm_msgs/SendScript.h"

#define STOP_MODE 0
#define JOINT_MODE 1
#define POSE_MODE 2
#define SERVO_POSE_MODE 3
#define SERVO_JOINT_MODE 4
#define SERVO_POSE_TEST 5

#define SERVO_NA 666

ros::ServiceClient client;
tm_msgs::SendScript srv;

ros::Subscriber sub_jog_pose;
ros::Time cmd_time;
double value[6];
int jog_mode;
void jog_looper(const ros::TimerEvent &e);
bool send_script(std::string cmd, std::string print);

//servo
tf2_ros::Buffer tfBuffer;
// tf2_ros::TransformListener tfListener;
/**
 * @brief jog mode value in tool space(Unit: meters degrees) or joint space(Unit: degrees) 
 */
void jog_cmd_Callback(const geometry_msgs::TwistStampedConstPtr &msg)
{
    cmd_time = msg->header.stamp;
    value[0] = msg->twist.linear.x;
    value[1] = msg->twist.linear.y;
    value[2] = msg->twist.linear.z;
    value[3] = msg->twist.angular.x;
    value[4] = msg->twist.angular.y;
    value[5] = msg->twist.angular.z;
}

bool start_jog_pose(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
    if (jog_mode == POSE_MODE)//return if already in same mode
        return false;

    jog_mode = POSE_MODE;
    ROS_INFO_STREAM("\033[0;32m[Jog]\033[0m\033[3;92mpose mode\033[0m");
    send_script("StopAndClearBuffer(0)", "");
    send_script("ContinueVLine(500,10000)", "Jog");
    send_script("SuspendContinueVmode()", "");
    send_script("SetContinueVLine(0,0,0,0,0,0)", "");
    return true;
}
bool start_jog_joint(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
    if (jog_mode == JOINT_MODE)//return if already in same mode
        return false;

    jog_mode = JOINT_MODE;
    ROS_INFO_STREAM("\033[0;32m[Jog]\033[0m\033[3;92mjoint mode\033[0m");
    send_script("StopAndClearBuffer(0)", "");
    send_script("ContinueVJog()", "Jog");
    send_script("SetContinueVJog(0,0,0,0,0,0)", "");
    return true;
}
bool start_jog_stop(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
    if (jog_mode == STOP_MODE)//return if already in same mode
        return false;

    jog_mode = STOP_MODE;
    ROS_INFO_STREAM("\033[0;32m[Jog]\033[0m\033[3;92mstop\033[0m");
    send_script("StopAndClearBuffer(0)", "");
    send_script("SuspendContinueVmode()", "");
    send_script("StopContinueVmode()", "Jog");
    return true;
}
bool start_servo_pose(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
    if (jog_mode == SERVO_POSE_MODE)//return if already in same mode
        return false;

    jog_mode = SERVO_POSE_MODE;
    ROS_INFO_STREAM("\033[0;32m[servo]\033[0m\033[3;92mservo pose mode\033[0m");
    send_script("StopAndClearBuffer(0)", "");
    send_script("ContinueVLine(500,10000)", "servo");
    send_script("SuspendContinueVmode()", "");
    send_script("SetContinueVLine(0,0,0,0,0,0)", "");
    return true;
}
bool start_servo_joint(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
    if (jog_mode == SERVO_JOINT_MODE)//return if already in same mode
        return false;

    jog_mode = SERVO_JOINT_MODE;
    ROS_INFO_STREAM("\033[0;32m[servo]\033[0m\033[3;92mservo joint mode\033[0m");
    ROS_INFO("Not yet");
    // send_script("StopAndClearBuffer(0)", "");
    // send_script("ContinueVJog()", "Jog");
    // send_script("SetContinueVJog(0,0,0,0,0,0)", "");
    return true;
}
bool test_servo_pose(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
    if (jog_mode == SERVO_POSE_TEST)//return if already in same mode
        return false;

    jog_mode = SERVO_POSE_TEST;
    ROS_INFO_STREAM("\033[0;32m[servo]\033[0m\033[3;92mservo pose test\033[0m");
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "jog_helper");
    ros::NodeHandle nh;

    jog_mode = STOP_MODE;
    client = nh.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");
    ros::ServiceServer service_start_jog_pose = nh.advertiseService("tm_helper/jog/start/pose", start_jog_pose);
    ros::ServiceServer service_start_jog_joint = nh.advertiseService("tm_helper/jog/start/joint", start_jog_joint);
    ros::ServiceServer service_stop_jog = nh.advertiseService("tm_helper/jog/stop", start_jog_stop);
    ros::ServiceServer service_start_servo_pose = nh.advertiseService("tm_helper/servo/start/pose", start_servo_pose);
    ros::ServiceServer service_test_servo_pose = nh.advertiseService("tm_helper/servo/test/pose", test_servo_pose);
    ros::ServiceServer service_start_servo_joint = nh.advertiseService("tm_helper/servo/start/joint", start_servo_joint);
    ros::ServiceServer service_stop_servo = nh.advertiseService("tm_helper/servo/stop", start_jog_stop);
    sub_jog_pose = nh.subscribe("tm_helper/jog", 1, jog_cmd_Callback);
    // tfListener(tfBuffer);
    // tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Timer timer = nh.createTimer(ros::Duration(0.1), jog_looper);
    // ros::AsyncSpinner spinner(0);
    // spinner.start();//will not block
    // timer.start();
    // sleep(3);
    // timer.stop();
    // ros::waitForShutdown();
    ros::spin();
}

void jog_looper(const ros::TimerEvent &e)
{
    if (jog_mode == POSE_MODE)
    {
        send_script("SetContinueVLine(" + std::to_string(value[0]) + "," + std::to_string(value[1]) + "," + std::to_string(value[2]) + "," + std::to_string(value[3]) + "," + std::to_string(value[4]) + "," + std::to_string(value[5]) + ")", "Jog");
    }
    else if (jog_mode == JOINT_MODE)
    {
        send_script("SetContinueVJog(" + std::to_string(value[0]) + "," + std::to_string(value[1]) + "," + std::to_string(value[2]) + "," + std::to_string(value[3]) + "," + std::to_string(value[4]) + "," + std::to_string(value[5]) + ")", "Jog");
    }
    else if (jog_mode == SERVO_POSE_MODE)
    {
        geometry_msgs::TransformStamped transformStamped1;
        transformStamped1 = tfBuffer.lookupTransform("TM_robot/base", "TM_robot/tcp", ros::Time(0));//get unit M

        tf::Quaternion q(
            transformStamped1.transform.rotation.x,
            transformStamped1.transform.rotation.y,
            transformStamped1.transform.rotation.z,
            transformStamped1.transform.rotation.w);
        tf::Matrix3x3 m(q);
        double Rx, Ry, Rz;//rad
        m.getRPY(Rx, Ry, Rz);
        //to deg
        Rx = Rx * 180 / M_PI;
        Ry = Ry * 180 / M_PI;
        Rz = Rz * 180 / M_PI;

        double delay_sec = ros::Time::now().toSec() - cmd_time.toSec();
        double dx = (value[0] == SERVO_NA) ? 0 : value[0] - transformStamped1.transform.translation.x;
        double dy = (value[1] == SERVO_NA) ? 0 : value[1] - transformStamped1.transform.translation.y;
        double dz = (value[2] == SERVO_NA) ? 0 : value[2] - transformStamped1.transform.translation.z;

        double drx = (value[3] == SERVO_NA) ? 0 : (value[3] - Rx < Rx - value[3]) ? value[3] - Rx : Rx - value[3];
        double dry = (value[4] == SERVO_NA) ? 0 : (value[4] - Ry < Ry - value[4]) ? value[4] - Ry : Ry - value[4];
        double drz = (value[5] == SERVO_NA) ? 0 : (value[5] - Rz < Rz - value[5]) ? value[5] - Rz : Rz - value[5];

        // ROS_INFO("\033[0;32m[get]\033[0m\033[2m(% 1.3lf,% 1.3lf,% 1.3lf,% 1.3lf,% 1.3lf,% 1.3lf)\033[0m", value[0], value[1], value[2], value[3], value[4], value[5]);

        double a = 0.2;
        double Vx = dx + (a * dx * dx);
        double Vy = dy + (a * dy * dy);
        double Vz = dz + (a * dz * dz);
        double Vrx = drx + (a * drx * drx);
        double Vry = dry + (a * dry * dry);
        double Vrz = drz + (a * drz * drz);

        // speed limit
        double limit = 0.03;// 0.1
        Vx = (Vx > limit) ? limit : Vx;
        Vx = (Vx < -limit) ? -limit : Vx;
        Vy = (Vy > limit) ? limit : Vy;
        Vy = (Vy < -limit) ? -limit : Vy;
        Vz = (Vz > limit) ? limit : Vz;
        Vz = (Vz < -limit) ? -limit : Vz;
        double limit_r = 0.03;// 0.1
        Vrx = (Vrx > limit_r) ? limit_r : Vrx;
        Vrx = (Vrx < -limit_r) ? -limit_r : Vrx;
        Vry = (Vry > limit_r) ? limit_r : Vry;
        Vry = (Vry < -limit_r) ? -limit_r : Vry;
        Vrz = (Vrz > limit_r) ? limit_r : Vrz;
        Vrz = (Vrz < -limit_r) ? -limit_r : Vrz;

        if (delay_sec > 0.5)
        {
            ROS_INFO("\033[0;32m[Servo]\033[0m\033[2mLoss tracker (% 1.3lfsec)\033[0m", delay_sec);
            send_script("SetContinueVLine(0,0,0,0,0,0)", "");
        }
        else
        {
            send_script("SetContinueVLine(" + std::to_string(Vx) + "," + std::to_string(Vy) + "," + std::to_string(Vz) + ",0,0,0)", "servo");
        }
    }
    else if (jog_mode == SERVO_POSE_TEST)
    {
        geometry_msgs::TransformStamped transformStamped1;
        transformStamped1 = tfBuffer.lookupTransform("TM_robot/base", "TM_robot/tcp", ros::Time(0));//get unit M

        tf::Quaternion q(
            transformStamped1.transform.rotation.x,
            transformStamped1.transform.rotation.y,
            transformStamped1.transform.rotation.z,
            transformStamped1.transform.rotation.w);
        tf::Matrix3x3 m(q);
        double Rx, Ry, Rz;//rad
        m.getRPY(Rx, Ry, Rz);
        //to deg
        Rx = Rx * 180 / M_PI;
        Ry = Ry * 180 / M_PI;
        Rz = Rz * 180 / M_PI;

        double delay_sec = ros::Time::now().toSec() - cmd_time.toSec();
        double dx = (value[0] == SERVO_NA) ? 0 : value[0] - transformStamped1.transform.translation.x;
        double dy = (value[1] == SERVO_NA) ? 0 : value[1] - transformStamped1.transform.translation.y;
        double dz = (value[2] == SERVO_NA) ? 0 : value[2] - transformStamped1.transform.translation.z;

        double drx = (value[3] == SERVO_NA) ? 0 : (value[3] - Rx < Rx - value[3]) ? value[3] - Rx : Rx - value[3];
        double dry = (value[4] == SERVO_NA) ? 0 : (value[4] - Ry < Ry - value[4]) ? value[4] - Ry : Ry - value[4];
        double drz = (value[5] == SERVO_NA) ? 0 : (value[5] - Rz < Rz - value[5]) ? value[5] - Rz : Rz - value[5];

        // ROS_INFO("\033[0;32m[get]\033[0m\033[2m(% 1.3lf,% 1.3lf,% 1.3lf,% 1.3lf,% 1.3lf,% 1.3lf)\033[0m", value[0], value[1], value[2], value[3], value[4], value[5]);

        double a = 0.2;
        double Vx = dx + (a * dx * dx);
        double Vy = dy + (a * dy * dy);
        double Vz = dz + (a * dz * dz);
        double Vrx = drx + (a * drx * drx);
        double Vry = dry + (a * dry * dry);
        double Vrz = drz + (a * drz * drz);

        // speed limit
        double limit = 0.03;// 0.1
        Vx = (Vx > limit) ? limit : Vx;
        Vx = (Vx < -limit) ? -limit : Vx;
        Vy = (Vy > limit) ? limit : Vy;
        Vy = (Vy < -limit) ? -limit : Vy;
        Vz = (Vz > limit) ? limit : Vz;
        Vz = (Vz < -limit) ? -limit : Vz;
        double limit_r = 0.03;// 0.1
        Vrx = (Vrx > limit_r) ? limit_r : Vrx;
        Vrx = (Vrx < -limit_r) ? -limit_r : Vrx;
        Vry = (Vry > limit_r) ? limit_r : Vry;
        Vry = (Vry < -limit_r) ? -limit_r : Vry;
        Vrz = (Vrz > limit_r) ? limit_r : Vrz;
        Vrz = (Vrz < -limit_r) ? -limit_r : Vrz;

        ROS_INFO("\033[0;32m[Servo semulation]\033[0m\033[2m(% 1.3lf,% 1.3lf,% 1.3lf,% 1.3lf,% 1.3lf,% 1.3lf)(% 1.3lfsec)\033[0m", Vx, Vy, Vz, Vrx, Vry, Vrz, delay_sec);
    }
    else if (jog_mode == SERVO_JOINT_MODE)
    {
        ROS_INFO("Not yet");
    }
}
bool send_script(std::string cmd, std::string print)
{
    if (cmd != "")
    {
        tm_msgs::SendScript srv;
        srv.request.id = "jog";
        srv.request.script = cmd;
        if (client.call(srv))
        {
            if (srv.response.ok)
            {
                if (print != "")
                    ROS_INFO_STREAM("\033[0;32m[" + print + "]\033[0m\033[2m" + cmd + "\033[0m");
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