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

#include "myThreadHandle.h"
#include <robot_vision/RotationTools.h>

#define C033 "\033[0;33m"
#define C094 "\033[0;94m"
#define C0 "\033[0m"


void MainWindow::on_pushButton_NE_light_clicked()
{
    // ros::NodeHandle n;
    ros::Publisher set_digit_pub;
    set_digit_pub = nh.advertise<const joyControl::DigitLEDControl>("/digit_led_command", 10);

    joyControl::DigitLEDControl DigitControl;
    DigitControl.left_dig_led = false;
    DigitControl.right_dig_led = true;
    DigitControl.left_run_time = 0.5;
    DigitControl.right_run_time = 0.5;

    // publish once!
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        int num = set_digit_pub.getNumSubscribers();
        if (num > 0)
        {
            set_digit_pub.publish(DigitControl);
            ROS_INFO_STREAM("control");
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void MainWindow::on_pushButton_NE_moveLittle_clicked()
{
    ros::Publisher set_velocity_pub;
    set_velocity_pub = nh.advertise<const geometry_msgs::Twist>("/joystick_set_velocity", 10);
    geometry_msgs::Twist geControlSpeed;
    geControlSpeed.linear.x = 0.5f * 0.1f;
    geControlSpeed.angular.z = 0 * 10.0f;

    geometry_msgs::Twist ControlSpeed_stop;
    ControlSpeed_stop.linear.x = 0.0f;
    ControlSpeed_stop.angular.z = 0.0f;

    // publish once!
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        int num = set_velocity_pub.getNumSubscribers();
        if (num > 0)
        {
            set_velocity_pub.publish(geControlSpeed);
            ROS_INFO_STREAM("go");
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    sleep(1);

    while (ros::ok())
    {
        int num = set_velocity_pub.getNumSubscribers();
        if (num > 0)
        {
            set_velocity_pub.publish(ControlSpeed_stop);
            ROS_INFO_STREAM("stop");
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}
void MainWindow::on_pushButton_NE_moveF_clicked()
{
    geometry_msgs::Twist geControlSpeed;
    geControlSpeed.linear.x = 0.6f * 0.1f;
    geControlSpeed.angular.z = 0 * 10.0f;

    geometry_msgs::Twist ControlSpeed_stop;
    ControlSpeed_stop.linear.x = 0.0f;
    ControlSpeed_stop.angular.z = 0.0f;
    NewEra_speed_pub.publish(geControlSpeed);
}

void MainWindow::on_pushButton_NE_moveB_clicked()
{
    geometry_msgs::Twist geControlSpeed;
    geControlSpeed.linear.x = -0.6f * 0.1f;
    geControlSpeed.angular.z = 0 * 10.0f;

    geometry_msgs::Twist ControlSpeed_stop;
    ControlSpeed_stop.linear.x = 0.0f;
    ControlSpeed_stop.angular.z = 0.0f;
    NewEra_speed_pub.publish(geControlSpeed);
}
void MainWindow::on_pushButton_NE_break_clicked()
{
    ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("/enable_wheel_brake");
    std_srvs::Empty srv;
    if (client.call(srv))
    {
        ROS_INFO_STREAM("New Era : Break");
    }
    else
    {
        ROS_ERROR("Failed to call service /enable_wheel_brake");
    }
}

void MainWindow::on_pushButton_NE_releaseBreak_clicked()
{
    ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("/enable_wheel_release");
    std_srvs::Empty srv;
    if (client.call(srv))
    {
        ROS_INFO_STREAM("New Era : Release Break");
    }
    else
    {
        ROS_ERROR("Failed to call service /enable_wheel_release");
    }
}

/*navigation*/
void MainWindow::on_pushButton_NE_get_pose_clicked()
{
    geometry_msgs::TransformStamped trans;
    trans = lookupTransform("world", "upper_camera2/track/aruco/5x5/5");
    ROS_INFO("M03(%.3lf,%.3lf,%.3lf)", trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z);
}
void MainWindow::on_pushButton_NE_goto_A_clicked()
{
    ROS_INFO("M03 Move to A");
    geometry_msgs::Twist geControlSpeed;
    geControlSpeed.linear.x = -0.6f * 0.1f;
    geControlSpeed.angular.z = 0 * 10.0f;
    geometry_msgs::TransformStamped trans;
    trans = lookupTransform("world", "upper_camera2/track/aruco/5x5/5");
    ros::Rate loop_rate(4);
    while (true)
    {
        trans = lookupTransform("world", "upper_camera2/track/aruco/5x5/5");
        if (trans.transform.translation.x < -1.7)
        {
            break;
        }
        NewEra_speed_pub.publish(geControlSpeed);
        loop_rate.sleep();
    }

    geometry_msgs::Twist ControlSpeed_stop;
    ControlSpeed_stop.linear.x = 0.0f;
    ControlSpeed_stop.angular.z = 0.0f;
    NewEra_speed_pub.publish(ControlSpeed_stop);
    ROS_INFO("M03 Arrive A");
}
void MainWindow::on_pushButton_NE_goto_WS_clicked()
{
    ROS_INFO("M03 Move to WS");
    geometry_msgs::Twist geControlSpeed;
    geControlSpeed.linear.x = 0.6f * 0.1f;
    geControlSpeed.angular.z = 0 * 10.0f;
    geometry_msgs::TransformStamped trans;
    trans = lookupTransform("world", "upper_camera2/track/aruco/5x5/5");
    ros::Rate loop_rate(4);
    while (true)
    {
        trans = lookupTransform("world", "upper_camera2/track/aruco/5x5/5");
        if (trans.transform.translation.x > -1.05)
        {
            break;
        }
        NewEra_speed_pub.publish(geControlSpeed);
        loop_rate.sleep();
    }

    geometry_msgs::Twist ControlSpeed_stop;
    ControlSpeed_stop.linear.x = 0.0f;
    ControlSpeed_stop.angular.z = 0.0f;
    NewEra_speed_pub.publish(ControlSpeed_stop);
    ROS_INFO("M03 Arrive WS");
}
