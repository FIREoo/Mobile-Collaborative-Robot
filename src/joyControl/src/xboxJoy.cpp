/* Includes ------------------------------------------------------------------*/
#include "ros/ros.h"
#include <iostream>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#include <joyControl/DigitLEDControl.h>

/* Defined Constants ---------------------------------------------------------*/
#define DIGIT_LED_CONTROL_TIME 10
#define RGB_LED_CONTROL_TIME 10

#define HIGH_LINEAR_SPEED 0.2f //max0.422
#define LOW_LINEAR_SPEED 0.1f
#define HIGH_ANGULAR_SPEED 20.0f
#define LOW_ANGULAR_SPEED 10.0f

/* Macros --------------------------------------------------------------------*/
#define PI() 4 * atan(1)

/* Global Variables ----------------------------------------------------------*/
ros::Publisher set_velocity_pub;
ros::Publisher set_rgb_pub;
ros::Publisher set_digit_pub;

ros::Subscriber joy_sub;

/* Declaration of Local Functions --------------------------------------------*/
void GetJoyPush(const sensor_msgs::Joy &JoyPush);

/* Definition of Local Functions ---------------------------------------------*/
int main(int argc, char **argv)
{
    //ROS Node initial
    ros::init(argc, argv, "joy_xbox");

    //make a node
    ros::NodeHandle nJoyCommand;

    //Publish
    set_digit_pub = nJoyCommand.advertise<const joyControl::DigitLEDControl>("/digit_led_command", 10);

    //Subscrible
    joy_sub = nJoyCommand.subscribe("/joy", 10, GetJoyPush);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void GetJoyPush(const sensor_msgs::Joy &JoyPush)
{
    bool bLeftLED = false, bRightLED = false;
    bool bHighSpeed = JoyPush.buttons[6]; //LB Xbox = #6
    bool bLowSpeed = JoyPush.buttons[7];  //RB Xbox = #7
    float fLTbutton = JoyPush.axes[5];    //RT Xbox = #4

    if (JoyPush.axes[6] == 1.0)
        bLeftLED = true;
    else
        bLeftLED = false;
    if (JoyPush.axes[6] == -1.0)
        bRightLED = true;
    else
        bRightLED = false;

    //Publish message
    if ((bLeftLED == true) || (bRightLED == true))
    {
        ROS_INFO_STREAM("setLED");
        joyControl::DigitLEDControl DigitControl;
        DigitControl.left_dig_led = bLeftLED;
        DigitControl.right_dig_led = bRightLED;
        DigitControl.left_run_time = 0.5;
        DigitControl.right_run_time = 0.5;
        set_digit_pub.publish(DigitControl);
    }
}
