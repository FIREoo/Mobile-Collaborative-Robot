// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>

// std header
#include <cstdlib>
#include <sstream>

// TM Driver header
#include "tm_msgs/SendScript.h"
#include "tm_msgs/SetPositions.h"

/* Declaration of Local Functions --------------------------------------------*/
void GetJoyPush(const sensor_msgs::Joy &JoyPush);
void sendCMD(std::string cmd);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_tm_node");
    //Subscrible
    ros::Subscriber joy_sub;
    ros::NodeHandle nJoyCommand;
    joy_sub = nJoyCommand.subscribe("/joy", 10, GetJoyPush);
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

bool Vline = false;
void GetJoyPush(const sensor_msgs::Joy &JoyPush)
{
    bool button_a = JoyPush.buttons[0];
    bool button_x = JoyPush.buttons[2];
    bool button_b = JoyPush.buttons[1];
    bool button_y = JoyPush.buttons[3];
    bool button_L = JoyPush.buttons[9];
    bool button_R = JoyPush.buttons[10];
    bool button_LB = JoyPush.buttons[4];
    bool button_RB = JoyPush.buttons[5];
    // bool button_up = false;
    // bool button_down = false;
    // if (JoyPush.axes[7] == 1)
    // 	button_up = true;
    // else if (JoyPush.axes[7] == 1)
    // 	button_down = true;

    std::string cmd = "";

    if (button_LB == true)//LB
    {

        sendCMD("StopAndClearBuffer(0)");
    }
    // else if (button_LB == false)
    // {	//release button
    // 	//Do nothing
    // }
    else
    {
        if (button_a == true)
        {
            sendCMD("ScriptExit()");
        }
        else if (button_b == true)
        {
        }
        else if (button_x == true)
        {
        }
        else if (button_y == true)
        {
        }
        else if (button_L == true)
        {
            sendCMD("PTP(\"JPP\",0,0,90,0,90,0,70,200,0,false)");
        }

        //-----Vmode-----

        if (button_RB == true)
        {
            if (Vline == false)
            {
                Vline = true;
                sendCMD("ContinueVLine(2000,10000)");
            }
            float Y = JoyPush.axes[0] * 0.05;
            float X = JoyPush.axes[1] * 0.05;
            float Z = JoyPush.axes[7] * 0.05;
            sendCMD("SetContinueVLine(" + std::to_string(X) + "," + std::to_string(Y) + "," + std::to_string(Z) + ",0,0,0)");
        }
        else if (button_RB == false)
        {
            if (Vline == true)
            {
                Vline = false;
                sendCMD("StopContinueVmode");
            }
            Vline = false;
        }
    }
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
