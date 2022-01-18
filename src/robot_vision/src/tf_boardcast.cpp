#include "tm_msgs/FeedbackState.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <robot_vision/PoseTrans.h>//service
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

// set transform //Abandoned
tf::Transform trans_kinect2TMbase;
bool setTrans(robot_vision::PoseTrans::Request &req, robot_vision::PoseTrans::Response &res)
{
    trans_kinect2TMbase.setOrigin(tf::Vector3(req.x, req.y, req.z));

    tf::Quaternion q;
    q.setRPY(0, M_PI, M_PI_4);//Order : ZYX
    trans_kinect2TMbase.setRotation(q);

    ROS_INFO("set /kinect to /TM_robot/base TF: x=%lf, y=%lf, z=%lf", (double)req.x, (double)req.y, (double)req.z);
    res.ok = true;
    return true;
}

void setStaticTF2()
{
    static tf2_ros::StaticTransformBroadcaster static_broadcaster1;
    geometry_msgs::TransformStamped static_transformStamped_kinect;
    static_transformStamped_kinect.header.stamp = ros::Time::now();
    static_transformStamped_kinect.header.frame_id = "map";
    static_transformStamped_kinect.child_frame_id = "camera_base";
    static_transformStamped_kinect.transform.translation.x = 0;
    static_transformStamped_kinect.transform.translation.y = 0;
    static_transformStamped_kinect.transform.translation.z = 0;
    static_transformStamped_kinect.transform.rotation.x = 0;
    static_transformStamped_kinect.transform.rotation.y = 0;
    static_transformStamped_kinect.transform.rotation.z = 0;
    static_transformStamped_kinect.transform.rotation.w = 1;
    static_broadcaster1.sendTransform(static_transformStamped_kinect);


    static tf2_ros::StaticTransformBroadcaster static_broadcaster2;
    geometry_msgs::TransformStamped static_transformStamped_tm;
    static_transformStamped_tm.header.stamp = ros::Time::now();
    static_transformStamped_tm.header.frame_id = "world";
    static_transformStamped_tm.child_frame_id = "kinect";
    static_transformStamped_tm.transform.translation.x = 0;
    static_transformStamped_tm.transform.translation.y = 0;
    static_transformStamped_tm.transform.translation.z = 0;
    static_transformStamped_tm.transform.rotation.x = 0;
    static_transformStamped_tm.transform.rotation.y = 0;
    static_transformStamped_tm.transform.rotation.z = 0;
    static_transformStamped_tm.transform.rotation.w = 1;
    static_broadcaster2.sendTransform(static_transformStamped_tm);

    static tf2_ros::StaticTransformBroadcaster static_broadcaster3;
    geometry_msgs::TransformStamped static_transformStamped;
    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "TM_robot/tcp";
    static_transformStamped.child_frame_id = "TM_robot/tcp/aruco";
    static_transformStamped.transform.translation.x = 0;
    static_transformStamped.transform.translation.y = 0;
    static_transformStamped.transform.translation.z = 0.1;
    tf2::Quaternion quat;
    quat.setRPY(M_PI_2, 0, M_PI);
    static_transformStamped.transform.rotation.x = quat.x();
    static_transformStamped.transform.rotation.y = quat.y();
    static_transformStamped.transform.rotation.z = quat.z();
    static_transformStamped.transform.rotation.w = quat.w();
    static_broadcaster3.sendTransform(static_transformStamped);
}

void fakeTM()
{
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "TM_robot/base";
    transformStamped.child_frame_id = "TM_robot/tcp";
    transformStamped.transform.translation.x = -0.50;
    transformStamped.transform.translation.y = -0.30;
    transformStamped.transform.translation.z = 0.30;
    tf2::Quaternion quat;
    quat.setRPY(M_PI_2, 0, -M_PI_4);
    transformStamped.transform.rotation.x = quat.x();
    transformStamped.transform.rotation.y = quat.y();
    transformStamped.transform.rotation.z = quat.z();
    transformStamped.transform.rotation.w = quat.w();
    static_broadcaster.sendTransform(transformStamped);
}

int main(int argc, char **argv)
{
    // tf world is
    ros::init(argc, argv, "tf_broadcaster");
    ros::NodeHandle node;

    ros::ServiceServer service = node.advertiseService("kinect_transform", setTrans);
    static tf::TransformBroadcaster br;

    setStaticTF2();
    fakeTM();

    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "TM_robot/base";
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = 0;
    transformStamped.transform.translation.z = 0;
    transformStamped.transform.rotation.x = 0;
    transformStamped.transform.rotation.y = 0;
    transformStamped.transform.rotation.z = 0;
    transformStamped.transform.rotation.w = 1;
    static_broadcaster.sendTransform(transformStamped);

    static tf2_ros::StaticTransformBroadcaster static_broadcaster2;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "upper_camera1";
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = 0;
    transformStamped.transform.translation.z = 0;
    transformStamped.transform.rotation.x = 0;
    transformStamped.transform.rotation.y = 0;
    transformStamped.transform.rotation.z = 0;
    transformStamped.transform.rotation.w = 1;
    static_broadcaster2.sendTransform(transformStamped);

    static tf2_ros::StaticTransformBroadcaster static_broadcaster3;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "upper_camera2";
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = 0;
    transformStamped.transform.translation.z = 0;
    transformStamped.transform.rotation.x = 0;
    transformStamped.transform.rotation.y = 0;
    transformStamped.transform.rotation.z = 0;
    transformStamped.transform.rotation.w = 1;
    static_broadcaster3.sendTransform(transformStamped);
    ros::spin();
    // ros::Rate rate(10.0);
    // while (node.ok())
    // {
    //     // br.sendTransform(tf::StampedTransform(trans_kinect2TMbase, ros::Time::now(), "/kinect", "/TM_robot/base"));
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    return 0;
};