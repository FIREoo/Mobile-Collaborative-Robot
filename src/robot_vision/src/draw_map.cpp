#include "geometry_msgs/PointStamped.h"
#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <robot_vision/YoloBoundingBox.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class PoseDrawer
{
  public:
    PoseDrawer() :
        tf2_(buffer_), target_frame_("TM_robot/base"),
        tf2_filter_(point_sub_, buffer_, target_frame_, 10, 0)
    {
        ROS_INFO("Go");
        point_sub_.subscribe(n_, "turtle_point_stamped", 10);
        tf2_filter_.registerCallback(boost::bind(&PoseDrawer::msgCallback, this, _1));
    }

    //  Callback to register with tf2_ros::MessageFilter to be called when transforms are available
    void msgCallback(const geometry_msgs::PointStampedConstPtr &point_ptr)
    {
        geometry_msgs::PointStamped point_out;
        try
        {
            buffer_.transform(*point_ptr, point_out, target_frame_);

            ROS_INFO("point of turtle 3 in frame of turtle 1 Position(x:%f y:%f z:%f)\n",
                     point_out.point.x,
                     point_out.point.y,
                     point_out.point.z);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Failure %s\n", ex.what());//Print exception which was caught
        }
    }

  private:
    std::string target_frame_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener tf2_;
    ros::NodeHandle n_;
    message_filters::Subscriber<geometry_msgs::PointStamped> point_sub_;
    tf2_ros::MessageFilter<geometry_msgs::PointStamped> tf2_filter_;
};



image_transport::Publisher pub_map;
cv::Mat map;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "draw_map_node");
    ros::NodeHandle nh;

    // map = Mat(384, 512, CV_8UC3, cv::Scalar(0, 0, 0));

    // // ImageTransport
    // image_transport::ImageTransport it(nh);
    // pub_map = it.advertise("/map", 1);


    // ros::Rate loop_rate(10);
    // while (ros::ok())
    // {



    //     sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", map).toImageMsg();
    //     pub_map.publish(msg);
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }


    PoseDrawer pd;//Construct class
    ros::spin();  // Run until interupted
}