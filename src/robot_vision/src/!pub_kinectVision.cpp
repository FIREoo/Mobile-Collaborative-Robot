#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);//camera/image
    cv::VideoCapture cap(2);
    // cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
    cv::waitKey(30);

    ros::Rate loop_rate(10);

    if (!cap.isOpened())
    {
        ROS_INFO("Cannot open camera\n");
        return 1;
    }
    cv::Mat frame;
    while (nh.ok())
    {
        bool ret = cap.read(frame);// or cap >> frame;
        if (!ret)
        {
            ROS_INFO("Can't receive frame (stream end?). Exiting ...\n");
            break;
        }
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}