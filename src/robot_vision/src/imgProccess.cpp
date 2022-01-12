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

#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

ros::Publisher pub_trackPinkPoint;
ros::Publisher pub_trackBluePoint;
ros::Publisher pub_trackYellowPoint;
ros::Publisher pub_trackRedPoint;

image_transport::Publisher pub_mixMask;
image_transport::Subscriber sub;

/*Point Cloud*/
struct PointCloud
{
    bool found;
    float x;
    float y;
    float z;
    uint8_t r;
    uint8_t g;
    uint8_t b;
};
std::vector<uint8_t> latestData;

/**@get x y z r g b from PointCloud2 msg
   * @param data msg->data
   * @param point_step msg->point_step 
   * @param index index of point(size = msg->width)*/
PointCloud getPointCloud2msg(std::vector<uint8_t> data, int point_step, int index)
{
    PointCloud pc;
    //if get 0, 0, -64, 127 => nan

    //x
    int offset = 0;
    char ptrc[4];
    ptrc[0] = data[index * point_step + 0];
    ptrc[1] = data[index * point_step + 1];
    ptrc[2] = data[index * point_step + 2];
    ptrc[3] = data[index * point_step + 3];
    if (ptrc[0] == 0 && ptrc[1] == 0 && ptrc[2] == -64 && ptrc[3] == 127)//check data
    {
        // ROS_INFO("no depth");
        return PointCloud { false, 0.0f, 0.0f, 0.0f, (uint8_t)-1, (uint8_t)-1, (uint8_t)-1 };
    }

    memccpy(&pc.x, &ptrc, index * point_step + offset, sizeof(float));

    offset = 4;//y
    ptrc[0] = data[index * point_step + offset + 0];
    ptrc[1] = data[index * point_step + offset + 1];
    ptrc[2] = data[index * point_step + offset + 2];
    ptrc[3] = data[index * point_step + offset + 3];
    // ROS_INFO_STREAM("y:" + std::to_string(ptrc[0]) + ", " + std::to_string(ptrc[1]) + ", " + std::to_string(ptrc[2]) + ", " + std::to_string(ptrc[3]));
    memccpy(&pc.y, &ptrc, index * point_step + offset, sizeof(float));

    offset = 8;//z
    ptrc[0] = data[index * point_step + offset + 0];
    ptrc[1] = data[index * point_step + offset + 1];
    ptrc[2] = data[index * point_step + offset + 2];
    ptrc[3] = data[index * point_step + offset + 3];
    memccpy(&pc.z, &ptrc, index * point_step + offset, sizeof(float));


    offset = 16;
    pc.r = data[index * point_step + offset + 0];
    pc.g = data[index * point_step + offset + 1];
    pc.b = data[index * point_step + offset + 2];

    pc.found = true;
    return pc;
}

PointCloud _getLatestPC(int col, int row)
{
    // ROS_INFO("find %d,%d", col, row);

    //I want to save the latest point cloud data and read it here
    //but saving the latest "true" data will need to deal with heavy-load processing
    //although the "getPointCloud()" will be more stable, I decide to return false data.
    //And the publish function will deal with it.

    if (latestData.size() == 0)
    {
        ROS_WARN("latestData.size() == 0");
        return PointCloud { false, 0.0f, 0.0f, 0.0f, (uint8_t)-1, (uint8_t)-1, (uint8_t)-1 };
    }
    //1536*2048
    if (col < 0 || col >= 2048 || row < 0 || row >= 1536)
    {
        ROS_ERROR("_getLatestPC() error col or row");
        throw std::runtime_error("fail index(col or row)");
    }

    int index = (row * 2048) + col;
    return getPointCloud2msg(latestData, 32, index);
}

void pointCloud2_Callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    /*save to leatest data*/
    latestData = std::vector<uint8_t>(msg->data);
    // latestData.swap(msg->data);
    // latestData = msg->data;
}

/*YOLO*/
void yolo_Callback(const robot_vision::YoloBoundingBoxConstPtr &msg)
{
    std::string name = std::string(msg->object_name);
    int x = uint16_t(msg->x);
    int y = uint16_t(msg->y);
    int width = uint16_t(msg->width);
    int height = uint16_t(msg->height);
    // ROS_INFO("(%d,%d,%d,%d)", x, y, width, height);
    PointCloud center_pc = _getLatestPC(x + width / 2, y + height / 2);
    // ROS_INFO("hand(%.3lf,%.3lf,%.3lf)", center_pc.x, center_pc.y, center_pc.z);
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(center_pc.x, center_pc.y, center_pc.z));
    tf::Quaternion q(0, 0, 0, 1);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "kinect", "yolo/hand"));
}

/*Image*/
cv::Point getColorCenter(cv::InputArray hsvSrc, cv::Scalar lower, cv::Scalar higher, cv::InputArray bgrSrc, cv::OutputArray mixMask)
{
    cv::Mat maskImg;
    cv::inRange(hsvSrc, lower, higher, maskImg);
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::erode(maskImg, maskImg, element, cv::Point(-1, -1), 1);
    cv::dilate(maskImg, maskImg, element, cv::Point(-1, -1), 2);

    cv::Moments m = cv::moments(maskImg, true);
    cv::Point point(m.m10 / m.m00, m.m01 / m.m00);
    mixMask.create(hsvSrc.size(), hsvSrc.type());
    cv::Mat rtn_mix = mixMask.getMat();
    cv::Mat maskImg_ch3;
    cv::cvtColor(maskImg, maskImg_ch3, CV_GRAY2BGR);
    cv::bitwise_and(bgrSrc, maskImg_ch3, rtn_mix);
    return point;
}
void publishPose(ros::Publisher pub, cv::Point p, std::string tf_child, bool send_unknow_depth)
{
    double x = p.x;
    double y = p.y;
    double z = 0;

    if (x > 0 && y > 0)
    {
        geometry_msgs::Pose msg_pose;
        PointCloud pc = _getLatestPC(x, y);
        if (pc.found == true)
        {
            x = pc.x;
            y = pc.y;
            z = pc.z;
            // ROS_INFO("get %lf,%lf,%lf", x, y, z);
        }
        else
        {
            if (send_unknow_depth == false)
                return;
            //publish with Depth 0.77M
            double fx = 978.451;
            double fy = 978.683;
            double cx = 1023.29;
            double cy = 779.833;
            x -= cx;
            y -= cy;
            //pixel*(D/f) = mm
            z = 770;
            x *= z / fx;
            y *= z / fy;
            x /= 1000;// mm to M
            y /= 1000;// mm to M
            z /= 1000;// mm to M
        }

        //boadcast pose
        msg_pose.position.x = x;
        msg_pose.position.y = y;
        msg_pose.position.z = z;

        msg_pose.orientation.x = 0;
        msg_pose.orientation.y = 0;
        msg_pose.orientation.z = 0;
        msg_pose.orientation.w = 1;
        pub.publish(msg_pose);

        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(msg_pose.position.x, msg_pose.position.y, msg_pose.position.z));
        tf::Quaternion q(msg_pose.orientation.x, msg_pose.orientation.y, msg_pose.orientation.z, msg_pose.orientation.w);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "kinect", tf_child));
    }
}
void img_callback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        // cv::Mat getImg = cv_bridge::toCvShare(msg, "16UC1")->image;//depth
        cv::Mat getImg = cv_bridge::toCvShare(msg, "bgr8")->image;

        //camera calibration//
        // cv::Mat cameraMatrix = (cv::Mat_<float>(3, 3) << 978.451, 0.0, 1023.29, 0.0, 978.683, 779.833, 0.0, 0.0, 1.0);
        // cv::Mat distCoeffs = (cv::Mat_<float>(8, 1) << 0.362504, -2.49193, 0.000118025, -0.000162683, 1.44097, 0.241069, -2.30814, 1.36445);
        // cv::undistort(getImg, getImg, cameraMatrix, distCoeffs);

        //get tracker mask
        cv::Mat hsvImg;
        cv::cvtColor(getImg, hsvImg, CV_BGR2HSV);
        //work area
        int Ax = 540;
        int Ay = 390;
        int Ab = 1460;
        int Ar = 1310;
        cv::Rect workArea(Ax, Ay, Ab - Ax, Ar - Ay);
        cv::Mat workMaskImg(getImg.rows, getImg.cols, CV_8UC3);
        cv::rectangle(workMaskImg, workArea, cv::Scalar(255, 255, 255), -1);
        cv::bitwise_and(hsvImg, workMaskImg, hsvImg);

        bool send_unknow_depth = false;
        cv::Mat totalMixMask(getImg.rows, getImg.cols, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::Mat tmpMixMask;

        // gripper Pink
        cv::Point point_gripper =
            getColorCenter(hsvImg, cv::Scalar(150, 40, 165), cv::Scalar(175, 155, 256), getImg, tmpMixMask);
        cv::bitwise_or(tmpMixMask, totalMixMask, totalMixMask);
        publishPose(pub_trackPinkPoint, point_gripper, "/track/pink", send_unknow_depth);

        // track Blue
        cv::Point point_track_blue =
            getColorCenter(hsvImg, cv::Scalar(95, 160, 135), cv::Scalar(110, 256, 256), getImg, tmpMixMask);
        cv::bitwise_or(tmpMixMask, totalMixMask, totalMixMask);
        publishPose(pub_trackBluePoint, point_track_blue, "/track/blue", send_unknow_depth);

        // track Yellow
        cv::Point point_track_yellow =
            getColorCenter(hsvImg, cv::Scalar(22, 110, 210), cv::Scalar(30, 204, 256), getImg, tmpMixMask);
        cv::bitwise_or(tmpMixMask, totalMixMask, totalMixMask);
        publishPose(pub_trackYellowPoint, point_track_yellow, "/track/yellow", send_unknow_depth);

        // track Red
        cv::Point point_track_red =
            getColorCenter(hsvImg, cv::Scalar(174, 115, 225), cv::Scalar(177, 160, 256), getImg, tmpMixMask);
        cv::bitwise_or(tmpMixMask, totalMixMask, totalMixMask);
        publishPose(pub_trackRedPoint, point_track_red, "/track/red", send_unknow_depth);

        //publish mix image
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", totalMixMask).toImageMsg();
        pub_mixMask.publish(msg);

        // cv::imshow("MixMask", totalMixMask);
        cv::waitKey(30);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_process_node");
    ros::NodeHandle n;
    // cv::namedWindow("MixMask");
    // cv::namedWindow("RGB");

    // trackPoint publisher
    pub_trackBluePoint = n.advertise<geometry_msgs::Pose>("img_process/track/blue/pose", 1);
    pub_trackYellowPoint = n.advertise<geometry_msgs::Pose>("img_process/track/yellow/pose", 1);
    pub_trackRedPoint = n.advertise<geometry_msgs::Pose>("img_process/track/red/pose", 1);
    pub_trackPinkPoint = n.advertise<geometry_msgs::Pose>("img_process/track/pink/pose", 1);

    // ImageTransport
    image_transport::ImageTransport it(n);
    sub = it.subscribe("/rgb/image_raw", 1, img_callback);
    pub_mixMask = it.advertise("img_process/image/mixMask", 1);// camera/image

    /*PointCloud2*/
    ros::Subscriber sub_pc = n.subscribe("points2", 1, pointCloud2_Callback);

    /*YOLO*/
    ros::Subscriber sub_yolo = n.subscribe("yolo/hand/box", 1, yolo_Callback);

    ros::spin();
    // cv::destroyWindow("MixMask");
    // cv::destroyWindow("RGB");
}