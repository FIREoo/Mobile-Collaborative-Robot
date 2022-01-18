#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "string.h"
#include "ui_mainwindow.h"
#include <QAbstractItemView>
#include <QMainWindow>
#include <QStandardItemModel>
#include <QStringList>
#include <QStringListModel>
#include <QtWidgets>
#include <map>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tm_msgs/FeedbackState.h"
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/image_encodings.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
// TM, NewEra
#include "tm_msgs/FeedbackState.h"
#include "tm_msgs/SendScript.h"
#include "tm_msgs/SetIO.h"
#include "tm_msgs/SetPositions.h"
#include <robot_vision/PoseTrans.h>//service
#include <ui/DigitLEDControl.h>

// #include "myThreadHandle.h"
#include "tmrobot.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


using std::string;

namespace Ui
{
    class MainWindow;
}

class Flags
{
  public:
    Flags();
    bool tf_calibration_upperCam1;
    bool tf_calibration_upperCam2;
    bool tf_m03Robot_inVision;
    bool tf_loop_set_tm;
    bool saveImage1;
    bool saveImage2;
    bool saveImage3;
    bool saveImage4;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

  public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    ros::NodeHandle nh;

    ros::Publisher NewEra_speed_pub;

    //callback function
    void imgCallback(const sensor_msgs::ImageConstPtr &msg);
    void img2Callback(const sensor_msgs::ImageConstPtr &msg);
    void img3Callback(const sensor_msgs::ImageConstPtr &msg);
    void img4Callback(const sensor_msgs::ImageConstPtr &msg);
    void TM_pos_Callback(const tm_msgs::FeedbackState::ConstPtr &msg);
    void getTrackerCallback(const geometry_msgs::PoseConstPtr &msg);

    void TM_servoOn(const string trackName, const double offset_x, const double offset_y);
    void thread_TM_servoOn(const string trackName);
    void TM_newServoOn();

    /**@brief continue publish tracker with target
   * @param target_fram the tf_fram_id of the target
   * @param x_offset offset axis x in Meters 
   * @param y_offset offset axis y in Meters*/
    void publishTracker(const std::string target_fram, const double x_offset, const double y_offset);
    void publishTmBase();

    void publishTracker_msgFilter();
    cv::Point2f getTrackPoint_yellow();
    cv::Point2f getTrackPoint_blue();

    int servo_track_offset_x = 0;// mm
    int servo_track_offset_y = 0;// mm

    TmRobot tm;
    // double tm_joint[6];
    // double tm_tcp_pos[6];
    // double tm_tcp_speed[6];
    // double tm_tcp_speed_x = 0;
    // double tm_tcp_speed_y = 0;
    // double tm_tcp_speed_z = 0;
    void waitTM(double delaySec);


    std::map<string, ros::Subscriber> map_sub_topic;//@subscriber map

  private slots:
    // void on_pushButton_clicked();

    void Test(int value);

    void on_btn_get_tm_pos_clicked();// get TM tcp pos

    void on_pushButton_voice_cmd_clicked();// testing voice cmd
    void on_pushButton_voice_cmd_stop_clicked();

    void on_pushButton_NE_light_clicked();
    void on_pushButton_NE_moveLittle_clicked();
    void on_pushButton_NE_break_clicked();
    void on_pushButton_NE_releaseBreak_clicked();


    void on_pushButton_NE_moveF_clicked();
    void on_pushButton_NE_moveB_clicked();



    void sendCMD(std::string cmd, bool print = true);
    void sendGrip(bool cmd);

    void on_btn_set_tm_pos_1_clicked();
    void on_btn_set_tm_pos_2_clicked();
    void on_btn_set_tm_pos_3_clicked();
    void on_btn_set_tm_pos_4_clicked();
    void on_btn_set_tm_pos_5_clicked();
    void on_btn_set_tm_pos_6_clicked();
    void on_btn_set_tm_pos_7_clicked();
    void on_btn_set_tm_pos_8_clicked();

    void on_btn_set_tm_grip_clicked();
    void on_btn_set_tm_unGrip_clicked();
    void on_btn_set_tm_pos_setTF_clicked();

    void on_btn_set_tm_servo_trackPose_clicked();
    void on_pushButton_creatActionBase_clicked();
    void on_btn_tm_execute_nlp_clicked();
    void on_btn_set_tm_servo_stop_clicked();
    void on_btn_set_tf_tracker_clicked();

    /*save image*/
    void on_btn_Image_1_save_clicked();
    void on_btn_Image_2_save_clicked();
    void on_btn_Image_save_upperCamera1_clicked();
    void on_btn_Image_save_upperCamera2_clicked();

    void on_btn_set_tm_pos_preTrack_clicked();
    void on_pushButton_clearTmMsg_clicked();

    /*get image topic*/
    void on_btn_getImage_clicked();//subscribe Kinect image
    void on_btn_get_tm_img_clicked();
    void on_btn_getImage_2_clicked();
    void on_btn_getImage_upperCamera1_clicked();
    void on_btn_getImage_upperCamera2_clicked();

    /*stop image*/
    void on_btn_stopImage_clicked();
    void on_btn_stopImage_2_clicked();
    void on_btn_stopImage_upperCamera1_clicked();
    void on_btn_stopImage_upperCamera2_clicked();

    /*set tf2 transform*/
    void on_btn_set_tf2_tm_base_clicked();
    void on_btn_set_tf2_upper_camera2_clicked();
    void on_btn_set_tf2_upper_camera1_clicked();

    /*test function*/
    void on_pushButton_ros_spin_clicked();
    void on_pushButton_test1_clicked();





    void on_pushButton_clear_execute_clicked();


    void on_pushButton_NE_get_pose_clicked();

    void on_pushButton_NE_goto_A_clicked();

    void on_pushButton_NE_goto_WS_clicked();

    void on_btn_set_tm_joint_clicked();

    void on_btn_set_tm_copy_pose_clicked();

    void on_btn_set_tm_copy_joint_clicked();

    void on_btn_set_tm_pose_clicked();

    void on_comboBox_hack_voice_cmd_currentIndexChanged(int index);

    void on_btn_getImage_tm_cam_clicked();

    void on_pushButton_reload_image_topic_clicked();

    void on_checkBox_real_move_stateChanged(int arg1);

    void on_pushButton_tf_br_NE_clicked();

  private:
    Ui::MainWindow *ui;
    QStandardItemModel *nlp_itemModel = new QStandardItemModel();
    QStringListModel *msg_listModel = new QStringListModel();
    QStringListModel *msg_tm_listModel = new QStringListModel();

    // save image flag
    Flags FLAG;

    int tf_correction_check;

    std::string Action = "";
    std::string InteractObject = "";
    std::string Destination = "";
    std::string GrippingObject = "";

    bool servoing = false;

    // UI
    QString setStringColor(QString qstr, QString color);
    void addItem(QStandardItemModel *m, QString str1, QString str2, QString str3);
    void clearItem(QStandardItemModel *m);
    void addItem(QStringListModel *theModel, QString str);
    void addItem(QStringListModel *theModel, QString str, QBrush brush);
    void clearItem(QStringListModel *m);


    // NLP
    int find_text(std::string text, std::string find_text);
    std::string replace_text(std::string str, std::string find, std::string replace);

    // tm
    void tm_grip_object(std::string obj);
    void tm_place_object(std::string obj);
    TmJoint tmj_outHome;
    TmJoint tmj_front;
    bool realMove;

    /*global tf2 listener*/
    tf2_ros::Buffer tfBuffer;             //global tf buffer
    tf2_ros::TransformListener tfListener;//global tf listener
    geometry_msgs::TransformStamped lookupTransform(const std::string target_frame, const std::string source_frame);
    bool checkTransform(const std::string target_frame, const std::string source_frame, geometry_msgs::TransformStamped check_value);
    void matchCalibration(std::string frame_calibration_target, std::string frame_matched_target, std::string frame_matched_marker, std::string frame_matching_source, std::string frame_matching_marker);
};



#endif// MAINWINDOW_H
