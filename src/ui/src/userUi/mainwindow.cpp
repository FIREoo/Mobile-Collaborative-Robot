#pragma region include
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
// ros
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/image_encodings.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
// TM, NewEra
#include "tm_msgs/FeedbackState.h"
#include "tm_msgs/SendScript.h"
#include "tm_msgs/SetIO.h"
#include "tm_msgs/SetPositions.h"
#include <robot_vision/PoseTrans.h>//service
#include <ui/DigitLEDControl.h>
// opencv
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// mine
#include "myThreadHandle.h"
#include "subclass.h"
#include <robot_vision/RotationTools.h>


using std::string;
#pragma endregion ___include___


//---ThreadHandle---\\.
#pragma region ThreadHandle
template<class M, class T>
SubscribeThreadHandle::SubscribeThreadHandle(std::string topic, uint32_t queue_size, void (T::*fp)(const boost::shared_ptr<M const> &), T *obj)
{
    _topic = topic;
    _obj = obj;

    ops.template init<M>(topic, queue_size, boost::bind(fp, obj, boost::placeholders::_1));
    ops.transport_hints = ros::TransportHints();// const TransportHints& transport_hints = TransportHints()
}
SubscribeThreadHandle::SubscribeThreadHandle(void (MainWindow::*pFunc)(), MainWindow *obj)
{
    _pFunc = pFunc;
    _obj = obj;
}
SubscribeThreadHandle::SubscribeThreadHandle(void (MainWindow::*pFunc)(double, double), double v0, double v1, MainWindow *obj)
{
    _pFunc2 = pFunc;
    _obj = obj;
    V0 = v0;
    V1 = v1;
}
SubscribeThreadHandle::SubscribeThreadHandle(void (MainWindow::*pFunc)(std::string, double, double), std::string s0, double v0, double v1, MainWindow *obj)
{
    _pFunc3 = pFunc;
    _obj = obj;
    S0 = s0;
    V0 = v0;
    V1 = v1;
}
SubscribeThreadHandle::~SubscribeThreadHandle()
{
    ROS_INFO("shutdown!! subscriber");
}

void SubscribeThreadHandle::subscribe()
{
    ros::AsyncSpinner spinner(0);
    spinner.start();

    auto iter = _obj->map_sub_topic.find(ops.topic);//@subscriber map
    if (iter != _obj->map_sub_topic.end())
    {
        iter->second = _obj->nh.subscribe(ops);
        ROS_INFO_STREAM("subscribe: " + ops.topic);
    }
    else
    {
        ROS_WARN_STREAM("Cant find topic: " + ops.topic);
    }
    ros::waitForShutdown();
}
void SubscribeThreadHandle::doFunction()
{
    (_obj->*_pFunc)();
}
void SubscribeThreadHandle::doFunction2()
{
    (_obj->*_pFunc2)(V0, V1);
}
void SubscribeThreadHandle::doFunction3()
{
    (_obj->*_pFunc3)(S0, V0, V1);
}

#pragma endregion ___ThreadHandle___

//---Main and subfunction---\\.
#pragma region Main and subfunction
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    tfListener(tfBuffer, true, ros::TransportHints())
{
    ui->setupUi(this);
    tf_correction_check = 0;

    Action = "";
    InteractObject = "";
    Destination = "";

    ui->listView_execute_msg->setModel(msg_listModel);
    ui->listView_tm_msg->setModel(msg_tm_listModel);

    nlp_itemModel->setColumnCount(3);
    nlp_itemModel->setHeaderData(0, Qt::Horizontal, QString::fromLocal8Bit("Action"));
    nlp_itemModel->setHeaderData(1, Qt::Horizontal, QString::fromLocal8Bit("Target"));
    nlp_itemModel->setHeaderData(2, Qt::Horizontal, QString::fromLocal8Bit("Destination"));
    ui->tableView_ab->setModel(nlp_itemModel);
    // Alignment => Left
    ui->tableView_ab->horizontalHeader()->setDefaultAlignment(Qt::AlignCenter);
    for (int i = 0; i < nlp_itemModel->rowCount(); i++)
        ui->tableView_ab->setColumnWidth(i, 110);

    NewEra_speed_pub = nh.advertise<const geometry_msgs::Twist>("/joystick_set_velocity", 10);

    //@subscriber map
    //mapStudent.insert(pair<string, string>("r000", "student_zero"));
    map_sub_topic["/rgb/image_raw"] = ros::Subscriber();
    map_sub_topic["/img_process/image/mixMask"] = ros::Subscriber();
    map_sub_topic["/feedback_states"] = ros::Subscriber();
    map_sub_topic["/track/tracker/pose"] = ros::Subscriber();
    map_sub_topic["/upper_camera1/image"] = ros::Subscriber();
    map_sub_topic["/upper_camera2/image"] = ros::Subscriber();
}
MainWindow::~MainWindow()
{
    nh.shutdown();
    ros::shutdown();
    delete ui;
}

/*ui function*/
void MainWindow::on_btn_Image_1_save_clicked()
{
    FLAG.saveImage1 = true;
}
void MainWindow::on_btn_Image_2_save_clicked()
{
    FLAG.saveImage2 = true;
}
void MainWindow::on_btn_Image_save_upperCamera1_clicked()
{
    FLAG.saveImage3 = true;
}
void MainWindow::on_btn_Image_save_upperCamera2_clicked()
{
    FLAG.saveImage4 = true;
}

/*subfunction*/
QString MainWindow::setStringColor(QString qstr, QString color)
{
    QString c = "<span style='color:" + color + ";'>%1</span>";
    return QString(c).arg(qstr);
}
void MainWindow::addItem(QStringListModel *theModel, QString str)
{
    theModel->insertRow(theModel->rowCount());                       //在尾部插入一空行
    QModelIndex index = theModel->index(theModel->rowCount() - 1, 0);//獲取最後一行
    theModel->setData(index, str, Qt::DisplayRole);                  //設置顯示文字
}
void MainWindow::clearItem(QStringListModel *m)
{
    m->removeRows(0, m->rowCount());
}
void MainWindow::addItem(QStandardItemModel *m, QString str1, QString str2, QString str3)
{
    //---add Item 1---
    int row = m->rowCount();
    m->setItem(row, 0, new QStandardItem(str1));
    // Color
    //  m->item(row, 0)->setForeground(QBrush(QColor(255, 0, 0)));
    //---add Item 2---
    m->setItem(row, 1, new QStandardItem(str2));
    //---add Item 3---
    m->setItem(row, 2, new QStandardItem(str3));
}
void MainWindow::clearItem(QStandardItemModel *m)
{
    m->removeRows(0, m->rowCount());
}

/*TF*/
geometry_msgs::TransformStamped MainWindow::lookupTransform(const std::string target_frame, const std::string source_frame)
{
    while (nh.ok())
    {
        try
        {
            geometry_msgs::TransformStamped transformStamped;
            transformStamped = tfBuffer.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(0.5));
            // ROS_INFO("tf T(Meter)[%lf,%lf,%lf]", transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
            return transformStamped;
            break;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
    }
    return geometry_msgs::TransformStamped();
}
bool MainWindow::checkTransform(const std::string target_frame, const std::string source_frame, geometry_msgs::TransformStamped check_value)
{
    while (nh.ok())
    {
        try
        {
            geometry_msgs::TransformStamped transformStamped;
            transformStamped = tfBuffer.lookupTransform(target_frame, source_frame, ros::Time(0));
            if (check_value.transform == transformStamped.transform)
            {
                return true;
            }
            // ROS_INFO("tf T(Meter)[%lf,%lf,%lf]", transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
            return false;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(0.1).sleep();
            continue;
        }
    }
    return false;
}

/*Flags*/
Flags::Flags()
{
    tf_m03Robot_inVision = false;
    tf_calibration_upperCam1 = false;
    tf_calibration_upperCam2 = false;
    saveImage1 = false;
    saveImage2 = false;
    saveImage3 = false;
    saveImage4 = false;
}
#pragma endregion ___Main and subfunction___

//---Subscribe Message---\\.
#pragma region Subscribe Message
void MainWindow::on_btn_getImage_clicked()
{
    QThread *thread_img = new QThread();
    SubscribeThreadHandle *thHandle = new SubscribeThreadHandle("/rgb/image_raw", 10, &MainWindow::imgCallback, this);
    thHandle->moveToThread(thread_img);
    connect(thread_img, &QThread::started, thHandle, &SubscribeThreadHandle::subscribe);
    thread_img->start();
}
void MainWindow::imgCallback(const sensor_msgs::ImageConstPtr &msg)
{
    //2048x1536
    try
    {
        cv::Mat getImg = cv_bridge::toCvShare(msg, "bgr8")->image;
        if (FLAG.saveImage1 == true)
        {
            cv::imwrite("/home/fire/Desktop/NewEra_ws/BGR.png", getImg);
            FLAG.saveImage1 = false;
        }

        cv::Mat showImg;
        cv::cvtColor(getImg, getImg, cv::COLOR_BGR2RGB);
        cv::resize(getImg, showImg, cv::Size(512, 384));
        ui->img_main->setPixmap(QPixmap::fromImage(QImage(showImg.data, showImg.cols, showImg.rows, showImg.step, QImage::Format_RGB888)));
    }
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM("imgCallback: Error");
    }
}
void MainWindow::on_btn_stopImage_clicked()
{
    auto iter = map_sub_topic.find("/rgb/image_raw");//@subscriber map
    iter->second.shutdown();
    cv::Mat showImg(384, 512, CV_8UC3, cv::Scalar(0, 0, 0));
    ui->img_main->setPixmap(QPixmap::fromImage(QImage(showImg.data, showImg.cols, showImg.rows, showImg.step, QImage::Format_RGB888)));
}

void MainWindow::on_btn_getImage_2_clicked()
{
    QThread *thread = new QThread();
    SubscribeThreadHandle *thHandle = new SubscribeThreadHandle("/img_process/image/mixMask", 10, &MainWindow::img2Callback, this);
    thHandle->moveToThread(thread);
    connect(thread, &QThread::started, thHandle, &SubscribeThreadHandle::subscribe);
    thread->start();
}
void MainWindow::img2Callback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        cv::Mat getImg = cv_bridge::toCvShare(msg, "bgr8")->image;
        if (FLAG.saveImage2 == true)
        {
            cv::imwrite("/home/fire/Desktop/NewEra_ws/Depth2BGR.png", getImg);
            FLAG.saveImage2 = false;
        }

        cv::Mat showImg;
        cv::cvtColor(getImg, getImg, cv::COLOR_BGR2RGB);
        cv::resize(getImg, showImg, cv::Size(512, 384));
        ui->img_main_2->setPixmap(QPixmap::fromImage(QImage(showImg.data, showImg.cols, showImg.rows, showImg.step, QImage::Format_RGB888)));
    }
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM("imgCallback: Error");
    }
}
void MainWindow::on_btn_stopImage_2_clicked()
{
    auto iter = map_sub_topic.find("/img_process/image/mixMask");//@subscriber map
    iter->second.shutdown();
    cv::Mat showImg(384, 512, CV_8UC3, cv::Scalar(0, 0, 0));
    ui->img_main_2->setPixmap(QPixmap::fromImage(QImage(showImg.data, showImg.cols, showImg.rows, showImg.step, QImage::Format_RGB888)));
}

void MainWindow::on_btn_get_tm_pos_clicked()
{
    QThread *thread = new QThread();
    SubscribeThreadHandle *thHandle = new SubscribeThreadHandle("/feedback_states", 1000, &MainWindow::TM_pos_Callback, this);
    thHandle->moveToThread(thread);
    connect(thread, &QThread::started, thHandle, &SubscribeThreadHandle::subscribe);
    thread->start();
}
void MainWindow::TM_pos_Callback(const tm_msgs::FeedbackState::ConstPtr &msg)
{
    try
    {
        if (msg->joint_pos.size() == 6)
        {
            tm_joint[0] = msg->joint_pos[0];
            tm_joint[1] = msg->joint_pos[1];
            tm_joint[2] = msg->joint_pos[2];
            tm_joint[3] = msg->joint_pos[3];
            tm_joint[4] = msg->joint_pos[4];
            tm_joint[5] = msg->joint_pos[5];
            ui->tm_joint_j0->setText(QString::number(tm_joint[0] / M_PI * 180.0, 'd', 3));
            ui->tm_joint_j1->setText(QString::number(msg->joint_pos[1], 'd', 3));
            ui->tm_joint_j2->setText(QString::number(msg->joint_pos[2], 'd', 3));
            ui->tm_joint_j3->setText(QString::number(msg->joint_pos[3], 'd', 3));
            ui->tm_joint_j4->setText(QString::number(msg->joint_pos[4], 'd', 3));
            ui->tm_joint_j5->setText(QString::number(msg->joint_pos[5], 'd', 3));
        }
        if (msg->tool_pose.size() == 6)
        {
            tm_tcp_pos[0] = msg->tool_pose[0];
            tm_tcp_pos[1] = msg->tool_pose[1];
            tm_tcp_pos[2] = msg->tool_pose[2];
            tm_tcp_pos[3] = msg->tool_pose[3];
            tm_tcp_pos[4] = msg->tool_pose[4];
            tm_tcp_pos[5] = msg->tool_pose[5];

            ui->tm_pos_x->setText(QString::number(msg->tool_pose[0], 'd', 3));
            ui->tm_pos_y->setText(QString::number(msg->tool_pose[1], 'd', 3));
            ui->tm_pos_z->setText(QString::number(msg->tool_pose[2], 'd', 3));
            ui->tm_pos_Rx->setText(QString::number(msg->tool_pose[3], 'd', 3));
            ui->tm_pos_Ry->setText(QString::number(msg->tool_pose[4], 'd', 3));
            ui->tm_pos_Rz->setText(QString::number(msg->tool_pose[5], 'd', 3));
        }
        if (msg->tcp_speed.size() == 6)
        {
            tm_tcp_speed[0] = msg->tcp_speed[0];
            tm_tcp_speed[1] = msg->tcp_speed[1];
            tm_tcp_speed[2] = msg->tcp_speed[2];
            tm_tcp_speed_x = msg->tcp_speed[0];
            tm_tcp_speed_y = msg->tcp_speed[1];
            tm_tcp_speed_z = msg->tcp_speed[2];
        }
    }
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM("Error get TM pos callback");
    }
}


void MainWindow::on_btn_getImage_upperCamera1_clicked()
{
    QThread *thread_img = new QThread();
    SubscribeThreadHandle *thHandle = new SubscribeThreadHandle("/upper_camera1/image", 10, &MainWindow::img3Callback, this);
    thHandle->moveToThread(thread_img);
    connect(thread_img, &QThread::started, thHandle, &SubscribeThreadHandle::subscribe);
    thread_img->start();
}
void MainWindow::img3Callback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        cv::Mat getImg = cv_bridge::toCvShare(msg, "bgr8")->image;//1920*1080
        if (FLAG.saveImage3 == true)
        {
            cv::imwrite("/home/fire/Desktop/NewEra_ws/upper_camera_1.png", getImg);
            FLAG.saveImage3 = false;
        }

        cv::Mat showImg;
        cv::cvtColor(getImg, getImg, cv::COLOR_BGR2RGB);
        cv::resize(getImg, showImg, cv::Size(512, 288));//16:9
        cv::Mat concat(384 - 288, 512, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::vconcat(showImg, concat, showImg);//to 4:3
        ui->img_main_3->setPixmap(QPixmap::fromImage(QImage(showImg.data, showImg.cols, showImg.rows, showImg.step, QImage::Format_RGB888)));
    }
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM("imgCallback: Error");
    }
}
void MainWindow::on_btn_stopImage_upperCamera1_clicked()
{
    auto iter = map_sub_topic.find("/upper_camera1/image");//@subscriber map
    iter->second.shutdown();

    cv::Mat showImg(384, 512, CV_8UC3, cv::Scalar(0, 0, 0));
    ui->img_main_3->setPixmap(QPixmap::fromImage(QImage(showImg.data, showImg.cols, showImg.rows, showImg.step, QImage::Format_RGB888)));
}

void MainWindow::on_btn_getImage_upperCamera2_clicked()
{
    QThread *thread_img = new QThread();
    SubscribeThreadHandle *thHandle = new SubscribeThreadHandle("/upper_camera2/image", 10, &MainWindow::img4Callback, this);
    thHandle->moveToThread(thread_img);
    connect(thread_img, &QThread::started, thHandle, &SubscribeThreadHandle::subscribe);
    thread_img->start();
}
void MainWindow::img4Callback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        cv::Mat getImg = cv_bridge::toCvShare(msg, "bgr8")->image;//1920*1080
        if (FLAG.saveImage4 == true)
        {
            cv::imwrite("/home/fire/Desktop/NewEra_ws/upper_camera_2.png", getImg);
            FLAG.saveImage4 = false;
        }

        cv::Mat showImg;
        cv::cvtColor(getImg, getImg, cv::COLOR_BGR2RGB);
        cv::resize(getImg, showImg, cv::Size(512, 384));
        ui->img_main_4->setPixmap(QPixmap::fromImage(QImage(showImg.data, showImg.cols, showImg.rows, showImg.step, QImage::Format_RGB888)));
    }
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM("imgCallback: Error");
    }
}
void MainWindow::on_btn_stopImage_upperCamera2_clicked()
{
    auto iter = map_sub_topic.find("/upper_camera2/image");//@subscriber map
    iter->second.shutdown();

    cv::Mat showImg(384, 512, CV_8UC3, cv::Scalar(0, 0, 0));
    ui->img_main_4->setPixmap(QPixmap::fromImage(QImage(showImg.data, showImg.cols, showImg.rows, showImg.step, QImage::Format_RGB888)));
}


void MainWindow::getTrackerCallback(const geometry_msgs::PoseConstPtr &msg)
{

    double x = msg->position.x;
    double y = msg->position.y;
    double z = msg->position.z;

    double a = 0.2;
    double Vx = x + (a * x * x);
    double Vy = y + (a * y * y);

    // speed limit
    double limit = 0.05;// 0.1
    Vx = (Vx > limit) ? limit : Vx;
    Vx = (Vx < -limit) ? -limit : Vx;
    Vy = (Vy > limit) ? limit : Vy;
    Vy = (Vy < -limit) ? -limit : Vy;

    ROS_INFO_STREAM("Sent script to robot: SetContinueVLine(" + std::to_string(Vx) + "," + std::to_string(Vy) + ",0,0,0,0)");
    //ros::ServiceClient client = nh.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");
    // tm_msgs::SendScript srv;
    // srv.request.id = "demo";
    // srv.request.script = "SetContinueVLine(" + std::to_string(Vx) + "," + std::to_string(Vy) + ",0,0,0,0)";
    // if (client.call(srv))
    // {
    //     if (srv.response.ok)
    //         ROS_INFO_STREAM("Sent script to robot: SetContinueVLine(" + std::to_string(Vx) + "," + std::to_string(Vy) + ",0,0,0,0)");
    //     else
    //         ROS_WARN_STREAM("Sent script to robot , but response not yet ok ");
    // }
    // else
    // {
    //     ROS_ERROR_STREAM("Error send script to robot");
    // }
    // sendCMD("SetContinueVLine(" + std::to_string(Vx) + "," + std::to_string(Vy) + ",0,0,0,0)");
}

#pragma endregion ___Subscribe Message___

//---voice command---\\.
#pragma region voice command

void MainWindow::on_pushButton_voice_cmd_clicked()
{
    ROS_INFO("\033[0;94m=====Start NLP=====\033[0m");
    ui->textEdit_voice->setText(QString::fromStdString(""));
    qApp->processEvents();
    //**service**//
    ros::ServiceClient client = nh.serviceClient<std_srvs::SetBool>("record_service");
    std_srvs::SetBool srv;
    srv.request.data = true;
    if (client.call(srv))
    {
        if (srv.response.success)
            ROS_INFO("Start Record");
        else
            ROS_INFO("fail record");
    }
    else
    {
        ROS_ERROR("Failed to call service record_service");
    }
}
void MainWindow::on_pushButton_voice_cmd_stop_clicked()
{
    ros::ServiceClient client = nh.serviceClient<std_srvs::SetBool>("record_service");
    std_srvs::SetBool srv;
    srv.request.data = false;
    if (client.call(srv))
    {
        if (srv.response.success)
        {
            ROS_INFO("Stop record");
            string str = srv.response.message;
            ROS_INFO("v2t:%s", str.c_str());

            /*test correct*/
            std::map<string, string> text_correct = { { "一腳", "椅腳" }, { "已面", "椅面" }, { "一面", "椅面" }, { "放行", "方形" }, { "已被", "椅背" }, { "西沉", "吸塵" }, { "沙子", "砂紙" }, { "影片", "椅面" }, { "爸爸說", "把板手" } };
            for (const auto &s : text_correct)
            {
                str = replace_text(str, s.first, s.second);
            }
            ROS_INFO("final:%s", str.c_str());
            ui->textEdit_voice->setText(QString::fromStdString(str));
        }
        else
            ROS_INFO("fail stoping record");
    }
    else
    {
        ROS_ERROR("Failed to call service record_service");
    }
    ROS_INFO("\033[0;94m=====NLP Done=====\033[0m");

    on_pushButton_creatActionBase_clicked();
    /*

    把刷子拿給我
幫我用吸塵器
好
人：打磨
人：砂紙放桌上
幫我打磨這裡
把砂紙拿回去

    */
}

int MainWindow::find_text(string text, string find_text)
{
    string::size_type index;
    index = text.find(find_text);
    if (index != std::string::npos)
    {
        int position = index / 2;
        return index;
    }

    return -1;
}
std::string MainWindow::replace_text(std::string str, std::string find, std::string replace)
{
    // ROS_INFO("find and replace:%s->%s", find, replace);
    int index = find_text(str, find);
    if (index == -1)
        return str;
    int size = find.size();
    str.replace(index, size, replace);
    // ROS_INFO("replace:%s->%s", find, replace);
    return str;
}
void MainWindow::on_pushButton_creatActionBase_clicked()
{
    // test input
    // ui->textEdit_voice->setText(QString::fromLocal8Bit("把板手拿給我"));
    std::string voiceCmd = ui->textEdit_voice->toPlainText().toStdString();
    if (voiceCmd == "")
    {
        QMessageBox::about(NULL, "Error", "No voice command");
        return;
    }

    if (voiceCmd == "好" || voiceCmd == "停")
    {
        Action = "Stop";
        InteractObject = "";
        Destination = "";
        clearItem(nlp_itemModel);
        addItem(nlp_itemModel, QString::fromStdString(Action), QString::fromStdString(InteractObject), QString::fromStdString(Destination));
        return;
    }
    // ROS_INFO_STREAM(voiceCmd);
    // NLP
    std::map<string, string> map_action = { { "給我", "Place" }, { "拿", "Place" }, { "放", "Place" }, { "磨", "Snad" }, { "吸", "Vacuum" }, { "掃", "Swipe" }, { "漆", "Paint" }, { "扶", "Hold" } };
    std::map<string, string> map_object = { { "起子", "Screwdriver" }, { "板手", "Wrench" }, { "砂紙", "Sand paper" }, { "刷子", "Brush" }, { "吸塵器", "Vacuum cleaner" }, { "木板", "Board" }, { "椅子", "Chair" } };
    std::map<string, string> map_destination = { { "這裡", "Here" }, { "桌", "Table" }, { "木板", "Board" }, { "回去", "Tool space" } };// , {"手", "Hand",}

    std::map<string, string> omit_action_fromDes = { { "Hand", "Place" } };
    std::map<string, string> omit_object_fromAct = { { "Snad", "Sand paper" }, { "Vacuum", "Vacuum cleaner" }, { "Paint", "Brush" } };
    std::map<string, string> omit_destination_fromObj = { {} };
    Action = "";
    InteractObject = "";
    Destination = "";

    int index_find = -1;
    int size_find = 0;
    // Action
    for (const auto &s : map_action)
    {
        int index = find_text(voiceCmd, s.first);
        if (index >= 0)
        {
            Action = s.second;
            int size = s.first.size();
            index_find = index;
            size_find = size;
        }
    }
    //省略 動作
    if (index_find >= 0)
    {
        //動作不做刪除
        string replace = "";
        replace += "[act]";
        string tmpT = voiceCmd;
        tmpT.replace(index_find, size_find, replace);
    }
    else if (Action == "")//省略 目標
    {
        Action = "Place";
    }

    index_find = -1;
    size_find = 0;
    // Destination
    for (const auto &s : map_destination)
    {
        int index = find_text(voiceCmd, s.first);
        if (index >= 0)
        {
            Destination = s.second;
            int size = s.first.size();
            index_find = index;
            size_find = size;
        }
    }

    if (index_find >= 0)
    {
        string replace = "";
        // for (int i = 0; i < size_find; i++)
        replace += "[des]";
        voiceCmd.replace(index_find, size_find, replace);
    }
    else if (Destination == "")//省略 目標
    {
        Destination = "Hand";
    }

    // interact
    index_find = -1;
    size_find = 0;
    for (const auto &s : map_object)
    {
        int index = find_text(voiceCmd, s.first);
        if (index >= 0)
        {
            InteractObject = s.second;
            int size = s.first.size();
            index_find = index;
            size_find = size;
        }
    }
    if (index_find >= 0)
    {
        string replace = "";
        replace += "[obj]";
        voiceCmd.replace(index_find, size_find, replace);
    }
    else if (InteractObject == "")//省略 互動物件
    {
        if (Action == "Place")//當沒有互動物件時 那一定是拿 那個唯一物件，但先被Destination搶走。
        {
            InteractObject = Destination;
            Destination = "Hand";
        }
        else
        {
            for (const auto &s : omit_object_fromAct)
            {
                int index = find_text(Action, s.first);
                if (index >= 0)
                {
                    InteractObject = s.second;
                }
            }
        }
    }
    clearItem(nlp_itemModel);
    addItem(nlp_itemModel, QString::fromStdString(Action), QString::fromStdString(InteractObject), QString::fromStdString(Destination));
}

#pragma endregion ___voice command___

//---New Era---\\.
#pragma region New Era
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
#pragma endregion

//---TM---\\.
#pragma region TM

/*TM subfunction*/
void MainWindow::sendCMD(std::string cmd, bool print /*= true*/)
{
    if (cmd != "")
    {
        ros::ServiceClient client = nh.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");
        tm_msgs::SendScript srv;
        srv.request.id = "demo";
        srv.request.script = cmd;
        if (client.call(srv))
        {
            if (srv.response.ok)
            {
                if (print)
                    ROS_INFO_STREAM("Sent script to robot:" + cmd);
            }
            else
                ROS_WARN_STREAM("Sent script to robot , but response not yet ok ");
        }
        else
        {
            ROS_ERROR_STREAM("Error send script to robot");
        }

        //"PTP(\"JPP\",0,0,90,0,90,0,70,200,0,false)"
        // 6 position
        //手臂末端移動速度百分比(%)
        //運動加速到最高速所花費時間(millisecond)
        //軌跡混合百分比(%)
        //取消精準到位
    }
}
void MainWindow::waitTM(double delaySec)
{
    sleep(delaySec);
    double t = 0.005;
    while (true)
    {
        try
        {
            if (tm_tcp_speed_x > -t && tm_tcp_speed_x < t &&
                tm_tcp_speed_y > -t && tm_tcp_speed_y < t &&
                tm_tcp_speed_z > -t && tm_tcp_speed_z < t)
                break;

            sleep(1);
            qApp->processEvents();
        }
        catch (const std::exception &e)
        {
            ROS_ERROR_STREAM("waitTM error");
        }
    }
    ROS_INFO("TM done check.");
}
void MainWindow::sendGrip(bool cmd)
{
    ros::ServiceClient client = nh.serviceClient<tm_msgs::SetIO>("tm_driver/set_io");
    tm_msgs::SetIO srv;

    // Request
    srv.request.module = tm_msgs::SetIO::Request::MODULE_ENDEFFECTOR;
    srv.request.type = tm_msgs::SetIO::Request::TYPE_DIGITAL_OUT;
    srv.request.pin = 0;
    if (cmd == true)
        srv.request.state = tm_msgs::SetIO::Request::STATE_ON;
    else
        srv.request.state = tm_msgs::SetIO::Request::STATE_OFF;

    if (client.call(srv))
    {
        if (srv.response.ok)
            ROS_INFO_STREAM("SetIO to robot");
        else
            ROS_WARN_STREAM("SetIO to robot , but response not yet ok ");
    }
    else
    {
        ROS_ERROR_STREAM("Error SetIO to robot");
    }
}

void MainWindow::on_btn_set_tm_grip_clicked()
{
    sendGrip(true);
}
void MainWindow::on_btn_set_tm_unGrip_clicked()
{
    sendGrip(false);
    GrippingObject = "";
    ui->label_grippingObject->setText("");
}

void MainWindow::on_btn_get_tm_img_clicked()
{
    sendCMD("ScriptExit()");
}


/*Tracker*/
void MainWindow::publishTracker(const std::string target_fram, const double x_offset, const double y_offset)
{
    // const std::string tf_topic = "track/blue";
    // const double servo_track_offset_x = x_offset;
    // const double servo_track_offset_y = y_offset;

    ros::Publisher pub_tracker = nh.advertise<geometry_msgs::Pose>("/track/tracker/pose", 1);

    static tf::TransformBroadcaster br;
    ros::Rate rate(5);
    servoing = true;

    bool flag_firstIn = true;
    geometry_msgs::TransformStamped transform_target;
    while (nh.ok() && servoing)
    {
        try
        {
            transform_target = tfBuffer.lookupTransform("kinect", target_fram, ros::Time(0));
            // ROS_INFO("tf blue \tT(Meter)[%lf,%lf,%lf]", tf_cam_aruco.transform.translation.x, tf_cam_aruco.transform.translation.y, tf_cam_aruco.transform.translation.z);

            double x = transform_target.transform.translation.x;
            double y = transform_target.transform.translation.y;
            double z = transform_target.transform.translation.z;

            //offset in world/kinect frame
            x += x_offset;// M
            y += y_offset;// M

            /*Publish tf tracker*/
            tf::Transform transform_out;
            transform_out.setOrigin(tf::Vector3(x, y, z));
            tf::Quaternion q;
            q.setRPY(0, 0, 0);
            transform_out.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform_out, ros::Time::now(), "kinect", "track/tracker"));


            /*Publish tracker*/
            geometry_msgs::Pose msg_pose;
            msg_pose.position.x = x;
            msg_pose.position.y = y;
            msg_pose.position.z = z;
            pub_tracker.publish(msg_pose);

            if (flag_firstIn)
            {
                ROS_INFO_STREAM("=====Boardcasting=====");
                ROS_INFO_STREAM("-- kinect->track/tracker");
                ROS_INFO_STREAM("-- from" + target_fram + "\n");
                addItem(msg_tm_listModel, "=====Boardcasting=====\n-- kinect->track/tracker\n");
                flag_firstIn = false;
            }
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        rate.sleep();
    }


    ROS_INFO("Publish tracker [close].");
}
void MainWindow::publishTracker_msgFilter()
{
}
cv::Point2f MainWindow::getTrackPoint_yellow()
{
    tf::TransformListener listener;
    static tf::TransformBroadcaster br;

    ros::Rate rate(10.0);
    while (nh.ok())
    {
        tf::StampedTransform transform;
        try
        {
            double x = 0;
            double y = 0;

            listener.lookupTransform("/world", "/track/yellow", ros::Time(0), transform);
            x = transform.getOrigin().x();
            y = transform.getOrigin().y();
            return cv::Point2f(x, y);
        }
        catch (tf::TransformException ex)
        {
            ROS_INFO("Wait tarck/.../ pose");
            ros::Duration(0.1).sleep();
        }
        ros::spinOnce();
        rate.sleep();
    }
    return cv::Point2f(-0.45, -0.35);
}
cv::Point2f MainWindow::getTrackPoint_blue()
{
    tf::TransformListener listener;
    static tf::TransformBroadcaster br;

    ros::Rate rate(10.0);
    while (nh.ok())
    {
        tf::StampedTransform transform;
        try
        {
            double x = 0;
            double y = 0;

            listener.lookupTransform("/world", "/track/blue", ros::Time(0), transform);
            x = transform.getOrigin().x();
            y = transform.getOrigin().y();
            return cv::Point2f(x, y);
        }
        catch (tf::TransformException ex)
        {
            ROS_INFO("Wait tarck/.../ pose");
            ros::Duration(0.1).sleep();
        }
        ros::spinOnce();
        rate.sleep();
    }
    return cv::Point2f(-0.45, -0.35);
}

/*servo*/
void MainWindow::on_btn_set_tm_pos_preTrack_clicked()
{
    sendCMD("PTP(\"CPP\",-450,-350,215,100,0,-45,100,200,0,false)", false);// mm-deg
}
void MainWindow::TM_servoOn(const string trackName, const double offset_x, const double offset_y)
{
    // safty check(need TF correction)
    if (tf_correction_check != 3)
    {
        QMessageBox::about(NULL, "Error", "TF /kinect does not transform to /world");
        return;
    }
    // safty function
    sendCMD("StopContinueVmode()");
    sendCMD("SuspendContinueVmode()");
    sleep(1);
    // start
    sendCMD("ContinueVLine(500,10000)");
    tf::TransformListener listener;
    ros::ServiceClient client = nh.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");
    ros::Rate rate(3);
    servoing = true;
    while (nh.ok() && servoing)
    {
        tf::StampedTransform transform;
        try
        {
            double x = 0;
            double y = 0;

            listener.lookupTransform("/TM_robot", "/track/blue", ros::Time(0), transform);
            x = transform.getOrigin().x();
            y = transform.getOrigin().y();
            // x += offset_x; // M
            // y += offset_y; // M
            x += -0.1;// M
            y += 0.1; // M
            // ROS_INFO_STREAM("vel\tx = " + std::to_string(x) + " y = " + std::to_string(y));
            double a = 0.2;
            double Vx = x + (a * x * x);
            double Vy = y + (a * y * y);
            // speed limit
            double limit = 0.05;// 0.1
            Vx = (Vx > limit) ? limit : Vx;
            Vx = (Vx < -limit) ? -limit : Vx;
            Vy = (Vy > limit) ? limit : Vy;
            Vy = (Vy < -limit) ? -limit : Vy;

            tm_msgs::SendScript srv;
            srv.request.id = "demo";
            srv.request.script = "SetContinueVLine(" + std::to_string(Vx) + "," + std::to_string(Vy) + ",0,0,0,0)";
            if (client.call(srv))
            {
                if (srv.response.ok)
                    ROS_INFO_STREAM("Sent script to robot: SetContinueVLine(" + std::to_string(Vx) + "," + std::to_string(Vy) + ",0,0,0,0)");
                else
                    ROS_WARN_STREAM("Sent script to robot , but response not yet ok ");
            }
            else
            {
                ROS_ERROR_STREAM("Error send script to robot");
            }
            // sendCMD("SetContinueVLine(" + std::to_string(Vx) + "," + std::to_string(Vy) + ",0,0,0,0)");
        }
        catch (tf::TransformException ex)
        {
            sendCMD("SuspendContinueVmode()");
            ROS_INFO("Track pose loss...");
            ros::Duration(0.1).sleep();
        }
        rate.sleep();
    }
    sendCMD("StopContinueVmode()");
}
void MainWindow::thread_TM_servoOn(const string trackName)
{
    // double x = 0;
    // double y = 0;
    // if (trackName == "/track/blue")
    // {
    //     x = -0.1;
    //     y = +0.1;
    // }
    // QThread *thread = new QThread();
    // ServoThreadHandle *thHandle = new ServoThreadHandle(this, trackName, x, y);
    // thHandle->moveToThread(thread);
    // connect(thread, &QThread::started, thHandle, &ServoThreadHandle::th_servoOn);
    // thread->start();
}
void MainWindow::TM_newServoOn()
{
    //safty check(need TF correction)
    if (!FLAG.tf_m03Robot_inVision)
    {
        QMessageBox::about(NULL, "Error", "TF /TM_robot/base does not transform to /world");
        return;
    }
    addItem(msg_tm_listModel, "=====Servoing=====\n-- servo point : track/tracker\n");
    bool realMove = false;
    if (realMove)
    {
        //safty function
        sendCMD("StopContinueVmode()");
        sendCMD("SuspendContinueVmode()");
        sleep(1);

        sendCMD("ContinueVLine(500,10000)");
    }
    ros::ServiceClient client = nh.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");

    ros::Rate rate(3);
    servoing = true;
    while (nh.ok() && servoing)
    {
        try
        {
            geometry_msgs::TransformStamped transformStamped1;
            geometry_msgs::TransformStamped transformStamped2;
            transformStamped1 = tfBuffer.lookupTransform("TM_robot/base", "TM_robot/tcp", ros::Time(0));
            transformStamped2 = tfBuffer.lookupTransform("TM_robot/base", "track/tracker", ros::Time(0));
            double x = transformStamped2.transform.translation.x - transformStamped1.transform.translation.x;
            double y = transformStamped2.transform.translation.y - transformStamped1.transform.translation.y;
            double z = transformStamped2.transform.translation.z - transformStamped1.transform.translation.z;

            double a = 0.2;
            double Vx = x + (a * x * x);
            double Vy = y + (a * y * y);
            // speed limit
            double limit = 0.03;// 0.1
            Vx = (Vx > limit) ? limit : Vx;
            Vx = (Vx < -limit) ? -limit : Vx;
            Vy = (Vy > limit) ? limit : Vy;
            Vy = (Vy < -limit) ? -limit : Vy;

            if (realMove)
            {
                tm_msgs::SendScript srv;
                srv.request.id = "demo";
                srv.request.script = "SetContinueVLine(" + std::to_string(Vx) + "," + std::to_string(Vy) + ",0,0,0,0)";
                if (client.call(srv))
                {
                    if (srv.response.ok)
                        ROS_INFO_STREAM("Sent script to robot: SetContinueVLine(" + std::to_string(Vx) + "," + std::to_string(Vy) + ",0,0,0,0)");
                    else
                        ROS_WARN_STREAM("Sent script to robot , but response not yet ok ");
                }
                else
                {
                    ROS_ERROR_STREAM("Error send script to robot");
                }
            }
            else
            {
                ROS_INFO("(dx,dy)=(% 1.3lf,% 1.3lf)", x, y);
                ROS_INFO("Semulation: SetContinueVLine(% 1.3lf,% 1.3lf,0,0,0,0)", Vx, Vy);
            }
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        rate.sleep();
    }
    sendCMD("StopContinueVmode()");
}
void MainWindow::Test(int value)
{
    // ROS_INFO("test!!");
    // emit cc(value);
}

/*button cick*/
void MainWindow::on_btn_set_tf_tracker_clicked()
{
    QThread *thread = new QThread();
    std::string str = "track/blue";
    double x = 0;
    double y = 0;
    SubscribeThreadHandle *thHandle = new SubscribeThreadHandle(&MainWindow::publishTracker, str, x, y, this);
    thHandle->moveToThread(thread);
    connect(thread, &QThread::started, thHandle, &SubscribeThreadHandle::doFunction3);
    thread->start();
}
void MainWindow::on_btn_set_tm_servo_trackPose_clicked()
{
    // thread_TM_servoOn("/track/blue");
    QThread *thread = new QThread();
    SubscribeThreadHandle *thHandle = new SubscribeThreadHandle(&MainWindow::TM_newServoOn, this);
    thHandle->moveToThread(thread);
    connect(thread, &QThread::started, thHandle, &SubscribeThreadHandle::doFunction);
    thread->start();
}
void MainWindow::on_btn_set_tm_servo_stop_clicked()
{
    servoing = false;
    sendCMD("StopContinueVmode()");
    // map_sub_topic["/track/tracker/pose"].shutdown();
}
void MainWindow::on_pushButton_clearTmMsg_clicked()
{
    clearItem(msg_tm_listModel);
}
void MainWindow::on_pushButton_clear_execute_clicked()
{
    clearItem(msg_listModel);
}
/*NLP execute*/
void MainWindow::tm_grip_object(std::string obj)
{
    if (obj == "Brush")
    {
        sendCMD("PTP(\"JPP\",-104.83,35.07,-82.00,-135.33,82.01,-163.61,100,200,0,false)");// out safe
        waitTM(1);
        sendCMD("PTP(\"JPP\",-104.84,-16.05,-77.61,-175.87,90.10,-149.95,100,200,0,false)");// up
        waitTM(1);
        sendCMD("PTP(\"JPP\",-104.83,-16.99,-83.98,-168.55,90.10,-149.96,100,200,0,false)");// grip
        waitTM(1);
        sendGrip(true);
        sleep(3);
        sendCMD("PTP(\"JPP\",-104.84,-16.05,-77.61,-175.87,90.10,-149.95,100,200,0,false)");// up
        waitTM(1);
        sendCMD("PTP(\"JPP\",-104.83,35.07,-82.00,-135.33,82.01,-163.61,100,200,0,false)");// out safe
        waitTM(1);
    }
    else if (obj == "Sand paper")
    {
        // sendCMD("PTP(\"JPP\",-9.36,7.43,-84.69,-152.81,48.53,-218.83,100,200,0,false)"); // joint out safe
        // waitTM(1);
        // sendCMD("PTP(\"CPP\",-450,-350,250,125,0,-45,100,200,0,false)"); // mm-deg up
        // waitTM(1);
        cv::Point2f p = getTrackPoint_yellow();
        sendCMD("PTP(\"CPP\"," + std::to_string(p.x * 1000) + "," + std::to_string(p.y * 1000) + ",250,125,0,-45,100,200,0,false)");// mm-deg up
        waitTM(1);
        sendCMD("PTP(\"CPP\"," + std::to_string(p.x * 1000) + "," + std::to_string(p.y * 1000) + ",216,125,0,-45,100,200,0,false)");// mm-deg down
        waitTM(1);
        sendGrip(true);
        sleep(3);
        sendCMD("PTP(\"CPP\"," + std::to_string(p.x * 1000) + "," + std::to_string(p.y * 1000) + ",250,125,0,-45,100,200,0,false)");// mm-deg up
        waitTM(1);
    }
    else if (obj == "Vacuum cleaner")
    {
        sendCMD("PTP(\"JPP\",-27.52,-12.44,-58.81,-90.08,20.23,-163.63,100,200,0,false)");// out safe
        waitTM(1);
        sendCMD("PTP(\"JPP\",-18.11,-25.40,-38.77,-102.67,29.30,-169.71,100,200,0,false)");// up
        waitTM(1);
        sendCMD("PTP(\"JPP\",-18.13,-20.83,-57.07,-88.90,29.27,-169.67,100,200,0,false)");// grip
        waitTM(1);
        sendGrip(true);
        sleep(3);
        sendCMD("PTP(\"JPP\",-18.11,-25.40,-38.77,-102.67,29.30,-169.71,100,200,0,false)");// up
        waitTM(1);
        sendCMD("PTP(\"JPP\",-27.52,-12.44,-58.81,-90.08,20.23,-163.63,100,200,0,false)");// out safe
        waitTM(1);
    }
    else
    {
        QMessageBox::about(NULL, "Error", "Unknow objet!");
    }
}
void MainWindow::tm_place_object(std::string obj)
{
    if (obj == "Brush")
    {
        sendCMD("PTP(\"JPP\",-104.83,35.07,-82.00,-135.33,82.01,-163.61,100,200,0,false)");// out safe
        waitTM(1);
        sendCMD("PTP(\"JPP\",-104.84,-16.05,-77.61,-175.87,90.10,-149.95,100,200,0,false)");// up
        waitTM(1);
        sendCMD("PTP(\"JPP\",-104.83,-16.99,-83.98,-168.55,90.10,-149.96,100,200,0,false)");// grip
        waitTM(1);
        sendGrip(false);
        sleep(1);
        sendCMD("PTP(\"JPP\",-104.84,-16.05,-77.61,-175.87,90.10,-149.95,100,200,0,false)");// up
        waitTM(1);
        sendCMD("PTP(\"JPP\",-104.83,35.07,-82.00,-135.33,82.01,-163.61,100,200,0,false)");// out safe
        waitTM(1);
    }
    else if (obj == "Sand paper")
    {
        sendCMD("PTP(\"JPP\",-18.46,8.62,-89.74,-135.47,72.34,-193.04,100,200,0,false)");// out safe
        waitTM(1);
        sendCMD("PTP(\"JPP\",-18.46,9.13,-95.06,-130.65,72.33,-193.05,100,200,0,false)");// down
        waitTM(1);
        sendGrip(false);
        sleep(1);
        sendCMD("PTP(\"JPP\",-18.46,8.62,-89.74,-135.47,72.34,-193.04,100,200,0,false)");// coner out safe
        waitTM(1);
    }
    else if (obj == "Vacuum cleaner")
    {
        sendCMD("PTP(\"JPP\",-27.52,-12.44,-58.81,-90.08,20.23,-163.63,100,200,0,false)");// out safe
        waitTM(1);
        sendCMD("PTP(\"JPP\",-18.11,-25.40,-38.77,-102.67,29.30,-169.71,100,200,0,false)");// up
        waitTM(1);
        sendCMD("PTP(\"JPP\",-18.13,-20.83,-57.07,-88.90,29.27,-169.67,100,200,0,false)");// grip
        waitTM(1);
        sendGrip(false);
        sleep(1);
        sendCMD("PTP(\"JPP\",-18.11,-25.40,-38.77,-102.67,29.30,-169.71,100,200,0,false)");// up
        waitTM(1);
        sendCMD("PTP(\"JPP\",-27.52,-12.44,-58.81,-90.08,20.23,-163.63,100,200,0,false)");// out safe
        waitTM(1);
    }
    else
    {
        QMessageBox::about(NULL, "Error", "Unknow objet!");
    }
}

void MainWindow::on_btn_tm_execute_nlp_clicked()
{
    if (Action == "Stop")
    {
        servoing = false;
        clearItem(msg_listModel);
        addItem(msg_listModel, "Stop");
        sendCMD("StopAndClearBuffer()");
        return;
    }

    // Action     InteractObject     Destination
    clearItem(msg_listModel);

    if (GrippingObject != "" && InteractObject != GrippingObject)
    {
        addItem(msg_listModel, QString::fromStdString("Place " + GrippingObject));
        tm_place_object(GrippingObject);
        GrippingObject = "";
        ui->label_grippingObject->setText("");
    }
    if (GrippingObject != InteractObject)
    {
        addItem(msg_listModel, QString::fromStdString("Pick " + InteractObject));
        tm_grip_object(InteractObject);
        GrippingObject = InteractObject;
        ui->label_grippingObject->setText(setStringColor(QString::fromStdString(GrippingObject), "#c25555"));
    }

    if (Action == "Place")
    {
        if (Destination == "Tool space")
        {
            tm_place_object(GrippingObject);
            GrippingObject = "";
            ui->label_grippingObject->setText(setStringColor(QString::fromStdString(GrippingObject), "#c25555"));
        }
        else if (Destination == "Hand")
        {
            cv::Point2f p = getTrackPoint_blue();
            sendCMD("PTP(\"CPP\"," + std::to_string(p.x * 1000) + "," + std::to_string(p.y * 1000) + ",300,125,0,-45,100,200,0,false)");// mm-deg up
            waitTM(2);
            sendGrip(false);
            sleep(1);
            sendCMD("PTP(\"JPP\",-18.46,8.62,-89.74,-135.47,72.34,-193.04,100,200,0,false)");// coner out safe
            waitTM(1);
            GrippingObject = "";
            ui->label_grippingObject->setText(setStringColor(QString::fromStdString(GrippingObject), "#c25555"));
        }
    }
    else if (Action == "Snad")
    {
        if (Destination == "Here")
        {
            cv::Point2f p = getTrackPoint_blue();
            p.x -= 0.03;
            p.y += 0.03;
            sendCMD("PTP(\"CPP\"," + std::to_string((p.x * 1000) + 30) + "," + std::to_string(p.y * 1000) + ",250,125,0,-45,100,200,0,false)");// mm-deg up
            sleep(3);
            sendCMD("PTP(\"CPP\"," + std::to_string((p.x * 1000) - 30) + "," + std::to_string(p.y * 1000) + ",250,125,0,-45,100,200,0,false)");// mm-deg up
            // sleep(0.5);
            sendCMD("PTP(\"CPP\"," + std::to_string((p.x * 1000) - 30) + "," + std::to_string(p.y * 1000) + ",250,125,0,-45,100,200,0,false)");// mm-deg up
            // sleep(0.5);
            sendCMD("PTP(\"CPP\"," + std::to_string((p.x * 1000) + 30) + "," + std::to_string(p.y * 1000) + ",250,125,0,-45,100,200,0,false)");// mm-deg up
            // sleep(0.5);
            sendCMD("PTP(\"CPP\"," + std::to_string((p.x * 1000) - 30) + "," + std::to_string(p.y * 1000) + ",250,125,0,-45,100,200,0,false)");// mm-deg up
            // sleep(0.5);
            sendCMD("PTP(\"CPP\"," + std::to_string((p.x * 1000) + 30) + "," + std::to_string(p.y * 1000) + ",250,125,0,-45,100,200,0,false)");// mm-deg up
            // sleep(0.5);
        }
    }
    else if (Action == "Vacuum")
    {
        sendCMD("PTP(\"JPP\",-28.00,-8.16,-89.18,-30.65,-14.29,-84.75,100,200,0,false)");// out safe
        waitTM(1);
        cv::Point2f p = getTrackPoint_blue();
        p.x -= 0.09;
        p.y += 0.09;
        sendCMD("PTP(\"CPP\"," + std::to_string(p.x * 1000) + "," + std::to_string(p.y * 1000) + ",302,105.48,-43.41,-29.67,100,200,0,false)");// mm-deg
        sleep(2);
        // sendCMD("PTP(\"CPP\"," + std::to_string(p.x * 1000) + "," + std::to_string(p.y * 1000) + ",320,105.48,-43.41,-29.67,100,200,0,false)"); // mm-deg
        waitTM(1);
        // Track
        // thread_TM_servoOn("/track/blue");
    }
    else
    {
        QMessageBox::about(NULL, "Error", "Unknow action!");
    }
}

/*PTP*/
void MainWindow::on_btn_set_tm_pos_1_clicked()
{
    sendCMD("PTP(\"CPP\",-500,-300,450,90,0,-45,100,200,0,false)", false);// mm-deg
    waitTM(1);
}
void MainWindow::on_btn_set_tm_pos_2_clicked()
{
    sendCMD("PTP(\"CPP\",-500,-300,500,90,0,-45,100,200,0,false)", false);// mm-deg
    waitTM(1);
}
void MainWindow::on_btn_set_tm_pos_3_clicked()
{
    sendCMD("PTP(\"CPP\",-500,-300,550,90,0,-45,100,200,0,false)", false);// mm-deg
    waitTM(1);
}
void MainWindow::on_btn_set_tm_pos_4_clicked()
{
    // cam
    sendCMD("PTP(\"JPP\",-168,-23,53,26,51,134,100,200,0,false)");
}
void MainWindow::on_btn_set_tm_pos_5_clicked()
{
    // pre grip
    sendCMD("PTP(\"JPP\",-227,20,127,-140,96,92,100,200,0,false)");
}
void MainWindow::on_btn_set_tm_pos_6_clicked()
{
    // grip
    sendCMD("PTP(\"JPP\",-226,28,114,-135,95,93,100,200,0,false)");
}
void MainWindow::on_btn_set_tm_pos_7_clicked()
{
    // up
    sendCMD("PTP(\"JPP\",-227,15,101,-109,96,93,100,200,0,false)");
}
void MainWindow::on_btn_set_tm_pos_8_clicked()
{
    // mid
}

#pragma endregion

//---TF transform---\\.
#pragma region TF transform
/*set tf match*/
void MainWindow::on_btn_set_tm_pos_setTF_clicked()
{
    sendCMD("PTP(\"JPP\",1.17,17.25,-115.47,-85.09,47.06,-189.51,100,200,0,false)", false);// joint  mm-deg
    waitTM(1);
    sendCMD("PTP(\"JPP\",1.17,7.25,-115.47,-85.09,47.06,-189.51,100,200,0,false)", false);// joint  mm-deg
    sendCMD("PTP(\"CPP\",-450,-350,215,100,0,-45,100,200,0,false)", false);               // mm-deg
    sendCMD("PTP(\"CPP\",-500,-300,300,90,0,-45,100,200,0,false)", false);                // mm-deg
    waitTM(1);
    tf_correction_check = 1;
}

void MainWindow::on_btn_set_tf2_tm_base_clicked()
{
    ROS_INFO("\033[0;33m=====Calibration=====\033[0m");
    ROS_INFO("--Match upper_camera1/aruco  and kinect/aruco");
    addItem(msg_tm_listModel, "=====Calibration=====\n--Match upper_camera1/aruco  and kinect/aruco\n");

    //transform target -> source
    std::string frame_calibration_target = "world";
    std::string frame_matched_target = "kinect";
    std::string frame_matched_marker = "kinect/track/aruco/4x4/0";
    geometry_msgs::TransformStamped trans_matched;//the matched transform
    std::string frame_matching_source = "TM_robot/base";
    std::string frame_matching_marker = "TM_robot/tcp/aruco";
    geometry_msgs::TransformStamped trans_matching;//the matching transform

    /*Zero*/
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = frame_calibration_target;
    transformStamped.child_frame_id = frame_matching_source;
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = 0;
    transformStamped.transform.translation.z = 0;
    transformStamped.transform.rotation.x = 0;
    transformStamped.transform.rotation.y = 0;
    transformStamped.transform.rotation.z = 0;
    transformStamped.transform.rotation.w = 1;
    static_broadcaster.sendTransform(transformStamped);
    ROS_INFO("--[Set] %s->%s to zero", frame_calibration_target.c_str(), frame_matching_source.c_str());
    // ros::Duration(1.0).sleep();
    while (checkTransform(frame_calibration_target, frame_matching_source, transformStamped) == false)
    { ros::spinOnce(); };


    /*set rotation first!!*/
    ROS_INFO("--Find rotation--");
    trans_matched = lookupTransform(frame_matched_target, frame_matched_marker);//Translation: [-0.239, 0.007, 0.653]
    // ROS_INFO("--[Get] kinect->aruco \tT(Meter)[%lf,%lf,%lf]", tf_cam_aruco.transform.translation.x, tf_cam_aruco.transform.translation.y, tf_cam_aruco.transform.translation.z);

    trans_matching = lookupTransform(frame_calibration_target, frame_matching_marker);//Translation: [-0.500, -0.300, 0.300]
    // ROS_INFO("--[Get] tm->aruco \tT(Meter)[%lf,%lf,%lf]", tf_tm_aruco.transform.translation.x, tf_tm_aruco.transform.translation.y, tf_tm_aruco.transform.translation.z);

    //get quaternion
    Rotation::Quaternion quat_cam(trans_matched.transform.rotation);
    Rotation::Quaternion quat_tm(trans_matching.transform.rotation);
    //to rotation matrix
    Rotation::RotationMatrix R_cam = quat_cam.ToRotationMatrix();
    Rotation::RotationMatrix R_tm = quat_tm.ToRotationMatrix();

    //show euler angle for debugging
    std::vector<Rotation::EulerAngles> EA_cam = R_cam.ToEulerAngles();
    std::vector<Rotation::EulerAngles> EA_tm = R_tm.ToEulerAngles();
    ROS_INFO("--[Get] %s->aruco \tR(deg): % 1.3lf\t% 1.3lf\t% 1.3lf", frame_matched_target.c_str(), toDeg(EA_cam[0].x), toDeg(EA_cam[0].y), toDeg(EA_cam[0].z));
    ROS_INFO("--[Get] %s->aruco \tR(deg) % 1.3lf\t% 1.3lf\t% 1.3lf", frame_matching_source.c_str(), toDeg(EA_tm[0].x), toDeg(EA_tm[0].y), toDeg(EA_tm[0].z));

    //Calibration
    Rotation::RotationMatrix R_calibration = Rotation::dot(R_cam, R_tm.T());
    // Rotation::Quaternion quat_calibration = R_calibration.ToQuaternion();

    std::vector<Rotation::EulerAngles> EA_final = R_calibration.ToEulerAngles();
    tf2::Quaternion quat;
    quat.setRPY(EA_final[0].x, EA_final[0].y, EA_final[0].z);
    transformStamped.transform.rotation.x = quat.x();
    transformStamped.transform.rotation.y = quat.y();
    transformStamped.transform.rotation.z = quat.z();
    transformStamped.transform.rotation.w = quat.w();
    static_broadcaster.sendTransform(transformStamped);
    ROS_INFO("--[Set] %s->%s \tR(deg) % 1.3lf\t% 1.3lf\t% 1.3lf", frame_calibration_target.c_str(), frame_matching_source.c_str(), toDeg(EA_final[0].x), toDeg(EA_final[0].y), toDeg(EA_final[0].z));

    while (checkTransform(frame_calibration_target, frame_matching_source, transformStamped) == false)
    { ros::spinOnce(); };

    /*find tf translation next*/
    ROS_INFO("--Find translation--");
    trans_matched = lookupTransform(frame_matched_target, frame_matched_marker);
    ROS_INFO("--[Get] %s->aruco \tT(Meter)[%lf,%lf,%lf]", frame_matched_target.c_str(), trans_matched.transform.translation.x, trans_matched.transform.translation.y, trans_matched.transform.translation.z);

    trans_matching = lookupTransform(frame_calibration_target, frame_matching_marker);
    ROS_INFO("--[Get] %s->aruco \tT(Meter)[%lf,%lf,%lf]", frame_matching_source.c_str(), trans_matching.transform.translation.x, trans_matching.transform.translation.y, trans_matching.transform.translation.z);

    double dx = trans_matched.transform.translation.x - trans_matching.transform.translation.x;
    double dy = trans_matched.transform.translation.y - trans_matching.transform.translation.y;
    double dz = trans_matched.transform.translation.z - trans_matching.transform.translation.z;
    transformStamped.transform.translation.x = dx;
    transformStamped.transform.translation.y = dy;
    transformStamped.transform.translation.z = dz;
    ROS_INFO("--[Set] %s->%s \tT(Meter)[%lf,%lf,%lf]", frame_calibration_target.c_str(), frame_matching_source.c_str(), dx, dy, dz);

    //use same rotation
    transformStamped.transform.rotation.x = quat.x();
    transformStamped.transform.rotation.y = quat.y();
    transformStamped.transform.rotation.z = quat.z();
    transformStamped.transform.rotation.w = quat.w();
    ROS_INFO("--[Set] %s->%s \tR(deg) % 1.3lf\t% 1.3lf\t% 1.3lf", frame_calibration_target.c_str(), frame_matching_source.c_str(), toDeg(EA_final[0].x), toDeg(EA_final[0].y), toDeg(EA_final[0].z));
    static_broadcaster.sendTransform(transformStamped);
    ros::spinOnce();
    FLAG.tf_m03Robot_inVision = true;
    ROS_INFO("\033[0;33m===Calibration Done===\033[0m");
}
void MainWindow::on_btn_set_tf2_upper_camera1_clicked()
{
    ROS_INFO("\033[0;33m=====Calibration=====\033[0m");
    ROS_INFO("--Match upper_camera1/aruco  and kinect/aruco");
    addItem(msg_tm_listModel, "=====Calibration=====\n--Match upper_camera1/aruco  and kinect/aruco\n");

    //transform target -> source
    std::string frame_calibration_target = "world";
    std::string frame_matched_target = "kinect";
    std::string frame_matched_marker = "kinect/track/aruco/5x5/1";
    geometry_msgs::TransformStamped trans_matched;//the matched transform
    std::string frame_matching_source = "upper_camera1";
    std::string frame_matching_marker = "upper_camera1/track/aruco/5x5/1";
    geometry_msgs::TransformStamped trans_matching;//the matching transform

    /*Zero*/
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = frame_calibration_target;
    transformStamped.child_frame_id = frame_matching_source;
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = 0;
    transformStamped.transform.translation.z = 0;
    transformStamped.transform.rotation.x = 0;
    transformStamped.transform.rotation.y = 0;
    transformStamped.transform.rotation.z = 0;
    transformStamped.transform.rotation.w = 1;
    static_broadcaster.sendTransform(transformStamped);
    ROS_INFO("--[Set] %s->%s to zero", frame_calibration_target.c_str(), frame_matching_source.c_str());
    // ros::Duration(1.0).sleep();
    while (checkTransform(frame_calibration_target, frame_matching_source, transformStamped) == false)
    { ros::spinOnce(); };


    /*set rotation first!!*/
    ROS_INFO("--Find rotation--");
    trans_matched = lookupTransform(frame_matched_target, frame_matched_marker);//Translation: [-0.239, 0.007, 0.653]
    // ROS_INFO("--[Get] kinect->aruco \tT(Meter)[%lf,%lf,%lf]", tf_cam_aruco.transform.translation.x, tf_cam_aruco.transform.translation.y, tf_cam_aruco.transform.translation.z);

    trans_matching = lookupTransform(frame_calibration_target, frame_matching_marker);//Translation: [-0.500, -0.300, 0.300]
    // ROS_INFO("--[Get] tm->aruco \tT(Meter)[%lf,%lf,%lf]", tf_tm_aruco.transform.translation.x, tf_tm_aruco.transform.translation.y, tf_tm_aruco.transform.translation.z);

    //get quaternion
    Rotation::Quaternion quat_cam(trans_matched.transform.rotation);
    Rotation::Quaternion quat_tm(trans_matching.transform.rotation);
    //to rotation matrix
    Rotation::RotationMatrix R_cam = quat_cam.ToRotationMatrix();
    Rotation::RotationMatrix R_tm = quat_tm.ToRotationMatrix();

    //show euler angle for debugging
    std::vector<Rotation::EulerAngles> EA_cam = R_cam.ToEulerAngles();
    std::vector<Rotation::EulerAngles> EA_tm = R_tm.ToEulerAngles();
    ROS_INFO("--[Get] %s->aruco \tR(deg): % 1.3lf\t% 1.3lf\t% 1.3lf", frame_matched_target.c_str(), toDeg(EA_cam[0].x), toDeg(EA_cam[0].y), toDeg(EA_cam[0].z));
    ROS_INFO("--[Get] %s->aruco \tR(deg) % 1.3lf\t% 1.3lf\t% 1.3lf", frame_matching_source.c_str(), toDeg(EA_tm[0].x), toDeg(EA_tm[0].y), toDeg(EA_tm[0].z));

    //Calibration
    Rotation::RotationMatrix R_calibration = Rotation::dot(R_cam, R_tm.T());
    // Rotation::Quaternion quat_calibration = R_calibration.ToQuaternion();

    std::vector<Rotation::EulerAngles> EA_final = R_calibration.ToEulerAngles();
    tf2::Quaternion quat;
    quat.setRPY(EA_final[0].x, EA_final[0].y, EA_final[0].z);
    transformStamped.transform.rotation.x = quat.x();
    transformStamped.transform.rotation.y = quat.y();
    transformStamped.transform.rotation.z = quat.z();
    transformStamped.transform.rotation.w = quat.w();
    static_broadcaster.sendTransform(transformStamped);
    ROS_INFO("--[Set] %s->%s \tR(deg) % 1.3lf\t% 1.3lf\t% 1.3lf", frame_calibration_target.c_str(), frame_matching_source.c_str(), toDeg(EA_final[0].x), toDeg(EA_final[0].y), toDeg(EA_final[0].z));

    while (checkTransform(frame_calibration_target, frame_matching_source, transformStamped) == false)
    { ros::spinOnce(); };

    /*find tf translation next*/
    ROS_INFO("--Find translation--");
    trans_matched = lookupTransform(frame_matched_target, frame_matched_marker);
    ROS_INFO("--[Get] %s->aruco \tT(Meter)[%lf,%lf,%lf]", frame_matched_target.c_str(), trans_matched.transform.translation.x, trans_matched.transform.translation.y, trans_matched.transform.translation.z);

    trans_matching = lookupTransform(frame_calibration_target, frame_matching_marker);
    ROS_INFO("--[Get] %s->aruco \tT(Meter)[%lf,%lf,%lf]", frame_matching_source.c_str(), trans_matching.transform.translation.x, trans_matching.transform.translation.y, trans_matching.transform.translation.z);

    double dx = trans_matched.transform.translation.x - trans_matching.transform.translation.x;
    double dy = trans_matched.transform.translation.y - trans_matching.transform.translation.y;
    double dz = trans_matched.transform.translation.z - trans_matching.transform.translation.z;
    transformStamped.transform.translation.x = dx;
    transformStamped.transform.translation.y = dy;
    transformStamped.transform.translation.z = dz;
    ROS_INFO("--[Set] %s->%s \tT(Meter)[%lf,%lf,%lf]", frame_calibration_target.c_str(), frame_matching_source.c_str(), dx, dy, dz);

    //use same rotation
    transformStamped.transform.rotation.x = quat.x();
    transformStamped.transform.rotation.y = quat.y();
    transformStamped.transform.rotation.z = quat.z();
    transformStamped.transform.rotation.w = quat.w();
    ROS_INFO("--[Set] %s->%s \tR(deg) % 1.3lf\t% 1.3lf\t% 1.3lf", frame_calibration_target.c_str(), frame_matching_source.c_str(), toDeg(EA_final[0].x), toDeg(EA_final[0].y), toDeg(EA_final[0].z));
    static_broadcaster.sendTransform(transformStamped);
    ros::spinOnce();
    FLAG.tf_calibration_upperCam1 = true;
    ROS_INFO("\033[0;33m===Calibration Done===\033[0m");
}
void MainWindow::on_btn_set_tf2_upper_camera2_clicked()
{
    ROS_INFO("\033[0;33m=====Calibration=====\033[0m");
    ROS_INFO("--Match upper_camera2/aruco  and kinect/aruco");
    addItem(msg_tm_listModel, "=====Calibration=====\n--Match upper_camera1/aruco  and kinect/aruco\n");

    //transform target -> source
    std::string frame_calibration_target = "world";
    std::string frame_matched_target = "kinect";
    std::string frame_matched_marker = "kinect/track/aruco/5x5/2";
    geometry_msgs::TransformStamped trans_matched;//the matched transform
    std::string frame_matching_source = "upper_camera2";
    std::string frame_matching_marker = "upper_camera2/track/aruco/5x5/2";
    geometry_msgs::TransformStamped trans_matching;//the matching transform

    /*Zero*/
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = frame_calibration_target;
    transformStamped.child_frame_id = frame_matching_source;
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = 0;
    transformStamped.transform.translation.z = 0;
    transformStamped.transform.rotation.x = 0;
    transformStamped.transform.rotation.y = 0;
    transformStamped.transform.rotation.z = 0;
    transformStamped.transform.rotation.w = 1;
    static_broadcaster.sendTransform(transformStamped);
    ROS_INFO("--[Set] %s->%s to zero", frame_calibration_target.c_str(), frame_matching_source.c_str());
    // ros::Duration(1.0).sleep();
    while (checkTransform(frame_calibration_target, frame_matching_source, transformStamped) == false)
    { ros::spinOnce(); };


    /*set rotation first!!*/
    ROS_INFO("--Find rotation--");
    trans_matched = lookupTransform(frame_matched_target, frame_matched_marker);//Translation: [-0.239, 0.007, 0.653]
    // ROS_INFO("--[Get] kinect->aruco \tT(Meter)[%lf,%lf,%lf]", tf_cam_aruco.transform.translation.x, tf_cam_aruco.transform.translation.y, tf_cam_aruco.transform.translation.z);

    trans_matching = lookupTransform(frame_calibration_target, frame_matching_marker);//Translation: [-0.500, -0.300, 0.300]
    // ROS_INFO("--[Get] tm->aruco \tT(Meter)[%lf,%lf,%lf]", tf_tm_aruco.transform.translation.x, tf_tm_aruco.transform.translation.y, tf_tm_aruco.transform.translation.z);

    //get quaternion
    Rotation::Quaternion quat_cam(trans_matched.transform.rotation);
    Rotation::Quaternion quat_tm(trans_matching.transform.rotation);
    //to rotation matrix
    Rotation::RotationMatrix R_cam = quat_cam.ToRotationMatrix();
    Rotation::RotationMatrix R_tm = quat_tm.ToRotationMatrix();

    //show euler angle for debugging
    std::vector<Rotation::EulerAngles> EA_cam = R_cam.ToEulerAngles();
    std::vector<Rotation::EulerAngles> EA_tm = R_tm.ToEulerAngles();
    ROS_INFO("--[Get] %s->aruco \tR(deg): % 1.3lf\t% 1.3lf\t% 1.3lf", frame_matched_target.c_str(), toDeg(EA_cam[0].x), toDeg(EA_cam[0].y), toDeg(EA_cam[0].z));
    ROS_INFO("--[Get] %s->aruco \tR(deg) % 1.3lf\t% 1.3lf\t% 1.3lf", frame_matching_source.c_str(), toDeg(EA_tm[0].x), toDeg(EA_tm[0].y), toDeg(EA_tm[0].z));

    //Calibration
    Rotation::RotationMatrix R_calibration = Rotation::dot(R_cam, R_tm.T());
    // Rotation::Quaternion quat_calibration = R_calibration.ToQuaternion();

    std::vector<Rotation::EulerAngles> EA_final = R_calibration.ToEulerAngles();
    tf2::Quaternion quat;
    quat.setRPY(EA_final[0].x, EA_final[0].y, EA_final[0].z);
    transformStamped.transform.rotation.x = quat.x();
    transformStamped.transform.rotation.y = quat.y();
    transformStamped.transform.rotation.z = quat.z();
    transformStamped.transform.rotation.w = quat.w();
    static_broadcaster.sendTransform(transformStamped);
    ROS_INFO("--[Set] %s->%s \tR(deg) % 1.3lf\t% 1.3lf\t% 1.3lf", frame_calibration_target.c_str(), frame_matching_source.c_str(), toDeg(EA_final[0].x), toDeg(EA_final[0].y), toDeg(EA_final[0].z));

    while (checkTransform(frame_calibration_target, frame_matching_source, transformStamped) == false)
    { ros::spinOnce(); };

    /*find tf translation next*/
    ROS_INFO("--Find translation--");
    trans_matched = lookupTransform(frame_matched_target, frame_matched_marker);
    ROS_INFO("--[Get] %s->aruco \tT(Meter)[%lf,%lf,%lf]", frame_matched_target.c_str(), trans_matched.transform.translation.x, trans_matched.transform.translation.y, trans_matched.transform.translation.z);

    trans_matching = lookupTransform(frame_calibration_target, frame_matching_marker);
    ROS_INFO("--[Get] %s->aruco \tT(Meter)[%lf,%lf,%lf]", frame_matching_source.c_str(), trans_matching.transform.translation.x, trans_matching.transform.translation.y, trans_matching.transform.translation.z);

    double dx = trans_matched.transform.translation.x - trans_matching.transform.translation.x;
    double dy = trans_matched.transform.translation.y - trans_matching.transform.translation.y;
    double dz = trans_matched.transform.translation.z - trans_matching.transform.translation.z;
    transformStamped.transform.translation.x = dx;
    transformStamped.transform.translation.y = dy;
    transformStamped.transform.translation.z = dz;
    ROS_INFO("--[Set] %s->%s \tT(Meter)[%lf,%lf,%lf]", frame_calibration_target.c_str(), frame_matching_source.c_str(), dx, dy, dz);

    //use same rotation
    transformStamped.transform.rotation.x = quat.x();
    transformStamped.transform.rotation.y = quat.y();
    transformStamped.transform.rotation.z = quat.z();
    transformStamped.transform.rotation.w = quat.w();
    ROS_INFO("--[Set] %s->%s \tR(deg) % 1.3lf\t% 1.3lf\t% 1.3lf", frame_calibration_target.c_str(), frame_matching_source.c_str(), toDeg(EA_final[0].x), toDeg(EA_final[0].y), toDeg(EA_final[0].z));
    static_broadcaster.sendTransform(transformStamped);
    ros::spinOnce();
    FLAG.tf_calibration_upperCam2 = true;
    ROS_INFO("\033[0;33m===Calibration Done===\033[0m");
}

#pragma endregion ___TF transform___

void MainWindow::on_pushButton_ros_spin_clicked()
{
}

#define C1 "\033[0;33m"
#define C0 "\033[0m"
void MainWindow::on_pushButton_test1_clicked()
{
    TmTcp tcp(10, 10, 10, 3, 3, 3, "mm", "deg");
    ROS_INFO("%s", tcp.ToCmdString().c_str());

    // ROS_INFO_STREAM(C1 + "bbb" + C0);
    // string str = "給我一腳給我";
    // string find = "一腳";
    // int index = find_text(str, find);
    // ROS_INFO("index:%d", index);
    // int size = find.size();
    // ROS_INFO("size find:%d", size);
    // str.replace(6, size, "椅腳");
    // ROS_INFO("index:%s", str.c_str());
}
