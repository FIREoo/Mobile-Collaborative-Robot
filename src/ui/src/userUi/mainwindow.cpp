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
// mine
#include "myThreadHandle.h"
#include <robot_vision/RotationTools.h>

using std::string;
#pragma endregion ___include___

#define C033 "\033[0;33m"
#define C094 "\033[0;94m"
#define C043 "\033[0;43m"//yellow back
#define C042 "\033[0;42m"//green back
#define C0 "\033[0m"


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
        ROS_INFO_STREAM("\033[0;35m[subscribe]\033[0m " + ops.topic);
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

    realMove = false;
    ui->label_real_move->setText(setStringColor("Simulate", "#08783c"));

    //TM pose joint init
    tmj_outHome = TmJoint(0, 60, -120, -30, 90, -180, "deg");
    tmj_front = TmJoint(-10.5, 8.7, -11.5, -77.8, 34.6, -180, "deg");//p[-500,-300,300,90,0,-45]
    //hacked mode
    //combobox
    ui->comboBox_hack_voice_cmd->clear();
    ui->comboBox_hack_voice_cmd->addItem("");
    ui->comboBox_hack_voice_cmd->addItem("給我三角椅面");
    ui->comboBox_hack_voice_cmd->addItem("給我方形椅面");
    ui->comboBox_hack_voice_cmd->addItem("給我椅腳");
    ui->comboBox_hack_voice_cmd->addItem("給我螺絲");
    ui->comboBox_hack_voice_cmd->addItem("給我六角板手");
    ui->comboBox_hack_voice_cmd->addItem("扶著椅面");
    ui->comboBox_hack_voice_cmd->addItem("幫我吸塵");
    ui->comboBox_hack_voice_cmd->addItem("幫我上漆");


    //@subscriber map
    //mapStudent.insert(pair<string, string>("r000", "student_zero"));
    map_sub_topic["/rgb/image_raw"] = ros::Subscriber();
    // map_sub_topic["/img_process/image/mixMask"] = ros::Subscriber();
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

void MainWindow::on_checkBox_real_move_stateChanged(int arg1)
{
    if (arg1 == 0)
    {
        ui->label_real_move->setText(setStringColor("Simulate", "#08783c"));
        ROS_INFO_STREAM("\033[7;32mSimulate" << C0);
        realMove = false;
    }
    else
    {
        ui->label_real_move->setText(setStringColor("Real Move!!", "#782f08"));
        ROS_INFO_STREAM("\033[7;33mReal Move!!" << C0);
        realMove = true;
    }
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
void MainWindow::addItem(QStringListModel *theModel, QString str, QBrush brush)
{
    theModel->insertRow(theModel->rowCount());                       //在尾部插入一空行
    QModelIndex index = theModel->index(theModel->rowCount() - 1, 0);//獲取最後一行
    theModel->setData(index, str, Qt::DisplayRole);                  //設置顯示文字
    theModel->setData(index, QColor(255, 0, 0), Qt::TextColorRole);
    ROS_INFO("in");
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
            ROS_INFO("\033[2;33m%s\033[0m", ex.what());
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
    tf_loop_set_tm = false;
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
    SubscribeThreadHandle *thHandle = new SubscribeThreadHandle("/rgb/image_raw", 1, &MainWindow::imgCallback, this);
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


void MainWindow::on_pushButton_reload_image_topic_clicked()
{
    ui->comboBox_observe_1->clear();
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    for (ros::master::V_TopicInfo::iterator it = master_topics.begin(); it != master_topics.end(); it++)
    {
        const ros::master::TopicInfo &info = *it;
        if (info.datatype == "sensor_msgs/Image")
        {
            ui->comboBox_observe_1->addItem(QString::fromStdString(info.name));
        }
    }
}
void MainWindow::on_btn_getImage_2_clicked()
{
    QString qstr = ui->comboBox_observe_1->currentText();
    std::string image_topic = qstr.toStdString();
    map_sub_topic[image_topic] = ros::Subscriber();
    QThread *thread = new QThread();
    SubscribeThreadHandle *thHandle = new SubscribeThreadHandle(image_topic, 1, &MainWindow::img2Callback, this);
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
    QString qstr = ui->comboBox_observe_1->currentText();
    std::string image_topic = qstr.toStdString();
    auto iter = map_sub_topic.find(image_topic);//@subscriber map
    iter->second.shutdown();
    cv::Mat showImg(384, 512, CV_8UC3, cv::Scalar(0, 0, 0));
    ui->img_main_2->setPixmap(QPixmap::fromImage(QImage(showImg.data, showImg.cols, showImg.rows, showImg.step, QImage::Format_RGB888)));
}

void MainWindow::on_btn_get_tm_pos_clicked()
{
    QThread *thread = new QThread();
    SubscribeThreadHandle *thHandle = new SubscribeThreadHandle("/feedback_states", 1, &MainWindow::TM_pos_Callback, this);
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
            // tm_joint[0] = msg->joint_pos[0];
            // tm_joint[1] = msg->joint_pos[1];
            // tm_joint[2] = msg->joint_pos[2];
            // tm_joint[3] = msg->joint_pos[3];
            // tm_joint[4] = msg->joint_pos[4];
            // tm_joint[5] = msg->joint_pos[5];
            tm.axis_joint = TmJoint(msg->joint_pos[0], msg->joint_pos[1], msg->joint_pos[2], msg->joint_pos[3], msg->joint_pos[4], msg->joint_pos[5], "rad");

            ui->tm_joint_j0->setText(QString::number(tm.axis_joint.j0, 'd', 1));
            ui->tm_joint_j1->setText(QString::number(tm.axis_joint.j1, 'd', 1));
            ui->tm_joint_j2->setText(QString::number(tm.axis_joint.j2, 'd', 1));
            ui->tm_joint_j3->setText(QString::number(tm.axis_joint.j3, 'd', 1));
            ui->tm_joint_j4->setText(QString::number(tm.axis_joint.j4, 'd', 1));
            ui->tm_joint_j5->setText(QString::number(tm.axis_joint.j5, 'd', 1));
        }
        if (msg->tool_pose.size() == 6)
        {
            // tm_tcp_pos[0] = msg->tool_pose[0];
            // tm_tcp_pos[1] = msg->tool_pose[1];
            // tm_tcp_pos[2] = msg->tool_pose[2];
            // tm_tcp_pos[3] = msg->tool_pose[3];
            // tm_tcp_pos[4] = msg->tool_pose[4];
            // tm_tcp_pos[5] = msg->tool_pose[5];


            tm.tcp_pose = TmPose(msg->tool_pose[0], msg->tool_pose[1], msg->tool_pose[2], msg->tool_pose[3], msg->tool_pose[4], msg->tool_pose[5], "M", "rad");

            ui->tm_pos_x->setText(QString::number(tm.tcp_pose.x * 1000, 'd', 1));
            ui->tm_pos_y->setText(QString::number(tm.tcp_pose.y * 1000, 'd', 1));
            ui->tm_pos_z->setText(QString::number(tm.tcp_pose.z * 1000, 'd', 1));
            ui->tm_pos_Rx->setText(QString::number(tm.tcp_pose.Rx, 'd', 1));
            ui->tm_pos_Ry->setText(QString::number(tm.tcp_pose.Ry, 'd', 1));
            ui->tm_pos_Rz->setText(QString::number(tm.tcp_pose.Rz, 'd', 1));
        }
        if (msg->tcp_speed.size() == 6)
        {
            tm.tcp_speed.x = msg->tcp_speed[0];
            tm.tcp_speed.y = msg->tcp_speed[1];
            tm.tcp_speed.z = msg->tcp_speed[2];
            // tm_tcp_speed_x = msg->tcp_speed[0];
            // tm_tcp_speed_y = msg->tcp_speed[1];
            // tm_tcp_speed_z = msg->tcp_speed[2];
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
    SubscribeThreadHandle *thHandle = new SubscribeThreadHandle("/upper_camera1/image", 1, &MainWindow::img3Callback, this);
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
    SubscribeThreadHandle *thHandle = new SubscribeThreadHandle("/upper_camera2/image", 1, &MainWindow::img4Callback, this);
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

void MainWindow::on_btn_getImage_tm_cam_clicked()
{
    sendCMD("ScriptExit()");
    sleep(2);//wait capture image

    try
    {
        //Width : 2592 Height: 1944
        cv::Mat getImg = cv::imread("/home/fire/Desktop/showimage.png");
        cv::cvtColor(getImg, getImg, cv::COLOR_BGR2RGB);

        cv::Mat showImg;

        cv::resize(getImg, showImg, cv::Size(512, 288));//16:9
        cv::Mat concat(384 - 288, 512, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::vconcat(showImg, concat, showImg);//to 4:3
        ui->img_main_5->setPixmap(QPixmap::fromImage(QImage(showImg.data, showImg.cols, showImg.rows, showImg.step, QImage::Format_RGB888)));
    }
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM("imgCallback: Error");
    }
}

#pragma endregion ___Subscribe Message___


//---TM---\\.
#pragma region TM

/*TM- subfunction*/
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
    ROS_WARN("Abandon function");
    // sleep(delaySec);
    // double t = 0.005;
    // while (true)
    // {
    //     try
    //     {
    //         if (tm_tcp_speed_x > -t && tm_tcp_speed_x < t &&
    //             tm_tcp_speed_y > -t && tm_tcp_speed_y < t &&
    //             tm_tcp_speed_z > -t && tm_tcp_speed_z < t)
    //             break;

    //         sleep(1);
    //         qApp->processEvents();
    //     }
    //     catch (const std::exception &e)
    //     {
    //         ROS_ERROR_STREAM("waitTM error");
    //     }
    // }
    // ROS_INFO("TM done check.");
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

void MainWindow::on_btn_set_tf_tracker_clicked()
{
    QThread *thread = new QThread();
    std::string str = "yolo/hand";
    double x = 0;
    double y = 0;
    SubscribeThreadHandle *thHandle = new SubscribeThreadHandle(&MainWindow::publishTracker, str, x, y, this);
    thHandle->moveToThread(thread);
    connect(thread, &QThread::started, thHandle, &SubscribeThreadHandle::doFunction3);
    thread->start();
}
void MainWindow::publishTracker(const std::string target_fram, const double x_offset, const double y_offset)
{
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
            double sec = ros::Time::now().toSec() - transform_target.header.stamp.toSec();
            if (sec > 0.5)
            {
                continue;
            }
            // ROS_INFO("tf blue \tT(Meter)[%lf,%lf,%lf]", tf_cam_aruco.transform.translation.x, tf_cam_aruco.transform.translation.y, tf_cam_aruco.transform.translation.z);

            double x = transform_target.transform.translation.x;
            double y = transform_target.transform.translation.y;
            double z = transform_target.transform.translation.z;

            //offset in world/kinect frame
            x += x_offset;// M
            y += y_offset;// M

            /*Publish tf tracker*/
            // tf::Transform transform_out;
            // transform_out.setOrigin(tf::Vector3(x, y, z));
            // tf::Quaternion q;
            // q.setRPY(0, 0, 0);
            // transform_out.setRotation(q);
            // br.sendTransform(tf::StampedTransform(transform_out, transform_target.header.stamp, "kinect", "track/tracker"));

            /*Publish tf2 tracker*/
            static tf2_ros::TransformBroadcaster br;
            geometry_msgs::TransformStamped transformStamped;

            transformStamped.header.stamp = ros::Time::now();//transform_target.header.stamp;
            transformStamped.header.frame_id = "kinect";
            transformStamped.child_frame_id = "track/tracker";
            transformStamped.transform.translation.x = x;
            transformStamped.transform.translation.y = y;
            transformStamped.transform.translation.z = z;
            transformStamped.transform.rotation.x = 0;
            transformStamped.transform.rotation.y = 0;
            transformStamped.transform.rotation.z = 0;
            transformStamped.transform.rotation.w = 1;

            br.sendTransform(transformStamped);

            /*Publish tracker*/
            geometry_msgs::Pose msg_pose;
            msg_pose.position.x = x;
            msg_pose.position.y = y;
            msg_pose.position.z = z;
            pub_tracker.publish(msg_pose);

            if (flag_firstIn)
            {
                ROS_INFO("\033[0;33m=====Boardcasting=====\033[0m");
                ROS_INFO_STREAM("-- kinect->track/tracker");
                ROS_INFO_STREAM("-- from" + target_fram + "\n");
                addItem(msg_tm_listModel, "=====Boardcasting=====\n-- kinect->track/tracker\n");
                flag_firstIn = false;
            }
        }
        catch (tf2::TransformException &ex)
        {
            ROS_INFO("\033[3;93m%s\033[0m", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        rate.sleep();
    }


    ROS_INFO("Publish tracker [close].");
}


/*TM- servo*/
void MainWindow::on_btn_set_tm_pos_preTrack_clicked()
{
    sendCMD("PTP(\"CPP\",-450,-350,215,100,0,-45,100,200,0,false)", false);// mm-deg
}
void MainWindow::TM_servoOn(const string trackName, const double offset_x, const double offset_y)
{
    // safty check(need TF correction)
    // if (tf_correction_check != 3)
    // {
    //     QMessageBox::about(NULL, "Error", "TF /kinect does not transform to /world");
    //     return;
    // }
    // // safty function
    // sendCMD("StopContinueVmode()");
    // sendCMD("SuspendContinueVmode()");
    // sleep(1);
    // // start
    // sendCMD("ContinueVLine(500,10000)");
    // tf::TransformListener listener;
    // ros::ServiceClient client = nh.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");
    // ros::Rate rate(3);
    // servoing = true;
    // while (nh.ok() && servoing)
    // {
    //     tf::StampedTransform transform;
    //     try
    //     {
    //         double x = 0;
    //         double y = 0;

    //         listener.lookupTransform("/TM_robot", "/track/blue", ros::Time(0), transform);
    //         x = transform.getOrigin().x();
    //         y = transform.getOrigin().y();
    //         // x += offset_x; // M
    //         // y += offset_y; // M
    //         x += -0.1;// M
    //         y += 0.1; // M
    //         // ROS_INFO_STREAM("vel\tx = " + std::to_string(x) + " y = " + std::to_string(y));
    //         double a = 0.2;
    //         double Vx = x + (a * x * x);
    //         double Vy = y + (a * y * y);
    //         // speed limit
    //         double limit = 0.05;// 0.1
    //         Vx = (Vx > limit) ? limit : Vx;
    //         Vx = (Vx < -limit) ? -limit : Vx;
    //         Vy = (Vy > limit) ? limit : Vy;
    //         Vy = (Vy < -limit) ? -limit : Vy;

    //         tm_msgs::SendScript srv;
    //         srv.request.id = "demo";
    //         srv.request.script = "SetContinueVLine(" + std::to_string(Vx) + "," + std::to_string(Vy) + ",0,0,0,0)";
    //         if (client.call(srv))
    //         {
    //             if (srv.response.ok)
    //                 ROS_INFO_STREAM("Sent script to robot: SetContinueVLine(" + std::to_string(Vx) + "," + std::to_string(Vy) + ",0,0,0,0)");
    //             else
    //                 ROS_WARN_STREAM("Sent script to robot , but response not yet ok ");
    //         }
    //         else
    //         {
    //             ROS_ERROR_STREAM("Error send script to robot");
    //         }
    //         // sendCMD("SetContinueVLine(" + std::to_string(Vx) + "," + std::to_string(Vy) + ",0,0,0,0)");
    //     }
    //     catch (tf::TransformException ex)
    //     {
    //         sendCMD("SuspendContinueVmode()");
    //         ROS_INFO("Track pose loss...");
    //         ros::Duration(0.1).sleep();
    //     }
    //     rate.sleep();
    // }
    // sendCMD("StopContinueVmode()");
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

void MainWindow::Test(int value)
{
    // ROS_INFO("test!!");
    // emit cc(value);
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
void MainWindow::TM_newServoOn()
{
    //safty check(need TF correction)
    if (!FLAG.tf_m03Robot_inVision)
    {
        QMessageBox::about(NULL, "Error", "TF /TM_robot/base does not transform to /world");
        return;
    }
    addItem(msg_tm_listModel, "=====Servoing=====\n-- servo point : track/tracker\n");

    if (realMove)
    {
        //safty function
        tm.sendScript("StopContinueVmode()");
        tm.sendScript("SuspendContinueVmode()");
        tm.sendScript("ContinueVLine(500,10000)");
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
            double sec = ros::Time::now().toSec() - transformStamped2.header.stamp.toSec();

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
                if (sec > 0.5)
                {
                    ROS_INFO("\033[0;32m[Semulation]\033[0m\033[2mLoss tracker\033[0m");
                    tm.sendScript("SuspendContinueVmode()", false);
                }
                else
                {
                    tm.sendScript("SetContinueVLine(" + std::to_string(Vx) + "," + std::to_string(Vy) + ",0,0,0,0)");
                }
            }
            else
            {
                if (sec > 1)
                {
                    ROS_INFO("\033[0;32m[Semulation]\033[0m\033[2mLoss tracker\033[0m");
                }
                else
                {
                    // ROS_INFO("(dx,dy)=(% 1.3lf,% 1.3lf)", x, y);
                    ROS_INFO("\033[0;32m[Semulation]\033[0m\033[2mSetContinueVLine(% 1.3lf,% 1.3lf,0,0,0,0)\033[0m  (% 1.3lf,% 1.3lf)", Vx, Vy, x, y);
                }
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
void MainWindow::on_btn_set_tm_servo_stop_clicked()
{
    servoing = false;
    sendCMD("StopContinueVmode()");
    // map_sub_topic["/track/tracker/pose"].shutdown();
}

/*TM- msg*/
void MainWindow::on_pushButton_clearTmMsg_clicked()
{
    clearItem(msg_tm_listModel);
}
void MainWindow::on_pushButton_clear_execute_clicked()
{
    clearItem(msg_listModel);
}

/*TM- pose cmd*/
void MainWindow::on_btn_set_tm_pose_clicked()
{
    double x = ui->lineEdit_tm_cmd_0->text().toDouble();
    double y = ui->lineEdit_tm_cmd_1->text().toDouble();
    double z = ui->lineEdit_tm_cmd_2->text().toDouble();
    double Rx = ui->lineEdit_tm_cmd_3->text().toDouble();
    double Ry = ui->lineEdit_tm_cmd_4->text().toDouble();
    double Rz = ui->lineEdit_tm_cmd_5->text().toDouble();
    tm.moveTo(TmPose(x, y, z, Rx, Ry, Rz, "mm", "deg"), 50, true, true);
}
void MainWindow::on_btn_set_tm_joint_clicked()
{
    double j0 = ui->lineEdit_tm_cmd_0->text().toDouble();
    double j1 = ui->lineEdit_tm_cmd_1->text().toDouble();
    double j2 = ui->lineEdit_tm_cmd_2->text().toDouble();
    double j3 = ui->lineEdit_tm_cmd_3->text().toDouble();
    double j4 = ui->lineEdit_tm_cmd_4->text().toDouble();
    double j5 = ui->lineEdit_tm_cmd_5->text().toDouble();
    tm.moveTo(TmJoint(j0, j1, j2, j3, j4, j5, "deg"), 50, true, true);
}
void MainWindow::on_btn_set_tm_copy_pose_clicked()
{
    ui->lineEdit_tm_cmd_0->setText(QString::number(tm.tcp_pose.x * 1000, 'd', 1));
    ui->lineEdit_tm_cmd_1->setText(QString::number(tm.tcp_pose.y * 1000, 'd', 1));
    ui->lineEdit_tm_cmd_2->setText(QString::number(tm.tcp_pose.z * 1000, 'd', 1));
    ui->lineEdit_tm_cmd_3->setText(QString::number(tm.tcp_pose.Rx, 'd', 1));
    ui->lineEdit_tm_cmd_4->setText(QString::number(tm.tcp_pose.Ry, 'd', 1));
    ui->lineEdit_tm_cmd_5->setText(QString::number(tm.tcp_pose.Rz, 'd', 1));
}
void MainWindow::on_btn_set_tm_copy_joint_clicked()
{
    ui->lineEdit_tm_cmd_0->setText(QString::number(tm.axis_joint.j0, 'd', 1));
    ui->lineEdit_tm_cmd_1->setText(QString::number(tm.axis_joint.j1, 'd', 1));
    ui->lineEdit_tm_cmd_2->setText(QString::number(tm.axis_joint.j2, 'd', 1));
    ui->lineEdit_tm_cmd_3->setText(QString::number(tm.axis_joint.j3, 'd', 1));
    ui->lineEdit_tm_cmd_4->setText(QString::number(tm.axis_joint.j4, 'd', 1));
    ui->lineEdit_tm_cmd_5->setText(QString::number(tm.axis_joint.j5, 'd', 1));
}



/*PTP*/
void MainWindow::on_btn_set_tm_pos_1_clicked()
{
    tm.moveTo(tmj_outHome, 100, true, true);
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

void MainWindow::on_btn_set_tm_pos_setTF_clicked()
{
    TmJoint joint_0(1.17, 17.25, -115.47, -85.09, 47.06, -189.51, "deg");
    TmJoint joint_1(1.17, 7.25, -115.47, -85.09, 47.06, -189.51, "deg");
    TmPose tcp0(-450, -350, 215, 100, 0, -45, "mm", "deg");
    TmPose tcp1(-500, -300, 300, 90, 0, -45, "mm", "deg");

    // tm.moveTo(joint_0, 50, true, true);
    tm.moveTo(joint_1, 100, true, true);
    // tm.moveTo(tcp0, 50, true, true);
    tm.moveTo(tcp1, 100, true, true);
}

void MainWindow::matchCalibration(std::string frame_calibration_target, std::string frame_matched_target, std::string frame_matched_marker, std::string frame_matching_source, std::string frame_matching_marker)
{
    ROS_INFO("\033[0;33m=====Calibration=====\033[0m");
    ROS_INFO("--Match upper_camera1/aruco  and kinect/aruco");
    addItem(msg_tm_listModel, "=====Calibration=====\n--Match upper_camera1/aruco  and kinect/aruco\n");

    //transform target -> source
    // std::string frame_calibration_target = "world";
    // std::string frame_matched_target = "kinect";
    // std::string frame_matched_marker = "kinect/track/aruco/4x4/0";
    geometry_msgs::TransformStamped trans_matched;//the matched transform
    // std::string frame_matching_source = "TM_robot/base";
    // std::string frame_matching_marker = "TM_robot/tcp/aruco";
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

void MainWindow::on_comboBox_hack_voice_cmd_currentIndexChanged(int index)
{
    QString str = ui->comboBox_hack_voice_cmd->itemText(index);
    ui->textEdit_voice->setText(str);
}


void MainWindow::on_pushButton_ros_spin_clicked()
{
    qApp->processEvents();
    TmJoint joint_0(1.17, 17.25, -115.47, -85.09, 47.06, -189.51, "deg");
    TmJoint joint_1(1.17, 7.25, -115.47, -85.09, 47.06, -189.51, "deg");
    TmPose tcp0(-450, -350, 215, 100, 0, -45, "mm", "deg");
    TmPose tcp1(-500, -300, 300, 90, 0, -45, "mm", "deg");


    tm.checkConnect();

    tm.moveTo(joint_0, 50, true, true);
    tm.moveTo(joint_1, 50, true, true);
    tm.moveTo(tcp0, 50, true, true);
    tm.moveTo(tcp1, 50, true, true);
}


void MainWindow::on_pushButton_test1_clicked()
{
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();//transform_target.header.stamp;
    transformStamped.header.frame_id = "upper_camera2/track/aruco/5x5/5";
    transformStamped.child_frame_id = "upper_camera2/track/aruco/5x5/5/TM_base";
    transformStamped.transform.translation.x = -0.225;
    transformStamped.transform.translation.y = 0.21;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, M_PI + M_PI_4);
    transformStamped.transform.rotation.x = quat.x();
    transformStamped.transform.rotation.y = quat.y();
    transformStamped.transform.rotation.z = quat.z();
    transformStamped.transform.rotation.w = quat.w();
    br.sendTransform(transformStamped);

    geometry_msgs::TransformStamped transform_target;
    transform_target = lookupTransform("world", "upper_camera2/track/aruco/5x5/5/TM_base");

    static tf2_ros::StaticTransformBroadcaster static_broadcaster;

    transform_target.header.stamp = ros::Time::now();
    transform_target.header.frame_id = "world";
    transform_target.child_frame_id = "TM_robot/base";
    static_broadcaster.sendTransform(transform_target);
}
