#ifndef MYTHREADHANDLE_H
#define MYTHREADHANDLE_H

#include "mainwindow.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ui_mainwindow.h"
#include <QMainWindow>
#include <sensor_msgs/Joy.h>

#include "geometry_msgs/PointStamped.h"
#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"

// template <class M, class T>
class SubscribeThreadHandle : public QObject
{
  public:
    template<class M, class T>
    SubscribeThreadHandle(std::string topic, uint32_t queue_size, void (T::*fp)(const boost::shared_ptr<M const> &), T *obj);
    SubscribeThreadHandle(void (MainWindow::*pFunc)(), MainWindow *obj);
    SubscribeThreadHandle(void (MainWindow::*pFunc)(double, double), double v0, double v1, MainWindow *obj);
    SubscribeThreadHandle(void (MainWindow::*pFunc)(std::string, double, double), std::string s0, double v0, double v1, MainWindow *obj);
    // template<typename A, typename B, typename C>
    // SubscribeThreadHandle(void (MainWindow::*pFunc)(A, B, C), A v0, B v1, C v2, MainWindow *obj);
    ros::SubscribeOptions ops;
    ~SubscribeThreadHandle();
  public slots:
    void subscribe();
    void doFunction();
    void doFunction2();
    void doFunction3();

  private:
    MainWindow *_obj;
    std::string _topic;
    std::string S0;
    double V0;
    double V1;


    void (MainWindow::*_pFunc)();

    void (MainWindow::*_pFunc2)(double, double);
    void (MainWindow::*_pFunc3)(std::string, double, double);
};

#endif// MYTHREADHANDLE_H