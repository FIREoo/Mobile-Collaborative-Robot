#ifndef ROTATIONTOOLS_H
#define ROTATIONTOOLS_H

#include <iostream>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/console.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace Rotation
{
    enum Unit
    {
        deg,
        rad
    };
    enum Order
    {
        zyx
    };
    class RotationMatrix;
    class EulerAngles;
    class Quaternion;

    class Quaternion
    {
      private:
        /* data */
      public:
        Quaternion(double _x, double _y, double _z, double _w);
        Quaternion(geometry_msgs::Quaternion tf2_quat);
        double x;
        double y;
        double z;
        double w;

        RotationMatrix ToRotationMatrix();
    };
    struct RotationVector
    {
        double x;
        double y;
        double z;
    };


    class RotationMatrix
    {
      private:
        /* data */
      public:
        RotationMatrix(cv::Mat matrix);
        RotationMatrix(cv::Mat matrix, Order ord);
        RotationMatrix(double matrix[3][3]);
        RotationMatrix(double matrix[3][3], Order ord);


        Order order;
        double R[3][3];
        RotationMatrix transpos();
        RotationMatrix T();

        std::vector<EulerAngles> ToEulerAngles();
        Quaternion ToQuaternion();
    };

    class EulerAngles
    {
      private:
        /* data */
      public:
        EulerAngles(double roll, double pitch, double yaw);
        EulerAngles(double roll, double pitch, double yaw, Unit u);
        EulerAngles(double roll, double pitch, double yaw, Unit u, Order ord);

        Order order;
        double x;
        double y;
        double z;

        RotationMatrix ToRotationMatrix();
    };


    std::vector<EulerAngles> rotationMatrix2eulerAngles(Rotation::RotationMatrix rotationMatrix);
    RotationMatrix eulerAngles2rotationMatrix(EulerAngles eulerAngles);

    Rotation::RotationMatrix rotateAxisX(Rotation::RotationMatrix rotationMatrix, double rad);
    Rotation::RotationMatrix rotateAxisY(Rotation::RotationMatrix rotationMatrix, double rad);
    Rotation::RotationMatrix rotateAxisZ(Rotation::RotationMatrix rotationMatrix, double rad);
    Rotation::RotationMatrix dot(Rotation::RotationMatrix M1, Rotation::RotationMatrix M2);


};// namespace Rotation

extern double toDeg(double rad);
extern double toRad(double deg);


#endif