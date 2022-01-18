#include "robot_vision/RotationTools.h"
// using namespace Rotation;

/*Quaternion*/
Rotation::Quaternion::Quaternion(double _x, double _y, double _z, double _w)
{
    x = _x;
    y = _y;
    z = _z;
    w = _w;
}
Rotation::Quaternion::Quaternion(geometry_msgs::Quaternion tf2_quat)
{
    x = tf2_quat.x;
    y = tf2_quat.y;
    z = tf2_quat.z;
    w = tf2_quat.w;
}
Rotation::RotationMatrix Rotation::Quaternion::ToRotationMatrix()
{
    double R[3][3] = {
        { 1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w) },
        { 2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w) },
        { 2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y) }
    };
    return Rotation::RotationMatrix(R);
}

/*EulerAngles*/
Rotation::EulerAngles::EulerAngles(double roll, double pitch, double yaw, Unit u, Order ord)
{
    order = ord;
    if (u == rad)
    {
        x = roll;
        y = pitch;
        z = yaw;
    }
    else if (u == deg)
    {
        x = roll / 180 * M_PI;
        y = pitch / 180 * M_PI;
        z = yaw / 180 * M_PI;
    }
}
Rotation::EulerAngles::EulerAngles(double roll, double pitch, double yaw, Unit u)
{
    order = zyx;
    if (u == rad)
    {
        x = roll;
        y = pitch;
        z = yaw;
    }
    else if (u == deg)
    {
        x = roll / 180 * M_PI;
        y = pitch / 180 * M_PI;
        z = yaw / 180 * M_PI;
    }
}
Rotation::EulerAngles::EulerAngles(double roll, double pitch, double yaw)
{
    order = zyx;
    x = roll;
    y = pitch;
    z = yaw;
}

Rotation::RotationMatrix Rotation::EulerAngles::ToRotationMatrix()
{
    if (order != zyx)
        throw std::runtime_error("Order needs z-y-x");

    double rotationMatrix[3][3] = {
        { cos(y) * cos(z), sin(x) * sin(y) * cos(z) - cos(x) * sin(z), cos(x) * sin(y) * cos(z) + sin(x) * sin(z) },
        { cos(y) * sin(z), sin(x) * sin(y) * sin(z) + cos(x) * cos(z), cos(x) * sin(y) * sin(z) - sin(x) * cos(z) },
        { -sin(y), sin(x) * cos(y), cos(x) * cos(y) },
    };
    return Rotation::RotationMatrix(rotationMatrix);
}

/*RotationMatrix*/
Rotation::RotationMatrix::RotationMatrix(cv::Mat matrix, Order ord)
{
    order = ord;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
        {
            R[i][j] = matrix.at<double>(i, j);
        }
}
Rotation::RotationMatrix::RotationMatrix(cv::Mat matrix)
{
    order = zyx;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
        {
            R[i][j] = matrix.at<double>(i, j);
        }
}
Rotation::RotationMatrix::RotationMatrix(double matrix[3][3], Order ord)
{
    order = ord;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
        {
            R[i][j] = matrix[i][j];
        }
}
Rotation::RotationMatrix::RotationMatrix(double matrix[3][3])
{
    order = zyx;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
        {
            R[i][j] = matrix[i][j];
        }
}

Rotation::RotationMatrix Rotation::RotationMatrix::transpos()
{
    double tmp;
    RotationMatrix RT(R);
    // RT.R[0][0] = R[0][0];
    // RT.R[1][1] = R[1][1];
    // RT.R[2][2] = R[2][2];

    RT.R[0][1] = R[1][0];
    RT.R[1][0] = R[0][1];
    RT.R[0][2] = R[2][0];
    RT.R[2][0] = R[0][2];
    RT.R[1][2] = R[2][1];
    RT.R[2][1] = R[1][2];

    // for (i = 0; i < 3; i++)
    //     for (j = 0; j < 3; j++)
    //     {
    //         if (j > i)
    //         {
    //             tmp = R[i][j];
    //             R[i][j] = R[j][i];
    //             R[j][i] = tmp;
    //         }
    //     }
    return RT;
}
Rotation::RotationMatrix Rotation::RotationMatrix::T()
{
    return transpos();
}

std::vector<Rotation::EulerAngles> Rotation::RotationMatrix::ToEulerAngles()
{
    if (order != Rotation::zyx)
        throw std::runtime_error("Rotation Matrix order needs z-y-x");

    double m20 = R[2][0];
    double m21 = R[2][1];
    double m22 = R[2][2];
    double m10 = R[1][0];
    double m00 = R[0][0];

    double x1, y1, z1;

    y1 = -asin(m20);
    double cos_y = cos(y1);
    x1 = atan2(m21 / cos_y, m22 / cos_y);
    z1 = atan2(m10 / cos_y, m00 / cos_y);

    double x2, y2, z2;

    y2 = M_PI - y1;
    x2 = atan2(m21 / cos_y, m22 / cos_y);
    z2 = atan2(m10 / cos_y, m00 / cos_y);

    std::vector<Rotation::EulerAngles> ea;
    ea.push_back(Rotation::EulerAngles(x1, y1, z1));
    ea.push_back(Rotation::EulerAngles(x2, y2, z2));

    return ea;
}
Rotation::Quaternion Rotation::RotationMatrix::ToQuaternion()
{
    double x, y, z, w;
    // qw = √(1 + m00 + m11 + m22) / 2
    //qx = (m21 - m12) / (4 * qw)
    //qy = (m02 - m20) / (4 * qw)
    //qz = (m10 - m01) / (4 * qw)

    w = sqrt(1.0 + R[0][0] + R[1][1] + R[2][2]) / 2.0;
    x = (R[2][1] - R[1][2]) / (4 * w);
    y = (R[0][2] - R[2][0]) / (4 * w);
    z = (R[1][0] - R[0][1]) / (4 * w);

    // double w = sqrt(std::max(0.0, 1 + R[0][0] + R[1][1] + R[2][2])) / 2.0;
    // double x = sqrt(std::max(0.0, 1 + R[0][0] - R[1][1] - R[2][2])) / 2.0;
    // double y = sqrt(std::max(0.0, 1 - R[0][0] + R[1][1] - R[2][2])) / 2.0;
    // double z = sqrt(std::max(0.0, 1 - R[0][0] - R[1][1] + R[2][2])) / 2.0;
    // return Rotation::Quaternion(x, y, z, w);

    // float trace = R[0][0] + R[1][1] + R[2][2];
    // if (trace > 0)
    // {
    //     double s = 0.5 / sqrt(trace + 1.0);
    //     w = 0.25 / s;
    //     x = (R[2][1] - R[1][2]) * s;
    //     y = (R[0][2] - R[2][0]) * s;
    //     z = (R[1][0] - R[0][1]) * s;
    // }
    // else
    // {
    //     if (R[0][0] > R[1][1] && R[0][0] > R[2][2])
    //     {
    //         double s = 2.0f * sqrt(1.0f + R[0][0] - R[1][1] - R[2][2]);
    //         w = (R[2][1] - R[1][2]) / s;
    //         x = 0.25 * s;
    //         y = (R[0][1] + R[1][0]) / s;
    //         z = (R[0][2] + R[2][0]) / s;
    //     }
    //     else if (R[1][1] > R[2][2])
    //     {
    //         double s = 2.0f * sqrt(1.0f + R[1][1] - R[0][0] - R[2][2]);
    //         w = (R[0][2] - R[2][0]) / s;
    //         x = (R[0][1] + R[1][0]) / s;
    //         y = 0.25 * s;
    //         z = (R[1][2] + R[2][1]) / s;
    //     }
    //     else
    //     {
    //         float s = 2.0f * sqrtf(1.0f + R[2][2] - R[0][0] - R[1][1]);
    //         w = (R[1][0] - R[0][1]) / s;
    //         x = (R[0][2] + R[2][0]) / s;
    //         y = (R[1][2] + R[2][1]) / s;
    //         z = 0.25f * s;
    //     }
    // }
    return Rotation::Quaternion(x, y, z, w);

    double b1_squared = 0.25 * (1.0 + R[1][1] + R[2][2] + R[3][3]);
    if (b1_squared >= 0.25)
    {
        // Equation (164)
        double b1 = sqrt(b1_squared);

        double over_b1_4 = 0.25 / b1;
        double b2 = (R[3][2] - R[2][3]) * over_b1_4;
        double b3 = (R[1][3] - R[3][1]) * over_b1_4;
        double b4 = (R[2][1] - R[1][2]) * over_b1_4;

        return Rotation::Quaternion(b1, b2, b3, b4);
    }
}

/*Function*/
std::vector<Rotation::EulerAngles> Rotation::rotationMatrix2eulerAngles(Rotation::RotationMatrix rotationMatrix)
{

    // double m20 = rotationMatrix.at<double>(2, 0);
    // double m21 = rotationMatrix.at<double>(2, 1);
    // double m22 = rotationMatrix.at<double>(2, 2);
    // double m10 = rotationMatrix.at<double>(1, 0);
    // double m00 = rotationMatrix.at<double>(0, 0);
    if (rotationMatrix.order != Rotation::zyx)
        throw std::runtime_error("Rotation Matrix order needs z-y-x");

    double m20 = rotationMatrix.R[2][0];
    double m21 = rotationMatrix.R[2][1];
    double m22 = rotationMatrix.R[2][2];
    double m10 = rotationMatrix.R[1][0];
    double m00 = rotationMatrix.R[0][0];

    double x, y, z;

    y = -asin(m20);
    double cos_y = cos(y);
    x = atan2(m21 / cos_y, m22 / cos_y);
    z = atan2(m10 / cos_y, m00 / cos_y);

    double x2, y2, z2;

    y2 = M_PI - y;
    x2 = atan2(m21 / cos_y, m22 / cos_y);
    z2 = atan2(m10 / cos_y, m00 / cos_y);

    std::vector<Rotation::EulerAngles> ea;
    ea.push_back(Rotation::EulerAngles(x, y, z));
    ea.push_back(Rotation::EulerAngles(x2, y2, z2));

    return ea;
}
Rotation::RotationMatrix eulerAngles2rotationMatrix(Rotation::EulerAngles eulerAngles)
{
    if (eulerAngles.order != Rotation::zyx)
        throw std::runtime_error("Rotation Matrix order needs z-y-x");

    double x = eulerAngles.x;
    double y = eulerAngles.y;
    double z = eulerAngles.z;
    double rotationMatrix[3][3] = {
        { cos(y) * cos(z), sin(x) * sin(y) * cos(z) - cos(x) * sin(z), cos(x) * sin(y) * cos(z) + sin(x) * sin(z) },
        { cos(y) * sin(z), sin(x) * sin(y) * sin(z) + cos(x) * cos(z), cos(x) * sin(y) * sin(z) - sin(x) * cos(z) },
        { -sin(y), sin(x) * cos(y), cos(x) * cos(y) },
    };
    return Rotation::RotationMatrix(rotationMatrix);


    // cv::Mat rotationMatrix = (cv::Mat_<float>(3, 3) << 0, 0, 0, 0, 0, 0, 0, 0, 0);
    // return Rotation::RotationMatrix((cv::Mat_<float>(3, 3) << 0, 0, 0, 0, 0, 0, 0, 0, 0));
}

Rotation::RotationMatrix Rotation::rotateAxisX(Rotation::RotationMatrix rotationMatrix, double rad)
{
    if (rotationMatrix.order != Rotation::zyx)
        throw std::runtime_error("Rotation Matrix order needs z-y-x");

    double Rx[3][3] = {
        { 1, 0, 0 },
        { 0, cos(rad), -sin(rad) },
        { 0, sin(rad), cos(rad) }
    };

    double results[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
        {
            for (int u = 0; u < 3; u++)
                results[i][j] += rotationMatrix.R[i][u] * Rx[u][j];
        }

    return Rotation::RotationMatrix(results);
}
Rotation::RotationMatrix Rotation::rotateAxisY(Rotation::RotationMatrix rotationMatrix, double rad)
{
    if (rotationMatrix.order != Rotation::zyx)
        throw std::runtime_error("Rotation Matrix order needs z-y-x");

    double Ry[3][3] = {
        { 1, 0, 0 },
        { 0, cos(rad), -sin(rad) },
        { 0, sin(rad), cos(rad) }
    };

    double results[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
        {
            for (int u = 0; u < 3; u++)
                results[i][j] += rotationMatrix.R[i][u] * Ry[u][j];
        }

    return Rotation::RotationMatrix(results);
}
Rotation::RotationMatrix Rotation::rotateAxisZ(Rotation::RotationMatrix rotationMatrix, double rad)
{
    if (rotationMatrix.order != Rotation::zyx)
        throw std::runtime_error("Rotation Matrix order needs z-y-x");

    double Rz[3][3] = {
        { 1, 0, 0 },
        { 0, cos(rad), -sin(rad) },
        { 0, sin(rad), cos(rad) }
    };

    double results[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
        {
            for (int u = 0; u < 3; u++)
                results[i][j] += rotationMatrix.R[i][u] * Rz[u][j];
        }

    return Rotation::RotationMatrix(results);
}

Rotation::RotationMatrix Rotation::dot(Rotation::RotationMatrix M1, Rotation::RotationMatrix M2)
{
    double results[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
        {
            for (int u = 0; u < 3; u++)
                results[i][j] += M1.R[i][u] * M2.R[u][j];
        }
    return Rotation::RotationMatrix(results);
}

double toDeg(double rad)
{
    return rad * 180 / M_PI;
}
double toRad(double deg)
{
    return deg * M_PI / 180;
}