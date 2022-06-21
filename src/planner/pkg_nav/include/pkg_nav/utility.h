#ifndef UTILITY_H
#define UTILITY_H
#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <cmath>
#include <geometry_msgs/Point.h>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

namespace np
{
    double caculate_distance(geometry_msgs::Pose pos1, geometry_msgs::Pose pos2)
    {
        double dx = pos1.position.x - pos2.position.x;
        double dy = pos1.position.y - pos2.position.y;
        double dz = pos1.position.z - pos2.position.z;
        return sqrt(dx * dx + dy * dy + dz * dz);
    }

    bool LineSpace(double start, double end, unsigned count, std::vector<double> &vec)
    {
        vec.clear();
        if (count < 2)
        {
            ROS_FATAL("The input count is zero !");
            return false;
        }
        if (count == 2)
        {
            vec.push_back(start);
            vec.push_back(end);
            return true;
        }
        double dt = (end - start) / (count - 1);
        for (size_t i = 0; i < count; i++)
        {
            vec.push_back(start + i * dt);
        }
        return true;
    }

    bool Arange(double start, double end, double step, std::vector<double> &vec, bool endPoint = true) //np.arange(0,5.1,0.1) #[0,5] step = 0.1
    {
        if (step == 0 || start == end)
        {
            ROS_FATAL("step == 0 || start == end");
            return false;
        }

        vec.clear();
        if (start < end)
        {
            if(step < 0)
            {
                ROS_FATAL("step < 0,if end > start then step > 0");
                return false;
            }
            double sum = start;
            if (endPoint)
            {
                while (sum <= end)
                {
                    vec.push_back(sum);
                    sum += step;
                }
            }
            else
            {
                while (sum < end)
                {
                    vec.push_back(sum);
                    sum += step;
                }
            }
            return true;
        }
        if (start > end)
        {
            if (step > 0)
            {
                ROS_FATAL("step > 0 ,if end < start then step < 0");
                return false;
            }
            double sum = start;
            if (endPoint)
            {
                while (sum >= end)
                {
                    vec.push_back(sum);
                    sum += step;
                }
            }
            else
            {
                while (sum > end)
                {
                    vec.push_back(sum);
                    sum += step;
                }
            }
            return true;
        }
       
    }

    template <class T>
    int length(T &data)
    {
        return sizeof(data) / sizeof(data[0]);
    }

    template <class T>
    T Min(std::vector<T> &data)
    {
        if (data.size() < 1)
        {
            ROS_FATAL("data.size() < 1");
            return 0;
        }
        T minValue = data[0];
        for (auto var : data)
        {
            if (minValue > var)
            {
                minValue = var;
            }
        }
        return minValue;
    }

    template <class T>
    T Max(std::vector<T> &data)
    {
        if (data.size() < 1)
        {
            ROS_FATAL("data.size() < 1");
            return 0;
        }
        T maxValue = data[0];
        for (auto var : data)
        {
            if (maxValue < var)
            {
                maxValue = var;
            }
        }
        return maxValue;
    }

    template <class T>
    bool ValueInVector(T value, std::vector<T> valueVec)
    {
        for (auto var : valueVec.size)
        {
            if (sqrt(value - var) < 1e-10)  //1^-10
            {
                return true;
            }
        }
        return false;
    }

    template <class T>
    int IndexGet(T value, std::vector<T> valueVec)
    {
        for (size_t i = 0; i < valueVec.size(); i++)
        {
            if (sqrt(value - valueVec[i]) < 10e-10) //10e3 //即1×10^3
            {
                // ROS_FATAL("IndexGet %d", i); 
                return i;
            }
       }
    }

    void erase_all_space(string &s)
    {
        int index = 0;
        if (!s.empty())
        {
            while ((index = s.find(' ', index)) != string::npos)
            {
                // s.erase(index, 1);
                s.replace(index, 1, "_");
            }
        }
    }

    class Line
    { 
        public:
            double A, B, C;

            Line(double A, double B, double C)
            {
                this->A = A;
                this->B = B;
                this->C = C;
            }
            Line()
            {
                ;
            }
    };

    void LineEquation(geometry_msgs::Point point1,geometry_msgs::Point point2,double &A,double &B,double &C)
    {
        //求直线方程： Ax + By + C = 0
            //         A = Y2 - Y1
            //         B = X1 - X2
            //         C = X2*Y1 - X1*Y2
        A = point2.y - point1.y;
        B = point1.x - point2.x;
        C = point2.x * point1.y - point1.x * point2.y;
    }

    void ParallelLineEquation(double A, double B, double C, double dist, double &C1,double &C2)
    {
        //求平行线方程：Ax + By + C = 0  =>  Ax + By + C1 = 0,Ax + By + C2 = 0
        //     d =|C2 - C1|/sqrt(A^2 + B^2)
        //     C1 = C1 + d*sqrt(A^2 + B^2)
        //     C2 = C1 - d*sqrt(A^2 + B^2)
        C1 = C + dist * sqrt(A * A + B * B); //inner
        C2 = C - dist * sqrt(A * A + B * B); //outer
    }

    void LineCrossPoint(double A1, double B1, double C1, double A2, double B2, double C2, geometry_msgs::Point &point)
    {
        // （2：直线交点方程 Ax + By + C1 = 0 ，Ax + By + C1 = 0
            // x = (B1 * C2 - B2 * C1) / (A1 * B2 - A2 * B1)
            // y = (A2 * C1 - A1 * C2) / (A1 * B2 - A2 * B1)
        point.x = (B1 * C2 - B2 * C1) / (A1 * B2 - A2 * B1);
        point.y = (A2 * C1 - A1 * C2) / (A1 * B2 - A2 * B1);
    }

    void GetFootPoint(double A, double B, double C, geometry_msgs::Point point0, geometry_msgs::Point &point)
    {
    // 垂足：
        //   求解两个方程：（a）、Ax + By + C = 0;（b）、(y - y0) / (x - x0) = B / A; c11 c12
        //   解得，x = (  B*B*x0  -  A*B*y0  -  A*C  ) / ( A*A + B*B );
        //                     y  =  ( -A*B*x0 + A*A*y0 - B*C  ) / ( A*A + B*B );
        point.x = (B * B * point0.x - A * B * point0.y - A * C) / (A * A + B * B);
        point.y = (-A * B * point0.x + A * A * point0.y - B * C) / (A * A + B * B);
    }

    void CalcOffsetPoint(geometry_msgs::Point point1, geometry_msgs::Point point2, geometry_msgs::Point point3, double dist, geometry_msgs::Point &pointLeft, geometry_msgs::Point &pointRight)
    {
        double A1, B1, C1, C11, C12;
        double A2, B2, C2, C21, C22;
        LineEquation(point1, point2, A1, B1, C1);
        LineEquation(point2, point3, A2, B2, C2);
        ParallelLineEquation(A1, B1, C1, dist, C11, C12);
        ParallelLineEquation(A2, B2, C2, dist, C21, C22);
        if (fabs(A1 * B2 - A2 * B1) < 1e-3)
        {
            GetFootPoint(A1, B1, C11, point2, pointLeft);
            GetFootPoint(A1, B1, C12, point2, pointRight);
            return;
        }
        LineCrossPoint(A1, B1, C11, A2, B2, C21, pointLeft);
        LineCrossPoint(A1, B1, C12, A2, B2, C22, pointRight);
    }

    bool CalcOffsetWith2Point(geometry_msgs::Point point1, geometry_msgs::Point point2, const double dist, geometry_msgs::Point &pointLeft, geometry_msgs::Point &pointRight)
    {
        //point1 == point2 return false
        geometry_msgs::Point point0; //中点
        point0.x = (point1.x + point2.x) / 2;
        point0.y = (point1.y + point2.y) / 2;
        np::CalcOffsetPoint(point1, point0, point2, dist, pointLeft, pointRight);
        return true;
    }

    // 检查以(x, y)为中心，raidus为半径的圆上是否有障碍物
    bool IsObstacleInCircle(const double x, const double y, const double raidus,const nav_msgs::OccupancyGrid &grid_map)
    {
        double robot_x;
        double robot_y;
        int index_x, index_y;
        for (double theta = -M_PI; theta < M_PI; theta += 1.0 / 6.0 * M_PI)
        {
            robot_x = x + raidus * std::cos(theta);
            robot_y = y + raidus * std::sin(theta);
            int index_x, index_y;
            Pose2Index(robot_x, robot_y, grid_map, &index_x, &index_y);
            if (!CheckPose2d(index_x, index_y, grid_map))
            {
                return true;
            }

            // for (double r = 1; r <= raidus; r += 1)
            // { 
                
            // }
        }
        return false;
    }


    // bool local_path_process
    VectorXd polynomial_fitting(VectorXd v_x, VectorXd v_y, const int m = 3) //polynomial_dimension
    {
        int len = v_x.size();
        MatrixXd A(m + 1, m + 1);
        VectorXd b(m + 1);
        for (int i = 0; i < m + 1; i++)
        {
            VectorXd temp(len);
            for (int k = 0; k < len; k++)
            {
                temp(k) = pow(v_x(k), i);
            }
            b(i) = v_y.dot(temp);
            for (int j = 0; j < m + 1; j++)
            {
                for (int k = 0; k < len; k++)
                {
                    temp(k) = pow(v_x(k), i + j);
                }
                A(i, j) = temp.sum();
            }
        }
        // std::cout << "b=\n"
        //           << b << std::endl;
        // std::cout << "A=\n"
        //           << A << std::endl;
        // VectorXd x = A.colPivHouseholderQr().solve(b);
        // std::cout << "v_a=\n"
        //           << v_a << std::endl;
        b = A.colPivHouseholderQr().solve(b);
        return b;
    }

    void SplinePlanning(float *x, float *y, int count, float *a0, float *a1, float *a2, float *a3, float begin_dy1 = 0, float end_dy1 = 0)
    {
        //分段函数的形式为 Si(x) =  a0 + a1(x-xi) + a2(x-xi)^2 + a3(x-xi)^3
        //xi为x[i]的值，xi_1为x[i+1]的值
        // float a0[count];
        // float a1[count];
        // float a2[count];
        // float a3[count];
        //中间变量
        float h[count];
        float fi[count];
        float u[count];
        float a[count];
        float d[count];
        float m[count];
        float b[count];
        float yy[count];
        float dy1[count];
        float dy2[count];
        //求h和fi
        for (size_t i = 0; i < count - 2; i++)
        {
            h[i] = x[i + 1] - x[i];
            fi[i] = (y[i + 1] - y[i]) / h[i];
        }
        for (size_t i = 1; i < count - 2; i++)
        {
            u[i] = h[i - 1] / (h[i - 1] + h[i]); //mu
            a[i] = h[i] / (h[i - 1] + h[i]);     //lambda ^A
            d[i] = 6 * (fi[i] - fi[i - 1]) / (h[i - 1] + h[i]);
        }

        //计算边界条件
        u[count - 1] = 1; //h[n] = 0
        a[0] = 1;         //h[0]=0
        d[0] = 6 * (fi[0] - begin_dy1) / h[0];
        d[count - 1] = 6 * (end_dy1 - fi[count - 2]) / h[count - 2];

        //追赶法求解M矩阵
        b[0] = a[0] / 2;
        for (size_t i = 1; i < count - 2; i++)
        {
            b[i] = a[i] / (2 - u[i] * b[i - 1]);
        }

        yy[0] = d[0] / 2;
        for (size_t i = 1; i < count - 1; i++)
        {
            yy[i] = (d[i] - u[i] * yy[i - 1]) / (2 - u[i] * b[i - 1]);
        }

        m[count - 1] = yy[count - 1];
        for (size_t i = count - 1; i > 0; i--)
        {
            m[i - 1] = yy[i - 1] - b[i - 1] * m[i];
        }

        //计算方程最终结果
        std::cout << "//计算方程最终结果" << std::endl;
        for (size_t i = 0; i < count - 2; i++)
        {
            a0[i] = y[i];
            a1[i] = fi[i] - h[i] * m[i] / 2 - h[i] * (m[i + 1] - m[i]) / 6;
            a2[i] = m[i] / 2;
            a3[i] = (m[i + 1] - m[i]) / (6 * h[i]);
            std::cout << a0[i] << std::endl;
            std::cout << a1[i] << std::endl;
            std::cout << a2[i] << std::endl;
            std::cout << a3[i] << std::endl;
        }

        dy1[0] = begin_dy1;
        dy1[count - 1] = end_dy1;
        dy2[0] = 2 * a2[0];
        for (size_t i = 1; i < count - 2; i++)
        {
            dy1[i] = a1[i - 1] + 2 * a2[i - 1] * h[i - 1] + 3 * a3[i - 1] * h[i - 1] * h[i - 1];
            dy2[i] = 2 * a2[i - 1] + 6 * a3[i - 1] * h[i - 1];
        }
        dy2[count - 1] = 2 * a2[count - 2] + 6 * a3[count - 2] * h[count - 2];
    }

#pragma region
    // bool PathFitter(std::vector<std::pair<double, double>> raw_point2d, std::vector<std::pair<double, double>> &ref_point2d)
    // {
    //     size_t count = raw_point2d.size();
    //     ref_point2d.clear();
    //     double dx0, dx1, dx2, dx3, dx4;
    //     double dy0, dy1, dy2, dy3, dy4;
    //     double d0, d1, d2, d3, d4;
    //     double d01, d02, d03, d04, d05;
    //     double x, y;
    //     int indexOffset = 3 / m_global_map.info.resolution;
    //     ref_point2d.push_back(raw_point2d[0]);
    //     for (size_t i = 0; i < count - 6 * indexOffset; i++)
    //     {
    //         dx0 = raw_point2d[i].first - raw_point2d[i + 1 * indexOffset].first;
    //         dy0 = raw_point2d[i].second - raw_point2d[i + 1 * indexOffset].second;
    //         d0 = sqrt(dx0 * dx0 + dy0 * dy0);
    //         d01 = d1;

    //         dx1 = raw_point2d[i + 1 * indexOffset].first - raw_point2d[i + 2 * indexOffset].first;
    //         dy1 = raw_point2d[i + 1 * indexOffset].second - raw_point2d[i + 2 * indexOffset].second;
    //         d1 = sqrt(dx1 * dx1 + dy1 * dy1);
    //         x = raw_point2d[i].first - raw_point2d[i + 2 * indexOffset].first;
    //         y = raw_point2d[i].second - raw_point2d[i + 2 * indexOffset].second;
    //         d02 = sqrt(x * x + y * y);

    //         dx2 = raw_point2d[i + 2 * indexOffset].first - raw_point2d[i + 3 * indexOffset].first;
    //         dy2 = raw_point2d[i + 2 * indexOffset].second - raw_point2d[i + 3 * indexOffset].second;
    //         d2 = sqrt(dx2 * dx2 + dy2 * dy2);
    //         x = raw_point2d[i].first - raw_point2d[i + 3 * indexOffset].first;
    //         y = raw_point2d[i].second - raw_point2d[i + 3 * indexOffset].second;
    //         d03 = sqrt(x * x + y * y);

    //         dx3 = raw_point2d[i + 3 * indexOffset].first - raw_point2d[i + 4 * indexOffset].first;
    //         dy3 = raw_point2d[i + 3 * indexOffset].second - raw_point2d[i + 4 * indexOffset].second;
    //         d3 = sqrt(dx3 * dx3 + dy3 * dy3);
    //         x = raw_point2d[i].first - raw_point2d[i + 4 * indexOffset].first;
    //         y = raw_point2d[i].second - raw_point2d[i + 4 * indexOffset].second;
    //         d04 = sqrt(x * x + y * y);

    //         dx4 = raw_point2d[i + 4 * indexOffset].first - raw_point2d[i + 5 * indexOffset].first;
    //         dy4 = raw_point2d[i + 4 * indexOffset].second - raw_point2d[i + 5 * indexOffset].second;
    //         d4 = sqrt(dx4 * dx4 + dy4 * dy4);
    //         x = raw_point2d[i].first - raw_point2d[i + 5 * indexOffset].first;
    //         y = raw_point2d[i].second - raw_point2d[i + 5 * indexOffset].second;
    //         d05 = sqrt(x * x + y * y);

    //         if (d05 < (d0 + d1 + d2 + d3 + d4)) //5
    //         {
    //             ref_point2d.push_back(raw_point2d[i + 5 * indexOffset]);
    //             i += 4 * indexOffset;
    //         }
    //         else if (d04 < (d0 + d1 + d2 + d3)) //4
    //         {
    //             ref_point2d.push_back(raw_point2d[i + 4 * indexOffset]);
    //             i += 3 * indexOffset;
    //         }
    //         else if (d03 < (d0 + d1 + d2)) //3
    //         {
    //             ref_point2d.push_back(raw_point2d[i + 3 * indexOffset]);
    //             i += 2 * indexOffset;
    //         }
    //         else if (d02 < (d0 + d1)) //2
    //         {
    //             ref_point2d.push_back(raw_point2d[i + 2 * indexOffset]);
    //             i += 1 * indexOffset;
    //         }
    //         else //1
    //         {
    //             ref_point2d.push_back(raw_point2d[i + 1 * indexOffset]);
    //         }
    //     }
    //     for (size_t i = count - indexOffset; i < count; i++)
    //     {
    //         ref_point2d.push_back(raw_point2d[i]);
    //     }
    //     return true;
    // }
#pragma endregion
}

#endif