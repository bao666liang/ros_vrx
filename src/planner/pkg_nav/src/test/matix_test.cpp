#include <iostream>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <chrono>
// #include <Eigen/Eigen>
using namespace std;
using namespace Eigen;
using Eigen::MatrixXd;

int main1() // //matrix vector test
{
    std::cout << "计算中......" << std::endl;
    MatrixXd m(3, 3);  //MatrixXd表示是任意尺寸的矩阵i x j, m(3,3)代表一个3x3的方块矩阵
    m(0, 0) = 1;       //代表矩阵元素a11
    m(0, 1) = 2;       //a12
    m(1, 0) = 3;       //a21
    m(1, 1) = 4;       //a22=a21+a12
    cout << m << endl; //输出矩阵m

    MatrixXd m0 = MatrixXd::Random(3, 3);        //随机初始化初始化的值在[-1,1]区间内,矩阵大小3X3
    MatrixXd m1 = MatrixXd::Constant(3, 3, 2.5); //常量值初始化,矩阵里面的值全部为2.4 ,三个参数分别代表：行数，列数，常量值
    Matrix2d m2 = Matrix2d::Zero();              //零初始化.矩阵里面的值全部为0
    Matrix3d m3 = Matrix3d::Ones();              // 矩阵里面的值全部初始化为1
    Matrix4d m4 = Matrix4d::Identity();          //初始化为单位矩阵
    Matrix3d m5;                                 //逗号初始化
    m5 << 1, 2, 3, 4, 5, 6, 7, 8, 9;
    cout << "m0 =" << endl
         << m0 << endl;
    cout << "m1 =" << endl
         << m1 << endl;
    cout << "m2 =" << endl
         << m2 << endl;
    cout << "m3 =" << endl
         << m3 << endl;
    cout << "m4 =" << endl
         << m4 << endl;
    cout << "m5 =" << endl
         << m5 << endl;
    cout << "m5.size= " << m5.size() << endl;
    cout << "m5.rows= " << m5.rows() << endl;
    cout << "m5.cols = " << m5.cols() << endl;
    m.resize(2, 2);
    cout << "m = " << m << endl;

    std::cout << "计算完毕......" << std::endl;
    return 0;
}

int main2() //matrix vector test
{
    MatrixXd m = MatrixXd::Random(3, 3);        //初始化动态矩阵m,使用Random函数,初始化的值在[-1,1]区间内,矩阵大小3X3
    cout << m << endl;                          //输出矩阵m
    m = (m + MatrixXd::Constant(3, 3, 1)) * 10; // MatrixXd::Constant(3, 3, 1.2)初始化3X3矩阵,矩阵里面的数值为常量,全部为1.2
    // Eigen重载了+ 运算符，两个矩阵有相同的行数和列数即可相加,对应位置上的值相加
    cout << "m =" << endl
         << m << endl;
    VectorXd v(3);
    v << 1, 2, 3; //逗号初始化，英文：comma-initializer,Eigen未提供c++11 的{}初始化方式
    cout << " v =" << endl
         << v << endl;
    cout << "m * v =" << endl
         << m * v << endl;
    RowVectorXd vec1(3);
    vec1 << 1, 2, 3;
    std::cout << "vec1 = " << vec1 << std::endl;
    std::cout << "vec1.sum = " << vec1.sum() << std::endl;
    RowVectorXd vec2(4);
    vec2 << 1, 4, 9, 16;
    std::cout << "vec2 = " << vec2 << std::endl;
    RowVectorXd joined(7);
    joined << vec1, vec2;
    std::cout << "joined = " << joined << std::endl;
}

VectorXd cubic_fitting(VectorXd v_x, VectorXd v_y, const int m = 3) //polynomial_dimension
{
    // std::cout << "v_x =\n"
    //           << v_x << std::endl;
    // std::cout << "v_y =\n"
    //           << v_y << std::endl;
    MatrixXd A(m + 1, m + 1);
    VectorXd b(m + 1);
    // std::cout << "v_x*v_y =\n"
    //           << v_x.dot(v_y) << std::endl;
    int len = v_x.size();
    for (int i = 0; i < m + 1; i++)
    {
        VectorXd temp(len);
        for (int k = 0; k < len; k++)
        {
            temp(k) = pow(v_x(k), i);
        }
        // std::cout << "temp" << i << "=\n"
        //           << temp << std::endl;
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
    VectorXd x = A.colPivHouseholderQr().solve(b);
    // std::cout << "x=\n"
    //           << x << std::endl;
    return x;
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

int main3() //Ax = b
{
    Matrix4d A;
    A << 1, 2, 3, 4,
        1, 4, 3, 2,
        1, 3, 2, 4,
        4, 1, 1, 3;
    Vector4d B(30, 26, 29, 21);
    Vector4d x = A.colPivHouseholderQr().solve(B); //                                      right Answer
    std::cout << "x = A.colPivHouseholderQr().solve(B) = \n"
              << x << std::endl;
    x = A.lu().solve(B);
    std::cout << "x = A.lu().solve(B)  = \n"
              << x << std::endl;
    std::cout << "A*x = \n"
              << A * x << std::endl;

    Matrix3d A2;
    A2 << 50.0, 122.5, 404.25000000000006,
        122.5, 404.25000000000006, 1500.6250000000005,
        404.25000000000006, 1500.6250000000005, 5941.666500000001;
    // Vector3d b(30, 26, 29);
    Vector3d b(2101.564162313241, 7470.673261477969, 29086.80080277384);
    Vector3d x1 = A2.colPivHouseholderQr().solve(b); //
    std::cout << "x1 = A2.colPivHouseholderQr().solve(b)\n"
              << x1 << std::endl;

    VectorXd v_x(9);
    VectorXd v_y(9);
    v_x << 1, 3, 4, 5, 6, 7, 8, 9, 10;
    v_y << 10, 5, 4, 2, 1, 1, 2, 3, 4;
    std::cout << cubic_fitting(v_x, v_y, 2) << std::endl;
}

int main4() //SplinePlanning
{
    int count = 10;
    float a0[count];
    float a1[count];
    float a2[count];
    float a3[count];
    float x_arr[count] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    float y_arr[count] = {1, 4, 9, 16, 25, 36, 49, 64, 81, 100};
    SplinePlanning(x_arr, y_arr, count, a0, a1, a2, a3, 0, 0);
}

float caculate_circle(float *point1, float *point2, float *point3, float *point4)
{
    float x1 = point1[0], y1 = point1[1];
    float x2 = point2[0], y2 = point2[1];
    float x3 = point3[0], y3 = point3[1];
    double a = x1 - x2;
    double b = y1 - y2;
    double c = x1 - x3;
    double d = y1 - y3;
    double e = 0.5 * ((x1 * x1 - x2 * x2) + (y1 * y1 - y2 * y2));
    double f = 0.5 * ((x1 * x1 - x3 * x3) + (y1 * y1 - y3 * y3));
    double det = a * d - b * c;
    if (fabs(det) < 1e-5) //三点不能共线。
    {
        // point4[0] = 0;
        // point4[1] = 0;
        return -1;
    }

    double x0 = (d * e - b * f) / det;
    double y0 = (a * f - c * e) / det;
    point4[0] = x0;
    point4[1] = y0;
    return sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
}

int main() //caculate_circle
{
    float point1[2] = {0, 0};
    float point2[2] = {2, 2};
    float point3[2] = {4, 0};
    float point4[2];
    float radius = 0;
    radius = caculate_circle(point1, point2, point3, point4);
    std::cout << radius << std::endl;
    std::cout << point4[0] << std::endl;
    main3();
}