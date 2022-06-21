#include <iostream>
using namespace std;

void LU(int n, double *a, double *b, double *c, double d, double e, double *pointx, double *pointy)
{
    double *f = new double[n + 1];
    double *g = new double[n + 1];
    double *h = new double[n + 1];
    double *k = new double[n + 1];
    //求解得到h[],f[]
    h[1] = b[1];
    for (int i = 1; i < n - 1; i++)
    {
        f[i] = a[i] / h[i];
        h[i + 1] = b[i + 1] - f[i] * c[i];
    }
    //求解得到g[]和f[n-1]
    g[1] = d / h[1];
    for (int i = 1; i < n - 2; i++)
    {
        g[i + 1] = -g[i] * c[i] / h[i + 1];
    }
    f[n - 1] = (a[n - 1] - g[n - 2] * c[n - 2]) / h[n - 1];
    //求解得到k[]和h[n]
    k[1] = e;
    for (int i = 1; i < n - 2; i++)
    {
        k[i + 1] = -f[i] * k[i];
    }
    k[n - 1] = c[n - 1] - f[n - 2] * k[n - 2];
    double gk_sum = 0;
    for (int i = 1; i < n - 1; i++)
    {
        gk_sum = gk_sum + g[i] * k[i];
    }
    h[n] = b[n] - gk_sum - f[n - 1] * c[n - 1];
    //矩阵求解过程
    //追的过程
    double *y = new double[n + 1];
    double *x = new double[n + 1];
    x[0] = 6 * pointx[n - 1];
    y[0] = 6 * pointy[n - 1];
    for (int i = 0; i < n - 2; i++)
    {
        x[i + 1] = 6 * pointx[i] - f[i + 1] * x[i];
        y[i + 1] = 6 * pointy[i] - f[i + 1] * y[i];
    }
    double gx_sum = 0, gy_sum = 0;
    for (int i = 0; i < n - 2; i++)
    {
        gx_sum = gx_sum + g[i + 1] * x[i];
        gy_sum = gy_sum + g[i + 1] * y[i];
    }
    x[n - 1] = 6 * pointx[n - 2] - gx_sum - f[n - 1] * x[n - 2];
    y[n - 1] = 6 * pointy[n - 2] - gy_sum - f[n - 1] * y[n - 2];
    //赶的过程
    double *px = new double[n + 2];
    double *py = new double[n + 2];
    px[n - 1] = x[n - 1] / h[n];
    px[n - 2] = (x[n - 2] - k[n - 1] * px[n - 1]) / h[n - 1];
    py[n - 1] = y[n - 1] / h[n];
    py[n - 2] = (y[n - 2] - k[n - 1] * py[n - 1]) / h[n - 1];
    for (int i = n - 3; i >= 0; i--)
    {
        px[i] = (x[i] - c[i + 1] * px[i + 1] - k[i + 1] * px[n - 1]) / h[i + 1];
        py[i] = (y[i] - c[i + 1] * py[i + 1] - k[i + 1] * py[n - 1]) / h[i + 1];
    }
    px[n] = px[0];
    px[n + 1] = px[1];
    py[n] = py[0];
    py[n + 1] = py[1];
}
void main()
{
    //初始化
    int num = 12;
    // double pointx[12] = {-10, -50, -50, -10, -20, 20, 10, 50, 50, 10, 20, -20};
    // double pointy[12] = {10, 20, -20, -10, -50, -50, -10, -20, 20, 10, 50, 50};
    double pointx[5] = {-100, 100, 100, 100, 20};
    double pointy[5] = {100, 50, -100, 100, 80};
    double *a = new double[num];
    double *b = new double[num];
    double *c = new double[num];
    int d = 1, e = 1;
    for (int i = 1; i <= num; i++)
    {
        b[i] = 4;
        if ((i + 1) <= num)
        {
            a[i] = 1;
            c[i] = 1;
        }
    }
    LU(num, a, b, c, d, e, pointx, pointy);
}