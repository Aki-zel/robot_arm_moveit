#include "cubicSpline.h"
#include <stdexcept>
#include <iostream>

using namespace std;
/* 初始化输入输出速度加速度 */

double x_out = 0, y_out = 0;

/* 三次样条无参构造 */
cubicSpline::cubicSpline()
{
}
/* 析构 */
cubicSpline::~cubicSpline()
{
    releaseMem();
}
/* 初始化参数 */
void cubicSpline::initParam()
{
    x_sample_ = y_sample_ = M_ = NULL;
    sample_count_ = 0;
    bound1_ = bound2_ = 0;
}
/* 释放参数 */
void cubicSpline::releaseMem()
{
    delete x_sample_;
    delete y_sample_;
    delete M_;

    initParam();
}
/* 加载关节位置数组等信息 */
bool cubicSpline::loadData(double *x_data, double *y_data, int count, double bound1, double bound2, BoundType type)
{
    if ((NULL == x_data) || (NULL == y_data) || (count < 3) || (type > BoundType_Second_Derivative) || (type < BoundType_First_Derivative))
    {
        return false;
    }

    initParam();

    x_sample_ = new double[count];
    y_sample_ = new double[count];
    M_ = new double[count];
    sample_count_ = count;

    memcpy(x_sample_, x_data, sample_count_ * sizeof(double));
    memcpy(y_sample_, y_data, sample_count_ * sizeof(double));

    bound1_ = bound1;
    bound2_ = bound2;

    return spline(type);
}
/* 计算样条插值 */
bool cubicSpline::spline(BoundType type)
{
    if ((type < BoundType_First_Derivative) || (type > BoundType_Second_Derivative))
    {
        return false;
    }

    //  追赶法解方程求二阶偏导数
    double f1 = bound1_, f2 = bound2_;

    double *a = new double[sample_count_]; //  a:稀疏矩阵最下边一串数
    double *b = new double[sample_count_]; //  b:稀疏矩阵最中间一串数
    double *c = new double[sample_count_]; //  c:稀疏矩阵最上边一串数
    double *d = new double[sample_count_];

    double *f = new double[sample_count_];

    double *bt = new double[sample_count_];
    double *gm = new double[sample_count_];

    double *h = new double[sample_count_];

    for (int i = 0; i < sample_count_; i++)
        b[i] = 2; //  中间一串数为2
    for (int i = 0; i < sample_count_ - 1; i++)
        h[i] = x_sample_[i + 1] - x_sample_[i]; // 各段步长
    for (int i = 1; i < sample_count_ - 1; i++)
        a[i] = h[i - 1] / (h[i - 1] + h[i]);
    a[sample_count_ - 1] = 1;

    c[0] = 1;
    for (int i = 1; i < sample_count_ - 1; i++)
        c[i] = h[i] / (h[i - 1] + h[i]);

    for (int i = 0; i < sample_count_ - 1; i++)
        f[i] = (y_sample_[i + 1] - y_sample_[i]) / (x_sample_[i + 1] - x_sample_[i]);

    for (int i = 1; i < sample_count_ - 1; i++)
        d[i] = 6 * (f[i] - f[i - 1]) / (h[i - 1] + h[i]);

    //  追赶法求解方程
    if (BoundType_First_Derivative == type)
    {
        d[0] = 6 * (f[0] - f1) / h[0];
        d[sample_count_ - 1] = 6 * (f2 - f[sample_count_ - 2]) / h[sample_count_ - 2];

        bt[0] = c[0] / b[0];
        for (int i = 1; i < sample_count_ - 1; i++)
            bt[i] = c[i] / (b[i] - a[i] * bt[i - 1]);

        gm[0] = d[0] / b[0];
        for (int i = 1; i <= sample_count_ - 1; i++)
            gm[i] = (d[i] - a[i] * gm[i - 1]) / (b[i] - a[i] * bt[i - 1]);

        M_[sample_count_ - 1] = gm[sample_count_ - 1];
        for (int i = sample_count_ - 2; i >= 0; i--)
            M_[i] = gm[i] - bt[i] * M_[i + 1];
    }
    else if (BoundType_Second_Derivative == type)
    {
        d[1] = d[1] - a[1] * f1;
        d[sample_count_ - 2] = d[sample_count_ - 2] - c[sample_count_ - 2] * f2;

        bt[1] = c[1] / b[1];
        for (int i = 2; i < sample_count_ - 2; i++)
            bt[i] = c[i] / (b[i] - a[i] * bt[i - 1]);

        gm[1] = d[1] / b[1];
        for (int i = 2; i <= sample_count_ - 2; i++)
            gm[i] = (d[i] - a[i] * gm[i - 1]) / (b[i] - a[i] * bt[i - 1]);

        M_[sample_count_ - 2] = gm[sample_count_ - 2];
        for (int i = sample_count_ - 3; i >= 1; i--)
            M_[i] = gm[i] - bt[i] * M_[i + 1];

        M_[0] = f1;
        M_[sample_count_ - 1] = f2;
    }
    else
        return false;

    delete a;
    delete b;
    delete c;
    delete d;
    delete gm;
    delete bt;
    delete f;
    delete h;

    return true;
}
/* 得到速度和加速度数组 */
bool cubicSpline::getYbyX(double &x_in, double &y_out)
{
    try
    {
        int klo, khi, k;
        klo = 0;
        khi = sample_count_ - 1;
        double hh, bb, aa;

        //  二分法查找x所在区间段
        while (khi - klo > 1)
        {
            k = (khi + klo) >> 1;
            if (x_sample_[k] > x_in)
                khi = k;
            else
                klo = k;
        }
        hh = x_sample_[khi] - x_sample_[klo];

        aa = (x_sample_[khi] - x_in) / hh;
        bb = (x_in - x_sample_[klo]) / hh;

        y_out = aa * y_sample_[klo] + bb * y_sample_[khi] + ((aa * aa * aa - aa) * M_[klo] + (bb * bb * bb - bb) * M_[khi]) * hh * hh / 6.0;

        acc = (M_[klo] * (x_sample_[khi] - x_in) + M_[khi] * (x_in - x_sample_[klo])) / hh;
        vel = M_[khi] * (x_in - x_sample_[klo]) * (x_in - x_sample_[klo]) / (2 * hh) - M_[klo] * (x_sample_[khi] - x_in) * (x_sample_[khi] - x_in) / (2 * hh) + (y_sample_[khi] - y_sample_[klo]) / hh - hh * (M_[khi] - M_[klo]) / 6;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }

    return true;
}
double cubicSpline::getAcc()
{
    return acc;
}
double cubicSpline::getVel()
{
    return vel;
}
