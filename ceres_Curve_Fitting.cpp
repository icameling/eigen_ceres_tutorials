#include <iostream>
#include <random>
#include <ceres/ceres.h>
#include <opencv2/core/core.hpp>

using namespace std;


//拟合曲线
//      m*x+c
//y = e


struct ExponentialResidual
{
    ExponentialResidual(double x, double y) : x_(x), y_(y) {}
    template <typename T>

    bool operator()(const T* const m, const T* const c, T* residual) const
    {
        residual[0] = T(y_) - exp(m[0] * T(x_) + c[0]);
        return true;
    }

private:
    const double x_, y_;
};

struct ExponentialResidual2
{
    ExponentialResidual2(double x, double y) : x_(x), y_(y) {}
    template <typename T>

    bool operator()(const T* const param, T* residual) const
    {
        residual[0] = T(y_) - exp(param[0] * T(x_) + param[1]);
        return true;
    }

private:
    const double x_, y_;
};


int main ( int argc, char** argv )
{
    google::InitGoogleLogging(argv[0]);

    //参数写法1
    double m = 0.0, c = 0.0;
    ceres::Problem problem;

    //参数写法2
    double param[2] = {0.0, 0.0};
    ceres::Problem problem2;

    double w_sigma=1.0;                 // 噪声Sigma值
    cv::RNG rng;                        // OpenCV随机数产生器
    double x, y_observed;

    cout << "true m = 0.3, c = 0.1" << endl;
    for(int i = 0; i < 100; i++)
    {
        x = (double)i / 20.0;
        y_observed = exp(0.3 * x + 0.1) + rng.gaussian ( w_sigma );

        //参数写法1
        ceres::CostFunction* cost_function =
                new ceres::AutoDiffCostFunction<ExponentialResidual, 1, 1, 1>(
                    new ExponentialResidual(x, y_observed));
        problem.AddResidualBlock(cost_function, NULL, &m, &c);

        //参数写法2
        y_observed += rng.gaussian(1.0);//再添些噪声
        ceres::CostFunction* cost_function2 =
                new ceres::AutoDiffCostFunction<ExponentialResidual2, 1, 2>(
                    new ExponentialResidual2(x, y_observed));
        //为减少异常点对拟合曲线的影响，可以加一个鲁棒核函数(LossFunction)
        //Using LossFunction to reduce the effect of outliers on a least squares fit.
        problem2.AddResidualBlock(cost_function2, new ceres::CauchyLoss(0.5), param);
    }



    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    //cout << summary.BriefReport() <<endl;
    cout << "m:" << m
         << " c:" << c <<'\n';

    ceres::Solve(options, &problem2, &summary);
    cout << "param = "<<param[0]<<" param2 = "<<param[1] << endl;

    return 0;
}
