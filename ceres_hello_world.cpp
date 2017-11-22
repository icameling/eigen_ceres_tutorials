#include <iostream>
#include <ceres/ceres.h>

using namespace std;

//定义了代价函数fi(.)
//重载（）函数，这个结构体将作为AutoDiffCostFunction的参数
struct CostFunctor
{
    template <typename T>
    bool operator()(const T* const x, T* residual) const
    {
        residual[0] = T(10.0) - x[0];
        return true;
    }
    CostFunctor() {}
};

int main ( int argc, char** argv )
{
    google::InitGoogleLogging(argv[0]);//日志系统初始化，初始化参数一般是第一个命令行参数--即程序的名称

    double initial_x = 5.0;
    double x = initial_x;

    ceres::Problem problem;

    //声明一个残差方程，利用AutoDiffCostFunction模板来构造
    //param1为误差类型(残差对象)，也就是带有重载（）运算符的结构体
    //param2为误差项维度
    //param3为优化变量的维度
    ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
    //向问题中添加 误差项、核函数、待估计参数
    problem.AddResidualBlock(cost_function, NULL, &x);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;//增量方程如何求解
    options.minimizer_progress_to_stdout = true;//输出到cout
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);//开始优化

    cout << summary.BriefReport() <<endl;
    cout << "x final = " << x << endl;

    cout << "ceres_hello_world finished..."<<endl;

    return 0;
}
