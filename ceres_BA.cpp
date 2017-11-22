///参考了 ceres 的 examples

#include <iostream>
#include <random>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

using namespace std;

// Read a Bundle Adjustment in the Large dataset.
class BALProblem {
 public:
  ~BALProblem() {
    delete[] point_index_;
    delete[] camera_index_;
    delete[] observations_;
    delete[] parameters_;
  }

  int num_observations()       const { return num_observations_;               }
  const double* observations() const { return observations_;                   }
  double* mutable_cameras()          { return parameters_;                     }
  double* mutable_points()           { return parameters_  + 9 * num_cameras_; }

  //dataset后面的数据 先是摄像头的估计值 再是3D点的估计值
  double* mutable_camera_for_observation(int i) {
    return mutable_cameras() + camera_index_[i] * 9;
  }
  double* mutable_point_for_observation(int i) {
    return mutable_points() + point_index_[i] * 3;
  }

  bool LoadFile(const char* filename) {
    FILE* fptr = fopen(filename, "r");
    if (fptr == NULL) {
      return false;
    };

    FscanfOrDie(fptr, "%d", &num_cameras_); //相机数量
    FscanfOrDie(fptr, "%d", &num_points_);  //路标数量
    FscanfOrDie(fptr, "%d", &num_observations_);    //总的观测数量

    // camera 看到 point 在图像上的坐标
    camera_index_ = new int[num_observations_]; //第k个观测点的相机是谁
    point_index_ = new int[num_observations_];  //第k个观测点的路标点是谁
    observations_ = new double[2 * num_observations_];  //第k个观测点

    num_parameters_ = 9 * num_cameras_ + 3 * num_points_;   //需要估计的参数
    parameters_ = new double[num_parameters_];

    for (int i = 0; i < num_observations_; ++i) {
      FscanfOrDie(fptr, "%d", camera_index_ + i);
      FscanfOrDie(fptr, "%d", point_index_ + i);
      for (int j = 0; j < 2; ++j) {
        FscanfOrDie(fptr, "%lf", observations_ + 2*i + j);
      }
    }

    //文件后面是参数初始估计吗
    for (int i = 0; i < num_parameters_; ++i) {
      FscanfOrDie(fptr, "%lf", parameters_ + i);
    }
    return true;
  }

 private:
  template<typename T>
  void FscanfOrDie(FILE *fptr, const char *format, T *value) {
    int num_scanned = fscanf(fptr, format, value);
    if (num_scanned != 1) {
      LOG(FATAL) << "Invalid UW data file.";
    }
  }

  int num_cameras_;
  int num_points_;
  int num_observations_;
  int num_parameters_;

  int* point_index_;
  int* camera_index_;
  double* observations_;
  double* parameters_;
};


struct SnavelyReprojectionError {
    //传入观测数据
    SnavelyReprojectionError(double observed_x, double observed_y)
        :observed_x_(observed_x), observed_y_(observed_y){}

    // The camera is parameterized using 9 parameters: 3 for rotation,
    // 3 for translation, 1 for focal length and 2 for radial distortion.
    // 也可以用4维来表示旋转，比如四元数可以用QuaternionRotatePoint
    //计算误差
    template <typename T>
    bool operator()(const T* const camera,  //待估计
                    const T* const point,   //待估计
                    T* residuals) const {     //残差一般放在最后面？？
        ///下面目标是将世界坐标系下的点point转化到像素平面
        T p[3];//转化在相机坐标系下
        //camera[0,1,2]是旋转矢量，camera[3,4,5]是平移
        //所以李代数在这儿是旋转在前，平移在后
        ceres::AngleAxisRotatePoint(camera, point, p);
        p[0] += camera[3];//1.先转换到相机坐标系
        p[1] += camera[4];
        p[2] += camera[5];

        //2.转化到归一化平面，根据Snavely's Bundler assumes, whereby
        // the camera coordinate system has a negative z axis.
        T xp = -p[0]/p[2];
        T yp = -p[1]/p[2];

        //3.得到去畸变前的原始像素坐标
        const T& l1 = camera[7];
        const T& l2 = camera[8];
        T r2 = xp*xp + yp*yp;
        T distortion = T(1.0) + r2*(l1+l2*r2);

        //4.根据内参模型，计算像素坐标
        const T& focal = camera[6];
        T predicted_x = focal * distortion * xp;
        T predicted_y = focal * distortion * yp;

        //和观测数据作差，得到残差
        residuals[0] = predicted_x - T(observed_x_);
        residuals[1] = predicted_y - T(observed_y_);

        return true;
    }

    //利用create添加一条观测数据
    static ceres::CostFunction* Create(const double observed_point_x,
                                       const double observed_poiny_y) {
        //变量顺序 residuals-camera-point 2-9-3
        return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 9, 3>(
                new SnavelyReprojectionError(observed_point_x, observed_poiny_y)));
    }

private:
    double observed_x_, observed_y_;
};


int main ( int argc, char** argv )
{
    google::InitGoogleLogging(argv[0]);

    if (argc != 2) {
        std::cerr << "please input dataset \n";
        return 1;
    }

    BALProblem bal_problem;
    if (!bal_problem.LoadFile(argv[1])) {
        std::cerr << "error to open file "<< argv[1] << "\n";
        return 1;
    }
    const double* observations = bal_problem.observations();

    ceres::Problem problem;
    for (int i = 0; i < bal_problem.num_observations(); i++) {
        ceres::CostFunction * cost_function = SnavelyReprojectionError::Create(observations[2*i],
                                                                              observations[2*i+1]);
        //向问题中添加 误差项、核函数、待估计参数（传入初始值）
        problem.AddResidualBlock(cost_function,
                                 NULL,
                                 bal_problem.mutable_camera_for_observation(i),
                                 bal_problem.mutable_point_for_observation(i));
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    return 0;
}
