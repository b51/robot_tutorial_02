/*************************************************************************
*
*              Author: b51
*                Mail: b51live@gmail.com
*            FileName: main.cc
*
*          Created On: Thu Jul  9 23:51:12 2020
*     Licensed under The MIT License [see LICENSE for details]
*
************************************************************************/

#include <iostream>
#include <fstream>
#include <vector>
#include <ceres/ceres.h>

void ReadDataFromFile(const std::string& fullpath,
                      std::vector<std::vector<double>>& datas) {
  std::ifstream f;
  // open file with file path
  f.open(fullpath.c_str());
  std::string line;

  // read data frome file line by line
  while (std::getline(f, line)) {
    std::istringstream iss(line);
    std::vector<double> data;
    std::string str_data;
    iss >> str_data;
    if (str_data == "#")
      continue;

    // std::stod, convert string -> double
    data.emplace_back(std::stod(str_data));
    while (iss >> str_data)
      data.emplace_back(std::stod(str_data));

    datas.emplace_back(data);
  }
  // close file
  f.close();
}

struct ExponentialResidual {
  ExponentialResidual(double x, double y)
      : x_(x), y_(y) {}
  template <typename T> bool operator()(const T* const m,
                                        const T* const c,
                                        T* residual) const {
    residual[0] = y_ - exp(m[0] * x_ + c[0]);
    return true;
  }
 private:
  const double x_;
  const double y_;
};

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " [data path]" << std::endl;
    return -1;
  }
  double m = 0.0;
  double c = 0.0;
  std::vector<std::vector<double>> datas;
  ReadDataFromFile(argv[1], datas);

  /**
   *  Optimize curve with ceres, check link below
   *  http://ceres-solver.org/nnls_tutorial.html#curve-fitting
   */
  ceres::Problem problem;
  for (size_t i = 0; i < datas.size(); i++) {
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<ExponentialResidual, 1, 1, 1>(
            new ExponentialResidual(datas[i][0], datas[i][1])),
        nullptr, &m, &c);
  }

  ceres::Solver::Options options;
  options.max_num_iterations = 25;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";
  std::cout << "Initial m: " << 0.0 << " c: " << 0.0 << "\n";
  std::cout << "Final   m: " << m << " c: " << c << "\n";

  /**
   *  Optimize curve with Gauss Newton Non-linear Least Squares method
   *  which written in your own code
   */
  std::vector<double> variables;
  GaussNewton gauss_newton(datas);
  gauss_newton.Iterate();
  results = gauss_newton.GetOptimizedVariables();
  
  return 0;
}
