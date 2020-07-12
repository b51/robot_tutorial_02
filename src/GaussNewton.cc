/*************************************************************************
*
*              Author: b51
*                Mail: b51live@gmail.com
*            FileName: GaussNewton.cc
*
*          Created On: Sat Jul 11 00:15:13 2020
*     Licensed under The MIT License [see LICENSE for details]
*
************************************************************************/

#include "GaussNewton.h"

constexpr double EPSILON = 1e-5;

// constructor
GaussNewton::GaussNewton(const std::vector<std::vector<double>>& datas,
                         const int max_iterations)
    : datas_(datas),
      max_iterations_(max_iterations) {
  variables_.resize(2);
  variables_[0] = 0.0;
  variables_[0] = 0.0;
}

// deconstructor
GaussNewton::~GaussNewton() {}

/**
 *  Gauss-Newton method:
 *
 *  find value of [m, c] allow F(x) reach min ( ^T means transpose )
 *    write [m, c] as t
 *    argmin F(t) = argmin [error(t)^T * error(t)]
 *    error(t) = e^(mx) + c
 *
 *  Taylor series of error(x), J -> Jacobian
 *    error(t + dt) = error(t) + J * dt
 *    ==>
 *    F(t+dt) = [error(t + dt)^T * error(t + dt)]
 *            = (error(t) + J * dt)^T * (error(t) + J * dt)
 *            = error(t) * error(t) + (J*dt)^T * error(t) + error(t) * (J*dt) + (J*dt)^T * (J*dt)
 *    dF(t+dt) / d(dt) = 2 * J^T * error(t) + 2 * J^T * J * dt = 0;
 *    ==>
 *    J^T * J * dt = - J^T * error(t)
 *  Solve dt and update once
 */
void GaussNewton::IterateOnce() {
  double m = variables_[0];
  double c = variables_[1];

  /**
   *  GaussNewton iterate once to update variables
   *  Pleaase fill here
   */

  // After calculation, upate variables with results
  /* variables_[0] += delta_t[0]; */
  /* variables_[1] += delta_t[1]; */
}

void GaussNewton::Optimize() {
  std::cout << "Initial value of m: " << variables_[0]
            << ", c: " << variables_[1] << std::endl;

  double last_eTe = 0.0;
  for (int i = 0; i < max_iterations_; i++) {
    double m = variables_[0];
    double c = variables_[1];
    double eTe = 0.0;
    for (size_t j = 0; j < datas_.size(); j++) {
      double x = datas_[j][0];
      double y = datas_[j][1];
      double error = exp(m * x) + c - y;
      eTe += error * error;
    }
    std::cout << "iteration " << i << " error: " << eTe << std::endl;

    // if error less than some value or error change little, break the optimize
    if (eTe < EPSILON or std::abs(eTe - last_eTe) < EPSILON) {
      std::cout << "Optimization done with " << i << " times iteration" << std::endl;
      std::cout << "error: " << eTe << std::endl;
      std::cout << "Optimized m: " << variables_[0] << ", c: " << variables_[1] << std::endl;
      return;
    }

    IterateOnce();
    last_eTe = eTe;
  }
}
