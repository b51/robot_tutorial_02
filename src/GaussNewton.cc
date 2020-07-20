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
  lamda=500;
  jd=1;
  variables_.resize(2);
  variables_[0] = 0.0;
  variables_[1] = 0.0;
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
   *  Please fill here
   */
  Eigen::MatrixXd error_;
  Eigen::MatrixXd J_; // 雅克比矩阵
  Eigen::Matrix2d H_; // H矩阵
  Eigen::Matrix2d H_lm;//LM算法下的H矩阵
  Eigen::Matrix2d I;//二阶单位阵
  Eigen::Vector2d B_;
  Eigen::Vector2d delta_t;
  Eigen::VectorXd J_d(datas_.size());
  I.setIdentity();

  J_ .resize(datas_.size(), 2);
  error_.resize(datas_.size(), 1);



  for (size_t i = 0; i < datas_.size(); i++) {
	  std::vector<double>& data = datas_.at(i);
                double& x = data.at(0);
                double& y = data.at(1);
                double j1 = -x*exp(m * x + c);//对m的偏导
                double j2 = -exp(m * x + c);//对c的偏导
                J_(i, 0 ) = j1;
                J_(i, 1) = j2;
                error_(i, 0) = y - exp(m * x + c);
            }
  H_=J_.transpose() * J_;//求Hessian矩阵
  B_ = -J_.transpose() * error_;
   
//    H * delta_t = - J^T * error(t)

  if (max_iterations_==100){
  H_lm=H_+lamda*I;
  delta_t = H_lm.ldlt().solve(B_);//LM算法下的dt
  J_d=J_*delta_t;
  jd=0;
  for(size_t i=0; i < datas_.size();i++){
        jd+=J_d[i]*J_d[i];//计算展开后一阶项的模
  }
 // std::cout<<"H_lm:  "<<H_lm<<"H_:  "<<H_<<std::endl; 
  }
  else delta_t = H_.ldlt().solve(B_);//GN算法下的dt





  // After calculation, upate variables with results
  variables_[0] += delta_t[0]; 
  variables_[1] += delta_t[1]; 
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
    std::cout << "iteration " << i << " error: " << eTe << " lamda: "<<lamda<<std::endl;

    // if error less than some value or error change little, break the optimize
    if (eTe < EPSILON or std::abs(eTe - last_eTe) < EPSILON) {
      std::cout << "Optimization done with " << i << " times iteration" << std::endl;
      std::cout << "error: " << eTe << std::endl;
      std::cout << "Optimized m: " << variables_[0] << ", c: " << variables_[1] << std::endl;
      return;
    }

    IterateOnce();
 if (max_iterations_==100){//LM算法下lamda的调节
      	 if ((eTe-last_eTe)/jd<0.25) lamda=lamda*2;//泰勒展开近似不精确，增大lamda
    	if ((eTe-last_eTe)/jd>0.75) lamda=lamda/3;//泰勒展开近似较精确，减少lamda
 }

    last_eTe = eTe;

}
}
