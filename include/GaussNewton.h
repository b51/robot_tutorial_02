/*************************************************************************
 *
 *              Author: b51
 *                Mail: b51live@gmail.com
 *            FileName: GaussNewton.h
 *
 *          Created On: Sat Jul 11 00:11:42 2020
 *     Licensed under The MIT License [see LICENSE for details]
 *
 ************************************************************************/

#ifndef GAUSS_NEWTON_H_
#define GAUSS_NEWTON_H_

#include <iostream>
#include <vector>

class GaussNewton() {
 public:
  GaussNewton(const std::vector<std::vector<double>> & datas);

  ~GaussNewton();

  void IterateOnce();

  void Optimize();

 private:
  std::vector<std::vector<double>> datas_;

  int max_iterations_;

  std::vector<double> variables_;
};

#endif
