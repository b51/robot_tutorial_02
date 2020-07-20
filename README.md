## Tutorial 02
### What this repository for
* 熟悉 ceres 函数库
* 熟悉 高斯牛顿优化方法

### Preinstall

#### Install ceres-solver

```bash
# preinstall some dependencis
$ sudo apt-get install cmake
$ sudo apt-get install libgoogle-glog-dev
$ sudo apt-get install libatlas-base-dev
$ sudo apt-get install libeigen3-dev
$ sudo apt-get install libsuitesparse-dev

$ git clone https://github.com/ceres-solver/ceres-solver
$ cd ceres-solver
$ mkdir build
$ cd build && cmake ..
$ make -j4
$ make test
$ sudo make install
```

### Usage

```bash
$ git clone https://github.com/b51/robot_tutorial_02
$ cd robot_tutorial_01
$ mkdir build
$ cd build
$ cmake ..
$ make -j
$ ./curve_fitting ../data/sample.txt
```

### Task
* 参考 ceres 补充 main.cc 内的 ExponentialResidual 函数
* 补充 GaussNewton.cc 内的 IteraceOnce 完成高斯牛顿优化法的一次迭代
* 比较使用 ceres 和自己编写的高斯牛顿方法优化得到的最终误差
* [扩展] 参考高斯牛顿方法编写 LM 优化的代码并理解其中意义

### Reference
* ceres 优化方法参考地址: http://ceres-solver.org/nnls_tutorial.html#curve-fitting

### Solution
* 参考解决方案参照 solution branch

```bash
$ git pull origin
$ git checkout solution
```
