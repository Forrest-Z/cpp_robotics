# cpp_robotics

C++ sample codes for robotics algorithms.

This is the C++ implementation of the [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics), some part of code refers to [CppRobotics](https://github.com/onlytailei/CppRobotics).

# Requirment
- cmake
- opencv 3.3.1
- Eigen 3.3
- [CppAD](https://www.coin-or.org/CppAD/Doc/install.htm) / [IPOPT](https://www.coin-or.org/Ipopt/documentation/node14.html) (*for MPC convex optimization*) [install tips](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/install_Ipopt_CppAD.md)

# Table of Contents
* localization
  * extended kalmam filter
  * particle filter
  * unscented kalman filter
* slam
  * ekf slam
  * ukf slam
  * fast slam
  * tiny slam
* path planning
  * dfs
  * bfs
  * dijkstra
  * a star
  * dubins path
  * reeds shepp path
  * rrt
  * rrt dubins
  * rrt reeds shepp
  * rrt star
  * rrt star dubins
  * rrt_star reeds shepp
  * closed loop rrt star
  * dynamic window approach
  * cubic spline
* path tracking
  * move to pose
  * pure pursuit
  * stanley controller
* path smooth
  * path smooth

# License 
- MIT

# Authors
- [zhouzuhong](https://github.com/Forrest-Z/)