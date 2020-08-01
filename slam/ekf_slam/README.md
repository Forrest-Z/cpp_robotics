###OVERIVEW

ekf-slam algorithm is referred from [here](http://ais.informatik.uni-freiburg.de/teaching/ws12/mapping/) and motion model is odometry model.

I have use [matplotlib](https://github.com/lava/matplotlib-cpp) for visualization. It a C++ wrapper header file for python matplotlib. So you need to have python matplotlib installed to run the visualization.
![](motion.gif)

If you find problems to wrap python in the C++ code. You can use the networked version. It used websocket to establish communication between EKF and data visualization. You have to install related modules (Websocket). 