###OVERIVEW

ukf-slam algorithm is referred from [here](http://ais.informatik.uni-freiburg.de/teaching/ws12/mapping/) and motion model is odometry model.

I have use [matplotlib](https://github.com/lava/matplotlib-cpp) for visualization. It a C++ wrapper header file for python matplotlib. So you need to have python matplotlib installed to run the visualization.
![](motion.gif)

% Odometry readings have the following fields:
% - r1 : rotation 1
% - t  : translation
% - r2 : rotation 2
% which correspond to the identically labeled variables in the motion
% mode.
%
% Sensor readings can again be indexed and each of the entris has the
% following fields:
% - id      : id of the observed landmark
% - range   : measured range to the landmark
% - bearing : measured angle to the landmark (you can ignore this)

ukfslam.mu
    5.00472     // robot x
    4.76418     // robot y
    1.5433      // robot theta[rad]
    1.91161     // landmark[0] x
    0.871578    // landmark[0] y
    -0.0648974  // ...
    3.88692
    4.9261
    2.85144
    4.9495
    4.85547
    1.98617
    6.86482
    4.98403
    8.86512
    8.97942
    7.78975
    9.95293
    4.80857
    8.9224      // landmark[ukfslam.landmarks.size()-1] x
     1.812      // landmark[ukfslam.landmarks.size()-1] y
