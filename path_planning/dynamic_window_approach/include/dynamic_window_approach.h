//
// Created by forrest on 18-7-6.
//

#ifndef DYNAMIC_WINDOW_APPROACH_DYNAMIC_WINDOW_APPROACH_H
#define DYNAMIC_WINDOW_APPROACH_DYNAMIC_WINDOW_APPROACH_H

#include <iostream>
#include <math.h>
#include <vector>

#include "matplotlibcpp.h" // for display

namespace dynamic_window_approach
{
    struct Config
    {
        // robot parameter
        float max_speed = 1.0;                     // [m/s]
        float min_speed = -0.5;                    // [m/s]
        float max_yawrate = 60.0 * M_PI / 180.0;   // [rad/s]

        float max_accel = 0.5;                     // [m/ss]
        float max_dyawrate = 60.0 * M_PI / 180.0;  // [rad/ss]

        float v_reso = 0.01;                       // [m/s]
        float yawrate_reso = 0.1 * M_PI / 180.0;   // [rad/s]

        float dt = 0.1;                            // [s]
        float predict_time = 3.0;                  // [s]

        float to_goal_cost_gain = 3.0;
        float speed_cost_gain = 1.0;
        float robot_radius = 1.0;                  // [m]
    };

    class DynamicWindowApproach
    {
    public:

        DynamicWindowApproach();
        ~DynamicWindowApproach();

        void setGoal(float goal_x, float goal_y);

        void testDynamicWindowApproach();

    private:

        void addObstacles(std::vector<std::vector<float>>& ob); // for test

        /*
         * @brief get dwa trajectory
         */
        void dwa_control(std::vector<float> x,
                         Config config,
                         std::vector<float> goal,
                         std::vector<std::vector<float>> ob,
                         std::vector<float>& u,
                         std::vector<std::vector<float>>& traj);

        /*
         * @brief update robot motion
         */
        void motion(std::vector<float> u,
                    float dt,
                    std::vector<float>& x);

        /*
         * @brief calculate speed dynamic window
         */
        void calc_dynamic_window(std::vector<float> x,
                                 Config config,
                                 std::vector<float>& dw);

        /*
         * @brief calculate trajectory with sampled input
         */
        void calc_trajectory(std::vector<float> xinit,
                             float v, float y,
                             Config config,
                             std::vector<std::vector<float>>& traj);

        /*
         * @brief calculate obstacle cost(inf: collistion, 0:free)
         */
        float calc_obstacle_cost(std::vector<std::vector<float>> traj,
                                 std::vector<std::vector<float>> ob,
                                 Config config);

        /*
         * @brief calculate to goal cost. It is 2D norm
         */
        float calc_to_goal_cost(std::vector<std::vector<float>> traj,
                                std::vector<float> goal, Config config);

        /*
         * @brief calculate speed cost
         */
        float calc_speed_cost(std::vector<std::vector<float>> traj, Config config);

        /*
         * @brief evalucate all trajectory with sampled input in dynamic window
         */
        void calc_final_input(std::vector<float> x,
                              std::vector<float> dw,
                              Config config,
                              std::vector<float> goal,
                              std::vector<std::vector<float>> ob,
                              std::vector<float>& u,
                              std::vector<std::vector<float>>& best_traj);

        // initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        std::vector<float> x_;
        // goal position [x(m), y(m)]
        std::vector<float> goal_;
        // obstacles [x(m) y(m), ....]
        std::vector<std::vector<float>> obs_;
        // v(m/s), omega(rad/s)]
        std::vector<float> u_;

        Config config_;
        std::vector<std::vector<float>> traj_; // for test

        bool show_animation = true;
        bool show_calc_animation = false;
    };
}

#endif //DYNAMIC_WINDOW_APPROACH_DYNAMIC_WINDOW_APPROACH_H
