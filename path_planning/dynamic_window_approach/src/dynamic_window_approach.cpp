//
// Created by forrest on 18-7-6.
//

#include "dynamic_window_approach.h"

namespace cpp_robotics
{
    DynamicWindowApproach::DynamicWindowApproach()
    {
        // init x
        this->x_ = std::vector<float>(5);
        this->x_[0] = 0.0;
        this->x_[1] = 0.0;
        this->x_[2] = M_PI / 8.0;
        this->x_[3] = 0.0;
        this->x_[4] = 0.0;

        // init goal
        this->goal_ = std::vector<float>(2);

        // init u
        this->u_ = std::vector<float>(2);

        // init traj_
        this->traj_.push_back(this->x_);
    }

    DynamicWindowApproach::~DynamicWindowApproach() {}

    void DynamicWindowApproach::setGoal(float goal_x, float goal_y)
    {
        this->goal_[0] = goal_x;
        this->goal_[1] = goal_y;
    }

    void DynamicWindowApproach::addObstacles(std::vector<std::vector<float>>& ob)
    {
        std::vector<float> obs0;
        obs0.push_back(-1.0);
        obs0.push_back(-1.0);
        ob.push_back(obs0);

        std::vector<float> obs1;
        obs1.push_back(0.0);
        obs1.push_back(2.0);
        ob.push_back(obs1);

        std::vector<float> obs2;
        obs2.push_back(4.0);
        obs2.push_back(2.0);
        ob.push_back(obs2);

        std::vector<float> obs3;
        obs3.push_back(5.0);
        obs3.push_back(4.0);
        ob.push_back(obs3);

        std::vector<float> obs4;
        obs4.push_back(5.0);
        obs4.push_back(5.0);
        ob.push_back(obs4);

        std::vector<float> obs5;
        obs5.push_back(5.0);
        obs5.push_back(6.0);
        ob.push_back(obs5);

        std::vector<float> obs6;
        obs6.push_back(5.0);
        obs6.push_back(9.0);
        ob.push_back(obs6);

        std::vector<float> obs7;
        obs7.push_back(8.0);
        obs7.push_back(9.0);
        ob.push_back(obs7);

        std::vector<float> obs8;
        obs8.push_back(7.0);
        obs8.push_back(9.0);
        ob.push_back(obs8);

        std::vector<float> obs9;
        obs9.push_back(12.0);
        obs9.push_back(12.0);
        ob.push_back(obs9);
    }

    void DynamicWindowApproach::testDynamicWindowApproach()
    {
        // set goal pose
        setGoal(10,8);

        // set obstacle list
        addObstacles(this->obs_);

        // run dwa
        for(int i=0; i<1000; i++)
        {
            // calculate dwa
            std::vector<std::vector<float>> ltraj;
            dwa_control(this->x_, this->config_, this->goal_, this->obs_, this->u_, ltraj);

            // move robot
            motion(this->u_, this->config_.dt, this->x_);
            this->traj_.push_back(this->x_);  // store state history

            // display calculate proccess
            if(this->show_animation)
            {
                //matplotlibcpp::clf();

                //display traj
                std::vector<float> Px, Py;
                for (int i = 0; i < ltraj.size(); i++)
                {
                    Px.push_back(ltraj[i][0]);
                    Py.push_back(ltraj[i][1]);
                }
                matplotlibcpp::plot(Px, Py, "-g");

                //display robot pose
                std::vector<float> Xx, Xy;
                Xx.push_back(this->x_[0]);
                Xy.push_back(this->x_[1]);
                matplotlibcpp::plot(Xx, Xy, "xr");

                //display goal pose
                std::vector<float> Gx, Gy;
                Gx.push_back(this->goal_[0]);
                Gy.push_back(this->goal_[1]);
                matplotlibcpp::plot(Gx, Gy, "xb");

                //display obstacle list
                std::vector<float> Obx, Oby;
                for (int i = 0; i < this->obs_.size(); i++)
                {
                    Obx.push_back(this->obs_[i][0]);
                    Oby.push_back(this->obs_[i][1]);
                }
                matplotlibcpp::plot(Obx, Oby, "ok");

                //
                matplotlibcpp::axis("equal");
                matplotlibcpp::grid(true);
                matplotlibcpp::pause(0.0001);
                matplotlibcpp::clf();
            }

            // check reached goal
            if(sqrt(pow((this->x_[0] - this->goal_[0]),2) +
                    pow((this->x_[1] - this->goal_[1]),2)) <=
               this->config_.robot_radius)
            {
                std::cout<<"Reached goal!!"<<std::endl;
                break;
            }
        }

        std::cout<<"Done"<<std::endl;

        // display final traj
        if(this->show_animation)
        {
            std::vector<float> Px, Py;
            for (int i = 0; i < this->traj_.size(); i++)
            {
                Px.push_back(this->traj_[i][0]);
                Py.push_back(this->traj_[i][1]);
            }
            matplotlibcpp::plot(Px, Py, "-r");

            matplotlibcpp::show();
        }
    }

    void DynamicWindowApproach::motion(std::vector<float> u,
                                       float dt,
                                       std::vector<float>& x)
    {
        // motion model
        x[0] += u[0] * cos(x[2]) * dt;
        x[1] += u[0] * sin(x[2]) * dt;
        x[2] += u[1] * dt;
        x[3] = u[0];
        x[4] = u[1];
    }

    void DynamicWindowApproach::calc_dynamic_window(std::vector<float> x,
                                                    Config config,
                                                    std::vector<float>& dw)
    {
        // Dynamic window from robot specification
        float Vs[4] = {config.min_speed, config.max_speed,
                       -config.max_yawrate, config.max_yawrate};

        // Dynamic window from motion model
        float Vd[4] = {x[3] - config.max_accel * config.dt,
                       x[3] + config.max_accel * config.dt,
                       x[4] - config.max_dyawrate * config.dt,
                       x[4] + config.max_dyawrate * config.dt};

        // [vmin,vmax, yawrate min, yawrate max]
        dw[0] = std::max(Vs[0], Vd[0]);
        dw[1] = std::min(Vs[1], Vd[1]);
        dw[2] = std::max(Vs[2], Vd[2]);
        dw[3] = std::min(Vs[3], Vd[3]);
    }

    void DynamicWindowApproach::calc_trajectory(std::vector<float> xinit,
                                                float v, float y,
                                                Config config,
                                                std::vector<std::vector<float>>& traj)
    {
        std::vector<float> x(xinit);

        std::vector<std::vector<float>> traj_temp;
        traj_temp.push_back(x);

        float time = 0.0;
        while(time <= config.predict_time)
        {
            std::vector<float> u(2);
            u[0] = v;
            u[1] = y;
            motion(u, config.dt, x);
            traj_temp.push_back(x);
            time += config.dt;
        }

        traj_temp.swap(traj);
    }

    float DynamicWindowApproach::calc_obstacle_cost(std::vector<std::vector<float>> traj,
                                                    std::vector<std::vector<float>> ob,
                                                    Config config)
    {
        // calc obstacle cost inf: collistion, 0:free

        float minr = std::numeric_limits<float>::infinity();

        for(int ii=0; ii<traj.size(); ii++)    // TODO: check correct
        {
            for(int i=0; i<ob.size(); i++)      // TODO: check correct
            {
                float ox = ob[i][0];
                float oy = ob[i][1];
                float dx = traj[ii][0] - ox;
                float dy = traj[ii][1] - oy;

                float r = sqrt(pow(dx,2) + pow(dy,2));
                if(r <= config.robot_radius){
                    return std::numeric_limits<float>::infinity();  // collisiton
                }

                if(minr >= r) {
                    minr = r;
                }
            }
        }

        return 1.0 / minr;  // OK
    }

    float DynamicWindowApproach::calc_to_goal_cost(std::vector<std::vector<float>> traj,
                                                   std::vector<float> goal, Config config)
    {
        // calc to goal cost. It is 2D norm.

        float dx = goal[0] - traj[traj.size()-1][0];  // TODO: check correct
        float dy = goal[1] - traj[traj.size()-1][1];
        float goal_dis = sqrt(pow(dx,2) + pow(dy,2));
        float cost = config.to_goal_cost_gain * goal_dis;

        return cost;
    }

    float DynamicWindowApproach::calc_speed_cost(std::vector<std::vector<float>> traj,
                                                   Config config)
    {
        float cost = config.speed_cost_gain * (config.max_speed - traj[traj.size()-1][3]);

        return cost;
    }

    void DynamicWindowApproach::calc_final_input(std::vector<float> x,
                                                 std::vector<float> dw,
                                                 Config config,
                                                 std::vector<float> goal,
                                                 std::vector<std::vector<float>> ob,
                                                 std::vector<float>& u,
                                                 std::vector<std::vector<float>>& best_traj)
    {
        std::vector<float> xinit(x);
        float min_cost = std::numeric_limits<float>::infinity();
        std::vector<float> min_u(u);
        min_u[0] = 0.0;
        std::vector<std::vector<float>> best_traj_temp;
        best_traj_temp.push_back(x);

        // evalucate all trajectory with sampled input in dynamic window
        for(float v = dw[0]; v <= dw[1]; v+=config.v_reso)
        {
            for(float y = dw[2]; y <= dw[3]; y+=config.yawrate_reso)
            {
                std::vector<std::vector<float>> traj;
                calc_trajectory(xinit, v, y, config, traj);

                // calc cost
                float to_goal_cost = calc_to_goal_cost(traj, goal, config);
                float speed_cost = calc_speed_cost(traj, config);
                float ob_cost = calc_obstacle_cost(traj, ob, config);

                float final_cost = to_goal_cost + speed_cost + ob_cost;

                // search minimum trajectory
                if(min_cost >= final_cost)
                {
                    min_cost = final_cost;
                    min_u[0] = v;
                    min_u[1] = y;
                    //best_traj = traj; // TODO: check
                    traj.swap(best_traj_temp);
                }

                // display calculate proccess
                if(this->show_calc_animation)
                {
                    //display traj
                    std::vector<float> Px, Py;
                    for (int i = 0; i < traj.size(); i++)
                    {
                        Px.push_back(traj[i][0]);
                        Py.push_back(traj[i][1]);
                    }
                    matplotlibcpp::plot(Px, Py, "-b");
                }
            }
        }

        min_u.swap(u);
        best_traj_temp.swap(best_traj);
    }

    void DynamicWindowApproach::dwa_control(std::vector<float> x,
                                            Config config,
                                            std::vector<float> goal,
                                            std::vector<std::vector<float>> ob,
                                            std::vector<float>& u,
                                            std::vector<std::vector<float>>& traj)
    {
        // Dynamic Window control

        // calculate dw,[vmin,vmax, yawrate min, yawrate max]
        std::vector<float> dw(4);
        calc_dynamic_window(x, config, dw);

        calc_final_input(x, dw, config, goal, ob, u, traj);
    }
}