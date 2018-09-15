//
// Created by forrest on 18-9-15.
//

#ifndef RRT_DUBINS_RRT_DUBINS_HPP
#define RRT_DUBINS_RRT_DUBINS_HPP

#include <iostream>
#include <vector>

#include "dubins_path_planning.hpp"

using std::vector;

namespace cpp_robotics
{
    struct Node
    {
    public:

        Node(){}
        Node(double x, double y, double yaw)
            : x(x), y(y), yaw(yaw)
        {}

        double x = 0;
        double y = 0;
        double yaw = 0;
        vector<double> path_x;
        vector<double> path_y;
        vector<double> path_yaw;
        double cost = 0;
        int parent = -1;
    };

    struct ObstacleType
    {
    public:

        ObstacleType(double x = 0,
                     double y = 0,
                     double size = 0)
        {
            this->x = x;
            this->y = y;
            this->size = size;
        }

        double x;
        double y;
        double size;
    };

    using ObstacleListType = vector<ObstacleType>;
    using NodeListType = vector<Node>;

    class RRTDubis
    {
    public:


        RRTDubis(double start[3],
                 double goal[3],
                 double rand_area[2],
                 ObstacleListType obstacleList,
                 double goal_sample_rate = 10.0,
                 unsigned int max_iter = 10000)
        {
            start_ = Node(start[0], start[1], start[2]);
            end_ = Node(goal[0], goal[1], goal[2]);
            min_rand_ = rand_area[0];
            max_rand_ = rand_area[1];
            goal_sample_rate_ = goal_sample_rate_;
            max_iter_ = max_iter;
            obstacle_list_ = obstacleList;

            srand(time(0));
        }

        bool Planning(std::vector<Node>& path, bool animation = false);

    private:

        std::vector<Node> genFinalCourse(int goalind);

        bool collisionCheck(const Node& node,
                            const ObstacleListType& obstacle_list);

        Node steer(const Node& rnd, int nind);

        int getBestLastIndex();
        int getNearestListIndex(const NodeListType& node_list,
                                const Node& rnd);

        void drawGraph(const Node& rnd);
        void drawPath(const std::vector<Node>& path);

        inline double calcDistToGoal(double x, double y)
        {
            double dx = x - end_.x;
            double dy = y - end_.y;
            return sqrt(pow(dx,2) + pow(dy,2));
        }

        inline double pi2pi(double angle)
        {
            return fmod((angle + M_PI), (2*M_PI)) - M_PI;
        }

        Node start_;
        Node end_;
        double min_rand_;
        double max_rand_;
        double goal_sample_rate_;
        unsigned int max_iter_;
        ObstacleListType obstacle_list_;

        NodeListType node_list_;

        DubinsPath dubins_path_;

        double map_resolution_ = 0.05;
    };
}

#endif //RRT_DUBINS_RRT_DUBINS_HPP
