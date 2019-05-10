//
// Created by forrest on 18-9-18.
//

#ifndef RRT_STAR_RRT_STAR_HPP
#define RRT_STAR_RRT_STAR_HPP

#include <iostream>
#include <vector>
#include <math.h>
#include <complex>
#include <limits>

using std::vector;

namespace cpp_robotics {

    struct Node {
        Node() {}
        Node(double x, double y)
            : x(x), y(y) {}
        double x = 0;
        double y = 0;
        double cost = 0;
        int parent = -1;
    };

    struct ObstacleType {
        ObstacleType() {}
        ObstacleType(double x, double y, double size)
            : x(x), y(y), size(size) {}
        double x = 0;
        double y = 0;
        double size = 0;
    };

    using ObstacleListType = vector<ObstacleType>;
    using NodeListType = vector<Node>;

    class RRTStar {
    public:

        RRTStar(double start[2],
                double goal[2],
                double rand_area[2],
                const ObstacleListType& obstacle_list,
                double expand_dist = 0.5,
                double goal_sample_rate = 20.0,
                unsigned int max_iter = 1000)
                : obstacle_list_(obstacle_list),
                  expand_dist_(expand_dist),
                  goal_sample_rate_(goal_sample_rate),
                  max_iter_(max_iter) {
            /*
            Setting Parameter
            start:Start Position [x,y]
            goal:Goal Position [x,y]
            obstacleList:obstacle Positions [[x,y,size],...]
            randArea:Ramdom Samping Area [min,max]
            */
            start_ = Node(start[0], start[1]);
            end_ = Node(goal[0], goal[1]);
            min_rand_ = rand_area[0];
            max_rand_ = rand_area[1];

            srand(time(0));
        }

        NodeListType planning(bool animation = true);

    private:

        NodeListType genFinalCourse(int goalind);

        void chooseParent(const vector<int>& nearinds, Node& new_node);

        Node steer(const Node& rnd, int nind);

        int getBestLastIndex();

        vector<int> findNearNodes(const Node& new_node);

        // FIXME: ?
        void rewire(const Node& new_node, vector<int> nearinds);

        int getNearestListIndex(const NodeListType& node_list,
                                const Node& rnd);

        bool checkCollisionExtend(const Node& near_node,
                                  double theta, double d);

        bool collisionCheck(const Node& node,
                            const ObstacleListType& obstacle_list);

        inline double calcDistToGoal(double x, double y) {
            double dx = x - end_.x;
            double dy = y - end_.y;
            std::complex<double> mycomplex(dx,dy);
            return std::norm(mycomplex);
        }

        Node getRandomPoint();

        void drawGraph(const Node& rnd);
        void drawPath(const NodeListType& path);

        Node start_;
        Node end_;
        double min_rand_;
        double max_rand_;
        double expand_dist_;
        double goal_sample_rate_;
        unsigned int max_iter_;
        ObstacleListType obstacle_list_;

        NodeListType node_list_;

        double map_resolution_ = 0.05;
    };
}

#endif //RRT_STAR_RRT_STAR_HPP
