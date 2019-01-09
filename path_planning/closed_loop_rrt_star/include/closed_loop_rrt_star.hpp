//
// Created by forrest on 18-11-1.
//

#ifndef CLOSED_LOOP_RRT_STAR_CLOSED_LOOP_RRT_STAR_HPP
#define CLOSED_LOOP_RRT_STAR_CLOSED_LOOP_RRT_STAR_HPP

#include <vector>
#include <math.h>
#include <complex>

#include "pure_pursuit.hpp"
#include "unicycle_model.hpp"
#include "reeds_shepp_path_planning.hpp"
#include "dubins_path_planning.hpp"

namespace cpp_robotics {
namespace closed_loop_rrt_star {

using std::vector;

struct Node {
    Node() {}
    Node(double x, double y, double yaw)
            : x(x), y(y), yaw(yaw) {}
    double x = 0;
    double y = 0;
    double yaw = 0;
    std::vector<double> path_x;
    std::vector<double> path_y;
    std::vector<double> path_yaw;
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

using ObstacleListType = std::vector<ObstacleType>;
using NodeListType = std::vector<Node>;

class ClosedLoopRRTStar {
public:

    ClosedLoopRRTStar(double start[3],
                      double goal[3],
                      double rand_area[2],
                      const ObstacleListType& obstacle_list,
                      unsigned int max_iter = 300)
            : obstacle_list_(obstacle_list),
              max_iter_(max_iter) {
        /*
            Setting Parameter
            start:Start Position [x,y]
            goal:Goal Position [x,y]
            obstacleList:obstacle Positions [[x,y,size],...]
            randArea:Ramdom Samping Area [min,max]
         */
        start_ = Node(start[0], start[1], start[2]);
        end_ = Node(goal[0], goal[1], goal[2]);
        min_rand_ = rand_area[0];
        max_rand_ = rand_area[1];

        srand(time(0));
    }

    void planning(bool show_animation,
                  vector<double>& x,
                  vector<double>& y,
                  vector<double>& yaw,
                  vector<double>& v,
                  vector<double>& t,
                  vector<double>& a,
                  vector<double>& d,
                  bool& flag);

    void drawPath(const NodeListType& path);

private:

    void searchBestFeasiblePath(const std::vector<int>& path_indexs,
                                vector<double>& fx,
                                vector<double>& fy,
                                vector<double>& fyaw,
                                vector<double>& fv,
                                vector<double>& ft,
                                vector<double>& fa,
                                vector<double>& fd,
                                bool& fflag);
    void checkTrackingPathIsFeasible(const NodeListType& path,
                                     vector<double>& x,
                                     vector<double>& y,
                                     vector<double>& yaw,
                                     vector<double>& v,
                                     vector<double>& t,
                                     vector<double>& a,
                                     vector<double>& d,
                                     bool& find_goal);
    void calcTrackingPath(); // not use
    NodeListType genFinalCourse(int goalind);
    std::vector<int> getBestLastIndexs();
    void tryGoalPath();
    Node chooseParent(const Node& new_node,
                      const std::vector<int>& nearinds);
    void rewire(const Node& new_node, std::vector<int> nearinds);
    Node steer(const Node& rnd, int nind);
    bool collisionCheck(const Node& node,
                        const ObstacleListType& obstacle_list);
    bool collisionCheckWithXY(const std::vector<double>& x,
                              const std::vector<double>& y,
                              const ObstacleListType& obstacle_list);
    int getNearestListIndex(const NodeListType& node_list,
                            const Node& rnd);
    std::vector<int> findNearNodes(const Node& new_node);
    Node getRandomPoint();

    inline double calcDistToGoal(double x, double y) {
        double dx = x - end_.x;
        double dy = y - end_.y;
        std::complex<double> mycomplex(dx,dy);
        return std::norm(mycomplex);
    }

    inline double pi2pi(double angle) {
        return fmod((angle + M_PI), (2 * M_PI)) - M_PI;
    }

    void drawGraph(const Node& rnd);

    Node start_;
    Node end_;
    double min_rand_;
    double max_rand_;
    ObstacleListType obstacle_list_;
    int max_iter_;

    NodeListType node_list_;

    bool use_rsp = false; // true - rsp, false - dp
    ReedsSheppPath rsp_;
    dubins_curves::DubinsPath dp_;

    unicycle_model::UnicycleModel um_;
    double step_size_ = 0.1;
    pure_pursuit::PurePursuit pp_;

    double target_speed_ = 10.0 / 3.6;

    double map_resolution_ = 0.05;
};

} // namespace closed_loop_rrt_star
} // namespace cpp_robotics

#endif //CLOSED_LOOP_RRT_STAR_CLOSED_LOOP_RRT_STAR_HPP
