//
// Created by forrest on 18-11-1.
//
#include "closed_loop_rrt_star.hpp"

#include <iostream>
#include <vector>
#include <math.h>
#include <limits>
#include <algorithm>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace cpp_robotics {
namespace closed_loop_rrt_star {

using std::vector;

void ClosedLoopRRTStar::planning(bool show_animation,
                                 vector<double>& x,
                                 vector<double>& y,
                                 vector<double>& yaw,
                                 vector<double>& v,
                                 vector<double>& t,
                                 vector<double>& a,
                                 vector<double>& d,
                                 bool& flag) {
    node_list_.emplace_back(start_);

    tryGoalPath();

    for (int i=0; i<max_iter_; ++i) {
        auto rnd = getRandomPoint();
        auto nind = getNearestListIndex(node_list_, rnd);

        auto new_node = steer(rnd, nind);
        if (use_rsp) {
            if (new_node.parent == -1) {
                continue;
            }
        }

        if (collisionCheck(new_node, obstacle_list_)) {
            auto nearinds = findNearNodes(new_node);
            new_node = chooseParent(new_node, nearinds);
            if (new_node.parent == -1) {
                continue;
            }

            node_list_.emplace_back(new_node);
            rewire(new_node, nearinds);

            tryGoalPath();
        }

        if (show_animation && (i % 5 == 0)) {
            drawGraph(rnd);
        }
    }

    // generate coruse
    auto path_indexs = getBestLastIndexs();
    searchBestFeasiblePath(path_indexs, x, y, yaw, v, t, a, d, flag);
}

void ClosedLoopRRTStar::searchBestFeasiblePath(const std::vector<int>& path_indexs,
                                               vector<double>& fx,
                                               vector<double>& fy,
                                               vector<double>& fyaw,
                                               vector<double>& fv,
                                               vector<double>& ft,
                                               vector<double>& fa,
                                               vector<double>& fd,
                                               bool& fflag) {
    std::cout << "Start search feasible path" << std::endl;
    auto best_time = std::numeric_limits<double>::infinity();

    // pure pursuit tracking
    for (auto ind : path_indexs) {
        auto path = genFinalCourse(ind);

        vector<double> x;
        vector<double> y;
        vector<double> yaw;
        vector<double> v;
        vector<double> t;
        vector<double> a;
        vector<double> d;
        bool flag;
        checkTrackingPathIsFeasible(path, x, y, yaw, v, t, a, d, flag);

        if (flag and (best_time >= t.back())) {
            std::cout << "feasible path is found." << std::endl;
            best_time = t.back();
            fx = x;
            fy = y;
            fyaw = yaw;
            fv  = v;
            ft = t;
            fa = a;
            fd = d;
        }
    }

    std::cout << "best time is:" << best_time << std::endl;

    if (!fx.empty()) {
        fx.emplace_back(end_.x);
        fy.emplace_back(end_.y);
        fyaw.emplace_back(end_.yaw);
        fflag = true;
    }
    else {
        fx = vector<double>();
        fy = vector<double>();
        fyaw = vector<double>();
        fv  = vector<double>();
        ft = vector<double>();
        fa = vector<double>();
        fd = vector<double>();
        fflag = false;
    }
}

void ClosedLoopRRTStar::checkTrackingPathIsFeasible(const NodeListType& path,
                                                    vector<double>& x,
                                                    vector<double>& y,
                                                    vector<double>& yaw,
                                                    vector<double>& v,
                                                    vector<double>& t,
                                                    vector<double>& a,
                                                    vector<double>& d,
                                                    bool& find_goal) {
    vector<double> cx;
    vector<double> cy;
    vector<double> cyaw;
    for (auto i : path) {
        cx.emplace_back(i.x);
        cy.emplace_back(i.y);
        cyaw.emplace_back(i.yaw);
    }

    double goal[3] = {cx.back(), cy.back(), cyaw.back()};

    pp_.extendPath(cx, cy, cyaw);

    auto speed_profile = pp_.calcSpeedProfile(cx, cy, cyaw, target_speed_);

    pp_.closedLoopPrediction(cx, cy, cyaw, speed_profile, goal,
                             &x, &y, &yaw, &v, &t, &a, &d, &find_goal);

    for (int i=0; i<yaw.size(); ++i) {
        yaw[i] = pi2pi(yaw[i]);
    }

    if (!find_goal) {
        std::cout << "cannot reach goal." << std::endl;
    }

    if (fabs(yaw.back() - goal[2]) >= (M_PI / 4.0)) {
        std::cout << "final angle is bad." << std::endl;
        find_goal = false;
    }

    double travel = 0;
    for (auto iv : v) {
        travel += (fabs(iv) * um_.get_dt());
    }

    double origin_travel = 0;
    for (int i=0; i<cx.size(); ++i) {
        origin_travel += (sqrt(pow(cx[i], 2) + pow(cy[i], 2)));
    }

    if ((travel / origin_travel) >= 5.0) {
        std::cout << "path is too long." << std::endl;
        find_goal = false;
    }

    if (!collisionCheckWithXY(x, y, obstacle_list_)) {
        std::cout << "This path is collision." << std::endl;
        find_goal = false;
    }

//    return find_goal, x, y, yaw, v, t, a, d
}

NodeListType ClosedLoopRRTStar::genFinalCourse(int goalind){
    NodeListType path;
    path.emplace_back(end_);

    int lastIndex = goalind;
    while(node_list_[lastIndex].parent != -1) {
        Node node = node_list_[lastIndex];
        for (int i=(node.path_x.size()-1); i>=0; i--) {
            path.emplace_back(Node(node.path_x[i],
                                node.path_y[i],
                                node.path_yaw[i]));
        }
        // path.emplace_back(node_list_[lastIndex]);
        lastIndex = node_list_[lastIndex].parent;
    }

    path.emplace_back(start_);

    // path = np.matrix(path[::-1]) // FIXME

    return path;
}

std::vector<int> ClosedLoopRRTStar::getBestLastIndexs() {
    double YAWTH = 1.0 * M_PI / 180.0;
    double XYTH = 0.5;

    std::vector<int> goalinds;
    for (int i=0; i<node_list_.size(); i++) {
        if (calcDistToGoal(node_list_[i].x, node_list_[i].y) <= XYTH) {
            goalinds.emplace_back(i);
        }
    }

    // angle check
    std::vector<int> fgoalinds;
    for (int i=0; i<goalinds.size(); i++) {
        if(fabs(node_list_[goalinds[i]].yaw - end_.yaw) <= YAWTH) {
            fgoalinds.emplace_back(goalinds[i]);
        }
    }

    return fgoalinds;
}

void ClosedLoopRRTStar::tryGoalPath() {
    auto goal = end_;
    auto new_node = steer(goal, (node_list_.size() - 1));
    if (use_rsp) {
        if (new_node.parent == -1) {
            return;
        }
    }
    if (collisionCheck(new_node, obstacle_list_)) {
        node_list_.emplace_back(new_node);
    }
}

Node ClosedLoopRRTStar::chooseParent(const Node& new_node,
                                     const std::vector<int>& nearinds) {
    if (nearinds.empty()) {
        Node temp_node = new_node;
        return temp_node;
    }

    vector<double> dlist;
    for (auto i : nearinds) {
        auto t_node = steer(new_node, i);
        if (use_rsp) {
            if (t_node.parent == -1) {
                continue;
            }
        }

        if (collisionCheck(t_node, obstacle_list_)) {
            dlist.emplace_back(t_node.cost);
        }
        else {
            dlist.emplace_back(std::numeric_limits<double>::infinity());
        }
    }

    double mincost = std::numeric_limits<double>::infinity();
    int mincost_index = -1;
    for (int i=0; i<dlist.size(); i++) {
        if (dlist[i] < mincost) {
            mincost = dlist[i];
            mincost_index = i;
        }
    }

    int minind = nearinds[mincost_index];

    if (mincost >= std::numeric_limits<double>::infinity()) {
        std::cout << "mincost is inf" << std::endl;
        Node temp_node = new_node;
        return temp_node;
    }

    auto out_node = steer(new_node, minind);
    if (use_rsp) {
        if (out_node.parent == -1) {
            return Node();
        }
    }

    return out_node;
}

void ClosedLoopRRTStar::rewire(const Node& new_node,
                               std::vector<int> nearinds) {
    int nnode = node_list_.size();
    for (auto i : nearinds) {
        Node near_node = node_list_[i];
        auto tNode = steer(near_node, nnode - 1);
        if (use_rsp) {
            if (tNode.parent == -1) {
                continue;
            }
        }

        auto obstacleOK = collisionCheck(tNode, obstacle_list_);
        if (obstacleOK && (near_node.cost > tNode.cost)) {
            node_list_[i] = tNode;
        }
    }
}

Node ClosedLoopRRTStar::steer(const Node& rnd, int nind) {
    // expand tree
    auto nearest_node = node_list_[nind];

    if (use_rsp) {
        auto path = rsp_.reedsSheppPathPlanning(nearest_node.x, nearest_node.y, nearest_node.yaw,
                                                rnd.x, rnd.y, rnd.yaw,
                                                um_.get_curvature_max(), step_size_);
        if (path.x.empty()) {
            return Node();
        }

        Node new_node = nearest_node;
        new_node.x = path.x.back();
        new_node.y = path.y.back();
        new_node.yaw = path.yaw.back();

        new_node.path_x = path.x;
        new_node.path_y = path.y;
        new_node.path_yaw = path.yaw;

        new_node.cost += (fabs(path.lengths.t) + fabs(path.lengths.u) + fabs(path.lengths.v));
        new_node.parent = nind;

        return new_node;
    }
    else {
        std::vector<double> px;
        std::vector<double> py;
        std::vector<double> pyaw;
        cpp_robotics::dubins_curves::Mode mode;
        double cost;
        double curvature = um_.get_curvature_max();
        dp_.dubinsPathPlanning(nearest_node.x, nearest_node.y, nearest_node.yaw,
                               rnd.x, rnd.y, rnd.yaw,
                               curvature,
                               px, py, pyaw,
                               mode, cost);

        Node new_node = nearest_node;
        new_node.x = px.back();
        new_node.y = py.back();
        new_node.yaw = pyaw.back();

        new_node.path_x = px;
        new_node.path_y = py;
        new_node.path_yaw = pyaw;

        new_node.cost += cost;
        new_node.parent = nind;

        return new_node;
    }
}

bool ClosedLoopRRTStar::collisionCheck(const Node& node,
                                       const ObstacleListType& obstacle_list) {
    for(int i=0; i< obstacle_list.size(); i++) {
        for (int j=0; j< node.path_x.size(); j++) {
            double dx = obstacle_list[i].x - node.path_x[j];
            double dy = obstacle_list[i].y - node.path_y[j];
            double d = sqrt(pow(dx,2) + pow(dy,2));
            if (d <= obstacle_list[i].size) {
                return false; // collision
            }
        }
    }
    return true; // safe
}

bool ClosedLoopRRTStar::collisionCheckWithXY(const std::vector<double>& x,
                                             const std::vector<double>& y,
                                             const ObstacleListType& obstacle_list) {
    for(int i=0; i< obstacle_list.size(); i++) {
        for (int j=0; j< x.size(); j++) {
            double dx = obstacle_list[i].x - x[j];
            double dy = obstacle_list[i].y - y[j];
            double d = sqrt(pow(dx,2) + pow(dy,2));
            if (d <= obstacle_list[i].size) {
                return false; // collision
            }
        }
    }
    return true; // safe
}

int ClosedLoopRRTStar::getNearestListIndex(const NodeListType& node_list,
                                           const Node& rnd) {
    double minind = std::numeric_limits<double>::infinity();
    int nind = -1;
    for (int i=0; i< node_list.size(); i++) {
        double dx = node_list[i].x - rnd.x;
        double dy = node_list[i].y - rnd.y;
        double dyaw = node_list[i].yaw - rnd.yaw;
        double d = sqrt(pow(dx,2) + pow(dy,2) + pow(dyaw,2));
        if (d < minind) {
            minind = d;
            nind = i;
        }
    }
    return nind;
}

std::vector<int> ClosedLoopRRTStar::findNearNodes(const Node& new_node) {
    int nnode = node_list_.size();
    double r = 50.0 * sqrt((log(nnode) / nnode));
    std::vector<int> nearinds;
    for (int i=0; i< node_list_.size(); i++) {
        double dlist = pow(node_list_[i].x - new_node.x, 2) +
                       pow(node_list_[i].y - new_node.y, 2) +
                       pow(node_list_[i].yaw - new_node.yaw, 2);
        if (dlist <= pow(r, 2)) {
            nearinds.emplace_back(i);
        }
    }
    return nearinds;
}

Node ClosedLoopRRTStar::getRandomPoint() {
    Node rnd;
    rnd.x = fmod(double(rand()),(max_rand_ - min_rand_ + 1)) + min_rand_;
    rnd.y = fmod(double(rand()),(max_rand_ - min_rand_ + 1)) + min_rand_;
    rnd.yaw = fmod(double(rand()),(M_PI - (-M_PI) + 1)) + (-M_PI);
    return rnd;
}

void ClosedLoopRRTStar::drawGraph(const Node& rnd) {
    int cols = (max_rand_ - min_rand_) / map_resolution_;
    int rows = (max_rand_ - min_rand_) / map_resolution_;
    cv::Mat display_img(rows, cols, CV_8UC3, cv::Scalar(255,255,255));

    // draw obstacles
    for(int i=0; i< obstacle_list_.size(); i++)
    {
        cv::circle(display_img,
                   cv::Point(obstacle_list_[i].x / map_resolution_,
                             rows - obstacle_list_[i].y / map_resolution_),
                   obstacle_list_[i].size  / map_resolution_,
                   cv::Scalar(0,0,0),-1);
    }

    // draw tree
    for(int i=1; i< node_list_.size(); i++)
    {
        if (node_list_[i].parent != -1) {
            for(int j=0; j<(node_list_[i].path_x.size()-1); j++)
            {
//                cv::circle(display_img,
//                           cv::Point(node_list_[i].path_x[j] / map_resolution_,
//                                     rows - node_list_[i].path_y[j] / map_resolution_),
//                           1, cv::Scalar(255,255,0), -1);
                cv::line(display_img,
                         cv::Point(node_list_[i].path_x[j] / map_resolution_,
                                   rows - node_list_[i].path_y[j] / map_resolution_),
                         cv::Point(node_list_[i].path_x[j+1] / map_resolution_,
                                   rows - node_list_[i].path_y[j+1] / map_resolution_),
                         cv::Scalar(255,255,0));
            }
        }
    }

    // draw random node
    cv::circle(display_img,
               cv::Point(rnd.x / map_resolution_,
                         rows - rnd.y / map_resolution_),
               3,cv::Scalar(255,0,0),-1);

    // draw start node
    cv::circle(display_img,
               cv::Point(start_.x / map_resolution_,
                         rows - start_.y / map_resolution_),
               3,cv::Scalar(0,255,0),-1);

    // draw end node
    cv::circle(display_img,
               cv::Point(end_.x / map_resolution_,
                         rows - end_.y / map_resolution_),
               3,
               cv::Scalar(0,0,255),1);

    cv::namedWindow("display_img",0);
    cv::imshow("display_img",display_img);
    cv::waitKey(10);
}

} // namespace closed_loop_rrt_star
} // namespace cpp_robotics
