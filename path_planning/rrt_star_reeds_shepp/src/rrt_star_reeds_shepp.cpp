//
// Created by forrest on 18-9-18.
//

#include "rrt_star_reeds_shepp.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace cpp_robotics {

    NodeListType RRTStarReedsShepp::planning(bool animation) {
        node_list_.push_back(start_);
        for(int i=0; i<max_iter_; i++) {
            /// Random Sampling
            Node rnd;
            if (fmod(double(i), goal_sample_rate_) <= 0) {
                rnd = end_;
            }
            else {
                rnd.x = fmod(double(rand()),(max_rand_ - min_rand_ + 1)) + min_rand_;
                rnd.y = fmod(double(rand()),(max_rand_ - min_rand_ + 1)) + min_rand_;
            }

            /// Find nearest node
            auto nind = getNearestListIndex(node_list_, rnd);

            auto new_node = steer(rnd, nind);
            if (new_node.parent == -1) {
                continue;
            }

            if (collisionCheck(new_node, obstacle_list_)) {
                auto nearinds = findNearNodes(new_node);
                chooseParent(nearinds, new_node);
                if (new_node.parent == -1) {
                    continue;
                }
                node_list_.push_back(new_node);
                rewire(new_node, nearinds);
            }

            if (animation) {
                drawGraph(rnd);
            }
        }

        // generate coruse
        auto lastIndex = getBestLastIndex();
        if (lastIndex == -1) {
            return NodeListType();
        }

        auto path = genFinalCourse(lastIndex);
        if (animation) {
            drawPath(path);
        }

        return path;
    }

    NodeListType RRTStarReedsShepp::genFinalCourse(int goalind){
        NodeListType path;
        path.push_back(end_);

        int lastIndex = goalind;
        while(node_list_[lastIndex].parent != -1) {
            Node node = node_list_[lastIndex];
            for (int i=0; i<node.path_x.size(); i++) {
                path.push_back(Node(node.path_x[i],
                                    node.path_y[i],
                                    node.path_yaw[i]));
            }
            // path.push_back(node_list_[lastIndex]);
            lastIndex = node_list_[lastIndex].parent;
        }

        path.push_back(start_);

        return path;
    }

    void RRTStarReedsShepp::chooseParent(const vector<int>& nearinds,
                                         Node& new_node) {
        if (nearinds.empty()) {
            return;
        }

        vector<double> dlist;
        for (int i=0; i<nearinds.size(); i++) {
            auto t_node = steer(new_node, nearinds[i]);
            if (t_node.parent == -1) {
                continue;
            }

            if (collisionCheck(t_node, obstacle_list_)) {
                dlist.push_back(t_node.cost);
            }
            else {
                dlist.push_back(std::numeric_limits<double>::infinity());
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
            return;
        }

        new_node = steer(new_node, minind);
    }

    Node RRTStarReedsShepp::steer(const Node& rnd, int nind) {
        // expand tree
        Node nearest_node = node_list_[nind];

        auto path = rsp_.reedsSheppPathPlanning(nearest_node.x, nearest_node.y, nearest_node.yaw,
                                                rnd.x, rnd.y, rnd.yaw,
                                                curvature_, step_size_);
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

    int RRTStarReedsShepp::getBestLastIndex() {
        double YAWTH = 3.0 * M_PI / 180.0;
        double XYTH = 0.5;

        std::vector<int> goalinds;
        for (int i=0; i<node_list_.size(); i++) {
            if (calcDistToGoal(node_list_[i].x, node_list_[i].y) <= XYTH) {
                goalinds.push_back(i);
            }
        }

        // angle check
        std::vector<int> fgoalinds;
        for (int i=0; i<goalinds.size(); i++) {
            if(fabs(node_list_[goalinds[i]].yaw - end_.yaw) <= YAWTH) {
                fgoalinds.push_back(goalinds[i]);
            }
        }

        if (fgoalinds.empty()) {
            return -1;
        }

        double mincost = std::numeric_limits<double>::infinity();
        for (int i=0; i<fgoalinds.size(); i++) {
            if (node_list_[fgoalinds[i]].cost < mincost) {
                mincost = node_list_[fgoalinds[i]].cost;
            }
        }

        for (int i=0; i<fgoalinds.size(); i++) {
            if (fabs(node_list_[fgoalinds[i]].cost - mincost) < 0.000001) {
                return fgoalinds[i];
            }
        }

        return -1;
    }

    vector<int> RRTStarReedsShepp::findNearNodes(const Node& new_node) {
        int nnode = node_list_.size();
        double r = 50.0 * sqrt((log(nnode) / nnode));
        vector<int> nearinds;
        for (int i=0; i< node_list_.size(); i++) {
            double dlist = pow(node_list_[i].x - new_node.x, 2) +
                           pow(node_list_[i].y - new_node.y, 2) +
                           pow(node_list_[i].yaw - new_node.yaw, 2);
            if (dlist <= pow(r, 2)) {
                nearinds.push_back(i);
            }
        }
        return nearinds;
    }

    void RRTStarReedsShepp::rewire(const Node& new_node,
                                   vector<int> nearinds) {
        int nnode = node_list_.size();
        for (int i=0; i<nearinds.size(); i++) {
            Node near_node = node_list_[nearinds[i]];
            auto tNode = steer(near_node, nnode - 1);
            if (tNode.parent == -1) {
                continue;
            }

            auto obstacleOK = collisionCheck(tNode, obstacle_list_);
            if (obstacleOK && (near_node.cost > tNode.cost)) {
                node_list_[nearinds[i]] = tNode;
            }
        }
    }

    int RRTStarReedsShepp::getNearestListIndex(const NodeListType& node_list,
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

    bool RRTStarReedsShepp::collisionCheck(const Node& node,
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

    void RRTStarReedsShepp::drawGraph(const Node& rnd) {
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

    void RRTStarReedsShepp::drawPath(const std::vector<Node>& path) {
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

        // draw path
        for(int i=0; i< (path.size()-1); i++)
        {
            cv::line(display_img,
                     cv::Point(path[i].x / map_resolution_,
                               rows - path[i].y / map_resolution_),
                     cv::Point(path[i+1].x / map_resolution_,
                               rows - path[i+1].y / map_resolution_),
                     cv::Scalar(255,0,0));
        }

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
        cv::waitKey(0);
    }
}
