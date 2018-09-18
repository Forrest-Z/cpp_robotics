//
// Created by forrest on 18-9-18.
//

#include "rrt_star.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace cpp_robotics {

    NodeListType RRTStar::planning(bool animation) {
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

            if (collisionCheck(new_node, obstacle_list_)) {
                auto nearinds = findNearNodes(new_node);
                chooseParent(nearinds, new_node);
                node_list_.push_back(new_node);
                rewire(new_node, nearinds); // FIXME: for what?
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

    NodeListType RRTStar::genFinalCourse(int goalind){
        NodeListType path;
        path.push_back(end_);

        int lastIndex = goalind;
        while(node_list_[lastIndex].parent != -1) {
            path.push_back(node_list_[lastIndex]);
            lastIndex = node_list_[lastIndex].parent;
        }

        path.push_back(start_);

        return path;
    }

    void RRTStar::chooseParent(const vector<int>& nearinds, Node& new_node) {
        if (nearinds.empty()) {
            return;
        }

        vector<double> dlist;
        for (int i=0; i<nearinds.size(); i++) {
            double dx = new_node.x - node_list_[nearinds[i]].x;
            double dy = new_node.y - node_list_[nearinds[i]].y;
            double d = sqrt(pow(dx, 2) + pow(dy, 2));
            double theta = atan2(dy, dx);
            if (checkCollisionExtend(node_list_[nearinds[i]], theta, d)) {
                dlist.push_back(node_list_[nearinds[i]].cost + d);
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

        new_node.cost = mincost;
        new_node.parent = minind;
    }

    Node RRTStar::steer(const Node& rnd, int nind) {
        // expand tree
        Node nearest_node = node_list_[nind];
        double theta = atan2(rnd.y - nearest_node.y, rnd.x - nearest_node.x);

        Node new_node = nearest_node;
        new_node.x += expand_dist_ * cos(theta);
        new_node.y += expand_dist_ * sin(theta);
        new_node.cost += expand_dist_;
        new_node.parent = nind;

        return new_node;
    }

    int RRTStar::getBestLastIndex() {
        std::vector<double> disglist;
        for(int i=0; i<node_list_.size(); i++)
        {
            double d = calcDistToGoal(node_list_[i].x, node_list_[i].y);
            disglist.push_back(d);
        }

        std::vector<int> goalinds;
        for(int i=0; i<disglist.size(); i++)
        {
            if(disglist[i] <= 0.1)
            {
                goalinds.push_back(i);
            }
        }

        double mincost = std::numeric_limits<double>::infinity();
        for(int i=0; i<goalinds.size(); i++)
        {
            if(node_list_[goalinds[i]].cost < mincost)
            {
                mincost = node_list_[goalinds[i]].cost;
            }
        }

        for(int i=0; i<goalinds.size(); i++)
        {
            if(node_list_[goalinds[i]].cost == mincost)
            {
                return goalinds[i];
            }
        }

        return -1;
    }

    vector<int> RRTStar::findNearNodes(const Node& new_node) {
        int nnode = node_list_.size();
        double r = 50.0 * sqrt((log(nnode) / nnode));
        vector<int> nearinds;
        for (int i=0; i< node_list_.size(); i++) {
            double dlist = pow(node_list_[i].x - new_node.x, 2) +
                           pow(node_list_[i].y - new_node.y, 2);
            if (dlist <= pow(r, 2)) {
                nearinds.push_back(i);
            }
        }
        return nearinds;
    }

    // FIXME: ?
    void RRTStar::rewire(const Node& new_node, vector<int> nearinds) {
        int nnode = node_list_.size();
        for (int i=0; i<nearinds.size(); i++) {
            Node near_node = node_list_[nearinds[i]];

            double dx = new_node.x - new_node.x;
            double dy = new_node.y - new_node.y;
            double d = sqrt(pow(dx, 2) + pow(dy, 2));

            double scost = new_node.cost + d;

            if (near_node.cost > scost) {
                near_node.parent = nnode - 1;
                near_node.cost = scost;
            }
        }
    }

    int RRTStar::getNearestListIndex(const NodeListType& node_list,
                                     const Node& rnd) {
        double minind = std::numeric_limits<double>::infinity();
        int nind = -1;
        for (int i=0; i< node_list.size(); i++) {
            double dx = node_list[i].x - rnd.x;
            double dy = node_list[i].y - rnd.y;
            double d = sqrt(pow(dx,2) + pow(dy,2));
            if (d < minind) {
                minind = d;
                nind = i;
            }
        }

        return nind;
    }

    bool RRTStar::checkCollisionExtend(const Node& near_node,
                                        double theta, double d) {
        Node tmp_node = near_node;

        for (int i=0; i<int(d / expand_dist_); i++) {
            tmp_node.x += expand_dist_ * cos(theta);
            tmp_node.y += expand_dist_ * sin(theta);
            if (!collisionCheck(tmp_node, obstacle_list_)) {
                return false;
            }
        }

        return true;
    }

    bool RRTStar::collisionCheck(const Node& node,
                                 const ObstacleListType& obstacle_list) {
        for(int i=0; i< obstacle_list.size(); i++)
        {
            double dx = obstacle_list[i].x - node.x;
            double dy = obstacle_list[i].y - node.y;
            double d = sqrt(pow(dx,2) + pow(dy,2));
            if(d <= obstacle_list[i].size)
            {
                return false; // collision
            }
        }

        return true; // safe
    }

    void RRTStar::drawGraph(const Node& rnd) {
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
            cv::line(display_img,
                     cv::Point(node_list_[i].x / map_resolution_,
                               rows - node_list_[i].y / map_resolution_),
                     cv::Point(node_list_[node_list_[i].parent].x / map_resolution_,
                               rows - node_list_[node_list_[i].parent].y / map_resolution_),
                     cv::Scalar(255,255,0));
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

    void RRTStar::drawPath(const std::vector<Node>& path) {
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

        for(int i=1; i< node_list_.size(); i++)
        {
            cv::line(display_img,
                     cv::Point(node_list_[i].x / map_resolution_,
                               rows - node_list_[i].y / map_resolution_),
                     cv::Point(node_list_[node_list_[i].parent].x / map_resolution_,
                               rows - node_list_[node_list_[i].parent].y / map_resolution_),
                     cv::Scalar(255,255,0));
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
