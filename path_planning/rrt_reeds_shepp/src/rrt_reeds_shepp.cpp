//
// Created by forrest on 18-7-27.
// Path Planning Sample Code with Randamized Rapidly-Exploring Random Trees (RRT).
//

#include "rrt_reeds_shepp.hpp"

#include <iostream>
#include <vector>
#include <math.h>
#include <limits>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace cpp_robotics
{
    bool RRTReedsShepp::Planning(std::vector<Node>& path, bool animation)
    {
        node_list_.push_back(start_);

        for(int i=0; i<max_iter_; i++)
        {
            /// Random Sampling
            Node rnd = getRandomPoint();

            /// Find nearest node
            int nind = getNearestListIndex(node_list_, rnd);

            /// Expand tree
            Node newNode = steer(rnd, nind);
            if(newNode.parent == -1)
            {
                continue;
            }

            if(collisionCheck(newNode, obstacle_list_))
            {
                node_list_.push_back(newNode);
            }

            if(animation && (i % 5 == 0))
            {
                drawGraph(rnd);
            }
        }

        /// generate coruse
        int lastIndex = getBestLastIndex();

        if(lastIndex == -1)
        {
            std::cerr << "Find best node fail." << std::endl;
            return false;
        }

        path = genFinalCourse(lastIndex);
        std::cout << "path node size:" << path.size() << std::endl;
        if(animation)
        {
            drawPath(path);
        }

        return true;
    }

    std::vector<Node> RRTReedsShepp::genFinalCourse(int goalind)
    {
        std::vector<Node> path;
        path.push_back(end_);

        int lastIndex = goalind;
        while(node_list_[lastIndex].parent != -1) {
            Node node = node_list_[lastIndex];
            for (int i=(node.path_x.size() - 1); i>=0; i--) {
                path.push_back(Node(node.path_x[i],
                                    node.path_y[i],
                                    node.path_yaw[i]));
            }
            lastIndex = node.parent;
        }

        path.push_back(start_);

        return path;
    }

    bool RRTReedsShepp::collisionCheck(const Node& node,
                                  const ObstacleListType& obstacle_list)
    {
        for(int i=0; i< obstacle_list.size(); i++)
        {
            for(int j=0; j< node.path_x.size(); j++)
            {
                double dx = obstacle_list[i].x - node.path_x[j];
                double dy = obstacle_list[i].y - node.path_y[j];
                double d = sqrt(pow(dx,2) + pow(dy,2));
                if(d <= obstacle_list[i].size)
                {
                    return false; // collision
                }
            }
        }

        return true; // safe
    }

    Node RRTReedsShepp::steer(const Node& rnd, int nind)
    {
        double curvature = 1.0;

        Node nearest_node = node_list_[nind];

        auto path = rsp_.reedsSheppPathPlanning(nearest_node.x, nearest_node.y, nearest_node.yaw,
                                                rnd.x, rnd.y, rnd.yaw,
                                                curvature, 0.1);
        if(path.x.empty())
        {
            return Node();
        }
//        matplotlibcpp::clf();
//        matplotlibcpp::plot(px, py, "b");
//        matplotlibcpp::axis("equal");
//        matplotlibcpp::grid(true);
//        matplotlibcpp::pause(0.001);

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

    int RRTReedsShepp::getBestLastIndex()
    {
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
            if(fabs(node_list_[goalinds[i]].cost - mincost) < 0.000001)
            {
                return goalinds[i];
            }
        }

        return -1;
    }

    int RRTReedsShepp::getNearestListIndex(const NodeListType& node_list,
                                           const Node& rnd)
    {
        double minind = std::numeric_limits<double>::infinity();
        int nind = -1;
        for(int i=0; i< node_list.size(); i++)
        {
            double dx = node_list[i].x - rnd.x;
            double dy = node_list[i].y - rnd.y;
            double dyaw = node_list[i].yaw - rnd.yaw;
            double d = sqrt(pow(dx,2) + pow(dy,2) + pow(dyaw,2));
            if(d < minind)
            {
                minind = d;
                nind = i;
            }
        }

        return nind;
    }

    Node RRTReedsShepp::getRandomPoint()
    {
        Node rnd;
        double sample_rate = fmod(double(rand()),(100 - 0 + 1)) + 0;
        if(sample_rate > goal_sample_rate_)
        {
            rnd.x = fmod(double(rand()),(max_rand_ - min_rand_ + 1)) + min_rand_;
            rnd.y = fmod(double(rand()),(max_rand_ - min_rand_ + 1)) + min_rand_;
            rnd.yaw = fmod(double(rand()),(M_PI - (-M_PI) + 1)) + (-M_PI);
        }
        else // goal point sampling
        {
            rnd = end_;
        }
        return rnd;
    }

    void RRTReedsShepp::drawGraph(const Node& rnd)
    {
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

    void RRTReedsShepp::drawPath(const std::vector<Node>& path)
    {
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