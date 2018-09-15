//
// Created by forrest on 18-7-27.
// Path Planning Sample Code with Randamized Rapidly-Exploring Random Trees (RRT).
//

#include "rrt_dubins.hpp"

#include <iostream>
#include <vector>
#include <math.h>
#include <limits>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace cpp_robotics
{
    bool RRTDubis::Planning(std::vector<Node>& path, bool animation)
    {
        // TODO: clear node_list_ ?
        node_list_.push_back(start_);

        for(int i=0; i<max_iter_; i++)
        {
            /// Random Sampling
            Node rnd;
            if(fmod(double(i), goal_sample_rate_) <= 0)
            {
                rnd = end_;
            }
            else
            {
                rnd.x = fmod(double(rand()),(max_rand_ - min_rand_ + 1)) + min_rand_;
                rnd.y = fmod(double(rand()),(max_rand_ - min_rand_ + 1)) + min_rand_;
                rnd.yaw = fmod(double(rand()),(M_PI - (-M_PI) + 1)) + (-M_PI);
            }

            /// Find nearest node
            int nind = getNearestListIndex(node_list_, rnd);

            /// Expand tree
            Node newNode = steer(rnd, nind);

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
            return false;
        }

        path = genFinalCourse(lastIndex);

        return true;
    }

    std::vector<Node> RRTDubis::genFinalCourse(int goalind)
    {
        // FIXME: goalind ?

        std::vector<Node> path;
        path.push_back(end_);

        int lastIndex = goalind;
        while(node_list_[lastIndex].parent != -1) {
            path.push_back(node_list_[lastIndex]);
            lastIndex = node_list_[lastIndex].parent;
        }

        path.push_back(start_);

        return path;
    }

    /*Node RRTDubis::chooseParent(const Node& new_node,
                                std::vector<int> nearinds)
    {
        Node parent_node;

        if(nearinds.size() == 0)
        {
            parent_node = new_node;
            return parent_node;
        }

        for(int i=0; i< nearinds.size(); i++)
        {
            double dx = new_node.x - node_list_[i].x;
            double dy = new_node.y - node_list_[i].y;
            double d = sqrt(pow(dx, 2) + pow(dy, 2));
            double theta = atan2(dy, dx);

//            if(collisionCheck())
        }
    }*/

    bool RRTDubis::collisionCheck(const Node& node,
                                  const ObstacleListType& obstacle_list)
    {
        for(int i=0; i< obstacle_list.size(); i++)
        {
            for(int j=0; j< node.path_x.size(); j++)
            {
                double dx = obstacle_list[i].x - node.path_x[j];
                double dy = obstacle_list[i].y - node.path_y[j];
                double d = sqrt(pow(dx,2) + pow(dy,2));
                if(d <= pow(obstacle_list[i].size,2))
                {
                    return false; // collision
                }
            }
        }

        return true; // safe
    }

    Node RRTDubis::steer(const Node& rnd, int nind)
    {
        double curvature = 1.0;

        Node nearest_node = node_list_[nind];

        std::vector<double> px;
        std::vector<double> py;
        std::vector<double> pyaw;
        cpp_robotics::Mode mode;
        double cost;
        dubins_path_.dubinsPathPlanning(nearest_node.x, nearest_node.y, nearest_node.yaw,
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

    int RRTDubis::getBestLastIndex()
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
            if(node_list_[i].cost < mincost)
            {
                mincost = node_list_[i].cost;
            }
        }

        for(int i=0; i<goalinds.size(); i++)
        {
            if(node_list_[i].cost == mincost)
            {
                return i;
            }
        }

        return -1;
    }

    int RRTDubis::getNearestListIndex(const NodeListType& node_list,
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

    Node RRTDubis::getRandomPoint()
    {
    }

    void RRTDubis::drawGraph(const Node& rnd)
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
        cv::waitKey(100);
    }

    void RRTDubis::drawPath(const std::vector<Node>& path)
    {
        int cols = (min_rand_ - max_rand_) / map_resolution_;
        int rows = (min_rand_ - max_rand_) / map_resolution_;
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