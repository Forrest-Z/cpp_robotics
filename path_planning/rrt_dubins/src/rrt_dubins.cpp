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
    bool RRTDubis::collisionCheck(Node node, ObstacleListType obstacle_list)
    {
        for(int i=0; i< obstacle_list.size(); i++)
        {
            double dx = obstacle_list[i].x - node.x;
            double dy = obstacle_list[i].y - node.y;
            double d = sqrt(pow(dx,2) + pow(dy,2));
            if(d <= pow(obstacle_list[i].size,2))
            {
                return false; // collision
            }
        }

        return true; // safe
    }

    Node RRTDubis::steer(Node rnd, int nind)
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

    int RRTDubis::getNearestListIndex(NodeListType node_list, Node rnd)
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
}