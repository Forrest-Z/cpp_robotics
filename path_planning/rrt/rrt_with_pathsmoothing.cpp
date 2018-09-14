//
// Created by forrest on 18-7-28.
// Path Planning Sample Code with Randamized Rapidly-Exploring Random Trees (RRT).
//

#include <iostream>
#include <vector>
#include <math.h>
#include <limits>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace cpp_robotics
{
    class Node
    {
    public:

        Node(double x = 0, double y = 0)
        {
            this->x = x;
            this->y = y;
            this->parent = -1;
        }

        double x = 0;
        double y = 0;
        int parent;
    };

    class ObstacleType
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

    using ObstacleListType = std::vector<ObstacleType>;
    using NodeListType = std::vector<Node>;
    using PathType = std::vector<Node>;

    class PathSmoothing
    {
    public:

        PathSmoothing(unsigned int maxIter = 1000)
        {
            this->maxIter = maxIter;
        }

        PathType pathSmoothing(const PathType& path,
                               const ObstacleListType& obstacleList)
        {
            PathType smoothPath(path);
            double le = getPathLength(smoothPath);

            for(int i=0; i<this->maxIter; i++)
            {
                /// Sample two points
                double pickPoints[2];
                pickPoints[0] = fmod(double(rand()),(le - 0+ 1)) + 0;
                pickPoints[1] = fmod(double(rand()),(le - 0+ 1)) + 0;

                std::vector<double> first = getTargetPoint(smoothPath, pickPoints[0]);
                std::vector<double> second = getTargetPoint(smoothPath, pickPoints[1]);

                if(first[2] <= 0 || second[2] <= 0){
                    continue;
                }

                if((second[2] + 1) > smoothPath.size()){
                    continue;
                }

                if((second[2] - first[2]) < 0.01){
                    continue;
                }

                /// collision check
                if(!lineCollisionCheck(first, second, obstacleList)){
                    continue;
                }

                /// Create New path
                PathType newPath;
                for(int i=0; i<(first[2] + 1); i++){
                    newPath.push_back(smoothPath[i]);
                }
                newPath.push_back(Node(first[0], first[1]));
                newPath.push_back(Node(second[0], second[1]));
                for(int i=(second[2] + 1); i<smoothPath.size(); i++){
                    newPath.push_back(smoothPath[i]);
                }
                newPath.swap(smoothPath);
                le = getPathLength(smoothPath);
            }

            return smoothPath;
        }

    private:

        bool lineCollisionCheck(std::vector<double> first,
                                std::vector<double> second,
                                const ObstacleListType& obstacleList)
        {
            /// Line Equation
            double x1 = first[0];
            double y1 = first[1];
            double x2 = second[0];
            double y2 = second[1];

            double a = y2 - y1;
            double b = -(x2 - x1);
            double c = y2 * (x2 - x1) - x2 * (y2 - y1);

            for(int i=0; i< obstacleList.size(); i++)
            {
                double d = fabs(a * obstacleList[i].x + b * obstacleList[i].y + c) /
                           (sqrt(a * a + b * b));
                if(d <= obstacleList[i].size)
                {
                    return false;
                }
            }

            return true;
        }

        std::vector<double> getTargetPoint(const PathType& path, double targetL)
        {
            double le = 0;
            double ti = 0;
            double lastPairLen = 0;
            for(int i=0; i<(path.size()-1); i++)
            {
                double dx = path[i+1].x - path[i].x;
                double dy = path[i+1].y - path[i].y;
                double d = sqrt(pow(dx,2) + pow(dy,2));
                le += d;
                if(le >= targetL)
                {
                    ti = i - 1;
                    lastPairLen = d;
                    break;
                }
            }

            double partRatio = (le - targetL) / lastPairLen;

            double x = path[ti].x + (path[ti + 1].x - path[ti].x) * partRatio;
            double y = path[ti].y + (path[ti + 1].y - path[ti].y) * partRatio;

            std::vector<double> point;
            point.push_back(x);
            point.push_back(y);
            point.push_back(ti);

            return point;
        }

        double getPathLength(const PathType& path)
        {
            double le = 0;
            for(int i=0; i<(path.size()-1); i++)
            {
                double dx = path[i+1].x - path[i].x;
                double dy = path[i+1].y - path[i].y;
                double d = sqrt(pow(dx,2) + pow(dy,2));
                le += d;
            }

            return le;
        }

        unsigned int maxIter = 1000;
    };

    class RRT
    {
    public:

        RRT(double start[2],
            double goal[2],
            double randArea[2],
            const ObstacleListType& obstacleList,
            double expandDis = 0.2,
            double goalSampleRate = 5.0,
            unsigned int maxIter = 10000)
        {
            /*
                Setting Parameter
                start:Start Position [x,y]
                goal:Goal Position [x,y]
                obstacleList:obstacle Positions [[x,y,size],...]
                randArea:Ramdom Samping Area [min,max]
            */
            this->start = Node(start[0], start[1]);
            this->end = Node(goal[0], goal[1]);
            this->minrand = randArea[0];
            this->maxrand = randArea[1];
            this->expandDis = expandDis;
            this->goalSampleRate = goalSampleRate;
            this->maxIter = maxIter;
            this->obstacleList = obstacleList;

            this->path_smoother = PathSmoothing(1000);
        }

        bool planning()
        {
            this->nodeList.push_back(this->start);

            srand(time(0));

            unsigned int SampleRate = 0;
            while(SampleRate <= this->maxIter)
            {
                SampleRate++;
                //std::cout<<"iteration:"<<SampleRate<<std::endl;
                if(SampleRate > this->maxIter)
                {
                    std::cout<<"Over iteration!!"<<std::endl;
                    return false;
                }

                /// Random Sampling
                Node rnd;
                if(fmod(double(SampleRate), this->goalSampleRate) <= 0)
                {
                    rnd = this->end;
                }
                else
                {
                    rnd.x = fmod(double(rand()),(this->maxrand - this->minrand + 1)) + this->minrand;
                    rnd.y = fmod(double(rand()),(this->maxrand - this->minrand + 1)) + this->minrand;
                }

                /// Find nearest node
                int nind = getNearestListIndex(this->nodeList, rnd);

                /// Expand tree
                Node nearestNode = this->nodeList[nind];
                double theta = atan2(rnd.y - nearestNode.y, rnd.x - nearestNode.x);

                Node newNode = nearestNode;
                newNode.x += this->expandDis * cos(theta);
                newNode.y += this->expandDis * sin(theta);
                newNode.parent = nind;

                if(!collisionCheck(newNode, this->obstacleList))
                {
                    continue;
                }

                this->nodeList.push_back(newNode);
                //std::cout<<"nNodelist:"<<this->nodeList.size()<<std::endl;

                /// Check goal
                double dx = newNode.x - this->end.x;
                double dy = newNode.y - this->end.y;
                double d = sqrt(dx * dx + dy * dy);
                if(d <= this->expandDis)
                {
                    std::cout<<"Goal!!"<<std::endl;

                    /// get path
                    PathType path;
                    path.push_back(this->end);
                    int lastIndex = this->nodeList.size() - 1;
                    while(this->nodeList[lastIndex].parent != -1)
                    {
                        path.push_back(this->nodeList[lastIndex]);
                        lastIndex = this->nodeList[lastIndex].parent;
                    }
                    path.push_back(this->start);

                    std::cout<<"path size:"<<path.size()<<std::endl;

                    drawPath(path);

                    PathType smoothPath = path_smoother.pathSmoothing(path, this->obstacleList);
                    std::cout<<"smoothing path size:"<<smoothPath.size()<<std::endl;

                    drawSmoothPath(path, smoothPath);

                    return true;
                }

                drawNode(rnd);
            }
        }

        int getNearestListIndex(NodeListType& nodeList, Node rnd)
        {
            double minind = std::numeric_limits<double>::infinity();
            int nind = -1;
            for(int i=0; i< nodeList.size(); i++)
            {
                double dx = nodeList[i].x - rnd.x;
                double dy = nodeList[i].y - rnd.y;
                double d = sqrt(pow(dx,2) + pow(dy,2));
                if(d < minind)
                {
                    minind = d;
                    nind = i;
                }
            }

            return nind;
        }

        bool collisionCheck(Node node, const ObstacleListType& obstacleList)
        {
            for(int i=0; i< obstacleList.size(); i++)
            {
                double dx = obstacleList[i].x - node.x;
                double dy = obstacleList[i].y - node.y;
                double d = sqrt(pow(dx,2) + pow(dy,2));
                if(d <= obstacleList[i].size)
                {
                    return false;
                }
            }

            return true;
        }

        void drawNode(Node rnd)
        {
            int cols = (this->maxrand - this->minrand) / map_resolution;
            int rows = (this->maxrand - this->minrand) / map_resolution;
            cv::Mat display_img(rows, cols, CV_8UC3, cv::Scalar(255,255,255));

            // draw obstacles
            for(int i=0; i< this->obstacleList.size(); i++)
            {
                cv::circle(display_img,
                           cv::Point(this->obstacleList[i].x / map_resolution,
                                     rows - this->obstacleList[i].y / map_resolution),
                           this->obstacleList[i].size  / map_resolution,
                           cv::Scalar(0,0,0),-1);
            }

            // draw tree
            for(int i=1; i< this->nodeList.size(); i++)
            {
                cv::line(display_img,
                         cv::Point(this->nodeList[i].x / map_resolution,
                                   rows - this->nodeList[i].y / map_resolution),
                         cv::Point(this->nodeList[this->nodeList[i].parent].x / map_resolution,
                                   rows - this->nodeList[this->nodeList[i].parent].y / map_resolution),
                         cv::Scalar(255,255,0));
            }

            // draw random node
            cv::circle(display_img,
                       cv::Point(rnd.x / map_resolution,
                                 rows - rnd.y / map_resolution),
                       3,cv::Scalar(255,0,0),-1);

            // draw start node
            cv::circle(display_img,
                       cv::Point(this->start.x / map_resolution,
                                 rows - this->start.y / map_resolution),
                       3,cv::Scalar(0,255,0),-1);

            // draw end node
            cv::circle(display_img,
                       cv::Point(this->end.x / map_resolution,
                                 rows - this->end.y / map_resolution),
                       this->expandDis / map_resolution,
                       cv::Scalar(0,0,255),1);

            cv::namedWindow("display_img",0);
            cv::imshow("display_img",display_img);
            cv::waitKey(100);
        }

        void drawPath(const PathType& path)
        {
            int cols = (this->maxrand - this->minrand) / map_resolution;
            int rows = (this->maxrand - this->minrand) / map_resolution;
            cv::Mat display_img(rows, cols, CV_8UC3, cv::Scalar(255,255,255));

            // draw obstacles
            for(int i=0; i< this->obstacleList.size(); i++)
            {
                cv::circle(display_img,
                           cv::Point(this->obstacleList[i].x / map_resolution,
                                     rows - this->obstacleList[i].y / map_resolution),
                           this->obstacleList[i].size  / map_resolution,
                           cv::Scalar(0,0,0),-1);
            }

            // draw tree
            for(int i=1; i< this->nodeList.size(); i++)
            {
                cv::line(display_img,
                         cv::Point(this->nodeList[i].x / map_resolution,
                                   rows - this->nodeList[i].y / map_resolution),
                         cv::Point(this->nodeList[this->nodeList[i].parent].x / map_resolution,
                                   rows - this->nodeList[this->nodeList[i].parent].y / map_resolution),
                         cv::Scalar(255,255,0));
            }

            // draw path
            for(int i=0; i< (path.size()-1); i++)
            {
                cv::line(display_img,
                         cv::Point(path[i].x / map_resolution,
                                   rows - path[i].y / map_resolution),
                         cv::Point(path[i+1].x / map_resolution,
                                   rows - path[i+1].y / map_resolution),
                         cv::Scalar(255,0,0));
            }

            // draw start node
            cv::circle(display_img,
                       cv::Point(this->start.x / map_resolution,
                                 rows - this->start.y / map_resolution),
                       3,cv::Scalar(0,255,0),-1);

            // draw end node
            cv::circle(display_img,
                       cv::Point(this->end.x / map_resolution,
                                 rows - this->end.y / map_resolution),
                       this->expandDis / map_resolution,
                       cv::Scalar(0,0,255),1);

            cv::namedWindow("display_img",0);
            cv::imshow("display_img",display_img);
            cv::waitKey(0);
        }

        void drawSmoothPath(const PathType& path,
                            const PathType& smoothPath)
        {
            int cols = (this->maxrand - this->minrand) / map_resolution;
            int rows = (this->maxrand - this->minrand) / map_resolution;
            cv::Mat display_img(rows, cols, CV_8UC3, cv::Scalar(255,255,255));

            // draw obstacles
            for(int i=0; i< this->obstacleList.size(); i++)
            {
                cv::circle(display_img,
                           cv::Point(this->obstacleList[i].x / map_resolution,
                                     rows - this->obstacleList[i].y / map_resolution),
                           this->obstacleList[i].size  / map_resolution,
                           cv::Scalar(0,0,0),-1);
            }

            // draw tree
            for(int i=1; i< this->nodeList.size(); i++)
            {
                cv::line(display_img,
                         cv::Point(this->nodeList[i].x / map_resolution,
                                   rows - this->nodeList[i].y / map_resolution),
                         cv::Point(this->nodeList[this->nodeList[i].parent].x / map_resolution,
                                   rows - this->nodeList[this->nodeList[i].parent].y / map_resolution),
                         cv::Scalar(255,255,0));
            }

            // draw path
            for(int i=0; i< (path.size()-1); i++)
            {
                cv::line(display_img,
                         cv::Point(path[i].x / map_resolution,
                                   rows - path[i].y / map_resolution),
                         cv::Point(path[i+1].x / map_resolution,
                                   rows - path[i+1].y / map_resolution),
                         cv::Scalar(255,0,0));
            }

            // draw smoothing path
            for(int i=0; i< (smoothPath.size()-1); i++)
            {
                cv::line(display_img,
                         cv::Point(smoothPath[i].x / map_resolution,
                                   rows - smoothPath[i].y / map_resolution),
                         cv::Point(smoothPath[i+1].x / map_resolution,
                                   rows - smoothPath[i+1].y / map_resolution),
                         cv::Scalar(255,0,255));
            }

            // draw start node
            cv::circle(display_img,
                       cv::Point(this->start.x / map_resolution,
                                 rows - this->start.y / map_resolution),
                       3,cv::Scalar(0,255,0),-1);

            // draw end node
            cv::circle(display_img,
                       cv::Point(this->end.x / map_resolution,
                                 rows - this->end.y / map_resolution),
                       this->expandDis / map_resolution,
                       cv::Scalar(0,0,255),1);

            cv::namedWindow("display_img",0);
            cv::imshow("display_img",display_img);
            cv::waitKey(0);
        }

    private:

        Node start;
        Node end;
        double minrand;
        double maxrand;
        double expandDis;
        double goalSampleRate;
        unsigned int maxIter;
        ObstacleListType obstacleList;
        NodeListType nodeList;

        PathSmoothing path_smoother;

        double map_resolution = 0.05;
    };
}

int main()
{
    std::cout << "start rrt_with_pathsmoothing path planning" << std::endl;

    cpp_robotics::ObstacleListType obstacleList;
    obstacleList.push_back(cpp_robotics::ObstacleType(5,5,1));
    obstacleList.push_back(cpp_robotics::ObstacleType(3,6,2));
    obstacleList.push_back(cpp_robotics::ObstacleType(3,8,2));
    obstacleList.push_back(cpp_robotics::ObstacleType(3,10,2));
    obstacleList.push_back(cpp_robotics::ObstacleType(7,5,2));
    obstacleList.push_back(cpp_robotics::ObstacleType(9,5,2));
    obstacleList.push_back(cpp_robotics::ObstacleType(9,12,2));

    double start[2] = {1,1};
    double goal[2] = {5,13};
    double randArea[2] = {0,15};
    cpp_robotics::RRT rrt(start,goal,randArea,obstacleList);

    rrt.planning();

    return 0;
}