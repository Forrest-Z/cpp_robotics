//
// Created by forrest on 19-8-25.
//

#ifndef DFS_DFS_HPP
#define DFS_DFS_HPP

#include <iostream>
#include <array>
#include <vector>
#include <stack>

#include <opencv2/opencv.hpp>

/**
 * 坐标系: ------> x
 *        |
 *        v
 *        y
 * 角度为:x轴0度,顺时针(-180~180)
 */
namespace cpp_robotics {
namespace path_planning {
namespace dfs {

using Coord = cv::Point;

enum State {
    FREE = 0,
    OBSTACLE
};

struct Cell {
    State state = State::FREE;
    bool visited = false;
    Coord parent;
};

class DFS {
public:
    DFS(bool display_flag = false) {
        display_flag_ = display_flag;

        directions_ = {Coord(-1, -1), Coord(0, -1), Coord(1, -1),
                       Coord(-1, 0),                Coord(1, 0),
                       Coord(-1, 1),  Coord(0, 1),  Coord(1, 1)};
    }

    void init(const cv::Mat& map) {
        display_img_ = map.clone();

        map_.resize(map.rows, std::vector<Cell>(map.cols));
        for (auto y = 0; y < map.rows; ++y) {
            for (auto x = 0; x < map.cols; ++x) {
                if (map.at<uchar>(y, x) > 250) {
                    map_[y][x].state = State::FREE;
                }
                else {
                    map_[y][x].state = State::OBSTACLE;
                }
            }
        }
    }

    std::vector<Coord> findPath(Coord start, Coord end) {
        //! reset dfs
        reset();

        //! check start and end point legal
        if (map_[start.y][start.x].state == State::OBSTACLE) {
            std::cerr << "start point in obstacle." << std::endl;
            return std::vector<Coord>();
        }
        if (map_[end.y][end.x].state == State::OBSTACLE) {
            std::cerr << "end point in obstacle." << std::endl;
            return std::vector<Coord>();
        }

        //! find path
        bool found_path = false;
        std::stack<Coord> open_list;
        open_list.push(start);
        map_[start.y][start.x].visited = true;
        while (!open_list.empty()) {
            auto current_corrd = open_list.top();
            open_list.pop();

            //! finish check
            if (current_corrd == end || found_path) {
                found_path = true;
                break;
            }

            //! search
            for (auto diretion : directions_) {
                auto next_coord = current_corrd + diretion;
                if (isOutOfMap(next_coord) ||
                    map_[next_coord.y][next_coord.x].visited ||
                    map_[next_coord.y][next_coord.x].state == State::OBSTACLE) {
                    continue;
                }
                open_list.push(next_coord);
                map_[next_coord.y][next_coord.x].visited = true;
                map_[next_coord.y][next_coord.x].parent = current_corrd;
                if (next_coord == end) {
                    found_path = true;
                    break;
                }
            }

            //! draw open_list
            if (display_flag_) {
                drawOpenList(open_list);
            }
        }

        //! get path
        std::vector<Coord> path;
        if (!found_path) {
            std::cerr << "find path fail." << std::endl;
        }
        else {
            auto coord = end;
            while (map_[coord.y][coord.x].parent != Coord(0, 0)) {
                path.emplace_back(coord);
                coord = map_[coord.y][coord.x].parent;
            }
            path.emplace_back(start);
            std::reverse(path.begin(),path.end());
            std::cout << "found path size:" << path.size() << std::endl;
        }

        //! draw path
        if (display_flag_) {
            drawPath(path);
        }

        return path;
    }

private:
    void reset() {
        for (auto y = 0; y < map_.size(); ++y) {
            for (auto x = 0; x < map_.front().size(); ++x) {
                map_[y][x].visited = false;
            }
        }
    }

    bool isOutOfMap(Coord coord) {
        if (coord.x < 0 || coord.x >= map_.front().size() ||
            coord.y < 0 || coord.y >= map_.size()) {
            return true;
        }
        return false;
    }

    void drawOpenList(const std::stack<Coord>& open_list) {
        auto op = open_list;
        while (!op.empty()) {
            auto coord = op.top();
            op.pop();
            display_img_.at<uchar>(coord.y, coord.x) = 192;
        }
        cv::namedWindow("dfs", 0);
        cv::imshow("dfs", display_img_);
        cv::waitKey(1);
    }

    void drawPath(const std::vector<Coord>& path) {
        for (auto coord : path) {
            display_img_.at<uchar>(coord.y, coord.x) = 64;
            cv::namedWindow("dfs", 0);
            cv::imshow("dfs", display_img_);
            cv::waitKey(10);
        }
        cv::namedWindow("dfs", 0);
        cv::imshow("dfs", display_img_);
        cv::waitKey(0);
    }

private:
    std::array<Coord, 8> directions_;
    std::vector<std::vector<Cell>> map_;

    //! for display
    cv::Mat display_img_;
    bool display_flag_ = false;
};

} // namespace dfs
} // namespace path_planning
} // namespace cpp_robotics

#endif //DFS_DFS_HPP
