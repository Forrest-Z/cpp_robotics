#include <iostream>
#include <vector>

#include "matplotlibcpp.h"

using VD = std::vector<double>;

// 梯度下降的路径平滑算法
// 假设运动规划的结果点序列为x,平滑后的运动规划的点序列y
// 平滑Cost函数 Cost = c1 * (x[i] - y[i]) + c2 * (y[i-1] - y[i] + y[i+1] - y[i])
// c1用于衡量平滑后的点偏离原始点的程度
// c2第二项用于衡量平滑点之间的距离
// 这两个Cost项相互制衡，平滑的过程就是最小化Cost的过程
// 其中c1与c2是对目标路线平滑程度的参数，c1相对于c2越大，平滑后的点就越接近于原始点，反之，路线就越平滑。
void pathSmooth(const VD& path_x, const VD& path_y,
                VD& smooth_x, VD& smooth_y,
                double weight_data = 0.5, double weight_smooth = 0.3, double tolerance = 0.000001) {
    // Make a deep copy of path into newpath
    smooth_x = path_x;
    smooth_y = path_y;

    double change = tolerance;
    while (change >= tolerance) {
        change = 0;
        for (int i = 1; i < path_x.size()-1; i++) {
            double x_d1 = weight_data * (path_x[i] - smooth_x[i]);
            double x_d2 = weight_smooth * (smooth_x[i-1] + smooth_x[i+1] - 2 * smooth_x[i]);
            change += fabs(x_d1 + x_d2);
            smooth_x[i] += (x_d1 + x_d2);

            double y_d1 = weight_data * (path_y[i] - smooth_y[i]);
            double y_d2 = weight_smooth * (smooth_y[i-1] + smooth_y[i+1] - 2 * smooth_y[i]);
            change += fabs(y_d1 + y_d2);
            smooth_y[i] += (y_d1 + y_d2);
        }
    }
}

int main() {
    std::cout << "Hello, World!" << std::endl;

    VD path_x = {0, 0, 0, 1, 2, 3, 4, 4, 4};
    VD path_y = {0, 1, 2, 2, 2, 2, 2, 3, 4};
    VD smooth_x, smooth_y;
    pathSmooth(path_x, path_y, smooth_x, smooth_y);
    for (int i = 0; i < path_x.size(); i++) {
        std::cout << "[" << smooth_x[i] << ", " << smooth_y[i] << "]" << std::endl;
    }

    matplotlibcpp::clf();
    matplotlibcpp::plot(path_x, path_y, "-r");
    matplotlibcpp::plot(smooth_x, smooth_y, "-b");
    matplotlibcpp::axis("equal");
    matplotlibcpp::grid(true);
    matplotlibcpp::show();

    return 0;
}
