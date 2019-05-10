#include <iostream>
#include <vector>
#include <array>

#include "Eigen/Dense"

#include "matplotlibcpp.h"

using Eigen::MatrixXd;

namespace cpp_robotics {

    class ICPSLAM {
    public:

        /*
         * Iterative Closest Point matching
         *
         * - input
         * ppoints: 2D points in the previous frame
         * cpoints: 2D points in the current frame
         *
         * - output
         * R: Rotation matrix
         * T: Translation vector
         */
        void icpMatching(const MatrixXd& ppoints,
                         const MatrixXd& cpoints,
                         MatrixXd& R, MatrixXd& T) {
            MatrixXd H; // homogeneraous transformation matrix

            double dError = 1000.0;
            double preError = 1000.0;
            int count = 0;

            while (dError >= EPS) {
                count += 1;

                if (show_animation) {
                    matplotlibcpp::clf();
//                    plt.plot(ppoints[0, :], ppoints[1, :], ".r")
//                    plt.plot(cpoints[0, :], cpoints[1, :], ".b")
//                    plt.plot(0.0, 0.0, "xr")
                    matplotlibcpp::axis("equal");
                    matplotlibcpp::pause(1.0);
                }

                MatrixXd inds, error;
                nearestNeighborAssosiation(ppoints, cpoints,
                                           inds, error);

                if (dError <= EPS) {
//                    print("Converge", error, dError, count)
                    break;
                }
                else if (MAXITER <= count) {
//                    print("Not Converge...", error, dError, count)
                    break;
                }
            }
        }

        MatrixXd randMatrixXd(int rows, int cols) {
            MatrixXd num(rows, cols);
            for (int i=0; i< rows; ++i) {
                for (int j=0; j< cols; ++j) {
                    num(i, j) = randu();
                }
            }
            return num;
        }

    private:
        
        void nearestNeighborAssosiation(const MatrixXd& ppoints,
                                        const MatrixXd& cpoints,
                                        MatrixXd& inds, MatrixXd& error) {
            // calc the sum of residual errors
            MatrixXd dcpoints = ppoints - cpoints;
            auto d = dcpoints.norm();
            std::cout << "d:" << std::endl << d <<std::endl;
        }

        void updateHomogenerousMatrix(const MatrixXd& Hin,
                                      const MatrixXd& R,
                                      const MatrixXd& T) {

        }

        void svdMotionEstimation(const MatrixXd& ppoints,
                                 const MatrixXd& cpoints,
                                 MatrixXd& R, MatrixXd& t) {

        }

        inline double randu() {
            return (double) rand()/RAND_MAX;
        }

        // ICP parameters
        double EPS = 0.0001;
        int MAXITER = 100;

        bool show_animation = true;
    };

} // namespace cpp_robotics

int main() {
    std::cout << "Hello, World!" << std::endl;

    // simulation parameters
    int nPoint = 10;
    double fieldLength = 50.0;
    // movement [x[m],y[m],yaw[deg]]
    std::array<double, 3>  motion = {0.5, 2.0, -10.0*M_PI/180.0};
    int nsim = 3;  // number of simulation

    cpp_robotics::ICPSLAM icp_slam;

    while (nsim) {
        nsim--;

        // previous points
        MatrixXd l_num(1, nPoint);
        l_num.fill(0.5);
        MatrixXd px = (icp_slam.randMatrixXd(1, nPoint) - l_num) * fieldLength;
        MatrixXd py = (icp_slam.randMatrixXd(1, nPoint) - l_num) * fieldLength;
//        std::cout << "px:" << std::endl << px <<std::endl;
//        std::cout << "py:" << std::endl << py <<std::endl;

        MatrixXd ppoints(2, px.size());
        ppoints << px, py;
        std::cout << "ppoints:" << std::endl << ppoints <<std::endl;

        // current points
        MatrixXd cx(1, ppoints.cols());
        MatrixXd cy(1, ppoints.cols());
        for (int i=0; i<ppoints.cols(); i++) {
            cx(0, i) = motion[0] +
                       cos(motion[2]) * ppoints(0, i) -
                       sin(motion[2]) * ppoints(1, i);
            cy(0, i) = motion[1] + sin(motion[2]) * ppoints(0, i) +
                       cos(motion[2]) * ppoints(1, i);
        }
//        std::cout << "cx:" << std::endl << cx <<std::endl;
//        std::cout << "cy:" << std::endl << cy <<std::endl;

        MatrixXd cpoints(2, cx.size());
        cpoints << cx, cy;
        std::cout << "cpoints:" << std::endl << cpoints <<std::endl;

        // icp
        MatrixXd R, T;
        icp_slam.icpMatching(ppoints, cpoints, R, T);
    }

    return 0;
}