#include <iostream>
#include <vector>
#include <math.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

enum Mode
{
    LSL,
    RSR,
    LSR,
    RSL,
    RLR,
    LRL
};

struct DubinsMode
{
    DubinsMode(double t,
               double p,
               double q,
               Mode mode)
    {
        this->t = t;
        this->p = p;
        this->q = q;
        this->mode = mode;
    }

    double t = 0;
    double p = 0;
    double q = 0;
    Mode mode;
};

struct DubinsPose
{
    DubinsPose(){}

    DubinsPose(double x,
               double y,
               double yaw)
    {
        this->x = x;
        this->y = y;
        this->yaw = yaw;
    }

    DubinsPose(double x,
               double y,
               double yaw,
               Mode mode,
               double cost)
    {
        this->x = x;
        this->y = y;
        this->yaw = yaw;
        this->mode = mode;
        this->cost = cost;
    }

    double x = 0;
    double y = 0;
    double yaw = 0;
    Mode mode;
    double cost = 0;
};

class Dubins
{
public:

private:

    void dubinsPathPlanningFromOrigin(double ex, double ey, double eyaw, double c)
    {
        /// nomalize
        double dx = ex;
        double dy = ey;
        double D = sqrt(dx * dx + dy * dy);
        double d = D * c;

        double theta = mod2pi(atan2(dy, dx));
        double alpha = mod2pi(- theta);
        double beta = mod2pi(eyaw - theta);

        double bcost = std::numeric_limits<double>::infinity();
        double bt, bp, bq;
        Mode bmode;

        std::vector<Mode> planners;
        planners.push_back(Mode::LSL);
        planners.push_back(Mode::RSR);
        planners.push_back(Mode::LSR);
        planners.push_back(Mode::RSL);
        planners.push_back(Mode::RLR);
        planners.push_back(Mode::LRL);

        for(int i=0; i<planners.size(); i++)
        {

        }

    }

    std::vector<DubinsPose> generateCourse(double length, Mode mode, double c,
                        std::vector<double>& px,
                        std::vector<double>& py,
                        std::vector<double>& pyaw)
    {
        px.push_back(0.0);
        py.push_back(0.0);
        pyaw.push_back(0.0);

        std::string mode_string = modeToString(mode);
        for(int i=0; i<mode_string.size(); i++)
        {
            double pd = 0.0;
            double d = 0.0;
            if(mode_string[i] == 'S')
            {
                d = 1.0 * c;
            }
            else // turning couse
            {
                d = 3.0 * M_PI / 180.0; // TODO: why?
            }

            while(pd < fabs(length -  d))
            {
                px.push_back(px.back() + d / c * cos(pyaw.back()));
                py.push_back(py.back() + d / c * sin(pyaw.back()));

                if(mode_string[i] == 'L') // left turn
                {
                    pyaw.push_back(pyaw.back() + d);
                }
                else if(mode_string[i] == 'S') // Straight
                {
                    pyaw.push_back(pyaw.back());
                }
                else if(mode_string[i] == 'R') // right turn
                {
                    pyaw.push_back(pyaw.back() - d);
                }

                pd += d;
            }

            if(!(pd < fabs(length -  d)))
            {
                d = length - pd;
                px.push_back(px.back() + d / c * cos(pyaw.back()));
                py.push_back(py.back() + d / c * sin(pyaw.back()));

                if(mode_string[i] == 'L') // left turn
                {
                    pyaw.push_back(pyaw.back() + d);
                }
                else if(mode_string[i] == 'S') // Straight
                {
                    pyaw.push_back(pyaw.back());
                }
                else if(mode_string[i] == 'R') // right turn
                {
                    pyaw.push_back(pyaw.back() - d);
                }

                pd += d;
            }
        }
    }

    DubinsMode LSL(double alpha, double beta, double d)
    {
        double sa = sin(alpha);
        double sb = sin(beta);
        double ca = cos(alpha);
        double cb = cos(beta);
        double c_ab = cos(alpha - beta);

        double tmp0 = d + sa - sb;

        Mode mode = Mode::LSL;
        double p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sa - sb));
        if(p_squared < 0){
            return DubinsMode(0,0,0,mode);
        }

        double tmp1 = atan2((cb - ca), tmp0);
        double t = mod2pi(-alpha + tmp1);
        double p = sqrt(p_squared);
        double q = mod2pi(beta - tmp1);

        return DubinsMode(t,p,q,mode);
    }

    DubinsMode RSR(double alpha, double beta, double d)
    {
        double sa = sin(alpha);
        double sb = sin(beta);
        double ca = cos(alpha);
        double cb = cos(beta);
        double c_ab = cos(alpha - beta);

        double tmp0 = d - sa + sb;

        Mode mode = Mode::RSR;
        double p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sb - sa));
        if(p_squared < 0){
            return DubinsMode(0,0,0,mode);
        }

        double tmp1 = atan2((ca - cb), tmp0);
        double t = mod2pi(alpha - tmp1);
        double p = sqrt(p_squared);
        double q = mod2pi(-beta + tmp1);

        return DubinsMode(t,p,q,mode);
    }

    DubinsMode LSR(double alpha, double beta, double d)
    {
        double sa = sin(alpha);
        double sb = sin(beta);
        double ca = cos(alpha);
        double cb = cos(beta);
        double c_ab = cos(alpha - beta);

        double p_squared = -2 + (d * d) + (2 * c_ab) + (2 * d * (sa + sb));
        Mode mode = Mode::LSR;
        if(p_squared < 0){
            return DubinsMode(0,0,0,mode);
        }

        double p = sqrt(p_squared);
        double tmp2 = atan2((-ca - cb), (d + sa + sb)) - atan2(-2.0, p);
        double t = mod2pi(-alpha + tmp2);
        double q = mod2pi(-mod2pi(beta) + tmp2);

        return DubinsMode(t,p,q,mode);
    }

    DubinsMode RSL(double alpha, double beta, double d)
    {
        double sa = sin(alpha);
        double sb = sin(beta);
        double ca = cos(alpha);
        double cb = cos(beta);
        double c_ab = cos(alpha - beta);

        double p_squared = (d * d) - 2 + (2 * c_ab) - (2 * d * (sa + sb));
        Mode mode = Mode::RSL;
        if(p_squared < 0){
            return DubinsMode(0,0,0,mode);
        }

        double p = sqrt(p_squared);
        double tmp2 = atan2((ca + cb), (d - sa - sb)) - atan2(2.0, p);
        double t = mod2pi(alpha - tmp2);
        double q = mod2pi(beta - tmp2);

        return DubinsMode(t,p,q,mode);
    }

    DubinsMode RLR(double alpha, double beta, double d)
    {
        double sa = sin(alpha);
        double sb = sin(beta);
        double ca = cos(alpha);
        double cb = cos(beta);
        double c_ab = cos(alpha - beta);

        double tmp_rlr = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (sa - sb)) / 8.0;
        Mode mode = Mode::RLR;
        if(fabs(tmp_rlr) > 1.0){
            return DubinsMode(0,0,0,mode);
        }

        double p = mod2pi(2 * M_PI - acos(tmp_rlr));
        double t = mod2pi(alpha - atan2(ca - cb, d - sa + sb) + mod2pi(p / 2.0));
        double q = mod2pi(alpha - beta - t + mod2pi(p));

        return DubinsMode(t,p,q,mode);
    }

    DubinsMode LRL(double alpha, double beta, double d)
    {
        double sa = sin(alpha);
        double sb = sin(beta);
        double ca = cos(alpha);
        double cb = cos(beta);
        double c_ab = cos(alpha - beta);

        double tmp_lrl = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (- sa + sb)) / 8.0;
        Mode mode = Mode::LRL;
        if(fabs(tmp_lrl) > 1.0){
            return DubinsMode(0,0,0,mode);
        }

        double p = mod2pi(2 * M_PI - acos(tmp_lrl));
        double t = mod2pi(-alpha - atan2(ca - cb, d + sa - sb) + p / 2.0);
        double q = mod2pi(mod2pi(beta) - alpha - t + mod2pi(p));

        return DubinsMode(t,p,q,mode);
    }

    double mod2pi(double theta)
    {
        return theta - 2.0 * M_PI * floor(theta / 2.0 / M_PI);
    }

    double pi2pi(double angle)
    {
        return fmod((angle + M_PI), (2*M_PI)) - M_PI;
    }

    std::string modeToString(Mode mode)
    {
        std::string string_mode("");

        switch(mode)
        {
            case Mode::LSL :
                string_mode = "LSL";
                break;
            case Mode::RSR :
                string_mode = "RSR";
                break;
            case Mode::LSR :
                string_mode = "LSR";
                break;
            case Mode::RSL :
                string_mode = "RSL";
                break;
            case Mode::RLR :
                string_mode = "RLR";
                break;
            case Mode::LRL :
                string_mode = "LRL";
                break;
            default:
                break;
        }

        return string_mode;
    }
};

int main()
{
    std::cout << "Hello, World!" << std::endl;

    return 0;
}