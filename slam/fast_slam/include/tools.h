#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;


struct LandMark {
  int id;
  Eigen::Vector2d mu;
  Eigen::Matrix2d sigma;
  bool observed;
};


struct Particle {
  int id;
  Eigen::Vector3d pose;
  vector<Eigen::Vector3d> history;
  vector<LandMark> landmarks;
  double weight;
};


class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  /**
  * A helper method to calculate Jacobians.
  */
  MatrixXd CalculateJacobian(const VectorXd& x_state);

  /**
  * Noarmlize the angle
  */
  float normalize_angle(float phi);
  
  /**
  * Normalized the angles from observation
  */
  void normalize_bearing(VectorXd& Z);
};

#endif /* TOOLS_H_ */
