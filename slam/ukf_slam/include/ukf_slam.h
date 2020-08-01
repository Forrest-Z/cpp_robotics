
#pragma once

#include "measurement_package.h"
#include "common.h"
#include "../lib/Eigen/Dense"
#include "tools.h"

//#define INF 1000

class UKFSLam
{
public:
  /**
  * Constructor.
  */
  UKFSLam();

  /**
  * Destructor.
  */
  virtual ~UKFSLam();

  /**
  * Initialize
  */
  void Initialize(float _motion_noise = 0.01);

  /**
  *  Prediction the state after motion
  */
  void Prediction(const OdoReading& motion);

  /**
  *  Correctthe state after observation
  */
  void Correction(const vector<RadarReading>& observation);

  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  void ProcessMeasurement(const Record& record);

  /**
  * Computes the 2n+1 sigma points according to the unscented transform,
  * where n is the dimensionality of the mean vector mu.
  * The sigma points should form the columns of sigma_points,
  * sigma_points is an nx2n+1 matrix
  **/
  void compute_sigma_points(MatrixXd& sigma_points, bool pred = false);

  /**
  * add new landmakr to the map if not observed before, update the database
  **/
  void add_landmark_to_map(const RadarReading& z);

  /**
  * Recover mu and sigma from unscented points after transformation
  **/
  void recover_mu_sigma(const MatrixXd& sig_pts);

  /**
  * Update the mu and sigma in the correction steps
  **/
  void update(const MatrixXd& sigma_points, const RadarReading& Z);

public:
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  float scale;

  // tool object used to compute Jacobian and RMSE
  Tools tools;
  Eigen::MatrixXd Sigma;
  Eigen::VectorXd mu;
  //motion noise
  Eigen::MatrixXd R_;
  //measurement noise
  Eigen::MatrixXd Q_;
  vector<int> landmarks;
  //Draw draw;
};
