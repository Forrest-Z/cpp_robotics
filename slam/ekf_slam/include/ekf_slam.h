
#pragma once

#include "measurement_package.h"
#include "common.h"
#include "Eigen/Dense"
#include "tools.h"
#define INF 1000

class EKFSLam {
public:
  /**
  * Constructor.
  */
  EKFSLam();

  /**
  * Destructor.
  */
  virtual ~EKFSLam();

  /**
  * Initialize
  */
  void Initialize(unsigned int landmark_size, unsigned int rob_pose_size = 3, float _motion_noise = 0.1);

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


  //-------------------------------------
  VectorXd getMu() const {
      return mu;
  }

  MatrixXd getSigma() const {
      return Sigma;
  }


public:
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // tool object used to compute Jacobian and RMSE
  Tools tools;
  Eigen::MatrixXd robSigma;
  Eigen::MatrixXd robMapSigma;
  Eigen::MatrixXd mapSigma;
  Eigen::MatrixXd Sigma;
  Eigen::VectorXd mu;
  Eigen::MatrixXd Q_;
  vector<bool> observedLandmarks;
  //Draw draw;
};
