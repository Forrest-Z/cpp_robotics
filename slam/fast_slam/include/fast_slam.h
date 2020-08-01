
#pragma once

#include "measurement_package.h"
#include "common.h"
#include "Eigen/Dense"
#include "tools.h"
#include <random>

#define INF 1000

class FastSlam {
public:
  /**
  * Constructor, initialize with 100 partilces;
  */
  FastSlam();

  /**
  * Destructor.
  */
  virtual ~FastSlam();

  /**
  * Initialize the landmark size and number of particles
  */
  void Initialize(unsigned int landmark_size, int N = 50);

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
  * Run the measurement model for each particle
  **/
  void Measurement_model(const Particle& p, const RadarReading& z, Eigen::Vector2d& h, Eigen::MatrixXd& H);

 /**
 * Run the resampling according to the weights for each particle
 **/
  void Resample();

  /**
  * Get Best Particle and its pose and landmark estimation
  **/
  void getBestPoseAndLandmark(VectorXd& mu);

  //================================================
  static int MaxIndex(const std::vector<Particle>& ps);

public:
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // tool object used to compute Jacobian and RMSE
  Tools tools;

  //number of particles
  int N;

  //a list of particles
  vector<Particle> particles;

  //noises for pose
  Eigen::Vector3d noises;

  //mesurement noise
  Eigen::Matrix2d Q_;

  //weights for each particle
  vector<double> weights;
};
