#include "ekf_slam.h"

/*
 * Constructor.
 */
EKFSLam::EKFSLam() {
  is_initialized_ = false;
}

/**
Initialize the parameters
**/

void EKFSLam::Initialize(unsigned int landmark_size, unsigned int rob_pose_size, float _motion_noise) {
  
  int N       = landmark_size;
  int r       = rob_pose_size;       
  mu          = VectorXd::Zero(2*N + r, 1);
  robSigma    = MatrixXd::Zero(r, r);
  robMapSigma = MatrixXd::Zero(r, 2*N);
  mapSigma    = INF*MatrixXd::Identity(2*N, 2*N);
  Sigma = MatrixXd::Zero(2*N + r, 2*N + r);

  Sigma.topLeftCorner(r, r)          = robSigma;
  Sigma.topRightCorner(r, 2*N)       = robMapSigma;
  Sigma.bottomLeftCorner(2*N, r)     = robMapSigma.transpose();
  Sigma.bottomRightCorner(2*N, 2*N)  = mapSigma;

  float motion_noise =_motion_noise;

  Q_  = MatrixXd::Zero(2*N + r, 2*N + r);
  Q_.topLeftCorner(3,3) << motion_noise, 0, 0,
        0, motion_noise , 0,
        0, 0,   motion_noise/10;

  observedLandmarks.resize(N);
  fill(observedLandmarks.begin(), observedLandmarks.end(), false);

}


/**
* Destructor.
*/
EKFSLam::~EKFSLam() {}


void EKFSLam::Prediction(const OdoReading& motion) {

  double angle = mu(2);
  double r1    = motion.r1;
  double t     = motion.t;
  double r2    = motion.r2;

  MatrixXd Gt = MatrixXd(3,3);
  Gt << 1, 0, -t*sin(angle + r1),
        0, 1,  t*cos(angle + r1),
        0, 0,  0;

  float c = cos(angle + r1);
  float s = sin(angle + r1);

  mu(0) = mu(0)  + t*c;
  mu(1) = mu(1) + t*s;
  mu(2) = mu(2) + r1 + r2;
  
  int size = Sigma.cols();
  Sigma.topLeftCorner(3,3) = Gt * Sigma.topLeftCorner(3,3) * Gt.transpose();
  Sigma.topRightCorner(3, size-3) = Gt * Sigma.topRightCorner(3, size-3);
  Sigma.bottomLeftCorner(size-3, 3) = Sigma.topRightCorner(3, size-3).transpose();
  Sigma = Sigma + Q_;

}


void EKFSLam::Correction(const vector<RadarReading>& observations) {

  // number of measurements in this step
  int m = observations.size();
  //[range, bearing, range, bearing, .....]
  VectorXd Z          = VectorXd::Zero(2*m);
  VectorXd expectedZ  = VectorXd::Zero(2*m);

  //Jacobian matrix;
  int N = observedLandmarks.size();
  MatrixXd H = MatrixXd::Zero(2*m, 2*N + 3);
  for (int i = 0; i < m; i++) {

      auto& reading = observations[i];
      long long landmarkId = reading.id;
      float     range      = reading.range;
      float     bearing    = reading.bearing;
      //landmark is not seen before, so to initialize the landmarks
      if (!observedLandmarks[landmarkId-1]) {
          mu(2*landmarkId + 1) = mu(0) + range*cos(mu(2) + bearing);
          mu(2*landmarkId+2) = mu(1) + range*sin(mu(2) + bearing);
          //Indicate in the observedLandmarks vector that this landmark has been observed
          observedLandmarks[landmarkId-1] = true;
      }

      //add the landmark meansurement to the Z vector
      Z(2*i) = range;
      Z(2*i+1) = bearing;
      //use the current estimate of the landmark poseq
      double deltax = mu(2*landmarkId+1) - mu(0);
      double deltay = mu(2*landmarkId+2) - mu(1);
      double q      = pow(deltax, 2) + pow(deltay, 2);
      expectedZ(2*i) = sqrt(q);
      expectedZ(2*i+1)   = atan2(deltay, deltax) - mu(2);

      H.block<2,3>(2*i,0) << -sqrt(q)*deltax/q, -sqrt(q)*deltay/q, 0,
                              deltay/q, -deltax/q, -1;
      H.block<2,2>(2*i, 2*landmarkId + 1) << sqrt(q)*deltax/q, sqrt(q)*deltay/q,
                                                -deltay/q, deltax/q; 
  }
  // cout << mu.transpose() << endl;
  //cout << H << endl;
  //construct the sensor noise 
  MatrixXd Q = MatrixXd::Identity(2*m, 2*m)*0.01;
  //compute the Kalman gain
  MatrixXd Ht = H.transpose();
  MatrixXd HQ = (H*Sigma*Ht) + Q;
  MatrixXd Si = HQ.inverse();
  MatrixXd K = Sigma*Ht*Si;
  //update 

  VectorXd diff = Z - expectedZ;
  tools.normalize_bearing(diff);
  mu = mu + K * diff;
  Sigma = Sigma - K*H*Sigma;
  mu(2) = tools.normalize_angle(mu(2));
}

void EKFSLam::ProcessMeasurement(const Record& record) {

      Prediction(record.odo); 
      Correction(record.radars);
}
