#include "../include/fast_slam.h"
/*
 * Constructor.
 */
FastSlam::FastSlam() {
  
  is_initialized_ = false;
}

/**
Initialize the parameters
**/

void FastSlam::Initialize(unsigned int landmark_size, int _N) {
  
  noises << 0.005, 0.01, 0.005;
  Q_ << 0.1, 0, 0, 0.1;

  N = _N;
  cout << "N : " << _N << endl;
  particles.resize(N);
  weights.resize(N);

  for (int i = 0; i < N; i++) {
    particles[i].weight = 1.0 / N;
    particles[i].pose << 0,0,0;
    particles[i].landmarks.resize(landmark_size);
    for (int j = 0; j < landmark_size; j++) {
      particles[i].landmarks[j].observed = false;
      particles[i].landmarks[j].mu << 0,0;
      particles[i].landmarks[j].sigma << 0, 0, 0, 0;
    }
  }
}


/**
* Destructor.
*/
FastSlam::~FastSlam() {}


void FastSlam::Measurement_model(const Particle& p, const RadarReading& z, Eigen::Vector2d& h, Eigen::MatrixXd& H) {

  int id = z.id;
  auto pose = p.landmarks[id-1].mu;
  //use the current state of particle to predict measuremen
  double delta_x = pose(0) - p.pose(0);
  double delta_y = pose(1) - p.pose(1);
  double expect_range   = sqrt(delta_x * delta_x + delta_y * delta_y);
  double expect_bearing = tools.normalize_angle(atan2(delta_y, delta_x) - p.pose(2));

  h << expect_range, expect_bearing;

  //compute the Jacobian H of the measurement function h wrt to the landmark location
  H = Eigen::MatrixXd::Zero(2, 2);
  H(0, 0) = delta_x  / expect_range;
  H(0, 1) = delta_y  / expect_range;
  H(1, 0) = -delta_y / (expect_range * expect_range);
  H(1, 1) = delta_x / (expect_range * expect_range);
}

void FastSlam::Prediction(const OdoReading& motion) {

  double r1 = motion.r1;
  double t  = motion.t;
  double r2 = motion.r2;

  std::default_random_engine generator;
  std::normal_distribution<double> r1_distribution(r1, noises(0));
  std::normal_distribution<double> t_distribution(t, noises(1));
  std::normal_distribution<double> r2_distribution(r2, noises(2));

  for (auto& p : particles) {
    p.history.push_back(p.pose);
    r1 = r1_distribution(generator);
    r2 = r2_distribution(generator);
    t  = t_distribution(generator); 
    p.pose(0) += t * cos(p.pose(2) + r1);
    p.pose(1) += t * sin(p.pose(2) + r1);
    p.pose(2) = tools.normalize_angle(p.pose(2) + r1 + r2);
  }

}


void FastSlam::Correction(const vector<RadarReading>& observations) {

  for (auto& p : particles) {
    const Eigen::Vector3d& robot = p.pose;
    for (const auto& z : observations) {
      int l = z.id;
      //if not seen before
      if (!p.landmarks[l-1].observed) {
        double gx = robot(0) + z.range * cos(robot(2) + z.bearing);
        double gy = robot(1) + z.range * sin(robot(2) + z.bearing);

        p.landmarks[l-1].mu << gx, gy;
        //get the Jacobian with respect to the landmark position
        Eigen::MatrixXd H;
        Eigen::Vector2d h;
        Measurement_model(p, z, h, H);

        //initialize the ekf for this landmark
	MatrixXd Hi = H.inverse();
        p.landmarks[l-1].sigma = Hi * Q_ * Hi.transpose();
        p.landmarks[l-1].observed = true;

      } else {	
	
        Eigen::Vector2d expect_Z;
        MatrixXd H;
        Measurement_model(p, z, expect_Z, H);
        //compute the measurement covariance
        MatrixXd sig = p.landmarks[l - 1].sigma;
        MatrixXd Q   = H * sig * H.transpose() + Q_;
        //calculate the Kalman gain K
        MatrixXd K = sig * H.transpose() * Q.inverse();
        //calculat the error between the z and expected Z
        Eigen::Vector2d z_actual;
        z_actual << z.range, z.bearing;
        Eigen::Vector2d z_diff = z_actual - expect_Z;
        z_diff(1) = tools.normalize_angle(z_diff(1));
        p.landmarks[l-1].mu    = p.landmarks[l-1].mu + K * z_diff;
        p.landmarks[l-1].sigma = p.landmarks[l-1].sigma - K * H * sig;
        //calculate the weight
        double w = exp(-0.5*z_diff.transpose()*Q.inverse()*z_diff)/sqrt(2 * M_PI * Q.determinant());
        p.weight *= w;
      }
    }
  }

}

void  FastSlam::Resample() {

  std::random_device rd;
  std::mt19937 gen(rd());
  
  double sum = 0.0;
  for (int i = 0; i < N; i++) {
    weights[i] = particles[i].weight;
    sum += weights[i];
  }  
  for (int i = 0; i < N; i++) {
    particles[i].weight = particles[i].weight/sum;
  }  

  std::discrete_distribution<> d(weights.begin(), weights.end());
  vector<Particle> resampParticle;
  resampParticle.resize(N);

  for (unsigned int i = 0; i < N; i++) {
    int idx = d(gen);
    resampParticle[i] = particles[idx];
  }
  particles = resampParticle;
}

void FastSlam::getBestPoseAndLandmark(VectorXd& mu_) {
  
  int index = MaxIndex(particles);
  Particle& p = particles[index];
  int N_ = p.landmarks.size();

  mu_ = Eigen::VectorXd::Zero(2 * N_ + 3);

  mu_(0) = p.pose(0);
  mu_(1) = p.pose(1);
  mu_(2) = p.pose(2);

  for (int i = 0; i < N_; i++) {
    mu_(2 * i + 3) = p.landmarks[i].mu(0);
    mu_(2 * i + 4) = p.landmarks[i].mu(1);
  }
}

void FastSlam::ProcessMeasurement(const Record& record) {

  Prediction(record.odo); 
  Correction(record.radars);
}


//static methods
int FastSlam::MaxIndex(const std::vector<Particle>& ps) {
  int max_Index = 0;
  double max_val = 0.0;
  for (int i = 1; i < ps.size(); i++) {
    if (max_val < ps[i].weight) {
      max_val = ps[i].weight;
      max_Index = i;
    }
  }
  return max_Index;
}
