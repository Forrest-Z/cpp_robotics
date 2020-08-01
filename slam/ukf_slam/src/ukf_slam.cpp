#include "../include/ukf_slam.h"
/*
 * Constructor.
 */
UKFSLam::UKFSLam()
{
  is_initialized_ = false;
  scale = 3.0;
}

/**
 * Destructor.
 */
UKFSLam::~UKFSLam() {}

/**
 * Initialize the parameters
 */
void UKFSLam::Initialize(float _motion_noise)
{
      int r       = 3;
      mu          = VectorXd::Zero(r);
//    mu = VectorXd(3);
//    mu << 10,10,0; // init robot pose

    Sigma = MatrixXd::Identity(3,3)*0.001;

    float motion_noise =_motion_noise;
    R_ = MatrixXd(3, 3);
    R_ << motion_noise, 0, 0,
          0, motion_noise , 0,
          0, 0,   motion_noise/10;

    Q_ = MatrixXd::Zero(2, 2);
    Q_ << 0.1, 0, 0, 0.01;
}

void UKFSLam::compute_sigma_points(MatrixXd& Xsig, bool pred)
{
    int n  = mu.size();
    if (pred) n = 3;
    int num_sig = 2 * n + 1;
    float lambda = scale - n;

    //get the square root of matrix sigma
    MatrixXd D = Sigma.topLeftCorner(n, n).llt().matrixL();

    Xsig = MatrixXd::Zero(n, 2 * n + 1);
    Xsig.col(0) = mu.head(n);

    for (int i = 0; i < n; i++)
    {
        Xsig.col(i + 1)     =   Xsig.col(0) + sqrt(n + lambda) *  D.col(i);
        Xsig.col(i + n + 1) =   Xsig.col(0) - sqrt(n + lambda) *  D.col(i);
    }
}

void UKFSLam::recover_mu_sigma(const MatrixXd& sig_pts)
{
  int  n = sig_pts.rows();
  float lambda = scale - n;

  //weight vector
  VectorXd weights = VectorXd::Zero(2*n + 1);
  weights(0) = lambda / scale;
  for (int i = 1; i < 2 * n + 1; i++) {
    weights(i) = 0.5 / scale;
  }

  VectorXd angle_c = sig_pts.row(2).array().cos();
  VectorXd angle_s = sig_pts.row(2).array().sin();

  double x_bar = angle_c.transpose() * weights;
  double y_bar = angle_s.transpose() * weights;
  
  //remove mu
  mu.head(n) = sig_pts * weights;
  double angle = atan2(y_bar, x_bar);
  mu(2) = tools.normalize_angle(angle);
 
 //remove sigma
  MatrixXd dsigma  = MatrixXd::Zero(n, n);
  for (int i = 0; i < 2 * n + 1; i++) {
    VectorXd diff = sig_pts.col(i) - mu.head(n);
    diff(2) = tools.normalize_angle(diff(2));
    dsigma = dsigma + weights(i) * diff * diff.transpose();
  }
  Sigma.topLeftCorner(n, n) = dsigma;
  //cout << "inside recover points" << endl;
}

void UKFSLam::add_landmark_to_map(const RadarReading& z)
{
    int n = mu.size();
    landmarks.push_back(z.id);

    //increae size of mu and sigma
    VectorXd tempmu = VectorXd::Zero(n + 2);
    tempmu.head(n) =  mu;
    tempmu(n)      = z.range;
    tempmu(n + 1)  = z.bearing;
    mu = tempmu;

    MatrixXd tempsigma = MatrixXd::Zero(n + 2, n + 2);
    tempsigma.topLeftCorner(n, n) = Sigma;
    tempsigma.bottomRightCorner(2, 2) = Q_;
    Sigma = tempsigma;
    //transform from [range, bearing] to the x/y location of the landmark
    //this operation initializes sthe uncertainty in the position of the landmark
    //sample sigma points
    MatrixXd sig_pts;
    compute_sigma_points(sig_pts);
    //normalize
    VectorXd Z = sig_pts.row(2);
    tools.normalize_bearing(Z);
    sig_pts.row(2) = Z;

    //compute newX and newY
    VectorXd angle_c = (sig_pts.row(2) + sig_pts.row(n + 1)).array().cos();
    VectorXd angle_s = (sig_pts.row(2) + sig_pts.row(n + 1)).array().sin();

    VectorXd delta_X = sig_pts.row(n).array() * angle_c.transpose().array();
    VectorXd delta_Y = sig_pts.row(n).array() * angle_s.transpose().array();

    sig_pts.row(n)   = sig_pts.row(0) + delta_X.transpose() ;
    sig_pts.row(n+1) = sig_pts.row(1) + delta_Y.transpose() ;
    //update mu and sigma
    recover_mu_sigma(sig_pts);
}

void UKFSLam::Prediction(const OdoReading& motion)
{
    MatrixXd Xsig;
    compute_sigma_points(Xsig, true);

    //dimensionality
    double r1 = motion.r1;
    double t  = motion.t;
    double r2 = motion.r2;

    for(int i = 0; i < Xsig.cols(); i++)
    {
        double angle = Xsig(2,i);
        double c = cos(angle + r1);
        double s = sin(angle + r1);
        Xsig(0, i) = Xsig(0, i) + t*c;
        Xsig(1, i) = Xsig(1, i) + t*s;
        Xsig(2, i) = Xsig(2, i) + r1 + r2;

        // TODO: change Odometry Model
        // particle motion model change to Gaussian probabilistic motion model
        // Xsig(0, i) = Xsig(0, i) + odom_x
        // Xsig(1, i) = Xsig(1, i) + odom_y;
        // Xsig(2, i) = Xsig(2, i) + odom_theta;
    }

    recover_mu_sigma(Xsig);
    Sigma.topLeftCorner(3,3) = Sigma.topLeftCorner(3,3) + R_;
}

void UKFSLam::update(const MatrixXd& sigma_points, const RadarReading& Z)
{
    int n = sigma_points.rows();
    int num_sig = sigma_points.cols();
    double lambda = scale - n;
    int index = 0;
    for (int i = 0; i < landmarks.size(); i++)
    {
      if (landmarks[i] == Z.id)
      {
        index = i;
        break;
      }
    }

    VectorXd DX = sigma_points.row(2*index + 3) - sigma_points.row(0);
    VectorXd DY = sigma_points.row(2*index + 4) - sigma_points.row(1);

    MatrixXd Z_points = MatrixXd::Zero(2, 2 * n + 1);
    for (int i = 0; i < num_sig; i++)
    {
      double q = pow(DX(i), 2) + pow(DY(i), 2);
      Z_points(0, i) = sqrt(q);
      Z_points(1, i) = tools.normalize_angle(atan2(DY(i), DX(i)) - sigma_points(2, i));
    }

    //weight vector
    VectorXd weights = VectorXd(num_sig);
    weights(0) = lambda / scale;
    for (int i = 1; i < num_sig; i++)
    {
      weights(i) = 0.5 / scale;
    }
    //zm is the recovered expected measurement mean from z_points.
    //It will be a 2x1 vector [expected_range; expected_bearing].
    VectorXd z_mean = VectorXd::Zero(2);
    z_mean(0) = Z_points.row(0)*weights;
    // cout << "Z: mean(0): " << z_mean(0) << endl;
    VectorXd angle_c = Z_points.row(1).array().cos();
    VectorXd angle_s = Z_points.row(1).array().sin();
    double x_bar = angle_c.transpose() * weights;
    double y_bar = angle_s.transpose() * weights;
    z_mean(1) = tools.normalize_angle(atan2(y_bar, x_bar));

    // TODO: Compute the innovation covariance matrix S (2x2).
    //Compute sigma_x_z (which is equivalent to sigma times the Jacobian H transposed in EKF).
    //sigma_x_z is an nx2 matrix, where n is the current dimensionality of mu
    MatrixXd S = MatrixXd::Zero(2,2);
    MatrixXd sigma_x_z = MatrixXd::Zero(n,2);
    for (int i = 0; i < num_sig; i++)
    {
        VectorXd tmpz = Z_points.col(i) - z_mean;
        tmpz(1) = tools.normalize_angle(tmpz(1));
        S = S + tmpz*tmpz.transpose()*weights(i);

        VectorXd tmpx = sigma_points.col(i) - mu;
        tmpx(2) = tools.normalize_angle(tmpx(2));
        sigma_x_z = sigma_x_z + weights(i)*tmpx*tmpz.transpose();
    }
    S = S + Q_;

   // Compute the Kalman gain
    MatrixXd Kt = sigma_x_z*(S.inverse());

    VectorXd z_actual = VectorXd(2);
    z_actual << Z.range, Z.bearing;
    VectorXd zdiff = z_actual-z_mean;
    zdiff(1) = tools.normalize_angle(zdiff(1));

    mu = mu + Kt*(zdiff);
    Sigma = Sigma - Kt*S*Kt.transpose();

    // TODO: Normalize the robot heading mu(3)
    mu(2) = tools.normalize_angle(mu(2));
}

void UKFSLam::Correction(const vector<RadarReading>& observations)
{
    // number of measurements in this step
    int m = observations.size();
    //[range, bearing, range, bearing, .....]
    //Jacobian matrix;
    for (int i = 0; i < m; i++)
    {
        auto&  reading  = observations[i];
        //landmark is not seen before, so to initialize the landmarks
        if (std::find(landmarks.begin(), landmarks.end(), reading.id) == landmarks.end())
        {
            add_landmark_to_map(reading);
        }
        else
        {
            //Compute sigma points from the predicted mean and covariance
            MatrixXd sigma_points;
            compute_sigma_points(sigma_points);
            update(sigma_points, reading);
        }
    }
}

void UKFSLam::ProcessMeasurement(const Record& record)
{
    Prediction(record.odo);
    Correction(record.radars);
//    vector<RadarReading> radars_zero;
//    Correction(radars_zero);

//    static int i=0;
//    i++;
//    if(i < 5)
//    {
//        vector<RadarReading> radars_zero;
//        Correction(radars_zero);
//    }
//    else if(i < 10){
//        Correction(record.radars);
//    }
//    else{
//        i = 0;
//    }
}

