#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

#define eps 0.001

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {

  //initialize
  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;

  // sigma points
  Xsig_pred_ = MatrixXd(5,15);

  //timing
  time_us_ = 0;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.3;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.45;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  n_x_ = 5;
  n_aug_ = n_x_ + 2;
  lambda_ = 3 - n_aug_;
  n_sigma_ = 2*n_aug_ + 1;

  //set vector for weights
  weights_ = VectorXd(n_sigma_);
  double weight_0 = lambda_ /(lambda_ + n_aug_);
  weights_(0) = weight_0;
  for (int i=1; i<n_sigma_; i++) {  //2n+1 weights_
    double weight = 0.5/(n_aug_ + lambda_);
    weights_(i) = weight;
  }

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  if(!is_initialized_)
    initializeStates(meas_package);


  Prediction((meas_package.timestamp_ - time_us_) / 1000000.0);
  time_us_ = meas_package.timestamp_;

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)
    UpdateRadar(meas_package);
  else if(meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)
    UpdateLidar(meas_package);

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  //generate sigma points
  MatrixXd Xsig_aug = gensigmapts();

  //predict sigma points
  Xsig_pred_ = predictSigmaPoints(Xsig_aug, delta_t);

  // the state
  x_ = Xsig_pred_ * weights_;

  //state covariance matrix;
  P_.fill(0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Xsig_pred_.col(i) - x_;

    //angle normalization
    normalizeAngles(z_diff,3);

    P_ = P_ + weights_(i) * z_diff * z_diff.transpose();
  }

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  int n_z_ = 2;
  
  // 1. Predict measurement
  MatrixXd Zsig = MatrixXd(n_z_, n_sigma_);
  for (int i = 0; i <  n_sigma_; i++) {  

    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);

    // measurement model
    Zsig(0,i) = p_x;                        
    Zsig(1,i) = p_y;                                
  }
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_);
  z_pred.fill(0.0);
  for (int i=0; i < n_sigma_; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z_,n_z_);
  S.fill(0.0);
  for (int i = 0; i < n_sigma_; i++) {  
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    normalizeAngles(z_diff,1);

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z_,n_z_);
  R <<    std_laspx_*std_laspx_, 0,
          0, std_laspy_*std_laspy_;

  S = S + R;

  VectorXd z = meas_package.raw_measurements_;

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_);

  Tc.fill(0.0);
  for (int i = 0; i < n_sigma_; i++) {  

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    normalizeAngles(z_diff, 1);

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    normalizeAngles(x_diff, 3);

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  normalizeAngles(z_diff, 1);

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  //NIS Update
  NIS_lidar_ = z_diff.transpose() * S.inverse() * z_diff;
}
  


/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  int n_z_ = 3;
  
  // 1. Predict measurement
  MatrixXd Zsig = MatrixXd(n_z_, n_sigma_);
  for (int i = 0; i < n_sigma_; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                       
    Zsig(1,i) = atan2(p_y,p_x);                                 
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   
  }
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_);
  z_pred.fill(0.0);
  for (int i=0; i < n_sigma_; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z_,n_z_);
  S.fill(0.0);
  for (int i = 0; i < n_sigma_; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    normalizeAngles(z_diff,1);

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z_,n_z_);
  R <<    std_radr_*std_radr_, 0, 0,
          0, std_radphi_*std_radphi_, 0,
          0, 0,std_radrd_*std_radrd_;
  S = S + R;

    VectorXd z = meas_package.raw_measurements_;

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_);

  Tc.fill(0.0);
  for (int i = 0; i < n_sigma_; i++) {  

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    normalizeAngles(z_diff, 1);

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    normalizeAngles(x_diff, 3);

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  normalizeAngles(z_diff, 1);

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  //NIS Update
  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
}

/**
 * Initializes the state based on the first measurement (LIDAR or RADAR).
 * @param {MeasurementPackage} meas_package
 */
void UKF::initializeStates(MeasurementPackage meas_package)
{
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
  {
    float rho =  meas_package.raw_measurements_[0];
    float phi =  meas_package.raw_measurements_[1];
    float d_rho =  meas_package.raw_measurements_[2];
    x_<<rho*cos(phi),rho*sin(phi),d_rho,0,0;
  }
  else
    x_<<meas_package.raw_measurements_[0],meas_package.raw_measurements_[1],0,0,0;

  is_initialized_ = true;
  time_us_ = meas_package.timestamp_;
}

/**
 * Generates sigma points and return them as Matrix.
 */
MatrixXd UKF::gensigmapts() {
  
    VectorXd x_aug = VectorXd(7);
    x_aug.head(5) = x_;
    x_aug(5) = 0;
    x_aug(6) = 0;

    //create augmented covariance matrix
    MatrixXd P_aug = MatrixXd(7, 7);
    P_aug.fill(0.0);
    P_aug.topLeftCorner(5,5) = P_;
    P_aug(5,5) = std_a_*std_a_;
    P_aug(6,6) = std_yawdd_*std_yawdd_;

    //create square root matrix
    MatrixXd L = P_aug.llt().matrixL();
    MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sigma_);

    //create augmented sigma points
    Xsig_aug.col(0)  = x_aug;
    for (int i = 0; i< n_aug_; i++)
    {
      Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
      Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
    }
    return Xsig_aug;
}

/**
 * Normalizes a value .
 * @param {VectorXd} a Vector whose index to normalize
 * @param {VectorXd} the index to normalize
 */
void UKF::normalizeAngles(VectorXd &vec,int index)
{
    while (vec(index)> M_PI) vec(index)-=2.*M_PI;
    while (vec(index)<-M_PI) vec(index)+=2.*M_PI;
}

/**
 * predicts the sigma points
 * @param {MatrixXd} a matrix holding n_aug_*2 + 1 sigma points
 * @param {double} the time elapsed
 */
MatrixXd UKF::predictSigmaPoints(MatrixXd Xsig_aug, double delta_t)
{
  MatrixXd Temp_sig = MatrixXd(n_x_, n_sigma_);
  for(int i=0;i<n_sigma_;i++)
  {
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > eps) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Temp_sig(0,i) = px_p;
    Temp_sig(1,i) = py_p;
    Temp_sig(2,i) = v_p;
    Temp_sig(3,i) = yaw_p;
    Temp_sig(4,i) = yawd_p;
  }

  return Temp_sig;
}

