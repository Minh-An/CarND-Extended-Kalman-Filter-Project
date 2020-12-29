#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * Finish initializing the FusionEKF.
   */

  // set measurement function matrix for laser
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  // set the acceleration noise components
  // use noise_ax = 9 and noise_ay = 9 for your Q matrix.
  noise_ax = 9;
  noise_ay = 9;

  // set process covariance and transition matrix 
  ekf_.Q_ = MatrixXd(4,4);
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;


}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * Initialize the state ekf_.x_ with the first measurement.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Convert radar from polar to cartesian coordinates 
      // and initialize state 
      float rho =     measurement_pack.raw_measurements_[0];
      float phi =     measurement_pack.raw_measurements_[1];

      ekf_.x_ << rho * cos(phi), 
                 rho * sin(phi), 
                 0, 
                 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // Initialize state 
      float px = measurement_pack.raw_measurements_[0];
      float py = measurement_pack.raw_measurements_[1];

      ekf_.x_ << px, 
                 py, 
                  0, 
                  0;
    }

    // Create covariance matrix
    ekf_.P_ = MatrixXd(4,4);
    ekf_.P_ << 1, 0,  0,   0,
               0, 1,  0,   0,
               0, 0, 1000, 0,
               0, 0,  0, 1000;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }


  /**
   * Prediction
   */

  // dt in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  // Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;
  
  // Update the process noise covariance matrix.

  ekf_.Q_ <<  pow(dt,4)/4*noise_ax, 0, pow(dt,3)/2*noise_ax, 0,
              0, pow(dt,4)/4*noise_ay, 0, pow(dt,3)/2*noise_ay,
              pow(dt,3)/2*noise_ax, 0, pow(dt,2)*noise_ax, 0,
              0, pow(dt,3)/2*noise_ay, 0, pow(dt,2)*noise_ay;

  ekf_.Predict();


  /**
   * Update
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }


  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
