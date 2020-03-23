#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include <cmath>
#include <cstdio>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;
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
  H_laser_<<1,0,0,0,
          0,1,0,0;
  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */


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
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.F_ = MatrixXd(4,4);
    ekf_.Q_ = MatrixXd(4,4);
    ekf_.P_ = MatrixXd(4,4);
    ekf_.x_ << 1, 1, 1, 1;
    ekf_.H_radar_ = VectorXd(3);

	previous_timestamp_=measurement_pack.timestamp_;
	
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
    ekf_.x_(0)=measurement_pack.raw_measurements_(0)*cos(measurement_pack.raw_measurements_(1));
    ekf_.x_(1)=measurement_pack.raw_measurements_(0)*sin(measurement_pack.raw_measurements_(1));
    ekf_.x_(2)=measurement_pack.raw_measurements_(2)*cos(measurement_pack.raw_measurements_(1));
    ekf_.x_(3)=measurement_pack.raw_measurements_(2)*sin(measurement_pack.raw_measurements_(1));
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      ekf_.x_(0)=measurement_pack.raw_measurements_(0);
      ekf_.x_(1)=measurement_pack.raw_measurements_(1);
      ekf_.x_(2)=0;
      ekf_.x_(3)=0;
	//do nothing as we need the radar measurement to update the velocities. 	
    }
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
    double dt, dt_2, dt_3, dt_4;
   	
   dt=(double)(measurement_pack.timestamp_-previous_timestamp_)/1000000;
   previous_timestamp_=measurement_pack.timestamp_;

	//computing all delta_t related values for Q matrix
	dt_2=dt*dt;
	dt_3=dt_2*dt;
	dt_4=dt_3*dt;
	ekf_.Q_<<9*dt_4/4, 0, dt_3/2*9, 0,
		0, 9*dt_4/4, 0, dt_3/2*9,
		dt_3/2*9, 0, dt_2*9, 0,
		0, dt_3/2*9, 0, dt_2*9;

	ekf_.F_<<1, 0, dt, 0,
		0, 1, 0, dt,
		0, 0, 1, 0,
		0, 0, 0, 1;
	
  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */


  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    Hj_=tools.CalculateJacobian(ekf_.x_);

  ekf_.H_radar_(0)=sqrt(ekf_.x_(0)*ekf_.x_(0)+ekf_.x_(1)*ekf_.x_(1)); 
  ekf_.H_radar_(1)=atan2(ekf_.x_(1),ekf_.x_(0));
  ekf_.H_radar_(2)=(ekf_.x_(0)*ekf_.x_(2)+ekf_.x_(1)*ekf_.x_(3))/ekf_.H_radar_(0);

  ekf_.H_=Hj_;
  ekf_.R_=R_radar_;
  
  ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // TODO: Laser updates
  ekf_.H_=H_laser_;
  ekf_.R_=R_laser_;
  ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout<<"x="<<ekf_.x_<<endl;
  cout<<"P="<<ekf_.P_<<endl;
}