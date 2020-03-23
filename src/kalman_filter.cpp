#include "kalman_filter.h"
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
   x_=F_*x_;
   P_=F_*P_*F_.transpose()+Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */

  VectorXd y(2);
  MatrixXd S=MatrixXd(2,2);
  MatrixXd K=MatrixXd(4,2);
  MatrixXd I=MatrixXd(4,4);

  I<<1,0,0,0,
    0,1,0,0,
    0,0,1,0,
    0,0,0,1;
y=z-H_*x_;
S=H_*P_*H_.transpose()+R_;
K= P_*H_.transpose()*S.inverse();
x_=x_+K*y;
P_=(I-(K*H_))*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */

    VectorXd y(3);
  MatrixXd S=MatrixXd(3,3);
  MatrixXd K=MatrixXd(4,3);
  MatrixXd I=MatrixXd(4,4);

  I<<1,0,0,0,
    0,1,0,0,
    0,0,1,0,
    0,0,0,1;

y=z-H_radar_;

  while ( y(1) > M_PI || y(1) < -M_PI ) {
    if ( y(1) > M_PI ) {
      y(1) -= M_PI;
    } else {
      y(1) += M_PI;
    }
  }

S=H_*P_*H_.transpose()+R_;
K= P_*H_.transpose()*S.inverse();
x_=x_+K*y;
P_=(I-(K*H_))*P_;
}
