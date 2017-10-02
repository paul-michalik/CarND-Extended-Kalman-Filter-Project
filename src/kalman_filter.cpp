#include "kalman_filter.h"
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  TODO:
    * predict the state
  */
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void update_from_residual(KalmanFilter& kf, const Eigen::VectorXd& y)
{
    /**
    TODO:
    * update the state by using Kalman Filter equations
    */
    MatrixXd Ht = kf.H_.transpose();
    MatrixXd S = kf.H_ * kf.P_ * Ht + kf.R_;
    MatrixXd K = kf.P_ * Ht * S.inverse();

    //new estimate
    kf.x_ = kf.x_ + (K * y);
    MatrixXd I = MatrixXd::Identity(kf.x_.size(), kf.x_.size());
    kf.P_ = (I - K * kf.H_) * kf.P_;
}

double normalize_angle(double phi)
{
    if (phi > M_PI) {
        phi = phi - 2 * M_PI;
    } else if (phi < -M_PI) {
        phi = phi + 2 * M_PI;
    }
    return phi;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
    update_from_residual(*this, z - H_ * x_ /* z_pred = H*x */);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
    auto rho = std::sqrt(x_(0)*x_(0) + x_(1)*x_(1));
    auto phi = normalize_angle(std::atan2(x_(1), x_(0)));
    auto rho_dot = 0.;

    if (fabs(rho) >= 0.0001) {
        rho_dot = (x_(0)*x_(2) + x_(1)*x_(3)) / rho;
    }

    VectorXd z_pred(3);
    z_pred << rho, phi, rho_dot;

    update_from_residual(*this, z - z_pred /* z_pred */);
}
