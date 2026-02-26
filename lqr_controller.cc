#include "lqr_controller.h"

#include <cmath>
#include <iostream>
#include <limits>

#include "Eigen/LU"

namespace {

void SolveLQRProblem(const Eigen::MatrixXd& A,
                     const Eigen::MatrixXd& B,
                     const Eigen::MatrixXd& Q,
                     const Eigen::MatrixXd& R,
                     const double tolerance,
                     const unsigned int max_num_iteration,
                     Eigen::MatrixXd* ptr_K) {
  if (A.rows() != A.cols() || B.rows() != A.rows() || Q.rows() != Q.cols() ||
      Q.rows() != A.rows() || R.rows() != R.cols() || R.rows() != B.cols()) {
    std::cerr << "LQR solver: incompatible matrix dimensions." << std::endl;
    return;
  }

  const Eigen::MatrixXd AT = A.transpose();
  const Eigen::MatrixXd BT = B.transpose();

  Eigen::MatrixXd P = Q;
  unsigned int num_iteration = 0;
  double diff = std::numeric_limits<double>::max();

  // 通过离散代数 Riccati 方程迭代求解增益矩阵。
  while (num_iteration++ < max_num_iteration && diff > tolerance) {
    const Eigen::MatrixXd P_next =
        AT * P * A - (AT * P * B) * (R + BT * P * B).inverse() * (BT * P * A) +
        Q;
    diff = std::fabs((P_next - P).maxCoeff());
    P = P_next;
  }

  static unsigned int warn_count = 0;
  if (num_iteration >= max_num_iteration && warn_count < 5) {
    std::cerr << "LQR solver cannot converge to a solution, last consecutive "
                 "result diff is: "
              << diff << std::endl;
    ++warn_count;
    if (warn_count == 5) {
      std::cerr << "LQR solver warning muted after 5 occurrences." << std::endl;
    }
  }

  *ptr_K = (R + BT * P * B).inverse() * (BT * P * A);
}

}  // namespace

LqrController::LqrController() : LqrController(Params{}) {}

LqrController::LqrController(const Params& params) : p_(params) {
  const double mass_front = p_.mass_fl + p_.mass_fr;
  const double mass_rear = p_.mass_rl + p_.mass_rr;
  mass_ = mass_front + mass_rear;
  lf_ = p_.wheelbase * (1.0 - mass_front / mass_);
  lr_ = p_.wheelbase * (1.0 - mass_rear / mass_);
  iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;

  A_ = Eigen::MatrixXd::Zero(4, 4);
  A_coeff_ = Eigen::MatrixXd::Zero(4, 4);
  B_ = Eigen::MatrixXd::Zero(4, 1);
  Bd_ = Eigen::MatrixXd::Zero(4, 1);
  Q_ = Eigen::MatrixXd::Zero(4, 4);
  R_ = Eigen::MatrixXd::Identity(1, 1);

  A_(0, 1) = 1.0;
  A_(1, 2) = (p_.cf + p_.cr) / mass_;
  A_(2, 3) = 1.0;
  A_(3, 2) = (lf_ * p_.cf - lr_ * p_.cr) / iz_;

  A_coeff_(1, 1) = -(p_.cf + p_.cr) / mass_;
  A_coeff_(1, 3) = (lr_ * p_.cr - lf_ * p_.cf) / mass_;
  A_coeff_(3, 1) = (lr_ * p_.cr - lf_ * p_.cf) / iz_;
  A_coeff_(3, 3) = -(lf_ * lf_ * p_.cf + lr_ * lr_ * p_.cr) / iz_;

  B_(1, 0) = p_.cf / mass_;
  B_(3, 0) = lf_ * p_.cf / iz_;
  Bd_ = B_ * p_.ts;

  Q_(0, 0) = p_.q11;
  Q_(1, 1) = p_.q22;
  Q_(2, 2) = p_.q33;
  Q_(3, 3) = p_.q44;
  R_(0, 0) = p_.r;
}

double LqrController::ComputeSteering(const VehicleState& st,
                                      const FrenetMatch& match,
                                      LqrDebug* debug) const {
  const double v_for_lqr = std::max(p_.minimum_speed_protection, st.v);

  // 速度相关线性化：低速时用保护速度避免矩阵病态。
  Eigen::MatrixXd A = A_;
  A(1, 1) = A_coeff_(1, 1) / v_for_lqr;
  A(1, 3) = A_coeff_(1, 3) / v_for_lqr;
  A(3, 1) = A_coeff_(3, 1) / v_for_lqr;
  A(3, 3) = A_coeff_(3, 3) / v_for_lqr;

  const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(4, 4);
  const Eigen::MatrixXd Ad = (I - p_.ts * 0.5 * A).inverse() * (I + p_.ts * 0.5 * A);

  Eigen::MatrixXd K = Eigen::MatrixXd::Zero(1, 4);
  SolveLQRProblem(Ad, Bd_, Q_, R_, p_.lqr_eps, p_.lqr_max_iter, &K);

  const double e_y = match.d;
  const double e_psi = NormalizeAngle(st.theta - match.ref.theta);
  const double e_y_dot = st.v * std::sin(e_psi);
  const double e_psi_dot = st.omega - match.ref.kappa * match.ref.v;

  Eigen::MatrixXd x_state = Eigen::MatrixXd::Zero(4, 1);
  x_state(0, 0) = e_y;
  x_state(1, 0) = e_y_dot;
  x_state(2, 0) = e_psi;
  x_state(3, 0) = e_psi_dot;

  const double delta_fb = -(K * x_state)(0, 0);

  // 曲率前馈项，用于减小稳态横向误差。
  const double kv =
      lr_ * mass_ / (2.0 * p_.cf * p_.wheelbase) -
      lf_ * mass_ / (2.0 * p_.cr * p_.wheelbase);
  const double delta_ff =
      p_.wheelbase * match.ref.kappa + kv * st.v * st.v * match.ref.kappa -
      K(0, 2) * (lr_ * match.ref.kappa -
                 lf_ * mass_ * st.v * st.v * match.ref.kappa /
                     (2.0 * p_.cr * p_.wheelbase));

  if (debug != nullptr) {
    debug->e_y = e_y;
    debug->e_psi = e_psi;
    debug->delta_fb = delta_fb;
    debug->delta_ff = delta_ff;
  }

  return Clamp(delta_fb + delta_ff, -p_.steer_limit, p_.steer_limit);
}
