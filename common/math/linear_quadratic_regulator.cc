#include "common/math/linear_quadratic_regulator.h"

#include <limits>

#include "Eigen/Dense"
#include <cmath>
#include <iostream>

using Matrix = Eigen::MatrixXd;

// solver with cross term
void SolveLQRProblem(const Matrix &A, const Matrix &B, const Matrix &Q,
                     const Matrix &R, const Matrix &M, const double tolerance,
                     const uint max_num_iteration, Matrix *ptr_K) {
  if (A.rows() != A.cols() || B.rows() != A.rows() || Q.rows() != Q.cols() ||
      Q.rows() != A.rows() || R.rows() != R.cols() || R.rows() != B.cols() ||
      M.rows() != Q.rows() || M.cols() != R.cols()) {
    std::cerr << "LQR solver: one or more matrices have incompatible dimensions." << std::endl;
    return;
  }

  Matrix AT = A.transpose();
  Matrix BT = B.transpose();
  Matrix MT = M.transpose();

  // Solves a discrete-time Algebraic Riccati equation (DARE)
  // Calculate Matrix Difference Riccati Equation, initialize P and Q
  Matrix P = Q;
  uint num_iteration = 0;
  double diff = std::numeric_limits<double>::max();
  while (num_iteration++ < max_num_iteration && diff > tolerance) {
    Matrix P_next =
        AT * P * A -
        (AT * P * B + M) * (R + BT * P * B).inverse() * (BT * P * A + MT) + Q;
    // check the difference between P and P_next
    diff = fabs((P_next - P).maxCoeff());
    P = P_next;
  }

  if (num_iteration >= max_num_iteration) {
    static uint warn_count = 0;
    if (warn_count < 5) {
      std::cerr << "LQR solver cannot converge to a solution, last consecutive result diff is: "
                << diff << std::endl;
      if (warn_count == 4) {
        std::cerr << "LQR solver warning muted after 5 occurrences." << std::endl;
      }
    }
    ++warn_count;
  }
  *ptr_K = (R + BT * P * B).inverse() * (BT * P * A + MT);
}

void SolveLQRProblem(const Matrix &A, const Matrix &B, const Matrix &Q,
                     const Matrix &R, const double tolerance,
                     const uint max_num_iteration, Matrix *ptr_K) {
  // create M as zero matrix of the right size:
  // M.rows() == Q.rows() && M.cols() == R.cols()
  Matrix M = Matrix::Zero(Q.rows(), R.cols());
  SolveLQRProblem(A, B, Q, R, M, tolerance, max_num_iteration, ptr_K);
}
